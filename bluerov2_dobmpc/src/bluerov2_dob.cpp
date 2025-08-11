#include <bluerov2_dobmpc/bluerov2_dob.h>

// Fallback definition for M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Initialize MPC
BLUEROV2_DOB::BLUEROV2_DOB(ros::NodeHandle& nh)
{
    ROS_INFO("Starting BLUEROV2_DOB constructor...");
    
    // read parameter
    nh.getParam("/bluerov2_dob_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_dob_node/read_wrench",READ_WRENCH);
    nh.getParam("/bluerov2_dob_node/compensate_d",COMPENSATE_D);
    nh.getParam("/bluerov2_dob_node/ref_traj", REF_TRAJ);
    nh.getParam("/bluerov2_dob_node/applied_forcex", WRENCH_FX);
    nh.getParam("/bluerov2_dob_node/applied_forcey", WRENCH_FY);
    nh.getParam("/bluerov2_dob_node/applied_forcez", WRENCH_FZ);
    nh.getParam("/bluerov2_dob_node/applied_torquez", WRENCH_TZ);
    nh.getParam("/bluerov2_dob_node/disturbance_x", solver_param.disturbance_x);
    nh.getParam("/bluerov2_dob_node/disturbance_y", solver_param.disturbance_y);
    nh.getParam("/bluerov2_dob_node/disturbance_z", solver_param.disturbance_z);
    nh.getParam("/bluerov2_dob_node/disturbance_phi", solver_param.disturbance_phi);
    nh.getParam("/bluerov2_dob_node/disturbance_theta", solver_param.disturbance_theta);
    nh.getParam("/bluerov2_dob_node/disturbance_psi", solver_param.disturbance_psi);
    
    ROS_INFO("Parameters loaded successfully");
    
    // Pre-load the trajectory
    ROS_INFO("Loading trajectory from: %s", REF_TRAJ.c_str());
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
	}

    // Initialize MPC
    ROS_INFO("Initializing MPC...");
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }
    ROS_INFO("MPC initialized successfully");

    // Initialize EKF
    ROS_INFO("Initializing EKF matrices...");
    M_values << mass + added_mass[0], mass + added_mass[1], mass + added_mass[2], Ix + added_mass[3], Iy + added_mass[4], Iz + added_mass[5];
    M = M_values.asDiagonal();
    M(0,4) = mass*ZG;
    M(1,3) = -mass*ZG;
    M(3,1) = -mass*ZG;
    M(4,0) = mass*ZG;
    invM = M.inverse();
    ROS_INFO("EKF matrices initialized");

    // Dl_values << -11.7391, -20, -31.8678, -25, -44.9085, -5;
    // Dl = Dl_values.asDiagonal();

    K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
       0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
       0, 0, 0, 0, 1, 1,
       0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
       -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
       0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
       
    Q_cov << pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2);
    noise_Q= Q_cov.asDiagonal();
    
    esti_x << 0,0,-20,0,0,0,0,0,0,0,0,0,6,6,6,0,0,0;
    esti_P = P0;
    
    // Initialize UKF
    ROS_INFO("Initializing UKF...");
    initialize_ukf();
    ROS_INFO("UKF initialized successfully");

    // Initialize body wrench force
    applied_wrench.fx = 0.0;
    applied_wrench.fy = 0.0;
    applied_wrench.fz = 0.0;
    applied_wrench.tx = 0.0;
    applied_wrench.ty = 0.0;
    applied_wrench.tz = 0.0;
    
    // Initialize current thrust
    current_t.t0 = 0.0;
    current_t.t1 = 0.0;
    current_t.t2 = 0.0;
    current_t.t3 = 0.0;
    current_t.t4 = 0.0;
    current_t.t5 = 0.0;

    // ros subsriber & publisher
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2/pose_gt", 20, &BLUEROV2_DOB::pose_cb, this);
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
    ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
    error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
    control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);
    control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
    control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
    control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);    
    esti_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/pose",20);
    esti_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/disturbance",20);
    applied_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/applied_disturbance",20);
    trajectory_pub = nh.advertise<nav_msgs::Path>("/bluerov2/trajectory", 10);
    subscribers.resize(6);
    for (int i = 0; i < 6; i++)
    {
        std::string topic = "/bluerov2/thrusters/" + std::to_string(i) + "/thrust";
        subscribers[i] = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(topic, 20, boost::bind(&BLUEROV2_DOB::thrusts_cb, this, _1, i));
    }
    client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &BLUEROV2_DOB::imu_cb, this);
    pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("/bluerov2/pressure", 20, &BLUEROV2_DOB::pressure_cb, this);
    
    // Initialize LiDAR subscribers - Use only forward LiDAR for simplicity
    ROS_INFO("Initializing LiDAR subscriber (forward only)...");
    forward_lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/bluerov2/forward_lidar_/scan", 20, &BLUEROV2_DOB::forward_lidar_cb, this);
    ROS_INFO("Forward LiDAR subscriber initialized");
    
    // Initialize LiDAR distance variables
    forward_lidar_distance = 10.0;  // Default distance
    left_lidar_distance = 10.0;     // Keep for compatibility
    right_lidar_distance = 10.0;    // Keep for compatibility
    
    // Initialize keyboard subscriber
    ROS_INFO("Initializing keyboard subscriber...");
    keyboard_sub = nh.subscribe<std_msgs::String>("/keyboard_input", 10, &BLUEROV2_DOB::keyboard_cb, this);
    pcl_sub = nh.subscribe("/camera/depth/color/points", 20, &BLUEROV2_DOB::pcl_cb, this);
    
    // Initialize fault detection timers
    ROS_INFO("Initializing fault detection timers...");
    fault_detection.last_imu_time = ros::Time::now();
    fault_detection.last_pressure_time = ros::Time::now();
    fault_detection.last_lidar_time = ros::Time::now();
    
    // Initialize trajectory visualization
    ROS_INFO("Initializing trajectory visualization...");
    trajectory_path.header.frame_id = "odom_frame";
    trajectory_path.header.stamp = ros::Time::now();

    // initialize
    ROS_INFO("Initializing control arrays...");
    for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;
    is_start = false;
    
    ROS_INFO("BLUEROV2_DOB constructor completed successfully");
}

void BLUEROV2_DOB::pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    is_start = true;
    // get linear position x, y, z
    local_pos.x = pose->pose.pose.position.x;
    local_pos.y = pose->pose.pose.position.y;
    local_pos.z = pose->pose.pose.position.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    // get linear velocity u, v, w
    local_pos.u = pose->twist.twist.linear.x;
    local_pos.v = pose->twist.twist.linear.y;
    local_pos.w = pose->twist.twist.linear.z;

    // get angular velocity p, q, r
    local_pos.p = pose->twist.twist.angular.x;
    local_pos.q = pose->twist.twist.angular.y;
    local_pos.r = pose->twist.twist.angular.z;

    // inertial frame velocity to body frame
    Matrix<double,3,1> v_linear_inertial;
    Matrix<double,3,1> v_angular_inertial;

    v_linear_inertial << local_pos.u, local_pos.v, local_pos.w;
    v_angular_inertial << local_pos.p, local_pos.q, local_pos.r;

    R_ib << cos(local_euler.psi)*cos(local_euler.theta), -sin(local_euler.psi)*cos(local_euler.phi)+cos(local_euler.psi)*sin(local_euler.theta)*sin(local_euler.phi), sin(local_euler.psi)*sin(local_euler.phi)+cos(local_euler.psi)*cos(local_euler.phi)*sin(local_euler.theta),
            sin(local_euler.psi)*cos(local_euler.theta), cos(local_euler.psi)*cos(local_euler.phi)+sin(local_euler.phi)*sin(local_euler.theta)*sin(local_euler.psi), -cos(local_euler.psi)*sin(local_euler.phi)+sin(local_euler.theta)*sin(local_euler.psi)*cos(local_euler.phi),
            -sin(local_euler.theta), cos(local_euler.theta)*sin(local_euler.phi), cos(local_euler.theta)*cos(local_euler.phi);
    T_ib << 1, sin(local_euler.psi)*sin(local_euler.theta)/cos(local_euler.theta), cos(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta),
            0, cos(local_euler.phi), sin(local_euler.phi),
            0, sin(local_euler.phi)/cos(local_euler.theta), cos(local_euler.phi)/cos(local_euler.theta);
    v_linear_body = R_ib.inverse()*v_linear_inertial;
    v_angular_body = T_ib.inverse()*v_angular_inertial;

    body_acc.x = (v_linear_body[0]-pre_body_pos.u)/dt;
    body_acc.y = (v_linear_body[1]-pre_body_pos.v)/dt;
    body_acc.z = (v_linear_body[2]-pre_body_pos.w)/dt;
    body_acc.phi = (v_angular_body[0]-pre_body_pos.p)/dt;
    body_acc.theta = (v_angular_body[1]-pre_body_pos.q)/dt;
    body_acc.psi = (v_angular_body[2]-pre_body_pos.r)/dt;

    pre_body_pos.u = v_linear_body[0];
    pre_body_pos.v = v_linear_body[1];
    pre_body_pos.w = v_linear_body[2];
    pre_body_pos.p = v_angular_body[0];
    pre_body_pos.q = v_angular_body[1];
    pre_body_pos.r = v_angular_body[2];

    Matrix<double,3,1> compensate_f_inertial;
    Matrix<double,3,1> compensate_f_body;
    compensate_f_inertial << 20,0,0;
    compensate_f_body = R_ib.inverse()*compensate_f_inertial;

    // Update trajectory visualization
    update_trajectory();
    publish_trajectory();

    }

// quaternion to euler angle
BLUEROV2_DOB::Euler BLUEROV2_DOB::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// euler angle to quaternion
geometry_msgs::Quaternion BLUEROV2_DOB::rpy2q(const Euler& euler){
    tf2::Quaternion quat;
    quat.setRPY(euler.phi, euler.theta, euler.psi);
    geometry_msgs::Quaternion quaternion;
    tf2::convert(quat, quaternion);
    return quaternion;
}

// read trajectory data
int BLUEROV2_DOB::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
        std::cout<<"file is open"<<std::endl;
		while(getline(file, line)){
			number_of_lines++;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
	}
	else
	{
        std::cout<<"file not open"<<std::endl;
		return 0;
	}

	return number_of_lines;
}
void BLUEROV2_DOB::ref_cb(int line_to_read)
{
    ROS_INFO("ref_cb called with line_to_read: %d", line_to_read);
    ROS_INFO("number_of_steps: %d", number_of_steps);
    ROS_INFO("BLUEROV2_N: %d", BLUEROV2_N);
    
    // Generate reference trajectory based on current robot position
    generate_dynamic_reference_trajectory();
}

// New function to generate reference trajectory based on current robot position
void BLUEROV2_DOB::generate_dynamic_reference_trajectory()
{
    ROS_INFO("Generating dynamic reference trajectory based on current robot position");
    
    // Get current robot position
    double current_x = local_pos.x;
    double current_y = local_pos.y;
    double current_z = local_pos.z;
    double current_psi = local_euler.psi;
    
    ROS_INFO("Current robot position: x=%.3f, y=%.3f, z=%.3f, psi=%.3f", 
             current_x, current_y, current_z, current_psi);
    
    // Define target position - use more reasonable targets to avoid large errors
    // For initial testing, use targets close to current position
    double target_x = current_x + 5.0;  // Move 5m forward from current position
    double target_y = current_y;        // Stay at current y position (straight line)
    double target_z = current_z;        // Maintain current depth
    double target_psi = current_psi;    // Maintain current heading
    
    // Try to get target from ROS parameters, otherwise use reasonable defaults
    if (!ros::param::get("/bluerov2_dob_node/target_x", target_x)) {
        // If no parameter set, use a reasonable offset from current position
        target_x = current_x + 5.0;  // 5m forward
        ROS_INFO("Using reasonable target_x: %.3f (%.3f + 5.0)", target_x, current_x);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_y", target_y)) {
        target_y = current_y;  // Stay at current y
        ROS_INFO("Using reasonable target_y: %.3f (current position)", target_y);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_z", target_z)) {
        target_z = current_z;  // Maintain current depth
        ROS_INFO("Using reasonable target_z: %.3f (current depth)", target_z);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_psi", target_psi)) {
        target_psi = current_psi;  // Maintain current heading
        ROS_INFO("Using reasonable target_psi: %.3f (current heading)", target_psi);
    }
    
    // Calculate distance to target
    double distance_to_target = sqrt(pow(target_x - current_x, 2) + 
                                   pow(target_y - current_y, 2) + 
                                   pow(target_z - current_z, 2));
    
    ROS_INFO("Distance to target: %.3f meters", distance_to_target);
    
    // Check if distance is reasonable for MPC solver
    if (distance_to_target > 20.0) {
        ROS_WARN("Target distance %.3f is too large, limiting to 20m for MPC stability", distance_to_target);
        // Limit the target to a reasonable distance
        double scale_factor = 20.0 / distance_to_target;
        target_x = current_x + (target_x - current_x) * scale_factor;
        target_y = current_y + (target_y - current_y) * scale_factor;
        target_z = current_z + (target_z - current_z) * scale_factor;
        distance_to_target = 20.0;
        ROS_INFO("Adjusted target: (%.3f, %.3f, %.3f), new distance: %.3f", 
                 target_x, target_y, target_z, distance_to_target);
    }
    
    // Generate reference trajectory for the entire horizon
    for (unsigned int i = 0; i <= BLUEROV2_N; i++) {
        // Calculate progress along the trajectory (0 to 1)
        double progress = (double)i / BLUEROV2_N;
        
        // Use smooth interpolation (sigmoid-like function for smooth acceleration/deceleration)
        double smooth_progress = 1.0 / (1.0 + exp(-10.0 * (progress - 0.5)));
        
        // Interpolate position
        acados_in.yref[i][0] = current_x + (target_x - current_x) * smooth_progress;  // x
        acados_in.yref[i][1] = current_y + (target_y - current_y) * smooth_progress;  // y
        acados_in.yref[i][2] = current_z + (target_z - current_z) * smooth_progress;  // z
        
        // Interpolate orientation (handle angle wrapping)
        double psi_diff = target_psi - current_psi;
        // Normalize angle difference to [-pi, pi]
        while (psi_diff > M_PI) psi_diff -= 2 * M_PI;
        while (psi_diff < -M_PI) psi_diff += 2 * M_PI;
        
        acados_in.yref[i][3] = 0.0;  // phi (roll) - keep level
        acados_in.yref[i][4] = 0.0;  // theta (pitch) - keep level
        acados_in.yref[i][5] = current_psi + psi_diff * smooth_progress;  // psi (yaw)
        
        // Set velocities to zero for now (could be calculated based on trajectory)
        acados_in.yref[i][6] = 0.0;   // u
        acados_in.yref[i][7] = 0.0;   // v
        acados_in.yref[i][8] = 0.0;   // w
        acados_in.yref[i][9] = 0.0;   // p
        acados_in.yref[i][10] = 0.0;  // q
        acados_in.yref[i][11] = 0.0;  // r
        
        // Set disturbance estimates to zero (could be updated from UKF)
        acados_in.yref[i][12] = 0.0;  // disturbance x
        acados_in.yref[i][13] = 0.0;  // disturbance y
        acados_in.yref[i][14] = 0.0;  // disturbance z
        acados_in.yref[i][15] = 0.0;  // disturbance phi
        acados_in.yref[i][16] = 0.0;  // disturbance theta
        acados_in.yref[i][17] = 0.0;  // disturbance psi
    }
    
    ROS_INFO("Dynamic reference trajectory generated successfully");
    ROS_INFO("Reference trajectory: start=(%.3f,%.3f,%.3f) -> end=(%.3f,%.3f,%.3f)", 
             acados_in.yref[0][0], acados_in.yref[0][1], acados_in.yref[0][2],
             acados_in.yref[BLUEROV2_N][0], acados_in.yref[BLUEROV2_N][1], acados_in.yref[BLUEROV2_N][2]);
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_DOB::solve(){
    ROS_INFO("Starting solve function...");
    
    // Check if we have valid pose data
    if (!is_start) {
        ROS_WARN("Solve called before pose data received, skipping...");
        return;
    }
    // identify turning direction
    yaw_diff = 0.0;  // Initialize yaw_diff
    
    if (pre_yaw >= 0 && local_euler.psi >=0)
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }
    else if (pre_yaw >= 0 && local_euler.psi <0)
    {
        if (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+abs(local_euler.psi))
        {
            yaw_diff = -(pre_yaw + abs(local_euler.psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + local_euler.psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && local_euler.psi >= 0)
    {
        if (2*M_PI-local_euler.psi+pre_yaw >= abs(pre_yaw)+local_euler.psi)
        {
            yaw_diff = abs(pre_yaw)+local_euler.psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-local_euler.psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }

    yaw_sum = yaw_sum + yaw_diff;
    pre_yaw = local_euler.psi;

    // set initial states
    ROS_INFO("Setting initial states...");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", local_pos.x, local_pos.y, local_pos.z);
    ROS_INFO("Euler angles: phi=%.2f, theta=%.2f, psi=%.2f", local_euler.phi, local_euler.theta, local_euler.psi);
    ROS_INFO("Yaw sum: %.2f", yaw_sum);
    
    // Check velocity data validity
    if (v_linear_body.size() < 3 || v_angular_body.size() < 3) {
        ROS_WARN("Velocity data not available, using zeros");
        v_linear_body = Vector3d::Zero();
        v_angular_body = Vector3d::Zero();
    }
    
    ROS_INFO("Linear velocity: u=%.2f, v=%.2f, w=%.2f", v_linear_body[0], v_linear_body[1], v_linear_body[2]);
    ROS_INFO("Angular velocity: p=%.2f, q=%.2f, r=%.2f", v_angular_body[0], v_angular_body[1], v_angular_body[2]);
    
    // Set initial state to match ACADOS model exactly
    // ACADOS expects: [u, v, w, p, q, r, x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
    acados_in.x0[0] = v_linear_body[0];  // u
    acados_in.x0[1] = v_linear_body[1];  // v
    acados_in.x0[2] = v_linear_body[2];  // w
    acados_in.x0[3] = v_angular_body[0]; // p
    acados_in.x0[4] = v_angular_body[1]; // q
    acados_in.x0[5] = v_angular_body[2]; // r
    acados_in.x0[6] = local_pos.x;       // x
    acados_in.x0[7] = local_pos.y;       // y
    acados_in.x0[8] = local_pos.z;       // z
    acados_in.x0[9] = local_euler.phi;   // phi
    acados_in.x0[10] = local_euler.theta; // theta
    acados_in.x0[11] = yaw_sum;          // psi (using yaw_sum for continuity)
    acados_in.x0[12] = 0.0;              // disturbance x
    acados_in.x0[13] = 0.0;              // disturbance y
    acados_in.x0[14] = 0.0;              // disturbance z
    acados_in.x0[15] = 0.0;              // disturbance phi
    acados_in.x0[16] = 0.0;              // disturbance theta
    acados_in.x0[17] = 0.0;              // disturbance psi
    
    // Debug: Print initial state values
    ROS_INFO("Initial state values:");
    ROS_INFO("x0[0-5] (velocities): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", 
             acados_in.x0[0], acados_in.x0[1], acados_in.x0[2], 
             acados_in.x0[3], acados_in.x0[4], acados_in.x0[5]);
    ROS_INFO("x0[6-11] (positions): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", 
             acados_in.x0[6], acados_in.x0[7], acados_in.x0[8], 
             acados_in.x0[9], acados_in.x0[10], acados_in.x0[11]);
    ROS_INFO("x0[12-17] (disturbances): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", 
             acados_in.x0[12], acados_in.x0[13], acados_in.x0[14], 
             acados_in.x0[15], acados_in.x0[16], acados_in.x0[17]);
    
    ROS_INFO("Initial states set successfully");
    ROS_INFO("=== DEBUG: NEW CODE COMPILED SUCCESSFULLY ===");
    ROS_INFO("Setting constraint bounds...");
    ROS_INFO("mpc_capsule pointer: %p", (void*)mpc_capsule);
    if (mpc_capsule == nullptr) {
        ROS_ERROR("mpc_capsule is null!");
        return;
    }
    ROS_INFO("nlp_config pointer: %p", (void*)mpc_capsule->nlp_config);
    ROS_INFO("nlp_dims pointer: %p", (void*)mpc_capsule->nlp_dims);
    ROS_INFO("nlp_in pointer: %p", (void*)mpc_capsule->nlp_in);
    
    // Create proper bounds arrays instead of using initial state for both bounds
    double lbx[18], ubx[18];
    
    // Set lower bounds - allow reasonable ranges for all states
    lbx[0] = -5.0;    // u velocity (reduced from -10)
    lbx[1] = -5.0;    // v velocity (reduced from -10)
    lbx[2] = -5.0;    // w velocity (reduced from -20)
    lbx[3] = -0.5;    // p angular velocity (reduced from -1)
    lbx[4] = -0.5;    // q angular velocity (reduced from -1)
    lbx[5] = -0.5;    // r angular velocity (reduced from -1)
    lbx[6] = -50.0;   // x position (reduced from -100)
    lbx[7] = -50.0;   // y position (reduced from -100)
    lbx[8] = -100.0;  // z position (depth) - keep large range for underwater
    lbx[9] = -M_PI/2; // phi (roll) - reduced from -M_PI
    lbx[10] = -M_PI/2; // theta (pitch) - reduced from -M_PI
    lbx[11] = -M_PI;   // psi (yaw) - keep full range for navigation
    lbx[12] = -5.0;    // disturbance x (reduced from -10)
    lbx[13] = -5.0;    // disturbance y (reduced from -10)
    lbx[14] = -5.0;    // disturbance z (reduced from -10)
    lbx[15] = -0.5;    // disturbance phi (reduced from -1)
    lbx[16] = -0.5;    // disturbance theta (reduced from -1)
    lbx[17] = -0.5;    // disturbance psi (reduced from -1)
    
    // Set upper bounds
    ubx[0] = 5.0;     // u velocity (reduced from 10)
    ubx[1] = 5.0;     // v velocity (reduced from 10)
    ubx[2] = 5.0;     // w velocity (reduced from 20)
    ubx[3] = 0.5;     // p angular velocity (reduced from 1)
    ubx[4] = 0.5;     // q angular velocity (reduced from 1)
    ubx[5] = 0.5;     // r angular velocity (reduced from 1)
    ubx[6] = 50.0;    // x position (reduced from 100)
    ubx[7] = 50.0;    // y position (reduced from 100)
    ubx[8] = 100.0;   // z position (depth) - keep large range for underwater
    ubx[9] = M_PI/2;  // phi (roll) - reduced from M_PI
    ubx[10] = M_PI/2; // theta (pitch) - reduced from M_PI
    ubx[11] = M_PI;   // psi (yaw) - keep full range for navigation
    ubx[12] = 5.0;    // disturbance x (reduced from 10)
    ubx[13] = 5.0;    // disturbance y (reduced from 10)
    ubx[14] = 5.0;    // disturbance z (reduced from 10)
    ubx[15] = 0.5;    // disturbance phi (reduced from 1)
    ubx[16] = 0.5;    // disturbance theta (reduced from 1)
    ubx[17] = 0.5;    // disturbance psi (reduced from 1)
    
    // Set the bounds in ACADOS
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", lbx);
    ROS_INFO("Lower bounds set successfully");
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", ubx);
    ROS_INFO("Upper bounds set successfully");
    
    // Debug: Check if initial state is within bounds
    ROS_INFO("Checking initial state vs bounds:");
    // Note: The bounds are set to the same values as the initial state in the current code
    // This is just a placeholder check - in a real implementation, you would compare against actual bounds
    for (int i = 0; i < 18; i++) {
        ROS_INFO("State %d: %.6f", i, acados_in.x0[i]);
    }
    
    // Check if the initial state matches the reference trajectory
    ROS_INFO("Checking initial state vs reference:");
    ROS_INFO("Initial position: (%.6f, %.6f, %.6f)", acados_in.x0[6], acados_in.x0[7], acados_in.x0[8]);
    ROS_INFO("Reference position: (%.6f, %.6f, %.6f)", acados_in.yref[0][0], acados_in.yref[0][1], acados_in.yref[0][2]);
    
    // Calculate position error
    double pos_error_x = acados_in.x0[6] - acados_in.yref[0][0];
    double pos_error_y = acados_in.x0[7] - acados_in.yref[0][1];
    double pos_error_z = acados_in.x0[8] - acados_in.yref[0][2];
    double total_pos_error = sqrt(pos_error_x*pos_error_x + pos_error_y*pos_error_y + pos_error_z*pos_error_z);
    ROS_INFO("Position error: (%.6f, %.6f, %.6f), total: %.6f", pos_error_x, pos_error_y, pos_error_z, total_pos_error);
    
    // Check if the error is too large (might cause infeasibility)
    if (total_pos_error > 10.0) {
        ROS_WARN("Large position error detected (%.6f), this might cause QP solver failure", total_pos_error);
    }
    
    ROS_INFO("Bounds check completed");
    
    ROS_INFO("Starting parameter setup...");
    ROS_INFO("BLUEROV2_N = %d", BLUEROV2_N);
    ROS_INFO("COMPENSATE_D = %s", COMPENSATE_D ? "true" : "false");
    
    // set parameters
    for (int i = 0; i < BLUEROV2_N+1; i++)
    {
        ROS_INFO("Setting parameters for step %d", i);
        
        if(COMPENSATE_D == false){
            ROS_INFO("Using disturbance compensation (false)");
            acados_param[i][0] = solver_param.disturbance_x;
            acados_param[i][1] = solver_param.disturbance_y;
            acados_param[i][2] = solver_param.disturbance_z;
            acados_param[i][3] = solver_param.disturbance_phi;
            acados_param[i][4] = solver_param.disturbance_theta;
            acados_param[i][5] = solver_param.disturbance_psi;
        }
        else if(COMPENSATE_D == true){
            ROS_INFO("Using disturbance compensation (true)");
            acados_param[i][0] = esti_x(12)/rotor_constant;
            acados_param[i][1] = esti_x(13)/rotor_constant;
            acados_param[i][2] = esti_x(14)/rotor_constant;
            acados_param[i][3] = solver_param.disturbance_phi;
            acados_param[i][4] = solver_param.disturbance_theta;
            acados_param[i][5] = esti_x(17)/rotor_constant;
            
            // acados_param[i][0] = compensate_f_body[0]/rotor_constant;
            // acados_param[i][1] = esti_x(13)/rotor_constant;
            // acados_param[i][2] = esti_x(14)/rotor_constant;
            // acados_param[i][3] = solver_param.disturbance_phi;
            // acados_param[i][4] = solver_param.disturbance_theta;
            // acados_param[i][5] = esti_x(17)/rotor_constant;
        }
        
        ROS_INFO("Calling bluerov2_acados_update_params for step %d", i);
        bluerov2_acados_update_params(mpc_capsule,i,acados_param[i],BLUEROV2_NP);
        ROS_INFO("Parameters updated for step %d", i);
    }
    
    ROS_INFO("Parameter setup completed successfully");
    
    // change into form of (-pi, pi)
    ROS_INFO("Processing yaw reference...");
    if(sin(acados_in.yref[0][5]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][5],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
    }
    
    ROS_INFO("Yaw reference processed: %.2f", yaw_ref);

    // set reference
    ROS_INFO("Setting reference trajectory...");
    ROS_INFO("Calling ref_cb with line_number: %d", line_number);
    ref_cb(line_number);
    ROS_INFO("ref_cb completed successfully"); 
    line_number++;
    
    // Note: Reference trajectory is now generated dynamically based on current robot position
    // No need to adjust it further since it already starts from current position
    ROS_INFO("Reference trajectory is dynamically generated from current robot position");
    
    // Debug: Print reference values
    ROS_INFO("Reference values for step 0:");
    for (int j = 0; j < BLUEROV2_NY; j++) {
        ROS_INFO("yref[0][%d] = %.6f", j, acados_in.yref[0][j]);
    }
    
    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }
    ROS_INFO("Reference trajectory set successfully");

    // Solve OCP
    ROS_INFO("Calling acados solver...");
    
    // Additional debugging: Check if initial state is feasible
    ROS_INFO("Final check before solving:");
    ROS_INFO("Initial state: x=%.3f, y=%.3f, z=%.3f", acados_in.x0[6], acados_in.x0[7], acados_in.x0[8]);
    ROS_INFO("Reference: x=%.3f, y=%.3f, z=%.3f", acados_in.yref[0][0], acados_in.yref[0][1], acados_in.yref[0][2]);
    ROS_INFO("Position error: %.3f", sqrt(pow(acados_in.x0[6] - acados_in.yref[0][0], 2) + 
                                         pow(acados_in.x0[7] - acados_in.yref[0][1], 2) + 
                                         pow(acados_in.x0[8] - acados_in.yref[0][2], 2)));
    
    // Check for numerical issues in the initial state
    bool has_numerical_issues = false;
    for (int i = 0; i < 18; i++) {
        if (std::isnan(acados_in.x0[i]) || std::isinf(acados_in.x0[i])) {
            ROS_ERROR("Numerical issue detected in state %d: %.6f", i, acados_in.x0[i]);
            has_numerical_issues = true;
        }
    }
    
    if (has_numerical_issues) {
        ROS_WARN("Numerical issues detected, using fallback control");
        acados_status = 4;  // Force fallback control
    } else {
        // Record start time for performance monitoring
        ros::Time solve_start_time = ros::Time::now();
        
        acados_status = bluerov2_acados_solve(mpc_capsule);
        
        // Calculate solve time
        ros::Duration solve_time = ros::Time::now() - solve_start_time;
        ROS_INFO("ACADOS solve completed in %.3f seconds", solve_time.toSec());
        
        // Performance monitoring
        if (solve_time.toSec() > 0.1) {
            ROS_WARN("ACADOS solve took %.3f seconds - this might indicate performance issues", solve_time.toSec());
        }
    }
    
    // If first attempt fails, try with much more relaxed bounds
    if (acados_status != 0) {
        ROS_WARN("First solver attempt failed, trying with much more relaxed bounds...");
        
        // Much more relaxed bounds - increase range by 10x instead of 2x
        double relaxed_lbx[18], relaxed_ubx[18];
        for (int i = 0; i < 18; i++) {
            relaxed_lbx[i] = lbx[i] * 10.0;  // 10x the range
            relaxed_ubx[i] = ubx[i] * 10.0;
        }
        
        // Set relaxed bounds
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", relaxed_lbx);
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", relaxed_ubx);
        
        // Try solving again
        acados_status = bluerov2_acados_solve(mpc_capsule);
        ROS_INFO("Second solver attempt completed with status: %d", acados_status);
    }
    
    // If still failing, try with even more aggressive relaxation
    if (acados_status != 0) {
        ROS_WARN("Second solver attempt failed, trying with extremely relaxed bounds...");
        
        // Extremely relaxed bounds - essentially no bounds
        double no_bounds_lbx[18], no_bounds_ubx[18];
        for (int i = 0; i < 18; i++) {
            no_bounds_lbx[i] = -1000.0;  // Very large negative bounds
            no_bounds_ubx[i] = 1000.0;   // Very large positive bounds
        }
        
        // Set extremely relaxed bounds
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", no_bounds_lbx);
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", no_bounds_ubx);
        
        // Try solving again
        acados_status = bluerov2_acados_solve(mpc_capsule);
        ROS_INFO("Third solver attempt completed with status: %d", acados_status);
    }
    
    ROS_INFO("Acados solver completed with status: %d", acados_status);
    
    // Add detailed status information
    if (acados_status == 0) {
        ROS_INFO("ACADOS: SUCCESS");
    } else if (acados_status == 1) {
        ROS_WARN("ACADOS: Maximum number of iterations reached");
    } else if (acados_status == 2) {
        ROS_WARN("ACADOS: Minimum step size reached");
    } else if (acados_status == 3) {
        ROS_WARN("ACADOS: Maximum CPU time reached");
    } else if (acados_status == 4) {
        ROS_ERROR("ACADOS: QP solver failed");
        // Additional debugging for QP failure
        ROS_ERROR("QP solver failed - this usually indicates:");
        ROS_ERROR("1. Infeasible problem (conflicting constraints)");
        ROS_ERROR("2. Numerical issues in the problem formulation");
        ROS_ERROR("3. Invalid bounds or reference values");
        ROS_ERROR("4. Cost function weights too small or large");
    } else if (acados_status == 5) {
        ROS_ERROR("ACADOS: Hessian matrix not positive definite");
    } else if (acados_status == 6) {
        ROS_ERROR("ACADOS: Invalid arguments");
    } else {
        ROS_ERROR("ACADOS: Unknown error status: %d", acados_status);
    }
    
    // Debug: Print control outputs
    ROS_INFO("Control outputs:");
    for (int i = 0; i < BLUEROV2_NU; i++) {
        ROS_INFO("Thruster %d: %.6f", i, acados_out.u0[i]);
    }
    
    // Apply control outputs to thrusters
    // Note: ACADOS model has 4 control inputs, but robot has 6 thrusters
    // We'll use the first 4 thrusters and set the last 2 to zero
    current_t.t0 = acados_out.u0[0];
    current_t.t1 = acados_out.u0[1];
    current_t.t2 = acados_out.u0[2];
    current_t.t3 = acados_out.u0[3];
    current_t.t4 = 0.0;  // Set to zero since ACADOS model only has 4 inputs
    current_t.t5 = 0.0;  // Set to zero since ACADOS model only has 4 inputs
    
    // If QP solver failed, use simple fallback control
    if (acados_status != 0) {
        ROS_WARN("QP solver failed, using fallback control");
        
        // Calculate position error to determine control direction
        double pos_error_x = acados_in.x0[6] - acados_in.yref[0][0];
        double pos_error_y = acados_in.x0[7] - acados_in.yref[0][1];
        double pos_error_z = acados_in.x0[8] - acados_in.yref[0][2];
        
        // Calculate velocity error for damping
        double vel_error_x = acados_in.x0[0];  // Current velocity
        double vel_error_y = acados_in.x0[1];
        double vel_error_z = acados_in.x0[2];
        
        // Adaptive control gains based on error magnitude
        double total_error = sqrt(pos_error_x*pos_error_x + pos_error_y*pos_error_y + pos_error_z*pos_error_z);
        
        // Reduce gains for large errors to prevent instability
        double kp_base = 0.02;
        double kd_base = 0.05;
        double max_thrust = 0.2;
        
        if (total_error > 5.0) {
            kp_base *= 0.5;  // Reduce gains for large errors
            kd_base *= 0.5;
            max_thrust *= 0.5;
            ROS_WARN("Large error detected (%.3f), reducing control gains for stability", total_error);
        } else if (total_error < 0.1) {
            kp_base *= 1.5;  // Increase gains for small errors
            kd_base *= 1.5;
            ROS_INFO("Small error detected (%.3f), increasing control gains for precision", total_error);
        }
        
        // Calculate control inputs based on position and velocity error
        double thrust_x = -kp_base * pos_error_x - kd_base * vel_error_x;
        double thrust_y = -kp_base * pos_error_y - kd_base * vel_error_y;
        double thrust_z = -kp_base * pos_error_z - kd_base * vel_error_z;
        
        // Limit thrust values
        thrust_x = std::max(-max_thrust, std::min(max_thrust, thrust_x));
        thrust_y = std::max(-max_thrust, std::min(max_thrust, thrust_y));
        thrust_z = std::max(-max_thrust, std::min(max_thrust, thrust_z));
        
        // Apply fallback control with better thruster mapping
        current_t.t0 = thrust_x;  // Forward/backward
        current_t.t1 = thrust_y;  // Lateral
        current_t.t2 = thrust_z;  // Vertical
        current_t.t3 = 0.0;       // No rotational control for safety
        current_t.t4 = 0.0;
        current_t.t5 = 0.0;
        
        ROS_INFO("Fallback control: thrust_x=%.3f, thrust_y=%.3f, thrust_z=%.3f", 
                 thrust_x, thrust_y, thrust_z);
        ROS_INFO("Position errors: x=%.3f, y=%.3f, z=%.3f", pos_error_x, pos_error_y, pos_error_z);
        ROS_INFO("Velocity errors: x=%.3f, y=%.3f, z=%.3f", vel_error_x, vel_error_y, vel_error_z);
        ROS_INFO("Total error: %.3f, using gains: kp=%.4f, kd=%.4f", total_error, kp_base, kd_base);
    }
    
    ROS_INFO("Applied thrust values: t0=%.6f, t1=%.6f, t2=%.6f, t3=%.6f, t4=%.6f, t5=%.6f", 
             current_t.t0, current_t.t1, current_t.t2, current_t.t3, current_t.t4, current_t.t5);

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);
    
    // Apply straight line navigation corrections if enabled
    if (straight_line_mode) {
        // Apply control corrections directly to acados output
        attitude_control();
        depth_control();
        velocity_control();
        path_correction();
    }
    
    thrust0.data=(-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    thrust1.data=(-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    thrust2.data=(acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    thrust3.data=(acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    thrust4.data=(-acados_out.u0[2])/rotor_constant;
    thrust5.data=(-acados_out.u0[2])/rotor_constant;
    
    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);

    // publish reference pose
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y = acados_in.yref[0][1];
    ref_pose.pose.pose.position.z = acados_in.yref[0][2];
    ref_pose.pose.pose.orientation.x = quat_msg.x;
    ref_pose.pose.pose.orientation.y = quat_msg.y;
    ref_pose.pose.pose.orientation.z = quat_msg.z;
    ref_pose.pose.pose.orientation.w = quat_msg.w;
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "odom_frame";
    ref_pose.child_frame_id = "base_link";
    ref_pose_pub.publish(ref_pose);

    // publish error pose
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][5];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.position.z = acados_in.x0[2] - acados_in.yref[0][2];
    error_pose.pose.pose.orientation.x = quat_error_msg.x;
    error_pose.pose.pose.orientation.y = quat_error_msg.y;
    error_pose.pose.pose.orientation.z = quat_error_msg.z;
    error_pose.pose.pose.orientation.w = quat_error_msg.w;
    error_pose.header.stamp = ros::Time::now();
    error_pose.header.frame_id = "odom_frame";
    error_pose.child_frame_id = "base_link";

    error_pose_pub.publish(error_pose);

    // publish conrtrol input
    control_input0.data = acados_out.u0[0];
    control_input1.data = acados_out.u0[1];
    control_input2.data = acados_out.u0[2];
    control_input3.data = acados_out.u0[3];

    control_input0_pub.publish(control_input0);
    control_input1_pub.publish(control_input1);
    control_input2_pub.publish(control_input2);
    control_input3_pub.publish(control_input3);
    
}

// IMU callback
void BLUEROV2_DOB::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Update fault detection timer
    fault_detection.last_imu_time = ros::Time::now();
    
    // get linear acceleraton
    imu_acc.x = round(msg->linear_acceleration.x*10000)/10000;
    imu_acc.y = round(msg->linear_acceleration.y*10000)/10000;
    imu_acc.z = round(msg->linear_acceleration.z*10000)/10000-g;
    
    // get angular velocity
    imu_q.w = msg->orientation.w;
    imu_q.x = msg->orientation.x;
    imu_q.y = msg->orientation.y;
    imu_q.z = msg->orientation.z;
}

// pressure sensor callback
void BLUEROV2_DOB::pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &pressure)
{
    // Update fault detection timer
    fault_detection.last_pressure_time = ros::Time::now();
    
    fluid_p = pressure->fluid_pressure;
}

// Pointcloud callback from Realsense d435
void BLUEROV2_DOB::pcl_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloudPtr);

    // Calculate the center point coordinates
    float centerX = cloudPtr->width / 2;
    float centerY = cloudPtr->height / 2;

    // Define the ROI size
    int roiSize = 40;

    // Calculate the ROI boundaries
    int roiMinX = centerX - roiSize / 2;
    int roiMaxX = centerX + roiSize / 2;
    int roiMinY = centerY - roiSize / 2;
    int roiMaxY = centerY + roiSize / 2;

    // Create a new point cloud for the ROI
    pcl::PointCloud<pcl::PointXYZ>::Ptr roiCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate over the points within the ROI boundaries
    for (int y = roiMinY; y < roiMaxY; y++)
    {
        for (int x = roiMinX; x < roiMaxX; x++)
        {
            // Add the point to the ROI point cloud
            roiCloud->push_back(cloudPtr->at(x, y));
        }
    }

}

void BLUEROV2_DOB::thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index)
{
    double input = msg->data;
    //ROS_INFO("Received input for thruster %d: %f", index, input);
    switch (index)
    {
        case 0:
            current_t.t0 = input;
            break;
        case 1:
            current_t.t1 = input;
            break;
        case 2:
            current_t.t2 = input;
            break;
        case 3:
            current_t.t3 = input;
            break;
        case 4:
            current_t.t4 = input;
            break;
        case 5:
            current_t.t5 = input;
            break;
        default:
            ROS_WARN("Invalid thruster index: %d", index);
            break;
    }
}



// 4th order RK for integration
MatrixXd BLUEROV2_DOB::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> k1;
    Matrix<double,18,1> k2;
    Matrix<double,18,1> k3;
    Matrix<double,18,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
MatrixXd BLUEROV2_DOB::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,18,1> xdot;

    KAu = K*u;
    xdot << (cos(x(5))*cos(x(4)))*x(6) + (-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7) + (sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),  //xdot
            (sin(x(5))*cos(x(4)))*x(6) + (cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7) + (-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
            (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
            x(9) + (sin(x(5))*sin(x(4))/cos(x(4)))*x(10) + cos(x(3))*sin(x(4))/cos(x(4))*x(11),
            (cos(x(3)))*x(10) + (sin(x(3)))*x(11),
            (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11), 
            invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl[0]*x(6)+Dnl[0]*abs(x(6))*x(6)),    // xddot: M^-1[tau+w-C-g-D]
            invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl[1]*x(7)+Dnl[1]*abs(x(7))*x(7)),
            invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl[2]*x(8)+Dnl[2]*abs(x(8))*x(8)),
            invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl[3]*x(9)+Dnl[3]*abs(x(9))*x(9)),
            invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl[4]*x(10)+Dnl[4]*abs(x(10))*x(10)),
            invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl[5]*x(11)+Dnl[5]*abs(x(11))*x(11)),
            // invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl(0,0)*x(6)+added_mass[2]*x(2)*x(4)),    // xddot: M^-1[tau+w-C-g-D]
            // invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl(1,1)*x(7)-added_mass[2]*x(2)*x(3)-added_mass[0]*x(0)*x(5)),
            // invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl(2,2)*x(8)-added_mass[1]*x(1)*x(3)+added_mass[0]*x(0)*x(4)),
            // invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl(3,3)*x(9)-added_mass[2]*x(2)*x(1)+added_mass[1]*x(1)*x(2)-added_mass[5]*x(5)*x(4)+added_mass[4]*x(4)*x(5)),
            // invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl(4,4)*x(10)+added_mass[2]*x(2)*x(0)-added_mass[0]*x(0)*x(2)+added_mass[5]*x(5)*x(3)-added_mass[3]*x(3)*x(5)),
            // invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl(5,5)*x(11)-added_mass[1]*x(1)*x(0)+added_mass[0]*x(0)*x(1)-added_mass[4]*x(4)*x(3)+added_mass[3]*x(3)*x(4)),
            0,0,0,0,0,0;
            
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd BLUEROV2_DOB::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,18,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),
        x(6),x(7),x(8),x(9),x(10),x(11),
        M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl[0]*x(6)-Dnl[0]*abs(x(6))*x(6),        
        M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl[1]*x(7)-Dnl[1]*abs(x(7))*x(7),
        M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl[2]*x(8)-Dnl[2]*abs(x(8))*x(8),
        M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl[3]*x(9)-Dnl[3]*abs(x(9))*x(9),
        M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl[4]*x(10)-Dnl[4]*abs(x(10))*x(10),
        M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl[5]*x(11)-Dnl[5]*abs(x(11))*x(11);
        // M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl(0,0)*x(6)-added_mass[2]*x(2)*x(4),        
        // M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl(1,1)*x(7)+added_mass[2]*x(2)*x(3)+added_mass[0]*x(0)*x(5),
        // M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl(2,2)*x(8)+added_mass[1]*x(1)*x(3)-added_mass[0]*x(0)*x(4),
        // M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl(3,3)*x(9)+added_mass[2]*x(2)*x(1)-added_mass[1]*x(1)*x(2)+added_mass[5]*x(5)*x(4)-added_mass[4]*x(4)*x(5),
        // M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl(4,4)*x(10)-added_mass[2]*x(2)*x(0)+added_mass[0]*x(0)*x(2)-added_mass[5]*x(5)*x(3)+added_mass[3]*x(3)*x(5),
        // M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl(5,5)*x(11)+added_mass[1]*x(1)*x(0)-added_mass[0]*x(0)*x(1)+added_mass[4]*x(4)*x(3)-added_mass[3]*x(3)*x(4);

    return y;
}

// UKF initialization
void BLUEROV2_DOB::initialize_ukf() {
    lambda = alpha * alpha * (L + kappa) - L;
    num_sigma_points = 2 * L + 1;
    
    // Initialize sigma points matrix
    sigma_points = MatrixXd::Zero(L, num_sigma_points);
    
    // Initialize weights
    weights_m = MatrixXd::Zero(1, num_sigma_points);
    weights_c = MatrixXd::Zero(1, num_sigma_points);
    
    // Set weights
    weights_m(0, 0) = lambda / (L + lambda);
    weights_c(0, 0) = lambda / (L + lambda) + (1 - alpha * alpha + beta);
    
    for (int i = 1; i < num_sigma_points; i++) {
        weights_m(0, i) = 1.0 / (2 * (L + lambda));
        weights_c(0, i) = 1.0 / (2 * (L + lambda));
    }
}

// Generate sigma points
MatrixXd BLUEROV2_DOB::generate_sigma_points(MatrixXd x, MatrixXd P) {
    MatrixXd sigma_points(L, num_sigma_points);
    
    // Add small positive values to diagonal to ensure positive definiteness
    MatrixXd P_safe = P;
    for (int i = 0; i < P.rows(); i++) {
        P_safe(i, i) = std::max(P_safe(i, i), 1e-6);
    }
    
    MatrixXd sqrt_P = P_safe.llt().matrixL();
    
    // Center sigma point
    sigma_points.col(0) = x;
    
    // Generate remaining sigma points
    double gamma = sqrt(L + lambda);
    for (int i = 0; i < L; i++) {
        sigma_points.col(i + 1) = x + gamma * sqrt_P.col(i);
        sigma_points.col(i + L + 1) = x - gamma * sqrt_P.col(i);
    }
    
    return sigma_points;
}

// Transform sigma points
MatrixXd BLUEROV2_DOB::transform_sigma_points(MatrixXd sigma_points, std::function<MatrixXd(MatrixXd)> transform_func) {
    MatrixXd transformed_points(L, num_sigma_points);
    
    for (int i = 0; i < num_sigma_points; i++) {
        transformed_points.col(i) = transform_func(sigma_points.col(i));
    }
    
    return transformed_points;
}

// Compute mean from sigma points
MatrixXd BLUEROV2_DOB::compute_mean_from_sigma_points(MatrixXd sigma_points) {
    MatrixXd mean = MatrixXd::Zero(L, 1);
    
    for (int i = 0; i < num_sigma_points; i++) {
        mean += weights_m(0, i) * sigma_points.col(i);
    }
    
    return mean;
}

// Compute covariance from sigma points
MatrixXd BLUEROV2_DOB::compute_covariance_from_sigma_points(MatrixXd sigma_points, MatrixXd mean) {
    MatrixXd covariance = MatrixXd::Zero(L, L);
    
    for (int i = 0; i < num_sigma_points; i++) {
        MatrixXd diff = sigma_points.col(i) - mean;
        covariance += weights_c(0, i) * diff * diff.transpose();
    }
    
    return covariance;
}

// UKF implementation (replacing EKF)
void BLUEROV2_DOB::UKF() {
    // Check if we have received pose data
    if (!is_start) {
        ROS_WARN("UKF called before pose data received, skipping...");
        return;
    }
    
    // Get input and measurement
    meas_u << current_t.t0, current_t.t1, current_t.t2, current_t.t3, current_t.t4, current_t.t5;
    Matrix<double,6,1> tau;
    tau = K*meas_u;
    
    // Check for valid velocity data
    if (v_linear_body.size() < 3 || v_angular_body.size() < 3) {
        ROS_WARN("Velocity data not available, using zeros");
        v_linear_body = Vector3d::Zero();
        v_angular_body = Vector3d::Zero();
    }
    
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_euler.phi, local_euler.theta, local_euler.psi,
            v_linear_body[0], v_linear_body[1], v_linear_body[2], v_angular_body[0], v_angular_body[1], v_angular_body[2],
            tau(0),tau(1),tau(2),tau(3),tau(4),tau(5);
    
    // Prediction step
    MatrixXd sigma_points_pred = generate_sigma_points(esti_x, esti_P);
    
    // Transform sigma points through system dynamics
    auto system_transform = [this](MatrixXd x) -> MatrixXd {
        return RK4(x, meas_u);
    };
    MatrixXd sigma_points_transformed = transform_sigma_points(sigma_points_pred, system_transform);
    
    // Compute predicted mean and covariance
    MatrixXd x_pred = compute_mean_from_sigma_points(sigma_points_transformed);
    MatrixXd P_pred = compute_covariance_from_sigma_points(sigma_points_transformed, x_pred) + noise_Q;
    
    // Update step
    // Transform sigma points through measurement model
    auto measurement_transform = [this](MatrixXd x) -> MatrixXd {
        return h(x);
    };
    MatrixXd sigma_points_measurement = transform_sigma_points(sigma_points_transformed, measurement_transform);
    
    // Compute predicted measurement mean and covariance
    MatrixXd y_pred = compute_mean_from_sigma_points(sigma_points_measurement);
    MatrixXd Pyy = compute_covariance_from_sigma_points(sigma_points_measurement, y_pred) + noise_R;
    
    // Compute cross-covariance
    MatrixXd Pxy = MatrixXd::Zero(L, 18);
    for (int i = 0; i < num_sigma_points; i++) {
        MatrixXd diff_x = sigma_points_transformed.col(i) - x_pred;
        MatrixXd diff_y = sigma_points_measurement.col(i) - y_pred;
        Pxy += weights_c(0, i) * diff_x * diff_y.transpose();
    }
    
    // Compute Kalman gain with safety check
    MatrixXd Kal;
    double det = Pyy.determinant();
    if (std::abs(det) > 1e-10) {
        Kal = Pxy * Pyy.inverse();
    } else {
        // Use pseudo-inverse or skip update if matrix is singular
        Kal = Pxy * Pyy.completeOrthogonalDecomposition().pseudoInverse();
    }
    
    // Update state and covariance
    MatrixXd y_err = meas_y - y_pred;
    esti_x = x_pred + Kal * y_err;
    esti_P = P_pred - Kal * Pyy * Kal.transpose();
    
    // Convert body frame disturbance to inertial frame
    wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
            (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
            (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
            esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
            (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
            (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);
    
    // Publish estimated pose
    tf2::Quaternion quat;
    quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    esti_pose.pose.pose.position.x = esti_x(0);
    esti_pose.pose.pose.position.y = esti_x(1);
    esti_pose.pose.pose.position.z = esti_x(2);
    esti_pose.pose.pose.orientation.x = quat_msg.x;
    esti_pose.pose.pose.orientation.y = quat_msg.y;
    esti_pose.pose.pose.orientation.z = quat_msg.z;
    esti_pose.pose.pose.orientation.w = quat_msg.w;
    esti_pose.twist.twist.linear.x = esti_x(6);
    esti_pose.twist.twist.linear.y = esti_x(7);
    esti_pose.twist.twist.linear.z = esti_x(8);
    esti_pose.twist.twist.angular.x = esti_x(9);
    esti_pose.twist.twist.angular.y = esti_x(10);
    esti_pose.twist.twist.angular.z = esti_x(11);
    esti_pose.header.stamp = ros::Time::now();
    esti_pose.header.frame_id = "odom_frame";
    esti_pose.child_frame_id = "base_link";
    esti_pose_pub.publish(esti_pose);
    
    // Publish estimated disturbance
    esti_disturbance.pose.pose.position.x = wf_disturbance(0);
    esti_disturbance.pose.pose.position.y = wf_disturbance(1);
    esti_disturbance.pose.pose.position.z = wf_disturbance(2);
    esti_disturbance.twist.twist.angular.x = wf_disturbance(3);
    esti_disturbance.twist.twist.angular.y = wf_disturbance(4);
    esti_disturbance.twist.twist.angular.z = wf_disturbance(5);
    esti_disturbance.header.stamp = ros::Time::now();
    esti_disturbance.header.frame_id = "odom_frame";
    esti_disturbance.child_frame_id = "base_link";
    esti_disturbance_pub.publish(esti_disturbance);
    
    // Publish applied disturbance
    applied_disturbance.pose.pose.position.x = applied_wrench.fx;
    applied_disturbance.pose.pose.position.y = applied_wrench.fy;
    applied_disturbance.pose.pose.position.z = applied_wrench.fz;
    applied_disturbance.twist.twist.angular.x = applied_wrench.tx;
    applied_disturbance.twist.twist.angular.y = applied_wrench.ty;
    applied_disturbance.twist.twist.angular.z = applied_wrench.tz;
    applied_disturbance.header.stamp = ros::Time::now();
    applied_disturbance.header.frame_id = "odom_frame";
    applied_disturbance.child_frame_id = "base_link";
    applied_disturbance_pub.publish(applied_disturbance);
}

// Trajectory visualization functions
void BLUEROV2_DOB::update_trajectory() {
    // Create pose stamped message for current position
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "odom_frame";
    pose_stamped.header.stamp = ros::Time::now();
    
    pose_stamped.pose.position.x = local_pos.x;
    pose_stamped.pose.position.y = local_pos.y;
    pose_stamped.pose.position.z = local_pos.z;
    
    // Convert euler angles to quaternion
    tf2::Quaternion quat;
    quat.setRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();
    
    // Add to trajectory points
    trajectory_points.push_back(pose_stamped);
    
    // Keep only last 1000 points to avoid memory issues
    if (trajectory_points.size() > max_trajectory_points) {
        trajectory_points.erase(trajectory_points.begin());
    }
    
    // Debug: Print trajectory info every 100 points
    if (trajectory_points.size() % 100 == 0) {
        ROS_INFO("Trajectory points: %zu, Current position: (%.2f, %.2f, %.2f)",
                 trajectory_points.size(), local_pos.x, local_pos.y, local_pos.z);
        
        // Calculate straightness if we have enough points
        if (trajectory_points.size() > 10) {
            calculate_trajectory_straightness();
        }
    }
}

void BLUEROV2_DOB::calculate_trajectory_straightness() {
    // Calculate how straight the trajectory is
    if (trajectory_points.size() < 10) return;
    
    // Get start and end points
    auto start_point = trajectory_points.front();
    auto end_point = trajectory_points.back();
    
    // Calculate ideal straight line distance
    double ideal_distance = sqrt(pow(end_point.pose.position.x - start_point.pose.position.x, 2) +
                                pow(end_point.pose.position.y - start_point.pose.position.y, 2) +
                                pow(end_point.pose.position.z - start_point.pose.position.z, 2));
    
    // Calculate actual trajectory distance
    double actual_distance = 0.0;
    for (size_t i = 1; i < trajectory_points.size(); i++) {
        auto prev = trajectory_points[i-1];
        auto curr = trajectory_points[i];
        actual_distance += sqrt(pow(curr.pose.position.x - prev.pose.position.x, 2) +
                              pow(curr.pose.position.y - prev.pose.position.y, 2) +
                              pow(curr.pose.position.z - prev.pose.position.z, 2));
    }
    
    // Calculate straightness ratio (1.0 = perfectly straight)
    double straightness = ideal_distance / actual_distance;
    
    // Calculate lateral deviation from straight line
    double max_lateral_deviation = 0.0;
    for (const auto& point : trajectory_points) {
        double lateral_deviation = abs(point.pose.position.y);  // Distance from x-axis
        if (lateral_deviation > max_lateral_deviation) {
            max_lateral_deviation = lateral_deviation;
        }
    }
    
    ROS_INFO("Trajectory Analysis - Straightness: %.3f, Max lateral deviation: %.2fm", 
             straightness, max_lateral_deviation);
    
    // Provide feedback on trajectory quality
    if (straightness > 0.95) {
        ROS_INFO("Excellent straight line performance!");
    } else if (straightness > 0.9) {
        ROS_INFO("Good straight line performance");
    } else if (straightness > 0.8) {
        ROS_WARN("Moderate straight line performance - some deviation detected");
    } else {
        ROS_WARN("Poor straight line performance - significant deviation detected");
    }
}

void BLUEROV2_DOB::publish_trajectory() {
    if (trajectory_points.empty()) return;
    
    nav_msgs::Path trajectory_path;
    trajectory_path.header.stamp = ros::Time::now();
    trajectory_path.header.frame_id = "odom_frame";
    
    // Add all recorded trajectory points
    for (const auto& pose : trajectory_points) {
        trajectory_path.poses.push_back(pose);
    }
    
    // Publish trajectory for RViz visualization
    trajectory_pub.publish(trajectory_path);
    
    // Limit trajectory size to prevent memory issues
    if (trajectory_points.size() > 1000) {
        trajectory_points.erase(trajectory_points.begin(), trajectory_points.begin() + 100);
    }
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DOB::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,18,18> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DOB::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,18,18> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}

void BLUEROV2_DOB::applyBodyWrench()
{
    // initialize periodic disturbance
    // double amplitudeScalingFactor;

    // initialize random disturbance
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(0.5, 1.0);

    // initialize disturbance data from file
    std::vector<double> data_fx;
    std::vector<double> data_fy;
    std::vector<double> data_fz;
    std::vector<double> data_tz;
    const char * fx_c = WRENCH_FX.c_str();
    const char * fy_c = WRENCH_FY.c_str();
    const char * fz_c = WRENCH_FZ.c_str();
    const char * tz_c = WRENCH_TZ.c_str();

    if(READ_WRENCH == 0){
        // generate periodical disturbance
        if(dis_time > periodic_counter*M_PI)
        {
            amplitudeScalingFactor_X = distribution(gen)*6;
            amplitudeScalingFactor_Y = distribution(gen)*6;
            amplitudeScalingFactor_Z = distribution(gen)*6;
            amplitudeScalingFactor_N = distribution(gen)*6;
            periodic_counter++;
        }
        applied_wrench.fx = sin(dis_time)*amplitudeScalingFactor_X;
        applied_wrench.fy = sin(dis_time)*amplitudeScalingFactor_Y;
        applied_wrench.fz = sin(dis_time)*amplitudeScalingFactor_Z;
        applied_wrench.tz = (sin(dis_time)*amplitudeScalingFactor_Y)/3;
        if(dis_time>10){
            applied_wrench.fx = applied_wrench.fx;
            applied_wrench.fy = applied_wrench.fy;
            applied_wrench.fz = applied_wrench.fz;
            applied_wrench.tz = applied_wrench.tz;
        }

        dis_time = dis_time+dt*2.5;
        // std::cout << "amplitudeScalingFactor_Z:  " << amplitudeScalingFactor_Z << "  amplitudeScalingFactor_N:  " << amplitudeScalingFactor_N << std::endl;
    }
    else if(READ_WRENCH == 1){
        // generate random disturbance
        // if(rand_counter > 10){
        //     applied_wrench.fx = distribution(gen)*5;
        //     applied_wrench.fy = distribution(gen)*5;
        //     applied_wrench.fz = distribution(gen)*5;
        //     applied_wrench.tx = distribution(gen);
        //     applied_wrench.ty = distribution(gen);
        //     applied_wrench.tz = distribution(gen);
        //     rand_counter = 0;
        // }
        // else{
        //     rand_counter++;
        // }
        // generate constant disturbance
        applied_wrench.fx = 10;
        applied_wrench.fy = 10;
        applied_wrench.fz = 10;
        applied_wrench.tz = 0;
    }
    else if(READ_WRENCH == 2){
        // std::cout << "read from file starts" << std::endl;
        // read disturbance from file
        // read force x
        std::ifstream fx_file(fx_c);
        if (!fx_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fx;
        while (fx_file >> value_fx) {
            data_fx.push_back(value_fx); // Load the data into the vector
        }
        fx_file.close();

        // read force y
        std::ifstream fy_file(fy_c);
        if (!fy_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fy;
        while (fy_file >> value_fy) {
            data_fy.push_back(value_fy); // Load the data into the vector
        }
        fy_file.close();

        // read force z
        std::ifstream fz_file(fz_c);
        if (!fz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fz;
        while (fz_file >> value_fz) {
            data_fz.push_back(value_fz); // Load the data into the vector
        }
        fz_file.close();

        // read torque z
        std::ifstream tz_file(tz_c);
        if (!tz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_tz;
        while (tz_file >> value_tz) {
            data_tz.push_back(value_tz); // Load the data into the vector
        }
        tz_file.close();

        applied_wrench.fx  = data_fx[fx_counter];
        applied_wrench.fy  = data_fy[fx_counter];
        applied_wrench.fz  = data_fz[fx_counter];
        applied_wrench.tz  = data_tz[fx_counter];
        fx_counter++;
    }
    
    // call ros service apply_body_wrench
    body_wrench.request.body_name = "bluerov2/base_link";
    body_wrench.request.start_time = ros::Time(0.0);
    body_wrench.request.reference_frame = "world";
    body_wrench.request.duration = ros::Duration(1000);
    body_wrench.request.reference_point.x = 0.0;
    body_wrench.request.reference_point.y = 0.0;
    body_wrench.request.reference_point.z = 0.0;
    body_wrench.request.wrench.force.x = applied_wrench.fx;
    body_wrench.request.wrench.force.y = applied_wrench.fy;
    body_wrench.request.wrench.force.z = applied_wrench.fz;
    body_wrench.request.wrench.torque.x = applied_wrench.tx;
    body_wrench.request.wrench.torque.y = applied_wrench.ty;
    body_wrench.request.wrench.torque.z = applied_wrench.tz;
    client.call(body_wrench);
    
}

// coriolis and centripetal forces C(v) = C_RB(v) + C_A(v)
// v(0-5):u, v, w, p, q, r
MatrixXd BLUEROV2_DOB::dynamics_C(MatrixXd v)
{
    Matrix<double,6,6> C;
    C<< 0, 0, 0, 0, mass*v(2)+added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1),
        0, 0, 0, -mass*v(2)-added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0),
        0, 0, 0, mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0,
        0, mass*v(2)-added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1), 0, Iz*v(5)-added_mass[5]*v(5), -Iy*v(4)+added_mass[4]*v(4),
        -mass*v(2)+added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0), -Iz*v(5)+added_mass[5]*v(5), 0, Ix*v(3)-added_mass[3]*v(3),
        mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0, Iy*v(4)-added_mass[4]*v(4), -Ix*v(3)+added_mass[3]*v(3), 0;
    return C;
}

// damping forces D(v) = D_L + D_NL(v)
// v(0-5):u, v, w, p, q, r
MatrixXd BLUEROV2_DOB::dynamics_D(MatrixXd v)
{
    Matrix<double,1,6> D_diagonal;
    D_diagonal << -Dl[0]-Dnl[0]*abs(v(0)), -Dl[1]-Dnl[1]*abs(v(1)), -Dl[2]-Dnl[2]*abs(v(2)),
                -Dl[3]-Dnl[3]*abs(v(3)), -Dl[4]-Dnl[4]*abs(v(4)), -Dl[5]-Dnl[5]*abs(v(5));

    Matrix<double,6,6> D;
    D = D_diagonal.asDiagonal();

    return D;
}

// gravitational and buoyancy forces g
// euler(0-2): phi, theta, psi
MatrixXd BLUEROV2_DOB::dynamics_g(MatrixXd euler)
{
    Matrix<double,6,1> g;

    g << bouyancy*sin(euler(1)),
        -bouyancy*cos(euler(1))*sin(euler(0)),
        -bouyancy*cos(euler(1))*cos(euler(0)),
        mass*ZG*g*cos(euler(1))*sin(euler(0)),
        mass*ZG*g*sin(euler(1)),
        0;

    return g;
}

// LiDAR callback functions
void BLUEROV2_DOB::forward_lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    fault_detection.last_lidar_time = ros::Time::now();
    
    // Find minimum distance
    float min_distance = scan->range_max;
    for (int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] < min_distance && scan->ranges[i] > scan->range_min) {
            min_distance = scan->ranges[i];
        }
    }
    
    forward_lidar_distance = min_distance;
}

// Straight line navigation functions
void BLUEROV2_DOB::straight_line_navigation() {
    if (!straight_line_mode) return;
    
    // Check if we have received pose data
    if (!is_start) {
        ROS_WARN("No pose data received yet, cannot perform navigation");
        return;
    }
    
    ROS_INFO("Straight line navigation active - Position: x=%.2f, y=%.2f, z=%.2f", 
             local_pos.x, local_pos.y, local_pos.z);
    
    // Record current position for trajectory visualization
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "odom_frame";
    current_pose.pose.position.x = local_pos.x;
    current_pose.pose.position.y = local_pos.y;
    current_pose.pose.position.z = local_pos.z;
    
    // Convert Euler angles to quaternion
    tf2::Quaternion quat;
    quat.setRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    current_pose.pose.orientation.x = quat.x();
    current_pose.pose.orientation.y = quat.y();
    current_pose.pose.orientation.z = quat.z();
    current_pose.pose.orientation.w = quat.w();
    
    trajectory_points.push_back(current_pose);
    
    // Publish trajectory for RViz visualization
    publish_trajectory();
    
    // Update sensor timeouts
    update_sensor_timeouts();
    
    // Check sensor health
    if (!check_sensor_health()) {
        ROS_WARN("Sensor fault detected, stopping straight line navigation");
        stop_straight_line_navigation();
        return;
    }
    
    // Perform sensor fusion
    sensor_fusion();
    
    // Generate straight line trajectory
    generate_straight_line_trajectory();
    
    // Apply control corrections
    attitude_control();
    depth_control();
    velocity_control();
    path_correction();
}

void BLUEROV2_DOB::attitude_control() {
    // Keep horizontal attitude
    double roll_correction = -local_euler.phi * 0.5;
    double pitch_correction = -local_euler.theta * 0.5;
    
    // Apply to control output
    acados_out.u0[3] += roll_correction;   // Roll control
    acados_out.u0[4] += pitch_correction;  // Pitch control
}

void BLUEROV2_DOB::depth_control() {
    double depth_error = target_depth - local_pos.z;
    double depth_correction = depth_error * 0.3;
    
    // Apply to vertical thrust
    acados_out.u0[2] += depth_correction;
}

void BLUEROV2_DOB::velocity_control() {
    // Maintain forward velocity
    double surge_error = target_surge_velocity - v_linear_body[0];
    acados_out.u0[0] += surge_error * 0.4;
    
    // Minimize lateral velocity
    double sway_correction = -v_linear_body[1] * 0.4;
    acados_out.u0[1] += sway_correction;
}

void BLUEROV2_DOB::path_correction() {
    // Use forward LiDAR for obstacle detection and path correction
    double obstacle_distance = forward_lidar_distance;
    double safe_distance = 5.0;  // Safe distance from obstacles
    
    // If obstacle is too close, reduce forward velocity
    if (obstacle_distance < safe_distance) {
        double velocity_reduction = (safe_distance - obstacle_distance) / safe_distance;
        acados_out.u0[0] *= (1.0 - velocity_reduction * 0.5);  // Reduce forward thrust
        ROS_WARN("Obstacle detected at %.2f m, reducing velocity", obstacle_distance);
    }
    
    // Simple lateral correction based on current position
    // Keep robot moving in a straight line along x-axis
    double lateral_error = local_pos.y;  // Distance from x-axis
    double lateral_correction = -lateral_error * 0.3;  // Proportional correction
    acados_out.u0[1] += lateral_correction;
    
    ROS_INFO("Path correction: obstacle=%.2fm, lateral_error=%.2fm, correction=%.3f", 
             obstacle_distance, lateral_error, lateral_correction);
}

void BLUEROV2_DOB::generate_straight_line_trajectory() {
    // Use current pose for trajectory generation
    double current_x = local_pos.x;
    double current_y = local_pos.y;
    double current_z = local_pos.z;
    double current_psi = local_euler.psi;
    
    // Use the same target logic as generate_dynamic_reference_trajectory
    // Calculate target position - use more reasonable targets to avoid large errors
    double target_x = current_x + 5.0;  // Move 5m forward from current position
    double target_y = current_y;        // Stay at current y position (straight line)
    double target_z = current_z;        // Maintain current depth
    double target_psi = current_psi;    // Maintain current heading
    
    // Try to get target from ROS parameters, otherwise use reasonable defaults
    if (!ros::param::get("/bluerov2_dob_node/target_x", target_x)) {
        // If no parameter set, use a reasonable offset from current position
        target_x = current_x + 5.0;  // 5m forward
        ROS_INFO("Using reasonable target_x: %.3f (%.3f + 5.0)", target_x, current_x);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_y", target_y)) {
        target_y = current_y;  // Stay at current y
        ROS_INFO("Using reasonable target_y: %.3f (current position)", target_y);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_z", target_z)) {
        target_z = current_z;  // Maintain current depth
        ROS_INFO("Using reasonable target_z: %.3f (current depth)", target_z);
    }
    if (!ros::param::get("/bluerov2_dob_node/target_psi", target_psi)) {
        target_psi = current_psi;  // Maintain current heading
        ROS_INFO("Using reasonable target_psi: %.3f (current heading)", target_psi);
    }
    
    // Calculate distance to target
    double distance_to_target = sqrt(pow(target_x - current_x, 2) + 
                                   pow(target_y - current_y, 2) + 
                                   pow(target_z - current_z, 2));
    
    ROS_INFO("Distance to target: %.3f meters", distance_to_target);
    
    // Check if distance is reasonable for MPC solver
    if (distance_to_target > 20.0) {
        ROS_WARN("Target distance %.3f is too large, limiting to 20m for MPC stability", distance_to_target);
        // Limit the target to a reasonable distance
        double scale_factor = 20.0 / distance_to_target;
        target_x = current_x + (target_x - current_x) * scale_factor;
        target_y = current_y + (target_y - current_y) * scale_factor;
        target_z = current_z + (target_z - current_z) * scale_factor;
        distance_to_target = 20.0;
        ROS_INFO("Adjusted target: (%.3f, %.3f, %.3f), new distance: %.3f", 
                 target_x, target_y, target_z, distance_to_target);
    }
    
    ROS_INFO("Generating straight line trajectory: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)", 
             current_x, current_y, current_z, target_x, target_y, target_z);
    
    // Update the reference trajectory for MPC
    // This will be used in the next solve() call
    for (unsigned int i = 0; i <= BLUEROV2_N; i++) {
        double progress = (double)i / BLUEROV2_N;
        
        // Use smooth interpolation (sigmoid-like function for smooth acceleration/deceleration)
        double smooth_progress = 1.0 / (1.0 + exp(-10.0 * (progress - 0.5)));
        
        // Interpolate position
        acados_in.yref[i][0] = current_x + (target_x - current_x) * smooth_progress;  // x
        acados_in.yref[i][1] = current_y + (target_y - current_y) * smooth_progress;  // y
        acados_in.yref[i][2] = current_z + (target_z - current_z) * smooth_progress;  // z
        
        // Interpolate orientation (handle angle wrapping)
        double psi_diff = target_psi - current_psi;
        // Normalize angle difference to [-pi, pi]
        while (psi_diff > M_PI) psi_diff -= 2 * M_PI;
        while (psi_diff < -M_PI) psi_diff += 2 * M_PI;
        
        acados_in.yref[i][3] = 0.0;  // phi (roll) - keep level
        acados_in.yref[i][4] = 0.0;  // theta (pitch) - keep level
        acados_in.yref[i][5] = current_psi + psi_diff * smooth_progress;  // psi (yaw)
        
        // Set velocities to zero for now (could be calculated based on trajectory)
        acados_in.yref[i][6] = 0.0;   // u
        acados_in.yref[i][7] = 0.0;   // v
        acados_in.yref[i][8] = 0.0;   // w
        acados_in.yref[i][9] = 0.0;   // p
        acados_in.yref[i][10] = 0.0;  // q
        acados_in.yref[i][11] = 0.0;  // r
        
        // Set disturbance estimates to zero (could be updated from UKF)
        acados_in.yref[i][12] = 0.0;  // disturbance x
        acados_in.yref[i][13] = 0.0;  // disturbance y
        acados_in.yref[i][14] = 0.0;  // disturbance z
        acados_in.yref[i][15] = 0.0;  // disturbance phi
        acados_in.yref[i][16] = 0.0;  // disturbance theta
        acados_in.yref[i][17] = 0.0;  // disturbance psi
    }
    
    ROS_INFO("Straight line trajectory generated and applied to MPC reference");
    ROS_INFO("Reference trajectory: start=(%.3f,%.3f,%.3f) -> end=(%.3f,%.3f,%.3f)", 
             acados_in.yref[0][0], acados_in.yref[0][1], acados_in.yref[0][2],
             acados_in.yref[BLUEROV2_N][0], acados_in.yref[BLUEROV2_N][1], acados_in.yref[BLUEROV2_N][2]);
}

// Sensor fusion functions
void BLUEROV2_DOB::sensor_fusion() {
    // Update sensor weights based on health
    update_sensor_weights();
    
    // Fuse sensor data in measurement vector
    // This is already handled in the UKF measurement model
}

void BLUEROV2_DOB::update_sensor_weights() {
    // Adjust weights based on sensor health
    if (!fault_detection.imu_healthy) {
        sensor_weights.imu_weight = 0.0;
        sensor_weights.pressure_weight += 0.1;
        sensor_weights.lidar_weight += 0.1;
    }
    
    if (!fault_detection.lidar_healthy) {
        sensor_weights.lidar_weight = 0.0;
        sensor_weights.pressure_weight += 0.1;
    }
    
    // Normalize weights
    double total_weight = sensor_weights.imu_weight + 
                         sensor_weights.pressure_weight + sensor_weights.gps_weight + 
                         sensor_weights.lidar_weight;
    
    if (total_weight > 0) {
        sensor_weights.imu_weight /= total_weight;
        sensor_weights.pressure_weight /= total_weight;
        sensor_weights.gps_weight /= total_weight;
        sensor_weights.lidar_weight /= total_weight;
    }
}

// Fault detection functions
bool BLUEROV2_DOB::check_sensor_health() {
    ros::Time current_time = ros::Time::now();
    
    // Check IMU timeout
    if ((current_time - fault_detection.last_imu_time).toSec() > 1.0) {
        fault_detection.imu_healthy = false;
        ROS_WARN("IMU sensor timeout");
    } else {
        fault_detection.imu_healthy = true;
    }
    
    // Check LiDAR timeout
    if ((current_time - fault_detection.last_lidar_time).toSec() > 1.0) {
        fault_detection.lidar_healthy = false;
        ROS_WARN("LiDAR sensor timeout");
    } else {
        fault_detection.lidar_healthy = true;
    }
    
    // Check pressure sensor timeout
    if ((current_time - fault_detection.last_pressure_time).toSec() > 1.0) {
        fault_detection.pressure_healthy = false;
        ROS_WARN("Pressure sensor timeout");
    } else {
        fault_detection.pressure_healthy = true;
    }
    
    return fault_detection.imu_healthy && fault_detection.lidar_healthy && fault_detection.pressure_healthy;
}

void BLUEROV2_DOB::update_sensor_timeouts() {
    ros::Time current_time = ros::Time::now();
    
    fault_detection.imu_timeout = (current_time - fault_detection.last_imu_time).toSec();
    fault_detection.pressure_timeout = (current_time - fault_detection.last_pressure_time).toSec();
    fault_detection.lidar_timeout = (current_time - fault_detection.last_lidar_time).toSec();
}

void BLUEROV2_DOB::handle_sensor_faults() {
    if (!fault_detection.imu_healthy) {
        ROS_ERROR("IMU fault detected - using backup estimation");
        // Use alternative estimation method
    }
    
    if (!fault_detection.lidar_healthy) {
        ROS_ERROR("LiDAR fault detected - using GPS for navigation");
        // Switch to GPS-based navigation
    }
}

// Keyboard control functions
void BLUEROV2_DOB::keyboard_cb(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received keyboard input: %s", msg->data.c_str());
    if (msg->data == "s" || msg->data == "S") {
        start_straight_line_navigation();
    } else if (msg->data == "e" || msg->data == "E") {
        stop_straight_line_navigation();
    }
}

void BLUEROV2_DOB::start_straight_line_navigation() {
    straight_line_mode = true;
    ROS_INFO("=== STARTING STRAIGHT LINE NAVIGATION ===");
    
    // Check if we have received pose data
    if (!is_start) {
        ROS_WARN("No pose data received yet, cannot start navigation");
        return;
    }
    
    ROS_INFO("Current position: x=%.2f, y=%.2f, z=%.2f", local_pos.x, local_pos.y, local_pos.z);
    ROS_INFO("Current orientation: phi=%.2f, theta=%.2f, psi=%.2f", 
             local_euler.phi, local_euler.theta, local_euler.psi);
    
    // Set navigation parameters
    target_depth = local_pos.z;  // Maintain current depth
    target_surge_velocity = 0.8; // 0.8 m/s forward velocity (moderate speed)
    
    // Clear previous trajectory for fresh start
    trajectory_points.clear();
    
    ROS_INFO("Navigation parameters set:");
    ROS_INFO("- Target depth: %.2f m", target_depth);
    ROS_INFO("- Target velocity: %.2f m/s", target_surge_velocity);
    ROS_INFO("- Forward LiDAR enabled for obstacle detection");
    ROS_INFO("- Trajectory visualization enabled");
    
    ROS_INFO("Press 'e' to stop navigation");
    ROS_INFO("=== STRAIGHT LINE NAVIGATION STARTED ===");
}

void BLUEROV2_DOB::stop_straight_line_navigation() {
    straight_line_mode = false;
    ROS_INFO("=== STOPPING STRAIGHT LINE NAVIGATION ===");
    
    // Calculate final trajectory statistics
    if (trajectory_points.size() > 10) {
        calculate_trajectory_straightness();
        
        // Calculate total distance traveled
        double total_distance = 0.0;
        for (size_t i = 1; i < trajectory_points.size(); i++) {
            auto prev = trajectory_points[i-1];
            auto curr = trajectory_points[i];
            total_distance += sqrt(pow(curr.pose.position.x - prev.pose.position.x, 2) +
                                 pow(curr.pose.position.y - prev.pose.position.y, 2) +
                                 pow(curr.pose.position.z - prev.pose.position.z, 2));
        }
        
        ROS_INFO("Navigation Summary:");
        ROS_INFO("- Total distance traveled: %.2f m", total_distance);
        ROS_INFO("- Trajectory points recorded: %zu", trajectory_points.size());
        ROS_INFO("- Final position: (%.2f, %.2f, %.2f)", 
                 local_pos.x, local_pos.y, local_pos.z);
    }
    
    // Reset control inputs
    for (int i = 0; i < BLUEROV2_NU; i++) {
        acados_out.u0[i] = 0.0;
    }
    
    ROS_INFO("=== STRAIGHT LINE NAVIGATION STOPPED ===");
}

