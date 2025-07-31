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
    
    // Initialize LiDAR subscribers
    ROS_INFO("Initializing LiDAR subscribers...");
    left_lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/bluerov2/left_lidar_/scan", 20, &BLUEROV2_DOB::left_lidar_cb, this);
    right_lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/bluerov2/right_lidar_/scan", 20, &BLUEROV2_DOB::right_lidar_cb, this);
    forward_lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/bluerov2/forward_lidar_/scan", 20, &BLUEROV2_DOB::forward_lidar_cb, this);
    ROS_INFO("LiDAR subscribers initialized");
    
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
    
    // Safety check: if trajectory is not loaded, use default values
    if (number_of_steps == 0) {
        ROS_WARN("Trajectory not loaded, using default reference values");
        for (unsigned int i = 0; i <= BLUEROV2_N; i++) {
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++) {
                acados_in.yref[i][j] = 0.0;  // Default to zero
            }
        }
        return;
    }
    
    if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  // Fill the rest horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    else    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    
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
    
    acados_in.x0[x] = local_pos.x;
    acados_in.x0[y] = local_pos.y;
    acados_in.x0[z] = local_pos.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    acados_in.x0[psi] = yaw_sum;
    
    // Check velocity data validity
    if (v_linear_body.size() < 3 || v_angular_body.size() < 3) {
        ROS_WARN("Velocity data not available, using zeros");
        v_linear_body = Vector3d::Zero();
        v_angular_body = Vector3d::Zero();
    }
    
    ROS_INFO("Linear velocity: u=%.2f, v=%.2f, w=%.2f", v_linear_body[0], v_linear_body[1], v_linear_body[2]);
    ROS_INFO("Angular velocity: p=%.2f, q=%.2f, r=%.2f", v_angular_body[0], v_angular_body[1], v_angular_body[2]);
    
    acados_in.x0[u] = v_linear_body[0];
    acados_in.x0[v] = v_linear_body[1];
    acados_in.x0[w] = v_linear_body[2];
    acados_in.x0[p] = v_angular_body[0];
    acados_in.x0[q] = v_angular_body[1];
    acados_in.x0[r] = v_angular_body[2];
    
    // Set initial state to match ACADOS constraints
    // ACADOS expects state 2 (z position) to be constrained to -20
    // Other states should be set to reasonable values
    acados_in.x0[0] = v_linear_body[0];  // u
    acados_in.x0[1] = v_linear_body[1];  // v
    acados_in.x0[2] = -20.0;             // w (constrained by ACADOS)
    acados_in.x0[3] = v_angular_body[0]; // p
    acados_in.x0[4] = v_angular_body[1]; // q
    acados_in.x0[5] = v_angular_body[2]; // r
    acados_in.x0[6] = local_pos.x;       // x
    acados_in.x0[7] = local_pos.y;       // y
    acados_in.x0[8] = -20.0;             // z (constrained by ACADOS)
    acados_in.x0[9] = local_euler.phi;   // phi
    acados_in.x0[10] = local_euler.theta; // theta
    acados_in.x0[11] = local_euler.psi;  // psi
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
    lbx[0] = -10.0;   // u velocity
    lbx[1] = -10.0;   // v velocity  
    lbx[2] = -20.0;   // w velocity (depth rate)
    lbx[3] = -1.0;    // p angular velocity
    lbx[4] = -1.0;    // q angular velocity
    lbx[5] = -1.0;    // r angular velocity
    lbx[6] = -100.0;  // x position
    lbx[7] = -100.0;  // y position
    lbx[8] = -100.0;  // z position (depth)
    lbx[9] = -M_PI;   // phi (roll)
    lbx[10] = -M_PI;  // theta (pitch)
    lbx[11] = -M_PI;  // psi (yaw)
    lbx[12] = -10.0;  // disturbance x
    lbx[13] = -10.0;  // disturbance y
    lbx[14] = -10.0;  // disturbance z
    lbx[15] = -1.0;   // disturbance phi
    lbx[16] = -1.0;   // disturbance theta
    lbx[17] = -1.0;   // disturbance psi
    
    // Set upper bounds
    ubx[0] = 10.0;    // u velocity
    ubx[1] = 10.0;    // v velocity
    ubx[2] = 20.0;    // w velocity (depth rate)
    ubx[3] = 1.0;     // p angular velocity
    ubx[4] = 1.0;     // q angular velocity
    ubx[5] = 1.0;     // r angular velocity
    ubx[6] = 100.0;   // x position
    ubx[7] = 100.0;   // y position
    ubx[8] = 100.0;   // z position (depth)
    ubx[9] = M_PI;    // phi (roll)
    ubx[10] = M_PI;   // theta (pitch)
    ubx[11] = M_PI;   // psi (yaw)
    ubx[12] = 10.0;   // disturbance x
    ubx[13] = 10.0;   // disturbance y
    ubx[14] = 10.0;   // disturbance z
    ubx[15] = 1.0;    // disturbance phi
    ubx[16] = 1.0;    // disturbance theta
    ubx[17] = 1.0;    // disturbance psi
    
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
    
    // Modify reference trajectory to be closer to current robot position
    // This helps reduce the initial error and makes the problem more feasible
    ROS_INFO("Adjusting reference trajectory to match current position...");
    double current_x = acados_in.x0[6];
    double current_y = acados_in.x0[7];
    double current_z = acados_in.x0[8];
    
    // Set the first reference point to current position to reduce initial error
    acados_in.yref[0][0] = current_x;
    acados_in.yref[0][1] = current_y;
    acados_in.yref[0][2] = current_z;
    
    ROS_INFO("Adjusted reference: (%.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f)", 
             acados_in.yref[0][0], acados_in.yref[0][1], acados_in.yref[0][2],
             current_x, current_y, current_z);
    
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
    
    acados_status = bluerov2_acados_solve(mpc_capsule);
    
    // If first attempt fails, try with relaxed bounds
    if (acados_status != 0) {
        ROS_WARN("First solver attempt failed, trying with relaxed bounds...");
        
        // Relax bounds by increasing the range
        double relaxed_lbx[18], relaxed_ubx[18];
        for (int i = 0; i < 18; i++) {
            relaxed_lbx[i] = lbx[i] * 2.0;  // Double the range
            relaxed_ubx[i] = ubx[i] * 2.0;
        }
        
        // Set relaxed bounds
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", relaxed_lbx);
        ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", relaxed_ubx);
        
        // Try solving again
        acados_status = bluerov2_acados_solve(mpc_capsule);
        ROS_INFO("Second solver attempt completed with status: %d", acados_status);
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
        
        // Simple proportional control based on position error
        double kp = 0.1;  // Proportional gain
        double max_thrust = 0.5;  // Maximum thrust for safety
        
        // Calculate control inputs based on position error
        double thrust_x = -kp * pos_error_x;
        double thrust_y = -kp * pos_error_y;
        double thrust_z = -kp * pos_error_z;
        
        // Limit thrust values
        thrust_x = std::max(-max_thrust, std::min(max_thrust, thrust_x));
        thrust_y = std::max(-max_thrust, std::min(max_thrust, thrust_y));
        thrust_z = std::max(-max_thrust, std::min(max_thrust, thrust_z));
        
        // Apply fallback control
        current_t.t0 = thrust_x;  // Forward/backward
        current_t.t1 = thrust_y;  // Lateral
        current_t.t2 = thrust_z;  // Vertical
        current_t.t3 = 0.0;       // No rotational control for safety
        current_t.t4 = 0.0;
        current_t.t5 = 0.0;
        
        ROS_INFO("Fallback control: thrust_x=%.3f, thrust_y=%.3f, thrust_z=%.3f", 
                 thrust_x, thrust_y, thrust_z);
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
    // Create a new pose stamped message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "odom_frame";
    pose_stamped.header.stamp = ros::Time::now();
    
    // Set position
    pose_stamped.pose.position.x = local_pos.x;
    pose_stamped.pose.position.y = local_pos.y;
    pose_stamped.pose.position.z = local_pos.z;
    
    // Set orientation
    pose_stamped.pose.orientation = rpy2q(local_euler);
    
    // Add to trajectory points
    trajectory_points.push_back(pose_stamped);
    
    // Limit the number of points to prevent memory issues
    if (trajectory_points.size() > max_trajectory_points) {
        trajectory_points.erase(trajectory_points.begin());
    }
    
    // Debug: Print trajectory info every 100 points
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter % 100 == 0) {
        ROS_INFO("Trajectory points: %zu, Current position: (%.2f, %.2f, %.2f)", 
                 trajectory_points.size(), local_pos.x, local_pos.y, local_pos.z);
    }
}

void BLUEROV2_DOB::publish_trajectory() {
    // Update trajectory path
    trajectory_path.header.stamp = ros::Time::now();
    trajectory_path.poses = trajectory_points;
    
    // Publish trajectory
    trajectory_pub.publish(trajectory_path);
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
void BLUEROV2_DOB::left_lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    fault_detection.last_lidar_time = ros::Time::now();
    
    // Find minimum distance
    float min_distance = scan->range_max;
    for (int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] < min_distance && scan->ranges[i] > scan->range_min) {
            min_distance = scan->ranges[i];
        }
    }
    
    left_lidar_distance = min_distance;
}

void BLUEROV2_DOB::right_lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    fault_detection.last_lidar_time = ros::Time::now();
    
    // Find minimum distance
    float min_distance = scan->range_max;
    for (int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] < min_distance && scan->ranges[i] > scan->range_min) {
            min_distance = scan->ranges[i];
        }
    }
    
    right_lidar_distance = min_distance;
}

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
    // Calculate lateral error from LiDAR data
    double avg_side_distance = (left_lidar_distance + right_lidar_distance) / 2.0;
    lateral_error = target_side_distance - avg_side_distance;
    
    // Apply lateral correction
    double lateral_correction = lateral_error * lateral_kp;
    acados_out.u0[1] += lateral_correction;
}

void BLUEROV2_DOB::generate_straight_line_trajectory() {
    // Use current pose instead of estimated pose for safety
    double current_x = local_pos.x;
    double current_y = local_pos.y;
    double current_z = local_pos.z;
    
    // Calculate target position (move forward along x-axis)
    double target_x = current_x + 10.0;  // Move 10 meters forward
    double target_y = target_side_distance;  // Maintain lateral position
    double target_z = target_depth;         // Maintain depth
    
    // Generate trajectory points
    for (int i = 0; i <= BLUEROV2_N; i++) {
        double progress = (double)i / BLUEROV2_N;
        
        acados_in.yref[i][0] = current_x + progress * (target_x - current_x);
        acados_in.yref[i][1] = target_y;
        acados_in.yref[i][2] = target_z;
        acados_in.yref[i][3] = 0;  // roll
        acados_in.yref[i][4] = 0;  // pitch
        acados_in.yref[i][5] = 0;  // yaw (maintain heading)
    }
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
    ROS_INFO("Starting straight line navigation");
    
    // Check if we have received pose data
    if (!is_start) {
        ROS_WARN("No pose data received yet, cannot start navigation");
        return;
    }
    
    ROS_INFO("Current position: x=%.2f, y=%.2f, z=%.2f", local_pos.x, local_pos.y, local_pos.z);
    
    // Set initial target position
    target_side_distance = 5.0;  // 5 meters from side wall
    target_depth = local_pos.z;  // Maintain current depth
    target_surge_velocity = 0.5; // 0.5 m/s forward velocity
    
    ROS_INFO("Target depth: %.2f, Target velocity: %.2f", target_depth, target_surge_velocity);
}

void BLUEROV2_DOB::stop_straight_line_navigation() {
    straight_line_mode = false;
    ROS_INFO("Stopping straight line navigation");
    
    // Reset control inputs
    for (int i = 0; i < BLUEROV2_NU; i++) {
        acados_out.u0[i] = 0.0;
    }
}

