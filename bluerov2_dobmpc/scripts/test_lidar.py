#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class LiDARTest:
    def __init__(self):
        rospy.init_node('lidar_test', anonymous=True)
        
        # Subscribe to LiDAR data
        self.lidar_sub = rospy.Subscriber('/bluerov2/forward_lidar_forward', LaserScan, self.lidar_callback)
        
        # TF listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Statistics
        self.scan_count = 0
        self.last_scan_time = None
        
        rospy.loginfo("LiDAR Test Node Started")
        rospy.loginfo("Listening to topic: /bluerov2/forward_lidar_forward")
        
    def lidar_callback(self, scan):
        """Callback function for LiDAR data"""
        self.scan_count += 1
        current_time = rospy.Time.now()
        
        if self.last_scan_time is not None:
            dt = (current_time - self.last_scan_time).to_sec()
            rate = 1.0 / dt if dt > 0 else 0
            rospy.loginfo(f"LiDAR Scan #{self.scan_count}: Rate={rate:.2f} Hz, Range count={len(scan.ranges)}")
        else:
            rospy.loginfo(f"LiDAR Scan #{self.scan_count}: First scan received, Range count={len(scan.ranges)}")
        
        # Log scan information
        if len(scan.ranges) > 0:
            min_range = min(scan.ranges)
            max_range = max(scan.ranges)
            avg_range = np.mean(scan.ranges)
            
            rospy.loginfo(f"  Range: min={min_range:.3f}m, max={max_range:.3f}m, avg={avg_range:.3f}m")
            rospy.loginfo(f"  Angle: min={np.degrees(scan.angle_min):.1f}°, max={np.degrees(scan.angle_max):.1f}°")
            rospy.loginfo(f"  Time: {scan.header.stamp}")
            rospy.loginfo(f"  Frame: {scan.header.frame_id}")
        
        self.last_scan_time = current_time
        
        # Log every 10th scan to avoid spam
        if self.scan_count % 10 == 0:
            rospy.loginfo(f"Processed {self.scan_count} LiDAR scans")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            if self.scan_count == 0:
                rospy.logwarn("No LiDAR data received yet...")
            else:
                rospy.loginfo(f"LiDAR is working! Total scans: {self.scan_count}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        test_node = LiDARTest()
        test_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("LiDAR Test Node interrupted")
    except Exception as e:
        rospy.logerr(f"LiDAR Test Node error: {e}")
