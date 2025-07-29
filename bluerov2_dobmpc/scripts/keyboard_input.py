#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
from std_msgs.msg import String

def get_key():
    """Get a single keypress from the user"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_input_node():
    """Keyboard input node for BlueROV2 control"""
    rospy.init_node('keyboard_input', anonymous=True)
    pub = rospy.Publisher('/keyboard_input', String, queue_size=10)
    
    print("Keyboard Control for BlueROV2:")
    print("Press 's' to start straight line navigation")
    print("Press 'e' to stop straight line navigation")
    print("Press 'q' to quit")
    print("Waiting for input...")
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        try:
            key = get_key()
            
            if key == 's' or key == 'S':
                print("Starting straight line navigation...")
                pub.publish(String("s"))
            elif key == 'e' or key == 'E':
                print("Stopping straight line navigation...")
                pub.publish(String("e"))
            elif key == 'q' or key == 'Q':
                print("Quitting...")
                break
            elif key == '\x03':  # Ctrl+C
                break
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            rospy.logerr(f"Error reading keyboard input: {e}")
            
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_input_node()
    except rospy.ROSInterruptException:
        pass 