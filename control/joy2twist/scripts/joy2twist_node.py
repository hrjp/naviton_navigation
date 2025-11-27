#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class JoyToTwist:
    def __init__(self):
        rospy.init_node('joy2twist')
        # Parameters: axis mapping and scale
        self.linear_axis = rospy.get_param('~linear_axis', 1)   # typically left stick vertical
        self.angular_axis = rospy.get_param('~angular_axis', 0)  # typically left stick horizontal
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)
        self.auto_button = rospy.get_param('~auto_button', 0)
        self.manual_button = rospy.get_param('~manual_button', 1)
        self.start_button = rospy.get_param('~start_button', 2)
        self.deadband = rospy.get_param('~deadband', 0.05)
        self.invert_linear = rospy.get_param('~invert_linear', True)
        self.invert_angular = rospy.get_param('~invert_angular', False)
        self.publish_zero_on_button = rospy.get_param('~publish_zero_on_button', -1)  # button index; -1 disabled

        # start service proxy (default name /naviton/core/start)
        self.start_srv_name = rospy.get_param('~start_service', '/naviton/core/start')
        try:
            rospy.wait_for_service(self.start_srv_name, timeout=2.0)
            self.start_srv = rospy.ServiceProxy(self.start_srv_name, Empty)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn("Start service '%s' not available at init; will try at runtime", self.start_srv_name)
            self.start_srv = None
        self.start_prev = False

        self.cmd_pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_cb)
        rospy.Subscriber('cmd_vel_in', Twist, self.cmdvel_cb)
        self.mode = 'auto'  # default mode

    def apply_deadband(self, v):
        return 0.0 if abs(v) < self.deadband else v

    def joy_cb(self, msg: Joy):
        twist = Twist()

        # detect rising edge of start button and call service once per press
        if 0 <= self.start_button < len(msg.buttons):
            current_start = bool(msg.buttons[self.start_button])
            if current_start and not self.start_prev:
                # ensure proxy exists, try to recreate if it was None
                if self.start_srv is None:
                    try:
                        rospy.wait_for_service(self.start_srv_name, timeout=2.0)
                        self.start_srv = rospy.ServiceProxy(self.start_srv_name, Empty)
                    except (rospy.ROSException, rospy.ROSInterruptException):
                        rospy.logwarn("Start service '%s' not available", self.start_srv_name)
                        self.start_srv = None
                if self.start_srv:
                    try:
                        resp = self.start_srv()
                        rospy.loginfo("Called start service '%s' success=%s message='%s'", self.start_srv_name, resp.success, getattr(resp, 'message', ''))
                    except rospy.ServiceException as e:
                        rospy.logwarn("Start service call failed: %s", e)
                else:
                    rospy.logwarn("Start service proxy not available: %s", self.start_srv_name)
            self.start_prev = current_start

        # Safety: if button index configured and pressed, force zero output
        if 0 <= self.publish_zero_on_button < len(msg.buttons) and msg.buttons[self.publish_zero_on_button]:
            self.cmd_pub.publish(twist)
            return
        
        # Mode switching
        if 0 <= self.auto_button < len(msg.buttons) and msg.buttons[self.auto_button]:
            self.mode = 'auto'
        elif 0 <= self.manual_button < len(msg.buttons) and msg.buttons[self.manual_button]:
            self.mode = 'manual'
        
        if self.mode == 'auto':
            return  # In auto mode, ignore joystick inputs

        if self.linear_axis < len(msg.axes):
            lin = msg.axes[self.linear_axis]
            if self.invert_linear:
                lin = -lin
            lin = self.apply_deadband(lin) * self.linear_scale
            twist.linear.x = lin
        if self.angular_axis < len(msg.axes):
            ang = msg.axes[self.angular_axis]
            if self.invert_angular:
                ang = -ang
            ang = self.apply_deadband(ang) * self.angular_scale
            twist.angular.z = ang
        self.cmd_pub.publish(twist)
    
    def cmdvel_cb(self, msg: Twist):
        if self.mode == 'auto':
            self.cmd_pub.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = JoyToTwist()
    node.spin()
