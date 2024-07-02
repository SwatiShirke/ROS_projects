import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32MultiArray



def angle_to_config(q_angles):
	## initialize link variables
	l1=1
	l2=1
	l3=1
	## initialize angle variables with converion of q angles to radians from degree
	q1=math.radians(q_angles[0])
	q2=math.radians(q_angles[1])
	q3=math.radians(q_angles[2])
	
	## calculate robot's position vector using forward kinematics
	x = (l3 * math.cos(q1)* math.cos(q2) * math.cos(q3)) - (l3 * math.cos(q1) * math.sin(q2) * math.sin(q3))
	y = (l3 * math.sin(q1)* math.cos(q3)) - (l2* math.sin(q1)* math.sin(q2) * math.sin(q3))
	z = -(l3 * math.sin(q2)* math.cos(q3)) - (l3* math.cos(q2) * math.sin(q3)) - (l2* math.sin(q2)) + l1
	
	## calculate rotation matrix components using forward kinematics
	r11 = (math.cos(q1)* math.cos(q2) * math.cos(q3) )- (math.cos(q1)* math.sin(q2) * math.sin(q3))
	r12 = -(math.cos(q1)* math.cos(q2) * math.sin(q3) ) - (math.cos(q1)* math.sin(q2) * math.cos(q3))
	r13 = -math.sin(q1)
	
	r21 = (math.cos(q3) * math.sin(q1)) - (math.sin(q1) * math.sin(q2) * math.sin(q3))
	r22 = -(math.sin(q1) * math.sin(q3)) - (math.sin(q1) * math.sin(q2) * math.cos(q3))
	r23 = math.cos(q1)
	
	r31 = -(math.sin(q2) * math.cos(q3)) - (math.cos(q2) * math.sin(q3))
	r32 = (math.sin(q2) * math.sin(q3)) - (math.cos(q2) * math.cos(q3))
	r33 = 0
	
	
	return [r11,r12,r13, r21,r22,r23, r31,r32,r33, x, y,z]


class my_node(Node):

    def __init__(self):
        super().__init__('my_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angle_to_config',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        configurations = angle_to_config(msg.data)
        self.get_logger().info('Robot position vector is: "%s"' % configurations[9:12])
        self.get_logger().info('Robot rotation matrix is: "%s"'% configurations[0:9])
	
	
def main(args=None):
    rclpy.init(args=args)
    my_node_subscriber = my_node()
    rclpy.spin(my_node_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
