import rclpy
from rclpy.node import Node

class ArmMotor(Node):
	def __init__(self):
		super().__init__('arm_motor')
		self.get_logger().info('Arm Motor Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = ArmMotor()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
