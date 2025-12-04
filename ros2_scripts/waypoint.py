#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import termios
import tty
import sys
import select
import os

class WaypointRecorder(Node):
	def __init__(self):
    	super().__init__('waypoint_recorder')
    	self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    	self.points = []
    	self.current_position = (0.0, 0.0)
    	self.file_path = os.path.join(os.path.expanduser("~"), 'waypoints.txt')
    	self.get_logger().info('📍 Appuie sur "s" pour enregistrer un point, "q" pour quitter et sauvegarder.')

	def odom_callback(self, msg):
    	self.current_position = (
        	msg.pose.pose.position.x,
        	msg.pose.pose.position.y
    	)

	def run(self):
    	try:
        	while rclpy.ok():
            	if self.kbhit():
                	key = sys.stdin.read(1)
                	if key == 's':
                    	x, y = self.current_position
                    	self.points.append((x, y))
                    	self.get_logger().info(f'✅ Point enregistré : x={x:.2f}, y={y:.2f}')
                	elif key == 'q':
                    	self.get_logger().info('💾 Fin de l\'enregistrement.')
                    	break
            	rclpy.spin_once(self, timeout_sec=0.1)
    	except KeyboardInterrupt:
        	pass
    	finally:
        	self.save_points()
        	self.destroy_node()
        	rclpy.shutdown()

	def kbhit(self):
    	dr, _, _ = select.select([sys.stdin], [], [], 0)
    	return dr != []

	def save_points(self):
    	with open(self.file_path, 'w') as f:
        	for x, y in self.points:
            	f.write(f'{x},{y}\n')
    	self.get_logger().info(f'📝 {len(self.points)} point(s) sauvegardé(s) dans {self.file_path}')

def main(args=None):
	settings = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())

	rclpy.init(args=args)
	node = WaypointRecorder()
	node.run()

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
	main()
