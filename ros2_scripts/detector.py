#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

class DetectorNode(Node):
	def __init__(self):
    	super().__init__('detector_node')

    	self.NBR_REGION = 8
    	self.OBSTACLE_THRESHOLD = 0.3
    	self.MAX_DISTANCE = 10.0

    	self.publisher_ = self.create_publisher(Int8, '/detector', 10)
    	self.subscription = self.create_subscription(
        	LaserScan,
        	'/scan',
        	self.scan_callback,
        	10
    	)

    	self.get_logger().info("✅ Node de détection d'obstacles initialisé.")

	def scan_callback(self, msg: LaserScan):
    	ranges = list(msg.ranges)
    	total_samples = len(ranges)
    	samples_per_region = total_samples // self.NBR_REGION

    	min_distances = []

    	for i in range(self.NBR_REGION):
        	start_index = i * samples_per_region
        	end_index = min((i + 1) * samples_per_region, total_samples)
        	region_data = ranges[start_index:end_index]
        	valid_distances = [d for d in region_data if 0.15 <= d <= self.MAX_DISTANCE]
        	min_distance = min(valid_distances) if valid_distances else float('inf')
        	min_distances.append(min_distance)

    	region_with_obstacle = -1
    	min_distance_global = float('inf')

    	for i, dist in enumerate(min_distances):
        	if dist < self.OBSTACLE_THRESHOLD and dist < min_distance_global:
            	min_distance_global = dist
            	region_with_obstacle = i

    	msg_out = Int8()
    	msg_out.data = region_with_obstacle
    	self.publisher_.publish(msg_out)

    	for i, dist in enumerate(min_distances):
        	if dist < self.OBSTACLE_THRESHOLD:
            	self.get_logger().warn(f"🚧 Obstacle détecté dans la région {i} : {dist:.2f} m")
        	elif dist < float('inf'):
            	self.get_logger().info(f"Région {i} : Distance minimale = {dist:.2f} m")

def main(args=None):
	rclpy.init(args=args)
	node = DetectorNode()
	try:
    	rclpy.spin(node)
	except KeyboardInterrupt:
    	pass
	finally:
    	node.destroy_node()
    	rclpy.shutdown()

if __name__ == '__main__':
	main()
