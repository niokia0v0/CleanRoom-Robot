#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock
import math
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0
        self.file_path = 'waypoints.txt'
        self.waypoints = self.load_waypoints()

        self.tolerance = 0.1
        self.angular_tolerance = 0.1
        self.linear_speed = 0.15
        self.angular_speed_gain = 1.5

        self.obstacle_detected = False
        self.obstacle_threshold = 0.4  # m
        self.obstacle_detection_enabled = True

        # Action clients
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.dock_client = ActionClient(self, Dock, '/dock')

    def load_waypoints(self):
        waypoints = []
        with open(self.file_path, 'r') as f:
            for line in f:
                if line.strip():
                    x, y = map(float, line.strip().split(','))
                    waypoints.append((x, y))
        self.get_logger().info(f"{len(waypoints)} waypoints chargés depuis {self.file_path}")
        return waypoints

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        if not self.obstacle_detection_enabled:
            return

        ranges = msg.ranges
        nb = len(ranges)
        region_size = nb // 8
        region_0 = ranges[0:region_size]               # Devant gauche
        region_7 = ranges[-region_size:]               # Devant droite

        def is_obstacle(region):
            return any(r < self.obstacle_threshold and r > msg.range_min for r in region)

        left = is_obstacle(region_0)
        right = is_obstacle(region_7)

        if left or right:
            self.obstacle_detected = True
            if left and not right:
                self.get_logger().warn("Obstacle détecté devant à GAUCHE (zone 0)")
            elif right and not left:
                self.get_logger().warn("Obstacle détecté devant à DROITE (zone 7)")
            else:
                self.get_logger().warn("Obstacle détecté devant au CENTRE (zones 0 et 7)")
        else:
            self.obstacle_detected = False

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_waypoints(self):
        twist = Twist()
        for idx, (goal_x, goal_y) in enumerate(self.waypoints):
            self.get_logger().info(f"Déplacement vers le point {idx+1}: x={goal_x:.2f}, y={goal_y:.2f}")
            while rclpy.ok():
                if self.obstacle_detected:
                    self.get_logger().warn("Obstacle détecté, arrêt du robot pendant 5 secondes.")
                    # Stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)

                    # Désactiver la détection pendant la pause
                    self.obstacle_detection_enabled = False
                    start_time = time.time()
                    obstacle_still_there = True

                    while time.time() - start_time < 5.0:
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if not self.obstacle_detected:
                            obstacle_still_there = False
                            self.get_logger().info("Obstacle disparu pendant l’attente.")
                            break

                    # Réactiver la détection
                    self.obstacle_detection_enabled = True

                    if not obstacle_still_there:
                        continue

                    # Manœuvre d’évitement
                    self.get_logger().warn("Obstacle toujours présent. Exécution de l’évitement.")
                    twist.angular.z = 0.6
                    twist.linear.x = 0.0
                    self.publisher_.publish(twist)
                    time.sleep(2.0)
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    time.sleep(0.5)
                    continue

                # Navigation normale
                x, y = self.current_position
                dx = goal_x - x
                dy = goal_y - y
                distance = math.hypot(dx, dy)

                if distance < self.tolerance:
                    self.get_logger().info(f"Point {idx+1} atteint")
                    break

                target_angle = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(target_angle - self.current_orientation)

                if abs(angle_diff) > self.angular_tolerance:
                    twist.angular.z = self.angular_speed_gain * angle_diff
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed_gain * angle_diff

                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)

            # Stop au point
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1)

    def send_undock(self):
        self.get_logger().info("Envoi de la commande UNDOCK...")
        self.obstacle_detection_enabled = False
        while not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Attente du serveur d’action /undock...')
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("UNDOCK terminé.")
        else:
            self.get_logger().warn("La commande UNDOCK a été refusée.")
        self.obstacle_detection_enabled = True

    def send_dock(self):
        self.get_logger().info("Envoi de la commande DOCK...")
        self.obstacle_detection_enabled = False
        while not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Attente du serveur d’action /dock...')
        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("DOCK terminé.")
        else:
            self.get_logger().warn("La commande DOCK a été refusée.")
        self.obstacle_detection_enabled = True

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    navigator.send_undock()
    navigator.move_to_waypoints()
    navigator.send_dock()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

