import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String as StringMsg
import math
import os
import sys

# Assumes this utility file exists in the same package
from husky_nav_ros2.robot_math_utils import get_yaw_from_quaternion, normalize_angle_radians, distance_2d

WAYPOINTS_FILENAME = "waypoints.csv"

class WaypointNavigatorPD(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_pd_node')
        self.get_logger().info('PD Waypoint Navigator initializing...')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/a200_1046/cmd_vel', 10)
        self.status_publisher = self.create_publisher(StringMsg, '/navigation_status', 10)

        # Subscriber
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',  # Using ZED camera for localized pose
            self.pose_callback,
            10
        )
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0
        self.pose_is_initialized = False

        # Waypoint data
        self.target_waypoints = []
        self.current_waypoint_index = -1

        # --- PD Controller & Navigation Parameters  ---
        self.goal_xy_tolerance = 0.1         # meters
        self.goal_final_theta_tolerance = 0.1  # radians
        self.approach_heading_tolerance = 0.2  

        self.max_linear_speed = 0.6     # m/s (Husky max is ~1.0 m/s)
        self.max_angular_speed = 0.4      

        # Proportional Gains
        self.linear_Kp = 0.5
        self.angular_Kp = 0.8

        # Derivative Gains
        self.linear_Kd = 0.1
        self.angular_Kd = 0.15

        # Variables for PD controller
        self.previous_distance_error = 0.0
        self.previous_heading_error_to_point = 0.0
        self.previous_final_heading_error = 0.0
        # Initialize with a valid time. It will be updated on first pose callback.
        self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9

        # Navigation States
        self.NAV_STATE_IDLE = "IDLE"
        self.NAV_STATE_LOADING_WAYPOINTS = "LOADING_WAYPOINTS"
        self.NAV_STATE_ALIGNING_TO_POINT = "ALIGNING_TO_POINT"
        self.NAV_STATE_MOVING_TO_POINT = "MOVING_TO_POINT"
        self.NAV_STATE_REACHED_POINT_XY = "REACHED_POINT_XY"
        self.NAV_STATE_ALIGNING_FINAL_ORIENTATION = "ALIGNING_FINAL_ORIENTATION"
        self.NAV_STATE_WAYPOINT_COMPLETE = "WAYPOINT_COMPLETE"
        self.NAV_STATE_ALL_WAYPOINTS_DONE = "ALL_WAYPOINTS_DONE"
        self.current_nav_state = self.NAV_STATE_LOADING_WAYPOINTS

        self.load_waypoints_from_file()

        if self.target_waypoints:
            self.current_waypoint_index = 0
            self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
            self.log_and_publish_status(f"Loaded {len(self.target_waypoints)} waypoints. Targeting waypoint 0.")
        else:
            self.current_nav_state = self.NAV_STATE_IDLE
            self.log_and_publish_status("No waypoints loaded. Navigator is idle.")

        self.control_loop_timer = self.create_timer(0.05, self.navigation_control_loop) # 20 Hz loop
        self.get_logger().info('Node initialized. Control loop started.')

    def pose_callback(self, msg: PoseStamped):
        self.current_robot_x = msg.pose.position.x
        self.current_robot_y = msg.pose.position.y
        raw_yaw = get_yaw_from_quaternion(msg.pose.orientation)
        self.current_robot_yaw = normalize_angle_radians(raw_yaw)

        if not self.pose_is_initialized:
            self.pose_is_initialized = True
            # Set the time here to get a valid first 'dt' in the control loop
            self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Initial robot pose: x={self.current_robot_x:.2f}, y={self.current_robot_y:.2f}, yaw={self.current_robot_yaw:.2f}")

    def load_waypoints_from_file(self):
        self.target_waypoints = []
        full_path = os.path.abspath(WAYPOINTS_FILENAME)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"Waypoint file not found: {full_path}")
            return
        try:
            with open(WAYPOINTS_FILENAME, 'r') as f:
                for line_number, line in enumerate(f):
                    line = line.strip()
                    if not line or line.startswith("#"): continue
                    try:
                        parts = line.split(',')
                        x = float(parts[0])
                        y = float(parts[1])
                        theta = normalize_angle_radians(float(parts[2]))
                        self.target_waypoints.append({'x': x, 'y': y, 'theta': theta})
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(f"Skipping malformed line {line_number+1} in {WAYPOINTS_FILENAME}: '{line}'. Error: {e}")
            self.get_logger().info(f"Loaded {len(self.target_waypoints)} waypoints from {full_path}")
        except IOError as e:
            self.get_logger().error(f"Could not read waypoints from file {full_path}: {e}")

    def navigation_control_loop(self):
        if not self.pose_is_initialized:
            self.log_and_publish_status("Waiting for initial robot pose...")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time_error_calculated
        if dt <= 0:
            # Handle case where dt is zero or negative to prevent errors
            return
        self.last_time_error_calculated = current_time

        if self.current_nav_state in [self.NAV_STATE_IDLE, self.NAV_STATE_ALL_WAYPOINTS_DONE]:
            self.stop_robot()
            return
        if self.current_waypoint_index < 0 or self.current_waypoint_index >= len(self.target_waypoints):
            self.log_and_publish_status("Error: Invalid waypoint index. Setting to IDLE.")
            self.current_nav_state = self.NAV_STATE_IDLE
            self.stop_robot()
            return

        current_goal = self.target_waypoints[self.current_waypoint_index]
        goal_x, goal_y, goal_theta = current_goal['x'], current_goal['y'], current_goal['theta']

        distance_to_goal_xy = distance_2d(self.current_robot_x, self.current_robot_y, goal_x, goal_y)
        angle_to_goal_point = math.atan2(goal_y - self.current_robot_y, goal_x - self.current_robot_x)
        heading_error_to_point = normalize_angle_radians(angle_to_goal_point - self.current_robot_yaw)
        final_heading_error = normalize_angle_radians(goal_theta - self.current_robot_yaw)

        twist_cmd = Twist()

        if self.current_nav_state == self.NAV_STATE_ALIGNING_TO_POINT:
            if abs(heading_error_to_point) > self.approach_heading_tolerance:
                self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Aligning to point. Err: {heading_error_to_point:.2f} rad")
                angular_error_derivative = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                angular_velocity = (self.angular_Kp * heading_error_to_point) + (self.angular_Kd * angular_error_derivative)
                twist_cmd.angular.z = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.current_nav_state = self.NAV_STATE_MOVING_TO_POINT
                self.previous_distance_error = distance_to_goal_xy # Reset for moving state
                self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Alignment complete.")
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_MOVING_TO_POINT:
            if distance_to_goal_xy > self.goal_xy_tolerance:
                self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Moving to point. Dist: {distance_to_goal_xy:.2f}m")
                if abs(heading_error_to_point) > self.approach_heading_tolerance * 1.5:
                    self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                    self.stop_robot()
                    return
                
                # ========================= CHANGED START =========================
                # Upgraded linear velocity from P to a full PD controller for smoother deceleration.
                
                # Proportional term (P): Based on how far the robot is.
                p_linear = self.linear_Kp * distance_to_goal_xy
                
                # Derivative term (D): Based on how fast the distance is changing.
                # This helps to dampen the response and prevent overshoot.
                distance_error_derivative = (distance_to_goal_xy - self.previous_distance_error) / dt
                d_linear = self.linear_Kd * distance_error_derivative
                
                # PD controller output for linear velocity
                linear_velocity = p_linear + d_linear
                
                # Clamp the velocity to the maximum allowed speed and ensure it's always positive (moving forward).
                twist_cmd.linear.x = max(min(linear_velocity, self.max_linear_speed), 0.0)

                # ========================== CHANGED END ==========================

                angular_error_derivative = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                angular_velocity = (self.angular_Kp * heading_error_to_point) + (self.angular_Kd * angular_error_derivative)
                # Reduce max angular speed while moving forward to prevent wild turns
                twist_cmd.angular.z = max(min(angular_velocity, self.max_angular_speed * 0.7), -self.max_angular_speed * 0.7)
            else:
                self.current_nav_state = self.NAV_STATE_REACHED_POINT_XY
                self.stop_robot()
                return
            self.previous_distance_error = distance_to_goal_xy
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_REACHED_POINT_XY:
            self.current_nav_state = self.NAV_STATE_ALIGNING_FINAL_ORIENTATION
            self.previous_final_heading_error = final_heading_error # Reset for final alignment
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Reached (x,y). Aligning final orientation.")

        elif self.current_nav_state == self.NAV_STATE_ALIGNING_FINAL_ORIENTATION:
            if abs(final_heading_error) > self.goal_final_theta_tolerance:
                self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Aligning final orientation. Err: {final_heading_error:.2f} rad")
                angular_error_derivative = (final_heading_error - self.previous_final_heading_error) / dt
                angular_velocity = (self.angular_Kp * final_heading_error) + (self.angular_Kd * angular_error_derivative)
                twist_cmd.angular.z = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.current_nav_state = self.NAV_STATE_WAYPOINT_COMPLETE
                self.stop_robot()
                return
            self.previous_final_heading_error = final_heading_error

        elif self.current_nav_state == self.NAV_STATE_WAYPOINT_COMPLETE:
            self.log_and_publish_status(f"Waypoint {self.current_waypoint_index} fully completed.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.target_waypoints):
                self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                # Reset all previous error terms for the new waypoint
                self.previous_heading_error_to_point = 0.0
                self.previous_distance_error = 0.0
                self.previous_final_heading_error = 0.0
                self.log_and_publish_status(f"Targeting next waypoint {self.current_waypoint_index}.")
            else:
                self.current_nav_state = self.NAV_STATE_ALL_WAYPOINTS_DONE
                self.log_and_publish_status("All waypoints successfully navigated!")
                self.stop_robot()
            return

        self.cmd_vel_publisher.publish(twist_cmd)

    def stop_robot(self):
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)

    def log_and_publish_status(self, status_text: str):
        self.get_logger().info(status_text)
        status_msg = StringMsg()
        status_msg.data = f"WptIdx: {self.current_waypoint_index}, State: {self.current_nav_state}, Msg: {status_text}"
        self.status_publisher.publish(status_msg)

    def on_shutdown(self):
        self.get_logger().info("PD Waypoint Navigator shutting down...")
        self.stop_robot()
        self.get_logger().info("Robot stopped. Shutdown complete.")

def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    navigator_node = WaypointNavigatorPD()
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        navigator_node.get_logger().info('Keyboard interrupt, shutting down navigator.')
    finally:
        # Graceful shutdown
        navigator_node.on_shutdown()
        if rclpy.ok():
            navigator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
