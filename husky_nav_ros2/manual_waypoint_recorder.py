import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import math
import sys
import os

# --- Configuration ---
# File to save waypoints to
WAYPOINTS_FILENAME = "manual_waypoints.csv" 
# PS4 Controller Button Mapping
JOYSTICK_BUTTON_ADD_WAYPOINT = 3  # 'Triangle' button
JOYSTICK_BUTTON_SAVE_AND_EXIT = 2 # 'Square' button

# --- Math Utilities (included for a self-contained script) ---
def get_yaw_from_quaternion(orientation):
    """
    Converts a quaternion message to a yaw angle (in radians).
    """
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle_radians(angle):
    """
    Normalizes an angle to the range [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi

class ManualWaypointRecorder(Node):
    def __init__(self):
        super().__init__('manual_waypoint_recorder_node')
        self.get_logger().info("Manual Waypoint Recorder initializing...")
        
        # Stores the collected waypoints
        self.waypoints = []
        # Stores the most recent pose received from the robot
        self.current_pose = None

        # State tracking for joystick buttons to avoid multiple triggers on one press
        self.add_button_previously_pressed = False
        self.save_button_previously_pressed = False

        # --- Subscribers ---
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",  # Topic for robot's position
            self.pose_callback,
            10
        )
        self.joystick_subscriber = self.create_subscription(
            Joy,
            "/a200_1046/joy_teleop/joy", # Topic for joystick commands
            self.joystick_callback,
            10
        )

        # --- User Instructions ---
        self.get_logger().info("Node initialized successfully.")
        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info(f"Drive the robot to a desired location and press Joystick Button {JOYSTICK_BUTTON_ADD_WAYPOINT} ('Triangle') to ADD a waypoint.")
        self.get_logger().info(f"Press Joystick Button {JOYSTICK_BUTTON_SAVE_AND_EXIT} ('Square') to SAVE the path and EXIT.")
        self.get_logger().info(f"Waypoints will be saved to: {os.path.abspath(WAYPOINTS_FILENAME)}")
        self.get_logger().info("--------------------------------------------------")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback to continuously update the robot's current pose.
        """
        self.current_pose = msg

    def joystick_callback(self, joy_msg: Joy):
        """
        Callback to handle joystick button presses for adding waypoints and saving.
        """
        # --- Check for 'Add Waypoint' Button Press ---
        add_button_currently_pressed = (len(joy_msg.buttons) > JOYSTICK_BUTTON_ADD_WAYPOINT and 
                                        joy_msg.buttons[JOYSTICK_BUTTON_ADD_WAYPOINT] == 1)
        
        # Trigger on the rising edge (button was not pressed, but now it is)
        if add_button_currently_pressed and not self.add_button_previously_pressed:
            self.add_waypoint()
        self.add_button_previously_pressed = add_button_currently_pressed

        # --- Check for 'Save and Exit' Button Press ---
        save_button_currently_pressed = (len(joy_msg.buttons) > JOYSTICK_BUTTON_SAVE_AND_EXIT and
                                         joy_msg.buttons[JOYSTICK_BUTTON_SAVE_AND_EXIT] == 1)
        
        if save_button_currently_pressed and not self.save_button_previously_pressed:
            self.save_waypoints_and_shutdown()
        self.save_button_previously_pressed = save_button_currently_pressed

    def add_waypoint(self):
        """
        Captures the robot's current pose and adds it to the list of waypoints.
        """
        if self.current_pose is None:
            self.get_logger().warn("Cannot add waypoint, no pose data received yet.")
            return

        pose = self.current_pose.pose
        yaw = get_yaw_from_quaternion(pose.orientation)
        
        # Append the processed waypoint to our list
        self.waypoints.append({
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': yaw
        })
        
        self.get_logger().info(f"✅ Waypoint #{len(self.waypoints)} added at (x={pose.position.x:.2f}, y={pose.position.y:.2f})")

    def save_waypoints_and_shutdown(self):
        """
        Saves the collected waypoints to a CSV file and then shuts down the node.
        """
        if not self.waypoints:
            self.get_logger().warn("No waypoints were recorded. Nothing to save. Shutting down.")
            rclpy.shutdown()
            return

        full_path = os.path.abspath(WAYPOINTS_FILENAME)
        try:
            with open(WAYPOINTS_FILENAME, 'w') as f:
                # Write header (optional, but good practice)
                # f.write("x,y,theta\n") 
                for wp in self.waypoints:
                    f.write(f"{wp['x']},{wp['y']},{wp['theta']}\n")
            
            self.get_logger().info("--------------------------------------------------")
            self.get_logger().info(f"✅ Successfully saved {len(self.waypoints)} waypoints to {full_path}")
            self.get_logger().info("--------------------------------------------------")

        except IOError as e:
            self.get_logger().error(f"❌ Could not write waypoints to file {full_path}: {e}")
        
        finally:
            self.get_logger().info("Shutting down node...")
            # This will stop the rclpy.spin() in main and allow the script to exit
            self.destroy_node() 
            sys.exit(0) # Forcing exit to ensure the script terminates cleanly


def main(args=None):
    rclpy.init(args=args)
    recorder_node = ManualWaypointRecorder()
    try:
        rclpy.spin(recorder_node)
    except (KeyboardInterrupt, SystemExit):
        recorder_node.get_logger().info('Node shutting down.')
    finally:
        if rclpy.ok() and recorder_node.is_valid():
            recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
