import threading
import roslibpy
import roslibpy.actionlib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from niryo_ned_ros2_interfaces.action import RobotMove
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

class ROS2DriverNode(Node):


    def __init__(self, robot_ip: str):
        super().__init__('ros2_driver_node')

        self.client = roslibpy.Ros(host=robot_ip, port=9090)
        self.client.run()
        
        self._joint_state_publisher = self.create_publisher(JointState, "/joint_state", qos_profile_sensor_data)

        self.action_server = ActionServer(
            self, RobotMove, "/niryo_robot_arm_commander/robot_action", 
            execute_callback=self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(f"Connected to ROSBridge at ws://{robot_ip}:9090")

        self.joint_state_listener = roslibpy.Topic(self.client, '/joint_states', 'sensor_msgs/JointState')
        self.joint_state_listener.subscribe(self.publish_joint_state)

    def publish_joint_state(self, data: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = data["name"]
        msg.position = data["position"]
        msg.velocity = data["velocity"]
        msg.effort = data["effort"]
        self._joint_state_publisher.publish(msg)

    def goal_callback(self, goal_request):
        """Handle incoming action goals."""
        self.get_logger().info(f"Received action goal: {goal_request.cmd}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle action cancellations."""
        self.get_logger().info("Canceling current action...")
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        """Handle action execution: send request to ROS1 and process feedback."""
        self.get_logger().info(f"Executing movement command: {goal_handle.request.cmd}")

        goal_data = {
            "cmd_type": goal_handle.request.cmd.cmd_type,
            "tcp_version": goal_handle.request.cmd.tcp_version,
            "joints": list(goal_handle.request.cmd.joints),
            "position": {
                "x": goal_handle.request.cmd.position.x,
                "y": goal_handle.request.cmd.position.y,
                "z": goal_handle.request.cmd.position.z
            },
            "rpy": {
                "roll": goal_handle.request.cmd.rpy.roll,
                "pitch": goal_handle.request.cmd.rpy.pitch,
                "yaw": goal_handle.request.cmd.rpy.yaw
            },
            "orientation": {
                "x": goal_handle.request.cmd.orientation.x,
                "y": goal_handle.request.cmd.orientation.y,
                "z": goal_handle.request.cmd.orientation.z,
                "w": goal_handle.request.cmd.orientation.w
            }
        }

        action_client = roslibpy.actionlib.ActionClient(self.client,
                                                '/niryo_robot_arm_commander/robot_action',
                                                'niryo_robot_arm_commander/RobotMoveAction')

         # Send the goal to ROS1
        action_goal = roslibpy.actionlib.Goal(action_client,
                               roslibpy.Message({'cmd': goal_data}))
        
        action_goal.on('feedback', lambda message: goal_handle.publish_feedback(self.convert_feedback(message)))

        action_goal.send()

        ros1_result = action_goal.wait(30)

        if ros1_result["status"] == 1:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        result = RobotMove.Result(status=ros1_result["status"], message=ros1_result["message"])
        
        action_client.dispose()
        return result

    def convert_feedback(self, feedback_data):
        """Converts received feedback into ROS2 ArmMove.Feedback message."""
        feedback_msg = RobotMove.Feedback()
        feedback_msg.state.position = Point(
            x=feedback_data["position"]["x"],
            y=feedback_data["position"]["y"],
            z=feedback_data["position"]["z"]
        )
        feedback_msg.state.rpy.roll = feedback_data["rpy"]["roll"]
        feedback_msg.state.rpy.pitch = feedback_data["rpy"]["pitch"]
        feedback_msg.state.rpy.yaw = feedback_data["rpy"]["yaw"]
        return feedback_msg

    def convert_result(self, result_data):
        """Converts received result into ROS2 ArmMove.Result message."""
        result_msg = RobotMove.Result()
        result_msg.status = result_data["status"]
        result_msg.message = result_data["message"]
        self.get_logger().info(f"Action completed: {result_msg.message}")
        return result_msg

def main():

    rclpy.init()

    robot_ip = "192.168.1.132"

    node = ROS2DriverNode(robot_ip)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()