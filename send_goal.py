import rclpy
import json
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal')

    publisher = node.create_publisher(String, '/set_state', 10)

    msg = String()
    pos = {"replan": False}

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    # set a state here to force replan
    pos = {
        "replan": True, 
        # "cyl_at_hcpos1": True, 
        # "cyl_at_hcpos2": False, 
        "goal_as_string": "cyl_at_pose_2"  # write a guard as a string that the guard parser can read
    }

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()