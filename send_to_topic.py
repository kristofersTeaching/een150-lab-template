import rclpy
import json
import time
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal')

    publisher = node.create_publisher(String, '/opc_command', 10)
    msg = String()
    pos = {"ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_1" : True}

    msg.data = json.dumps(pos)
    time.sleep(0.1)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()