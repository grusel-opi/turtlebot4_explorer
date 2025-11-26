import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class RecorderClient(Node):

    def __init__(self):
        super().__init__('rosbag 2 remote recorder client')
        self.cli = self.create_client(Trigger, 'rosbag2_record')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.req
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    recorder_client = RecorderClient()

    future = recorder_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    rclpy.spin_until_future_complete(recorder_client, future)
    
    response = future.result()
    
    recorder_client.get_logger().info(response)

    recorder_client.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()