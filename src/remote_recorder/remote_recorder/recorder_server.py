import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rosbag2_remote_record_action_interfaces.action import Start, Stop
import subprocess
import signal


class RecorderActionServer(Node):

    def __init__(self):
        super().__init__('rosbag2_action_server')
        self.start_server = ActionServer(self, Start, 'rosbag2_record_start', self.start_record_callback)
        self.stop_server = ActionServer(self, Stop, 'rosbag2_record_stop', self.stop_record_callback)

        self.recorder = None
        self.record_args = ["ros2", "bag", "record"]
        

    def start_record_callback(self, goal_handle):
        self.get_logger().info('Received action request.')

        if self.recorder:
            self.get_logger().info('Recorder already started.')
            feedback = Start.Feedback()
            feedback.status = "Recording already started."
            goal_handle.publish_feedback(feedback)
            goal_handle.canceled()    
            return
        
        self.get_logger().info('Starting rosbag2 record.')
        feedback = Start.Feedback()
        feedback.status = "Starting rosbag2 record."
        goal_handle.publish_feedback(feedback)

        self.recorder = subprocess.Popen(self.record_args + [goal_handle.request.topics], text=True)

        feedback.status = "Waiting for rosbag2 startup.."
        goal_handle.publish_feedback(feedback)

        rate = self.create_rate(1)
        count = 10
        while rclpy.ok() and count > 0:
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(str(count))
            rate.sleep()
            count -= 1

        rate = self.create_rate(1)
        outs, errs = self.recorder.communicate()

        while "Recording..." not in str(outs) and rclpy.ok():
            goal_handle.publish_feedback(feedback)
            rate.sleep()
            outs, errs = self.recorder.communicate()

        goal_handle.succeed()
        result = Start.Result()
        result.stdout = outs
        result.stderr = errs



    def stop_record_callback(self, goal_handle):
        feedback = Stop.Feedback()
        feedback.status = "Sending SIGINT to rosbag2 record process."
        goal_handle.publish_feedback(feedback)

        self.recorder.send_signal(signal.SIGINT)
        
        feedback.status = "Waiting for rosbag2 to write all messages."
        goal_handle.publish_feedback(feedback)
        
        outs, errs = self.recorder.communicate()
        
        result = Stop.Result()
        result.stdout = outs
        result.stderr = errs
        
        goal_handle.succeed()
        
        return result


def main(args=None):
    rclpy.init()
    recorder = RecorderActionServer()
    rclpy.spin(recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()