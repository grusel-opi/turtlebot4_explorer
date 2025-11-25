import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick
from builtin_interfaces.msg import Time as TimeMsg

class MadgwickEstimator(Node):
    """
    ROS 2 node subscribing to oakd IMU messages to estimate the orientation of
    the camera. Usefull to automatically change the URDF of the robot after
    mounting the camera in a different orientation.
    Uses the Madgwick AHRS-filter for estimation and prints euler angles.
    """

    def __init__(self):
        super().__init__('madgwick_estimator')
        
        self.is_initialized = False
        self.last_timestamp = None
        self.deltat_samples = []
        self.SAMPLE_COUNT = 20
        self.estimated_frequency = 0.0
        
        self.madgwick = Madgwick() 
        
        self.subscription = self.create_subscription(
            Imu,
            '/oakd/imu/data',
            self.imu_callback,
            10
        )
        self.get_logger().info('Madgwick AHRS filter: estimating publishing frequency of "/oakd/imu/data"...')
        

    def imu_callback(self, msg: Imu):
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        if not self.is_initialized:
            if self.last_timestamp is not None:
                deltat = current_time - self.last_timestamp
                if deltat > 0.0001 and deltat < 1.0: 
                    self.deltat_samples.append(deltat)
                
            self.last_timestamp = current_time

            if len(self.deltat_samples) >= self.SAMPLE_COUNT:
                avg_deltat = np.mean(self.deltat_samples)
                self.estimated_frequency = 1.0 / avg_deltat

                self.get_logger().info(
                    f'Got frequency of {self.estimated_frequency:.2f} Hz'
                )

                self.madgwick = Madgwick(frequency=self.estimated_frequency)
                self.is_initialized = True
            
            if not self.is_initialized:
                return 

        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        self.madgwick.update_imu(gyr, acc)
        
        q_madgwick = self.madgwick.Q

        # [w, x, y, z] -> [x, y, z, w]
        q_scipy = np.array([q_madgwick[1], q_madgwick[2], q_madgwick[3], q_madgwick[0]])
        r = R.from_quat(q_scipy)
        euler_rad = r.as_euler('zyx', degrees=False) 
        roll_deg = np.rad2deg(euler_rad[2])
        pitch_deg = np.rad2deg(euler_rad[1])
        yaw_deg = np.rad2deg(euler_rad[0])

        self.get_logger().info(
            f"Roll: {roll_deg:.2f}, Pitch: {pitch_deg:.2f}, Yaw: {yaw_deg:.2f}",
            throttle_duration_sec=0.1
        )

def main(args=None):
    rclpy.init(args=args)
    madgwick_estimator = MadgwickEstimator()
    try:
        rclpy.spin(madgwick_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        madgwick_estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()