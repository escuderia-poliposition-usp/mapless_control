from ament_index_python.packages import get_package_share_directory
import os

import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class SteeringController:
    def __init__(self, params):
        self.params = params
        self.ud_1 = 0
        self.steering_measurement_1 = 0
        self.angle_old = 0

    def compute_control_signal(self, desired_angle, current_angle):
        error = desired_angle - current_angle
        up = self.params['kp_steer'] * error
        td = self.params['td']
        n = self.params['n']
        ts = self.params['ts']
        kp_steer = self.params['kp_steer']

        ud = (
            (td / (td + n * ts)) * self.ud_1
            - (kp_steer * td / (td + n * ts))
            * (current_angle - self.steering_measurement_1)
        )
        control_signal = up + ud
        self.ud_1 = ud
        self.steering_measurement_1 = current_angle
        return control_signal

    def safe_curve_exit_factor(self, angle):
        if abs(self.angle_old - angle) < np.radians(
            self.params['steering_saturation'] * self.params['safe_curve_exit_ratio']
        ):
            self.angle_old = angle
            return 0.5
        self.angle_old = angle
        return 1.0


class FollowTheGap(Node):
    def __init__(self, config):
        super().__init__('follow_the_gap')

        # Publishers
        self.steer_publisher = self.create_publisher(
            Float32, config['topics']['steering_command'], 10)
        self.throttle_publisher = self.create_publisher(
            Float32, config['topics']['throttle_command'], 10)
        self.debug_lidar_publisher = self.create_publisher(
            LaserScan, config['topics']['debug_lidar'], 10)

        # Subscriptions
        self.create_subscription(
            LaserScan, config['topics']['lidar'], self.lidar_callback, 10)
        self.create_subscription(
            Float32, config['topics']['steering_angle'], self.steering_callback, 10)

        # Internal State
        self.lidar_msg = LaserScan()
        self.steering_msg = Float32()

        # Parameters from config
        self.params = config['parameters']
        self.controller = SteeringController(config['steering_controller'])

    def get_lidar_beam_index_by_angle(self, angle_degree):
        """Get the index of the LiDAR beam by angle."""
        angle = np.radians(angle_degree)
        angle_min = self.lidar_msg.angle_min
        angle_max = self.lidar_msg.angle_max
        angle_increment = self.lidar_msg.angle_increment
        if angle_min <= angle <= angle_max:
            index = int((angle - angle_min) / angle_increment)
            return index
        return -1

    def lidar_preprocessing(self, lidar):
        """Preprocess the LiDAR data by replacing invalid entries with max range."""
        max_val = lidar.range_max
        lidar.ranges = [
            r if r != float('inf') and r != float('-inf') and not np.isnan(r) else max_val
            for r in lidar.ranges
        ]
        return lidar

    def find_disparities(self, lidar):
        """Find disparities in the LiDAR data."""
        return [
            i for i in range(len(lidar.ranges) - 1)
            if abs(lidar.ranges[i + 1] - lidar.ranges[i]) > self.params['disparity_threshold']
        ]

    def mask_disparities(self, disp_index, lidar):
        """Mask disparities in the LiDAR data."""
        closer = disp_index if lidar.ranges[disp_index] < lidar.ranges[disp_index + 1] \
            else disp_index + 1
        alpha = np.arctan(
            (self.params['car_diameter'] / 2 + self.params['tol_mask_disparities'])
            / lidar.ranges[closer]
        )
        mask_count = int(np.ceil(alpha / lidar.angle_increment))

        start = max(0, closer - mask_count if closer == disp_index + 1 else closer)
        end = min(len(lidar.ranges), closer + mask_count if closer == disp_index else closer + 1)

        for i in range(start, end):
            lidar.ranges[i] = lidar.ranges[closer]

        return lidar

    def get_angle_and_distance_farthest(self, ranges, lidar):
        """Get the angle and distance of the farthest point in the LiDAR data."""
        for i in range(len(ranges)):
            angle = self.get_angle_from_lidar_beam_index(i, lidar)
            if abs(angle) > np.radians(90):
                ranges[i] = 0

        index_of_max = np.argmax(ranges)
        max_range = ranges[index_of_max]
        angle_of_max_range = self.get_angle_from_lidar_beam_index(index_of_max, lidar)
        return angle_of_max_range, max_range

    def get_angle_from_lidar_beam_index(self, index, lidar):
        """Get the angle from the LiDAR beam index."""
        return lidar.angle_min + index * lidar.angle_increment

    def get_range_from_angle(self, angle, ranges):
        """Get the range from the angle."""
        index = self.get_lidar_beam_index_by_angle(angle)
        if index != -1:
            return ranges[index]
        return None

    def set_safe_max_speed(self, distance, angle):
        """Set the safe maximum speed based on distance and angle."""
        speed_dist = distance / self.params['time_to_collision']
        steer_factor = 1 - abs(np.degrees(angle)) / (
            self.params['min_speed_factor'] * self.controller.params['steering_saturation']
        )
        curve_exit_factor = self.controller.safe_curve_exit_factor(angle)
        return curve_exit_factor * steer_factor * speed_dist

    def publish_control(self, steer, throttle):
        """Publish the control commands for steering and throttle."""
        steer = np.clip(
            steer,
            -np.radians(self.controller.params['steering_saturation']),
            np.radians(self.controller.params['steering_saturation'])
        )
        control_signal = self.controller.compute_control_signal(
            steer, self.steering_msg.data
        )
        self.steer_publisher.publish(Float32(data=control_signal))
        self.throttle_publisher.publish(Float32(data=throttle))

    def min_lateral_distance(self, lidar: LaserScan):
        """Calculate the minimum lateral distance from obstacles."""
        right_distances = []
        left_distances = []
        robustness_range = self.params['robustness_range']

        # For the left side (negative angles)
        for angle_degree in range(-robustness_range[0], -robustness_range[1], -1):
            range_at_angle = self.get_range_from_angle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                left_distances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))

        # For the right side (positive angles)
        for angle_degree in range(robustness_range[0], robustness_range[1]):
            range_at_angle = self.get_range_from_angle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                right_distances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))

        # Check if there are valid distances for both sides
        if left_distances and right_distances:
            mean_right_distance = np.mean(right_distances)
            mean_left_distance = np.mean(left_distances)
            return min(mean_right_distance, mean_left_distance)
        else:
            return 0.0

    def safe_steering(self, lidar):
        """Determine if steering is safe based on lateral distances."""
        min_lateral_distance = self.min_lateral_distance(lidar)
        return min_lateral_distance > self.params['safe_lateral_distance']

    def follow_the_gap(self):
        """Implement the Follow the Gap algorithm."""
        lidar = self.lidar_msg
        disparities = self.find_disparities(lidar)
        masked_lidar = lidar
        for disparity in disparities:
            masked_lidar = self.mask_disparities(disparity, masked_lidar)

        self.debug_lidar_publisher.publish(masked_lidar)

        angle, distance = self.get_angle_and_distance_farthest(masked_lidar.ranges, lidar)

        if not self.safe_steering(lidar):
            angle = 0.0

        max_speed = self.set_safe_max_speed(distance, angle)
        self.publish_control(angle, max_speed)

    def lidar_callback(self, msg):
        self.lidar_msg = self.lidar_preprocessing(msg)
        self.follow_the_gap()

    def steering_callback(self, msg):
        self.steering_msg = msg


def load_config(config_relative_path):
    package_name = 'ebva_mapless'
    package_share_directory = get_package_share_directory(package_name)
    config_path = os.path.join(package_share_directory, config_relative_path)
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)
    config = load_config('config/config.yaml')
    follow_the_gap = FollowTheGap(config)

    try:
        rclpy.spin(follow_the_gap)
    except KeyboardInterrupt:
        pass

    follow_the_gap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
