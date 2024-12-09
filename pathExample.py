import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np


def interpolate_between_points(start, end, interval_cm):
    """
    Given two points, this function interpolates additional points at a specified interval in cm.
    """
    interpolated_positions = []

    # Calculate the distance between the points in meters
    distance = np.linalg.norm(end - start)
    distance_cm = distance * 100.0  # Convert to cm

    # Determine the number of intervals
    num_intervals = int(distance_cm / interval_cm)

    # Generate interpolated points
    for i in range(num_intervals + 1):
        t = i / num_intervals
        interpolated_pos = linear_interpolate(start, end, t)
        interpolated_positions.append(interpolated_pos)

    # Ensure the end point is included
    interpolated_positions.append(end)

    return interpolated_positions


def linear_interpolate(start, end, t):
    """
    Linearly interpolate between two points based on a parameter t (0 <= t <= 1).
    """
    return start + t * (end - start)

def catmull_rom(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ((2.0 * p1) +
                  (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)


def handle_edge_cases(points):
    if len(points) == 2:
        p0 = points[0]
        p3 = points[1]
        p1 = p0 + (p3 - p0) * 0.33
        p2 = p0 + (p3 - p0) * 0.66
        return [p0, p1, p2, p3]
    elif len(points) == 3:
        p0 = points[0]
        p1 = points[1]
        p3 = points[2]
        p2 = p1 + (p3 - p1) * 0.5
        return [p0, p1, p2, p3]
    return points


def sample_catmull_rom_spline(points, samples_per_segment):
    points = handle_edge_cases(points)
    result = []
    for i in range(1, len(points) - 2):
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            point = catmull_rom(points[i - 1], points[i], points[i + 1], points[i + 2], t)
            result.append(point)
    result.append(points[-1])  # Add the last point
    return result


class CatmullRomNode(Node):
    def __init__(self):
        super().__init__('catmull_rom_path_node')
        self.subscription = self.create_subscription(
            Path,
            'plan',
            self.path_callback,
            10
        )
        self.publisher = self.create_publisher(
            Path,
            'smooth_path',
            10
        )

    def path_callback(self, msg):
        points = []
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position
            points.append(np.array([position.x, position.y, position.z]))

        if len(points) < 2:
            self.get_logger().warn('Not enough points for Catmull-Rom interpolation')
            return

        # Catmull-Rom spline sampling
        smoothed_points = sample_catmull_rom_spline(points, samples_per_segment=20)

        if not smoothed_points:
            self.get_logger().warn('Catmull-Rom sampling returned an empty path')
            return

        # Interpolation between points
        final_path = []
        for i in range(1, len(smoothed_points)):
            src_pose = smoothed_points[i - 1]
            target_pose = smoothed_points[i]

            interpolated_points = interpolate_between_points(src_pose, target_pose, interval_cm=4.0)
            final_path.extend(interpolated_points)

        # Publish the final path
        smooth_path = Path()
        smooth_path.header = msg.header

        for point in final_path:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = point[2]
            pose_stamped.pose.orientation.w = 1.0  # Default orientation
            smooth_path.poses.append(pose_stamped)

        self.publisher.publish(smooth_path)

def main(args=None):
    rclpy.init(args=args)
    node = CatmullRomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

