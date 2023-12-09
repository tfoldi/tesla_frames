# Copyright 2023, Tamas Foldi
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class TeslaTf2Publisher(Node):
    def __init__(self):
        super().__init__("tesla_tf2_publisher")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        # Base Link to Base Footprint
        base_link_to_footprint = TransformStamped()
        base_link_to_footprint.header.stamp = self.get_clock().now().to_msg()
        base_link_to_footprint.header.frame_id = "base_footprint"
        base_link_to_footprint.child_frame_id = "base_link"
        base_link_to_footprint.transform.translation.x = 0.0
        base_link_to_footprint.transform.translation.y = 0.0
        base_link_to_footprint.transform.translation.z = 0.7215
        base_link_to_footprint.transform.rotation.w = 1.0  # No rotation

        # IMU to Base Link (same position as base_link)
        imu_to_base_link = TransformStamped()
        imu_to_base_link.header.stamp = self.get_clock().now().to_msg()
        imu_to_base_link.header.frame_id = "base_link"
        imu_to_base_link.child_frame_id = "imu"
        imu_to_base_link.transform.translation.x = 0.517  # 183 from the car's front bumper
        imu_to_base_link.transform.translation.y = 0.0
        imu_to_base_link.transform.translation.z = -0.7215 + 0.6
        imu_to_base_link.transform.rotation.w = 1.0  # No rotation

        # ublox gps to Base Link (same position as base_link)
        ubx_to_base_link = TransformStamped()
        ubx_to_base_link.header.stamp = self.get_clock().now().to_msg()
        ubx_to_base_link.header.frame_id = "base_link"
        ubx_to_base_link.child_frame_id = "ubx"
        ubx_to_base_link.transform.translation.x = 0.0
        ubx_to_base_link.transform.translation.y = 0.0
        ubx_to_base_link.transform.translation.z = 0.0
        ubx_to_base_link.transform.rotation.w = 1.0  # No rotation

        # Velodyne to Base Link
        velodyne_to_base_link = TransformStamped()
        velodyne_to_base_link.header.stamp = self.get_clock().now().to_msg()
        velodyne_to_base_link.header.frame_id = "base_link"
        velodyne_to_base_link.child_frame_id = "velodyne"
        velodyne_to_base_link.transform.translation.x = 0.052
        velodyne_to_base_link.transform.translation.y = 0.0
        velodyne_to_base_link.transform.translation.z = 0.8385
        velodyne_to_base_link.transform.rotation.w = 1.0  # No rotation

        # camera 128, 73
        cam_left_to_base_link = TransformStamped()
        cam_left_to_base_link.header.stamp = self.get_clock().now().to_msg()
        cam_left_to_base_link.header.frame_id = "base_link"
        cam_left_to_base_link.child_frame_id = "cam_left"
        cam_left_to_base_link.transform.translation.x = 2.347 - 1.28
        cam_left_to_base_link.transform.translation.y = 0.896
        cam_left_to_base_link.transform.translation.z = -0.7215 + 0.73
        cam_left_to_base_link.transform.rotation = quaternion_from_euler(0, 0, math.radians(135))

        # camera 128, 73
        cam_right_to_base_link = TransformStamped()
        cam_right_to_base_link.header.stamp = self.get_clock().now().to_msg()
        cam_right_to_base_link.header.frame_id = "base_link"
        cam_right_to_base_link.child_frame_id = "cam_right"
        cam_right_to_base_link.transform.translation.x = 2.347 - 1.28
        cam_right_to_base_link.transform.translation.y = -0.896
        cam_right_to_base_link.transform.translation.z = -0.7215 + 0.73
        cam_right_to_base_link.transform.rotation = quaternion_from_euler(0, 0, math.radians(-135))

        # Broadcast the static transforms
        self.broadcaster.sendTransform(
            [
                base_link_to_footprint,
                imu_to_base_link,
                ubx_to_base_link,
                velodyne_to_base_link,
                cam_left_to_base_link,
                cam_right_to_base_link,
            ]
        )

        self.get_logger().info("Published static transforms from Tesla model 3")


def main(args=None):
    rclpy.init(args=args)
    node = TeslaTf2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
