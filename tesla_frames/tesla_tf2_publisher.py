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
from geometry_msgs.msg import TransformStamped
import tf_transformations


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
        base_link_to_footprint.transform.translation.z = 1.0
        base_link_to_footprint.transform.rotation.w = 1.0  # No rotation

        # IMU to Base Link (same position as base_link)
        imu_to_base_link = TransformStamped()
        imu_to_base_link.header.stamp = self.get_clock().now().to_msg()
        imu_to_base_link.header.frame_id = "base_link"
        imu_to_base_link.child_frame_id = "imu"
        imu_to_base_link.transform.translation.x = 0.0
        imu_to_base_link.transform.translation.y = 0.0
        imu_to_base_link.transform.translation.z = 0.0
        imu_to_base_link.transform.rotation.w = 1.0  # No rotation

        # Velodyne to Base Link
        velodyne_to_base_link = TransformStamped()
        velodyne_to_base_link.header.stamp = self.get_clock().now().to_msg()
        velodyne_to_base_link.header.frame_id = "base_link"
        velodyne_to_base_link.child_frame_id = "velodyne"
        velodyne_to_base_link.transform.translation.x = 0.0
        velodyne_to_base_link.transform.translation.y = 0.0
        velodyne_to_base_link.transform.translation.z = 0.65  # 65 cm
        velodyne_to_base_link.transform.rotation.w = 1.0  # No rotation

        # Broadcast the static transforms
        self.broadcaster.sendTransform(
            [base_link_to_footprint, imu_to_base_link, velodyne_to_base_link]
        )


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
