#!/usr/bin/env python3

# Copyright 2026 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Rescale the Isaac ROS detections to the ZED image and publish Foxglove image annotations.

The Isaac ROS decoders publish the bounding boxes in the coordinates of the network input
image (640x640): the ZED image is resized keeping its aspect ratio, then padded on the
bottom and on the right side. This node maps the bounding boxes back to the coordinates of
the ZED image, using the image size from the ZED `camera_info` topic, and publishes:

* `detections` (`vision_msgs/Detection2DArray`): the detections in ZED image coordinates.
* `annotations` (`foxglove_msgs/ImageAnnotations`): the bounding boxes and the labels, to
  be displayed over the ZED image in the Foxglove Image panel.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from foxglove_msgs.msg import Color, ImageAnnotations, Point2, PointsAnnotation, TextAnnotation
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray


class ZedDetectionRescaler(Node):

    BOX_COLOR = Color(r=0.0, g=1.0, b=0.0, a=1.0)
    TEXT_COLOR = Color(r=0.0, g=0.0, b=0.0, a=1.0)
    TEXT_BG_COLOR = Color(r=0.0, g=1.0, b=0.0, a=0.8)

    def __init__(self):
        super().__init__('zed_detection_rescaler')

        self._net_width = self.declare_parameter('network_image_width', 640).value
        self._net_height = self.declare_parameter('network_image_height', 640).value
        # Names of the classes, indexed by class ID. The class ID is displayed if empty
        self._class_names = self.declare_parameter('class_names', ['']).value

        self._img_width = 0
        self._img_height = 0
        self._scale_x = 0.0
        self._scale_y = 0.0

        self.create_subscription(
            CameraInfo, 'camera_info', self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, 'detections_output', self._detections_callback, 10)

        self._detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self._annotations_pub = self.create_publisher(ImageAnnotations, 'annotations', 10)

    def _camera_info_callback(self, msg):
        if msg.width == self._img_width and msg.height == self._img_height:
            return

        # Same computation as the Isaac ROS ResizeNode with `keep_aspect_ratio: True`
        # and `disable_padding: True`, in single precision as the original code
        res_width = self._net_width
        res_height = self._net_height
        height_factor = np.float32(res_height) / np.float32(msg.height)
        width_factor = np.float32(res_width) / np.float32(msg.width)
        if height_factor < width_factor:
            res_width = int(np.float32(msg.width) * height_factor)
            if res_width % 2 != 0:
                res_width += 1
        elif width_factor < height_factor:
            res_height = int(np.float32(msg.height) * width_factor)
            if res_height % 2 != 0:
                res_height += 1

        self._img_width = msg.width
        self._img_height = msg.height
        self._scale_x = msg.width / res_width
        self._scale_y = msg.height / res_height
        self.get_logger().info(
            f'ZED image size: {msg.width}x{msg.height}. Network input: {res_width}x{res_height}'
            f' padded to {self._net_width}x{self._net_height}')

    def _get_label(self, class_id):
        try:
            idx = int(class_id)
        except ValueError:
            return class_id
        if 0 <= idx < len(self._class_names) and self._class_names[idx]:
            return self._class_names[idx]
        return class_id

    def _detections_callback(self, msg):
        if self._img_width == 0:
            self.get_logger().warning(
                'Waiting for the ZED camera_info topic to rescale the detections',
                throttle_duration_sec=5.0)
            return

        annotations = ImageAnnotations()
        font_size = max(12.0, self._img_height / 40.0)

        for det in msg.detections:
            det.bbox.center.position.x *= self._scale_x
            det.bbox.center.position.y *= self._scale_y
            det.bbox.size_x *= self._scale_x
            det.bbox.size_y *= self._scale_y

            min_x = det.bbox.center.position.x - det.bbox.size_x / 2.0
            min_y = det.bbox.center.position.y - det.bbox.size_y / 2.0
            max_x = min_x + det.bbox.size_x
            max_y = min_y + det.bbox.size_y

            box = PointsAnnotation()
            box.timestamp = msg.header.stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.points = [
                Point2(x=min_x, y=min_y), Point2(x=max_x, y=min_y),
                Point2(x=max_x, y=max_y), Point2(x=min_x, y=max_y)]
            box.outline_color = self.BOX_COLOR
            box.thickness = 2.0
            annotations.points.append(box)

            if det.results:
                hyp = det.results[0].hypothesis
                text = TextAnnotation()
                text.timestamp = msg.header.stamp
                text.position = Point2(x=min_x, y=min_y)
                text.text = f'{self._get_label(hyp.class_id)} {hyp.score:.2f}'
                text.font_size = font_size
                text.text_color = self.TEXT_COLOR
                text.background_color = self.TEXT_BG_COLOR
                annotations.texts.append(text)

        self._detections_pub.publish(msg)
        self._annotations_pub.publish(annotations)


def main():
    rclpy.init()
    node = ZedDetectionRescaler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A SIGTERM shuts down the context while the executor is waiting: the
        # executor can then raise an RCLError instead of ExternalShutdownException
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
