import json
import math
from threading import Lock
import time

import cv2
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node as GzNode
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BallDetectorNode(Node):
    SUPPORTED_REQUESTS = {'red', 'green', 'blue', 'yellow', 'all'}

    def __init__(self):
        super().__init__('ball_detector')

        self.declare_parameter('world_name', 'hydrolab_built')
        self.declare_parameter('camera_model_name', 'x500_gimbal_0')
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('publish_interval_s', 0.1)
        self.declare_parameter('min_detection_area_px', 80.0)
        self.declare_parameter('show_debug_view', False)

        self.world_name = str(self.get_parameter('world_name').value)
        self.camera_model_name = str(
            self.get_parameter('camera_model_name').value
        )
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.publish_interval_s = max(
            float(self.get_parameter('publish_interval_s').value),
            0.05,
        )
        self.min_detection_area_px = max(
            float(self.get_parameter('min_detection_area_px').value),
            1.0,
        )
        self.show_debug_view = bool(
            self.get_parameter('show_debug_view').value
        )

        if not self.camera_topic:
            self.camera_topic = (
                f'/world/{self.world_name}/model/{self.camera_model_name}'
                '/link/camera_link/sensor/camera/image'
            )

        self.result_publisher = self.create_publisher(
            String,
            '/vision/ball_detection/result',
            10,
        )
        self.request_subscriber = self.create_subscription(
            String,
            '/vision/ball_detection/request',
            self.request_callback,
            10,
        )

        self.gz_node = GzNode()
        self.processing_lock = Lock()
        self.detector_active = False
        self.requested_color = 'all'
        self.last_publish_ns = 0
        self.last_log_ns = 0

        success = self.gz_node.subscribe(
            Image,
            self.camera_topic,
            self.image_callback,
        )
        if not success:
            self.get_logger().error(
                f'Failed to subscribe to Gazebo camera topic: {self.camera_topic}'
            )
        else:
            self.get_logger().info(
                f'Ball Detector subscribed to camera topic: {self.camera_topic}'
            )

    def request_callback(self, msg):
        request = msg.data.strip().lower()

        with self.processing_lock:
            if request in ('', 'stop', 'off', 'disable', 'none'):
                self.detector_active = False
                self.requested_color = 'all'
                self.publish_result([], active=False)
                self.get_logger().info('Ball detection disabled.')
                if self.show_debug_view:
                    cv2.destroyAllWindows()
                return

            if request not in self.SUPPORTED_REQUESTS:
                self.get_logger().warn(
                    f'Unsupported ball detection request: "{request}"'
                )
                return

            self.detector_active = True
            self.requested_color = request
            self.get_logger().info(
                f'Ball detection enabled. Requested color: {request}'
            )

    def image_callback(self, msg):
        with self.processing_lock:
            if not self.detector_active:
                return
            requested_color = self.requested_color

        try:
            expected_size = msg.width * msg.height * 3
            if len(msg.data) != expected_size:
                self.get_logger().warn(
                    'Unexpected image payload size from Gazebo camera.'
                )
                return

            image_rgb = np.frombuffer(msg.data, dtype=np.uint8)
            image_rgb = image_rgb.reshape((msg.height, msg.width, 3))
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            detections = self.detect_balls(image_bgr, requested_color)
            now_ns = time.time_ns()

            if self.show_debug_view:
                self.show_debug_frame(image_bgr, detections, requested_color)

            if now_ns - self.last_publish_ns >= int(
                self.publish_interval_s * 1e9
            ):
                self.publish_result(detections, active=True)
                self.last_publish_ns = now_ns

            if detections and now_ns - self.last_log_ns >= int(1e9):
                summary = ', '.join(
                    (
                        f'{item["color"]}'
                        f'@({item["center_px"][0]},{item["center_px"][1]})'
                    )
                    for item in detections
                )
                self.get_logger().info(f'Ball detections: {summary}')
                self.last_log_ns = now_ns
        except Exception as exc:
            self.get_logger().error(f'Ball detector failed to process frame: {exc}')

    def detect_balls(self, image_bgr, requested_color):
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        colors_to_process = (
            ['red', 'green', 'blue', 'yellow']
            if requested_color == 'all'
            else [requested_color]
        )
        detections = []

        for color_name in colors_to_process:
            mask = self.build_mask(hsv, color_name)
            detection = self.extract_largest_detection(
                mask,
                color_name,
                image_bgr.shape[1],
                image_bgr.shape[0],
            )
            if detection is not None:
                detections.append(detection)

        return detections

    def build_mask(self, hsv, color_name):
        if color_name == 'red':
            # Broadened Saturation and Value to catch darker/paler red in shadows
            lower_1 = np.array([0, 70, 30], dtype=np.uint8)
            upper_1 = np.array([10, 255, 255], dtype=np.uint8)
            lower_2 = np.array([170, 70, 30], dtype=np.uint8)
            upper_2 = np.array([180, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_1, upper_1)
            mask |= cv2.inRange(hsv, lower_2, upper_2)
        elif color_name == 'blue':
            # Broadened Saturation and Value to catch darker blue
            lower = np.array([90, 70, 30], dtype=np.uint8)
            upper = np.array([135, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        elif color_name == 'green':
            # Pool / turf green; hue above yellow band to reduce cross-talk
            lower = np.array([42, 70, 30], dtype=np.uint8)
            upper = np.array([88, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        elif color_name == 'yellow':
            lower = np.array([18, 70, 40], dtype=np.uint8)
            upper = np.array([38, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        else:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        # Use a smaller kernel for OPEN to not erase small/distant balls
        kernel_open = np.ones((2, 2), dtype=np.uint8)
        # Use a larger kernel for CLOSE to fill in gaps inside the ball mask
        kernel_close = np.ones((7, 7), dtype=np.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        return mask

    def extract_largest_detection(self, mask, color_name, width, height):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        # Filter contours by shape and area to avoid finding the "sky" or reflections
        valid_contours = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_detection_area_px:
                continue
                
            # If the area is massive (like half the screen), it's likely the sky
            if area > (width * height * 0.4):
                continue
                
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / max(float(h), 1.0)
            
            # A tennis ball should be roughly square in its bounding box
            if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                continue
                
            # Check circularity: contour area compared to its bounding circle area
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circle_area = math.pi * (radius ** 2)
            if circle_area <= 0:
                continue
                
            fill_ratio = area / circle_area
            # A perfect circle has fill ratio of ~1.0; squares are less, weird blobs even less.
            if fill_ratio < 0.4:
                continue

            valid_contours.append(cnt)

        if not valid_contours:
            return None

        # Take the largest valid circular contour
        largest = max(valid_contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))

        (center_x, center_y), radius = cv2.minEnclosingCircle(largest)
        moments = cv2.moments(largest)
        if moments['m00'] == 0.0:
            return None

        centroid_x = int(moments['m10'] / moments['m00'])
        centroid_y = int(moments['m01'] / moments['m00'])
        bb_x, bb_y, bb_w, bb_h = cv2.boundingRect(largest)
        
        return {
            'color': color_name,
            'center_px': [centroid_x, centroid_y],
            'center_uv': [
                round(float(center_x) / float(width), 4),
                round(float(center_y) / float(height), 4),
            ],
            'bounding_box_px': [bb_x, bb_y, bb_w, bb_h],
            'radius_px': round(float(radius), 2),
            'area_px': round(area, 2),
        }

    def publish_result(self, detections, active):
        result = {
            'active': active,
            'request': self.requested_color if active else 'none',
            'detections': detections,
        }
        msg = String()
        msg.data = json.dumps(result)
        self.result_publisher.publish(msg)

    def show_debug_frame(self, image_bgr, detections, requested_color):
        debug_frame = image_bgr.copy()
        for detection in detections:
            center_x, center_y = detection['center_px']
            radius_px = int(detection['radius_px'])
            color_bgr = self.debug_color(detection['color'])
            cv2.circle(debug_frame, (center_x, center_y), radius_px, color_bgr, 2)
            cv2.putText(
                debug_frame,
                detection['color'],
                (center_x + 10, center_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color_bgr,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            debug_frame,
            f'Request: {requested_color}',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow('Ball Detector', debug_frame)
        cv2.waitKey(1)

    def debug_color(self, color_name):
        if color_name == 'red':
            return (0, 0, 255)
        if color_name == 'blue':
            return (255, 0, 0)
        if color_name == 'green':
            return (0, 255, 0)
        return (0, 255, 255)

    def destroy_node(self):
        if self.show_debug_view:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
