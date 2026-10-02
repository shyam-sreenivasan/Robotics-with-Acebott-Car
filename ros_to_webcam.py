#!/usr/bin/env python3

import subprocess

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


TOPIC = '/conduit/camera/front/image_raw/compressed'
VIDEO_DEVICE = '/dev/video10'

# Portrait, matching the 480x640 frames Conduit publishes. Any 3:4 size
# keeps the aspect ratio; 1280x720 would stretch the image sideways.
WIDTH = 720
HEIGHT = 1080
FPS = 30


class RosToWebcam(Node):

    def __init__(self):
        super().__init__('ros_to_webcam')

        self.get_logger().info(
            f'Starting ROS camera bridge:\n'
            f'  Topic: {TOPIC}\n'
            f'  Output: {VIDEO_DEVICE}'
        )

        self.ffmpeg = subprocess.Popen(
            [
                'ffmpeg',
                '-loglevel', 'warning',

                # Input from Python/OpenCV
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-video_size', f'{WIDTH}x{HEIGHT}',
                '-framerate', str(FPS),
                '-i', '-',

                # Convert to format suitable for V4L2 webcam
                '-vf', 'format=yuv420p',

                # Output to virtual webcam
                '-f', 'v4l2',
                VIDEO_DEVICE
            ],
            stdin=subprocess.PIPE
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            TOPIC,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.frame_count = 0

        self.get_logger().info('Waiting for camera frames...')

    def image_callback(self, msg):

        # Convert ROS compressed-image bytes -> numpy array
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)

        # Decode JPEG/PNG compressed image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Could not decode camera frame')
            return

        # Resize all frames to a fixed webcam resolution
        frame = cv2.resize(
            frame,
            (WIDTH, HEIGHT),
            interpolation=cv2.INTER_LINEAR
        )

        self.frame_count += 1

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'Received {self.frame_count} frames'
            )

        # Check if ffmpeg exited unexpectedly
        if self.ffmpeg.poll() is not None:
            self.get_logger().error(
                'FFmpeg exited unexpectedly'
            )
            return

        try:
            self.ffmpeg.stdin.write(frame.tobytes())

        except BrokenPipeError:
            self.get_logger().error(
                'FFmpeg pipe closed'
            )

        except Exception as e:
            self.get_logger().error(
                f'Error writing frame: {e}'
            )

    def cleanup(self):

        self.get_logger().info('Stopping camera bridge...')

        if self.ffmpeg.stdin:
            try:
                self.ffmpeg.stdin.close()
            except Exception:
                pass

        try:
            self.ffmpeg.terminate()
            self.ffmpeg.wait(timeout=2)

        except Exception:
            self.ffmpeg.kill()


def main(args=None):

    rclpy.init(args=args)

    node = RosToWebcam()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.cleanup()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()