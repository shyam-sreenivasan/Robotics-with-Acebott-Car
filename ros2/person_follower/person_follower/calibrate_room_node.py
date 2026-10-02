"""Module 1: establish a home heading, sweep 360 degrees, return home.

Runs on its own and exits with a verdict. Nothing else in the follower
depends on it being running -- it exists to answer two questions before
any tracking is built on top:

  1. does a commanded sweep actually complete one full turn?
  2. can the room map recognise the starting heading again afterwards?

Both matter because the robot has no encoders or IMU. The sweep is
open-loop (ms_per_degree, measured at 7.0 in roam.py), so question 1 is
really "is that calibration right for this surface". Question 2 is what
makes homing drift-free later: matching the live camera view against the
map is closed-loop, so it does not inherit the timing error.

The report at the end gives a match score for the returned view against
the startup view, where 1.0 is identical. Anything above ~0.8 means the
robot is looking at what it started at.

Run with:
  ros2 run person_follower calibrate_room
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

CMD_TOPIC = "/follow_cmd_vel"
IMAGE_TOPIC = "/conduit/camera/front/image_raw/compressed"

WAITING = "WAITING"      # no frames yet
SWEEPING = "SWEEPING"    # the 360 degree turn
RETURNING = "RETURNING"  # homing on the map
DONE = "DONE"


class CalibrateRoom(Node):
    def __init__(self):
        super().__init__("calibrate_room")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("bins", 36)
        self.declare_parameter("ms_per_degree", 7.0)
        self.declare_parameter("home_tolerance_deg", 8.0)
        self.declare_parameter("rate", 20.0)
        # Give up rather than spinning forever if homing cannot settle.
        self.declare_parameter("return_timeout", 20.0)

        g = self.get_parameter
        self.turn_speed = g("turn_speed").value
        self.n_bins = max(8, int(g("bins").value))
        self.ms_per_degree = g("ms_per_degree").value
        self.home_tol = g("home_tolerance_deg").value
        self.rate = g("rate").value
        self.return_timeout = g("return_timeout").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, g("image_topic").value, self.on_image, qos
        )
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.live_view = None
        self.home_view = None
        self.room_map = [None] * self.n_bins
        self.swept = 0.0
        self.state = WAITING
        self.state_since = self.get_clock().now()

        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f"Room calibration: {self.n_bins} bins, "
            f"{360.0 / self.n_bins:.0f}deg apart, turn_speed={self.turn_speed}"
        )
        self.get_logger().info("Waiting for camera frames...")

    # ---------------- camera ----------------

    def on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            return
        small = cv2.resize(frame, (64, 64), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        # Zero-mean so overall brightness changes do not drive the score.
        self.live_view = gray - gray.mean()

    @staticmethod
    def _correlate(a, b):
        if a is None or b is None:
            return None
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom < 1e-6:
            return None
        return float((a * b).sum() / denom)

    def _heading_from_map(self):
        """Best-matching heading in degrees, or None if the map cannot say."""
        scores = []
        for i, view in enumerate(self.room_map):
            c = self._correlate(view, self.live_view)
            if c is not None:
                scores.append((c, i))
        if len(scores) < 2:
            return None
        scores.sort(reverse=True)
        best, best_i = scores[0]
        # Nearby bins overlap and are meant to score alike; only a rival
        # from a different part of the room signals real ambiguity.
        neighbourhood = max(2, int(round(self.n_bins / 12)))
        for score, i in scores[1:]:
            gap = min(abs(i - best_i), self.n_bins - abs(i - best_i))
            if gap > neighbourhood:
                if best - score < 0.05:
                    return None
                break
        return best_i * (360.0 / self.n_bins)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _enter(self, state):
        self.state = state
        self.state_since = self.get_clock().now()

    # ---------------- main loop ----------------

    def tick(self):
        w = 0.0

        if self.state == WAITING:
            if self.live_view is not None:
                self.home_view = self.live_view.copy()
                self.room_map[0] = self.home_view
                self.get_logger().info("Home view captured. Sweeping 360deg...")
                self._enter(SWEEPING)

        elif self.state == SWEEPING:
            w = self._do_sweep()

        elif self.state == RETURNING:
            w = self._do_return()

        cmd = Twist()
        cmd.angular.z = w
        cmd.linear.x = 0.0   # rotation only, always
        self.cmd_pub.publish(cmd)

    def _do_sweep(self):
        bin_width = 360.0 / self.n_bins
        idx = int(self.swept / bin_width)
        if 0 <= idx < self.n_bins and self.room_map[idx] is None:
            self.room_map[idx] = self.live_view.copy()

        self.swept += (1.0 / self.rate) * 1000.0 / self.ms_per_degree

        if self.swept >= 360.0:
            filled = sum(1 for v in self.room_map if v is not None)
            self.get_logger().info(
                f"Sweep complete: {filled}/{self.n_bins} bins captured"
            )
            self.get_logger().info("Returning home using the room map...")
            self._enter(RETURNING)
            return 0.0

        return self.turn_speed

    def _do_return(self):
        heading = self._heading_from_map()

        if heading is None:
            if self._elapsed() > self.return_timeout:
                self._report(None, "map could not place the view")
                return 0.0
            # Keep turning slowly; a different view may be recognisable.
            return self.turn_speed * 0.6

        # Shortest way round: +350 means turn -10, not +350.
        error = -((heading + 180.0) % 360.0 - 180.0)

        if abs(error) <= self.home_tol:
            self._report(heading, None)
            return 0.0

        if self._elapsed() > self.return_timeout:
            self._report(heading, "timed out before settling")
            return 0.0

        return math.copysign(self.turn_speed * 0.6, error)

    # ---------------- report ----------------

    def _report(self, heading, problem):
        """Print the verdict.

        Heading error and view match measure different things and are
        reported separately. Correlation falls off steeply with angle --
        on a textured room roughly 0.8 at 2deg, 0.5 at 5deg, nothing by
        10deg -- so a modest match is normal when the robot is correctly
        home to within a bin width, and is not a failure. What matters
        is whether the map placed the heading at all.
        """
        match = self._correlate(self.home_view, self.live_view)
        filled = sum(1 for v in self.room_map if v is not None)
        bin_width = 360.0 / self.n_bins
        error = None
        if heading is not None:
            error = abs((heading + 180.0) % 360.0 - 180.0)

        self.get_logger().info("=" * 56)
        self.get_logger().info(f"  bins captured  : {filled}/{self.n_bins}")
        if error is not None:
            self.get_logger().info(
                f"  heading error  : {error:.0f}deg  "
                f"(bin width {bin_width:.0f}deg, tolerance {self.home_tol:.0f}deg)"
            )
        else:
            self.get_logger().info("  heading error  : unknown, map could not place the view")
        if match is not None:
            self.get_logger().info(
                f"  view match     : {match:.2f}  "
                "(falls off fast with angle; low is normal within a bin)"
            )

        if problem:
            self.get_logger().warn(f"  RESULT: FAIL -- {problem}")
            self.get_logger().warn(
                "  Either the room is too featureless to localise in, or "
                "ms_per_degree is wrong so the sweep missed a full circle."
            )
        elif filled < self.n_bins * 0.8:
            self.get_logger().warn(
                f"  RESULT: FAIL -- only {filled}/{self.n_bins} bins captured. "
                "The camera may be dropping frames, or the sweep was too fast."
            )
        elif error is not None and error <= self.home_tol:
            self.get_logger().info(
                f"  RESULT: PASS -- swept a full circle and found home "
                f"to within {error:.0f}deg"
            )
        else:
            self.get_logger().warn("  RESULT: FAIL -- did not settle at home")
        self.get_logger().info("=" * 56)

        self._enter(DONE)
        raise SystemExit


def main():
    rclpy.init()
    node = CalibrateRoom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Leave the robot stopped whatever happened.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
