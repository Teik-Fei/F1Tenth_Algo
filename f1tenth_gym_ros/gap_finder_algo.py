import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

MAX_RANGE         = 8.0    # look ahead distance
BUBBLE_THRESHOLD  = 1.5    # zero a safety bubble around any point closer than this
SAFETY_RADIUS     = 0.35   # radius of safety bubble around near points
MIN_GAP_BEAMS     = 30     # minimum gap width in beams to consider valid
FOV_TRIM_DEG      = 70     # ignore points outside of this forward-facing cone
STEER_SMOOTH      = 0.25   # steering gain
MAX_STEER         = 0.4189 # max steering angle
SPEED_MAX         = 55.0   # max speed when travelling straight
SPEED_MIN         = 2.0    # speed when cornering
STRAIGHT_THRESH   = 0.20   # if steer angle below this, speed = max
FWRD_CLEAR_DEG    = 25     # forward clearance window
SIDE_CLEAR_DEG_LO = 30     # side clearance start angle
SIDE_CLEAR_DEG_HI = 70     # side clearance end angle
SIDE_BRAKE_DIST   = 0.8    # brake if a wall is closer than this in the side sectors
SIDE_BRAKE_SPEED  = 4.0    # speed cap when a wall is too close in the side sectors
TURN_PERSIST      = 0.30   # turn-continuation bias
CORNER_EXP        = 3.0    # exponent: higher = sharper speed drop at corner entry


class GapFinder(Node):
    """ROS 2 node — Follow-the-Gap race-track follower."""

    def __init__(self):
        super().__init__('gap_finder')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self._prev_steer = 0.0
        self.get_logger().info('GapFinder (race) node started.')

    def preprocess(self, ranges: np.ndarray) -> np.ndarray:
        proc = np.array(ranges, dtype=np.float32)
        proc = np.nan_to_num(proc, nan=0.0, posinf=MAX_RANGE, neginf=0.0)
        np.clip(proc, 0.0, MAX_RANGE, out=proc)
        return proc

    def trim_fov(self, ranges: np.ndarray, angle_min: float,
                  angle_inc: float) -> np.ndarray:
        n = len(ranges)
        fov_rad = np.deg2rad(FOV_TRIM_DEG)
        centre = int(round(-angle_min / angle_inc))
        half = max(1, int(fov_rad / angle_inc))
        lo = max(0, centre - half)
        hi = min(n - 1, centre + half)
        trimmed = np.zeros(n, dtype=np.float32)
        trimmed[lo: hi + 1] = ranges[lo: hi + 1]
        return trimmed

    def apply_bubbles(self, ranges: np.ndarray, angle_inc: float) -> np.ndarray:
        """Zero a safety arc around every beam closer than BUBBLE_THRESHOLD."""
        masked = ranges.copy()
        near_indices = np.where((masked > 0) & (masked < BUBBLE_THRESHOLD))[0]
        for idx in near_indices:
            dist = masked[idx]
            if dist <= 0:
                continue
            half_angle = np.arctan2(SAFETY_RADIUS, dist)
            half_beams = max(1, int(half_angle / angle_inc))
            lo = max(0, idx - half_beams)
            hi = min(len(masked) - 1, idx + half_beams)
            masked[lo: hi + 1] = 0.0
        return masked

    def find_best_gap(self, ranges: np.ndarray, angle_min: float,
                       angle_inc: float, prev_steer: float):
        """
        Score = max_depth x width x persist_weight
          persist_weight = exp(+TURN_PERSIST x sign(prev_steer) x gap_centre_angle)
        """
        n = len(ranges)
        free = ranges > 0

        gaps = []        
        run_start = None
        for i in range(n):
            if free[i]:
                if run_start is None:
                    run_start = i
            else:
                if run_start is not None:
                    gaps.append((run_start, i - 1))
                    run_start = None
        if run_start is not None:
            gaps.append((run_start, n - 1))

        # filter by minimum width
        valid_gaps = [(s, e) for s, e in gaps if (e - s + 1) >= MIN_GAP_BEAMS]

        if not valid_gaps:
            # fallback: widest gap regardless
            if not gaps:
                return 0, n - 1
            valid_gaps = [max(gaps, key=lambda g: g[1] - g[0])]

        # sign of current steering direction (+1 left, -1 right, 0 neutral)
        steer_sign = np.sign(prev_steer) if abs(prev_steer) > 0.05 else 0.0

        global_max_idx = int(np.argmax(ranges))
        for s, e in valid_gaps:
            if s <= global_max_idx <= e:
                return (s, e)

        best_gap = None
        best_score = -1.0
        for s, e in valid_gaps:
            width    = e - s + 1
            depth    = float(np.max(ranges[s: e + 1]))
            mid_ang  = angle_min + ((s + e) / 2.0) * angle_inc
            persist_w = np.exp(TURN_PERSIST * steer_sign * mid_ang)
            score = depth * width * persist_w
            if score > best_score:
                best_score = score
                best_gap = (s, e)

        return best_gap

    def best_heading(self, ranges: np.ndarray, start: int, end: int,
                      angle_min: float, angle_inc: float) -> float:
        """
        find the best heading in the gap as the range-weighted mean angle of the beams in the gap
        """
        window  = ranges[start: end + 1]
        indices = np.arange(start, end + 1, dtype=np.float32)
        weights = window ** 2         
        total_w = weights.sum()
        if total_w < 1e-6:
            idx = (start + end) / 2.0
        else:
            idx = float(np.dot(weights, indices) / total_w)
        return angle_min + idx * angle_inc

    def side_clearance(self, ranges: np.ndarray, angle_min: float,
                        angle_inc: float) -> float:
        """
        measure the closest obstacle in the side sectors (excluding the forward cone)
        """
        n = len(ranges)
        centre = int(round(-angle_min / angle_inc))
        lo_beam = max(0, centre - int(np.deg2rad(SIDE_CLEAR_DEG_HI) / angle_inc))
        hi_beam = min(n - 1, centre + int(np.deg2rad(SIDE_CLEAR_DEG_HI) / angle_inc))
        fwd_lo  = max(0, centre - int(np.deg2rad(SIDE_CLEAR_DEG_LO) / angle_inc))
        fwd_hi  = min(n - 1, centre + int(np.deg2rad(SIDE_CLEAR_DEG_LO) / angle_inc))
        # combine left and right side sectors (excluding the forward cone)
        left  = ranges[lo_beam: fwd_lo]
        right = ranges[fwd_hi + 1: hi_beam + 1]
        side  = np.concatenate([left, right])
        valid = side[(side > 0)]
        return float(valid.min()) if valid.size > 0 else MAX_RANGE

    def forward_clearance(self, ranges: np.ndarray, angle_min: float,
                           angle_inc: float) -> float:
        n = len(ranges)
        fwd_rad = np.deg2rad(FWRD_CLEAR_DEG)
        centre  = int(round(-angle_min / angle_inc))
        half    = max(1, int(fwd_rad / angle_inc))
        lo = max(0, centre - half)
        hi = min(n - 1, centre + half)
        sector = ranges[lo: hi + 1]
        valid  = sector[sector > 0]
        return float(valid.min()) if valid.size > 0 else MAX_RANGE

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # preprocess
        proc = self.preprocess(np.array(msg.ranges, dtype=np.float32))

        # trim FOV
        proc = self.trim_fov(proc, angle_min, angle_inc)

        # measure forward and side clearance before bubbles wipe near beams
        fwd_clear  = self.forward_clearance(proc, angle_min, angle_inc)
        side_clear = self.side_clearance(proc, angle_min, angle_inc)

        # multi-bubble
        proc = self.apply_bubbles(proc, angle_inc)

        # find best gap (pass prev_steer for turn persistence)
        gap_start, gap_end = self.find_best_gap(proc, angle_min, angle_inc, self._prev_steer)

        # best heading (range-weighted mean angle in gap)
        target_angle = self.best_heading(
            proc, gap_start, gap_end, angle_min, angle_inc)

        # steering with exponential smoothing
        raw_steer = float(np.clip(target_angle, -MAX_STEER, MAX_STEER))
        steer = (STEER_SMOOTH * self._prev_steer
                 + (1.0 - STEER_SMOOTH) * raw_steer)
        steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))
        self._prev_steer = steer

        # adaptive speed
        # clearance-based: squared curve — brakes hard as forward wall approaches
        clear_ratio = (min(fwd_clear / MAX_RANGE, 1.0)) ** 2
        speed_clear = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * clear_ratio

        # steer-based: full speed when straight, sharp cosine taper only in corners
        # steer below STRAIGHT_THRESH → treat as straight 
        abs_steer = abs(steer)
        if abs_steer < STRAIGHT_THRESH:
            speed_steer = SPEED_MAX
        else:
            # remap steer from [STRAIGHT_THRESH, MAX_STEER] → [0, 1]
            corner_ratio = min((abs_steer - STRAIGHT_THRESH) / (MAX_STEER - STRAIGHT_THRESH), 1.0)
            speed_steer = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * (np.cos(corner_ratio * np.pi / 2) ** CORNER_EXP)

        # side-proximity brake: hard cap when a corner wall is closing in
        speed_side = SIDE_BRAKE_SPEED if side_clear < SIDE_BRAKE_DIST else SPEED_MAX

        speed = float(max(SPEED_MIN, min(speed_clear, speed_steer, speed_side)))

        # publish
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GapFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()