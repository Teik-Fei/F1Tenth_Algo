import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


# ── tuneable parameters ────────────────────────────────────────────────────────
MAX_RANGE         = 8.0    # [m]  clip distant readings — long lookahead
BUBBLE_THRESHOLD  = 1.5    # [m]  apply safety bubbles to all points closer than this
SAFETY_RADIUS     = 0.35   # [m]  physical radius zeroed around each near point
MIN_GAP_BEAMS     = 30     # minimum gap width in beams to consider valid
FOV_TRIM_DEG      = 70     # [°]  half-FOV kept either side of forward
STEER_SMOOTH      = 0.25   # exponential smoothing
MAX_STEER         = 0.4189 # [rad] ≈ 24°  physical steering limit
SPEED_MAX         = 15.0   # [m/s] straight-line cap
SPEED_MIN         = 2.0    # [m/s] tight-corner floor
STRAIGHT_THRESH   = 0.06   # [rad] steer below this → full SPEED_MAX (ignore tiny corrections)
FWRD_CLEAR_DEG    = 25     # [°]  forward clearance cone
SIDE_CLEAR_DEG_LO = 30     # [°]  side proximity window start
SIDE_CLEAR_DEG_HI = 70     # [°]  side proximity window end
SIDE_BRAKE_DIST   = 0.8    # [m]  only brake when wall is truly close
SIDE_BRAKE_SPEED  = 4.0    # [m/s] side-wall speed cap
TURN_PERSIST      = 0.30   # turn-continuation bias
CORNER_EXP        = 3.0    # exponent: higher = sharper speed drop at corner entry
# ──────────────────────────────────────────────────────────────────────────────


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

    # ── step 1: preprocess ────────────────────────────────────────────────────

    def _preprocess(self, ranges: np.ndarray) -> np.ndarray:
        proc = np.array(ranges, dtype=np.float32)
        proc = np.nan_to_num(proc, nan=0.0, posinf=MAX_RANGE, neginf=0.0)
        np.clip(proc, 0.0, MAX_RANGE, out=proc)
        return proc

    # ── step 2: FOV trim ─────────────────────────────────────────────────────

    def _trim_fov(self, ranges: np.ndarray, angle_min: float,
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

    # ── step 3: multi-point safety bubbles ───────────────────────────────────

    def _apply_bubbles(self, ranges: np.ndarray, angle_inc: float) -> np.ndarray:
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

    # ── step 4: find & score all gaps ────────────────────────────────────────

    def _find_best_gap(self, ranges: np.ndarray, angle_min: float,
                       angle_inc: float, prev_steer: float):
        """
        Enumerate free-space runs and return (start, end) of the best gap.

        Score = max_depth × width × persist_weight
          persist_weight = exp(+TURN_PERSIST × sign(prev_steer) × gap_centre_angle)

        Using max_depth (not mean) means the open track corridor—which
        always has the deepest readings—wins over shallow wide openings.
        persist_weight gives a small continuing bonus so the car commits
        to a corner once it starts turning instead of flip-flopping.
        """
        n = len(ranges)
        free = ranges > 0

        gaps = []          # (start, end)
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

        # Primary criterion: the gap that CONTAINS the globally deepest beam.
        # On a race track the open corridor always has the farthest visible
        # point — this correctly handles intersections where a side opening
        # is wide but the track corridor is deeper.
        global_max_idx = int(np.argmax(ranges))
        for s, e in valid_gaps:
            if s <= global_max_idx <= e:
                return (s, e)

        # Fallback (global max landed in a bubble): score by depth × width × persist
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

    # ── step 5: range-weighted target angle ───────────────────────────────────

    def _best_heading(self, ranges: np.ndarray, start: int, end: int,
                      angle_min: float, angle_inc: float) -> float:
        """
        Compute the range-weighted mean angle of the gap.
        This naturally points the heading toward the deepest, widest
        part of the gap — i.e., the inside of the upcoming corner.
        """
        window  = ranges[start: end + 1]
        indices = np.arange(start, end + 1, dtype=np.float32)
        weights = window ** 2          # square-weight to emphasise deep beams
        total_w = weights.sum()
        if total_w < 1e-6:
            idx = (start + end) / 2.0
        else:
            idx = float(np.dot(weights, indices) / total_w)
        return angle_min + idx * angle_inc

    # ── side proximity check ──────────────────────────────────────────────────

    def _side_clearance(self, ranges: np.ndarray, angle_min: float,
                        angle_inc: float) -> float:
        """Return the minimum range in the lateral ±[LO, HI] sectors.
        Detects corner walls that haven't swung into the forward cone yet."""
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

    # ── step 7: forward clearance ─────────────────────────────────────────────

    def _forward_clearance(self, ranges: np.ndarray, angle_min: float,
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

    # ── main callback ─────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # 1 — preprocess
        proc = self._preprocess(np.array(msg.ranges, dtype=np.float32))

        # 2 — trim FOV
        proc = self._trim_fov(proc, angle_min, angle_inc)

        # measure forward and side clearance before bubbles wipe near beams
        fwd_clear  = self._forward_clearance(proc, angle_min, angle_inc)
        side_clear = self._side_clearance(proc, angle_min, angle_inc)

        # 3 — multi-bubble
        proc = self._apply_bubbles(proc, angle_inc)

        # 4 — find best gap (pass prev_steer for turn persistence)
        gap_start, gap_end = self._find_best_gap(proc, angle_min, angle_inc, self._prev_steer)

        # 5 — best heading (range-weighted mean angle in gap)
        target_angle = self._best_heading(
            proc, gap_start, gap_end, angle_min, angle_inc)

        # 6 — steering with exponential smoothing
        raw_steer = float(np.clip(target_angle, -MAX_STEER, MAX_STEER))
        steer = (STEER_SMOOTH * self._prev_steer
                 + (1.0 - STEER_SMOOTH) * raw_steer)
        steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))
        self._prev_steer = steer

        # 7 — adaptive speed
        #   a) clearance-based: squared curve — brakes hard as forward wall approaches
        clear_ratio = (min(fwd_clear / MAX_RANGE, 1.0)) ** 2
        speed_clear = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * clear_ratio

        #   b) steer-based: full speed when straight, sharp cosine taper only in corners
        #      steer below STRAIGHT_THRESH → treat as straight (ignore tiny tracking corrections)
        abs_steer = abs(steer)
        if abs_steer < STRAIGHT_THRESH:
            speed_steer = SPEED_MAX
        else:
            # remap steer from [STRAIGHT_THRESH, MAX_STEER] → [0, 1]
            corner_ratio = min((abs_steer - STRAIGHT_THRESH) / (MAX_STEER - STRAIGHT_THRESH), 1.0)
            speed_steer = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * (np.cos(corner_ratio * np.pi / 2) ** CORNER_EXP)

        #   c) side-proximity brake: hard cap when a corner wall is closing in
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
