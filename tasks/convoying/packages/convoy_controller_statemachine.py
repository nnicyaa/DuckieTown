import time
from typing import Optional

from tasks.convoying.packages.follow_types import (
    TargetInfo,
    ConvoyCommand,
    FAR,
    GOOD,
    CLOSE,
    TOO_CLOSE,
    LOST,
    SEARCH,
    FOLLOWING,
    STOPPED,
    LOST_TARGET,
    TOO_CLOSE_STATE,
)

_IDEAL_BOTTOM_RATIO = 0.58
_NARROW_WIDTH_RATIO = 0.12


class ConvoyController:
    """
    Explicit state-machine convoy controller, driven by the leader's marker
    bracket (see MarkerBracketDetector) rather than the whole truck bbox.

    States:
        SEARCH          no trusted marker yet (or never had one) -- steer
                         using lane following if available and not
                         disabled (red-line gate); otherwise crawl forward
                         with no steering bias, looking.
        FOLLOWING       marker trusted this frame -- steer continuously
                         toward the marker's position, speed from distance.
        TOO_CLOSE_STATE marker very close -- hard stop, distinct from a
                         lost target so it's clear from state alone why the
                         robot stopped.
        STOPPED         marker visible and trusted, but its position/size
                         hasn't changed in stop_detection_timeout seconds --
                         leader is inferred to have stopped. Distinct from
                         LOST_TARGET: here we can still see the leader, it's
                         just not moving.
        LOST_TARGET     marker not found this frame, but we were FOLLOWING
                         recently. Steers using lane following if available
                         and not disabled; otherwise continues in the last
                         known marker-steering direction. Either way, after
                         lost_target_timeout seconds with no marker, drops
                         to SEARCH.

    Steering note: per spec, the natural region-vote design is left/center/
    right with 2/3-majority voting. This implementation instead uses
    continuous proportional steering driven by the marker's centroid
    position -- a deliberate deviation, chosen because discrete region
    voting introduces steering-angle discontinuities at region boundaries,
    which is the same class of issue that caused oscillation in earlier
    iterations of this robot's lane controller. Continuous control fed by a
    clean, gated signal (the marker centroid, only trusted once dot_count
    clears minimum_detected_dots) satisfies the same intent -- smooth,
    noise-resistant steering toward the leader -- without that failure mode.
    """

    def __init__(
        self,
        close_multiplier: float = 0.05,
        good_multiplier: float = 0.75,
        far_multiplier: float = 1.20,
        max_speed: float = 0.28,

        # Steering (continuous, from marker centroid error).
        steering_gain: float = 1.25,
        max_steer_ratio: float = 0.95,
        steering_sign: float = 1.0,

        # SEARCH state behavior.
        search_speed: float = 0.08,

        # LOST_TARGET behavior.
        lost_target_timeout: float = 2.5,    # seconds before LOST_TARGET -> SEARCH
        lost_target_speed_multiplier: float = 0.35,

        # STOPPED detection: leader inferred stationary if marker centroid
        # and size haven't moved more than these tolerances for
        # stop_detection_timeout seconds.
        stop_detection_timeout: float = 1.2,
        stationary_position_tolerance_px: float = 4.0,
        stationary_size_tolerance_ratio: float = 0.05,  # fraction of marker bbox diagonal

        # Resume-from-stop smoothing.
        resume_ramp_seconds: float = 0.8,
    ):
        self.close_multiplier = close_multiplier
        self.good_multiplier = good_multiplier
        self.far_multiplier = far_multiplier
        self.max_speed = max_speed

        self.steering_gain = steering_gain
        self.max_steer_ratio = max_steer_ratio
        self.steering_sign = steering_sign

        self.search_speed = search_speed

        self.lost_target_timeout = lost_target_timeout
        self.lost_target_speed_multiplier = lost_target_speed_multiplier

        self.stop_detection_timeout = stop_detection_timeout
        self.stationary_position_tolerance_px = stationary_position_tolerance_px
        self.stationary_size_tolerance_ratio = stationary_size_tolerance_ratio

        self.resume_ramp_seconds = resume_ramp_seconds

        # State machine.
        self._state = SEARCH

        # Last known steering direction/error, used by LOST_TARGET to keep
        # steering the same way while searching, per spec.
        self._last_steering_error = 0.0
        self._lost_target_since: Optional[float] = None

        # Stationary-leader detection bookkeeping.
        self._stationary_since: Optional[float] = None
        self._last_marker_center: Optional[tuple] = None
        self._last_marker_diagonal: Optional[float] = None

        # Resume-from-stop ramp bookkeeping.
        self._resume_started_at: Optional[float] = None

    def reset(self) -> None:
        self._state = SEARCH
        self._last_steering_error = 0.0
        self._lost_target_since = None
        self._stationary_since = None
        self._last_marker_center = None
        self._last_marker_diagonal = None
        self._resume_started_at = None

    @property
    def state(self) -> str:
        return self._state

    def decide(
        self,
        target: TargetInfo,
        lane_left: float = 0.0,
        lane_right: float = 0.0,
        image_width: Optional[int] = None,
        image_height: Optional[int] = None,
        lane_disabled: bool = False,
    ) -> ConvoyCommand:
        """
        Marker-bracket position is the primary steering signal whenever the
        marker is trusted (FOLLOWING state). lane_left/lane_right are used
        as a fallback steering source in SEARCH and LOST_TARGET -- i.e.
        whenever the marker isn't currently trusted -- so the robot keeps
        driving correctly within its lane while looking for the leader,
        rather than crawling at a fixed speed with no steering input.
        lane_disabled (from the red-line gate) suppresses that fallback the
        same way it suppresses lane following elsewhere in this project --
        if we're in a red-line/crossroad window AND the marker is lost,
        there's no reliable steering source at all, so we stop rather than
        guess.
        """
        now = time.monotonic()
        marker_found = target.found and target.dot_count > 0 and target.marker_center_x is not None

        if marker_found:
            self._lost_target_since = None
            self._update_stationary_tracking(target, now)

            if target.distance_state == TOO_CLOSE:
                self._state = TOO_CLOSE_STATE
                self._resume_started_at = None
                return self._stop("target_too_close_stop")

            if self._is_stationary(now):
                self._state = STOPPED
                self._resume_started_at = None
                return self._stop("leader_stationary_stop")

            # Leader confirmed moving/changing again after a stop -- ramp
            # speed back up smoothly instead of jumping straight to full
            # multiplier, so resuming doesn't look like a speed lurch.
            if self._state == STOPPED:
                self._resume_started_at = now

            self._state = FOLLOWING
            return self._following_command(target, image_width, image_height, now)

        # Marker not found this frame.
        if self._state in (FOLLOWING, STOPPED, TOO_CLOSE_STATE):
            self._state = LOST_TARGET
            self._lost_target_since = now
            self._stationary_since = None

        lane_fallback_available = not lane_disabled and (lane_left != 0.0 or lane_right != 0.0)

        if self._state == LOST_TARGET:
            elapsed = now - self._lost_target_since if self._lost_target_since else 0.0
            if elapsed > self.lost_target_timeout:
                self._state = SEARCH
                self._lost_target_since = None
            elif lane_fallback_available:
                return self._lane_fallback_command(
                    lane_left, lane_right, "lost_target_lane_fallback", LOST_TARGET
                )
            else:
                return self._lost_target_command()

        # SEARCH.
        self._state = SEARCH

        if lane_fallback_available:
            return self._lane_fallback_command(
                lane_left, lane_right, "search_lane_fallback", SEARCH
            )

        if lane_disabled:
            # No marker, no lane signal available (red-line window) -- no
            # reliable steering source at all. Stop rather than guess.
            return self._stop("search_no_marker_no_lane_lane_disabled_stop")

        return self._search_command()

    # -- FOLLOWING -----------------------------------------------------

    def _following_command(
        self,
        target: TargetInfo,
        image_width: Optional[int],
        image_height: Optional[int],
        now: float,
    ) -> ConvoyCommand:
        if image_width is None or image_width <= 0:
            return self._stop("following_no_image_width_stop")

        error = self._marker_center_error(target, image_width)
        error *= self.steering_sign
        self._last_steering_error = error

        multiplier = self._follow_multiplier(target, image_height, image_width)
        if target.distance_state == CLOSE:
            multiplier = self.close_multiplier
            reason = "following_close_slow_down"
        elif target.distance_state == FAR:
            multiplier = max(multiplier, self.far_multiplier)
            reason = "following_far_speed_up"
        else:
            reason = "following_good_distance"

        # Smooth resume after a STOPPED period rather than jumping straight
        # to the target multiplier.
        multiplier = self._apply_resume_ramp(multiplier, now)

        base_speed = self.max_speed * min(1.0, multiplier / max(self.far_multiplier, 1e-6))
        max_steer = self.max_speed * self.max_steer_ratio
        steering = error * self.steering_gain * self.max_speed
        steering = self._clamp_range(steering, -max_steer, max_steer)

        left_speed = self._clamp(base_speed - steering)
        right_speed = self._clamp(base_speed + steering)

        return ConvoyCommand(
            should_move=left_speed > 0.0 or right_speed > 0.0,
            left_speed=left_speed,
            right_speed=right_speed,
            speed_multiplier=multiplier,
            reason=reason,
            state=FOLLOWING,
        )

    def _apply_resume_ramp(self, target_multiplier: float, now: float) -> float:
        if self._resume_started_at is None:
            return target_multiplier

        elapsed = now - self._resume_started_at
        if elapsed >= self.resume_ramp_seconds or self.resume_ramp_seconds <= 0:
            self._resume_started_at = None
            return target_multiplier

        ramp_fraction = elapsed / self.resume_ramp_seconds
        return target_multiplier * ramp_fraction

    def _follow_multiplier(
        self,
        target: TargetInfo,
        image_height: Optional[int],
        image_width: Optional[int],
    ) -> float:
        behind = 0.0

        if target.bottom_y is not None and image_height and image_height > 0:
            bottom_ratio = target.bottom_y / float(image_height)
            behind = max(
                0.0,
                min(1.0, (_IDEAL_BOTTOM_RATIO - bottom_ratio) / _IDEAL_BOTTOM_RATIO),
            )

        if target.bbox is not None and image_width and image_width > 0:
            width_ratio = max(0, target.bbox[2] - target.bbox[0]) / float(image_width)
            narrow = max(
                0.0,
                min(1.0, (_NARROW_WIDTH_RATIO - width_ratio) / _NARROW_WIDTH_RATIO),
            )
            behind = max(behind, narrow)

        return self.good_multiplier + behind * (self.far_multiplier - self.good_multiplier)

    def _marker_center_error(self, target: TargetInfo, image_width: int) -> float:
        half_width = image_width / 2.0
        return (half_width - float(target.marker_center_x)) / half_width

    # -- STOPPED / stationary detection ---------------------------------

    def _update_stationary_tracking(self, target: TargetInfo, now: float) -> None:
        center = (target.marker_center_x, target.marker_center_y)
        diagonal = self._marker_bbox_diagonal(target)

        if self._last_marker_center is None or self._last_marker_diagonal is None:
            self._last_marker_center = center
            self._last_marker_diagonal = diagonal
            self._stationary_since = now
            return

        dx = center[0] - self._last_marker_center[0]
        dy = center[1] - self._last_marker_center[1]
        position_shift = (dx * dx + dy * dy) ** 0.5

        size_shift_ratio = 0.0
        if diagonal is not None and self._last_marker_diagonal:
            size_shift_ratio = abs(diagonal - self._last_marker_diagonal) / self._last_marker_diagonal

        moved = (
            position_shift > self.stationary_position_tolerance_px
            or size_shift_ratio > self.stationary_size_tolerance_ratio
        )

        if moved:
            self._stationary_since = now
        elif self._stationary_since is None:
            self._stationary_since = now

        self._last_marker_center = center
        self._last_marker_diagonal = diagonal

    def _is_stationary(self, now: float) -> bool:
        if self._stationary_since is None:
            return False
        return (now - self._stationary_since) >= self.stop_detection_timeout

    @staticmethod
    def _marker_bbox_diagonal(target: TargetInfo) -> Optional[float]:
        if target.marker_bbox is None:
            return None
        x1, y1, x2, y2 = target.marker_bbox
        w = max(0, x2 - x1)
        h = max(0, y2 - y1)
        return (w * w + h * h) ** 0.5

    # -- Lane fallback (SEARCH / LOST_TARGET, marker not trusted) ---------

    def _lane_fallback_command(
        self,
        lane_left: float,
        lane_right: float,
        reason: str,
        state: str,
    ) -> ConvoyCommand:
        """
        Used by both SEARCH and LOST_TARGET when the marker isn't currently
        trusted but lane servoing is available and not disabled. Scaled by
        lost_target_speed_multiplier in both cases -- deliberately cautious
        (not full speed) since we don't currently know where the leader is,
        only that the lane itself is still trustworthy to drive in.
        """
        left_speed = self._clamp(lane_left * self.lost_target_speed_multiplier)
        right_speed = self._clamp(lane_right * self.lost_target_speed_multiplier)

        return ConvoyCommand(
            should_move=left_speed > 0.0 or right_speed > 0.0,
            left_speed=left_speed,
            right_speed=right_speed,
            speed_multiplier=self.lost_target_speed_multiplier,
            reason=reason,
            state=state,
        )

    # -- LOST_TARGET -----------------------------------------------------

    def _lost_target_command(self) -> ConvoyCommand:
        """
        Keep steering in the last known direction while searching, per
        spec, rather than going straight or stopping outright -- this gives
        the leader's marker a chance to drift back into frame if it was
        only briefly clipped at the edge.
        """
        max_steer = self.max_speed * self.max_steer_ratio
        steering = self._last_steering_error * self.steering_gain * self.max_speed
        steering = self._clamp_range(steering, -max_steer, max_steer)

        base_speed = self.max_speed * self.lost_target_speed_multiplier

        left_speed = self._clamp(base_speed - steering)
        right_speed = self._clamp(base_speed + steering)

        return ConvoyCommand(
            should_move=left_speed > 0.0 or right_speed > 0.0,
            left_speed=left_speed,
            right_speed=right_speed,
            speed_multiplier=self.lost_target_speed_multiplier,
            reason="lost_target_continue_last_direction",
            state=LOST_TARGET,
        )

    # -- SEARCH ------------------------------------------------------------

    def _search_command(self) -> ConvoyCommand:
        """
        Per spec: move slowly, searching. No directional bias here (unlike
        LOST_TARGET) since by the time we reach SEARCH, the last-known
        direction is considered stale.
        """
        return ConvoyCommand(
            should_move=True,
            left_speed=self.search_speed,
            right_speed=self.search_speed,
            speed_multiplier=self.search_speed / self.max_speed if self.max_speed > 0 else 0.0,
            reason="search_no_target_moving_slowly",
            state=SEARCH,
        )

    # -- shared helpers ----------------------------------------------------

    def _stop(self, reason: str) -> ConvoyCommand:
        return ConvoyCommand(
            should_move=False,
            left_speed=0.0,
            right_speed=0.0,
            speed_multiplier=0.0,
            reason=reason,
            state=self._state,
        )

    def _clamp(self, value: float) -> float:
        return self._clamp_range(value, 0.0, self.max_speed)

    @staticmethod
    def _clamp_range(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(float(value), maximum))