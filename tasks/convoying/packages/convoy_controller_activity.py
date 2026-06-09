from tasks.convoying.packages.follow_types import (
    TargetInfo,
    ConvoyCommand,
    FAR,
    GOOD,
    CLOSE,
    TOO_CLOSE,
    LOST,
)


class ConvoyController:
    """
    Lane-following controls steering.
    Convoying controls speed.

    Important safety rule:
    If target is lost while it was CLOSE or TOO_CLOSE, stop immediately.
    Grace movement is allowed only when target was FAR or GOOD.
    """

    def __init__(
            self,
            close_multiplier: float = 0.05,
            good_multiplier: float = 0.35,
            far_multiplier: float = 0.60,
            max_speed: float = 0.13,
            lost_grace_frames: int = 35,
            lost_grace_multiplier: float = 0.25,
    ):
        self.close_multiplier = close_multiplier
        self.good_multiplier = good_multiplier
        self.far_multiplier = far_multiplier
        self.max_speed = max_speed

        self.lost_grace_frames = lost_grace_frames
        self.lost_grace_multiplier = lost_grace_multiplier

        self._had_target = False
        self._lost_frames = 0
        self._last_distance_state = LOST

    def decide(
        self,
        target: TargetInfo,
        lane_left: float,
        lane_right: float,
    ) -> ConvoyCommand:
        if target.found and target.distance_state != LOST:
            self._had_target = True
            self._lost_frames = 0
            self._last_distance_state = target.distance_state

            if target.distance_state == TOO_CLOSE:
                return self._stop("target_too_close_stop")

            if target.distance_state == CLOSE:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.close_multiplier,
                    "target_close_slow_down",
                )

            if target.distance_state == GOOD:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.good_multiplier,
                    "good_distance_following",
                )

            if target.distance_state == FAR:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.far_multiplier,
                    "target_far_speed_up",
                )

        # Target is lost.
        self._lost_frames += 1

        # If we lose target while close, stop. Do not keep moving into it.
        if self._last_distance_state == TOO_CLOSE:
            return self._stop("target_lost_after_too_close_emergency_stop")

        if self._last_distance_state == CLOSE and self._lost_frames <= 15:
            return self._scale(
                lane_left,
                lane_right,
                0.25,
                "target_lost_after_close_creep_forward",
            )

        if self._last_distance_state == CLOSE:
            return self._stop("target_lost_after_close_stop")

        # If target was far/good, allow short lane-following grace through turn.
        if self._had_target and self._lost_frames <= self.lost_grace_frames:
            return self._scale(
                lane_left,
                lane_right,
                self.lost_grace_multiplier,
                "target_temporarily_lost_keep_lane_slowly",
            )

        return self._stop("target_lost_stop")

    def _scale(
        self,
        lane_left: float,
        lane_right: float,
        multiplier: float,
        reason: str,
    ) -> ConvoyCommand:
        left_speed = self._clamp(lane_left * multiplier)
        right_speed = self._clamp(lane_right * multiplier)

        return ConvoyCommand(
            should_move=left_speed > 0.0 or right_speed > 0.0,
            left_speed=left_speed,
            right_speed=right_speed,
            speed_multiplier=multiplier,
            reason=reason,
        )

    def _stop(self, reason: str) -> ConvoyCommand:
        return ConvoyCommand(
            should_move=False,
            left_speed=0.0,
            right_speed=0.0,
            speed_multiplier=0.0,
            reason=reason,
        )

    def _clamp(self, value: float) -> float:
        return max(0.0, min(float(value), self.max_speed))