from typing import Optional

from tasks.convoying.packages.follow_types import (
    TargetInfo,
    ConvoyCommand,
    FAR,
    GOOD,
    CLOSE,
    TOO_CLOSE,
    LOST,
)

_IDEAL_BOTTOM_RATIO = 0.58
_NARROW_WIDTH_RATIO = 0.12


class ConvoyController:
    """
    Shared convoy controller for simulation and real robot.

    Normal mode:
        lane following controls steering
        leader distance controls speed

    Red-line / crossroad mode:
        lane following is disabled
        robot follows only the leader truck for steering
    """

    def __init__(
        self,
        close_multiplier: float = 0.05,
        good_multiplier: float = 0.75,
        far_multiplier: float = 1.20,
        max_speed: float = 0.28,
        lost_grace_frames: int = 35,
        lost_grace_multiplier: float = 0.25,

        # Leader-only steering settings.
        leader_steering_gain: float = 1.25,
        leader_max_steer_ratio: float = 0.95,
        leader_steering_sign: float = 1.0,
    ):
        self.close_multiplier = close_multiplier
        self.good_multiplier = good_multiplier
        self.far_multiplier = far_multiplier
        self.max_speed = max_speed

        self.lost_grace_frames = lost_grace_frames
        self.lost_grace_multiplier = lost_grace_multiplier

        self.leader_steering_gain = leader_steering_gain
        self.leader_max_steer_ratio = leader_max_steer_ratio
        self.leader_steering_sign = leader_steering_sign

        self._had_target = False
        self._lost_frames = 0
        self._last_distance_state = LOST

    def decide(
        self,
        target: TargetInfo,
        lane_left: float,
        lane_right: float,
        image_width: Optional[int] = None,
        image_height: Optional[int] = None,
        lane_disabled: bool = False,
    ) -> ConvoyCommand:
        if target.found and target.distance_state != LOST:
            self._had_target = True
            self._lost_frames = 0
            self._last_distance_state = target.distance_state

            if target.distance_state == TOO_CLOSE:
                return self._stop("target_too_close_stop")

            if lane_disabled:
                return self._leader_only_command(
                    target=target,
                    image_width=image_width,
                    image_height=image_height,
                )

            if target.distance_state == CLOSE:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.close_multiplier,
                    "target_close_slow_down_lane_following",
                )

            multiplier = self._follow_multiplier(target, image_height, image_width)
            if target.distance_state == FAR:
                multiplier = max(multiplier, self.far_multiplier)

            reason = (
                "target_far_speed_up_lane_following"
                if target.distance_state == FAR
                else "good_distance_lane_following"
            )
            return self._scale(lane_left, lane_right, multiplier, reason)

        self._lost_frames += 1

        if lane_disabled:
            return self._stop("lane_disabled_target_lost_stop")

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

        if self._had_target and self._lost_frames <= self.lost_grace_frames:
            return self._scale(
                lane_left,
                lane_right,
                self.lost_grace_multiplier,
                "target_temporarily_lost_keep_lane_slowly",
            )

        return self._stop("target_lost_stop")

    def _follow_multiplier(
        self,
        target: TargetInfo,
        image_height: Optional[int],
        image_width: Optional[int],
    ) -> float:
        """
        Continuous catch-up gain from leader position and apparent size.
        Ramps from good_multiplier at ideal follow distance up to far_multiplier
        when the leader is small or high in the frame.
        """
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

    def _leader_only_command(
        self,
        target: TargetInfo,
        image_width: Optional[int],
        image_height: Optional[int],
    ) -> ConvoyCommand:
        if image_width is None or image_width <= 0 or target.center_x is None:
            return self._stop("leader_only_no_target_center_stop")

        error = self._target_center_error(target, image_width)
        error *= self.leader_steering_sign

        if target.distance_state == CLOSE:
            base_speed = self.max_speed * 0.30
            reason = "red_line_leader_only_close_slow_down"
        else:
            multiplier = self._follow_multiplier(target, image_height, image_width)
            if target.distance_state == FAR:
                multiplier = max(multiplier, self.far_multiplier)
            speed_frac = min(1.0, multiplier / self.far_multiplier) if self.far_multiplier > 0 else 0.0
            base_speed = self.max_speed * speed_frac
            reason = (
                "red_line_leader_only_far_speed_up"
                if target.distance_state == FAR
                else "red_line_leader_only_good_distance"
            )

        max_steer = self.max_speed * self.leader_max_steer_ratio
        steering = error * self.leader_steering_gain * self.max_speed
        steering = self._clamp_range(steering, -max_steer, max_steer)

        left_speed = base_speed - steering
        right_speed = base_speed + steering

        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)

        return ConvoyCommand(
            should_move=left_speed > 0.0 or right_speed > 0.0,
            left_speed=left_speed,
            right_speed=right_speed,
            speed_multiplier=base_speed / self.max_speed if self.max_speed > 0 else 0.0,
            reason=reason,
        )

    def _target_center_error(
        self,
        target: TargetInfo,
        image_width: int,
    ) -> float:
        half_width = image_width / 2.0
        return (half_width - float(target.center_x)) / half_width

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
        return self._clamp_range(value, 0.0, self.max_speed)

    @staticmethod
    def _clamp_range(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(float(value), maximum))
