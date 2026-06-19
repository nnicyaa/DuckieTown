from typing import Optional

from tasks.convoying.packages.follow_enum import (
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
    Shared convoy controller for simulation and real robot.

    Normal mode:
        lane following controls steering
        leader distance controls speed

    Red-line / crossroad mode:
        if the leader is going straight  -> follow only the leader truck for steering
        if the leader is turning         -> use lane following to execute the turn smoothly
        (lane following is only consulted for steering during a turn; the rest
        of the time in this mode the leader's bbox position controls steering)
    """

    def __init__(
        self,
        close_multiplier: float = 0.05,
        good_multiplier: float = 0.35,
        far_multiplier: float = 0.60,
        max_speed: float = 0.13,
        lost_grace_frames: int = 35,
        lost_grace_multiplier: float = 0.25,

        # Leader-only steering settings.
        leader_steering_gain: float = 1.25,
        leader_max_steer_ratio: float = 0.95,
        leader_steering_sign: float = 1.0,

        # Leader-turn detection settings (used only while lane_disabled=True).
        # The leader's center-x error is smoothed (EMA) to avoid noise causing
        # false turn detections. Hysteresis (different enter/exit thresholds
        # plus frame counts) prevents flapping between leader-only and
        # lane-following steering near the threshold.
        turn_error_enter_threshold: float = 0.22,
        turn_error_exit_threshold: float = 0.10,
        turn_enter_frames: int = 3,
        turn_exit_frames: int = 5,
        turn_error_smoothing: float = 0.4,
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

        self.turn_error_enter_threshold = turn_error_enter_threshold
        self.turn_error_exit_threshold = turn_error_exit_threshold
        self.turn_enter_frames = turn_enter_frames
        self.turn_exit_frames = turn_exit_frames
        self.turn_error_smoothing = turn_error_smoothing

        self._had_target = False
        self._lost_frames = 0
        self._last_distance_state = LOST

        # Leader-turn detection state.
        self._smoothed_error: Optional[float] = None
        self._leader_is_turning = False
        self._above_threshold_frames = 0
        self._below_threshold_frames = 0

    def decide(
        self,
        target: TargetInfo,
        lane_left: float,
        lane_right: float,
        image_width: Optional[int] = None,
        lane_disabled: bool = False,
    ) -> ConvoyCommand:
        if target.found and target.distance_state != LOST:
            self._had_target = True
            self._lost_frames = 0
            self._last_distance_state = target.distance_state

            if target.distance_state == TOO_CLOSE:
                self._reset_turn_state()
                return self._stop("target_too_close_stop")

            if lane_disabled:
                leader_is_turning = self._update_turn_detection(
                    target=target,
                    image_width=image_width,
                )

                if leader_is_turning:
                    # Leader is turning: use lane following to make the turn
                    # smooth instead of chasing the leader's swinging bbox.
                    return self._scale_for_distance(
                        target=target,
                        lane_left=lane_left,
                        lane_right=lane_right,
                        reason_suffix="red_line_leader_turning_lane_following",
                    )

                return self._leader_only_command(
                    target=target,
                    image_width=image_width,
                )

            # Not in red-line mode -- reset turn state so we start clean
            # the next time we enter a red-line window.
            self._reset_turn_state()

            # Normal lane-following behavior.
            if target.distance_state == CLOSE:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.close_multiplier,
                    "target_close_slow_down_lane_following",
                )

            if target.distance_state == GOOD:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.good_multiplier,
                    "good_distance_lane_following",
                )

            if target.distance_state == FAR:
                return self._scale(
                    lane_left,
                    lane_right,
                    self.far_multiplier,
                    "target_far_speed_up_lane_following",
                )

        # Target is lost.
        self._lost_frames += 1

        # Important:
        # If lane is disabled because of red line, do NOT continue lane following.
        # If leader is lost at crossroad, stop.
        if lane_disabled:
            self._reset_turn_state()
            return self._stop("lane_disabled_target_lost_stop")

        self._reset_turn_state()

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

    def _update_turn_detection(
        self,
        target: TargetInfo,
        image_width: Optional[int],
    ) -> bool:
        """
        Tracks whether the leader appears to be turning, using a smoothed
        center-x error with hysteresis so the decision doesn't flicker.

        Returns True if we should use lane following this frame, False if
        we should use leader-only steering this frame.
        """
        if image_width is None or image_width <= 0 or target.center_x is None:
            # No usable signal this frame -- keep previous decision rather
            # than guessing.
            return self._leader_is_turning

        raw_error = self._target_center_error(target, image_width)

        if self._smoothed_error is None:
            self._smoothed_error = raw_error
        else:
            alpha = self.turn_error_smoothing
            self._smoothed_error = (alpha * raw_error) + (1.0 - alpha) * self._smoothed_error

        magnitude = abs(self._smoothed_error)

        if not self._leader_is_turning:
            if magnitude >= self.turn_error_enter_threshold:
                self._above_threshold_frames += 1
                self._below_threshold_frames = 0
            else:
                self._above_threshold_frames = 0

            if self._above_threshold_frames >= self.turn_enter_frames:
                self._leader_is_turning = True
                self._above_threshold_frames = 0
        else:
            if magnitude <= self.turn_error_exit_threshold:
                self._below_threshold_frames += 1
                self._above_threshold_frames = 0
            else:
                self._below_threshold_frames = 0

            if self._below_threshold_frames >= self.turn_exit_frames:
                self._leader_is_turning = False
                self._below_threshold_frames = 0

        return self._leader_is_turning

    def _reset_turn_state(self) -> None:
        self._smoothed_error = None
        self._leader_is_turning = False
        self._above_threshold_frames = 0
        self._below_threshold_frames = 0

    def _scale_for_distance(
        self,
        target: TargetInfo,
        lane_left: float,
        lane_right: float,
        reason_suffix: str,
    ) -> ConvoyCommand:
        if target.distance_state == CLOSE:
            multiplier = self.close_multiplier
        elif target.distance_state == GOOD:
            multiplier = self.good_multiplier
        elif target.distance_state == FAR:
            multiplier = self.far_multiplier
        else:
            return self._stop("red_line_turn_invalid_distance_state")

        return self._scale(lane_left, lane_right, multiplier, reason_suffix)

    def _leader_only_command(
        self,
        target: TargetInfo,
        image_width: Optional[int],
    ) -> ConvoyCommand:
        if image_width is None or image_width <= 0 or target.center_x is None:
            return self._stop("leader_only_no_target_center_stop")

        error = self._target_center_error(target, image_width)
        error *= self.leader_steering_sign

        # Distance still controls forward speed.
        if target.distance_state == CLOSE:
            base_speed = self.max_speed * 0.30
            reason = "red_line_leader_only_close_slow_down"
        elif target.distance_state == GOOD:
            base_speed = self.max_speed * 0.70
            reason = "red_line_leader_only_good_distance"
        elif target.distance_state == FAR:
            base_speed = self.max_speed * 0.95
            reason = "red_line_leader_only_far_speed_up"
        else:
            return self._stop("red_line_leader_only_invalid_distance_state")

        max_steer = self.max_speed * self.leader_max_steer_ratio
        steering = error * self.leader_steering_gain * self.max_speed
        steering = self._clamp_range(steering, -max_steer, max_steer)

        # Differential drive:
        # left slower + right faster = turn left.
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

        # Positive error = leader is left of image center.
        # Negative error = leader is right of image center.
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