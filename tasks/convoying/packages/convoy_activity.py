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

        # Leader-only steering noise reduction.
        # At long range the leader's bbox is small, so its center estimate
        # is noisier relative to the box itself, and that noise gets
        # amplified after normalization by image half-width. Without
        # smoothing/deadband this produces oscillation specifically when
        # the leader is far (matches the FAR-distance symptom).
        leader_error_smoothing: float = 0.3,    # EMA alpha, lower = smoother/slower
        leader_error_deadband: float = 0.04,    # ignore tiny noise around zero
        leader_far_gain_scale: float = 0.6,     # extra damping applied only when FAR

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

        # Short-loss handling (target briefly undetected, e.g. mid-turn
        # occlusion/edge-clipping). These prevent a single missed detection
        # from nuking EMA smoothing state or causing a hard stop, which was
        # producing visible stop/lurch and sharp-correction artifacts that
        # look like oscillation.
        turn_lost_grace_frames: int = 6,
        smoothing_reset_lost_frames: int = 4,

        # Approach-rate speed compensation.
        # Distance-bucket multipliers (close/good/far) only react after the
        # gap to the leader has already changed enough to cross a bucket
        # boundary -- inherently a step behind a leader that's actively
        # accelerating or decelerating. approach_rate (px/frame change in
        # the leader's bbox bottom_y, smoothed in TargetTracker) lets us
        # react to the leader's motion directly: pulling away -> speed up
        # immediately; closing in -> slow down immediately. This is added
        # as a multiplicative adjustment on top of the existing bucket
        # multiplier, not a replacement for it.
        approach_rate_gain: float = 0.012,      # speed adjustment per px/frame of approach_rate
        approach_rate_max_adjust: float = 0.35,  # clamp on the adjustment, as a fraction of base speed
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

        self.leader_error_smoothing = leader_error_smoothing
        self.leader_error_deadband = leader_error_deadband
        self.leader_far_gain_scale = leader_far_gain_scale

        # EMA state for the leader-only steering error (separate from the
        # turn-detection EMA below -- different purposes, can use different
        # smoothing factors).
        self._smoothed_leader_error: Optional[float] = None

        self.turn_error_enter_threshold = turn_error_enter_threshold
        self.turn_error_exit_threshold = turn_error_exit_threshold
        self.turn_enter_frames = turn_enter_frames
        self.turn_exit_frames = turn_exit_frames
        self.turn_error_smoothing = turn_error_smoothing

        self.turn_lost_grace_frames = turn_lost_grace_frames
        self.smoothing_reset_lost_frames = smoothing_reset_lost_frames

        self.approach_rate_gain = approach_rate_gain
        self.approach_rate_max_adjust = approach_rate_max_adjust

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
                        lane_left=lane_left,
                        lane_right=lane_right,
                        reason_suffix="red_line_leader_turning_lane_following",
                        target=target,
                    )

                return self._leader_only_command(
                    target=target,
                    image_width=image_width,
                )

            # Not in red-line mode -- reset turn state so we start clean
            # the next time we enter a red-line window.
            self._reset_turn_state()

            # Normal lane-following behavior.
            # The bucket multiplier (close/good/far) is adjusted by
            # approach_rate so we react to the leader actively pulling away
            # or closing in, instead of only reacting once the gap has
            # already crossed a distance-state boundary.
            if target.distance_state == CLOSE:
                return self._scale(
                    lane_left,
                    lane_right,
                    self._adjusted_multiplier(self.close_multiplier, target.approach_rate),
                    "target_close_slow_down_lane_following",
                )

            if target.distance_state == GOOD:
                return self._scale(
                    lane_left,
                    lane_right,
                    self._adjusted_multiplier(self.good_multiplier, target.approach_rate),
                    "good_distance_lane_following",
                )

            if target.distance_state == FAR:
                return self._scale(
                    lane_left,
                    lane_right,
                    self._adjusted_multiplier(self.far_multiplier, target.approach_rate),
                    "target_far_speed_up_lane_following",
                )

        # Target is lost.
        self._lost_frames += 1

        # Important:
        # If lane is disabled because of red line, prefer NOT to slam to a
        # full stop on every brief loss -- a leader can disappear from frame
        # for a few frames purely because it's mid-turn (swinging toward the
        # edge, partial occlusion). Stopping immediately and then lurching
        # back to speed on reacquisition looks identical to oscillation.
        #
        # If we were already tracking the leader as "turning" right before
        # losing it, keep using lane following at a reduced speed for a
        # short grace window -- this is the most likely turn-occlusion case.
        # If we were NOT mid-turn (leader was going straight) or the grace
        # window expires, fall back to a full stop as before -- that case is
        # more likely a genuine leader loss at a crossroad, where stopping
        # is the safe choice.
        if lane_disabled:
            if self._leader_is_turning and self._lost_frames <= self.turn_lost_grace_frames:
                return self._scale_for_distance(
                    lane_left=lane_left,
                    lane_right=lane_right,
                    reason_suffix="red_line_leader_turning_lost_briefly_keep_lane",
                    target_distance_state=self._last_distance_state,
                )
            self._reset_turn_state()
            return self._stop("lane_disabled_target_lost_stop")

        # Outside a red-line window: don't nuke turn-detection/error
        # smoothing on every single lost frame. A leader briefly dropping
        # out of detection for 1-2 frames during a turn is normal noise --
        # resetting the EMA here means the next reacquired frame uses a
        # fresh, unsmoothed error as its seed, producing one sharp
        # uncorrected steering kick right when the leader reappears
        # off-center (most likely right after a turn). Only reset once the
        # loss has actually persisted.
        if self._lost_frames > self.smoothing_reset_lost_frames:
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

    def _adjusted_multiplier(self, base_multiplier: float, approach_rate: Optional[float]) -> float:
        """
        Adjusts a distance-bucket speed multiplier using the leader's
        approach_rate (smoothed px/frame change in its bbox bottom_y).

        approach_rate > 0  -> leader getting closer/bigger -> we should slow
                               down a bit even within the same bucket.
        approach_rate < 0  -> leader pulling away/shrinking -> we should
                               speed up to keep pace, instead of waiting for
                               the gap to cross into the next bucket first.

        The adjustment is intentionally small and clamped
        (approach_rate_max_adjust) -- this is a continuous nudge on top of
        the existing bucket logic, not a replacement for it. distance_state
        still does the heavy lifting (stopping when TOO_CLOSE, etc).
        """
        if not approach_rate:
            return base_multiplier

        # Negative approach_rate (pulling away) should INCREASE speed, so
        # the sign is flipped here.
        adjustment = -approach_rate * self.approach_rate_gain
        adjustment = max(-self.approach_rate_max_adjust, min(self.approach_rate_max_adjust, adjustment))

        return max(0.0, base_multiplier * (1.0 + adjustment))

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
        self._smoothed_leader_error = None

    def _scale_for_distance(
        self,
        lane_left: float,
        lane_right: float,
        reason_suffix: str,
        target: Optional[TargetInfo] = None,
        target_distance_state: Optional[str] = None,
    ) -> ConvoyCommand:
        distance_state = target.distance_state if target is not None else target_distance_state
        approach_rate = target.approach_rate if target is not None else None

        if distance_state == CLOSE:
            multiplier = self.close_multiplier
        elif distance_state == GOOD:
            multiplier = self.good_multiplier
        elif distance_state == FAR:
            multiplier = self.far_multiplier
        else:
            return self._stop("red_line_turn_invalid_distance_state")

        multiplier = self._adjusted_multiplier(multiplier, approach_rate)

        return self._scale(lane_left, lane_right, multiplier, reason_suffix)

    def _leader_only_command(
        self,
        target: TargetInfo,
        image_width: Optional[int],
    ) -> ConvoyCommand:
        if image_width is None or image_width <= 0 or target.center_x is None:
            return self._stop("leader_only_no_target_center_stop")

        raw_error = self._target_center_error(target, image_width)
        raw_error *= self.leader_steering_sign

        # Smooth the error (EMA) -- the leader's bbox is small at long range,
        # so its center estimate is noisy relative to box size, and that
        # noise gets amplified by normalization. Without this, oscillation
        # shows up specifically when the leader is FAR.
        if self._smoothed_leader_error is None:
            self._smoothed_leader_error = raw_error
        else:
            alpha = self.leader_error_smoothing
            self._smoothed_leader_error = (
                alpha * raw_error + (1.0 - alpha) * self._smoothed_leader_error
            )

        error = self._smoothed_leader_error

        # Deadband: ignore tiny residual noise around zero so the bot
        # doesn't twitch left-right when the leader is essentially centered.
        if abs(error) < self.leader_error_deadband:
            error = 0.0

        # Distance still controls forward speed, adjusted by approach_rate
        # so the bot reacts to the leader actively accelerating/decelerating
        # rather than only reacting once distance_state itself changes.
        if target.distance_state == CLOSE:
            speed_fraction = 0.30
            reason = "red_line_leader_only_close_slow_down"
            gain_scale = 1.0
        elif target.distance_state == GOOD:
            speed_fraction = 0.70
            reason = "red_line_leader_only_good_distance"
            gain_scale = 1.0
        elif target.distance_state == FAR:
            speed_fraction = 0.95
            reason = "red_line_leader_only_far_speed_up"
            # Extra damping at FAR distance, where the bbox-center estimate
            # is least reliable -- this is the case that was oscillating.
            gain_scale = self.leader_far_gain_scale
        else:
            return self._stop("red_line_leader_only_invalid_distance_state")

        speed_fraction = self._adjusted_multiplier(speed_fraction, target.approach_rate)
        base_speed = self.max_speed * speed_fraction

        max_steer = self.max_speed * self.leader_max_steer_ratio
        steering = error * self.leader_steering_gain * gain_scale * self.max_speed
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