from collections import deque
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


class ConvoyController:
    """
    Shared convoy controller for simulation and real robot.

    Normal mode:
        leader following controls steering and distance
        lane following is only a fallback when the leader is lost
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
        leader_steering_gain: float = 0.55,
        leader_max_steer_ratio: float = 0.35,
        leader_steering_sign: float = 1.0,
        leader_deadband: float = 0.12,
        leader_history_frames: int = 12,
        leader_memory_frames: int = 12,
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
        self.leader_deadband = leader_deadband
        self.leader_memory_frames = leader_memory_frames

        self._had_target = False
        self._lost_frames = 0
        self._last_distance_state = LOST
        self._leader_evidence_history = deque(maxlen=leader_history_frames)
        self._leader_memory_count = 0
        self._last_leader_evidence = 0.0

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

            # If we can see the leader, follow the leader. The lane mask is
            # unreliable at intersections and should not steer the robot.
            return self._leader_only_command(
                target=target,
                image_width=image_width,
            )

        # Target is lost.
        self._lost_frames += 1
        if self._leader_memory_count > 0:
            self._leader_memory_count -= 1

        # Important:
        # If lane is disabled because of red line, do NOT continue lane following.
        # If leader is lost at crossroad, creep straight instead of trusting
        # ambiguous intersection lane markings.
        if lane_disabled:
            return self._remembered_leader_command("lane_disabled_target_lost_follow_memory")

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

        if self._had_target and self._leader_memory_count > 0:
            return self._remembered_leader_command("target_temporarily_lost_follow_memory")

        if lane_left > 0.0 or lane_right > 0.0:
            return self._scale(
                lane_left,
                lane_right,
                self.lost_grace_multiplier,
                "leader_lost_lane_fallback",
            )

        return self._stop("target_lost_stop")

    def _leader_only_command(
        self,
        target: TargetInfo,
        image_width: Optional[int],
    ) -> ConvoyCommand:
        if image_width is None or image_width <= 0 or target.center_x is None:
            return self._stop("leader_only_no_target_center_stop")

        error = self._leader_mask_evidence(target, image_width)
        self._remember_leader_evidence(error)
        error = self._last_leader_evidence
        error *= self.leader_steering_sign
        if abs(error) < self.leader_deadband:
            error = 0.0

        # Distance still controls forward speed.
        if target.distance_state == TOO_CLOSE:
            base_speed = self.max_speed * 0.12
            reason = "leader_following_too_close_creep"
        elif target.distance_state == CLOSE:
            base_speed = self.max_speed * 0.30
            reason = "leader_following_close_slow_down"
        elif target.distance_state == GOOD:
            base_speed = self.max_speed * 0.70
            reason = "leader_following_good_distance"
        elif target.distance_state == FAR:
            base_speed = self.max_speed * 0.95
            reason = "leader_following_far_speed_up"
        else:
            return self._stop("leader_following_invalid_distance_state")

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

    def _remember_leader_evidence(self, evidence: float) -> None:
        evidence = self._clamp_range(evidence, -1.0, 1.0)
        self._leader_evidence_history.append(evidence)
        self._last_leader_evidence = self._smoothed_leader_evidence()
        self._leader_memory_count = self.leader_memory_frames

    def _smoothed_leader_evidence(self) -> float:
        if not self._leader_evidence_history:
            return 0.0

        values = list(self._leader_evidence_history)
        avg = sum(values) / len(values)

        if len(values) >= 4:
            mid = len(values) // 2
            old_avg = sum(values[:mid]) / len(values[:mid])
            new_avg = sum(values[mid:]) / len(values[mid:])
            trend = new_avg - old_avg
        else:
            trend = 0.0

        return self._clamp_range(0.95 * avg + 0.05 * trend, -1.0, 1.0)

    def _leader_mask_evidence(
        self,
        target: TargetInfo,
        image_width: int,
    ) -> float:
        if target.bbox is None:
            return self._target_center_error(target, image_width)

        x1, _, x2, _ = target.bbox
        half_width = image_width / 2.0

        left_width = max(0.0, min(float(x2), half_width) - max(float(x1), 0.0))
        right_width = max(0.0, min(float(x2), float(image_width)) - max(float(x1), half_width))
        total_width = left_width + right_width

        if total_width <= 0.0:
            return self._target_center_error(target, image_width)

        # Positive evidence means more leader-mask mass is on the left side
        # of the camera, so the follower should turn left. Negative means
        # more mass is on the right, so the follower should turn right.
        evidence = (left_width - right_width) / total_width

        # Blend with the box center so a fully left/right box still gives a
        # smooth correction instead of an all-or-nothing steering command.
        center_error = self._target_center_error(target, image_width)
        return self._clamp_range(0.35 * evidence + 0.65 * center_error, -1.0, 1.0)

    def _remembered_leader_command(self, reason: str) -> ConvoyCommand:
        if self._leader_memory_count <= 0:
            return self._straight(self.max_speed * 0.12, reason + "_straight")

        base_speed = self.max_speed * 0.18
        error = self._last_leader_evidence * self.leader_steering_sign
        if abs(error) < self.leader_deadband:
            error = 0.0

        max_steer = self.max_speed * self.leader_max_steer_ratio
        steering = error * self.leader_steering_gain * self.max_speed
        steering = self._clamp_range(steering, -max_steer, max_steer)

        left_speed = self._clamp(base_speed - steering)
        right_speed = self._clamp(base_speed + steering)

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

    def _straight(self, speed: float, reason: str) -> ConvoyCommand:
        speed = self._clamp(speed)
        return ConvoyCommand(
            should_move=speed > 0.0,
            left_speed=speed,
            right_speed=speed,
            speed_multiplier=speed / self.max_speed if self.max_speed > 0 else 0.0,
            reason=reason,
        )

    def _clamp(self, value: float) -> float:
        return self._clamp_range(value, 0.0, self.max_speed)

    @staticmethod
    def _clamp_range(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(float(value), maximum))
