import math

import numpy as np


class DepthAvoidanceController:
    """
    基于 MuJoCo 摄像头深度图的局部避障控制器。

    不需要深度学习。MuJoCo 可以直接渲染 depth image，
    每个像素表示摄像头方向上看到的物体距离。
    """

    def __init__(
        self,
        danger_distance: float = 0.75,
        slow_distance: float = 1.25,
        side_speed: float = 1.4,
        reverse_speed: float = 0.75,
        max_speed: float = 2.4,
        hold_frames: int = 24,
    ):
        self.danger_distance = danger_distance
        self.slow_distance = slow_distance
        self.side_speed = side_speed
        self.reverse_speed = reverse_speed
        self.max_speed = max_speed
        self.hold_frames = hold_frames

        self.locked_direction = 0.0
        self.lock_frames_remaining = 0

        self.previous_vx = 0.0
        self.previous_vy = 0.0
        self.previous_wz = 0.0
        self.first_command = True

    def apply(
        self,
        vx: float,
        vy: float,
        wz: float,
        current_yaw: float,
        depth: np.ndarray,
    ) -> tuple[float, float, float]:
        obstacle = self.detect_obstacle(
            depth
        )

        if obstacle["detected"]:
            self.update_direction_lock(
                obstacle["avoidance_direction"]
            )

        elif self.lock_frames_remaining > 0:
            self.lock_frames_remaining -= 1

        else:
            self.locked_direction = 0.0

        if (
            not obstacle["detected"]
            and self.locked_direction == 0.0
        ):
            return self.smooth_command(
                vx,
                vy,
                wz,
            )

        forward, left = self.world_to_robot_velocity(
            vx,
            vy,
            current_yaw,
        )

        strength = obstacle.get(
            "strength",
            0.35 if self.locked_direction else 0.0,
        )

        forward_scale = obstacle.get(
            "forward_scale",
            0.75,
        )

        nearest_distance = obstacle.get(
            "nearest_distance",
            self.slow_distance,
        )

        if nearest_distance <= self.danger_distance:
            forward = 0.18 * (1.0 - strength)
        else:
            forward *= forward_scale

        left += (
            self.side_speed
            * strength
            * self.locked_direction
        )

        adjusted_vx, adjusted_vy = self.robot_to_world_velocity(
            forward,
            left,
            current_yaw,
        )

        adjusted_vx, adjusted_vy = self.limit_speed(
            adjusted_vx,
            adjusted_vy,
        )

        return self.smooth_command(
            adjusted_vx,
            adjusted_vy,
            wz,
        )

    def detect_obstacle(
        self,
        depth: np.ndarray,
    ) -> dict:
        if depth is None or depth.size == 0:
            return self.no_detection()

        height, width = depth.shape

        crop_top = int(
            height * 0.12
        )

        crop_bottom = int(
            height * 0.62
        )

        view = depth[
            crop_top:crop_bottom,
            :,
        ]

        valid = np.isfinite(
            view
        ) & (
            view > 0.05
        )

        if not valid.any():
            return self.no_detection()

        left_region = view[
            :,
            : int(width * 0.34),
        ]

        center_region = view[
            :,
            int(width * 0.34): int(width * 0.66),
        ]

        right_region = view[
            :,
            int(width * 0.66):,
        ]

        left_distance = self.near_distance(
            left_region
        )

        center_distance = self.near_distance(
            center_region
        )

        right_distance = self.near_distance(
            right_region
        )

        nearest_distance = min(
            left_distance,
            center_distance,
            right_distance,
        )

        if nearest_distance >= self.slow_distance:
            return self.no_detection()

        if center_distance < self.slow_distance:
            if self.locked_direction != 0.0:
                direction = self.locked_direction
            elif left_distance > right_distance:
                direction = 1.0
            else:
                direction = -1.0

        elif left_distance < self.slow_distance:
            direction = -1.0

        else:
            direction = 1.0

        strength = (
            self.slow_distance
            - nearest_distance
        ) / (
            self.slow_distance
            - self.danger_distance
        )

        strength = max(
            0.0,
            min(1.0, strength),
        )

        if nearest_distance <= self.danger_distance:
            forward_scale = 0.12
        else:
            forward_scale = 1.0 - 0.75 * strength

        forward_scale = max(
            0.12,
            min(1.0, forward_scale),
        )

        return {
            "detected": True,
            "strength": strength,
            "avoidance_direction": direction,
            "forward_scale": forward_scale,
            "nearest_distance": nearest_distance,
            "left_distance": left_distance,
            "center_distance": center_distance,
            "right_distance": right_distance,
        }

    def near_distance(
        self,
        region: np.ndarray,
    ) -> float:
        valid = region[
            np.isfinite(region)
            & (region > 0.05)
        ]

        if valid.size == 0:
            return float("inf")

        return float(
            np.percentile(
                valid,
                8,
            )
        )

    def no_detection(
        self,
    ) -> dict:
        return {
            "detected": False,
        }

    def update_direction_lock(
        self,
        direction: float,
    ) -> None:
        if self.locked_direction == 0.0:
            self.locked_direction = direction

        self.lock_frames_remaining = self.hold_frames

    def world_to_robot_velocity(
        self,
        vx: float,
        vy: float,
        yaw: float,
    ) -> tuple[float, float]:
        cos_yaw = math.cos(
            yaw
        )

        sin_yaw = math.sin(
            yaw
        )

        forward = (
            cos_yaw * vx
            + sin_yaw * vy
        )

        left = (
            -sin_yaw * vx
            + cos_yaw * vy
        )

        return forward, left

    def robot_to_world_velocity(
        self,
        forward: float,
        left: float,
        yaw: float,
    ) -> tuple[float, float]:
        cos_yaw = math.cos(
            yaw
        )

        sin_yaw = math.sin(
            yaw
        )

        vx = (
            cos_yaw * forward
            - sin_yaw * left
        )

        vy = (
            sin_yaw * forward
            + cos_yaw * left
        )

        return vx, vy

    def limit_speed(
        self,
        vx: float,
        vy: float,
    ) -> tuple[float, float]:
        speed = math.hypot(
            vx,
            vy,
        )

        if speed <= self.max_speed:
            return vx, vy

        scale = self.max_speed / speed

        return vx * scale, vy * scale

    def smooth_command(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> tuple[float, float, float]:
        if self.first_command:
            self.previous_vx = vx
            self.previous_vy = vy
            self.previous_wz = wz
            self.first_command = False

            return vx, vy, wz

        alpha = 0.35

        smoothed_vx = (
            self.previous_vx
            + alpha * (vx - self.previous_vx)
        )

        smoothed_vy = (
            self.previous_vy
            + alpha * (vy - self.previous_vy)
        )

        smoothed_wz = (
            self.previous_wz
            + alpha * (wz - self.previous_wz)
        )

        self.previous_vx = smoothed_vx
        self.previous_vy = smoothed_vy
        self.previous_wz = smoothed_wz

        return smoothed_vx, smoothed_vy, smoothed_wz
