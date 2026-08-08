import math

import numpy as np


class CameraAvoidanceController:
    """
    基于机器人前向摄像头的简单局部避障控制器。

    它不替代 A*，而是在 A* 输出速度后做局部修正：
        1. 从摄像头图像中提取高饱和度障碍物区域
        2. 判断障碍物在画面左侧、中央还是右侧
        3. 降低前进速度，并向空的一侧横移
    """

    def __init__(
        self,
        obstacle_threshold: float = 0.07,
        center_threshold: float = 0.05,
        avoidance_speed: float = 0.65,
        minimum_forward_scale: float = 0.45,
        max_speed: float = 2.4,
        hold_frames: int = 18,
    ):
        self.obstacle_threshold = obstacle_threshold
        self.center_threshold = center_threshold
        self.avoidance_speed = avoidance_speed
        self.minimum_forward_scale = minimum_forward_scale
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
        image: np.ndarray,
    ) -> tuple[float, float, float]:
        obstacle = self.detect_obstacle(
            image
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
            0.45 if self.locked_direction else 0.0,
        )

        forward_scale = obstacle.get(
            "forward_scale",
            0.70,
        )

        forward *= forward_scale

        left += (
            self.avoidance_speed
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

        adjusted_wz = (
            wz
            + 0.55
            * strength
            * self.locked_direction
        )
        adjusted_wz = max(
            -2.0,
            min(2.0, adjusted_wz),
        )

        return self.smooth_command(
            adjusted_vx,
            adjusted_vy,
            adjusted_wz,
        )

    def detect_obstacle(
        self,
        image: np.ndarray,
    ) -> dict:
        if image is None or image.size == 0:
            return self.no_detection()

        height, width, _ = image.shape

        crop_start = int(
            height * 0.48
        )

        view = image[
            crop_start:,
            :,
            :,
        ].astype(
            np.int16
        )

        max_channel = view.max(
            axis=2
        )

        min_channel = view.min(
            axis=2
        )

        saturation = (
            max_channel
            - min_channel
        )

        obstacle_mask = (
            (saturation > 45)
            & (max_channel > 70)
        )

        if not obstacle_mask.any():
            return self.no_detection()

        rows, cols = np.indices(
            obstacle_mask.shape
        )

        x_center = (
            obstacle_mask.shape[1] - 1
        ) / 2.0

        y_bottom = max(
            1.0,
            obstacle_mask.shape[0] - 1,
        )

        x_normalized = (
            cols - x_center
        ) / x_center

        y_normalized = rows / y_bottom

        closeness_weight = (
            y_normalized
            * y_normalized
            * y_normalized
        )

        weighted_mask = (
            obstacle_mask
            * closeness_weight
        )

        total_weight = float(
            weighted_mask.sum()
        )

        obstacle_score = (
            total_weight
            / float(obstacle_mask.size)
        )

        middle_start = int(
            width * 0.34
        )

        middle_end = int(
            width * 0.66
        )

        center_mask = obstacle_mask[
            :,
            middle_start:middle_end,
        ]

        center_score = (
            float(center_mask.sum())
            / float(center_mask.size)
        )

        if (
            obstacle_score < self.obstacle_threshold
            and center_score < self.center_threshold
        ):
            return self.no_detection()

        obstacle_x = float(
            (
                weighted_mask
                * x_normalized
            ).sum()
            / max(total_weight, 1e-6)
        )

        if abs(obstacle_x) < 0.12:
            avoidance_direction = self.locked_direction or 1.0
        else:
            avoidance_direction = math.copysign(
                1.0,
                obstacle_x,
            )

        strength = max(
            obstacle_score / self.obstacle_threshold,
            center_score / self.center_threshold,
        )

        strength = max(
            0.0,
            min(1.0, strength),
        )

        forward_scale = 1.0 - 0.75 * strength
        forward_scale = max(
            self.minimum_forward_scale,
            forward_scale,
        )

        return {
            "detected": True,
            "strength": strength,
            "avoidance_direction": avoidance_direction,
            "forward_scale": forward_scale,
        }

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

        alpha = 0.28

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

        return (
            vx * scale,
            vy * scale,
        )
