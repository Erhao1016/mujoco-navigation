import math


class PDController:
    """
    二维位置 PD 路径跟踪控制器。

    输入：
        当前机器人位置 current = (x, y)
        当前路径目标点 target = (x, y)

    输出：
        vx：世界坐标系 X 方向速度
        vy：世界坐标系 Y 方向速度
        wz：绕 Z 轴角速度

    当前机器人拥有 X、Y 两个平移关节，因此可以直接进行全向移动。
    """

    def __init__(
        self,
        kp: float = 2.0,
        kd: float = 0.1,
        max_speed: float = 0.9,
        reach_threshold: float = 0.08,
    ):
        if kp < 0:
            raise ValueError("kp 不能小于 0。")

        if kd < 0:
            raise ValueError("kd 不能小于 0。")

        if max_speed <= 0:
            raise ValueError("max_speed 必须大于 0。")

        if reach_threshold <= 0:
            raise ValueError("reach_threshold 必须大于 0。")

        self.kp = kp
        self.kd = kd
        self.max_speed = max_speed
        self.reach_threshold = reach_threshold

        self.previous_error_x = 0.0
        self.previous_error_y = 0.0

        # 第一次计算时没有上一时刻误差，不计算微分项
        self.first_update = True

    def reset(self) -> None:
        """
        重置控制器历史状态。

        开始跟踪一条新路径时可以调用。
        """
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.first_update = True

    def distance_to_target(
        self,
        current: tuple[float, float],
        target: tuple[float, float],
    ) -> float:
        """
        计算机器人当前位置到目标点的欧氏距离。
        """
        error_x = float(target[0]) - float(current[0])
        error_y = float(target[1]) - float(current[1])

        return math.hypot(error_x, error_y)

    def reached(
        self,
        current: tuple[float, float],
        target: tuple[float, float],
    ) -> bool:
        """
        判断机器人是否已经到达当前路径点。
        """
        return (
            self.distance_to_target(current, target)
            <= self.reach_threshold
        )

    def compute_velocity(
        self,
        current: tuple[float, float],
        target: tuple[float, float],
        dt: float,
        current_yaw: float | None = None,
    ) -> tuple[float, float, float]:
        """
        根据当前位置和目标位置计算速度指令。

        参数：
            current：机器人当前位置 (x, y)
            target：当前目标路径点 (x, y)
            dt：两次控制计算之间的时间，单位为秒
            current_yaw：机器人当前朝向，单位为弧度

        返回：
            (vx, vy, wz)
        """
        if dt <= 0:
            raise ValueError("dt 必须大于 0。")

        error_x = float(target[0]) - float(current[0])
        error_y = float(target[1]) - float(current[1])

        if self.first_update:
            derivative_x = 0.0
            derivative_y = 0.0
            self.first_update = False
        else:
            derivative_x = (
                error_x - self.previous_error_x
            ) / dt

            derivative_y = (
                error_y - self.previous_error_y
            ) / dt

        self.previous_error_x = error_x
        self.previous_error_y = error_y

        # PD 控制
        vx = self.kp * error_x + self.kd * derivative_x
        vy = self.kp * error_y + self.kd * derivative_y

        # 限制二维合速度，避免超过机器人 actuator 范围
        speed = math.hypot(vx, vy)

        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale

        wz = 0.0

        if current_yaw is not None and speed > 0.01:
            target_yaw = math.atan2(
                vy,
                vx,
            )

            yaw_error = math.atan2(
                math.sin(target_yaw - current_yaw),
                math.cos(target_yaw - current_yaw),
            )

            wz = 4.0 * yaw_error
            wz = max(
                -2.0,
                min(2.0, wz),
            )

        return vx, vy, wz
