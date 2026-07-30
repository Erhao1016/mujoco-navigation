import mujoco


class RobotController:


    def __init__(self, model, data):

        """
        MuJoCo控制器

        输入:
            model
            data

        """

        self.model = model
        self.data = data



        # 默认速度

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0



    # ==========================
    # 设置速度
    # ==========================

    def set_velocity(
            self,
            vx,
            vy,
            wz
    ):

        """
        设置机器人速度

        vx:
            X方向速度

        vy:
            Y方向速度

        wz:
            绕Z轴角速度

        """


        self.vx = vx
        self.vy = vy
        self.wz = wz



    # ==========================
    # 更新控制量
    # ==========================

    def update(self):


        """
        将速度发送给MuJoCo actuator
        """


        if self.model.nu >= 3:


            # X速度

            self.data.ctrl[0] = self.vx


            # Y速度

            self.data.ctrl[1] = self.vy


            # 角速度

            self.data.ctrl[2] = self.wz



    # ==========================
    # 停止
    # ==========================

    def stop(self):


        self.set_velocity(
            0,
            0,
            0
        )

        self.update()