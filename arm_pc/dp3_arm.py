import panda_py
import panda_py.controllers
import numpy as np
import torch
import math
import zerorpc
import time

# Constants
MOVE_INCREMENT = 0.0002
SPEED = 0.05  # [m/s]
FORCE = 20.0  # [N]

class armCtrl():
    def __init__(self):
        # 初始化 PandaArm
        hostname = '172.16.0.2'
        self.arm = panda_py.Panda(hostname)
        self.gripper = panda_py.libfranka.Gripper(hostname)
        self.gripper.homing()
        # Recover robot to a default state
        self.arm.recover()

    def set_n_obs_steps(self,obs_steps):
        self.n_obs_steps=obs_steps

    def set_n_action_steps(self,act_steps):
        self.n_action_steps=act_steps

    def get_current_pose(self):
        """
        获取机械臂当前的位姿（x, y, z, roll, pitch, yaw）,一共frames_num帧。
        """
        print("执行 get_current_pose 函数")

        trans = self.arm.get_position() # 获取末端执行器的当前位姿
        # print(f"transition = {trans}")
        quat = self.arm.get_orientation()
        eluer= self.quaternion_to_euler(quat)
        # print(f"eluer = {eluer}")
        gripper_width=self.gripper.read_once().width
        eluer.append(gripper_width)
        agent_pos=np.concatenate((trans,eluer),axis=0).tolist()
        # print(f"agent_pos = {agent_pos}")
        # print(type(agent_pos))
        # agent_pos_frames.append(torch.Tensor(agent_pos))

        # return torch.stack(agent_pos_frames,dim=0)
        print("执行结束")
        return agent_pos

    def move_relative(self,action):
        print("执行 move_relative 函数")
        """
        按照相对位移和旋转移动机械臂。
        """

        # action_frames_num=actions.shape[0]

        # Initialize robot controller
        controller = panda_py.controllers.CartesianImpedance()
        self.arm.start_controller(controller)
        # print(actions)
        start_time = time.perf_counter()
        print(f"动作： {action}")
        # 获取当前的位姿
        current_translation=self.arm.get_position()
        current_quat = self.arm.get_orientation()
        current_rpy= self.quaternion_to_euler(current_quat)

        gripper_width=self.gripper.read_once().width
        print(f"读夹爪的宽度 {gripper_width}")

        # 计算新的目标位置
        new_translation = current_translation + np.array(action[:3])
        new_rpy = current_rpy + np.array(action[3:6])
        new_gripper_width=gripper_width+action[6]
        if new_gripper_width<0.06 or gripper_width<0.06:
            new_gripper_width=0.0

        new_quat=self.euler_to_quaternion(current_rpy)
        print(f"平移： {new_translation}")
        print(f"旋转： {new_rpy}")
        print(f"夹爪: {new_gripper_width}")

        # 移动到目标位置
        controller.set_control(new_translation, new_quat)
        self.gripper.move(new_gripper_width, speed=SPEED)

        # Maintain loop frequency at 1000 Hz
        end_time = time.perf_counter()
        loop_duration = end_time - start_time
        loop_frequency = 1.0 / loop_duration
        time.sleep(max(0, (1/1000) - loop_duration))
        
        print("执行结束")



    def quaternion_to_euler(self,quat):
        """
        将四元数转换为欧拉角 (roll, pitch, yaw)
        :param w: 四元数的实部
        :param x: 四元数的 x 分量
        :param y: 四元数的 y 分量
        :param z: 四元数的 z 分量
        :return: 欧拉角 (roll, pitch, yaw)，单位为弧度
        """
        x=quat[0]
        y=quat[1]
        z=quat[2]
        w=quat[3]
        # 计算 roll (x 轴旋转)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # 计算 pitch (y 轴旋转)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
        # 使用 90 度限制
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # 计算 yaw (z 轴旋转)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def euler_to_quaternion(self, rpy):
        """
        将欧拉角转换为四元数
        参数:
        roll: 绕X轴的旋转角度（弧度）
        pitch: 绕Y轴的旋转角度（弧度）
        yaw: 绕Z轴的旋转角度（弧度）
        返回:
        四元数 (w, x, y, z)
        """
        roll=rpy[0]
        pitch=rpy[1]
        yaw=rpy[2]
        # 计算每个轴的半角正弦和余弦
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        # 计算四元数
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    # 示例用法
    # roll = np.radians(30) # 绕X轴旋转30度
    # pitch = np.radians(45) # 绕Y轴旋转45度
    # yaw = np.radians(60) # 绕Z轴旋转60度

    # quaternion = euler_to_quaternion(roll, pitch, yaw)
    # print("四元数:", quaternion)

# 示例使用
# w, x, y, z = 0.707, 0, 0.707, 0
# roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
# print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

if __name__ == "__main__":
# 获取当前位姿
    # x, y, z, roll, pitch, yaw = get_current_pose()
    # print(f"Current Pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
    sever=zerorpc.Server(armCtrl())
    sever.bind("tcp://0.0.0.0:4242")
    print("启动服务器")
    sever.run()
    # arm_ctrl=armCtrl()
    # pos=arm_ctrl.get_current_pose(2)
    # print("====================")
    # print(pos.shape)

# 移动机械臂相对位移
# move_relative(x_offset=0.1, y_offset=0.0, z_offset=0.0, roll_offset=0.0, pitch_offset=0.0, yaw_offset=0.0)

# # 再次获取位姿以确认位置
# x, y, z, roll, pitch, yaw = get_current_pose()
# print(f"New Pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
