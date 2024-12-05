import time
# import hand
from avp_stream import VisionProStreamer
import numpy as np
import panda_py
from panda_py import controllers
from change_avg_arm import transform_pose_to_a
import threading
import csv
from scipy.spatial.transform import Rotation as R, Slerp

def interpolate_pose(x0, q0, x1, q1, steps=500):
    """
    平滑插值从(x0, q0)到(x1, q1)

    :param x0: 初始位置 (3D坐标)
    :param q0: 初始姿态 (四元数)
    :param x1: 目标位置 (3D坐标)
    :param q1: 目标姿态 (四元数)
    :param steps: 插值步数
    :return: 每一步的插值结果列表 [(x, q), ...]
    """
    # 线性插值位置
    x_interp = [x0 + (x1 - x0) * t for t in np.linspace(0, 1, steps)]

    # 球面线性插值姿态
    key_times = [0, 1]  # 时间点
    rotations = R.from_quat([q0, q1])  # 转换为 Rotation 对象
    slerp = Slerp(key_times, rotations)
    interp_times = np.linspace(0, 1, steps)  # 插值时间点
    q_interp = slerp(interp_times).as_quat()

    return list(zip(x_interp, q_interp))

def armRead():
    fields = ['set','pose','time']
    # panda = panda_py.Panda('172.16.0.2')

    with open('arm_control_3.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(fields)
    while True:
        x0 = panda.get_position()
        q0 = panda.get_orientation()
        timeLine = time.time()
        with open('arm_control_3.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([x0,q0,timeLine])
        time.sleep(0.1)



def armControl():
    # Initialize robot interface
    # panda = panda_py.Panda('172.16.0.2')
    # panda.move_to_start()

    avp_ip = "192.168.8.99"  # example IP
    s = VisionProStreamer(ip=avp_ip, record=True)

    ctrl = controllers.CartesianImpedance(filter_coeff=1.0)
    x0 = panda.get_position()
    q0 = panda.get_orientation()
    runtime = np.pi * 400
    panda.start_controller(ctrl)

    # poses = []

    with panda.create_context(frequency=1e3, max_runtime=runtime) as ctx:
        while ctx.ok():
            r = s.latest
            left_wrist = r['left_wrist'][0]
            # print(left_wrist[0:3, 3])
            head = r['head'][0]
            head_s = [head[0][3],head[1][3],head[2][3]-0.6]
            # print(head_s)
            # print(left_wrist)
            # print(left_wrist[0])
            left_wrist_arm = transform_pose_to_a(left_wrist,head_s)
            x_goto = left_wrist_arm[0]
            print(x_goto)
            q_goto = left_wrist_arm[1]
            # print(x_goto)
            x0 = panda.get_position()
            q0 = panda.get_orientation()
            poses = interpolate_pose(x0, q0, x_goto, q_goto, steps=700)
            # print(x0,q0)
            # print(x_goto,q_goto)
            # print(poses)
            # print("\n")
            for pose in poses:
                x_next,q_next = pose
                ctrl.set_control(x_next, q_next)
                time.sleep(0.001)




            # q_0 = turn.turn(q_0)
            # pose = (np.array(x_0),np .array(q_0))
            # poses.append(pose)
            # print([x_0,q_0])
            #
            # ctrl.set_control(x_0, q_0)




if __name__ == "__main__":
    panda = panda_py.Panda('172.16.0.2')
    controlThread = threading.Thread(target = armControl)
    readThread = threading.Thread(target = armRead)
    controlThread.start()
    readThread.start()
    controlThread.join()
    readThread.join()
    # animate_motion(poses)

