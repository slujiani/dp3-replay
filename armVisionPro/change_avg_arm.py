import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation as R

# 坐标变换
def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix.

    Parameters:
        x, y, z, w: Components of the quaternion.

    Returns:
        A 3x3 numpy array representing the rotation matrix.
    """
    # Normalize the quaternion to ensure it's a unit quaternion
    x,y,z,w = q
    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    # Compute the rotation matrix
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return R


def calculate_quaternion(S_to_A_quaternion, B_to_A_euler):
    """
    计算物体 B 相对于坐标系 S 的四元数姿态。
    :param S_to_A_quaternion: 物体 A 相对于坐标系 S 的四元数 ( x, y, z, w)
    :param B_to_A_euler: 物体 B 相对于物体 A 的欧拉角 (roll, pitch, yaw)，单位为度
    :return: 物体 B 相对于坐标系 S 的四元数 (x, y, z, w)
    """

    B_to_A_quaternion = R.from_euler('xyz', B_to_A_euler, degrees=True).as_quat()


    S_to_A_quat = R.from_quat(S_to_A_quaternion[[0,1,2,3]])
    B_to_A_quat = R.from_quat(B_to_A_quaternion)
    S_to_B_quat = S_to_A_quat * B_to_A_quat  # 四元数乘法


    return S_to_B_quat.as_quat()[[0,1,2,3]]

def transform_pose_to_a(pose_b,head_s):
    # 定义坐标系b到坐标系a的旋转矩阵（绕z轴旋转-90度）
    R_ba = np.array([[0, 1, 0],
                     [-1, 0, 0],
                     [0, 0, 1]])




    # 提取物体在b坐标系的旋转矩阵和位移向量
    R_b = pose_b[0:3, 0:3]
    t_b = pose_b[0:3, 3]
    t_b[0] = t_b[0] - head_s[0]
    t_b[1] = t_b[1] - head_s[1]
    t_b[2] = t_b[2] - head_s[2]


    # 计算物体在a坐标系中的旋转矩阵和位移向量
    R_a = R_ba @ R_b

    t_a = R_ba @ t_b

    # 将旋转矩阵R_a转换为四元数
    rot_a = R.from_matrix(R_a)
    q_a = rot_a.as_quat()  # 返回的四元数格式是 [x, y, z, w]
    B_to_A_euler = [-90,0,-90]
    q_a = calculate_quaternion(q_a,B_to_A_euler)
    for i in range(3):
        t_a[i] = t_a[i]
    t_a[0] = t_a[0] * 0.8 + 0.55
    t_a[1] = t_a[1] * 0.6
    t_a[2] = t_a[2] * 0.4 + 0.1
    if t_a[2] < 0.2 :
        t_a[2] = 0.2
    # q_a_copy = (q_a[1] , q_a[2], q_a[3] ,q_a[0])
    result = [t_a.tolist() , q_a.tolist()]

    return result

