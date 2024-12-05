import impire_hand_write_read as hand_wr
import numpy as np
from handPos import getPosition
from scipy.optimize import minimize
from avp_stream import VisionProStreamer
import time
import csv
import threading
# sudo chmod 777 /dev/ttyUSB0

def write_csv(read_value, timeLine):

    csv_file_path = "operator_hand.csv"

    with open(csv_file_path, 'a', newline='') as fileLine:
        writerLine = csv.writer(fileLine)
        writerLine.writerow([read_value, timeLine])



def getTranslation(matrix):
    translation = matrix[0:3, 3]
    return translation


def L_hand(q, operator_hand, q_pre):
    N = operator_hand.shape[0]
    # alpha = [1.22, 1.24, 1.13, 1.23, 1.36]
    alpha = [1.36, 1.23, 1.23, 1.24, 1.22]
    # alpha = 1.086
    beta = 0.0000000085
    delta_q = q - q_pre
    L_hand_value = 0

    # 将 q 转换为列表
    q_list = q.tolist()

    # 连接两个列表
    q_c = q_list

    Fkis = getPosition(q_c)
    for i in range(N):
        v_i = operator_hand[i]
        Fki = Fkis[i]
        L_hand_value += (np.linalg.norm(alpha[i] * v_i - Fki)) ** 2
    L_hand_value += beta * (np.linalg.norm(delta_q) ** 2)

    return L_hand_value


fun = lambda q: L_hand(q, operator_hand, q_pre)

if __name__ == '__main__':
    avp_ip = "192.168.8.99"  # example IP
    s = VisionProStreamer(ip=avp_ip, record=True)
    ser = hand_wr.openSerial('/dev/ttyUSB0', 115200)

    fields = ['angleRead', 'time']
    with open('operator_hand.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(fields)
    # print("Writing")
    # i = 0
    # write_thread = threading.Thread(target=write_csv)
    # write_thread.start()

    while True:
        r = s.latest
        fingers = r['left_fingers']
        operator_hand = np.array([getTranslation(fingers[24]), getTranslation(fingers[19]), getTranslation(fingers[14]),
                                  getTranslation(fingers[9]), getTranslation(fingers[4])])
        # print("operator_hand :")
        # print(operator_hand)
        read_value = hand_wr.read6(ser, 1, 'angleAct')
        timeLine = time.time()
        write_csv(read_value,timeLine)
        # print("read_value : " + str(time.time()))
        # print(read_value)
        q_pre = read_value
        fun = lambda q: L_hand(q, operator_hand, q_pre)

        # # 初始猜测值
        # n = 4  # 假设q是一个长度为10的向量，根据实际情况修改
        # q0 = np.array(q_pre)  # 初始猜测值在1到999之间
        # print(q0)
        # 定义边界约束
        bounds = [(0, 1000) for _ in range(6)]
        #
        options = {'ftol': 1e-4, 'disp': True, 'gtol': 1e-6}
        value_fun = 0
        value_x = []
        # for i in range(2):
        #     if i == 0:
        #         # 调用 minimize 函数
        #         result = minimize(fun, q0, bounds=bounds, method='TNC', options=options)
        #         if result.success:
        #             value_x = result.x
        #             value_fun = result.fun
        #             print(value_x, value_fun)
        #     else:
        #         # 调用 minimize 函数
        #         result = minimize(fun, q0, bounds=bounds, method='TNC', options=options)
        #         if result.success:
        #             if result.fun < value_fun or value_fun == 0:
        #                 value_fun = result.fun
        #                 value_x = result.x
        #                 print(value_x, value_fun)

        for i in range(2):
            if i == 0:
                q0 = np.array(q_pre)
                result = minimize(fun, q0, bounds=bounds, method='TNC', options=options)
                if result.success:
                    value_x = result.x
                    value_fun = result.fun
                    print(value_x, value_fun)
            if i == 1:
                q_pre_x = [1000 - q_pre[0], 1000 - q_pre[1], 1000 - q_pre[2], 1000 - q_pre[3], 1000 - q_pre[4],
                           1000 - q_pre[5]]
                q0 = np.array(q_pre_x)
                result = minimize(fun, q0, bounds=bounds, method='TNC', options=options)
                if result.success:
                    if result.fun < value_fun or value_fun == 0:
                        value_fun = result.fun
                        value_x = result.x
                        print(value_x, value_fun)

        # 输出结果
        if value_fun != 0:
            print("找到最小化 L_hand 的 q:", value_x)
            write_value = value_x.astype(int).tolist()
            hand_wr.write6(ser, 1, 'angleSet', write_value)

            # print(write_value)


        else:
            print("优化过程失败")
        time.sleep(0.04)
