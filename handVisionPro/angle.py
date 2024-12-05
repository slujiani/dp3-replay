def angle(num):
    angle1 = 0.08 * num + 90
    # 四指关节角90-170
    angle2 = 0.000000009202 * angle1 ** 5 - 0.000006814 * angle1 ** 4 + 0.002025 * angle1 ** 3 - 0.3064 * angle1 ** 2 + 24.82 * angle1 - 754.5
    # 四指的P2关节角
    angle3 = -0.04 * num + 170
    # 大拇指弯曲关节角
    angle4 = -0.00003409 * (270 - angle3) ** 3 + 0.01698 * (270 - angle3) ** 2 - 1.464 * (270 - angle3) + 154.7
    # 大拇指指腹关节角
    angle5 = -0.000008363 * (270 - angle3) ** 3 - 0.008375 * (270 - angle3) ** 2 + 3.265 * (270 - angle3) - 99.73
    # 大拇指指尖关节角
    angle6 = 0.08 * num + 85
    # 大拇指侧摆关节角

    result = ()
    result += (angle1, angle2, angle3, angle4, angle5, angle6)

    return result


