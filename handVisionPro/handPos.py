from angle import angle
import math


def getPosition(nums):
    ABof2 = 125.60
    BCof2 = 32.13
    CDof2 = 43.73
    alpha1of2 = angle(nums[3])[0]
    alpha2of2 = angle(nums[3])[1]
    YCof2 = ABof2 + BCof2 * math.cos(math.radians(180 - alpha1of2))
    YDof2 = YCof2 + CDof2 * math.cos(math.radians(360 - alpha1of2 - alpha2of2))
    # print(360 - alpha1of2 - alpha2of2)
    ZCof2 = BCof2 * math.sin(math.radians(180 - alpha1of2)) - 0.30
    ZDof2 = ZCof2 + CDof2 * math.sin(math.radians(360 - alpha1of2 - alpha2of2))
    result6 = [-28.91, 125.73, -0.30]
    result7 = [-28.91, YCof2, ZCof2]
    result8 = [-28.91, YDof2, ZDof2]

    ABof3 = 126.1
    BCof3 = 32.13
    CDof3 = 47.19
    alpha1of3 = angle(nums[2])[0]
    alpha2of3 = angle(nums[2])[1]
    YCof3 = ABof3 + BCof3 * math.cos(math.radians(180 - alpha1of3))
    YDof3 = YCof3 + CDof3 * math.cos(math.radians(360 - alpha1of3 - alpha2of3))

    ZCof3 = BCof3 * math.sin(math.radians(180 - alpha1of3)) - 0.30
    ZDof3 = ZCof3 + CDof3 * math.sin(math.radians(360 - alpha1of3 - alpha2of3))
    result9 = [-6.95, 126.1, -0.30]
    result10 = [-6.95, YCof3, ZCof3]
    result11 = [-6.95, YDof3, ZDof3]
    XE = -20.45
    YE = 49.23
    ZE = 16.96
    result1 = [XE, YE, ZE]
    alpha6 = angle(nums[5])[5]
    EF = 16.46
    XF = XE + EF * math.cos(math.radians(alpha6))
    ZF = ZE + EF * math.sin(math.radians(alpha6))
    YF = YE + 1.12
    result2 = [XF, YF, ZF]

    FG = 50.01
    GH = 22.36
    HI = 24.74

    alpha3 = angle(nums[4])[2]
    alpha4 = angle(nums[4])[3]
    alpha5 = angle(nums[4])[4]

    YG = YF + math.cos(math.radians(180 - alpha3)) * FG
    FGvertical = FG * math.sin(math.radians(180 - alpha3))
    XG = XF + FGvertical * math.cos(math.radians(alpha6))
    ZG = ZF + FGvertical * math.sin(math.radians(alpha6))
    result3 = [XG, YG, ZG]

    YH = YG + math.cos(math.radians(alpha4 - alpha3)) * GH
    FHvertical = FGvertical + GH * math.sin(math.radians(alpha4 - alpha3))
    XH = XF + FHvertical * math.cos(math.radians(alpha6))
    ZH = ZF + FHvertical * math.sin(math.radians(alpha6))
    result4 = [XH, YH, ZH]

    YI = YH + math.cos(math.radians(alpha5 + alpha4 - alpha3 - 180)) * HI
    FIvertical = FHvertical + HI * math.sin(math.radians(alpha5 + alpha4 - alpha3 - 180))
    XI = XF + FIvertical * math.cos(math.radians(alpha6))
    ZI = ZF + FIvertical * math.sin(math.radians(alpha6))
    result5 = [XI, YI, ZI]

    ABof4 = 125.5
    BCof4 = 32.13
    CDof4 = 43

    alpha1of4 = angle(nums[1])[0]
    alpha2of4 = angle(nums[1])[1]
    YCof4 = ABof4 + BCof4 * math.cos(math.radians(180 - alpha1of4))
    YDof4 = YCof4 + CDof4 * math.cos(math.radians(360 - alpha1of4 - alpha2of4))

    ZCof4 = BCof4 * math.sin(math.radians(180 - alpha1of4)) - 0.30
    ZDof4 = ZCof4 + CDof4 * math.sin(math.radians(360 - alpha1of4 - alpha2of4))
    result12 = [10.55, YDof4, ZDof4]

    ABof5 = 125.2
    BCof5 = 32
    CDof5 = 38

    alpha1of5 = angle(nums[0])[0]
    alpha2of5 = angle(nums[0])[1]
    YCof5 = ABof5 + BCof5 * math.cos(math.radians(180 - alpha1of5))
    YDof5 = YCof5 + CDof5 * math.cos(math.radians(360 - alpha1of5 - alpha2of5))

    ZCof5 = BCof5 * math.sin(math.radians(180 - alpha1of5)) - 0.30
    ZDof5 = ZCof5 + CDof5 * math.sin(math.radians(360 - alpha1of5 - alpha2of5))
    result13 = [28.05, YDof5, ZDof5]

    # return result1, result2, result3, result4, result5, result6, result7, result8, result9, result10, result11
    # 大拇指第0关节，大拇指第1关节，大拇指第2关节，大拇指第3关节，大拇指指尖，食指第0关节，食指第1关节，食指指尖，中指第0关节，中指第1关节，中指指尖，
    # return result5, result8, result11
    result5_c = (result5[1] * 0.001, result5[2] * 0.001, result5[0] * 0.001)
    result8_c = (result8[1] * 0.001, result8[2] * 0.001, result8[0] * 0.001)
    result11_c = (result11[1] * 0.001, result11[2] * 0.001, result11[0] * 0.001)
    result12_c = (result12[1] * 0.001, result12[2] * 0.001, result12[0] * 0.001)
    result13_c = (result13[1] * 0.001, result13[2] * 0.001, result13[0] * 0.001)
    return [result13_c, result12_c, result11_c, result8_c, result5_c]

# print(([-100+15+31+10, 17.21, 148.15+26], [-0.64085638, -0.64085638, 0.29883624, 0.29883624]))
# print(getPosition([1000,1000,1000,1000,1000,1000,1000]))