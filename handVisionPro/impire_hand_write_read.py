import serial  # 调用串口通信库
import time  # 调用时间库

def benormal(list):
    for i,v in enumerate(list):
        if v >= 32768:
            list[i] = v - 65536

# 寄存器地址说明，对应仿人五指灵巧手——RH56用户手册11页，2.4寄存器说明
regdict = {
    'ID': 1000,  # ID
    'baudrate': 1001,  # 波特率设置
    'clearErr': 1004,  # 清除错误
    'forceClb': 1009,  # 手里传感器校准
    'angleSet': 1486,  # 各自由度的角度设置值
    'forceSet': 1498,  # 各自由度的力控阈值设置值
    'speedSet': 1522,  # 各自由度的速度设置值
    'angleAct': 1546,  # 各自由度的角度实际值
    'forceAct': 1582,  # 各手指的实际受力
    'errCode': 1606,  # 各自由度的电缸故障信息
    'statusCode': 1612,  # 各自由度的状态信息
    'temp': 1618,  # 各自由度的电缸的温度
    'actionSeq': 2320,  # 当前动作序列索引号
    'actionRun': 2322  # 运行当前动作序列
}


# 函数说明：设置串口号和波特率并且打开串口；参数：port为串口号，baudrate为波特率
def openSerial(port, baudrate):
    ser = serial.Serial()  # 调用串口通信函数
    ser.port = port
    ser.baudrate = baudrate
    ser.open()  # 打开串口
    return ser


# 函数说明：写灵巧手寄存器操作函数；参数：id为灵巧手ID号，add为起始地址，num为该帧数据的部分长度，val为所要写入寄存器的数据
def writeRegister(ser, id, add, num, val):
    bytes = [0xEB, 0x90]  # 帧头
    bytes.append(id)  # id
    bytes.append(num + 3)  # len
    bytes.append(0x12)  # cmd 写寄存器命令标志
    bytes.append(add & 0xFF)  # 寄存器起始地址低八位
    bytes.append((add >> 8) & 0xFF)  # 寄存器起始地址高八位
    for i in range(num):
        bytes.append(val[i])
    checksum = 0x00  # 校验和初始化为0
    for i in range(2, len(bytes)):
        checksum += bytes[i]  # 对数据进行加和处理
    checksum &= 0xFF  # 对校验和取低八位
    bytes.append(checksum)  # 低八位校验和
    ser.write(bytes)  # 向串口写入数据
    time.sleep(0.01)  # 延时10ms
    ser.read_all()  # 把返回帧读掉，不处理


# 函数说明：读灵巧手寄存器操作；参数：id为灵巧手ID号，add为起始地址，num为该帧数据的部分长度，mute为
def readRegister(ser, id, add, num, mute=False):
    bytes = [0xEB, 0x90]  # 帧头
    bytes.append(id)  # id
    bytes.append(0x04)  # len 该帧数据长度
    bytes.append(0x11)  # cmd 读寄存器命令标志
    bytes.append(add & 0xFF)  # 寄存器起始地址低八位
    bytes.append((add >> 8) & 0xFF)  # 寄存器起始地址高八位
    bytes.append(num)
    checksum = 0x00  # 校验和赋值为0
    for i in range(2, len(bytes)):
        checksum += bytes[i]  # 对数据进行加和处理
    checksum &= 0xFF  # 对校验和取低八位
    bytes.append(checksum)  # 低八位校验和
    ser.write(bytes)  # 向串口写入数据
    time.sleep(0.01)  # 延时10ms
    recv = ser.read_all()  # 从端口读字节数据
    if len(recv) == 0:  # 如果返回的数据长度为0，直接返回
        return []
    num = (recv[3] & 0xFF) - 3  # 寄存器数据所返回的数量
    val = []
    for i in range(num):
        val.append(recv[7 + i])
    if not mute:
        print('读到的寄存器值依次为：', end='')
        for i in range(num):
            print(val[i], end=' ')
        print()
    return val


# 函数功能：写入灵巧手六个电缸数据函数，angleSet设置灵巧手运动角度参数、forceSet设置灵巧手抓握力度参数、speedSet设置灵巧手运动速度参数
# 参数说明：ID为灵巧手对应ID号，str为灵巧手选取设置的参数，val为设置数据
def write6(ser, id, str, val):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet':
        val_reg = []
        for i in range(6):
            val_reg.append(val[i] & 0xFF)
            val_reg.append((val[i] >> 8) & 0xFF)
        writeRegister(ser, id, regdict[str], 12, val_reg)
    else:
        print(
            '函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'，val为长度为6的list，值为0~1000，允许使用-1作为占位符')


# 函数功能：读取灵巧手六个电缸数据函数
# angleSet为灵巧手运动角度参数、forceSet为灵巧手抓握力度参数、speedSet为灵巧手运动速度参数、angleAct为灵巧手角度实际值、forceAct为灵巧手各手指的实际受力值
def read6(ser, id, str):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet' or str == 'angleAct' or str == 'forceAct':
        val = readRegister(ser, id, regdict[str], 12, True)  # 读取
        if len(val) < 12:  # 读取到的数据小于12直接舍弃
            print('没有读到数据')
            return
        val_act = []
        for i in range(6):
            val_act.append((val[2 * i] & 0xFF) + (val[1 + 2 * i] << 8))  #
        if str == 'forceAct':

            benormal(val_act)

        print('读到的值依次为：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()
        return val_act
    elif str == 'errCode' or str == 'statusCode' or str == 'temp':
        val_act = readRegister(ser, id, regdict[str], 6, True)
        if len(val_act) < 6:  # 读取的数据小于6直接舍弃
            print('没有读到数据')
            return
        print('读到的值依次为：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()
    else:
        print(
            '函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')


