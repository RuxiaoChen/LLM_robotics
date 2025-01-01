import threading
import serial
import time
import struct
import struct
import time

XP, YP, ZP = 0, 0, 0   # 机械臂坐标
a0, a1, a2 = 0, 0, 0   # 机械臂轴角度
b0, b1, aw = 0, 0, 0   # 姿态角
wk_X0, wk_Y0, wk_Z0 = 0, 0, 0  # 工作台坐标
dx_get = 0             # 数据接收标志
w0, w1=0,0

# 定义全局变量和状态信息
hCom = None  # 串口对象
comm_connect = 0  # 连接状态变量，0表示断开，1表示接通
state = 100  # 串口控制消息变量，100表示没有消息

# 数据缓冲区和指针
dat = bytearray(1100)
di = 0  # 数组指针

# 定义其他全局变量（根据需要，可以在后续添加）

# 读取线程
class ReadThread(threading.Thread):
    def __init__(self, serial_port):
        threading.Thread.__init__(self)
        self.serial_port = serial_port
        self.terminated = False

    def run(self):
        global di, dat, XP, YP, ZP, a0, a1, a2, b0, b1, aw, dx_get
        while not self.terminated:
            if self.serial_port.in_waiting > 0:
                try:
                    # 读取一个字节
                    read_byte = self.serial_port.read(1)
                    # 数据处理逻辑
                    if di == 0:
                        if read_byte[0] == 206:
                            dat[di] = read_byte[0]
                            di += 1
                    else:
                        if di < 1100:
                            dat[di] = read_byte[0]
                            di += 1

                    if di >= 11:
                        # 数据帧解析
                        self.parse_data_frame()
                except Exception as e:
                    print(f"读取串口数据时发生错误: {e}")
                    self.terminated = True

    def parse_data_frame(self):
        global di, dat, XP, YP, ZP, a0, a1, a2, b0, b1, aw, dx_get, w0, w1
        # 在这里添加数据帧的解析逻辑，类似于C++代码中的实现
        # 解析完成后，调整dat数组和di指针
        if dat[0] == 206 and dat[10] == 207:
            # 根据dat[9]的值，解析不同的数据类型
            data_type = dat[9]
            if data_type == 0:
                # 解析X, Y, Z坐标和a0
                XP = (dat[1] << 8) | dat[2]
                YP = (dat[3] << 8) | dat[4]
                ZP = (dat[5] << 8) | dat[6]
                a0 = (dat[7] << 8) | dat[8]
                # 在这里可以将解析后的数据进行处理或存储
            elif data_type == 1:
                # 解析a1, a2, b0, b1
                a1 = (dat[1] << 8) | dat[2]
                a2 = (dat[3] << 8) | dat[4]
                b0 = (dat[5] << 8) | dat[6]
                b1 = (dat[7] << 8) | dat[8]
            elif dat[9] == 2:  # 姿态数据帧
                aw = (dat[1] << 8) | dat[2]
            elif dat[9] == 3:  # 数据类型标识为 3
                # 解析 w0 和 w1
                w0 = (dat[5] << 8) | dat[6]  # 高字节左移 8 位后与低字节拼接
                w1 = (dat[7] << 8) | dat[8]  # 高字节左移 8 位后与低字节拼接

            # 处理完一个数据帧后，调整dat数组和di指针
            if di == 11:
                di = 0
            else:
                dat = dat[11:] + bytearray(11)
                di -= 11
        else:
            # 头或尾帧错误，调整dat数组和di指针
            dat = dat[1:] + bytearray(1)
            di -= 1
            while di > 1 and dat[0] != 206:
                dat = dat[1:] + bytearray(1)
                di -= 1

        dx_get += 1

    def stop(self):
        self.terminated = True

# 打开串口函数
def OpenCom(port_name):
    global hCom, comm_connect, state, Read232

    if hCom is not None:
        # 如果串口已打开，先关闭
        StopCom()

    try:
        hCom = serial.Serial(
            port=port_name,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0  # 非阻塞模式
        )
        comm_connect = 1
        state = 5  # 串口打开成功
        # 创建并启动读取线程
        Read232 = ReadThread(hCom)
        Read232.start()
        return True
    except serial.SerialException as e:
        print(f"无法打开串口 {port_name}: {e}")
        comm_connect = 0
        state = 0  # 未能打开串口
        return False

# 发送数据函数
def SendDatas(chStr,hCom1):
    try:
        hCom1.write(chStr)
    except serial.SerialException as e:
        print(f"发送数据错误: {e}")
        state = 9  # 发送数据错误


# 关闭串口函数
def StopCom():
    global hCom, comm_connect, state, Read232

    if hCom is not None:
        try:
            if Read232.is_alive():
                Read232.stop()
                Read232.join()
            hCom.close()
            state = 6  # 串口关闭成功
            comm_connect = 0
            hCom = None
            return True
        except Exception as e:
            print(f"关闭串口时发生错误: {e}")
            state = 7  # 串口未能关闭
            return False
    else:
        state = 8  # 串口已关闭
        return True

# 枚举可用的串口
def EnumSerial():
    import serial.tools.list_ports
    ports = []
    port_list = list(serial.tools.list_ports.comports())
    for port in port_list:
        ports.append(port.device)
    return ports


# 浮点数转字节数组函数
def Float2Byte(floatNum):
    return struct.pack('<f', floatNum)

def ResetArm():
    # 构造复位指令的字节数组
    reset_command = bytearray(48)
    reset_command[0] = 252  # 帧头
    reset_command[1] = 12   # 指令代码
    reset_command[2] = 3    # 子指令代码，复位
    reset_command[47] = 253 # 帧尾

    # 发送复位指令
    print("发送机械臂复位指令...")
    SendDatas(reset_command)
    print("机械臂复位指令已发送。")

# 实现 TForm1::Button8Click 的功能
def Button8Click(ax1, ax2, ax3, ax5,ax6,pmw, hCom):
    # # Step 1: 设置机械臂模式为模式0（垂直手腕结构）
    # mode_set_command = bytearray(48)
    # mode_set_command[0] = 251  # 帧头
    # mode_set_command[1] = 30   # 指令代码
    # mode_set_command[2] = 9    # 子指令
    # mode_set_command[3] = 0    # 模式0：垂直手腕结构   模式9：垂直手腕结构     5:强水平手腕
    # mode_set_command[47] = 253  # 帧尾
    #
    # print("发送机械臂模式设置指令...")
    # SendDatas(mode_set_command)  # 发送模式设置指令
    # time.sleep(0.05)  # 等待50毫秒

    # Step 2: 准备直线插补指令数据
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    # interpolation_command[1] = 51   # 插补指令（ASCII '3' + 48）
    interpolation_command[1] = 49
    interpolation_command[2] = 1    # 子指令代码

    # 填充直线插补指令数据字段
    # ax1, ax2, ax3 = 334.0, 0.0, 417.5  # X, Y, Z 目标坐标（单位：mm）
    # ax4, ax5, ax6 = 0.0, 90.0, 150.0      # B0, B1, W 姿态角（单位：度）
    ax4=0.0
    # pmw = 1150                        # PWM 信号
    at = 150                          # 加速度
    spd = 5500                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(ax1)  # X
    interpolation_command[7:11] = Float2Byte(ax2)  # Y
    interpolation_command[11:15] = Float2Byte(ax3)  # Z
    interpolation_command[15:19] = Float2Byte(ax4)  # B0
    interpolation_command[19:23] = Float2Byte(ax5)  # B1
    interpolation_command[23:27] = Float2Byte(ax6)  # W
    interpolation_command[27:31] = Float2Byte(pmw)  # PWM
    interpolation_command[39:43] = Float2Byte(at)  # 加速度
    interpolation_command[43:47] = Float2Byte(spd)  # 速度
    interpolation_command[47] = 239  # 帧尾

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command,hCom)
    print("指令已发送完成。")

def change_one_angle(choice, angle,pwm):

    # Step 2: 准备直线插补指令数据
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 5+48   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 5    # 子指令代码

    # 填充直线插补指令数据字段
    # ax1, ax2, ax3 = 334.0, 0.0, 417.5  # X, Y, Z 目标坐标（单位：mm）
    # ax4, ax5, ax6 = 0.0, 90.0, 150.0      # B0, B1, W 姿态角（单位：度）
    # ax4=0.0
    # pmw = 1150                        # PWM 信号
    spd = 5000/300                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(choice)  # X
    interpolation_command[7:11] = Float2Byte(angle)  # Y
    interpolation_command[11:15] = Float2Byte(0)  # Z
    interpolation_command[15:19] = Float2Byte(0)  # B0
    interpolation_command[19:23] = Float2Byte(0)  # B1
    interpolation_command[23:27] = Float2Byte(0)  # W
    interpolation_command[27:31] = Float2Byte(pwm)  # PWM
    interpolation_command[39:43] = Float2Byte(0)  # 加速度
    interpolation_command[43:47] = Float2Byte(spd)  # 速度
    interpolation_command[47] = 239  # 帧尾

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")

def change_angle(ax1, ax2, ax3, ax4, ax5,ax6,pmw):
    # Step 1: 设置机械臂模式为模式0（垂直手腕结构）
    # mode_set_command = bytearray(48)
    # mode_set_command[0] = 251  # 帧头
    # mode_set_command[1] = 30   # 指令代码
    # mode_set_command[2] = 9    # 子指令
    # mode_set_command[3] = 9    # 模式0：垂直手腕结构   模式9：垂直手腕结构     5:强水平手腕
    # mode_set_command[47] = 253  # 帧尾
    #
    # print("发送机械臂模式设置指令...")
    # SendDatas(mode_set_command)  # 发送模式设置指令
    # time.sleep(0.05)  # 等待50毫秒
    if ax1>600:
        ax1=ax1-654.36 - 1
    if ax2>600:
        ax2 = ax2 - 654.36 - 1

    # Step 2: 准备直线插补指令数据
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 51   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 1    # 子指令代码

    # 填充直线插补指令数据字段
    # ax1, ax2, ax3 = 334.0, 0.0, 417.5  # X, Y, Z 目标坐标（单位：mm）
    # ax4, ax5, ax6 = 0.0, 90.0, 150.0      # B0, B1, W 姿态角（单位：度）
    # ax4=0.0
    # pmw = 1150                        # PWM 信号
    at = 1500/400                           # 加速度
    spd = 5000/300                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(ax1)  # X
    interpolation_command[7:11] = Float2Byte(ax2)  # Y
    interpolation_command[11:15] = Float2Byte(ax3)  # Z
    interpolation_command[15:19] = Float2Byte(ax4)  # B0
    interpolation_command[19:23] = Float2Byte(ax5)  # B1
    interpolation_command[23:27] = Float2Byte(ax6)  # W
    interpolation_command[27:31] = Float2Byte(pmw)  # PWM
    interpolation_command[39:43] = Float2Byte(at)  # 加速度
    interpolation_command[43:47] = Float2Byte(spd)  # 速度
    interpolation_command[47] = 239  # 帧尾

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")

def set_change_angle(ax1, ax2, ax3, ax4, ax5,ax6,pmw):
    # Step 1: 设置机械臂模式为模式0（垂直手腕结构）
    mode_set_command = bytearray(48)
    mode_set_command[0] = 251  # 帧头
    mode_set_command[1] = 30   # 指令代码
    mode_set_command[2] = 9    # 子指令
    mode_set_command[3] = 9    # 模式0：垂直手腕结构   模式9：垂直手腕结构     5:强水平手腕
    mode_set_command[47] = 253  # 帧尾

    print("发送机械臂模式设置指令...")
    SendDatas(mode_set_command)  # 发送模式设置指令
    time.sleep(0.05)  # 等待50毫秒

    # Step 2: 准备直线插补指令数据
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 51   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 1    # 子指令代码

    # 填充直线插补指令数据字段
    # ax1, ax2, ax3 = 334.0, 0.0, 417.5  # X, Y, Z 目标坐标（单位：mm）
    # ax4, ax5, ax6 = 0.0, 90.0, 150.0      # B0, B1, W 姿态角（单位：度）
    # ax4=0.0
    # pmw = 1150                        # PWM 信号
    at = 1500/100                           # 加速度
    spd = 5000/100                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(ax1)  # X
    interpolation_command[7:11] = Float2Byte(ax2)  # Y
    interpolation_command[11:15] = Float2Byte(ax3)  # Z
    interpolation_command[15:19] = Float2Byte(ax4)  # B0
    interpolation_command[19:23] = Float2Byte(ax5)  # B1
    interpolation_command[23:27] = Float2Byte(ax6)  # W
    interpolation_command[27:31] = Float2Byte(pmw)  # PWM
    interpolation_command[39:43] = Float2Byte(at)  # 加速度
    interpolation_command[43:47] = Float2Byte(spd)  # 速度
    interpolation_command[47] = 239  # 帧尾

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")


# # # 示例使用
if __name__ == "__main__":
    available_ports = EnumSerial()
    print("可用的串口:")
    for port in available_ports:
        print(port)
#
    # 打开串口（请根据实际情况修改端口名称）
    if OpenCom("COM7"):
        print("串口已打开")
        # ResetArm()
        time.sleep(0.5)
        # change_one_angle(0,-10,2500)
        # 发送数据示例
        change_angle(0,639.79,80.0,0.0,70.0, 140.0,1800)     # if>600, =654.36+1+x  ,x is ax2
        # for i in range(10):
        #     Button8Click(360.0, 0.0+i*10, 260.0+i*10, 60.0, 0.0, 2100)
        #     time.sleep(0.01)
        # Button8Click(455.0, 0.0, 250.0, 70.0, 0.0, 2500)
        # Button8Click(360.0, 0.0, 350.0, 60.0, 0.0, 2100)
        while True:
            a0_old=a0
            if a0-a0_old<0.001:
                break
        # # 关闭串口
        # StopCom()
        # print("串口已关闭")
        if dx_get > 0:
            print(f"接收到数据: X={XP}, Y={YP}, Z={ZP}")
            print(f"接收到数据: a1={a0}, a2={a1}, a3={a2},a4={w0},a5={w1},a6={aw}")
            dx_get = 0
            time.sleep(0.1)
        StopCom()


    else:
        print("无法打开指定的串口")
