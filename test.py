from classic_method import *
from detect import *
import serial
from robotic_systems import Float2Byte
from PyQt5.QtCore import pyqtSignal, QObject
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QListWidget, QListWidgetItem,
    QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QLineEdit,
    QMessageBox,QMenu,QComboBox
)
from PyQt5.QtCore import Qt, QMimeData
from PyQt5.QtGui import QDrag
angles_dict={'0':0,'1':639.86,'2':80.0,'3':0,'4':70.0,'5':140, 'pwm':2500}

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
# ========== 1. 机械臂操作函数（示例）==========
# def change_angle(ax1=0):
#     print(f"[change_angle] ax1 = {ax1}")


def ResetArm():
    print("[ResetArm] Arm has been reset.")


# ========== 2. 自定义小部件 ==========
# 用于在左侧显示："change_angle(ax1= [QLineEdit])"
class ChangeAngleWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("change_angle")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要变换的轴")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.ax2_edit = QLineEdit()
        self.ax2_edit.setPlaceholderText("请输入要变换的角度值")
        self.ax2_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.label_after = QLabel(")")
        font = self.label_before.font()
        font.setPointSize(14)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.ax2_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

    def get_ax1_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

    def get_ax2_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax2_edit.text().strip()

class ChangeCoorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("change_coordinates")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要变换的新坐标值(x,y,z)")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.label_after = QLabel(")")
        font = self.label_before.font()
        font.setPointSize(14)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

    def get_coordinates_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class WaitWindows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("Wait_a_while")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要等待的时间（秒）")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.label_after = QLabel(")")
        font = self.label_before.font()
        font.setPointSize(14)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

    def get_wait_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class PWMwindows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("clamping_force")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入夹爪力度")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.label_after = QLabel(")")
        font = self.label_before.font()
        font.setPointSize(14)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

    def get_pwm_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class Follow_hands(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("user_instructions")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入任何指令")
        self.ax1_edit.setMinimumWidth(20)  # 防止编辑框太窄

        self.label_after = QLabel(")")
        font = self.label_before.font()
        font.setPointSize(14)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

    def get_ins_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()


# ========== 3. 左侧列表(接收拖拽) ==========
class CommandsListWidget(QListWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)                   # 接收拖拽
        self.setDragDropMode(self.InternalMove)     # 支持内部拖拽改变顺序
        self.setContextMenuPolicy(Qt.CustomContextMenu)  # 启用右键菜单
        self.customContextMenuRequested.connect(self.show_context_menu)

    def dragEnterEvent(self, event):
        """
        重载拖拽进入事件，允许外部或内部拖拽进入。
        """
        if event.mimeData().hasText() or event.source() == self:
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event):
        """
        重载拖拽移动事件，允许外部或内部拖拽。
        """
        if event.mimeData().hasText() or event.source() == self:
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        """
        重载放下事件，区分内部拖拽和外部拖入：
        - 内部拖拽：默认行为
        - 外部拖入：添加新项目
        """
        if event.source() == self:
            # 内部拖拽，调用父类默认实现
            super().dropEvent(event)
        elif event.mimeData().hasText():
            # 外部拖拽，添加新动作
            action_name = event.mimeData().text()

            if action_name == "change_angle":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "change_angle")
                widget = ChangeAngleWidget()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "change_coordinates":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "change_coordinates")
                widget = ChangeCoorWidget()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "wait_a_while":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "wait_a_while")
                widget = WaitWindows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "clamping_force":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "clamping_force")
                widget = PWMwindows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "user_instructions":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "user_instructions")
                widget = Follow_hands()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "ResetArm":
                item = QListWidgetItem("ResetArm")
                item.setData(Qt.UserRole, "ResetArm")
                self.addItem(item)

            event.acceptProposedAction()
        else:
            event.ignore()

    def show_context_menu(self, position):
        """显示右键菜单，用于删除选中的动作。"""
        menu = QMenu(self)
        remove_action = menu.addAction("删除")
        action = menu.exec_(self.mapToGlobal(position))
        if action == remove_action:
            selected_items = self.selectedItems()
            for item in selected_items:
                self.takeItem(self.row(item))



# ========== 4. 右侧列表(拖拽源) ==========
class ActionsListWidget(QListWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)             # 允许拖拽
        self.setDragDropMode(self.DragOnly)   # 只能拖出，不能接收
        # 添加可用动作
        self.init_actions()

    def init_actions(self):
        for action_name in ["change_angle", "change_coordinates", "ResetArm", "wait_a_while","clamping_force", "user_instructions"]:
            item = QListWidgetItem(action_name)
            # 设置可被拖拽的标志
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled)
            self.addItem(item)

    def startDrag(self, supportedActions):
        """自定义开始拖拽时的行为，确保带有文本信息。"""
        item = self.currentItem()
        if not item:
            return
        drag = QDrag(self)
        mime_data = QMimeData()
        mime_data.setText(item.text())  # 将当前Item的文本放入mimeData
        drag.setMimeData(mime_data)
        drag.exec_(Qt.CopyAction)       # 这里用复制（也可用MoveAction），看你需求


# ========== 5. 主窗体 ==========
class MainWindow(QMainWindow):
    # 添加一个信号，用于接收实时坐标更新
    update_coordinates_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Control Example")
        self.resize(800, 400)

        # 添加显示机械臂坐标的 QLabel
        self.coordinates_label = QLabel("当前坐标: X=0, Y=0, Z=0", self)
        self.coordinates_label.setAlignment(Qt.AlignLeft)
        self.coordinates_label.setStyleSheet("font-size: 14px; padding: 5px;")
        self.coordinates_label.setFixedHeight(30)

        # 添加右上角的串口选择下拉框和按钮
        self.port_selector = QComboBox(self)
        self.port_selector.addItems(EnumSerial())  # 加载可用串口
        self.port_selector.setFixedWidth(150)

        self.open_port_button = QPushButton("打开串口", self)
        self.open_port_button.clicked.connect(self.open_selected_port)

        # 主容器 + 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # 串口选择布局（放在右上角）
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("选择串口:", self))
        port_layout.addWidget(self.port_selector)
        port_layout.addWidget(self.open_port_button)
        port_layout.addStretch()

        # 左侧：CommandsList + Execute 按钮
        self.commands_list = CommandsListWidget()
        self.execute_button = QPushButton("Execute Actions")
        self.execute_button.clicked.connect(self.execute_actions)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.coordinates_label)
        left_layout.addWidget(self.commands_list)
        left_layout.addWidget(self.execute_button)

        # 右侧：ActionsList
        self.actions_list = ActionsListWidget()

        # 合并布局
        main_layout.addLayout(port_layout)  # 串口选择布局
        main_layout.addLayout(left_layout)
        main_layout.addWidget(self.actions_list)

        # 连接信号与槽，更新坐标显示
        self.update_coordinates_signal.connect(self.update_coordinates)

    def open_selected_port(self):
        """打开选定的串口"""
        selected_port = self.port_selector.currentText()
        if selected_port:
            try:
                StopCom()  # 如果已有串口打开，先关闭
                if OpenCom(selected_port, self):
                    QMessageBox.information(self, "Success", f"成功打开串口: {selected_port}")
                else:
                    QMessageBox.warning(self, "Error", f"无法打开串口: {selected_port}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"打开串口时发生错误: {e}")

    def update_coordinates(self, coordinates_text):
        """更新 QLabel 显示的坐标信息"""
        self.coordinates_label.setText(coordinates_text)

    def execute_actions(self):
        """
        按顺序执行左侧列表中的动作：
          - 若是change_angle，则从绑定的小部件中获取ax1，
            若空或非数字则报错并停止执行。
          - 若是ResetArm，直接执行。
        """
        count = self.commands_list.count()
        if count == 0:
            QMessageBox.information(self, "Info", "请先将动作拖拽到左侧再执行。")
            return

        for i in range(count):
            item = self.commands_list.item(i)
            action_name = item.data(Qt.UserRole)  # 获取存储的动作名称

            if action_name == "change_angle":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_angle' 未找到对应的输入控件，无法执行。"
                    )
                    return

                ax1_str = widget.get_ax1_value()
                ax2_str = widget.get_ax2_value()
                if not ax1_str or not ax2_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_angle' 未填写所有输入！"
                    )
                    return

                try:
                    ax1_val = float(ax1_str)  # 或 int(ax1_str)
                    ax2_val = float(ax2_str)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_angle' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_one_angle(ax1_val, ax2_val)

            elif action_name == "change_coordinates":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_coordinates' 未找到对应的输入控件，无法执行。"
                    )
                    return

                coor_str = widget.get_coordinates_value()
                if not coor_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_coordinates' 未填写所有输入！"
                    )
                    return

                try:
                    coordi_list = coor_str.split(',')
                    coordi_list = [float(i) for i in coordi_list]
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'change_coordinates' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_coordinates(coordi_list)

            elif action_name == "clamping_force":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'clamping_force' 未找到对应的输入控件，无法执行。"
                    )
                    return

                pwm_str = widget.get_pwm_value()
                if not pwm_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'clamping_force' 未填写所有输入！"
                    )
                    return

                try:
                    pwm_val = float(pwm_str)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'clamping_force' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_pwm(pwm_val)

            elif action_name == "wait_a_while":
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'wait_a_while' 未找到对应的输入控件，无法执行。"
                    )
                    return
                wait_time = widget.get_wait_value()
                if not wait_time:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'wait_a_while' 未填写所有输入！"
                    )
                    return

                try:
                    wait_time = float(wait_time)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 'wait_a_while' 存在不是数字的输入！"
                    )
                    return

                # 执行
                time.sleep(wait_time)

            elif action_name == "user_instructions":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 'user_instructions' 未找到对应的输入控件，无法执行。"
                    )
                    return

                ins_str = widget.get_ins_value()
                if not ins_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 'user_instructions' 未填写所有输入！"
                    )
                    return
                if 'follow hand' in ins_str:
                    detect(hCom)
                if 'cup' in ins_str:
                    detect2(hCom)

            elif action_name == "ResetArm":
                ResetArm()

            else:
                print(f"未知动作: {action_name}")

        QMessageBox.information(self, "Info", "已按顺序执行所有动作。")

    def closeEvent(self, event):
        """
        在窗口关闭时执行清理操作
        - 调用 StopCom() 关闭串口
        - 结束后台线程（如果需要）
        """
        try:
            StopCom()  # 调用关闭串口的函数
        except Exception as e:
            print(f"关闭串口时出错: {e}")
        finally:
            event.accept()  # 确保窗口可以正常关闭

class ReadThread(threading.Thread, QObject):
    # 添加信号
    coordinates_signal = pyqtSignal(str)

    def __init__(self, serial_port):
        threading.Thread.__init__(self)
        QObject.__init__(self)
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
        if dat[0] == 206 and dat[10] == 207:
            data_type = dat[9]
            if data_type == 0:
                XP = (dat[1] << 8) | dat[2]
                YP = (dat[3] << 8) | dat[4]
                ZP = (dat[5] << 8) | dat[6]
                a0 = (dat[7] << 8) | dat[8]
            elif data_type == 1:
                a1 = (dat[1] << 8) | dat[2]
                a2 = (dat[3] << 8) | dat[4]
                b0 = (dat[5] << 8) | dat[6]
                b1 = (dat[7] << 8) | dat[8]
            elif data_type == 2:
                aw = (dat[1] << 8) | dat[2]
            elif data_type == 3:
                w0 = (dat[5] << 8) | dat[6]
                w1 = (dat[7] << 8) | dat[8]

            # 处理完一个数据帧后，调整 dat 数组和指针
            if di == 11:
                di = 0
            else:
                dat = dat[11:] + bytearray(11)
                di -= 11

            dx_get += 1

            # 发送信号，更新坐标
            self.coordinates_signal.emit(f"当前坐标: X={XP/10}, Y={YP/10}, Z={ZP/10},a1={a0/100}, a2={a1/100}, a3={a2/100},a4={w0/100},a5={w1/100},a6={aw/100}")

        else:
            # 无效数据帧，调整缓冲区
            dat = dat[1:] + bytearray(1)
            di -= 1
            while di > 1 and dat[0] != 206:
                dat = dat[1:] + bytearray(1)
                di -= 1

    def stop(self):
        self.terminated = True


def EnumSerial():
    import serial.tools.list_ports
    ports = []
    port_list = list(serial.tools.list_ports.comports())
    for port in port_list:
        ports.append(port.device)
    return ports

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

def OpenCom(port_name, main_window):
    global hCom, comm_connect, state, Read232

    if hCom is not None:
        StopCom()

    try:
        hCom = serial.Serial(
            port=port_name,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0
        )
        comm_connect = 1
        state = 5
        Read232 = ReadThread(hCom)

        # 连接线程的信号与主窗口的槽
        Read232.coordinates_signal.connect(main_window.update_coordinates_signal)

        Read232.start()
        return True
    except serial.SerialException as e:
        print(f"无法打开串口 {port_name}: {e}")
        comm_connect = 0
        state = 0
        return False

def change_one_angle(choice, angle):
    time.sleep(1)
    # Step 2: 准备直线插补指令数据
    angles_dict[str(int(choice))]=float(angle)
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 5+48   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 5    # 子指令代码

    spd = 5000/250                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(choice)
    interpolation_command[7:11] = Float2Byte(angle)
    interpolation_command[11:15] = Float2Byte(0)
    interpolation_command[15:19] = Float2Byte(0)
    interpolation_command[19:23] = Float2Byte(0)
    interpolation_command[23:27] = Float2Byte(0)
    interpolation_command[27:31] = Float2Byte(angles_dict['pwm'])
    interpolation_command[39:43] = Float2Byte(0)
    interpolation_command[43:47] = Float2Byte(spd)
    interpolation_command[47] = 239

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")

def change_pwm(force):
    time.sleep(1)
    angles_dict['pwm']=force
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 5+48   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 5    # 子指令代码

    spd = 5000/250                           # 速度（单位：度/秒）

    # 将浮点数转换为字节数组并填充指令帧
    interpolation_command[3:7] = Float2Byte(3.0)
    interpolation_command[7:11] = Float2Byte(0.0)
    interpolation_command[11:15] = Float2Byte(0)
    interpolation_command[15:19] = Float2Byte(0)
    interpolation_command[19:23] = Float2Byte(0)
    interpolation_command[23:27] = Float2Byte(0)
    interpolation_command[27:31] = Float2Byte(angles_dict['pwm'])
    interpolation_command[39:43] = Float2Byte(0)
    interpolation_command[43:47] = Float2Byte(spd)
    interpolation_command[47] = 239

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")


def change_coordinates(coordinates):
    time.sleep(1)
    x,y,z=coordinates

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
    interpolation_command[3:7] = Float2Byte(x)  # X
    interpolation_command[7:11] = Float2Byte(y)  # Y
    interpolation_command[11:15] = Float2Byte(z)  # Z
    interpolation_command[15:19] = Float2Byte(ax4)  # B0
    interpolation_command[19:23] = Float2Byte(angles_dict['4'])  # B1
    interpolation_command[23:27] = Float2Byte(angles_dict['5'])  # W
    interpolation_command[27:31] = Float2Byte(angles_dict['pwm'])  # PWM
    interpolation_command[39:43] = Float2Byte(at)  # 加速度
    interpolation_command[43:47] = Float2Byte(spd)  # 速度
    interpolation_command[47] = 239  # 帧尾

    # 发送插补指令
    print("发送直线插补运动指令...")
    SendDatas(interpolation_command)
    print("指令已发送完成。")

def SendDatas(chStr):
    global hCom, state
    if hCom is not None and hCom.is_open:
        try:
            hCom.write(chStr)
        except serial.SerialException as e:
            print(f"发送数据错误: {e}")
            state = 9  # 发送数据错误
    else:
        print("串口未打开，无法发送数据")
        state = 9  # 发送数据错误

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    # OpenCom("COM7",window)
    window.show()
    sys.exit(app.exec_())

