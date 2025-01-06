import pdb
import re
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
    QMessageBox,QMenu,QComboBox,QStackedWidget
)
from PyQt5.QtCore import Qt, QMimeData
from PyQt5.QtGui import QDrag,QPixmap
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
    calibrate()
    print("[ResetArm] Arm has been reset.")


# ========== 2. 自定义小部件 ==========
# 用于在左侧显示："change_angle(ax1= [QLineEdit])"
class ChangeAngleWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("改变单轴角度")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要变换的轴")
        self.ax1_edit.setMinimumWidth(50)   # 防止编辑框太窄

        self.ax2_edit = QLineEdit()
        self.ax2_edit.setPlaceholderText("请输入要变换的角度值")
        self.ax2_edit.setMinimumWidth(20)   # 防止编辑框太窄

        self.label_after = QLabel(")")

        self.ax1_edit.setStyleSheet("""
            QLineEdit {
                font-size: 14px;
                padding: 5px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #ffffff;
                color: #333333;
            }
            QLineEdit:focus {
                border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                background-color: #f9f9f9;
            }
        """)

        self.ax2_edit.setStyleSheet("""
            QLineEdit {
                font-size: 14px;
                padding: 5px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #ffffff;
                color: #333333;
            }
            QLineEdit:focus {
                border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                background-color: #f9f9f9;
            }
        """)

        font = self.label_before.font()
        font.setPointSize(20)  # 设置字体大小为14
        font.setBold(False)

        self.label_before.setFont(font)
        self.label_after.setFont(font)
        layout = QHBoxLayout(self)
        layout.addWidget(self.label_before)
        layout.addWidget(self.ax1_edit)
        layout.addWidget(self.ax2_edit)
        layout.addWidget(self.label_after)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(9)

    def get_ax1_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

    def get_ax2_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax2_edit.text().strip()

class ChangeCoorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("改变坐标")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要变换的新坐标值(x,y,z)")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

    def get_coordinates_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class WaitWindows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("停止若干秒")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要等待的时间（秒）")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

    def get_wait_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class PWMwindows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("改变夹爪力度")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入夹爪力度")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

    def get_pwm_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class Visualwindows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("开启摄像头进行目标检测")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入要检测的物体")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

    def get_obj_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()

class endpoint_windows(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("改变夹爪终端角度")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入终端角度")
        self.ax1_edit.setMinimumWidth(20)   # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

    def get_end_value(self):
        """获取用户输入的 ax1（字符串），再由外部转数值。"""
        return self.ax1_edit.text().strip()


class Follow_hands(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label_before = QLabel("视觉大语言模型理解指令")
        self.ax1_edit = QLineEdit()
        self.ax1_edit.setPlaceholderText("请输入任何指令")
        self.ax1_edit.setMinimumWidth(20)  # 防止编辑框太窄
        self.ax1_edit.setStyleSheet("""
                    QLineEdit {
                        font-size: 14px;
                        padding: 5px;
                        border: 1px solid #cccccc;
                        border-radius: 4px;
                        background-color: #ffffff;
                        color: #333333;
                    }
                    QLineEdit:focus {
                        border: 1px solid #4caf50;  /* 聚焦时的边框颜色 */
                        background-color: #f9f9f9;
                    }
                """)
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
        layout.setSpacing(9)

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

            if action_name == "改变单轴角度":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "改变单轴角度")
                widget = ChangeAngleWidget()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "改变坐标":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "改变坐标")
                widget = ChangeCoorWidget()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "停止若干秒":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "停止若干秒")
                widget = WaitWindows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "改变夹爪力度":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "改变夹爪力度")
                widget = PWMwindows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "开启摄像头进行目标检测":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "开启摄像头进行目标检测")
                widget = Visualwindows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)


            elif action_name == "改变夹爪终端角度":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "改变夹爪终端角度")
                widget = endpoint_windows()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)

            elif action_name == "视觉大语言模型理解指令":
                item = QListWidgetItem()
                item.setData(Qt.UserRole, "视觉大语言模型理解指令")
                widget = Follow_hands()
                self.addItem(item)
                item.setSizeHint(widget.sizeHint())
                self.setItemWidget(item, widget)


            elif action_name == "机械臂校准":
                item = QListWidgetItem("机械臂校准")
                item.setData(Qt.UserRole, "机械臂校准")
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

        # 添加样式表
        self.setStyleSheet("""
            QListWidget {
                background-color: #f2f2f2;  /* 浅灰色背景 */
                border: 1px solid #cccccc;  /* 浅灰色边框 */
                border-radius: 6px;
                padding: 5px;
            }
            QListWidget::item {
                color: #333333;  /* 深灰色文字 */
                background-color: #e6e6e6;  /* 浅灰色背景 */
                border: 1px solid #d9d9d9;  /* 更浅的灰色边框 */
                border-radius: 4px;
                padding: 8px;
                margin: 4px;
            }
            QListWidget::item:hover {
                background-color: #cccccc;  /* 鼠标悬停时浅灰 */
                border: 1px solid #bfbfbf;  /* 鼠标悬停时边框颜色 */
            }
            QListWidget::item:selected {
                color: white;
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, 
                                                 stop:0 #4caf50, stop:1 #388e3c); /* 绿色渐变背景 */
                border: 1px solid #6a6a6a;
            }
        """)

        # 添加可用动作
        self.init_actions()

    def init_actions(self):
        for action_name in ["改变单轴角度", "改变坐标", "机械臂校准", "停止若干秒","改变夹爪力度", "视觉大语言模型理解指令","开启摄像头进行目标检测","改变夹爪终端角度"]:
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
        self.setWindowTitle("CogniCore Robot Control")
        self.resize(1300, 800)

        # 应用样式表
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QLabel {
                font-size: 14px;
                color: #333333;
            }
            QComboBox {
                font-size: 14px;
                padding: 5px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #ffffff;
            }
            QPushButton {
                font-size: 14px;
                padding: 5px 10px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #4caf50;
                color: white;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QListWidget {
                font-size: 14px;
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 4px;
            }
        """)

        # 创建主界面和子界面
        self.main_page = self.create_main_page()
        self.sub_page = self.create_sub_page()

        # 创建 QStackedWidget
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.main_page)  # 主界面
        self.stacked_widget.addWidget(self.sub_page)  # 子界面

        # 左侧切换按钮
        self.main_page_button = QPushButton("主界面")
        self.main_page_button.clicked.connect(lambda: self.switch_page(0))

        self.sub_page_button = QPushButton("示教功能")
        self.sub_page_button.clicked.connect(lambda: self.switch_page(1))

        # 左侧按钮布局
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.main_page_button)
        button_layout.addWidget(self.sub_page_button)
        button_layout.addStretch()

        # 添加公司 Logo 到按钮布局的底部
        self.logo_label = QLabel(self)
        self.logo_label.setPixmap(QPixmap("logo1.png").scaled(150, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setStyleSheet("margin-top: 10px;")
        button_layout.addWidget(self.logo_label, alignment=Qt.AlignBottom)

        # 主布局
        main_layout = QHBoxLayout()
        main_layout.addLayout(button_layout, stretch=1)  # 左侧切换按钮
        main_layout.addWidget(self.stacked_widget, stretch=4)  # 主体内容

        # 设置中央小部件
        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # 连接信号与槽
        self.update_coordinates_signal.connect(self.update_coordinates)

    def create_main_page(self):
        """
        创建主界面
        """
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)

        # 串口选择布局
        port_layout = QHBoxLayout()
        port_label = QLabel("选择串口:", self)
        port_label.setFixedWidth(80)
        self.port_selector = QComboBox(self)
        self.port_selector.addItems(EnumSerial())
        self.port_selector.setFixedWidth(150)
        self.open_port_button = QPushButton("打开串口", self)
        self.open_port_button.clicked.connect(self.open_selected_port)
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_selector)
        port_layout.addWidget(self.open_port_button)
        port_layout.addStretch()

        # 左侧：CommandsList + Execute 按钮
        self.coordinates_label = QLabel("当前坐标: X=0, Y=0, Z=0", self)
        self.coordinates_label.setAlignment(Qt.AlignCenter)  # 居中显示
        self.coordinates_label.setStyleSheet("""
            QLabel {
                font-size: 16px;             /* 字体大小 */
                font-weight: bold;           /* 加粗字体 */
                color: #0084E9;              /* 字体颜色：绿色 */
                border: 2px solid #4caf50;   /* 边框：绿色 */
                border-radius: 10px;         /* 圆角边框 */
                padding: 10px;               /* 内边距 */
                background-color: #f0f0f0;   /* 背景颜色：浅灰 */
            }
        """)
        self.coordinates_label.setFixedHeight(50)  # 设置固定高度
        self.commands_list = CommandsListWidget()
        self.execute_button = QPushButton("执行动作")
        self.execute_button.clicked.connect(self.execute_actions)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.coordinates_label)
        action_txt=QLabel("已选动作列表:", self)
        action_txt.setStyleSheet("""
            QLabel {
                font-size: 16px;             /* 字体大小 */
                font-weight: bold;           /* 加粗字体 */
            }
        """)
        left_layout.addWidget(action_txt)
        left_layout.addWidget(self.commands_list)
        left_layout.addWidget(self.execute_button)

        # 右侧：ActionsList
        self.actions_list = ActionsListWidget()
        right_layout = QVBoxLayout()
        action_txt=QLabel("动作列表:", self)
        action_txt.setStyleSheet("""
                    QLabel {
                        font-size: 16px;             /* 字体大小 */
                        font-weight: bold;           /* 加粗字体 */
                    }
                """)
        right_layout.addWidget(action_txt)
        right_layout.addWidget(self.actions_list)

        # 主布局合并
        main_split_layout = QHBoxLayout()
        main_split_layout.addLayout(left_layout, stretch=2)
        main_split_layout.addLayout(right_layout, stretch=1)

        main_layout.addLayout(port_layout)
        main_layout.addLayout(main_split_layout)


        return main_widget

    def start_execution(self):
        """
        模拟依次移动到记录的坐标
        """
        # 检查列表是否有记录
        if self.coordinates_list.count() == 0:
            QMessageBox.warning(self, "警告", "坐标列表为空，无法执行！")
            return

        # 依次移动到坐标
        for index in range(self.coordinates_list.count()):
            item = self.coordinates_list.item(index)
            coordinate_text = item.text()
            # 正则表达式提取 X, Y, Z 和 a1-a6 的值
            pattern = r"(X|Y|Z|a[1-6])=(-?\d+\.?\d*)"
            matches = re.findall(pattern, coordinate_text)

            # 将结果存储到字典中
            coordinates = {key: float(value) for key, value in matches}
            change_angle(coordinates['a1'],coordinates['a2'],coordinates['a3'],coordinates['a4'],coordinates['a5'],coordinates['a6'],2000)
            # 模拟移动操作
            print("执行中", f"正在移动到: {coordinate_text}")

        # 动态添加“重置”按钮
        if not hasattr(self, "reset_button"):
            self.reset_button = QPushButton("重置")
            self.reset_button.setFixedSize(100, 30)
            self.reset_button.setStyleSheet(
                "background-color: #f44336; color: white; border-radius: 5px;")
            self.reset_button.clicked.connect(self.reset_to_initial_state)
            self.teaching_buttons_layout.addWidget(self.reset_button)
        QMessageBox.information(self, "完成", "已完成所有坐标的执行！")


    def reset_to_initial_state(self):
        """
        将界面恢复到初始状态
        """
        # 清空坐标列表
        self.coordinates_list.clear()

        # 删除“开始执行”和“重置”按钮
        if hasattr(self, "start_execution_button"):
            self.teaching_buttons_layout.removeWidget(self.start_execution_button)
            self.start_execution_button.deleteLater()
            del self.start_execution_button

        if hasattr(self, "reset_button"):
            self.teaching_buttons_layout.removeWidget(self.reset_button)
            self.reset_button.deleteLater()
            del self.reset_button

        # # 恢复“开始示教”按钮
        # self.start_teaching_button = QPushButton("开始示教")
        # self.start_teaching_button.setFixedSize(100, 30)
        # self.start_teaching_button.setStyleSheet(
        #     "background-color: #2196f3; color: white; border-radius: 5px;")
        # self.start_teaching_button.clicked.connect(self.show_teaching_buttons)
        # self.teaching_buttons_layout.addWidget(self.start_teaching_button)

        QMessageBox.information(self, "重置", "界面已恢复到初始状态！")

    def create_sub_page(self):
        """
        创建子界面
        """
        sub_widget = QWidget()
        self.sub_layout = QVBoxLayout(sub_widget)
        self.sub_coordinates_label = QLabel("当前坐标: X=0, Y=0, Z=0", self)
        self.sub_coordinates_label.setAlignment(Qt.AlignCenter)  # 居中显示
        self.sub_coordinates_label.setStyleSheet("""
                    QLabel {
                        font-size: 16px;             /* 字体大小 */
                        font-weight: bold;           /* 加粗字体 */
                        color: #0084E9;              /* 字体颜色：绿色 */
                        border: 2px solid #4caf50;   /* 边框：绿色 */
                        border-radius: 10px;         /* 圆角边框 */
                        padding: 10px;               /* 内边距 */
                        background-color: #f0f0f0;   /* 背景颜色：浅灰 */
                    }
                """)

        # 子界面标题
        label = QLabel("示教功能界面")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 16px; color: #333333;")
        self.sub_layout.addWidget(label)
        self.sub_layout.addWidget(self.sub_coordinates_label)

        # 添加 "开始示教" 按钮
        self.start_teaching_button = QPushButton("开始示教")
        self.start_teaching_button.setStyleSheet(
            "background-color: #2196f3; color: white; padding: 10px; border-radius: 5px;")
        self.start_teaching_button.clicked.connect(self.show_teaching_buttons)  # 连接功能
        self.sub_layout.addWidget(self.start_teaching_button)

        # 添加占位符，用于动态添加按钮和列表
        self.teaching_buttons_layout = QVBoxLayout()
        self.sub_layout.addLayout(self.teaching_buttons_layout)

        # 添加记录的坐标列表
        self.coordinates_list = QListWidget()
        self.coordinates_list.setStyleSheet("""
            QListWidget {
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 4px;
                font-size: 14px;
                color: #333333;
                padding: 5px;
            }
            QListWidget::item {
                margin: 2px;
                padding: 5px;
            }
        """)
        self.sub_layout.addWidget(self.coordinates_list)

        self.sub_layout.addStretch()
        return sub_widget

    def show_teaching_buttons(self):
        """
        显示“记录坐标”和“完成记录”按钮
        """
        # 检查是否已添加，避免重复添加
        if hasattr(self, "record_coordinate_button") and hasattr(self, "finish_recording_button"):
            return

        # 创建水平布局，用于并排放置按钮
        button_layout = QHBoxLayout()

        # 创建 "记录坐标" 按钮
        self.record_coordinate_button = QPushButton("记录坐标")
        self.record_coordinate_button.setFixedSize(100, 30)  # 设置固定尺寸
        self.record_coordinate_button.setStyleSheet(
            "background-color: #4caf50; color: white; border-radius: 5px;")
        self.record_coordinate_button.clicked.connect(self.record_coordinate)  # 连接功能
        button_layout.addWidget(self.record_coordinate_button)

        # 创建 "完成记录" 按钮
        self.finish_recording_button = QPushButton("完成记录")
        self.finish_recording_button.setFixedSize(100, 30)  # 设置固定尺寸
        self.finish_recording_button.setStyleSheet(
            "background-color: #f44336; color: white; border-radius: 5px;")
        self.finish_recording_button.clicked.connect(self.finish_recording)  # 连接功能
        button_layout.addWidget(self.finish_recording_button)

        # 将水平布局添加到教学按钮的主布局
        self.teaching_buttons_layout.addLayout(button_layout)

    def record_coordinate(self):
        """
        模拟记录当前坐标功能
        """
        # 获取当前坐标（从子界面坐标显示标签中获取）
        current_coordinates = self.sub_coordinates_label.text()

        # 添加到列表
        self.coordinates_list.addItem(current_coordinates)

        # 显示提示信息
        QMessageBox.information(self, "记录坐标", f"已记录坐标: {current_coordinates}")

    def finish_recording(self):
        """
        模拟完成记录功能，显示“开始执行”按钮
        """
        # 在实际应用中，可以保存记录的所有坐标数据
        QMessageBox.information(self, "完成记录", "记录完成！")

        # 隐藏 "记录坐标" 和 "完成记录" 按钮
        self.teaching_buttons_layout.removeWidget(self.record_coordinate_button)
        self.teaching_buttons_layout.removeWidget(self.finish_recording_button)
        self.record_coordinate_button.deleteLater()
        self.finish_recording_button.deleteLater()
        del self.record_coordinate_button
        del self.finish_recording_button

        # 添加 "开始执行" 按钮
        self.start_execution_button = QPushButton("开始执行")
        self.start_execution_button.setFixedSize(100, 30)  # 设置按钮尺寸
        self.start_execution_button.setStyleSheet(
            "background-color: #2196f3; color: white; border-radius: 5px;")
        self.start_execution_button.clicked.connect(self.start_execution)  # 连接功能
        self.teaching_buttons_layout.addWidget(self.start_execution_button)

    def switch_page(self, index):
        """
        切换页面
        """
        self.stacked_widget.setCurrentIndex(index)

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
        self.sub_coordinates_label.setText(coordinates_text)

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

            if action_name == "改变单轴角度":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变单轴角度' 未找到对应的输入控件，无法执行。"
                    )
                    return

                ax1_str = widget.get_ax1_value()
                ax2_str = widget.get_ax2_value()
                if not ax1_str or not ax2_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变单轴角度' 未填写所有输入！"
                    )
                    return

                try:
                    ax1_val = float(ax1_str)  # 或 int(ax1_str)
                    ax2_val = float(ax2_str)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变单轴角度' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_one_angle(ax1_val, ax2_val)

            elif action_name == "改变坐标":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变坐标' 未找到对应的输入控件，无法执行。"
                    )
                    return

                coor_str = widget.get_coordinates_value()
                if not coor_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变坐标' 未填写所有输入！"
                    )
                    return

                try:
                    coordi_list = coor_str.split(',')
                    coordi_list = [float(i) for i in coordi_list]
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变坐标' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_coordinates(coordi_list)

            elif action_name == "改变夹爪力度":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪力度' 未找到对应的输入控件，无法执行。"
                    )
                    return

                pwm_str = widget.get_pwm_value()
                if not pwm_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪力度' 未填写所有输入！"
                    )
                    return

                try:
                    pwm_val = float(pwm_str)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪力度' 存在不是数字的输入！"
                    )
                    return

                # 执行
                change_pwm(pwm_val)

            elif action_name == "改变夹爪终端角度":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪终端角度' 未找到对应的输入控件，无法执行。"
                    )
                    return

                end_angle = widget.get_end_value()
                if not end_angle:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪终端角度' 未填写所有输入！"
                    )
                    return

                try:
                    end_value = float(end_angle)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '改变夹爪终端角度' 存在不是数字的输入！"
                    )
                    return

                # 执行
                angles_dict['4']=end_value

            elif action_name == "停止若干秒":
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '停止若干秒' 未找到对应的输入控件，无法执行。"
                    )
                    return
                wait_time = widget.get_wait_value()
                if not wait_time:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '停止若干秒' 未填写所有输入！"
                    )
                    return

                try:
                    wait_time = float(wait_time)  # 或 int(ax1_str)
                except ValueError:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i+1} 个动作 '停止若干秒' 存在不是数字的输入！"
                    )
                    return

                # 执行
                time.sleep(wait_time)

            elif action_name == "视觉大语言模型理解指令":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 '视觉大语言模型理解指令' 未找到对应的输入控件，无法执行。"
                    )
                    return

                ins_str = widget.get_ins_value()
                if not ins_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 '视觉大语言模型理解指令' 未填写所有输入！"
                    )
                    return
                if 'follow hand' or '跟随手' in ins_str:
                    detect(hCom)
                if 'cup' or '把水从瓶子倒入杯子里' in ins_str:
                    detect2(hCom,[])

            elif action_name == "开启摄像头进行目标检测":
                # 读取小部件
                widget = self.commands_list.itemWidget(item)
                if not widget:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 '开启摄像头进行目标检测' 未找到对应的输入控件，无法执行。"
                    )
                    return

                obj_str = widget.get_obj_value()
                if not obj_str:
                    QMessageBox.warning(
                        self, "Warning",
                        f"第 {i + 1} 个动作 '开启摄像头进行目标检测' 未填写所有输入！"
                    )
                    return
                obj_list = obj_str.split(',')
                detect2(hCom, obj_list)


            elif action_name == "机械臂校准":
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

def change_angle(ax1, ax2, ax3, ax4, ax5, ax6, pmw):
    if ax1>600:
        ax1=ax1-654.36 - 1
    if ax2>600:
        ax2 = ax2 - 654.36 - 1
    if ax4>600:
        ax4 = ax4 - 654.36 - 1

    # Step 2: 准备直线插补指令数据
    interpolation_command = bytearray(48)
    interpolation_command[0] = 238  # 帧头
    interpolation_command[1] = 51   # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 1    # 子指令代码

    # 填充直线插补指令数据字段
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

def change_pwm(force):
    time.sleep(1)
    force=max(1000,force)
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

def calibrate():
    time.sleep(1)
    interpolation_command = bytearray(48)
    interpolation_command[0] = 252  # 帧头
    interpolation_command[1] = 12  # 插补指令（ASCII '3' + 48）
    interpolation_command[2] = 5    # 子指令代码
    interpolation_command[47] = 253 # 子指令代码

    # 发送插补指令
    print("发送校准指令...")
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
    window.show()
    sys.exit(app.exec_())

