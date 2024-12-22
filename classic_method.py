import pdb
import time

from openai import OpenAI
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from robotic_systems import *
import robotic_systems as rbot
from collections import defaultdict, deque
import os
import base64
import io
from PIL import Image
import yaml

this_port = "COM8"  # 根据实际情况更改
path = 'tasks.yaml'
with open(path, 'r', encoding='utf-8') as file:
    tasks = yaml.safe_load(file)


os.environ['OPENAI_API_KEY'] = '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'

client = OpenAI()

recent_positions = defaultdict(lambda: deque(maxlen=50))
# 双目相机参数
B = 60.0   # 基线 mm
f = 3.0    # 焦距 mm
sensor_width = 10.4  # mm
image_width = 1920   # 单眼图像宽度像素
pixel_size = 50/178

device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Load the YOLO11 model
model = YOLO("yolo11n-seg.pt").to(device)
model.fuse()  # 对模型进行融合优化
desired_classes = [39, 67, 41]



# 打开双目摄像头
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

cv2.namedWindow("YOLO11 Tracking Left")
cv2.namedWindow("Right Image")

scale_factor = 0.5
frame_count = 0
N = 5  # 每隔 N 帧进行一次检测

left_boxes = []
right_boxes = []

def frame_to_base64(frame):
    # frame为OpenCV的BGR图像格式
    # 将frame转换为PNG并编码为base64
    # 先将frame从BGR转换为RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # 使用PIL转换为图像对象
    pil_img = Image.fromarray(rgb_frame)
    # 将图像保存到内存缓冲
    buffered = io.BytesIO()
    pil_img.save(buffered, format="PNG")
    # 获取二进制数据
    img_bytes = buffered.getvalue()
    # base64编码
    base64_str = base64.b64encode(img_bytes).decode('utf-8')
    return base64_str

def convert_coordinates(obj):
    x = float(obj[obj.find('X='):obj.find('mm')][2:])  # z
    y = float(obj[obj.find('Y='):obj.find('mm', obj.find('mm') + 1)][2:])  # x
    z = float(obj[obj.find('Z='):][2:-2])  # y
    # if 'bottle' in obj:
    #     y=y-54
    #     x=x+40
    x_r = -4.604 * z + 838.765 - 30    # additional 30 according to actual test
    y_r = 0.96630872 * x - 241.92799
    z_r = -5.37 * y + 963.440
    if z_r < 130:
        z_r = 130
    return x_r, y_r, z_r
def change_object_position(my_prompt, system_prompt, obj, img):
    # convert object coordinates into robot system coordinates
    x_r, y_r, z_r=convert_coordinates(obj)
    # move robot to object posistion
    Button8Click(x_r, y_r, z_r, 0.0, 40.0,2500)
    time.sleep(1)
    # control robot to catach object
    Button8Click(x_r, y_r, z_r, 0.0, 40.0,1100)
    coordinate = f'X={x_r}, Y={y_r}, Z={z_r}'
    ai_prompt = tasks['decide_move'].format(task=my_prompt, coordinate=coordinate)
    # 根据prompt决定下一步如何移动
    coord = get_response_with_img(system_prompt, ai_prompt, img)
    ai_prompt = tasks['decide_move2'].format(task=coord)
    coord = get_response_with_img(system_prompt, ai_prompt, img)
    x = float(coord[coord.find('X='):coord.find('mm')][2:])
    y = float(coord[coord.find('Y='):coord.find('mm', coord.find('mm') + 1)][2:])
    z = float(coord[coord.find('Z='):][2:-2])
    if z < 160:
        z = 160

    Button8Click(x, y, z, 0.0, 40.0,1100)
    time.sleep(1)
    Button8Click(x, y, z, 0.0, 40.0,2500)

    # 关闭串口
    StopCom()
    print("串口已关闭")

def get_response_with_img(system_prompt, my_prompt, base64_image):
    response = client.chat.completions.create(
        model="gpt-4o",
        temperature=0.7,
        messages=[
            {"role": "system", "content": system_prompt},

            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": my_prompt,
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{base64_image}"
                        },
                    },
                ],
            }
        ],
    )
    res = response.choices[0].message.content
    return res

def action_start(img, result_list):
    # 这里是action_start的代码逻辑（用户自行实现）
    my_prompt='把瓶子里的水倒到杯子中'
    system_prompt = tasks['role']

    # Decide 需要操作什么类别的物体
    ai_prompt = tasks['decide_object'].format(task=my_prompt)
    obj_type=get_response_with_img(system_prompt,ai_prompt,img)

    # Decide 需要进行的任务类型
    ai_prompt = tasks['decide_task'].format(task=my_prompt)
    task_type=get_response_with_img(system_prompt,ai_prompt,img)
    print("Action Start Triggered!")
    available_ports = EnumSerial()
    print("可用的串口:")
    for port in available_ports:
        print(port)
    if OpenCom("COM8"):
        bottle_captured = False
        cup_recognized = False
        x_r_cup = y_r_cup = z_r_cup = None  # 初始化杯子的坐标变量
        for obj in result_list:
            if 'change object position' in task_type:
                change_object_position(my_prompt, system_prompt, obj, img)
            if 'pour water' in task_type:
                if 'bottle' in obj:
                    # 转换瓶子的坐标
                    x_r_bottle, y_r_bottle, z_r_bottle = convert_coordinates(obj)
                    if x_r_bottle < 370:
                        x_r_bottle = 370
                        print('选择的坐标可能会导致奇异点，已将坐标限制在合理范围。')
                    if z_r_bottle < 160:
                        z_r_bottle = 160

                    # 移动机械臂到瓶子位置并抓取
                    Button8Click(x_r_bottle, y_r_bottle, z_r_bottle+200, 80.0, 0.0, 2500)
                    time.sleep(2)
                    Button8Click(x_r_bottle, y_r_bottle, z_r_bottle, 80.0, 0.0, 2500)
                    time.sleep(2)
                    Button8Click(x_r_bottle, y_r_bottle, z_r_bottle, 80.0, 0.0, 1800)
                    time.sleep(2)
                    # 将瓶子抬高
                    Button8Click(x_r_bottle, y_r_bottle, z_r_bottle + 100, 80.0, 0.0, 1800)
                    time.sleep(2)
                    bottle_captured = True

                    # 如果杯子已识别，执行倒水动作
                    if cup_recognized:
                        # 移动到杯子位置
                        Button8Click(x_r_cup, y_r_cup+110, z_r_bottle + 100, 80.0, 0.0, 1800)
                        time.sleep(2)
                        if rbot.dx_get > 0:
                            print(f"接收到数据: X={rbot.XP}, Y={rbot.YP}, Z={rbot.ZP}")
                            print(
                                f"接收到数据: a0={rbot.a0}, a1={rbot.a1}, a2={rbot.a2},w0={rbot.w0},w1={rbot.w1},aw={rbot.aw}")
                            rbot.dx_get = 0
                            time.sleep(4)
                        # 调整角度进行倒水
                        change_angle(rbot.a0 / 100, rbot.a1 / 100, rbot.a2 / 100,
                                     rbot.w0 / 100, rbot.w1 / 100, -80, 1800)
                        time.sleep(4)
                        change_angle(rbot.a0 / 100, rbot.a1 / 100, rbot.a2 / 100,
                                     rbot.w0 / 100, rbot.w1 / 100, 0.0, 1800)

                elif 'cup' in obj:
                    # 转换杯子的坐标
                    x_r_cup, y_r_cup, z_r_cup = convert_coordinates(obj)
                    cup_recognized = True

                    if bottle_captured:
                        # 如果已抓取瓶子，移动到杯子位置并倒水
                        Button8Click(x_r_cup, y_r_cup, z_r_cup, 80.0, 0.0, 1800)
                        if rbot.dx_get > 0:
                            print(f"接收到数据: X={rbot.XP}, Y={rbot.YP}, Z={rbot.ZP}")
                            print(
                                f"接收到数据: a0={rbot.a0}, a1={rbot.a1}, a2={rbot.a2},w0={rbot.w0},w1={rbot.w1},aw={rbot.aw}")
                            rbot.dx_get = 0
                            time.sleep(0.1)
                            # 调整角度进行倒水
                        change_angle(rbot.a0 / 100, rbot.a1 / 100, rbot.a2 / 100,
                                     rbot.w0 / 100, rbot.w1 / 100, rbot.aw / 100 - 60, 1800)
                else:
                    # 处理其他物体或忽略
                    print('没有识别到bottle和cup，请调整物体')
                    pass
        # 关闭串口
        StopCom()
        print("串口已关闭")
    else:
        print("无法打开指定的串口")
    return None

while True:
    ret, frame = camera.read()
    frame=frame.to(device)
    if not ret:
        break

    left_frame = frame[:, :1920]   # 左图像
    right_frame = frame[:, 1920:]  # 右图像

    if frame_count % N == 0:
        # 缩小图像进行推理
        small_left = cv2.resize(left_frame, None, fx=scale_factor, fy=scale_factor)
        small_right = cv2.resize(right_frame, None, fx=scale_factor, fy=scale_factor)

        # 检测
        results_left = model.track(small_left, persist=True, classes=desired_classes)
        results_right = model.track(small_right, persist=True, classes=desired_classes)

        left_boxes = results_left[0].boxes
        right_boxes = results_right[0].boxes

    left_objects = []
    for box in left_boxes:
        cls_id = int(box.cls.item())
        x_min = int(box.xyxy[0, 0].item() / scale_factor)
        y_min = int(box.xyxy[0, 1].item() / scale_factor)
        x_max = int(box.xyxy[0, 2].item() / scale_factor)
        y_max = int(box.xyxy[0, 3].item() / scale_factor)
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0
        conf = box.conf.item()
        left_objects.append((cls_id, x_center, y_center, conf, (x_min, y_min, x_max, y_max)))

    right_objects = []
    for box in right_boxes:
        cls_id = int(box.cls.item())
        x_min_r = int(box.xyxy[0, 0].item() / scale_factor)
        y_min_r = int(box.xyxy[0, 1].item() / scale_factor)
        x_max_r = int(box.xyxy[0, 2].item() / scale_factor)
        y_max_r = int(box.xyxy[0, 3].item() / scale_factor)
        x_center_r = (x_min_r + x_max_r) / 2.0
        y_center_r = (y_min_r + y_max_r) / 2.0
        conf_r = box.conf.item()
        right_objects.append((cls_id, x_center_r, y_center_r, conf_r, (x_min_r, y_min_r, x_max_r, y_max_r)))

    matched_pairs = []  # 存储 (left_obj, right_obj)
    used_right_indices = set()

    for left_obj in left_objects:
        left_cls, lx, ly, lconf, lbox = left_obj
        candidates = [(i, ro) for i, ro in enumerate(right_objects) if ro[0] == left_cls and i not in used_right_indices]
        if len(candidates) == 0:
            matched_pairs.append((left_obj, None))
            continue

        # 找到距离最小的右目标（这里的距离仅考虑y方向，可根据需要修改）
        min_dist = float('inf')
        best_idx = None
        best_obj = None
        for i, ro in candidates:
            rx, ry = ro[1], ro[2]
            dist = np.sqrt((ly - ry)**2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_obj = ro

        if best_idx is not None:
            matched_pairs.append((left_obj, best_obj))
            used_right_indices.add(best_idx)
        else:
            matched_pairs.append((left_obj, None))

    all_detect_result = []
    for (left_obj, right_obj) in matched_pairs:
        left_cls, lx, ly, lconf, (x_min, y_min, x_max, y_max) = left_obj
        label = results_left[0].names[left_cls]

        cv2.rectangle(left_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        if right_obj is not None:
            right_cls, rx, ry, rconf, (x_min_r, y_min_r, x_max_r, y_max_r) = right_obj
            disparity_pixels = (lx - rx)
            Z_mm = B * f * 100/disparity_pixels
        else:
            Z_mm = None

        X_mm = lx * pixel_size
        Y_mm = ly * pixel_size

        # 将当前帧的坐标加入recent_positions
        # 如果Z_mm是None，这里可选择跳过或用0代替。
        # 这里选择跳过(即只对有Z的帧进行平均)：
        if Z_mm is not None:
            recent_positions[label].append((X_mm, Y_mm, Z_mm))

        # 对最近的坐标进行平均
        # 如果当前没有有效的数据点，则使用当前值（或显示na）
        if len(recent_positions[label]) > 0:
            X_avg = sum(pos[0] for pos in recent_positions[label]) / len(recent_positions[label])
            Y_avg = sum(pos[1] for pos in recent_positions[label]) / len(recent_positions[label])
            Z_values = [pos[2] for pos in recent_positions[label] if pos[2] is not None]
            if len(Z_values) > 0:
                Z_avg = sum(Z_values) / len(Z_values)
            else:
                Z_avg = None
        else:
            # 没有数据，使用当前值或na
            X_avg = X_mm
            Y_avg = Y_mm
            Z_avg = Z_mm

        if Z_avg is not None:
            text = f"{label}: {lconf:.2f}, X={X_avg:.2f}mm, Y={Y_avg:.2f}mm, Z={Z_avg:.2f}mm"
        else:
            text = f"{label}: {lconf:.2f}, X={X_avg:.2f}mm, Y={Y_avg:.2f}mm, Z=na"
        print(text)
        cv2.putText(left_frame, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)
        all_detect_result.append(text)

    # 注意，此处不再直接调用 action_start，而是在下面通过用户输入决定是否执行
    left_frame_resized = cv2.resize(left_frame, (0, 0), fx=scale_factor, fy=scale_factor)
    right_frame_resized = cv2.resize(right_frame, (0, 0), fx=scale_factor, fy=scale_factor)

    cv2.imshow("YOLO11 Tracking Left", left_frame_resized)
    cv2.imshow("Right Image", right_frame_resized)

    frame_count += 1

    # 按下 'q' 退出
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    # 按下 'a' 来输入命令
    if key == ord('a'):
        user_input = input("请输入指令：")  # 阻塞式输入
        if user_input.strip() == 'action':
            base_left_frame=frame_to_base64(right_frame)
            action_start(base_left_frame, all_detect_result)

camera.release()
cv2.destroyAllWindows()
