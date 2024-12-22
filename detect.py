import pdb
import warnings
warnings.filterwarnings("ignore")
import argparse
import time
import os
import cv2
import torch
import numpy as np
from collections import defaultdict, deque
from robotic_systems import *
import robotic_systems as rbot
# 如果您还有其他用到的函数，需要从 utils 里引入
from utils.utils import (non_max_suppression, scale_coords,
                         torch_utils, google_utils)

# ------------------ 用户根据实际设备和相机参数进行修改 ------------------
B = 60.0          # 双目摄像头的基线距离 (mm)
f = 3.0           # 摄像头镜头焦距 (mm)，例如手机摄像头
pixel_size = 50/178     # 像素尺寸 (mm/pixel)，需根据具体相机传感器尺寸计算
scale_factor = 0.5    # 如果需要缩放图像做推理，请相应修改
# 若推理时有缩放/resize操作，记得把检测框坐标再除以 scale_factor
last_position=None
# 用于在一定时间窗口内对同一物体坐标进行平滑平均
recent_positions = defaultdict(lambda: deque(maxlen=10))  # 每个类别存储最近10帧的坐标

def letterbox(img, new_shape=640, color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    """
    YOLO 原项目中的 letterbox 函数，用于等比例缩放并在边缘填充
    """
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # 只缩小，不放大
        r = min(r, 1.0)

    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # 取整到 32 的倍数
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)
    elif scaleFill:  # 不留边直接填满
        dw, dh = 0, 0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

def convert_coordinates(corrd):
    x, y, z = corrd[0],corrd[1],corrd[2]
    x_r = -4.604 * z + 838.765 - 30    # additional 30 according to actual test
    y_r = 0.8 * x - 141.92799
    z_r = -0.8 * y + 500
    x_r = max(270, min(x_r, 480))
    z_r = max(150, min(z_r, 480))
    return x_r, y_r, z_r

def follow_hands(camera_coordinates,frame_count):
    global last_position
    if camera_coordinates[2] is not None:
        x_r, y_r, z_r = convert_coordinates(camera_coordinates)
        x_buffer, y_buffer, z_buffer = deque(maxlen=5), deque(maxlen=5), deque(maxlen=5)
        x_buffer.append(x_r)
        y_buffer.append(y_r)
        z_buffer.append(z_r)
        # 计算滑动窗口内的平均值
        x_r = sum(x_buffer) / len(x_buffer)
        y_r = sum(y_buffer) / len(y_buffer)
        z_r = sum(z_buffer) / len(z_buffer)

        print(f'X={x_r}, Y={y_r}, Z={z_r}')
        new_position = np.array([x_r, y_r, z_r])
        if last_position is None:
            last_position=new_position
        delta = np.linalg.norm(new_position - last_position)  # 欧几里得距离
        threshold = 40  # 允许的坐标波动范围（单位：mm）
        if delta > threshold:
            last_position=np.array([x_r,y_r,z_r])
            Button8Click(x_r, y_r, z_r, 60.0, 0.0, 2500)

def detect(save_img=False):
    out, source, weights, half, view_img, save_txt, imgsz = \
        opt.output, opt.source, opt.weights, opt.half, opt.view_img, opt.save_txt, opt.img_size

    # 判断是否是摄像头/流
    webcam = source.isdigit() or source.startswith('rtsp') or source.startswith('http')

    # 初始化设备
    device = torch_utils.select_device(opt.device)

    # 下载或检查权重
    google_utils.attempt_download(weights)

    # 加载模型
    model = torch.load(weights, map_location=device)['model']
    model.to(device).eval()

    # 是否使用 half 精度
    half = half and device.type != 'cpu'
    if half:
        model.half()

    # 预先创建保存目录（如果需要）
    os.makedirs(out, exist_ok=True)

    camera_coordinates = None

    if webcam:
        # 摄像头或网络流
        cap = cv2.VideoCapture(source if (source.startswith('rtsp') or source.startswith('http'))
                               else int(source))

        if not cap.isOpened():
            print(f"Error: Unable to open camera/stream {source}")
            return

        # 强制设置分辨率 (双目示例: 3840x1080)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        t0 = time.time()
        action_flag=0
        frame_count = 0  # 初始化帧计数
        last_left_boxes, last_right_boxes = None, None  # 存储最近一次推理的结果
        OpenCom("COM8")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("No more frames or failed to read.")
                break

            h, w, c = frame.shape

            # 分割左右图像
            left_frame = frame[:, :1920]
            right_frame = frame[:, 1920:]

            # 每隔 4 帧进行一次推理
            if frame_count % 4 == 0:
                # 对左右图像分别进行推理
                last_left_boxes = run_inference(model, left_frame, device, imgsz, half)
                last_right_boxes = run_inference(model, right_frame, device, imgsz, half)

            # 如果存在最近一次推理的结果，则使用其绘制检测框和深度信息
            if last_left_boxes is not None and last_right_boxes is not None:
                # 匹配左右目标并计算深度
                matched_pairs = match_left_right(last_left_boxes, last_right_boxes, model.names)

                # 在左图像上绘制检测框和深度信息
                camera_coordinates = draw_results(left_frame, matched_pairs, model.names)

            # 显示左右图像（根据需要调整显示尺寸）
            left_resized = cv2.resize(left_frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
            right_resized = cv2.resize(right_frame, None, fx=scale_factor, fy=scale_factor,
                                       interpolation=cv2.INTER_AREA)

            cv2.imshow("Left Image", left_resized)
            cv2.imshow("Right Image", right_resized)

            # 退出条件
            if cv2.waitKey(1) == ord('q'):
                break

            # 更新帧计数
            frame_count += 1
            if cv2.waitKey(1) == ord('a'):
                action_flag = 1

            if action_flag==1:
                follow_hands(camera_coordinates, frame_count)

        cap.release()

        cv2.destroyAllWindows()

        print(f"Done. ({time.time() - t0:.3f}s)")



def run_inference(model, frame, device, imgsz, half):
    """
    对单帧图像执行推理，返回检测到的框信息列表。
    每个框信息返回一个元组： (cls_id, x_center, y_center, conf, [x_min, y_min, x_max, y_max])
    """
    # 如果有需要可以进行 letterbox 或其他 resize，这里简单直接 resize
    # 也可以保持原图推理，这里示例使用 letterbox
    resized_frame, _, _ = letterbox(frame, new_shape=imgsz)
    img_rgb = resized_frame[:, :, ::-1].transpose(2, 0, 1)
    img_rgb = np.ascontiguousarray(img_rgb)

    # 转 Tensor
    img_tensor = torch.from_numpy(img_rgb).to(device)
    img_tensor = img_tensor.half() if half else img_tensor.float()
    img_tensor /= 255.0
    if img_tensor.ndimension() == 3:
        img_tensor = img_tensor.unsqueeze(0)

    # 推理
    pred = model(img_tensor, augment=opt.augment)[0]
    if half:
        pred = pred.float()

    # NMS
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
                               fast=True, classes=opt.classes, agnostic=opt.agnostic_nms)

    # 将结果映射回原图
    boxes = []
    if pred is not None and len(pred):
        det = pred[0]
        if det is not None and len(det):
            # 映射回原尺寸
            det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], frame.shape).round()
            for *xyxy, conf, cls in det:
                x_min, y_min, x_max, y_max = xyxy
                x_center = (x_min + x_max) / 2.0
                y_center = (y_min + y_max) / 2.0
                # 将 float Tensor 转为 int 或 float
                cls_id = int(cls.item())
                conf_val = float(conf.item())
                # 注意这里如果在推理前对图像进行了缩放，这里应当除以相同的 scale_factor
                boxes.append((cls_id, x_center.item(), y_center.item(), conf_val,
                              [int(x_min.item()), int(y_min.item()),
                               int(x_max.item()), int(y_max.item())]))
    return boxes


def match_left_right(left_boxes, right_boxes, class_names):
    """
    匹配左右检测框并计算视差，用于后续深度估计。
    返回列表：[(left_obj, right_obj), ...]
    其中 left_obj, right_obj 的结构为：
        (cls_id, x_center, y_center, conf, [x_min, y_min, x_max, y_max])
    如果没有匹配，则 right_obj 为 None。
    """
    matched_pairs = []
    used_right_indices = set()

    for left_obj in left_boxes:
        left_cls, lx, ly, lconf, lbox = left_obj
        # 在 right_boxes 中找相同类别、尚未使用过的候选
        candidates = [(i, ro) for i, ro in enumerate(right_boxes)
                      if (ro[0] == left_cls) and (i not in used_right_indices)]
        if len(candidates) == 0:
            matched_pairs.append((left_obj, None))
            continue

        # 找到距离最小的右目标（可改进为考虑 (x, y) 或更多特征）
        min_dist = float('inf')
        best_idx = None
        best_obj = None
        for i, ro in candidates:
            rx, ry = ro[1], ro[2]
            # 这里只比较 y 方向距离；可根据实际需求同时比较 x、y 或其他特征
            dist = abs(ly - ry)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_obj = ro

        if best_obj is not None:
            matched_pairs.append((left_obj, best_obj))
            used_right_indices.add(best_idx)
        else:
            matched_pairs.append((left_obj, None))

    return matched_pairs


def draw_results(left_frame, matched_pairs, class_names):
    """
    在左图像上绘制检测框，并计算深度(Z)后写在图像上。
    同时对 (X, Y, Z) 进行一定的平滑。
    """
    for (left_obj, right_obj) in matched_pairs:
        left_cls, lx, ly, lconf, (x_min, y_min, x_max, y_max) = left_obj
        label = class_names[left_cls]

        # 画左目标框
        cv2.rectangle(left_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # 计算深度
        # 若右图像匹配成功，则用双目视差计算Z
        if right_obj is not None:
            right_cls, rx, ry, rconf, (x_min_r, y_min_r, x_max_r, y_max_r) = right_obj
            # 这里是非常简化的计算方式：只用 x_center 的差
            disparity_pixels = (lx - rx)
            if disparity_pixels != 0:
                # Z(mm) = (B * f * 100) / disparity_pixels
                # B(mm), f(mm)，乘 100 的原因是：像素尺寸的处理或把 f 从mm转成更细的单位
                # 如有更复杂的标定关系，请使用标定矩阵进行计算
                Z_mm = (B * f * 100) / disparity_pixels
            else:
                Z_mm = None
        else:
            Z_mm = None

        # 计算 X, Y (仅基于像素坐标做近似换算)
        X_mm = lx * pixel_size
        Y_mm = ly * pixel_size

        # 将当前帧的坐标加入队列进行平滑
        if Z_mm is not None:
            recent_positions[label].append((X_mm, Y_mm, Z_mm))
        else:
            # 如果没有 Z，可以只使用 X, Y，Z 放 None
            recent_positions[label].append((X_mm, Y_mm, None))

        # 计算最近若干帧的平均值
        positions = recent_positions[label]
        if len(positions) > 0:
            X_vals = [p[0] for p in positions]
            Y_vals = [p[1] for p in positions]
            Z_vals = [p[2] for p in positions if p[2] is not None]

            X_avg = sum(X_vals) / len(X_vals)
            Y_avg = sum(Y_vals) / len(Y_vals)
            if len(Z_vals) > 0:
                Z_avg = sum(Z_vals) / len(Z_vals)
            else:
                Z_avg = None
        else:
            # 没有历史数据，就直接使用当前
            X_avg = X_mm
            Y_avg = Y_mm
            Z_avg = Z_mm

        if Z_avg is not None:
            text = f"{label}: {lconf:.2f}, X={X_avg:.1f}mm, Y={Y_avg:.1f}mm, Z={Z_avg:.1f}mm"
            coordinates=(X_avg,Y_avg,Z_avg)
        else:
            text = f"{label}: {lconf:.2f}, X={X_avg:.1f}mm, Y={Y_avg:.1f}mm, Z=na"
            coordinates = (X_avg, Y_avg, None)

        # 在图像上显示文本
        cv2.putText(left_frame, text, (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        return coordinates


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='hand_m.pt', help='model.pt path')
    parser.add_argument('--source', type=str, default='0', help='source (0 for webcam or path to video/image folder)')
    parser.add_argument('--output', type=str, default='./inference/output', help='output folder')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.36, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.55, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--half', action='store_true', help='use FP16 half precision inference')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    print(opt)

    with torch.no_grad():
        detect(save_img=True)
