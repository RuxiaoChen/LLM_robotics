import json
import os
import csv
import pdb
import cv2
import base64
from openai import OpenAI

# os.environ['OPENAI_API_KEY'] = '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'
client = OpenAI(
    api_key='sk-dA85I4Bo06hodLvVga1VAnOLSKBNGU9InUa4xD7uS8RiTvna',
    base_url='https://api.openai-proxy.org/v1'
)
# 从 JSON 文件读取问题数据
def load_questions_data(file_path):
    """
    从指定的 JSON 文件加载问题数据。

    参数：
        file_path (str): JSON 文件的路径。

    返回：
        dict: 包含问题的字典
    """
    with open(file_path, 'r') as file:
        return json.load(file)


# 编码图像为 base64
def encode_image(image_path):
    """
    将图片编码为 base64 格式。

    参数：
        image_path (str): 图片路径。

    返回：
        str: base64 编码的图片字符串。
    """
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')


# 使用图片和大模型回答问题
def get_response_with_img(system_prompt, my_prompt, image_list):
    """
    通过图片和提示获取大模型的回答。

    参数：
        system_prompt (str): 系统提示。
        my_prompt (str): 用户提示。
        image_list (list): 编码后的图片列表。

    返回：
        str: 大模型的响应。
    """
    content_user = [{"type": "text", "text": my_prompt}]
    for image in image_list:
        content_user.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/png;base64,{image}"
            },
        })
    # 假设 client.chat.completions.create 是已配置的模型调用方法
    response = client.chat.completions.create(
        model="gpt-4o",
        temperature=0.7,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": content_user},
        ],
    )

    res = response.choices[0].message.content
    return res


# 模拟回答函数
def answer_question(question, frames):
    """
    使用提取的视频帧回答问题。

    参数：
        question (dict): 包含问题和答案选项的数据。
        frames (list): 提取的视频帧路径列表。

    返回：
        int: 预测的答案 ID。
    """
    # 编码所有帧
    encoded_images = [encode_image(frame) for frame in frames]
    # 使用大模型生成回答（假设固定的提示）
    system_prompt = "You are an AI that answers questions based on video frames."
    my_prompt = 'Please answer the question using one of the options'+question["question"]+'Your options are:' + str(question['options'])
    response = get_response_with_img(system_prompt, my_prompt, encoded_images)
    print(f"Model response for question '{question['question']}': {response}")
    # 模拟将模型响应解析为选项索引（实际实现需要根据模型输出调整）
    return response


# 测试函数
def evaluate_vqa(video_file_name, questions_data, frames):
    """
    根据视频文件名查找问题并计算正确率。

    参数：
        video_file_name (str): 视频文件名，例如 "video_9431"
        questions_data (dict): 包含问题的字典
        frames (list): 提取的视频帧路径列表

    返回：
        dict: 包含正确回答数、总问题数和正确率
    """
    video_id = video_file_name.split(".")[0]  # 提取视频ID，例如 "video_9431"
    if video_id not in questions_data:
        return {"error": "Video ID not found in questions data."}

    mc_questions = questions_data[video_id]["mc_question"]
    correct_answers = 0
    total_questions = len(mc_questions)

    for question in mc_questions:
        predicted_answer = answer_question(question, frames)  # 使用帧回答问题
        if question['options'][question["answer_id"]] in predicted_answer:
            correct_answers += 1

    accuracy = correct_answers / total_questions * 100 if total_questions > 0 else 0
    return {
        "correct_answers": correct_answers,
        "total_questions": total_questions,
        "accuracy": accuracy
    }


# 初始化 CSV 文件
def initialize_csv(output_file):
    """
    初始化 CSV 文件，写入标题行。

    参数：
        output_file (str): 输出 CSV 文件路径。
    """
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Video ID", "Correct Answers", "Total Questions", "Accuracy (%)"])


# 追加单个结果到 CSV 文件
def append_result_to_csv(video_file, result, output_file):
    """
    将单个视频的测试结果追加到 CSV 文件。

    参数：
        video_file (str): 视频文件名。
        result (dict): 单个视频的测试结果。
        output_file (str): 输出 CSV 文件路径。
    """
    with open(output_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        video_id = video_file.split(".")[0]
        if "error" in result:
            writer.writerow([video_id, "Error: Video ID not found", "", ""])
        else:
            writer.writerow([
                video_id,
                result["correct_answers"],
                result["total_questions"],
                round(result["accuracy"], 2)
            ])


# 从视频中提取每五十帧一帧并保存为图片序列
def extract_frames(video_path, output_folder, frame_interval=50):
    """
    从视频中每隔指定帧提取一帧并保存为图片。

    参数：
        video_path (str): 视频文件路径。
        output_folder (str): 保存图片的文件夹。
        frame_interval (int): 每隔多少帧提取一帧。

    返回：
        list: 提取的帧文件路径列表。
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    video_id = os.path.basename(video_path).split(".")[0]
    cap = cv2.VideoCapture(video_path)
    frame_count = 0
    saved_count = 0
    frame_files = []

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_folder, f"{video_id}_frame_{saved_count}.jpg")
            cv2.imwrite(frame_filename, frame)
            frame_files.append(frame_filename)
            saved_count += 1

        frame_count += 1

    cap.release()
    print(f"Extracted {saved_count} frames from {video_id}.")
    return frame_files


# 对文件夹中的所有视频进行测试并实时保存
def evaluate_all_videos_and_save(videos_folder, questions_data, output_file, frame_output_folder):
    """
    对指定文件夹中的所有视频文件进行 VQA 测试，并实时将结果保存到 CSV 文件。

    参数：
        videos_folder (str): 视频文件夹路径。
        questions_data (dict): 包含问题的字典。
        output_file (str): 输出 CSV 文件路径。
        frame_output_folder (str): 保存提取帧的文件夹。
    """
    initialize_csv(output_file)  # 初始化 CSV 文件
    for video_file in os.listdir(videos_folder):
        if video_file.endswith(('.mp4', '.avi', '.mkv')):  # 检查视频文件扩展名
            video_path = os.path.join(videos_folder, video_file)
            frames = extract_frames(video_path, frame_output_folder)  # 提取帧
            result = evaluate_vqa(video_file, questions_data, frames)
            append_result_to_csv(video_file, result, output_file)


# 示例运行
questions_file_path = "mc_question_train.json"  # 指定 JSON 文件路径
questions_data = load_questions_data(questions_file_path)

videos_folder = "videos"  # 视频文件夹路径
output_csv_file = "vqa_results.csv"
frame_output_folder = "extracted_frames"  # 保存图片的文件夹

evaluate_all_videos_and_save(videos_folder, questions_data, output_csv_file, frame_output_folder)

# 打印保存完成消息
print(f"Results are being saved incrementally to {output_csv_file}.")
