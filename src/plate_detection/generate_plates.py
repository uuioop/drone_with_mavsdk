import os
import random
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --- 配置参数 ---
# 门牌图片尺寸
IMG_WIDTH = 640
IMG_HEIGHT = 640

# 门牌实际比例（根据用户提供的图片计算）
PLATE_ASPECT_RATIO = 1.679  # 564:336 = 1.679

# 蓝色背景颜色 (RGB) - 更接近真实门牌的颜色
BLUE_BACKGROUND = (0, 102, 204)  # 典型的蓝色

# 白色文字和边框颜色 (RGB)
WHITE_COLOR = (255, 255, 255)

# 字体文件路径 - 使用用户指定的字体
FONT_PATH = "fonts/platech.ttf"  # 使用粗体字体

# 输出目录结构
OUTPUT_DIR = "output"
IMAGES_DIR = os.path.join(OUTPUT_DIR, "images")
LABELS_DIR = os.path.join(OUTPUT_DIR, "labels")

# 生成数量
NUM_IMAGES = 1000

# 门牌类型和样式
DOOR_PLATE_STYLES = [
    {
        "name": "标准门牌",
        "background": BLUE_BACKGROUND,
        "text_color": WHITE_COLOR,
        "border_color": WHITE_COLOR,
        "font_size": 80  # 增大字体
    },
    # {
    #     "name": "深蓝门牌", 
    #     "background": (0, 51, 102),
    #     "text_color": WHITE_COLOR,
    #     "border_color": WHITE_COLOR,
    #     "font_size": 80
    # },
    # {
    #     "name": "浅蓝门牌",
    #     "background": (51, 153, 255),
    #     "text_color": WHITE_COLOR,
    #     "border_color": WHITE_COLOR,
    #     "font_size": 80
    # }
]

# 街道名称列表
STREET_NAMES = [
    "天马路", "中山路", "人民路", "建设路", "和平路", "解放路", "新华路", "文化路",
    "科技路", "工业路", "商业路", "教育路", "健康路", "幸福路", "和谐路", "发展路"
]

def convert_to_yolo_format(bbox, img_width, img_height):
    """
    将边界框坐标转换为YOLO格式 (center_x, center_y, width, height)
    坐标需要归一化到0-1之间
    """
    x1, y1, x2, y2 = bbox
    
    # 计算中心点坐标
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    
    # 计算宽度和高度
    width = x2 - x1
    height = y2 - y1
    
    # 归一化到0-1
    center_x /= img_width
    center_y /= img_height
    width /= img_width
    height /= img_height
    
    return [center_x, center_y, width, height]

def calculate_plate_dimensions(img_width, img_height, aspect_ratio, margin_ratio=0.15):
    """
    计算门牌尺寸，保持指定长宽比，并留出适当边距
    """
    # 计算可用空间（减去边距）
    available_width = img_width * (1 - 2 * margin_ratio)
    available_height = img_height * (1 - 2 * margin_ratio)
    
    # 根据长宽比计算门牌尺寸
    if available_width / available_height > aspect_ratio:
        # 宽度足够，以高度为准
        plate_height = available_height
        plate_width = plate_height * aspect_ratio
    else:
        # 高度足够，以宽度为准
        plate_width = available_width
        plate_height = plate_width / aspect_ratio
    
    # 计算门牌位置（居中）
    plate_x1 = (img_width - plate_width) / 2
    plate_y1 = (img_height - plate_height) / 2
    plate_x2 = plate_x1 + plate_width
    plate_y2 = plate_y1 + plate_height
    
    return int(plate_x1), int(plate_y1), int(plate_x2), int(plate_y2)

def generate_door_plate(font_path, style):
    """
    根据规则生成一个门牌图像和对应的YOLO标注数据。
    样式严格按照用户提供的图片：矩形，1.679长宽比，白色边框外有适当空间
    """
    # 创建背景图像
    img = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), color=style["background"])
    draw = ImageDraw.Draw(img)

    # 计算门牌区域 - 严格按照1.679长宽比
    plate_x1, plate_y1, plate_x2, plate_y2 = calculate_plate_dimensions(
        IMG_WIDTH, IMG_HEIGHT, PLATE_ASPECT_RATIO, margin_ratio=0.15
    )
    
    plate_width = plate_x2 - plate_x1
    plate_height = plate_y2 - plate_y1

    # 绘制门牌背景（矩形）
    draw.rectangle(
        [(plate_x1, plate_y1), (plate_x2, plate_y2)],
        fill=style["background"],
        outline=style["border_color"],
        width=8  # 更粗的边框
    )

    # 生成门牌文字内容
    street_name = random.choice(STREET_NAMES)
    house_num = random.randint(1, 999)
    first_line_text = f"{street_name}-{house_num:03d}"
    
    building_num = random.randint(1, 9)
    room_num = random.randint(101, 999)
    second_line_text = f"{building_num}-{room_num}"

    try:
        # 加载字体
        font_size = style["font_size"]
        font = ImageFont.truetype(font_path, font_size)

        # 测量文字尺寸
        first_line_bbox = draw.textbbox((0, 0), first_line_text, font=font)
        first_line_width = first_line_bbox[2] - first_line_bbox[0]
        first_line_height = first_line_bbox[3] - first_line_bbox[1]

        second_line_bbox = draw.textbbox((0, 0), second_line_text, font=font)
        second_line_width = second_line_bbox[2] - second_line_bbox[0]
        second_line_height = second_line_bbox[3] - second_line_bbox[1]

        # 计算文字绘制位置 - 在门牌区域内居中
        total_text_height = first_line_height + second_line_height + 30  # 30是行间距
        text_y_start = plate_y1 + (plate_height - total_text_height) / 2
        
        # 绘制第一行
        x1 = plate_x1 + (plate_width - first_line_width) / 2
        y1 = text_y_start
        draw.text((x1, y1), first_line_text, font=font, fill=style["text_color"])
        
        # 绘制第二行
        x2 = plate_x1 + (plate_width - second_line_width) / 2
        y2 = y1 + first_line_height + 30
        draw.text((x2, y2), second_line_text, font=font, fill=style["text_color"])

        # 计算整个门牌的边界框（包含边框）
        min_x = plate_x1
        max_x = plate_x2
        min_y = plate_y1
        max_y = plate_y2
        
        # 转换为YOLO格式
        yolo_bbox = convert_to_yolo_format([min_x, min_y, max_x, max_y], IMG_WIDTH, IMG_HEIGHT)
        
        return img, yolo_bbox, first_line_text, second_line_text
        
    except IOError as e:
        print(f"警告：无法加载字体文件 '{font_path}'。错误：{e}")
        return None, None, None, None

def create_dataset_structure():
    """
    创建数据集目录结构
    """
    # 创建主输出目录
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"已创建输出目录：{OUTPUT_DIR}")
    
    # 创建images目录
    if not os.path.exists(IMAGES_DIR):
        os.makedirs(IMAGES_DIR)
        print(f"已创建图像目录：{IMAGES_DIR}")
    
    # 创建labels目录
    if not os.path.exists(LABELS_DIR):
        os.makedirs(LABELS_DIR)
        print(f"已创建标签目录：{LABELS_DIR}")

def save_yolo_label(label_path, yolo_bbox, class_id=0):
    """
    保存YOLO格式的标签文件
    """
    with open(label_path, 'w', encoding='utf-8') as f:
        # YOLO格式：class_id center_x center_y width height
        f.write(f"{class_id} {yolo_bbox[0]:.6f} {yolo_bbox[1]:.6f} {yolo_bbox[2]:.6f} {yolo_bbox[3]:.6f}\n")

def main():
    """
    主函数
    """
    # 检查字体文件是否存在
    if not os.path.exists(FONT_PATH):
        print(f"错误：找不到字体文件 '{FONT_PATH}'。")
        print("请确保字体文件存在并修改脚本中的 FONT_PATH 变量。")
        return

    # 创建数据集目录结构
    create_dataset_structure()

    print(f"开始生成 {NUM_IMAGES} 张门牌图像...")
    print(f"门牌长宽比: {PLATE_ASPECT_RATIO:.3f}")
    
    generated_count = 0
    for i in range(NUM_IMAGES):
        # 随机选择样式
        style = random.choice(DOOR_PLATE_STYLES)
        
        # 生成图像和标注
        img, yolo_bbox, first_line, second_line = generate_door_plate(FONT_PATH, style)
        
        if img and yolo_bbox:
            # 构造文件名
            filename_base = f"doorplate_{i:04d}"
            image_path = os.path.join(IMAGES_DIR, f"{filename_base}.jpg")
            label_path = os.path.join(LABELS_DIR, f"{filename_base}.txt")
            
            # 保存图像
            img.save(image_path, "JPEG", quality=95)
            
            # 保存YOLO格式的标签文件
            save_yolo_label(label_path, yolo_bbox)
            
            generated_count += 1
            
            if (i + 1) % 100 == 0:
                print(f"已生成 {i + 1} 张图像...")
        else:
            print(f"警告：第 {i+1} 张图像生成失败")

    print(f"\n生成完成！")
    print(f"总共生成了 {generated_count} 张门牌图像")
    print(f"图像保存在：{IMAGES_DIR}")
    print(f"标签保存在：{LABELS_DIR}")
    print(f"\n数据集结构：")
    print(f"{OUTPUT_DIR}/")
    print(f"├── images/     # 图像文件 (.jpg)")
    print(f"└── labels/     # 标签文件 (.txt, YOLO格式)")
    print(f"\n标签格式说明：")
    print(f"每行格式：class_id center_x center_y width_height")
    print(f"- class_id: 0 (门牌类别)")
    print(f"- center_x, center_y: 边界框中心点坐标 (归一化到0-1)")
    print(f"- width, height: 边界框宽度和高度 (归一化到0-1)")
    print(f"\n门牌规格：")
    print(f"- 长宽比: {PLATE_ASPECT_RATIO:.3f}")
    print(f"- 外边距比例: 15%")

if __name__ == "__main__":
    main()