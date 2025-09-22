import cv2
import numpy as np
import os
from PIL import Image, ImageDraw, ImageFont

def cv2ImgAddText(img, text, left, top, textColor=(0, 255, 0), textSize=20):
    if (isinstance(img, np.ndarray)):  #判断是否OpenCV图片类型
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    
    # 尝试多个可能的字体路径
    font_paths = [
        "fonts/platech.ttf",  # 相对路径
        os.path.join(os.path.dirname(__file__), "..", "fonts", "platech.ttf"),  # 相对于当前文件
        os.path.join(os.path.dirname(__file__), "..", "..", "fonts", "platech.ttf"),  # 相对于utils目录
        "/opt/ros/humble/share/plate_detection/fonts/platech.ttf",  # 安装后的路径
    ]
    
    font_path = None
    for path in font_paths:
        if os.path.exists(path):
            font_path = path
            break
    
    if font_path is None:
        # 如果找不到字体文件，使用默认字体
        print("Warning: Font file not found, using default font")
        fontText = ImageFont.load_default()
    else:
        try:
            fontText = ImageFont.truetype(font_path, textSize, encoding="utf-8")
        except Exception as e:
            print(f"Warning: Failed to load font {font_path}: {e}")
            fontText = ImageFont.load_default()
    
    draw.text((left, top), text, textColor, font=fontText)
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    imgPath = "result.jpg"
    img = cv2.imread(imgPath)
    
    saveImg = cv2ImgAddText(img, '中国加油！', 50, 100, (255, 0, 0), 50)
    
    # cv2.imshow('display',saveImg)
    cv2.imwrite('save.jpg',saveImg)
    # cv2.waitKey()