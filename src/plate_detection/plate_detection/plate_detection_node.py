#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# 应用typing补丁
import sys
import typing
if not hasattr(typing, '_ClassVar'):
    typing._ClassVar = typing.ClassVar

import os
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

# 导入自定义消息类型
from drone_control_interfaces.msg import LicenseInfo
# 获取包的路径
from ament_index_python.packages import get_package_share_directory

# 获取当前包的路径
package_share_directory = get_package_share_directory('plate_detection')

# 添加当前包的路径到sys.path，用于导入本地复制的模块
if package_share_directory not in sys.path:
    sys.path.append(package_share_directory)

# Local imports - 使用本地复制的模块
try:
    from models.experimental import attempt_load
    from utils.datasets import letterbox
    from utils.general import check_img_size, non_max_suppression_face, apply_classifier, scale_coords, xyxy2xywh, \
        strip_optimizer, set_logging, increment_path
    from utils.plots import plot_one_box
    from utils.torch_utils import select_device, load_classifier, time_synchronized
    from utils.cv_puttext import cv2ImgAddText
    from plate_recognition.plate_rec import get_plate_result, allFilePath, init_model, cv_imread
    from plate_recognition.double_plate_split_merge import get_split_merge
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please run setup_models.py first to copy all necessary files")
    raise

clors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(0,255,255)]
danger=['危','险']

def order_points(pts):                   
    """作用：将四个点按照左上 右上 右下 左下排列
    步骤：计算四个点坐标和->计算左上和右下点->计算右上和左下点"""
    rect = np.zeros((4, 2), dtype = "float32")
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def four_point_transform(image, pts):     
    """作用：对车牌区域进行透视变换，得到正视图
    步骤：计算车牌区域宽高->计算透视变换矩阵->应用透视变换"""
    rect = pts.astype('float32')
    (tl, tr, br, bl) = rect
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

def load_model(weights, device):   
    """作用：加载检测模型
    步骤：加载模型->返回模型"""
    model = attempt_load(weights, map_location=device)  # load FP32 model
    return model

def scale_coords_landmarks(img1_shape, coords, img0_shape, ratio_pad=None): 
    """作用：将检测框坐标从缩放后的图像尺寸转换回原始图像尺寸
    步骤：计算缩放比例->计算填充量->调整坐标->限制坐标范围"""
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2, 4, 6]] -= pad[0]  # x padding
    coords[:, [1, 3, 5, 7]] -= pad[1]  # y padding
    coords[:, :8] /= gain
    coords[:, 0].clamp_(0, img0_shape[1])  # x1
    coords[:, 1].clamp_(0, img0_shape[0])  # y1
    coords[:, 2].clamp_(0, img0_shape[1])  # x2
    coords[:, 3].clamp_(0, img0_shape[0])  # y2
    coords[:, 4].clamp_(0, img0_shape[1])  # x3
    coords[:, 5].clamp_(0, img0_shape[0])  # y3
    coords[:, 6].clamp_(0, img0_shape[1])  # x4
    coords[:, 7].clamp_(0, img0_shape[0])  # y4
    return coords

def get_plate_rec_landmark(img, xyxy, conf, landmarks, class_num, device, plate_rec_model, is_color=False): 
    """作用：对检测到的车牌区域进行文字识别
    步骤：提取车牌区域坐标->透视变换得到正视图->调用识别模型识别车牌号
    ->检查是否为危险品车牌（包含"危"或"险"字）"""
    
    h,w,c = img.shape
    result_dict={}
    tl = 1 or round(0.002 * (h + w) / 2) + 1  # line/font thickness

    x1 = int(xyxy[0])
    y1 = int(xyxy[1])
    x2 = int(xyxy[2])
    y2 = int(xyxy[3])
    height=y2-y1
    landmarks_np=np.zeros((4,2))
    rect=[x1,y1,x2,y2]
    for i in range(4):
        point_x = int(landmarks[2 * i])
        point_y = int(landmarks[2 * i + 1])
        landmarks_np[i]=np.array([point_x,point_y])

    roi_img = four_point_transform(img,landmarks_np)   #透视变换得到车牌小图
    if roi_img.shape[0]==0 or roi_img.shape[1]==0:
        return
    plate_number,rec_prob,plate_color,color_conf=get_plate_result(roi_img,device,plate_rec_model,is_color)  #对车牌小图进行识别
    for dan in danger:   #只要出现'危'或者'险'就是危险品车牌
        if dan in plate_number:
            plate_number='危险品'
    result_dict['plate_no']=plate_number   #车牌号
    result_dict['confidence']=rec_prob   #置信度
    result_dict['plate_type']=plate_color   #车牌类型
    result_dict['roi_height']=height
    result_dict['plate_color']=""
    if is_color:
        result_dict['plate_color']=plate_color   #车牌颜色
    result_dict['rect']=rect                      #车牌roi区域
    result_dict['landmarks']=landmarks_np.tolist() #车牌角点坐标
    result_dict['color_conf']=color_conf
    result_dict['rec_conf']=rec_prob
    return result_dict

def detect_Recognition_plate(model, orgimg, device, plate_rec_model, img_size, is_color=False):    #获取车牌信息
    """"整个检测识别流程的主函数
    图像预处理（调整大小、格式转换）->模型推理检测车牌位置->
    非极大值抑制(NMS)去除重复检测->对每个检测结果进行文字识别->返回所有识别结果"""
    # 图像预处理（调整大小、格式转换）
    img_size = (640,640)
    # Padded resize
    img = letterbox(orgimg, img_size, auto=False)[0]   #调整大小
    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    # Inference
    pred = model(img)[0]
    # Apply NMS
    pred = non_max_suppression_face(pred, conf_thres=0.3, iou_thres=0.5)
    # Process detections
    result_list=[]
    for i, det in enumerate(pred):  # detections per image
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], orgimg.shape).round()
            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
            det[:, 5:13] = scale_coords_landmarks(img.shape[2:], det[:, 5:13] , orgimg.shape).round()
            for j in range(det.size()[0]):
                xyxy = det[j, :4].view(-1).tolist()
                conf = det[j, 4].cpu().numpy()
                landmarks = det[j, 5:13].view(-1).tolist()
                class_num = det[j, 13].cpu().numpy()
                result_dict = get_plate_rec_landmark(orgimg, xyxy, conf, landmarks, class_num, device, plate_rec_model, is_color)
                if result_dict:
                    result_list.append(result_dict)
    return result_list

def draw_result(orgimg, dict_list, is_color=False):   
    """在图像上绘制检测结果->画矩形框标记车牌位置->
    在框上显示识别出的车牌号->危险品车牌用红色，普通车牌用绿色"""
    result_str = ""
    for result in dict_list:
        rect_area = result['rect']
        x, y, w, h = rect_area[0], rect_area[1], rect_area[2] - rect_area[0], rect_area[3] - rect_area[1]
        length = max(w, h)
        if length < 30:
            continue
        if not result['plate_no']:
            continue
        green = (0, 255, 0) if result['plate_no'] != '危险品' else (0, 0, 255)
        if 'color_conf' in result:
            green = (0, 255, 0) if result['color_conf'] > 0.5 else (0, 0, 255)
        cv2.rectangle(orgimg, (int(x), int(y)), (int(x + w), int(y + h)), green, 2)
        cv2.rectangle(orgimg, (int(x), int(y - 30)), (int(x + w), int(y)), green, -1)
        orgimg = cv2ImgAddText(orgimg, result['plate_no'], int(x), int(y - 30), (0, 0, 0), 20)
        # orgimg = cv2ImgAddText(orgimg,result['plate_no'],(int(x),int(y)),(0,0,0),20)
        result_str += result['plate_no'] + " "
    return orgimg

class PlateDetectionNode(Node):
    def __init__(self):
        super().__init__('plate_detection_node')
        
        # 声明参数
        self.declare_parameter('detect_model_path', 'weights/plate_detect.pt')
        self.declare_parameter('rec_model_path', 'weights/plate_rec_color.pth')
        self.declare_parameter('is_color', True)
        self.declare_parameter('img_size', 640)
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/license_detection_result')
        self.declare_parameter('result_image_topic', '/license_detection_result_image')
        
        # 获取参数
        detect_model_path = self.get_parameter('detect_model_path').value
        rec_model_path = self.get_parameter('rec_model_path').value
        self.is_color = self.get_parameter('is_color').value
        self.img_size = self.get_parameter('img_size').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        result_image_topic = self.get_parameter('result_image_topic').value
        
        # CameraInfo 订阅和位姿发布
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 2. 定义号牌的真实世界尺寸 (单位：米)
        # 标准蓝牌: 440mm x 140mm
        plate_width = 0.440
        plate_height = 0.140
        # 定义3D模型点 (以号牌中心为原点)
        self.object_points = np.array([
            [-plate_width / 2, plate_height / 2, 0],  # Top-left
            [plate_width / 2, plate_height / 2, 0],   # Top-right
            [plate_width / 2, -plate_height / 2, 0],  # Bottom-right
            [-plate_width / 2, -plate_height / 2, 0]  # Bottom-left
        ], dtype=np.float32)
        # 构建模型路径（相对于包的共享目录）
        detect_model_path = os.path.join(package_share_directory, detect_model_path)
        rec_model_path = os.path.join(package_share_directory, rec_model_path)
        
        # 检查模型文件是否存在
        if not os.path.exists(detect_model_path):
            self.get_logger().error(f'Detection model not found: {detect_model_path}')
            return
        if not os.path.exists(rec_model_path):
            self.get_logger().error(f'Recognition model not found: {rec_model_path}')
            return
        
        # Initialize models
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'Using device: {self.device}')
        
        try:
            self.detect_model = load_model(detect_model_path, self.device)
            self.plate_rec_model = init_model(self.device, rec_model_path, is_color=self.is_color)
            self.get_logger().info('Models loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading models: {str(e)}')
            return
        
        self.bridge = CvBridge()
        
        # Create publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        self.result_pub = self.create_publisher(
            LicenseInfo,
            output_topic,
            10
        )
        
        self.result_image_pub = self.create_publisher(
            Image,
            result_image_topic,
            10
        )
        
        self.get_logger().info('Plate detection node initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing results to: {output_topic}')
        self.get_logger().info(f'Publishing result images to: {result_image_topic}')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            dict_list = detect_Recognition_plate(
                self.detect_model, 
                cv_image, 
                self.device, 
                self.plate_rec_model, 
                self.img_size, 
                is_color=self.is_color
            )
            
            # Draw results on image
            result_image = draw_result(cv_image, dict_list, self.is_color)
            
            # Publish result image
            result_image_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_image_msg.header = msg.header
            self.result_image_pub.publish(result_image_msg)
            
            # 如果检测结果列表不为空，则遍历处理每一个识别到的车牌
            if dict_list:
                # 循环处理列表中的每一个车牌信息字典
                for result in dict_list:
                    # 确保车牌号存在且不为空白字符
                    if result['plate_no'] and result['plate_no'].strip():
                        
                        # 初始化中心点坐标
                        center_x = 0.0
                        center_y = 0.0
                        
                        # 使用 'landmarks' (四个角点) 来计算中心点
                        if 'landmarks' in result and isinstance(result['landmarks'], list) and len(result['landmarks']) == 4:
                            # 计算四个角点坐标的平均值作为中心点
                            x_sum = sum(p[0] for p in result['landmarks'])
                            y_sum = sum(p[1] for p in result['landmarks'])
                            center_x = x_sum / 4.0
                            center_y = y_sum / 4.0
                        
                        # 创建并填充要发布的消息
                        result_msg = LicenseInfo()
                        result_msg.header = msg.header  # 复制原始图像消息的header，包含时间戳等信息
                        result_msg.plate_no = result['plate_no'].strip()
                        result_msg.center_x = center_x
                        result_msg.center_y = center_y
                        
                        # 为当前这一个车牌发布独立的消息
                        self.result_pub.publish(result_msg)
                        self.get_logger().info(f"Published plate: {result_msg.plate_no} at ({center_x:.2f}, {center_y:.2f})")
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = PlateDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()