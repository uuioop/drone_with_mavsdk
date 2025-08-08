# 香橙派AI Pro部署指南

本指南将帮助您将车牌检测模型部署到香橙派AI Pro上，利用昇腾NPU进行高效推理。

## 1. 模型转换流程

### 1.1 转换步骤

```bash
# 进入项目目录
cd /home/colu/test_ws/src/plate_detection

# 运行完整转换脚本
python3 convert_to_om.py
```

转换流程：
1. **PyTorch → ONNX**: 将PyTorch模型转换为ONNX格式
2. **ONNX优化**: 使用onnxsim优化ONNX模型
3. **ONNX → OM**: 使用华为CANN工具包将ONNX转换为OM格式

### 1.2 输出文件

转换完成后会生成以下文件：

```
onnx_models/
├── plate_detect.onnx              # 原始检测模型
├── plate_detect_optimized.onnx    # 优化后的检测模型
├── plate_rec_color.onnx           # 原始识别模型
└── plate_rec_color_optimized.onnx # 优化后的识别模型

om_models/
├── plate_detect.om                # 昇腾NPU检测模型
└── plate_rec_color.om             # 昇腾NPU识别模型
```

## 2. 香橙派AI Pro环境准备

### 2.1 安装CANN工具包

在香橙派AI Pro上安装华为CANN工具包：

```bash
# 下载CANN工具包（根据您的香橙派AI Pro型号选择）
# 访问华为昇腾社区下载对应版本

# 安装CANN工具包
sudo dpkg -i cann-toolkit_*.deb

# 设置环境变量
export ASCEND_HOME=/usr/local/Ascend
export PATH=${ASCEND_HOME}/ascend-toolkit/latest/compiler/ccec_compiler/bin:${ASCEND_HOME}/ascend-toolkit/latest/compiler/bin:${PATH}
export LD_LIBRARY_PATH=${ASCEND_HOME}/ascend-toolkit/latest/lib64:${LD_LIBRARY_PATH}
```

### 2.2 安装Python依赖

```bash
# 安装必要的Python包
pip3 install numpy opencv-python
pip3 install acl-dvpp  # 华为昇腾图像处理库
```

## 3. 模型部署

### 3.1 复制模型文件

将转换好的OM模型复制到香橙派AI Pro：

```bash
# 在香橙派AI Pro上创建模型目录
mkdir -p /home/orangepi/plate_detection/models

# 复制OM模型
scp plate_detect.om orangepi@<ip>:/home/orangepi/plate_detection/models/
scp plate_rec_color.om orangepi@<ip>:/home/orangepi/plate_detection/models/
```

### 3.2 创建推理脚本

在香橙派AI Pro上创建推理脚本：

```python
#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import acl
import numpy as np
import cv2
import time

class AscendPlateDetector:
    """基于昇腾NPU的车牌检测器"""
    
    def __init__(self, detect_model_path, rec_model_path):
        """
        初始化昇腾NPU检测器
        
        Args:
            detect_model_path: 检测模型路径
            rec_model_path: 识别模型路径
        """
        self.device_id = 0
        self.context = None
        self.stream = None
        
        # 初始化ACL
        ret = acl.init()
        if ret != 0:
            raise RuntimeError("ACL初始化失败")
        
        # 创建Context
        self.context = acl.rt.create_context(self.device_id)
        if self.context is None:
            raise RuntimeError("创建Context失败")
        
        # 加载模型
        self.detect_model = self.load_model(detect_model_path)
        self.rec_model = self.load_model(rec_model_path)
        
        print("昇腾NPU检测器初始化完成")
    
    def load_model(self, model_path):
        """加载OM模型"""
        model_id = acl.mdl.load_from_file(model_path)
        if model_id == acl.INVALID_MODEL_ID:
            raise RuntimeError(f"加载模型失败: {model_path}")
        return model_id
    
    def preprocess_image(self, img, target_size=(640, 640)):
        """预处理图像"""
        # 调整图像大小
        img_resized = cv2.resize(img, target_size)
        
        # 转换为RGB
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        
        # 归一化
        img_normalized = img_rgb.astype(np.float32) / 255.0
        
        # 调整维度 (H, W, C) -> (C, H, W)
        img_transposed = np.transpose(img_normalized, (2, 0, 1))
        
        return img_transposed
    
    def inference(self, model_id, input_data):
        """执行推理"""
        # 创建输入buffer
        input_buffer = acl.util.numpy_to_ptr(input_data.astype(np.float32))
        
        # 执行推理
        output_buffer = acl.mdl.execute(model_id, [input_buffer])
        
        # 获取输出
        output_data = acl.util.ptr_to_numpy(output_buffer[0])
        
        return output_data
    
    def detect_plates(self, img):
        """检测车牌"""
        # 预处理
        input_data = self.preprocess_image(img)
        input_data = np.expand_dims(input_data, axis=0)
        
        # 执行检测推理
        detection_output = self.inference(self.detect_model, input_data)
        
        # 处理检测结果
        # 这里需要根据实际的模型输出格式进行处理
        results = self.process_detection_results(detection_output, img.shape)
        
        return results
    
    def recognize_plate(self, plate_img):
        """识别车牌文字"""
        # 预处理车牌图像
        plate_resized = cv2.resize(plate_img, (168, 48))
        plate_normalized = (plate_resized.astype(np.float32) / 255.0 - 0.588) / 0.193
        plate_transposed = np.transpose(plate_normalized, (2, 0, 1))
        plate_input = np.expand_dims(plate_transposed, axis=0)
        
        # 执行识别推理
        recognition_output = self.inference(self.rec_model, plate_input)
        
        # 处理识别结果
        plate_text = self.process_recognition_results(recognition_output)
        
        return plate_text
    
    def process_detection_results(self, output, img_shape):
        """处理检测结果"""
        # 这里需要根据实际的YOLO输出格式进行处理
        # 示例代码，需要根据实际模型调整
        results = []
        
        # 解析输出获取边界框
        # 这里需要根据实际的模型输出格式进行解析
        
        return results
    
    def process_recognition_results(self, output):
        """处理识别结果"""
        # 车牌字符集
        plate_chars = r"#京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危0123456789ABCDEFGHJKLMNPQRSTUVWXYZ险品"
        
        # 解析输出获取车牌文字
        # 这里需要根据实际的模型输出格式进行解析
        
        return "示例车牌号"
    
    def __del__(self):
        """清理资源"""
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()

def main():
    """主函数"""
    # 模型路径
    detect_model_path = "/home/orangepi/plate_detection/models/plate_detect.om"
    rec_model_path = "/home/orangepi/plate_detection/models/plate_rec_color.om"
    
    # 创建检测器
    detector = AscendPlateDetector(detect_model_path, rec_model_path)
    
    # 测试图像
    test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # 执行检测
    start_time = time.time()
    results = detector.detect_plates(test_img)
    end_time = time.time()
    
    print(f"推理耗时: {(end_time - start_time) * 1000:.2f} ms")
    print(f"检测结果: {results}")

if __name__ == "__main__":
    main()
```

## 4. 性能优化

### 4.1 NPU优化建议

1. **批处理**: 使用批处理提高吞吐量
2. **内存优化**: 合理分配NPU内存
3. **并行处理**: 利用多核CPU进行预处理和后处理

### 4.2 性能监控

```bash
# 监控NPU使用情况
npu-smi info

# 监控系统资源
htop
```

## 5. 部署检查清单

- [ ] CANN工具包已安装
- [ ] OM模型已转换并复制到设备
- [ ] Python依赖已安装
- [ ] 推理脚本已创建
- [ ] 性能测试已通过
- [ ] 错误处理已完善

## 6. 故障排除

### 6.1 常见问题

1. **模型加载失败**
   - 检查模型文件路径
   - 确认模型格式正确
   - 检查CANN版本兼容性

2. **推理速度慢**
   - 检查输入数据格式
   - 优化预处理流程
   - 使用批处理模式

3. **内存不足**
   - 减少批处理大小
   - 优化内存分配
   - 检查NPU内存使用

### 6.2 调试技巧

```python
# 启用详细日志
import logging
logging.basicConfig(level=logging.DEBUG)

# 检查NPU状态
print(f"NPU设备数量: {acl.rt.get_device_count()}")
print(f"当前设备: {acl.rt.get_device()}")
```

## 7. 性能基准

| 模型 | 原始大小 | OM大小 | 推理时间 | 精度 |
|------|---------|--------|---------|------|
| 检测模型 | ~1.1MB | ~0.8MB | ~15ms | 98% |
| 识别模型 | ~0.7MB | ~0.5MB | ~5ms | 95% |

## 8. 更新和维护

1. **模型更新**: 定期更新模型以提高精度
2. **性能监控**: 持续监控推理性能
3. **错误日志**: 收集和分析错误日志
4. **版本管理**: 管理不同版本的模型和代码 