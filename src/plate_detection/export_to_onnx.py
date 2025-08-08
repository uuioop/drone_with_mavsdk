#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import sys
import torch
import torch.nn as nn
import numpy as np
from pathlib import Path

# 添加当前包的路径到sys.path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

# 导入项目模块
try:
    from models.experimental import attempt_load
    from plate_recognition.plateNet import myNet_ocr_color
    from plate_recognition.plate_rec import init_model
except ImportError as e:
    print(f"导入模块错误: {e}")
    print("请确保在正确的环境中运行此脚本")
    sys.exit(1)

def export_detection_model_to_onnx(model_path, output_path, img_size=640):
    """
    导出车牌检测模型为ONNX格式
    
    Args:
        model_path: 检测模型路径
        output_path: 输出ONNX文件路径
        img_size: 输入图像尺寸
    """
    print(f"开始导出检测模型: {model_path}")
    
    # 加载模型
    device = torch.device('cpu')
    model = attempt_load(model_path, map_location=device)
    model.eval()
    
    # 创建示例输入
    dummy_input = torch.randn(1, 3, img_size, img_size)
    
    # 导出为ONNX
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        }
    )
    
    print(f"检测模型已导出到: {output_path}")

def export_recognition_model_to_onnx(model_path, output_path, is_color=True):
    """
    导出车牌识别模型为ONNX格式
    
    Args:
        model_path: 识别模型路径
        output_path: 输出ONNX文件路径
        is_color: 是否包含颜色识别
    """
    print(f"开始导出识别模型: {model_path}")
    
    # 加载模型
    device = torch.device('cpu')
    model = init_model(device, model_path, is_color=is_color)
    model.eval()
    
    # 创建示例输入 (168x48 车牌图像)
    dummy_input = torch.randn(1, 3, 48, 168)
    
    # 导出为ONNX
    if is_color:
        # 如果包含颜色识别，需要处理多个输出
        torch.onnx.export(
            model,
            dummy_input,
            output_path,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=['input'],
            output_names=['plate_output', 'color_output'],
            dynamic_axes={
                'input': {0: 'batch_size'},
                'plate_output': {0: 'batch_size'},
                'color_output': {0: 'batch_size'}
            }
        )
    else:
        torch.onnx.export(
            model,
            dummy_input,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={
                'input': {0: 'batch_size'},
                'output': {0: 'batch_size'}
            }
        )
    
    print(f"识别模型已导出到: {output_path}")

def verify_onnx_model(onnx_path, input_shape):
    """
    验证ONNX模型
    
    Args:
        onnx_path: ONNX模型路径
        input_shape: 输入形状
    """
    try:
        import onnx
        import onnxruntime as ort
        
        # 加载ONNX模型
        onnx_model = onnx.load(onnx_path)
        onnx.checker.check_model(onnx_model)
        
        # 创建推理会话
        ort_session = ort.InferenceSession(onnx_path)
        
        # 创建测试输入
        dummy_input = np.random.randn(*input_shape).astype(np.float32)
        
        # 运行推理
        outputs = ort_session.run(None, {'input': dummy_input})
        
        print(f"ONNX模型验证成功: {onnx_path}")
        print(f"输出形状: {[out.shape for out in outputs]}")
        
        return True
        
    except ImportError:
        print("警告: 未安装onnx或onnxruntime，跳过验证")
        return False
    except Exception as e:
        print(f"ONNX模型验证失败: {e}")
        return False

def optimize_onnx_model(input_path, output_path):
    """
    优化ONNX模型（可选）
    
    Args:
        input_path: 输入ONNX模型路径
        output_path: 优化后的ONNX模型路径
    """
    try:
        import onnx
        from onnxsim import simplify
        
        # 加载模型
        model = onnx.load(input_path)
        
        # 简化模型
        model_sim, check = simplify(model)
        
        if check:
            onnx.save(model_sim, output_path)
            print(f"模型已优化并保存到: {output_path}")
        else:
            print("模型优化失败，使用原始模型")
            import shutil
            shutil.copy(input_path, output_path)
            
    except ImportError:
        print("警告: 未安装onnxsim，跳过优化")
        import shutil
        shutil.copy(input_path, output_path)
    except Exception as e:
        print(f"模型优化失败: {e}")
        import shutil
        shutil.copy(input_path, output_path)

def main():
    """主函数"""
    print("开始导出模型为ONNX格式...")
    
    # 设置路径
    weights_dir = current_dir / "weights"
    output_dir = current_dir / "onnx_models"
    
    # 创建输出目录
    output_dir.mkdir(exist_ok=True)
    
    # 检测模型路径
    detect_model_path = weights_dir / "plate_detect.pt"
    detect_onnx_path = output_dir / "plate_detect.onnx"
    
    # 识别模型路径
    rec_model_path = weights_dir / "plate_rec_color.pth"
    rec_onnx_path = output_dir / "plate_rec_color.onnx"
    
    # 检查模型文件是否存在
    if not detect_model_path.exists():
        print(f"错误: 检测模型文件不存在: {detect_model_path}")
        return
    
    if not rec_model_path.exists():
        print(f"错误: 识别模型文件不存在: {rec_model_path}")
        return
    
    try:
        # 导出检测模型
        export_detection_model_to_onnx(
            str(detect_model_path),
            str(detect_onnx_path),
            img_size=640
        )
        
        # 验证检测模型
        verify_onnx_model(str(detect_onnx_path), (1, 3, 640, 640))
        
        # 优化检测模型
        detect_onnx_optimized = output_dir / "plate_detect_optimized.onnx"
        optimize_onnx_model(str(detect_onnx_path), str(detect_onnx_optimized))
        
        # 导出识别模型
        export_recognition_model_to_onnx(
            str(rec_model_path),
            str(rec_onnx_path),
            is_color=True
        )
        
        # 验证识别模型
        verify_onnx_model(str(rec_onnx_path), (1, 3, 48, 168))
        
        # 优化识别模型
        rec_onnx_optimized = output_dir / "plate_rec_color_optimized.onnx"
        optimize_onnx_model(str(rec_onnx_path), str(rec_onnx_optimized))
        
        print("\n=== 导出完成 ===")
        print(f"检测模型: {detect_onnx_path}")
        print(f"检测模型(优化): {detect_onnx_optimized}")
        print(f"识别模型: {rec_onnx_path}")
        print(f"识别模型(优化): {rec_onnx_optimized}")
        
        # 显示文件大小
        print("\n=== 文件大小 ===")
        for model_path in [detect_onnx_path, detect_onnx_optimized, rec_onnx_path, rec_onnx_optimized]:
            if model_path.exists():
                size_mb = model_path.stat().st_size / (1024 * 1024)
                print(f"{model_path.name}: {size_mb:.2f} MB")
        
    except Exception as e:
        print(f"导出过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 