#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import sys
import torch
import numpy as np
from pathlib import Path
import subprocess
import shutil

# 添加当前包的路径到sys.path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

def export_to_onnx(model_path, output_path, input_shape, model_type="detection"):
    """
    将PyTorch模型导出为ONNX格式
    
    Args:
        model_path: PyTorch模型路径
        output_path: ONNX输出路径
        input_shape: 输入形状
        model_type: 模型类型 ("detection" 或 "recognition")
    """
    print(f"开始导出 {model_type} 模型为ONNX格式...")
    
    try:
        device = torch.device('cpu')
        
        if model_type == "detection":
            # 加载检测模型
            from models.experimental import attempt_load
            model = attempt_load(model_path, map_location=device)
            model.eval()
            
            # 创建示例输入
            dummy_input = torch.randn(input_shape)
            
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
            
        elif model_type == "recognition":
            # 加载识别模型
            checkpoint = torch.load(model_path, map_location=device)
            
            if isinstance(checkpoint, dict) and 'state_dict' in checkpoint:
                state_dict = checkpoint['state_dict']
                cfg = checkpoint.get('cfg', {})
                
                # 创建模型实例
                from plate_recognition.plateNet import myNet_ocr_color
                
                model = myNet_ocr_color(
                    num_classes=len(r"#京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危0123456789ABCDEFGHJKLMNPQRSTUVWXYZ险品"),
                    export=True,
                    cfg=cfg,
                    color_num=5
                )
                
                model.load_state_dict(state_dict, strict=False)
                model.to(device)
                model.eval()
                
                # 创建示例输入
                dummy_input = torch.randn(input_shape)
                
                # 导出为ONNX
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
        
        print(f"{model_type} 模型已导出到: {output_path}")
        return True
        
    except Exception as e:
        print(f"导出 {model_type} 模型失败: {e}")
        return False

def optimize_onnx(onnx_path, optimized_path):
    """
    优化ONNX模型
    
    Args:
        onnx_path: 原始ONNX模型路径
        optimized_path: 优化后的ONNX模型路径
    """
    try:
        import onnx
        from onnxsim import simplify
        
        print(f"优化ONNX模型: {onnx_path}")
        
        # 加载模型
        model = onnx.load(onnx_path)
        
        # 简化模型
        model_sim, check = simplify(model)
        
        if check:
            onnx.save(model_sim, optimized_path)
            print(f"模型已优化并保存到: {optimized_path}")
            return True
        else:
            print("模型优化失败，使用原始模型")
            shutil.copy(onnx_path, optimized_path)
            return True
            
    except ImportError:
        print("警告: 未安装onnxsim，跳过优化")
        shutil.copy(onnx_path, optimized_path)
        return True
    except Exception as e:
        print(f"模型优化失败: {e}")
        shutil.copy(onnx_path, optimized_path)
        return False

def convert_to_om(onnx_path, om_path, target_device="Ascend310"):
    """
    将ONNX模型转换为OM模型（华为昇腾格式）
    
    Args:
        onnx_path: ONNX模型路径
        om_path: OM模型输出路径
        target_device: 目标设备类型
    """
    print(f"开始转换ONNX到OM格式: {onnx_path}")
    
    try:
        # 检查atc工具是否可用
        atc_cmd = shutil.which('atc')
        if not atc_cmd:
            print("错误: 未找到atc工具，请确保已安装CANN工具包")
            return False
        
        # 构建atc命令
        cmd = [
            'atc',
            '--model', onnx_path,
            '--output', om_path,
            '--framework', '5',  # ONNX
            '--soc_version', target_device,
            '--input_shape', 'input:1,3,640,640',  # 根据实际输入调整
            '--precision_mode', 'allow_fp32_to_fp16',
            '--op_select_implmode', 'high_precision',
            '--output_type', 'FP16'
        ]
        
        print(f"执行命令: {' '.join(cmd)}")
        
        # 执行转换
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"OM模型转换成功: {om_path}")
            return True
        else:
            print(f"OM模型转换失败: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"转换过程中出现错误: {e}")
        return False

def verify_models():
    """
    验证转换后的模型
    """
    print("验证转换后的模型...")
    
    # 检查ONNX模型
    onnx_models = list(current_dir.glob("onnx_models/*.onnx"))
    for model_path in onnx_models:
        try:
            import onnx
            model = onnx.load(model_path)
            onnx.checker.check_model(model)
            print(f"✓ ONNX模型验证成功: {model_path.name}")
        except Exception as e:
            print(f"✗ ONNX模型验证失败: {model_path.name} - {e}")
    
    # 检查OM模型
    om_models = list(current_dir.glob("om_models/*.om"))
    for model_path in om_models:
        if model_path.exists():
            size_mb = model_path.stat().st_size / (1024 * 1024)
            print(f"✓ OM模型: {model_path.name} ({size_mb:.2f} MB)")

def main():
    """主函数"""
    print("开始完整的模型转换流程...")
    
    # 设置路径
    weights_dir = current_dir / "weights"
    onnx_dir = current_dir / "onnx_models"
    om_dir = current_dir / "om_models"
    
    # 创建输出目录
    onnx_dir.mkdir(exist_ok=True)
    om_dir.mkdir(exist_ok=True)
    
    # 模型路径
    detect_model_path = weights_dir / "plate_detect.pt"
    rec_model_path = weights_dir / "plate_rec_color.pth"
    
    # ONNX输出路径
    detect_onnx_path = onnx_dir / "plate_detect.onnx"
    detect_onnx_opt_path = onnx_dir / "plate_detect_optimized.onnx"
    rec_onnx_path = onnx_dir / "plate_rec_color.onnx"
    rec_onnx_opt_path = onnx_dir / "plate_rec_color_optimized.onnx"
    
    # OM输出路径
    detect_om_path = om_dir / "plate_detect.om"
    rec_om_path = om_dir / "plate_rec_color.om"
    
    success_count = 0
    
    try:
        # 1. 导出检测模型为ONNX
        if export_to_onnx(str(detect_model_path), str(detect_onnx_path), (1, 3, 640, 640), "detection"):
            # 优化检测模型
            if optimize_onnx(str(detect_onnx_path), str(detect_onnx_opt_path)):
                # 转换为OM模型
                if convert_to_om(str(detect_onnx_opt_path), str(detect_om_path)):
                    success_count += 1
        
        # 2. 导出识别模型为ONNX
        if export_to_onnx(str(rec_model_path), str(rec_onnx_path), (1, 3, 48, 168), "recognition"):
            # 优化识别模型
            if optimize_onnx(str(rec_onnx_path), str(rec_onnx_opt_path)):
                # 转换为OM模型
                if convert_to_om(str(rec_onnx_opt_path), str(rec_om_path)):
                    success_count += 1
        
        print(f"\n=== 转换完成 ===")
        print(f"成功转换 {success_count}/2 个模型")
        
        # 验证模型
        verify_models()
        
        # 显示文件大小
        print("\n=== 文件大小 ===")
        for model_path in [detect_onnx_path, detect_onnx_opt_path, rec_onnx_path, rec_onnx_opt_path, detect_om_path, rec_om_path]:
            if model_path.exists():
                size_mb = model_path.stat().st_size / (1024 * 1024)
                print(f"{model_path.name}: {size_mb:.2f} MB")
        
        print(f"\n=== 部署说明 ===")
        print("1. ONNX模型可用于CPU/GPU推理")
        print("2. OM模型可用于香橙派AI Pro的昇腾NPU推理")
        print("3. 将OM模型复制到香橙派AI Pro设备上使用")
        
    except Exception as e:
        print(f"转换过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 