#!/usr/bin/env python3
"""
号牌数据库管理脚本
用于向数据库中添加、查询、删除号牌数据
"""

import sys
import os

# 添加路径以导入数据库模块
sys.path.append('src/drone_control/drone_control')

from SQLite.license_plate_database import LicensePlateDatabase

def main():
    # 连接到数据库
    db = LicensePlateDatabase("test_license_plates.db")
    
    print("=== 号牌数据库管理工具 ===")
    print("1. 添加号牌")
    print("2. 查看所有号牌")
    print("3. 删除号牌")
    print("4. 设置目标号牌")
    print("5. 查看目标号牌")
    print("6. 删除目标号牌")
    print("7. 添加识别记录")
    print("8. 查看数据库统计")
    print("9. 清空数据库")
    print("10. 退出")
    
    while True:
        choice = input("\n请选择操作 (1-10): ").strip()
        
        if choice == '1':
            # 添加号牌
            plate_number = input("请输入号牌号码: ").strip()
            plate_type = input("请输入号牌类型 (默认: 普通车牌): ").strip() or "普通车牌"
            confidence = float(input("请输入置信度 (0-1, 默认: 1.0): ").strip() or "1.0")
            location = input("请输入位置信息 (可选): ").strip()
            notes = input("请输入备注 (可选): ").strip()
            
            if db.add_license_plate(plate_number, plate_type, confidence, location, notes):
                print(f"✅ 成功添加号牌: {plate_number}")
            else:
                print(f"❌ 添加号牌失败: {plate_number}")
        
        elif choice == '2':
            # 查看所有号牌
            plates = db.get_all_license_plates()
            if plates:
                print("\n📋 所有号牌:")
                for plate in plates:
                    print(f"  {plate['plate_number']} | {plate['plate_type']} | 置信度: {plate['confidence']} | {plate['location']}")
            else:
                print("📭 数据库中没有号牌数据")
        
        elif choice == '3':
            # 删除号牌
            plate_number = input("请输入要删除的号牌号码: ").strip()
            confirm = input(f"确认删除号牌 '{plate_number}' 吗? (y/N): ").strip().lower()
            
            if confirm in ['y', 'yes', '是']:
                if db.remove_license_plate(plate_number):
                    print(f"🗑️ 成功删除号牌: {plate_number}")
                else:
                    print(f"❌ 删除号牌失败: {plate_number}")
            else:
                print("❌ 取消删除操作")
        
        elif choice == '4':
            # 设置目标号牌
            plate_number = input("请输入目标号牌号码: ").strip()
            priority = int(input("请输入优先级 (1-10, 默认: 1): ").strip() or "1")
            notes = input("请输入备注 (可选): ").strip()
            
            if db.set_target_plate(plate_number, priority, notes):
                print(f"🎯 成功设置目标号牌: {plate_number}")
            else:
                print(f"❌ 设置目标号牌失败: {plate_number}")
        
        elif choice == '5':
            # 查看目标号牌
            targets = db.get_target_plates()
            if targets:
                print("\n🎯 目标号牌:")
                for target in targets:
                    print(f"  {target['plate_number']} | 优先级: {target['priority']} | {target['notes']}")
            else:
                print("🎯 没有设置目标号牌")
        
        elif choice == '6':
            # 删除目标号牌
            plate_number = input("请输入要删除的目标号牌号码: ").strip()
            confirm = input(f"确认删除目标号牌 '{plate_number}' 吗? (y/N): ").strip().lower()
            
            if confirm in ['y', 'yes', '是']:
                if db.remove_target_plate(plate_number):
                    print(f"🗑️ 成功删除目标号牌: {plate_number}")
                else:
                    print(f"❌ 删除目标号牌失败: {plate_number}")
            else:
                print("❌ 取消删除操作")
        
        elif choice == '7':
            # 添加识别记录
            plate_number = input("请输入识别到的号牌号码: ").strip()
            confidence = float(input("请输入识别置信度 (0-1): ").strip())
            image_path = input("请输入图像路径 (可选): ").strip()
            location = input("请输入位置信息 (可选): ").strip()
            notes = input("请输入备注 (可选): ").strip()
            
            if db.add_recognition_record(plate_number, confidence, image_path, location, notes):
                print(f"📸 成功添加识别记录: {plate_number}")
            else:
                print(f"❌ 添加识别记录失败: {plate_number}")
        
        elif choice == '8':
            # 查看数据库统计
            stats = db.get_database_stats()
            print(f"\n📊 数据库统计:")
            print(f"  总号牌数: {stats['total_plates']}")
            print(f"  识别记录数: {stats['total_records']}")
            print(f"  目标号牌数: {stats['total_targets']}")
        
        elif choice == '9':
            # 清空数据库
            confirm = input("⚠️  确认清空整个数据库吗? 这将删除所有数据! (y/N): ").strip().lower()
            
            if confirm in ['y', 'yes', '是']:
                double_confirm = input("🚨 最后确认: 输入 'DELETE' 来确认清空数据库: ").strip()
                if double_confirm == 'DELETE':
                    if db.clear_database():
                        print("🗑️ 数据库已清空!")
                    else:
                        print("❌ 清空数据库失败")
                else:
                    print("❌ 确认码不正确，取消清空操作")
            else:
                print("❌ 取消清空操作")
        
        elif choice == '10':
            print("👋 再见!")
            break
        
        else:
            print("❌ 无效选择，请输入 1-10")
    
    # 关闭数据库连接
    db.close()

if __name__ == "__main__":
    main()
