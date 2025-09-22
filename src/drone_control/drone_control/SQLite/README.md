# SQLite号牌数据库

这是一个简单的SQLite数据库类，用于管理号牌识别系统的数据。

## 功能特性

- **号牌管理**: 添加、删除、查询号牌信息
- **识别记录**: 记录每次号牌识别的详细信息
- **目标号牌**: 管理需要跟踪的目标号牌
- **统计信息**: 提供数据库使用统计

## 数据库结构

### 1. license_plates 表 (号牌表)
- `id`: 主键
- `plate_number`: 号牌号码 (唯一)
- `plate_type`: 号牌类型
- `confidence`: 识别置信度
- `location`: 位置信息
- `timestamp`: 创建时间
- `status`: 状态 (active/inactive)
- `notes`: 备注

### 2. recognition_records 表 (识别记录表)
- `id`: 主键
- `plate_number`: 号牌号码
- `confidence`: 识别置信度
- `image_path`: 图像文件路径
- `timestamp`: 识别时间
- `location`: 位置信息
- `notes`: 备注

### 3. target_plates 表 (目标号牌表)
- `id`: 主键
- `plate_number`: 号牌号码 (唯一)
- `priority`: 优先级
- `status`: 状态 (active/inactive)
- `created_at`: 创建时间
- `notes`: 备注

## 使用方法

### 基本使用

```python
from SQLite.license_plate_database import LicensePlateDatabase

# 创建数据库实例
db = LicensePlateDatabase("license_plates.db")

# 添加号牌
db.add_license_plate("京A12345", "普通车牌", 0.95, "北京市朝阳区", "测试数据")

# 设置目标号牌
db.set_target_plate("京A12345", 1, "主要目标")

# 添加识别记录
db.add_recognition_record("京A12345", 0.95, "/path/to/image.jpg", "北京市朝阳区", "自动识别")

# 检查号牌是否存在
exists = db.is_license_plate_exists("京A12345")

# 获取所有号牌
all_plates = db.get_all_license_plates()

# 获取目标号牌
target_plates = db.get_target_plates()

# 获取统计信息
stats = db.get_database_stats()

# 关闭数据库
db.close()
```

### 与LicensePlateProcessor集成

```python
from core.license_plate import LicensePlateProcessor
from SQLite.license_plate_database import LicensePlateDatabase

class EnhancedLicensePlateProcessor(LicensePlateProcessor):
    def __init__(self, logger, db_path="license_plates.db"):
        super().__init__(logger)
        self.db = LicensePlateDatabase(db_path)
    
    def process_license_plate(self, plate_text, confidence=0.0, image_path="", location=""):
        """处理号牌识别结果"""
        try:
            self.logger.info(f"[号牌处理] 收到号牌识别结果: {plate_text}")
            self.recognized_plate = plate_text
            
            # 添加到数据库
            self.db.add_license_plate(plate_text, "自动识别", confidence, location)
            
            # 添加识别记录
            self.db.add_recognition_record(plate_text, confidence, image_path, location)
            
            # 执行比对逻辑
            self._compare_license_plate()
            
        except Exception as e:
            self.logger.error(f"处理号牌识别结果失败: {e}")
    
    def set_target_license_plate(self, plate_number):
        """设置目标号牌"""
        super().set_target_license_plate(plate_number)
        self.db.set_target_plate(plate_number, 1, "无人机目标")
    
    def get_database_stats(self):
        """获取数据库统计"""
        return self.db.get_database_stats()
```

## 测试

运行测试文件来验证数据库功能：

```bash
cd src/drone_control/drone_control/SQLite
python test_database.py
```

## 注意事项

1. **数据库文件**: 数据库文件会在首次使用时自动创建
2. **连接管理**: 使用完毕后记得调用 `close()` 方法关闭连接
3. **并发访问**: 当前实现不支持多线程并发访问
4. **数据备份**: 建议定期备份数据库文件

## 扩展建议

1. **添加索引**: 为提高查询性能，可以添加适当的索引
2. **数据验证**: 可以添加号牌格式验证
3. **批量操作**: 可以添加批量插入和删除功能
4. **数据导出**: 可以添加数据导出功能
5. **日志记录**: 可以添加更详细的日志记录 