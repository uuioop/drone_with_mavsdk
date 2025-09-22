# 门牌数据集生成工具

这个工具用于生成门牌检测模型的训练数据集。

## 功能特性

- 🎯 生成标准YOLO格式的门牌数据集
- 🎨 支持多种门牌样式（标准蓝、深蓝、浅蓝）
- 📝 自动生成多样化的门牌文字内容
- 🏷️ 自动生成YOLO格式的标签文件
- �� 支持批量生成（默认1000张图像）
- 📏 严格按照真实门牌比例（1.679:1）

## 文件结构

```
plate_detection/
├── generate_plates.py          # 主生成脚本
├── verify_dataset.py           # 数据集验证脚本
├── visualize_plate.py          # 可视化脚本
├── preview_plates.py           # 预览脚本
├── SourceHanSansCN-Heavy.ttf   # 中文字体文件
├── output/                     # 生成的数据集
│   ├── images/                 # 图像文件 (.jpg)
│   └── labels/                 # 标签文件 (.txt, YOLO格式)
└── DATASET_GENERATION.md       # 本文档
```

## 使用方法

### 1. 生成数据集

```bash
# 在plate_detection目录下运行
python3 generate_plates.py
```

### 2. 验证数据集

```bash
# 验证生成的数据集是否正确
python3 verify_dataset.py
```

### 3. 可视化数据集

```bash
# 可视化生成的门牌图像和边界框
python3 visualize_plate.py
```

### 4. 预览数据集

```bash
# 预览数据集基本信息
python3 preview_plates.py
```

## 配置参数

在 `generate_plates.py` 中可以修改以下参数：

```python
# 图像尺寸
IMG_WIDTH = 640
IMG_HEIGHT = 640

# 门牌长宽比（根据真实门牌计算）
PLATE_ASPECT_RATIO = 1.679  # 564:336 = 1.679

# 生成数量
NUM_IMAGES = 1000

# 字体文件路径
FONT_PATH = "SourceHanSansCN-Heavy.ttf"

# 输出目录
OUTPUT_DIR = "output"

# 外边距比例
margin_ratio = 0.15  # 15%外边距
```

## 数据集格式

### 图像格式
- **尺寸**: 640x640 像素
- **格式**: JPG
- **质量**: 95%

### 标签格式
YOLO格式，每行一个目标：
```
class_id center_x center_y width height
```

- `class_id`: 0 (门牌类别)
- `center_x, center_y`: 边界框中心点坐标 (归一化到0-1)
- `width, height`: 边界框宽度和高度 (归一化到0-1)

### 示例标签文件
```
0 0.500000 0.499219 0.700000 0.417187
```

## 门牌样式

脚本支持三种门牌样式：

1. **标准门牌**: 蓝色背景 (0, 102, 204)
2. **深蓝门牌**: 深蓝色背景 (0, 51, 102)
3. **浅蓝门牌**: 浅蓝色背景 (51, 153, 255)

## 门牌规格

- **长宽比**: 1.679:1（严格按照真实门牌比例）
- **外边距**: 15%（图像边缘到门牌的距离）
- **边框**: 8像素白色边框
- **字体**: SourceHanSansCN-Heavy.ttf（粗体）
- **字体大小**: 80像素

## 门牌内容

生成的门牌包含以下内容：

- **第一行**: 街道名称-门牌号 (如: 天马路-001)
- **第二行**: 楼栋号-房间号 (如: 1-101)

### 街道名称列表
- 天马路, 中山路, 人民路, 建设路, 和平路
- 解放路, 新华路, 文化路, 科技路, 工业路
- 商业路, 教育路, 健康路, 幸福路, 和谐路, 发展路

## 训练使用

生成的数据集可以直接用于YOLOv5训练：

1. **准备数据集**:
   ```bash
   # 将output目录重命名为你的数据集名称
   mv output doorplate_dataset
   ```

2. **创建数据集配置文件**:
   ```yaml
   # data/doorplate.yaml
   path: ../doorplate_dataset  # 数据集根目录
   train: images  # 训练图像路径
   val: images    # 验证图像路径
   
   nc: 1  # 类别数量
   names: ['doorplate']  # 类别名称
   ```

3. **开始训练**:
   ```bash
   python train.py --data data/doorplate.yaml --weights yolov5s.pt --epochs 100
   ```

## 注意事项

1. **字体文件**: 确保 `SourceHanSansCN-Heavy.ttf` 字体文件存在
2. **存储空间**: 1000张图像大约需要40MB存储空间
3. **生成时间**: 生成1000张图像大约需要1-2分钟
4. **图像质量**: 生成的图像质量较高，适合训练使用
5. **长宽比**: 严格按照1.679:1的比例生成，确保与真实门牌一致

## 故障排除

### 常见问题

1. **字体文件未找到**
   ```
   错误：找不到字体文件 'SourceHanSansCN-Heavy.ttf'
   ```
   - 解决方案：确保字体文件在正确位置

2. **权限问题**
   ```
   Permission denied: 'output'
   ```
   - 解决方案：检查目录权限，确保有写入权限

3. **内存不足**
   ```
   MemoryError
   ```
   - 解决方案：减少 `NUM_IMAGES` 数量，分批生成

## 扩展功能

可以根据需要扩展以下功能：

1. **更多样式**: 添加更多门牌样式和颜色
2. **数据增强**: 添加旋转、缩放、噪声等增强
3. **多类别**: 支持多种门牌类型
4. **背景变化**: 添加不同的背景纹理
5. **文字变化**: 支持更多文字格式和字体

## 许可证

本项目遵循项目主许可证。 