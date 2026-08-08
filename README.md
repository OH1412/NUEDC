# NUEDC 项目

> 本仓库为 NUEDC（竞赛与视觉方案）相关代码与工具集合，包含相机/深度感知、YOLO 摄像头、蓝牙串口、比赛控制与若干脚本/配置。

## 项目概述
- 相机与深度：`realsense` 相关脚本与绑定（见 `build-librealsense-python/`）
- 目标检测：`yolo_camera.py` 与若干启动脚本（`start_yolo_camera.sh`、`start_yolo_camera_trt.sh` 等）
- 比赛控制：`H/` 子目录包含比赛运行器、控制器与标定脚本
- 蓝牙：`bluetooth_uart/` 包含 BLE 相关依赖与启动脚本
- ROS2：包含若干 ROS2 启动/环境脚本（例如 `ros2_env.sh`, `start_realsense_ros2.sh`）

## 目录（部分）
- `H/`：比赛控制与相机标定
- `bluetooth_uart/`：蓝牙通信相关
- `build-librealsense-python/`：librealsense Python 包构建产物
- `ros2_ws/`：ROS2 工作空间（若存在）
- `yolo_camera_captures/`：YOLO 摄像头捕获样本

## 依赖
- Python 3.8+（视具体脚本要求）
- 系统依赖：`librealsense`、相机驱动、ROS2 Foxy（如使用 ROS2 部分）
- Python 依赖文件：

```bash
pip install -r nuedc-extra-requirements.txt
# 若使用蓝牙子模块：
pip install -r bluetooth_uart/requirements-ble.txt
```

## 环境设置（快速）
1. 创建并激活 Python 虚拟环境：

```bash
python -m venv .venv
source .venv/bin/activate    # Linux / macOS
.venv\\Scripts\\activate     # Windows (PowerShell)
pip install -r nuedc-extra-requirements.txt
```

2. 若使用 RealSense：安装 librealsense 并参考 `build-librealsense-python/` 中的说明进行绑定安装。
3. 若使用 ROS2：运行 `ros2_env.sh` 来准备 ROS2 环境（适用于已安装 ROS2 的系统）。

## 常用脚本
- 启动 YOLO 摄像头（CPU）：`./start_yolo_camera.sh`
- 启动 YOLO 摄像头（TensorRT）：`./start_yolo_camera_trt.sh`
- 启动 RealSense ROS2 节点：`./start_realsense_ros2.sh`
- 运行相机测试脚本：`python realsense_test.py`
- 启动比赛核心：`./H/start_competition.sh` 或参考 `H/` 中的说明

## 运行示例
使用默认摄像头运行 YOLO 摄像头：

```bash
./start_yolo_camera.sh
```

采集并保存图片（示例）：

```bash
python H/capture_photos.py --output yolo_camera_captures/
```

## 测试
- 仓库中存在 `tests/` 目录，可运行其中的测试脚本或自定义测试流程。

## 贡献与联系方式
欢迎提交 issue 或 PR。对于大改动，请先在 issue 中讨论设计与兼容性。

## 许可证
请在提交前确认本仓库适用的许可证信息（如需添加 LICENSE 文件，请告知）。
