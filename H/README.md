# H 题：钢珠 RGB-D 定位

本目录使用现有钢珠 YOLO 模型和 RealSense D435，输出：

1. 钢珠在彩色画面中的中心像素 `(u, v)`；
2. 彩色图对齐深度图中钢珠中心区域测得的球面深度 `surface_depth_m`；
3. 球面深度加0.005 m半径后的球心深度 `depth_m`；
4. 球心在RealSense彩色光学坐标系中的三维点 `camera_point_m`；
5. 球心在摄像头底座坐标系中的三维点 `camera_base_point_m`。

## 拍照和继续标注

启动RealSense手动拍照工具：

```bash
./H/start_capture_photos.sh
```

预览窗口按键：

- `空格` 或 `S`：保存当前无叠加的原始彩色照片；
- `Q` 或 `Esc`：退出。

拍照工具默认以640×480@30采集、每15帧刷新一次320×240窗口（约2 FPS
预览），保存的仍是当时最新的完整640×480原图。照片默认保存到
`H/captured_photos`，文件名形如
`ppr_20260729_130000_123_000001.jpg`。可通过参数修改目录或前缀：

```bash
./H/start_capture_photos.sh \
  --output-dir H/captured_photos \
  --prefix ppr
```

需要更高预览速度时可运行：

```bash
./H/start_capture_photos.sh \
  --fps 60 \
  --display-every 2 \
  --preview-scale 1.0
```

这台Jetson桌面负载较高，提高预览刷新率会增加X11窗口提交压力；仅为拍照
取景时建议保留默认2 FPS预览。

## 局域网视频推流

如果只需要PC显示纯摄像头画面，使用独立轻量推流程序。它不会加载YOLO、
TensorRT、Torch、深度流或任何识别算法：

```bash
./H/start_camera_video_stream.sh
```

纯视频程序默认推送到PC `192.168.50.43:5600`；兼容简写命令
`./H/starcamera_video_stream.sh`。如PC地址变化，仍可用
`--stream-host` 和 `--stream-port` 覆盖默认值。

默认以640×480@60采集、稳定30 FPS推送，使用3 Mbit/s的Jetson硬件H.264
编码。采集保持60 FPS可以总是选取较新的画面，发送端按单调时钟每秒发送30
帧，避免实际性能低于60 FPS时RTP时间戳持续漂移。按 `Ctrl+C` 停止。

Jetson和PC连接同一个局域网后，先在PC上查询PC自身IP。假设PC地址为
`10.6.99.50`，钢珠程序的纯摄像头画面推流命令为：

```bash
./H/start_ball_depth_tracker.sh \
  --no-cpu-fallback \
  --stream-host 10.6.99.50
```

PPR程序的纯摄像头画面推流：

```bash
./H/start_ppr_pipe_detector.sh \
  --mode light \
  --stream-host 10.6.99.50
```

注意 `--stream-host` 填PC地址，不是Jetson地址。当前Jetson无线地址为
`10.6.99.212/23`，PC通常也应位于 `10.6.98.0/23` 网段。

默认使用NVIDIA硬件H.264编码、RTP/UDP端口5600、30 FPS、2 Mbit/s。可调整：

```text
--stream-port 5600
--stream-fps 30
--stream-bitrate 2000000
```

PC使用GStreamer接收：

```bash
gst-launch-1.0 -v \
  udpsrc port=5600 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" \
  ! rtpjitterbuffer latency=50 drop-on-latency=true \
  ! rtph264depay ! h264parse ! avdec_h264 \
  ! videoconvert ! autovideosink sync=false
```

也可以把 `H/pc_receiver.sdp` 复制到PC，用VLC打开该SDP文件。Windows防火墙
需要允许VLC或UDP 5600入站。

Windows PC的一键接收脚本和完整操作说明位于 `H/PC`。将整个文件夹复制到
PC，连接 `NUEDC-H` 后双击 `start_receiver.bat`，脚本会显示PC地址、检查
防火墙并启动VLC。

推流使用单帧后台队列：网络或编码器繁忙时丢弃旧帧、保留最新帧，不阻塞
视觉识别和控制。视频不包含识别框、钢珠坐标、深度值、PPR轴线或检测
状态；识别结果只在Jetson本地使用。未指定 `--stream-host` 时不会启动编码器。

### 无路由器、无互联网时的离线热点

比赛现场不需要校园网。Jetson的 `wlan0` 支持AP模式，可让Jetson自身成为
Wi-Fi热点，PC直接连接。启动：

```bash
./H/start_offline_hotspot.sh
```

默认网络参数：

```text
热点名称：NUEDC-H
热点密码：NUEDC2026
Jetson固定地址：192.168.50.1
子网：192.168.50.0/24
```

启动热点会断开Jetson当前连接的校园Wi-Fi，这是无线网卡切换到AP模式的正常
现象。PC连接 `NUEDC-H` 后，通过 `ipconfig`（Windows）或 `ip addr`
（Linux）查看PC获得的 `192.168.50.x` 地址。必须使用本次实际获得的
地址，不能照抄旧地址。假设PC本次得到 `192.168.50.115`：

```bash
./H/start_camera_video_stream.sh --stream-host 192.168.50.115
```

`192.168.50.2` 只是旧示例，不是固定的PC地址。默认DHCP可能分配
`192.168.50.10`～`192.168.50.254` 中的任意地址；PC接收脚本会显示
本次正确地址。

Jetson上也可以用以下命令查看已经连接的PC地址：

```bash
ip neigh show dev wlan0
```

停止热点并恢复已保存的校园网连接：

```bash
./H/stop_offline_hotspot.sh SCUNET
```

热点名称和密码可在首次启动时指定，例如：

```bash
./H/start_offline_hotspot.sh MyNUEDC 12345678
```

## PPR 管道识别

独立运行：

```bash
./H/start_ppr_pipe_detector.sh
```

检测方案通过参数切换：

```bash
# 完整方案：轮廓 + HoughLinesP 双边线校正
./H/start_ppr_pipe_detector.sh --mode full

# 轻量方案：只做轮廓筛选和 fitLine，不运行霍夫
./H/start_ppr_pipe_detector.sh --mode light
```

两种模式的JSON字段和原图坐标定义完全相同：

| 模式 | 默认处理尺寸 | 算法 | 特点 |
|---|---:|---|---|
| `full` | 320×240 | 轮廓 + 霍夫双边线 | 断边适应性更好、中心轴校正更充分 |
| `light` | 320×240 | Otsu二值分割 + 连通轮廓 + `fitLine` | 独立轻量算法，不运行Canny和霍夫 |

`--process-scale`可以覆盖模式的默认处理比例。

轻量模式会同时尝试“亮管/暗背景”和“暗管/亮背景”两种二值极性，并使用
连通区域长宽比、长度、宽度、面积和矩形填充率过滤杂物。输出中的
`method` 为 `binary_fitline`，`binary_polarity` 表示最终采用的极性。
当前D435以640×480@60采集、无显示窗口连续运行120帧，轻量二值方案端到端
实测约39.7 FPS。

为防止Jetson桌面在识别时卡顿，程序默认：

- OpenCV最多使用2个CPU线程；
- 主结果窗口每2帧刷新一次；
- 轻量二值方案每种极性最多检查面积最大的64个轮廓；
- 默认不逐帧向VS Code终端打印JSON。

每条JSON结果包含 `processing_ms`，表示纯检测耗时。如果窗口卡但
`processing_ms`仍只有几毫秒，瓶颈在桌面/X11渲染而不是识别算法。可进一步
使用 `--display-every 3` 或 `--no-display`。

默认打开 D435 的 640×480、60 FPS 彩色流。处理流程为：

```text
灰度 → 高斯模糊 → Canny → 闭运算 → 轮廓筛选
                                    ↓
                        HoughLinesP 双长边配对
                                    ↓
                      中心轴、端点、角度、长宽
```

为保证Jetson实时显示流畅，相机仍采集640×480，但默认把检测图缩小为320×240
处理，最终中心和端点自动映射回640×480原图坐标。可用
`--process-scale 1.0` 强制全分辨率处理，但帧率会降低。

默认轮廓筛选条件：

- 轴线长度至少 180 px；
- 管道宽度 5～100 px；
- 长宽比至少 5；
- 轮廓面积至少 300 px²。

霍夫阶段只配对夹角不超过6°、投影重叠至少45%、间距5～100 px的两条长线，
降低桌面边缘、短电线和椅子扶手的误检。按 `q` 或 `Esc` 退出；添加
`--show-debug` 可同时查看 Canny 和闭运算结果。调试窗口默认每5帧刷新一次，
避免三个窗口每帧刷新造成桌面卡顿。

有效输出示例：

```json
{
  "valid": true,
  "center": {"u": 320.0, "v": 240.0},
  "endpoint_1": {"u": 140.0, "v": 210.0},
  "endpoint_2": {"u": 500.0, "v": 270.0},
  "angle_deg": 9.46,
  "length_px": 364.97,
  "width_px": 31.2,
  "aspect_ratio": 11.7,
  "method": "contour+hough"
}
```

只输出 JSON：

```bash
./H/start_ppr_pipe_detector.sh --no-display --print-every 1
```

正常启动时默认不向终端逐帧打印JSON，避免VS Code终端渲染大量文本拖慢桌面。
需要终端输出时使用 `--print-every N`；需要持续记录但不刷屏时使用
`--jsonl H/output/ppr.jsonl`。其他阈值可通过 `--help` 中的参数调整。

## 坐标关系

RealSense 摄像头光学坐标系采用：

- `+X`：画面向右；
- `+Y`：画面向下；
- `+Z`：镜头朝前。

定义依据为Intel RealSense SDK 2.0官方Projection文档。

底座坐标系采用 `+X` 向前、`+Y` 向左、`+Z` 向上。2026-07-30地面
标定结果为：相机光心高度0.262249 m、下俯37.98°、横滚-9.46°；
偏航按安装约束固定为镜头朝底座 `+X`，水平平移为零。转换公式为：

```text
x_base = -0.101179197 x_camera - 0.606963894 y_camera + 0.788262394 z_camera
y_base = -0.986389035 x_camera + 0.164428316 y_camera
z_base = -0.129612658 x_camera - 0.777533382 y_camera - 0.615339255 z_camera + 0.262249
```

`camera_to_base.json` 中采用：

```text
R_base_from_camera =
[[ -0.101179197, -0.606963894,  0.788262394],
 [ -0.986389035,  0.164428316,  0          ],
 [ -0.129612658, -0.777533382, -0.615339255]]

t_base_from_camera = [0, 0, 0.262249] m
```

该矩阵正交且行列式约为1，是右手旋转矩阵。这里按安装约束假设相机相对
底座没有前后和左右偏移。

### 用地面反推高度、俯仰角和横滚角

让RealSense画面只包含水平地面，然后运行：

```bash
./H/start_ground_calibration.sh
```

程序从深度点云中用RANSAC拟合地面，终端持续输出单次高度、下俯角、横滚
角、内点比例和平面RMSE。稳定样本至少达到3个后：

- 按空格或 `S`：在终端输出稳健高度、下俯角、旋转矩阵、平移向量和完整
  `camera_to_base.json` 建议；
- 按 `Q` 或 `Esc`：退出，不修改现有配置。

无窗口自动采集100帧并输出：

```bash
./H/start_ground_calibration.sh \
  --no-display \
  --auto-frames 100
```

水平地面能观测相机高度、俯仰角和横滚角。按安装约束固定镜头朝底座正前方，
即 `yaw=0°`，并固定水平平移 `x=0、y=0`。终端只输出当前实测的
`z、roll、pitch`、完整4×4齐次矩阵及JSON建议，不与任何名义值比较，也不会
自动覆盖 `H/camera_to_base.json`。应用前应核对内点比例和RMSE。

## RealSense 内参

相机内参直接使用 RealSense 设备的出厂标定结果，不手工填写。程序在取到
彩色帧后执行：

```python
color_frame.profile.as_video_stream_profile().get_intrinsics()
```

得到当前彩色流分辨率对应的 `fx、fy、ppx、ppy` 和畸变模型。深度图先通过
`rs.align(rs.stream.color)` 对齐到彩色图，再使用该彩色内参和
`rs.rs2_deproject_pixel_to_point()` 将 `(u, v, depth)` 反投影成摄像头光学
坐标系三维点。程序启动后会把实际读取到的内参打印一次，便于核对。

若以后改用 RealSense ROS 节点，同一类内参可从
`/camera/color/camera_info`（实际名称以 launch 的 namespace 为准）的
`K、D` 字段读取。当前程序直接控制相机，因此直接读 SDK profile 更简单，
也不会把其他分辨率的旧内参误用于当前图像。

## 运行

连接 D435 后：

```bash
./H/start_ball_depth_tracker.sh
```

按 `q` 或 `Esc` 退出。默认使用：

- `H/weights/steel_ball_best_2.pt`
- 640×480、60 fps RGB 与深度流
- YOLO输入尺寸320、CUDA设备0
- `H/camera_to_base.json`

默认直接加载项目内的钢珠权重
`H/weights/steel_ball_best_2.pt`。它复制自 `best(2).pt`，源文件保持
不变。原有
`/home/pangolin/Downloads/best.engine` 和
`/home/pangolin/Downloads/best.pt` 均保留，可通过 `--weights` 显式切换，
没有删除或覆盖。显式使用 `.engine` 时若加载失败，仍可回退到默认
`steel_ball_best_2.pt` CPU推理；添加
`--no-cpu-fallback` 可禁止回退。
默认CUDA设备0若因统一内存不足无法启动项目PT权重，同样会自动切换到CPU，
不会直接退出；CPU帧率会明显低于CUDA。

所有当前及历史钢珠权重已统一复制到 `H/weights`，文件用途、原始来源、
SHA-256及切换命令见 `H/weights/README.md`。

只输出 JSON、不打开窗口：

```bash
./H/start_ball_depth_tracker.sh --no-display
```

同时保存 JSON Lines：

```bash
./H/start_ball_depth_tracker.sh \
  --no-display \
  --jsonl H/output/ball_positions.jsonl
```

有效帧输出示例：

```json
{
  "valid": true,
  "pixel": {"u": 321.4, "v": 238.8},
  "surface_depth_m": 0.507,
  "ball_radius_m": 0.005,
  "depth_m": 0.512,
  "camera_point_m": {"x": 0.0012, "y": -0.0011, "z": 0.512},
  "camera_base_point_m": {"x": 0.443955, "y": -0.0012, "z": -0.005047}
}
```

实际输出还包含帧号、时间戳、检测框、类别、置信度、有效深度点数量和深度离散程度。检测不到钢珠时返回 `ball_not_detected`；检测到钢珠但有效深度不足时返回 `depth_invalid`，不会用 0 或背景距离冒充钢珠深度。

同一帧识别到多个合格钢珠时，程序只选择置信度最高的一个，且只输出该球的
像素、球心深度、摄像头坐标和底座坐标。其他低置信度球不会进入输出。

### 大框误识别过滤

根据实测日志，正常钢珠框约为 35～43 px × 40～50 px，面积占 640×480
画面的 0.47%～0.70%；偶发误框覆盖画面的 95%以上。程序默认拒绝：

- 面积超过整幅画面 3%的检测框；
- 宽度超过画面宽度 25%的检测框；
- 高度超过画面高度 25%的检测框。

被拒绝的框只在 JSON 中返回 `reason: detection_filtered`，不会出现在显示窗口。
若同一帧同时存在大误框和正常小框，程序忽略大框并继续使用正常小框。

阈值可以通过以下参数调整：

```text
--max-bbox-area-ratio 0.03
--max-bbox-width-ratio 0.25
--max-bbox-height-ratio 0.25
```

## 深度说明

`surface_depth_m` 是钢珠可见表面的RealSense测量值；程序默认加钢珠半径
0.005 m，得到球心 `depth_m`，再反投影得到球心相机坐标和底座坐标。钢珠
为反光金属，双目深度可能出现空洞，因此程序只统计YOLO框中心椭圆区域，
并过滤0值、超范围值和明显离群值。若有效点不足，结果标记为无效。

可调参数：

```text
--depth-roi-scale 0.45
--min-depth 0.08
--max-depth 3.0
--min-depth-samples 8
--ball-radius-m 0.005
```

## 测试

不连接相机也可以测试外参、深度采样以及模拟钢珠检测的完整反投影链路：

```bash
./.conda/envs/yolo-steel-ball/bin/python \
  -m unittest discover -s H/tests -v
```
