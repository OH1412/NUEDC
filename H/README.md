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

纯视频程序默认推送到PC `192.168.50.199:5600`；兼容简写命令
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

## 钢珠位置闭环控制

新增从球心三维定位到管道目标倾角的完整闭环，包括零点标定、常加速度
Kalman状态估计、串级PID、约束MPC、固定50 Hz八字节串口发送及10°自由滚落
参数辨识。完整符号定义、操作命令、参数含义、必测物理性质和安全调试步骤见
[`H/BALL_CONTROL.md`](BALL_CONTROL.md)。

首次使用必须依次完成：

```bash
# 1. 把球贴近原零点端（球心物理刻度0.5 cm）并标定
./H/start_ball_zero_calibration.sh -- --no-display --no-cpu-fallback

# 2. 先控制到管道几何中心（新坐标0 cm），只计算、不驱动电机
./H/start_ball_controller.sh --target-cm 0

# 3. 确认正角使位置下降后才启用串口
./H/start_ball_controller.sh \
  --target-cm 0 \
  --enable-serial

# 4. 实机调参时打开只读诊断（参数必须写在分隔符 -- 前）
./H/start_ball_controller.sh \
  --target-cm 0 \
  --enable-serial \
  --tuning-debug \
  -- \
  --no-display
```

控制器默认串级PID、工作限角 `±2°`，串口位移量程对应的倾角硬限位为
`±21.80°`。输入目标使用
以管道几何中心12.5 cm为0的新坐标：朝原零点端为负，朝原25 cm端为正。
零点标定实际记录物理刻度0.5 cm处的球心，所以该标定点显示为 `-12.0 cm`；
新坐标目标换成内部位移时统一加12.0 cm，不再另扣球半径。球心理论可达输入
范围为 `-12.0～+12.0 cm`，其中输入0控制到管道几何中心。实测视觉
约15～20 FPS，因此只在每个新视觉测量到达时更新
倾角，25 Hz循环负责超时监督，串口独立以50 Hz重发最新指令；超过0.25 s
没有新鲜测量会发送0°。控制器先在目标来向侧0.6 cm安全预停，连续低速后
再缓慢推进到内部±0.3 cm目标带。随机仿真和联合边界压力测试的命令、结果及
尚不能保证的极端工况见 [`H/BALL_CONTROL.md`](BALL_CONTROL.md)。
越过目标另一侧1 cm仍会锁存比赛失败记录，但默认不中断，而是继续控制钢珠
返回目标；仅显式加入 `--stop-on-competition-failure` 时才归零退出。
控制器默认约每秒输出1行精简状态；`--print-every 0` 可完全关闭，
`--telemetry full --print-every 5` 可恢复原完整调试JSON。
精简状态中的 `command_deg` 是控制倾角，`motor_mm` 是按
`250*tan(theta)`换算并量化后实际写入串口帧的电机升降毫米数。
默认精简状态使用短字段：`tgt`目标位置(cm)、`pos`实际位置(cm)、`err`位置
误差(cm)、`v_tgt`位置环给出的目标速度(cm/s)、`vel`视觉估计实际速度(cm/s)、
`deg`倾角命令和`mm`电机升降量。速度正值朝电机端、负值朝原固定端；失去
有效视觉时速度显示 `null`。需要完整名称及其他诊断字段时使用
`--telemetry full`。

钢珠速度在Kalman状态估计后额外经过按视觉时间戳计算的一阶低通，默认时间
常数为0.12 s。控制器、终端 `vel` 和实时曲线使用同一个滤波结果，位置曲线不
经过该层。配置项为 `velocity_filter_time_constant_s`；值越大越平滑但速度环
滞后越明显。该项可在调参UI“视觉速度滤波”分区实时修改并保存到参数方案；
失视重置和速度环重新启动会同步清空滤波器。
低通后还会判断静止：最近0.4 s位置跨度≤0.08 cm且速度绝对值≤0.10 cm/s时，
`vel`直接归零并清除滤波残余；明显移动时不钳零。这用于消除钢珠不动时由
0.2～0.4 mm视觉位置抖动产生的正负假速度。

普通启动的 `--target-cm` 可以省略，默认以管中心 `0 cm` 为目标。需要先单独
调速度环时使用 `--control-mode velocity`；该模式不设目标位置，绕过位置环、
预停点和位置死区，UI顶部改为设置 `-5～+5 cm/s` 目标速度，也可用
`--target-speed-cm-s` 给启动初值。速度PI、起滚补偿、角度限制以及按不同管段
更新的临时局部角度零点仍然生效。速度模式compact只输出
`pos、v_tgt、vel、deg、mm`，不会显示虚假的目标位置和位置误差。非零速度在
25 cm有限管道上采用方向相关端点保护：球心达到+11.5 cm且目标速度为正，或
达到-11.5 cm且目标速度为负时，锁存触发前最后倾角并暂停速度PI。速度模式
启动后默认0°暂停；UI提供“倾斜角返回0”和“启动速度环”按钮。锁存后先返回
0°，再启动；启动会丢弃旧视觉状态，下一次有效检测到钢珠才重新跟踪目标速度。
返回0和重新启动均保留已学习的局部角度零点。
控制器默认同时把原始视频推到 `192.168.50.199:5600`；`--no-stream` 可关闭，
`--stream-host PC_IP` 可在分隔符 `--` 前覆盖地址。
车辆纵向加速度前馈默认关闭；正方向定义为固定端（零点标定端）指向电机端，
正加速度对应负前馈倾角。当前只保留数据源接口、尚未读取加速度串口；以后用
`--enable-acceleration-feedforward` 显式开启。无有效样本时前馈安全归零，
`--test-cart-acceleration-m-s2 VALUE` 可在不访问硬件时验证符号。
不同球位置允许对应不同的小保持角且不必为0°：PID速度环积分会在当前目标
自动形成保持角；已知初值时可用 `--equilibrium-angle-bias-deg ANGLE`
按本次目标单独指定，MPC预测同样围绕该目标专属偏置进行。

串级PID启动时还会打开独立的实时调参窗口。窗口包含当前全部串级PID数字
参数、工作倾角上限、每视觉帧最大角度变化和平衡角初值。编辑后点击
“应用到当前控制器（不保存）”可立即试验；应用时清除旧积分，但从当前安全
倾角继续经过变化量限制，不会阻塞视觉循环和50 Hz串口线程。
启动时还会另开一个独立实时曲线窗口：位置模式显示速度和位置两个标签页，
速度模式只显示速度标签页。每页以视觉采样时间为横轴，用不同颜色同时绘制
目标值和实际值，保留最近30秒。曲线使用有界非阻塞队列，窗口渲染落后时丢
旧绘图点而不阻塞控制；比赛不需要本机曲线时用 `--no-plot-ui` 关闭。
窗口顶部还可实时设置本次运行目标点：范围 `-12.0～+12.0 cm`，滑条、输入框
和上下箭头联动，点击“应用目标点”后无需重启。切换目标会清除旧积分、接近
状态及旧目标的到达/越界判定，从当前倾角平滑控制新目标。目标点不保存进PID
参数方案，下次启动仍以命令行 `--target-cm` 为初始目标。
界面采用大号字体和分组滚动布局；每项参数同时提供联动的横向拖动条、可直接
键入的数值框以及上下箭头微调，右侧显示单位和允许范围。
其中“电机保持死区”默认是 `0.003 m`（0.3 cm）：位置进入该范围且钢珠已经
低速时保持上一电机位置；高速经过死区时仍继续制动，防止带速度越过目标。
该死区仅以最终请求目标判断，不会吞掉两阶段安全预停点前的剩余控制误差。
安全预停点的二阶段解除阈值还加入0.15 cm视觉噪声余量：最终误差不超过
0.75 cm并持续低速后即可开始推进。只要尚未进入最终0.3 cm保持死区，预停点
附近的小控制误差也允许静摩擦补偿按原1 deg/s速度缓慢建立，不再因小于
0.6 cm起滚门槛而停死；整体PID增益和2°工作限角均未提高。
管道局部不平引起“持续给非零角但球不动”时，控制器还会在运行期自动学习
临时局部角度零点：默认连续2秒不动后，把扣除当前动态P项、静摩擦补偿和
前馈后的保持偏置转入该管段临时0°；PID和静摩擦补偿仍在其上继续叠加。
刷新采用无扰转移，不会重复叠加补偿。运动时继续携带该偏置；在
新位置再次停滞时用当前倾角覆盖旧值。更新时间可通过参数方案中的
`local_zero_stall_time_s` 设置，默认2.0秒；临时零点数值本身不保存到
参数文件，失视重置或程序重启后不会保留；完整遥测可查看
`temporary_local_zero_deg`。

持续停滞时，控制器还会按 `stall_drive_boost_ramp_deg_s` 缓慢增加附加倾角，
上限为 `stall_drive_boost_max_deg`；默认分别为0.25 deg/s和1.0 deg。球恢复
运动后附加量以同样斜率退回0°，最终命令始终受工作倾角上限约束。

位置模式还会使用 `ball_control_config.json -> position_local_zero_prior` 给目标点
一个局部零点初值：当前 `-5 cm` 约为电机 `+0.65 mm`，`+5 cm` 约为
`-1.5 mm`，中间线性插值，范围外夹紧端点。毫米按 `atan(mm/250)` 换算为
倾角；这不会跳过停滞判断，达到原更新时间阈值后仍以实测学习值覆盖先验。

参数方案保存在 `H/control_profiles/名称.json`。窗口可以：

- 把当前输入另存为命名参数文件并立即应用；
- 从下拉框选择已有参数文件并应用；
- 重命名所选参数文件；
- 将最后一次选择并应用的参数文件记录为下次启动默认方案。

当前初始方案为 `low_oscillation_default.json`。仅临时点击“不保存”不会改变
下次启动默认方案。比赛无界面运行时增加 `--no-control-ui`：

```bash
./H/start_ball_controller.sh \
  --target-cm 5 \
  --enable-serial \
  --no-control-ui
```

无需相机、直接交互测试电机倾角：

```bash
./H/start_motor_angle_test.sh
```

默认测试限角为±10°，退出时停止周期发送后同步发送最终0°。具体命令和
`serial/send.py` 的适用区别见 [`H/BALL_CONTROL.md`](BALL_CONTROL.md)。
所有电机发送入口在每次打开串口后的第一帧位移命令前，都会先发送一次
`92 4F 4B 00 00 00 00 29` 电机使能帧，紧接着发送一次
`92 00 00 00 00 00 00 29` 零位初始化帧。控制倾角仅用于终端和内部计算，
线上数值按 `250*tan(倾角)` 换成毫米：正数抬高、负数下降，范围
`±99.99 mm`。
串口符号已恢复正常：正倾角发送正毫米数，负倾角发送负毫米数。公共开关位于
`H/angle_serial.py -> SERIAL_SIGN_INVERTED=False`。

直接持续发送单一电机角度也可使用统一环境入口：

```bash
./serial/start_send.sh 2.00
```

`--tuning-debug` 默认关闭；打开后每2秒在后台输出实际处理/有效视觉FPS、
延迟、位置误差、速度、参考速度和倾角，并根据不起滚、接近过快、振荡、
响应慢或方向异常给出下一次试验应修改的单个参数。它只给建议，不自动修改
配置或控制量；后台只保留最新诊断快照，不会因终端刷屏反压控制循环。

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

- `H/weights/steel_ball_v6_server_real_env.pt`
- 640×480、60 fps RGB 与深度流
- YOLO输入尺寸320、CUDA设备0
- `H/camera_to_base.json`

默认直接加载项目内的钢珠权重
`H/weights/steel_ball_v6_server_real_env.pt`。它复制自
`v6_server_real_env.pt`，源文件保持
不变。原有
`/home/pangolin/Downloads/best.engine` 和
`/home/pangolin/Downloads/best.pt` 均保留，可通过 `--weights` 显式切换，
没有删除或覆盖。显式使用 `.engine` 时若加载失败，仍可回退到默认
`steel_ball_v6_server_real_env.pt` CPU推理；添加
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
