# H题PC纯视频接收方案

PC只负责接收和显示RealSense纯彩色视频，不接收识别框、坐标、深度或JSON。
默认协议为H.264 over RTP/UDP，端口5600。

## Windows + VLC

### 一、准备

1. 在PC安装VLC播放器。
2. 将整个 `H/PC` 文件夹复制到PC，不能只复制BAT文件。
3. Jetson运行 `./H/start_offline_hotspot.sh`。
4. PC连接Wi-Fi `NUEDC-H`，密码为 `NUEDC2026`。
5. Windows显示“无Internet”属于正常现象，不要断开该Wi-Fi。

### 二、启动PC接收端

双击：

```text
start_receiver.bat
```

第一次建议右键该文件并选择“以管理员身份运行”，脚本会：

1. 检查PC是否取得 `192.168.50.x` 地址；
2. 在窗口中显示PC接收地址和Jetson所需参数；
3. 放行Windows防火墙UDP 5600入站；
4. 使用200 ms抗抖缓存启动VLC并打开 `receiver.sdp`。

例如窗口显示PC地址为 `192.168.50.115`，就在Jetson运行：

```bash
PC_IP=192.168.50.115
./H/start_camera_video_stream.sh \
  --stream-host "$PC_IP" \
  --stream-port 5600
```

该程序以640×480@60采集、稳定30 FPS推送纯彩色视频，不加载YOLO、
TensorRT、Torch、深度流或识别算法。若确实需要一边识别、一边推流，才使用
钢珠或PPR程序的 `--stream-host` 参数。

PPR识别并推流的可选命令：

```bash
./H/start_ppr_pipe_detector.sh \
  --mode light \
  --no-display \
  --print-every 0 \
  --stream-host "$PC_IP" \
  --stream-port 5600
```

先启动PC接收端还是先启动Jetson推流端都可以。UDP没有连接握手，VLC启动后会
从收到下一个H.264关键帧开始显示。

## Linux PC + GStreamer

PC连接 `NUEDC-H` 后先查看本机地址：

```bash
ip -4 addr
```

然后启动接收端：

```bash
gst-launch-1.0 -v \
  udpsrc port=5600 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" \
  ! rtpjitterbuffer latency=50 drop-on-latency=true \
  ! rtph264depay ! h264parse ! avdec_h264 \
  ! videoconvert ! autovideosink sync=false
```

## 收不到画面时

按顺序检查：

1. PC地址必须是 `192.168.50.x`，Jetson固定为 `192.168.50.1`。
2. Jetson的 `--stream-host` 必须填写接收脚本本次显示的PC地址，不能照抄
   旧示例，也不能填写Jetson地址。
3. 两端端口必须同为5600。
4. 以管理员身份运行一次 `start_receiver.bat`，确保防火墙规则已建立。
5. 关闭可能占用UDP 5600的其他接收程序，只保留一个VLC实例。
6. Jetson运行 `ip neigh show dev wlan0`，应能看到PC地址。
