# NUEDC 蓝牙串口（BT24 / HC-05 / BT04）

本目录用于把小车 MCU 的 UART 协议无线传到 Jetson 小电脑。

先确认模块型号：

| 模块 | 空中协议 | Jetson 端形态 | 推荐入口 |
|---|---|---|---|
| BT24 | BLE GATT | 特征值 Write/Notify | `start_bt24_listen.sh` |
| HC-05 | 经典蓝牙 SPP | `/dev/rfcomm0` | `host/hc05_rfcomm.sh` |
| DX-BT04 双模的 SPP 模式 | 经典蓝牙 SPP | `/dev/rfcomm0` | 同 HC-05 |
| BT04/HM-10 类 BLE 模式 | BLE GATT | 特征值 Write/Notify | `host/ble_pty_mission.py` |

不要只根据“蓝牙 4.0”字样判断。先扫描：经典蓝牙能配对并发现
Serial Port 服务；BLE 设备则需要查看 GATT characteristic。

## 1. 接线

模块装在小车 MCU 一侧：

```text
MCU 3.3V UART TX  ─────>  蓝牙模块 RX
MCU 3.3V UART RX  <─────  蓝牙模块 TX
MCU GND            ─────  蓝牙模块 GND
电源               ─────  模块 VCC（按具体底板标注）
```

必须共地，TX/RX 必须交叉。裸模块通常是 3.3 V；带稳压底板的 HC-05
常允许 VCC 接 5 V，但 RX 逻辑脚仍应按 3.3 V 对待。STM32 的 3.3 V UART
一般可以直接连接。不要把 5 V TTL 直接送进模块 RX 或 Jetson UART。

详细内容见 [docs/wiring_and_at.md](docs/wiring_and_at.md)。

## 2. MCU 串口参数

本项目蓝牙 UART 固定为：

```text
9600 baud, 8 data bits, no parity, 1 stop bit（8N1）
```

BT24 和 MCU 两端都使用 `9600 8N1`、关闭硬件及软件流控。Jetson 与
BT24 之间使用 BLE GATT，不存在需要设置的串口波特率。

## 3. HC-05 / SPP 快速使用

扫描：

```bash
cd /home/pangolin/NUEDC/bluetooth_uart
./host/hc05_rfcomm.sh scan
```

记下类似 `AA:BB:CC:DD:EE:FF` 的地址，然后配对：

```bash
./host/hc05_rfcomm.sh pair AA:BB:CC:DD:EE:FF
```

常见 PIN 为 `1234` 或 `0000`。绑定 Serial Port channel 1：

```bash
./host/hc05_rfcomm.sh bind AA:BB:CC:DD:EE:FF 1
ls -l /dev/rfcomm0
```

让 XML 行为树通过蓝牙串口运行：

```bash
cd /home/pangolin/NUEDC/mission_bt
./start_serial_mission.sh --port /dev/rfcomm0 --baud 9600
```

释放：

```bash
/home/pangolin/NUEDC/bluetooth_uart/host/hc05_rfcomm.sh release
```

## 4. 当前 BT24（已实机确认）

当前模块装在单片机一侧，Jetson 是 BLE Central。它不是经典蓝牙 SPP，
因此不需要 `/dev/rfcomm0`，也不要把“系统设置里配对成功”作为通信条件。

本机已经实测连接并识别出：

```text
名称：BT24
地址：48:87:2D:73:E1:B0
UART 服务：0000ffe0-0000-1000-8000-00805f9b34fb
收发/通知：0000ffe1-0000-1000-8000-00805f9b34fb
只写特征：0000ffe2-0000-1000-8000-00805f9b34fb
```

只监听单片机发来的数据，不向小车发送任何内容：

```bash
cd /home/pangolin/NUEDC/bluetooth_uart
./start_bt24_listen.sh
```

脚本会一直监听并实时输出十六进制数据，按 `Ctrl+C` 停止；蓝牙意外断开后
每 2 秒自动尝试重连。旧命令 `./start_bt24_listen.sh 30` 仍可运行，但
参数 `30` 不再限制监听时间。

测试时可让单片机每秒发送一次已知帧。监听脚本收到
`76 01 00 00 00 00 00 67` 即证明 MCU -> BT24 -> Jetson 全链路正常。

运行 XML 任务前先架空车轮或清空场地。脚本有安全锁，必须显式解锁：

```bash
./start_bt24_xml_mission.sh --arm
```

BT24 与 Jetson 之间走 BLE；“波特率”只作用于单片机 UART 与 BT24 模块
之间。本项目固定 BT24 和 MCU 都使用 9600 8N1。BLE 能连接但收不到
正确数据时，优先检查：

1. MCU TX 是否接 BT24 RX、MCU RX 是否接 BT24 TX，并且已经共地；
2. MCU UART 与 BT24 透明传输波特率是否一致；
3. 单片机是否确实在测试监听期间调用了串口发送；
4. 手机或其他主机是否仍占用这个 BLE 连接。

## 5. 通用 BLE 快速使用

安装到本目录私有 `vendor/`（不写系统 Python）：

```bash
cd /home/pangolin/NUEDC/bluetooth_uart
./setup_ble_env.sh
```

扫描：

```bash
./host/ble_tool.sh scan
```

检查服务和特征：

```bash
./host/ble_tool.sh inspect AA:BB:CC:DD:EE:FF
```

找出：

- 带 `write` 或 `write-without-response` 属性的特征 UUID；
- 带 `notify` 属性的特征 UUID。

不同 BT04 固件 UUID 不完全相同，不要盲目照抄。确定 UUID 后，让 BLE
连接桥接到 XML 行为树：

```bash
./host/run_ble_xml_mission.sh \
  --address AA:BB:CC:DD:EE:FF \
  --write-char 0000xxxx-0000-1000-8000-00805f9b34fb \
  --notify-char 0000yyyy-0000-1000-8000-00805f9b34fb
```

## 6. MCU 参考代码

[mcu/](mcu/) 提供：

- 单生产者/单消费者循环缓冲区；
- UART DMA + IDLE 接收适配示例；
- 逐字节状态机拆包；
- 黏包、分包、噪声恢复；
- 小车端任务状态机示例。

本机测试：

```bash
cd /home/pangolin/NUEDC/bluetooth_uart
make test
```

## 7. 比赛建议

- 蓝牙模块只做透明传输，包边界由 MCU/小电脑协议层处理。
- 不要假设一次 UART/DMA/BLE 回调就是一整帧。
- 当前两次完成帧相同，不能安全重发；比赛版应先加入阶段号/序号和 CRC，
  再设置 ACK 超时和有限次数重发。
- 电机电源与逻辑电源做好去耦，蓝牙模块旁放置 0.1 µF + 10 µF。
- 天线区域远离电机、电调、大面积铜皮和金属支架。
- 首次联调先关闭电机，只验证十六进制帧收发。
