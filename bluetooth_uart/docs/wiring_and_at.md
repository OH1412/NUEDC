# 接线与 AT 配置

## 硬件接线

### STM32/3.3 V MCU

```text
STM32 USART_TX -> 模块 RXD
STM32 USART_RX <- 模块 TXD
STM32 GND      -- 模块 GND
```

模块 VCC 必须根据实际底板丝印/说明书确定：

- 裸蓝牙核心板通常使用 3.3 V；
- 常见 HC-05 六针底板带稳压器，VCC 常可接 5 V；
- 即使底板 VCC 可接 5 V，也不要默认 RXD 是 5 V 容忍。

### 通过 USB-TTL 配置

```text
USB-TTL TXD -> 模块 RXD
USB-TTL RXD <- 模块 TXD
USB-TTL GND -- 模块 GND
```

USB-TTL 的逻辑电平选择 3.3 V。模块供电仍按底板要求连接。

## HC-05

常见完整 AT 模式：

1. 断电；
2. 按住底板按钮或把 KEY/EN 拉高；
3. 上电；
4. LED 慢闪；
5. 串口使用 `38400 8N1`，命令通常带 `CRLF`。

建议先查询，再修改：

```text
AT
AT+VERSION?
AT+ROLE?
AT+UART?
AT+NAME?
AT+PSWD?
```

典型从机配置：

```text
AT+ROLE=0
AT+NAME=NUEDC_CAR
AT+PSWD=1234
AT+UART=9600,0,0
AT+RESET
```

克隆固件的命令、等号和换行要求可能不同，以模块实际返回为准。

## DX-BT04

DX-BT04 不同后缀可能是 BLE 或经典+BLE 双模。常见配置规律：

- 未连接时进入 AT/命令模式；
- 默认 UART 常为 `9600 8N1`；
- 有些固件要求命令后加 `CRLF`；
- 建立蓝牙连接后自动进入透明传输，AT 命令不再生效。

先发送：

```text
AT
AT+VERSION
AT+NAME
```

不要在不知道具体后缀和固件时直接写入 `AT+BAUD...`。记录原始波特率、
名称、版本和 UUID 后再修改。

## Jetson AT 命令工具

模块通过 USB-TTL 接入后：

```bash
cd /home/pangolin/NUEDC/bluetooth_uart

./host/at_console.py \
  --port /dev/ttyUSB0 \
  --baud 38400 \
  --ending crlf \
  AT AT+VERSION?
```

BT04 默认 9600 的示例：

```bash
./host/at_console.py \
  --port /dev/ttyUSB0 \
  --baud 9600 \
  --ending crlf \
  AT AT+VERSION AT+NAME
```

每次只修改一项，重新读取确认，并在配置记录中写下最终波特率。
