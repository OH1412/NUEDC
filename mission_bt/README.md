# NUEDC XML 行为树框架

这是运行在小电脑上的轻量行为树框架，结构参考 ROS 2 常用的
BehaviorTree.CPP：

- XML 只描述任务结构和节点参数；
- Python 插件注册具体行为节点；
- 核心执行器只处理节点状态和控制流；
- 串口、黑板等对象通过运行时上下文注入；
- 小车运动仍由 MCU 执行，小电脑只处理协议和上层任务。

## 当前 ACK 任务

默认树为 [config/simple_ack.xml](config/simple_ack.xml)：

```text
MCU 自行前进 1 m
→ 小电脑等待 76 01 00 00 00 00 00 67
→ 小电脑发送 92 10 00 00 00 00 00 29
→ MCU 自行左转 90°
→ 小电脑等待 76 01 00 00 00 00 00 67
→ 小电脑发送 92 11 00 00 00 00 00 29
→ MCU 自行前进 0.5 m
```

模拟运行：

```bash
cd /home/pangolin/NUEDC/mission_bt
./start_mock_mission.sh
```

串口实机：

```bash
cd /home/pangolin/NUEDC/mission_bt
./start_serial_mission.sh \
  --port /dev/ttyUSB0 \
  --baud 9600
```

串口默认值已经统一为 `9600 8N1`，因此也可以省略 `--baud 9600`。

指定其他 XML：

```bash
./run_mission.py \
  --tree /absolute/path/mission.xml \
  --transport serial \
  --port /dev/ttyUSB0
```

## XML 格式

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="Mission">
      <WaitSerialFrame
          name="WaitCar"
          frame="76 01 00 00 00 00 00 67"
          timeout_s="30"/>
      <SendSerialFrame
          name="ReplyCar"
          frame="92 10 00 00 00 00 00 29"/>
    </Sequence>
  </BehaviorTree>
</root>
```

支持多棵树和子树：

```xml
<SubTree ID="AnotherTree"/>
```

递归子树、重复 ID、未知节点、未知属性和缺少必填属性都会在启动阶段报错。

## 已注册节点

控制节点：

- `Sequence`：全部子节点成功才成功；
- `Fallback`：依次尝试，任一子节点成功即成功；
- `Inverter`：交换子节点的成功和失败状态；
- `SubTree`：引用另一棵 `BehaviorTree`。

通用插件节点：

- `Log message="..."`；
- `Delay seconds="1.0"`；
- `SetBlackboard key="..." value="..."`；
- `CheckBlackboard key="..." equals="..."`。

串口插件节点：

- `WaitSerialFrame frame="..." timeout_s="30"`；
- `SendSerialFrame frame="..."`。

查看实际注册结果：

```bash
./run_mission.py --list-nodes
```

## 插件规范

插件是普通 Python 文件，必须提供：

```python
def register_plugin(registry):
    registry.register("MyNode", builder)
```

`builder` 接口：

```python
def builder(name, attributes, children, context):
    return MyNode(...)
```

其中：

- `attributes` 是 XML 属性；
- `children` 是已经构建好的子节点；
- `context.transport` 是通信对象；
- `context.blackboard` 是共享黑板。

完整示例位于 [examples/custom_plugin.py](examples/custom_plugin.py)。运行：

```bash
./run_mission.py \
  --tree config/plugin_demo.xml \
  --plugin examples/custom_plugin.py \
  --transport mock
```

可以多次使用 `--plugin` 加载不同插件。

## 目录结构

```text
mission_bt/
├── config/                 XML 任务
├── examples/               外部插件示例
├── plugins/                内置行为插件
├── mission_bt/
│   ├── behavior.py         节点状态与控制流
│   ├── builtin_nodes.py    控制节点注册
│   ├── plugin_api.py       插件注册表、上下文和属性校验
│   ├── xml_loader.py       XML、SubTree 和插件加载
│   ├── transport.py        串口与 MCU 模拟器
│   └── protocol.py         8 字节协议解析
├── run_mission.py          通用运行器
└── tests/                  自动测试
```

## 测试

```bash
cd /home/pangolin/NUEDC/mission_bt
/usr/bin/python3 -m unittest discover -s tests -v
```

串口解析支持拆包、粘包和帧前杂散字节。等待帧超时后行为树返回失败，
不会执行后续 `SendSerialFrame`。
