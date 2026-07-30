# E 题工具

## 图形界面

运行简易桌面界面：

```bash
./E/start_polygon_gui.sh
```

也可以运行：

```bash
python3 E/polygon_gui.py
```

使用流程：

1. 选择“绝对方向角”或“内角”。
2. 输入边数并点击“生成输入框”。
3. 逐边输入边长和角度，未知量所在的输入框填写 `x`。
4. 点击“计算并绘图”。
5. 在左侧查看数值结果，在右侧查看图形；需要时点击“保存 PNG”。

## 多边形单未知量求解器

`polygon_solver.py` 根据多边形闭合条件计算一个未知边长或角度。输入中必须恰好有一个 `x`。

交互运行：

```bash
python3 E/polygon_solver.py
```

也可直接通过参数运行。下面的四边形缺少第三条边的长度：

```bash
python3 E/polygon_solver.py \
  --mode direction \
  --sides 4 \
  --data 3 0 4 90 x 180 4 270
```

结果为 `x = 3`。

程序会同时绘制多边形，并默认保存到：

```text
E/output/polygon.png
```

桌面环境下会同时弹出绘图窗口。只保存、不弹窗：

```bash
python3 E/polygon_solver.py \
  --mode direction \
  --sides 4 \
  --data 3 0 4 90 x 180 4 270 \
  --no-show
```

指定图片位置：

```bash
python3 E/polygon_solver.py \
  --mode direction \
  --sides 4 \
  --data 3 0 4 90 x 180 4 270 \
  --output E/output/my_polygon.png \
  --no-show
```

图中会显示顶点编号、边号、边长和角度，含未知量 `x` 的边用红色突出显示。若数据不能闭合，还会用橙色虚线标出闭合残差。

两种角度模式：

- `direction`：角度是每条边相对 x 轴正方向的绝对方向角，逆时针为正。该模式适合视觉轮廓坐标。
- `interior`：角度是每条边终点处的多边形内角，按逆时针方向遍历。第一条边会被视为 0°方向。

内角未知示例：

```bash
python3 E/polygon_solver.py \
  --mode interior \
  --sides 4 \
  --data 3 90 4 x 3 90 4 90
```

结果为 `x = 90°`。

脚本求出 `x` 后会重建全部顶点，并打印闭合残差。如果残差超过容差，说明虽然能得到一个候选值，但其余已知数据无法共同组成闭合多边形。
