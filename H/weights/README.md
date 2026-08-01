# 钢珠识别权重

本目录统一保存H题钢珠识别使用过的权重。复制后原始文件均保留，不在这里
删除或覆盖。

| 项目文件 | 状态 | 原始文件 | SHA-256 |
|---|---|---|---|
| `steel_ball_v6_server_real_env.pt` | 当前默认 | 微信 `v6_server_real_env.pt` | `37ab4b267984a2b3974b4fd9596ad92d375a4a2baefa4e1d568a7eca8f902ad0` |
| `steel_ball_best_4.pt` | 历史PT | 微信 `best(4).pt` | `75d6dabe3205a9c6dac4d24fc15c46886a3a28093bf8653a6f16fb00ab420aff` |
| `steel_ball_best_3.pt` | 历史PT | 微信 `best(3).pt` | `1e0ca5134ccfeb4629ff6cf5e9a9fc80221bba84daccdf8e03d4cca284438aaf` |
| `steel_ball_v5.pt` | 历史PT | 微信 `v5.pt` | `4498de7d983f0a4a96bdcdf15151e2be188888ef2d22f3b6b4e6bbd02a1b9486` |
| `steel_ball_best_2.pt` | 历史PT | 微信 `best(2).pt` | `bad647dfbaaa381234e55ee8a47b48c8cc2b74a5f94771b4e69a007f99fb0697` |
| `steel_ball_v4_interrupted_best.pt` | 历史PT | 微信 `v4_interrupted_best.pt` | `43d0e30b356999b3a0301144ddf19310da297afe66d0d4590e60e7ff41b375e2` |
| `steel_ball_best.pt` | 历史PT | 微信 `best(1).pt` | `e44621f19377cab0d6ec59c048ff5b0ca8a84b8f99ebbc127e21f95b5c74c7a7` |
| `steel_ball_v3.pt` | 历史PT | 微信 `v3.pt` | `a8ace794752e7f4f85079e56497c08c9554bee078b65b52ade0e2350181a1eaf` |
| `steel_ball_best_legacy.pt` | 历史PT | Downloads `best.pt` | `bd1a1a447bfcece5fcd71cc4fbc3451bfd664aec1330bf86a992541d8999dc8d` |
| `steel_ball_best_legacy.engine` | 历史TensorRT | Downloads `best.engine` | `d78a50a8443d1517f167e817027e42c6d26fbdcdc8375d10368aff1dffffd0ac` |

默认运行：

```bash
./H/start_ball_depth_tracker.sh
```

临时切换旧权重：

```bash
./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_best_3.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_v5.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_best_2.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_v4_interrupted_best.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_best.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_v3.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_best_legacy.pt

./H/start_ball_depth_tracker.sh \
  --weights H/weights/steel_ball_best_legacy.engine
```
