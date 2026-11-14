# lib —— 纯算法层

## 职责

放三块不依赖 ESP-IDF 的代码，让它们能在 PC 上用 gcc 直接编译测试：

| 文件 | 内容 |
|---|---|
| `quat.h` | 四元数与三维向量运算，全部 inline |
| `vqf.c` / `vqf.h` | 单颗 IMU 的姿态滤波 |
| `frame_codec.c` / `frame_codec.h` | 姿态帧打包与 CRC |

上层模块（`fusion`、`io`）调用它们，PC 测试也调用它们。同一份代码，两边跑的是一模一样的逻辑。

## 为什么要单独抽出来

姿态算法出错时，如果只能在板子上调试，改一次要烧一次，一个晚上调不了几轮。

把这层抽成纯 C 之后，改完直接 `gcc` 编译跑测试，秒级看到结果。上板只剩「数据对不对」这一件事要验。

帧协议同理：C 端打包、Python 端解析，两边对拍通过才算协议定死。

## 接口约定

四元数一律 `[w, x, y, z]`，表示从机体系到世界系的旋转，单位四元数。

世界系 Z 轴朝上，与重力相反。机体系 x 朝前、y 朝右、z 按右手定则指向掌心。

所有角度用弧度，长度用米，加速度用 m/s²，磁场用微特斯拉。

## 测试

```bash
# 姿态滤波五个场景
gcc -O2 -I src -o /tmp/test_vqf tests/test_vqf.c src/lib/vqf.c -lm && /tmp/test_vqf

# 帧打包，再交给 Python 解析对拍
gcc -O2 -I src -o /tmp/test_frame tests/test_frame.c src/lib/frame_codec.c -lm
/tmp/test_frame /tmp/frame.bin && python3 tools/eic_frame.py --dump /tmp/frame.bin
```

两组测试都不需要连板子。

## 当前覆盖

| 场景 | 判据 |
|---|---|
| 静止 10 秒 | 姿态保持单位四元数，零偏收敛到零 |
| 绕 z 转 90 度 | 积分角误差小于 2 度 |
| 初始倾角偏 20 度 | 3 秒内收敛到 3 度以内 |
| 注入三轴零偏 | 20 秒静止后估计误差小于 0.005 rad/s |
| 航向清零 | 清零后 yaw 归零，误差小于 0.5 度 |
| 帧往返 | C 打包、Python 解析，四元数与温度全部一致 |
| 流式同步 | 混入日志文本后仍能正确提取所有帧 |

七项都通过。
