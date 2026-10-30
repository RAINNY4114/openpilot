# patches/

设备侧的源码补丁存档。目的是**让改动跟着 git 走**，不再因为升级/重装而静默丢失。

## 0001-camerad-ife-resize-1344x760.patch

**作用**：narrow road 相机（`camera_num = 1`，也就是 `modeld` 主输入 `narrowRoadCameraState`）
在 IFE 里做硬件降采样，`1928x1208 → 1344x760`，用来压低 MIPI / USB 带宽。

**原理**：`cameras/ife.h` 的缩放寄存器是 16.16 定标比 `((out-1) << 16) | (in-1)`，
非整数比也支持。但原来的 out 尺寸只能由 `frame_width / out_scale` 得到（只支持整数倍），
所以给 `CameraConfig` 加了 `out_width` / `out_height`，0 表示跟随传感器全分辨率。

改动文件：

| 文件 | 改动 |
|---|---|
| `openpilot/system/camerad/cameras/hw.h` | `CameraConfig` 加 `out_width`/`out_height`；`NARROW_ROAD_CAMERA_CONFIG` 设 1344/760，另两个设 0 |
| `openpilot/system/camerad/cameras/spectra.cc` | `camera_open()` 里 out 尺寸优先取配置值 |

**历史**：sptest1026 的 `9eee54b` 做过同样的事，但没入库；2026-10-28 升级到
sunnypilot v2026.003.000 后又丢了一次。注意 camerad 已从 `selfdrive/camerad`
搬到 `system/camerad`，老的 patch 路径全部失效。

## 怎么用

```bash
cd /data/openpilot

# 应用（改动已在工作区时用 -R 反向校验）
git apply --check patches/0001-camerad-ife-resize-1344x760.patch
git apply         patches/0001-camerad-ife-resize-1344x760.patch

# 回退
git apply -R      patches/0001-camerad-ife-resize-1344x760.patch
```

## ⚠️ 只改源码不够，必须重新编译 camerad

当前这个 release（sunnypilot v2026.003.000）的仓库里**没有 `SConstruct` / `SConscript`**，
`openpilot/system/camerad/camerad` 是随包预编译的二进制。所以在设备上**无法直接重编**。

改完源码后，`camerad` 二进制仍是全分辨率版本，**resize 不会生效**。要生效需要：

1. 在一份**带完整构建工具**的 openpilot 源码树里编译 camerad（同架构 aarch64，同版本依赖）；
2. 把产物覆盖到 `/data/openpilot/openpilot/system/camerad/camerad`；
3. 重启 comma（见 `/data/safe_restart.sh`）。

已做的验证（2026-10-28）：用 `compile_commands.json` 里的真实编译参数
（含 `-Werror`、`-mcpu=cortex-a57`）对 camerad 全部 7 个翻译单元做 `-fsyntax-only`，
全部通过。也就是说**语法和类型层面是干净的，剩下的是链接/产物层面的验证**。

运行时验证必须在车上做：onroad 后看 `narrowRoadCameraState` 的 `frameWidth`/`frameHeight`
是否为 1344/760，以及画面比例是否正常（IFE 缩放不改变视场，只是降采样）。
