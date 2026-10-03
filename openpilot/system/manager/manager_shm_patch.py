#!/usr/bin/env python3
"""
manager_shm_patch.py  (ported from xiansfyt/cpv9-corolla)

启动时清理 /dev/shm 中残留的 cereal / openpilot 共享内存段与 POSIX 信号量。

背景问题
--------
在设备上修改任意 Python 文件后触发热重启时，manager 会重新拉起
controlsd / modeld / plannerd，但 /dev/shm 里可能仍残留上一代进程的
cereal msgq 段（以及 sem.cereal.* 信号量）。新进程 attach 到陈旧段后，
第一帧读到的数据是旧的（例如陈旧的 modelV2 / carState），
MPC 首次求解异常，最终表现为：

    Lateral Fault / Steering Fault  （"转向故障"）

这是一个全局竞态，跟具体改了哪个文件无关。启动时先清干净即可消除。

安全性
------
- 纯增量：只删除 /dev/shm 下的 cereal*/openpilot*/sem.cereal* 文件。
- 单个文件删除失败（权限 / 仍被占用）已 try/except 忽略，不影响主流程。
- 任何异常都不向外抛出，保证 manager 启动不受影响。
- 不依赖 openpilot 任何内部包，可独立 import。
"""

import glob
import os
import time

# 与 upstream cereal/messaging 一致的共享内存路径：
#   SHM_DIR = "/dev/shm"
#   segment_path = "/dev/shm/cereal.<name>"
# 这里按前缀匹配，不依赖 Paths.shm_path() 的具体返回值。
_SHM_DIR = "/dev/shm"

# 需要清理的共享内存段名前缀（cereal msgq 段 / openpilot 自定义段）
_SEGMENT_PREFIXES = ("cereal", "openpilot")

# POSIX 命名信号量（sem_open 创建的会在 /dev/shm 以 sem.<name> 出现）
_SEM_PREFIX = "sem.cereal"


def _cleanup_shm() -> int:
    removed = 0
    if not os.path.isdir(_SHM_DIR):
        return removed

    for name in os.listdir(_SHM_DIR):
        if name.startswith(_SEGMENT_PREFIXES) or name.startswith(_SEM_PREFIX):
            try:
                os.remove(os.path.join(_SHM_DIR, name))
                removed += 1
            except Exception:
                # 仍被占用或无权限：跳过，不影响主流程
                pass

    # glob 兜底，清理 sem.cereal* 形式（兼容不同挂载/命名）
    for seg in glob.glob(os.path.join(_SHM_DIR, "sem.cereal*")):
        try:
            os.remove(seg)
            removed += 1
        except Exception:
            pass

    return removed


def cleanup_shared_memory_on_start() -> None:
    """manager_init() 启动时调用：清残留 shm 段并让 shm 子系统 settle。"""
    try:
        removed = _cleanup_shm()
        if removed:
            print(f"[manager] cleaned {removed} residual cereal/openpilot shm segment(s)")
        # 给 shm 子系统一个 settle 时间，降低边缘竞态概率
        time.sleep(0.05)
    except Exception:
        # 任何异常都不向外抛出，保证 manager 启动不受影响
        pass


if __name__ == "__main__":
    # 手动验证入口：python3 manager_shm_patch.py
    cleanup_shared_memory_on_start()
    print("[manager_shm_patch] done (dry-run safe)")
