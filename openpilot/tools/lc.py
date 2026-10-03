#!/usr/bin/env python3
"""lc.py — 车道居中微调 / 横向间距避让 调参小工具

用法（在设备上）：
    python3 /data/openpilot/openpilot/tools/lc.py                 # 看当前配置
    python3 /data/openpilot/openpilot/tools/lc.py on              # 启用
    python3 /data/openpilot/openpilot/tools/lc.py off             # 关闭
    python3 /data/openpilot/openpilot/tools/lc.py trim 0.05       # 居中微调 +5cm(向右)
    python3 /data/openpilot/openpilot/tools/lc.py set barrier.bias_m 0.35
    python3 /data/openpilot/openpilot/tools/lc.py set vehicle.large_bias_m 0.55
    python3 /data/openpilot/openpilot/tools/lc.py log on          # 打开节流日志(2s一条)
    python3 /data/openpilot/openpilot/tools/lc.py status          # 读最近一次运行状态

配置文件 /data/lateral_clearance.json 会被 controlsd 热加载，改完约 1 秒生效。
"""
import json
import os
import sys

CONFIG = "/data/lateral_clearance.json"


def load():
    if not os.path.exists(CONFIG):
        print(f"配置文件不存在: {CONFIG}")
        sys.exit(1)
    with open(CONFIG) as f:
        return json.load(f)


def save(cfg):
    tmp = CONFIG + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    os.replace(tmp, CONFIG)
    print(f"已写入 {CONFIG}")


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("show", "get"):
        cfg = load()
        for k, v in cfg.items():
            if k.startswith("_"):
                continue
            print(f"  {k}: {v}")
        return

    cmd = args[0]
    cfg = load()

    if cmd == "on":
        cfg["enabled"] = True
        save(cfg)
    elif cmd == "off":
        cfg["enabled"] = False
        save(cfg)
    elif cmd == "log":
        cfg["log"] = (len(args) > 1 and args[1] in ("on", "1", "true"))
        save(cfg)
    elif cmd == "trim":
        cfg["trim_m"] = float(args[1])
        save(cfg)
        print(f"trim_m = {cfg['trim_m']:+.3f} m  (>0 向右挪)")
    elif cmd == "set":
        path = args[1].split(".")
        val = args[2]
        try:
            val = float(val)
        except ValueError:
            if val.lower() in ("true", "false"):
                val = (val.lower() == "true")
        node = cfg
        for p in path[:-1]:
            node = node.setdefault(p, {})
        node[path[-1]] = val
        save(cfg)
        print(f"{'.'.join(path)} = {val}")
    elif cmd == "status":
        # 状态由 controlsd 打印在日志里，这里只提示
        print("运行时状态看日志：")
        print("  journalctl -u comma -n 200 2>/dev/null | grep lateral_clearance")
        print("  或先把 log 打开: lc.py log on")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
