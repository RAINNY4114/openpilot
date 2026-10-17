#!/usr/bin/env bash
# ============================================================================
# 重编 openpilot/common/libparams_c.so
#
# 为什么需要手工编：本机是「预编译分发树」，仓库里没有 SConstruct / SConscript，
# 跑不了 scons。但 openpilot/common/params.py 是用 ctypes 直接加载
# libparams_c.so 的，参数键表（params_keys.h）被编进这个 .so ——
# 所以任何对 params_keys.h 的增删都必须重编本文件，否则 Python 侧读新键会
# 抛 UnknownKeyName / RuntimeError。
#
# 用法：  bash /data/openpilot/system/params_build/build_libparams_c.sh
# 回滚：  备份在 /data/op_patch_backup_*.tar.gz 里（含改动前的 .so）
# ============================================================================
set -e
cd /data/openpilot

CAP=/usr/local/venv/lib/python3.12/site-packages/capnproto/install/include
SRC="openpilot/common/params_c.cc openpilot/common/params.cc openpilot/common/util.cc"

if [ -f openpilot/third_party/json11/json11.hpp ]; then
  echo "[build] third_party/json11 存在 -> 链接完整 swaglog（日志进 loggerd）"
  EXTRA="openpilot/common/swaglog.cc openpilot/third_party/json11/json11.cpp"
  LINK="-lzmq -lpthread"
else
  echo "[build] 无 third_party/json11 -> 使用 cloudlog_stub.cc（日志退化为 stderr）"
  EXTRA="system/params_build/cloudlog_stub.cc"
  LINK=""
fi

# shellcheck disable=SC2086
g++ -shared -fPIC -O2 -std=c++17 \
  -D__COMMA_HARDWARE__=1 \
  -I. -Iopenpilot -Iopenpilot/cereal/gen/cpp -Iopenpilot/third_party -I"$CAP" \
  $SRC $EXTRA -o openpilot/common/libparams_c.so.new $LINK

# 原子替换：正在跑的进程把旧 .so mmap 着，原地覆盖会 SIGBUS。
mv -f openpilot/common/libparams_c.so.new openpilot/common/libparams_c.so
chmod 755 openpilot/common/libparams_c.so

echo "[build] 未定义符号检查（应只剩 libc/libstdc++）："
nm -D -u openpilot/common/libparams_c.so \
  | grep -v "GLIBC\|CXXABI\|__cxa\|_ZSt\|_ZNSt\|__gxx\|__gmon\|_ITM\|_Unwind" || true

echo "[build] 自检：新键可读？"
/usr/local/venv/bin/python - <<'PY'
import sys
sys.path.insert(0, "/data/openpilot")
from openpilot.common.params import Params
p = Params()
for k in ("GasLongCancel", "GasLongCancelResumeMs", "ChestnutModelError"):
    print(f"  {k:24s} = {p.get(k, return_default=True)!r}")
PY

echo "[build] done: $(ls -la openpilot/common/libparams_c.so)"
