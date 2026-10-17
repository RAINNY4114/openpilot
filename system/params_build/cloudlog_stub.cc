// ============================================================================
// cloudlog_e 的最小替身 —— 只为把 openpilot/common/libparams_c.so 编出来。
//
// 背景：本机是「预编译分发树」，third_party 下没有 json11，因此无法编译
// openpilot/common/swaglog.cc（它 #include "json11/json11.hpp"）。
// 而 libparams_c.so（params_c.cc + params.cc + util.cc）会引用到 cloudlog_e。
//
// 官方 .so 里链接的是完整 swaglog（走 zmq 发到 loggerd）。这里退化成写 stderr：
// cloudlog_e 只在本库的「异常/错误」路径上被调用，正常读写参数不会走到，
// 所以对功能没有影响 —— 只是这类错误日志不再出现在 swaglog 里。
//
// 想要完整日志：把 third_party/json11 放回仓库后重跑
// system/params_build/build_libparams_c.sh，脚本会自动改走 swaglog 分支。
// ============================================================================
#include <cstdarg>
#include <cstdio>

// 注意：必须是 C++ 链接（swaglog.h 里就是普通 C++ 声明），
// 符号名要与官方 .so 一致：_Z10cloudlog_eiPKciS0_S0_z
void cloudlog_e(int levelnum, const char* filename, int lineno, const char* func,
                const char* fmt, ...) {
  char msg[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  fprintf(stderr, "[params_c] %s:%d %s(): %s\n",
          filename ? filename : "?", lineno, func ? func : "?", msg);
  fflush(stderr);
}
