"""Keep the precompiled worker off modeld's core.

modeld binds its main thread to a single core via config_realtime_process(7, 54),
and this worker inherits that affinity as a child process. Sharing one core makes
the two threads preempt each other thousands of times per second. The worker is
single-threaded and mostly waits on the GPU, so giving it its own big core is
enough.

Set WORKER_CORES to override (e.g. "6"), or to "" to keep the inherited affinity.
"""

import os
import sys

DEFAULT_WORKER_CORES = '4,5,6'


def set_worker_affinity(cores: str = DEFAULT_WORKER_CORES) -> None:
  if sys.platform != 'linux' or not cores.strip():
    return
  try:
    wanted = sorted({int(c) for c in cores.split(',') if c.strip()})
    # Do NOT use sched_getaffinity(0) as the machine's CPU list: this worker
    # inherits modeld's single-core mask, so intersecting against it would yield
    # nothing and silently keep the inherited core.
    ncpu = os.cpu_count() or len(os.sched_getaffinity(0))
    allowed = set(range(ncpu))
    usable = [c for c in wanted if c in allowed]
    if not usable:
      return
    os.sched_setaffinity(0, usable)
  except Exception:
    # Affinity is a tuning hint; never fail startup because of it.
    pass
