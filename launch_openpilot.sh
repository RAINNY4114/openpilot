#!/usr/bin/env bash

# ==========================
# [NAV_HOST_20261102] 这两行曾经被误写成 `Export ATHENA HOST=...` / `Export APl HOST=...`，
# 是 bash 语法错误（Export 大小写、下划线被写成空格、APl 应为 API），
# 导致 ATHENA_HOST/API_HOST 实际未被设置，athena 退回默认 wss://athena.comma.ai。
# 现已恢复为 konik 原值。
#
# 导航路线源不在这里配置：navd 有独立的 NAV_HOST，见
# openpilot/selfdrive/navigation/navd.py 的 [NAV_HOST_20261102] 段，
# 当前固定为 http://192.168.0.106:8899，可用 /data/ford_params/nav_host 覆盖。
export ATHENA_HOST='wss://athena.konik.ai/'
export API_HOST='https://api.konik.ai'
export MAPBOX_TOKEN='pk.eyJ1IjoibXJvbmVjYyIsImEiOiJjbHhqbzlkbTYxNXUwMmtzZjdoMGtrZnVvIn0.SC7GNLtMFUGDgC2bAZcKzg'
export SKIP_FW_QUERY=1
export NAVD=true
export NAVMODEL=true

# ==========================

export FINGERPRINT="FORD_EXPLORER_MK6"
export SKIP_FW_QUERY="1"

exec ./launch_chffrplus.sh
