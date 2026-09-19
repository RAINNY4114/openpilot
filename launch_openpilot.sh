#!/usr/bin/env bash

# ==========================
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
