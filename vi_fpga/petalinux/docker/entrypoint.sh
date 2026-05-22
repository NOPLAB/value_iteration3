#!/bin/bash
# EDF container entrypoint
# Sources the Yocto build environment if it exists, then exec's the command.
set -e

SAVED_ARGS=("$@")
set --

# Source EDF build env if setup has been run
if [ -f /work/edf/build/conf/local.conf ]; then
    cd /work/edf
    source edf-init-build-env build 2>/dev/null || true
fi

cd /work
exec "${SAVED_ARGS[@]}"
