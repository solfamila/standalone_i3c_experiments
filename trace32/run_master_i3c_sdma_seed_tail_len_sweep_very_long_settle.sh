#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
TRACE32_WRAPPER=${TRACE32_WRAPPER:-$(command -v t32cmd_nostop || true)}

if [[ -z "$TRACE32_WRAPPER" ]]; then
  echo "t32cmd_nostop not found. Set TRACE32_WRAPPER to your TRACE32 command wrapper." >&2
  exit 1
fi

exec "$TRACE32_WRAPPER" timeout=660 wait=610000 \
  "DO $SCRIPT_DIR/run_master_i3c_sdma_seed_tail_len_sweep_very_long_settle.cmm"