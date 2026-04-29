#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  run_experiment.sh <experiment-dir>
  run_experiment.sh clean [experiment-dir]

environment:
  RT595_MASTER_RUN_MODE=linkserver|trace32|none  default: linkserver
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 2
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_DIR="$SCRIPT_DIR"
SDK_ROOT="$REPO_DIR/sdk"

resolve_experiment_dir() {
  local experiment_arg=$1

  if [[ -d "$REPO_DIR/$experiment_arg" ]]; then
    cd "$REPO_DIR/$experiment_arg" && pwd
  elif [[ -d "$experiment_arg" ]]; then
    cd "$experiment_arg" && pwd
  else
    echo "unknown experiment directory: $experiment_arg" >&2
    exit 1
  fi
}

clean_build_outputs() {
  local target=${1:-all}
  local build_dir

  remove_build_dir() {
    local build_dir=$1

    if [[ -d "$build_dir" ]]; then
      echo "Removing $build_dir"
      rm -rf "$build_dir"
    fi
  }

  case "$target" in
    all)
      while IFS= read -r build_dir; do
        remove_build_dir "$build_dir"
      done < <(find "$REPO_DIR" -mindepth 2 -maxdepth 2 -type d -name _build | sort)
      ;;
    *)
      remove_build_dir "$(resolve_experiment_dir "$target")/_build"
      ;;
  esac
}

MODE=run
TARGET_ARG=$1
if [[ $1 == clean ]]; then
  MODE=clean
  TARGET_ARG=${2:-all}
fi

if [[ $MODE == clean ]]; then
  clean_build_outputs "$TARGET_ARG"
  exit 0
fi

EXPERIMENT_DIR=$(resolve_experiment_dir "$TARGET_ARG")

if [[ -d "$EXPERIMENT_DIR" ]]; then
  :
else
  echo "unknown experiment directory: $TARGET_ARG" >&2
  exit 1
fi

EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")
LOCAL_MASTER_DIR="$EXPERIMENT_DIR"
MASTER_SDK_DIR="$SDK_ROOT/master/evkmimxrt595_ezhb"
SLAVE_SDK_DIR="$SDK_ROOT/slave"

if [[ ! -d "$MASTER_SDK_DIR" || ! -d "$SLAVE_SDK_DIR" ]]; then
  echo "standalone SDK payload missing for $EXPERIMENT_NAME under $SDK_ROOT" >&2
  exit 1
fi

BUILD_DIR="$EXPERIMENT_DIR/_build"
MASTER_BUILD_DIR="$BUILD_DIR/master"
SLAVE_BUILD_DIR="$BUILD_DIR/slave"

default_linkserver_bin() {
  local repo_local=$REPO_DIR/.local/linkserver/extracted/flatten_LinkServer_25.12.83.pkg/Payload/dist/LinkServer

  if [[ -x "$repo_local" ]]; then
    printf '%s\n' "$repo_local"
    return 0
  fi

  command -v LinkServer 2>/dev/null || true
}

default_tool_binary() {
  local binary_name=$1
  local repo_root=${RT595_TOOLCHAIN_ROOT:-$REPO_DIR/.local/toolchains/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi}
  local repo_binary=$repo_root/bin/$binary_name

  if [[ -x "$repo_binary" ]]; then
    printf '%s\n' "$repo_binary"
    return 0
  fi

  command -v "$binary_name" 2>/dev/null || true
}

LINKSERVER=${LINKSERVER_BIN:-$(default_linkserver_bin)}
CC=${RT595_CC:-$(default_tool_binary arm-none-eabi-gcc)}
OBJCOPY=${RT595_OBJCOPY:-$(default_tool_binary arm-none-eabi-objcopy)}
GDBBIN=${RT595_GDB:-$(default_tool_binary arm-none-eabi-gdb)}
DEVICE=${RT595_DEVICE:-MIMXRT595S:EVK-MIMXRT595}
MASTER_PROBE=${RT595_MASTER_PROBE:-${RT595_PROBE:-PRASAQKQ}}
SLAVE_PROBE=${RT595_SLAVE_PROBE:-GRA1CQLQ}
EXIT_TIMEOUT=${RT595_EXIT_TIMEOUT:-20}
FLASH_STATE_DIR=${RT595_FLASH_STATE_DIR:-$REPO_DIR/.local/state}
SLAVE_RESET_PULSE_MS=${RT595_SLAVE_RESET_PULSE_MS:-50}
SLAVE_RESET_SETTLE=${RT595_SLAVE_RESET_SETTLE:-1}
MASTER_RESET_PULSE_MS=${RT595_MASTER_RESET_PULSE_MS:-50}
MASTER_RESET_SETTLE=${RT595_MASTER_RESET_SETTLE:-0.2}
MASTER_RESET_VIA_LINKSERVER=${RT595_MASTER_RESET_VIA_LINKSERVER:-0}
SLAVE_READY_TIMEOUT=${RT595_SLAVE_READY_TIMEOUT:-20}
SLAVE_READY_PATTERN=${RT595_SLAVE_READY_PATTERN:-'slave: waiting for master traff|slave: armed'}
SLAVE_ATTACH_GDB_PORT=${RT595_SLAVE_ATTACH_GDB_PORT:-3337}
SLAVE_LINKSERVER_MODE=${RT595_SLAVE_LINKSERVER_MODE:-serial:/dev/cu.usbmodem${SLAVE_PROBE}2:115200}
SLAVE_POST_MASTER_WAIT=${RT595_SLAVE_POST_MASTER_WAIT:-1}
MASTER_RUN_MODE=${RT595_MASTER_RUN_MODE:-linkserver}

if [[ -z "$CC" || -z "$OBJCOPY" || -z "$GDBBIN" ]]; then
  echo "Arm GNU toolchain not found. Set RT595_TOOLCHAIN_ROOT or RT595_CC/RT595_OBJCOPY/RT595_GDB." >&2
  exit 1
fi

if [[ "$MASTER_RUN_MODE" == linkserver && -z "$LINKSERVER" ]]; then
  echo "LinkServer not found. Set LINKSERVER_BIN or install LinkServer in PATH." >&2
  exit 1
fi

case "$MASTER_RUN_MODE" in
  linkserver|trace32|none)
    ;;
  *)
    echo "unsupported RT595_MASTER_RUN_MODE: $MASTER_RUN_MODE" >&2
    exit 2
    ;;
esac

CPU_FLAGS=(
  -mcpu=cortex-m33
  -mfpu=fpv5-sp-d16
  -mfloat-abi=hard
  -mthumb
)

COMMON_DEFINES=(
  CPU_MIMXRT595SFFOC
  CPU_MIMXRT595SFFOC_cm33
  MCUXPRESSO_SDK
  BOOT_HEADER_ENABLE=1
  FSL_SDK_DRIVER_QUICK_ACCESS_ENABLE=1
  MCUX_META_BUILD
  MIMXRT595S_cm33_SERIES
  CR_INTEGER_PRINTF
  PRINTF_FLOAT_ENABLE=0
  __MCUXPRESSO
  __USE_CMSIS
  DEBUG
)

COMMON_CFLAGS=(
  -std=gnu99
  -O0
  -g3
  -gdwarf-4
  -ffunction-sections
  -fdata-sections
  -fno-builtin
  -fno-common
  -fmerge-constants
)

run_with_eintr_retry() {
  local attempt
  local status
  local log_file
  local -a cmd=("$@")

  for attempt in 1 2 3; do
    log_file=$(mktemp)
    set +e
    "${cmd[@]}" >"$log_file" 2>&1
    status=$?
    set -e

    if [[ $status -eq 0 ]]; then
      cat "$log_file"
      rm -f "$log_file"
      return 0
    fi

    if grep -q 'Interrupted system call' "$log_file" && [[ $attempt -lt 3 ]]; then
      cat "$log_file" >&2
      echo "retrying after interrupted system call ($attempt/3): ${cmd[0]}" >&2
      rm -f "$log_file"
      continue
    fi

    cat "$log_file" >&2
    rm -f "$log_file"
    return "$status"
  done
}

ensure_flash_state_dir() {
  mkdir -p "$FLASH_STATE_DIR"
}

file_hash() {
  local file=$1

  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$file" | awk '{print $1}'
  elif command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$file" | awk '{print $1}'
  else
    cksum "$file" | awk '{print $1 ":" $2}'
  fi
}

slave_flash_stamp_file() {
  printf '%s/slave-flash-%s.sha256\n' "$FLASH_STATE_DIR" "$SLAVE_PROBE"
}

slave_flash_required() {
  local current_hash=$1
  local stamp_file=$2
  local previous_hash

  if [[ "${RT595_FORCE_SLAVE_FLASH:-0}" == 1 ]]; then
    return 0
  fi

  if [[ ! -f "$stamp_file" ]]; then
    return 0
  fi

  previous_hash=$(tr -d '[:space:]' < "$stamp_file")
  [[ "$previous_hash" != "$current_hash" ]]
}

record_slave_flash() {
  local current_hash=$1
  local stamp_file=$2

  ensure_flash_state_dir
  printf '%s\n' "$current_hash" > "$stamp_file"
}

reset_slave_board() {
  local log_file=$1

  : > "$log_file"
  printf 'slave reset-only start on %s\n' "$SLAVE_PROBE" >> "$log_file"
  run_with_eintr_retry "$LINKSERVER" probe "$SLAVE_PROBE" wiretimedreset "$SLAVE_RESET_PULSE_MS"
  sleep "$SLAVE_RESET_SETTLE"
}

reset_master_board_quiet() {
  if [[ "$MASTER_RESET_VIA_LINKSERVER" != 1 ]]; then
    echo "Skipping master quiet reset via LinkServer; use Lauterbach/TRACE32 on the master side"
    return 0
  fi

  echo "Resetting master to quiet bus on $MASTER_PROBE"
  run_with_eintr_retry "$LINKSERVER" probe "$MASTER_PROBE" wiretimedreset "$MASTER_RESET_PULSE_MS"
  sleep "$MASTER_RESET_SETTLE"
}

wait_for_slave_ready() {
  local log_file=$1
  local ready_pattern=$2
  local timeout_seconds=$3
  local elapsed=0

  while (( elapsed < timeout_seconds * 10 )); do
    if [[ -f "$log_file" ]] && grep -Eq "$ready_pattern" "$log_file"; then
      return 0
    fi

    if [[ -n "${SLAVE_RUN_PID:-}" ]] && ! kill -0 "$SLAVE_RUN_PID" 2>/dev/null; then
      break
    fi

    sleep 0.1
    ((elapsed += 1))
  done

  return 1
}

cleanup_slave_run() {
  if [[ -n "${SLAVE_RUN_PID:-}" ]]; then
    kill "$SLAVE_RUN_PID" 2>/dev/null || true
    wait "$SLAVE_RUN_PID" 2>/dev/null || true
    SLAVE_RUN_PID=
  fi
}

probe_redlink_pids() {
  local probe_serial=$1

  ps -ax -o pid=,command= | awk -v probe="$probe_serial" '
    /crt_emu_cm_redlink/ && index($0, probe) { print $1 }
  '
}

kill_probe_redlink_processes() {
  local probe_serial=$1
  local pid
  local -a pids=()

  while IFS= read -r pid; do
    if [[ -n "$pid" ]]; then
      pids+=("$pid")
    fi
  done < <(probe_redlink_pids "$probe_serial")

  if (( ${#pids[@]} == 0 )); then
    return 0
  fi

  kill "${pids[@]}" 2>/dev/null || true
  sleep 0.2

  while IFS= read -r pid; do
    if [[ -n "$pid" ]]; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done < <(probe_redlink_pids "$probe_serial")
}

slave_only_quiet_bus_check() {
  local gdb_log
  local gdbserver_log
  local gdbserver_pid
  local gdb_status

  gdb_log=$(mktemp)
  gdbserver_log=$(mktemp)

  "$LINKSERVER" gdbserver -p "$SLAVE_PROBE" --semihost-port -1 --gdb-port "$SLAVE_ATTACH_GDB_PORT" -a "$DEVICE" \
    >"$gdbserver_log" 2>&1 &
  gdbserver_pid=$!

  set +e
  "$GDBBIN" -q "$SLAVE_ELF" \
    -ex 'set pagination off' \
    -ex "target remote :$SLAVE_ATTACH_GDB_PORT" \
    -ex 'printf "pc=0x%08lx\\n", (unsigned long)$pc' \
    -ex 'printf "sp=0x%08lx\\n", (unsigned long)$sp' \
    -ex 'printf "lr=0x%08lx\\n", (unsigned long)$lr' \
    -ex 'detach' \
    -ex 'quit' >"$gdb_log" 2>&1
  gdb_status=$?
  set -e

  wait "$gdbserver_pid" >/dev/null 2>&1 || true

  if [[ $gdb_status -ne 0 ]]; then
    cat "$gdb_log" >&2
    printf '\n--- slave gdbserver log ---\n' >&2
    cat "$gdbserver_log" >&2
    rm -f "$gdb_log" "$gdbserver_log"
    return 1
  fi

  if grep -q '^pc=0x00000000$' "$gdb_log" && \
     grep -q '^sp=0x00000000$' "$gdb_log" && \
     grep -q '^lr=0x00000000$' "$gdb_log"; then
    cat "$gdb_log" >&2
    printf '\n--- slave gdbserver log ---\n' >&2
    cat "$gdbserver_log" >&2
    rm -f "$gdb_log" "$gdbserver_log"
    return 1
  fi

  rm -f "$gdb_log" "$gdbserver_log"
  return 0
}

start_slave_run() {
  local log_file=$1
  local -a run_cmd=("$LINKSERVER" run -p "$SLAVE_PROBE" --exit-timeout "$EXIT_TIMEOUT")

  : >"$log_file"
  kill_probe_redlink_processes "$SLAVE_PROBE"

  if [[ "$SLAVE_LINKSERVER_MODE" != "semihost" ]]; then
    run_cmd+=(--mode "$SLAVE_LINKSERVER_MODE")
  fi
  run_cmd+=("$DEVICE" "$SLAVE_ELF")

  nohup "${run_cmd[@]}" >"$log_file" 2>&1 < /dev/null &
  SLAVE_RUN_PID=$!

  if wait_for_slave_ready "$log_file" "$SLAVE_READY_PATTERN" "$SLAVE_READY_TIMEOUT"; then
    return 0
  fi

  if slave_only_quiet_bus_check; then
    echo "slave ready pattern not seen; slave-only quiet-bus check passed; continuing with background slave run" >&2
    return 0
  fi

  echo "slave did not become ready before timeout or quiet-bus check" >&2
  cat "$log_file" >&2
  cleanup_slave_run
  return 1
}

compile_master() {
  local link_script=$1
  local link_dir
  local source_root="$MASTER_SDK_DIR/source"
  local master_app_source="$LOCAL_MASTER_DIR/ezh_test_standalone.c"
  local master_features_source="$LOCAL_MASTER_DIR/ezh_features_standalone.c"
  local master_driver_source="$LOCAL_MASTER_DIR/fsl_i3c_smartdma.c"
  local shared_master_driver_include="$SCRIPT_DIR/../src/master/drivers"
  local src
  local rel
  local obj
  local -a includes
  local -a defines
  local -a extra_master_defines=()
  local -a cflags=("${COMMON_CFLAGS[@]}")
  local -a objects=()
  local -a sources=()

  rm -rf "$MASTER_BUILD_DIR"
  mkdir -p "$MASTER_BUILD_DIR"
  link_dir=$(dirname "$link_script")

  includes=(
    "$REPO_DIR"
    "$LOCAL_MASTER_DIR"
    "$MASTER_SDK_DIR"
    "$MASTER_SDK_DIR/source"
    "$MASTER_SDK_DIR/flash_config"
    "$MASTER_SDK_DIR/CMSIS"
    "$MASTER_SDK_DIR/CMSIS/m-profile"
    "$MASTER_SDK_DIR/device"
    "$MASTER_SDK_DIR/device/periph"
    "$MASTER_SDK_DIR/drivers"
    "$MASTER_SDK_DIR/drivers/rt500"
    "$MASTER_SDK_DIR/utilities"
    "$MASTER_SDK_DIR/utilities/str"
    "$MASTER_SDK_DIR/utilities/debug_console_lite"
    "$MASTER_SDK_DIR/component/uart"
    "$MASTER_SDK_DIR/board"
  )

  defines=("${COMMON_DEFINES[@]}" SDK_DEBUGCONSOLE=1)
  if [[ -n "${RT595_EXTRA_MASTER_DEFINES:-}" ]]; then
    read -r -a extra_master_defines <<<"${RT595_EXTRA_MASTER_DEFINES}"
    defines+=("${extra_master_defines[@]}")
  fi

  if [[ "$EXPERIMENT_NAME" == "master_i3c_dma_seed_tail_ibi_probe" || "$EXPERIMENT_NAME" == "master_i3c_sdma_seed_tail_len_sweep" || "$EXPERIMENT_NAME" == "master_interrupt" ]]; then
    master_driver_source="$SCRIPT_DIR/../src/master/drivers/fsl_i3c_smartdma.c"
    includes=(
      "$shared_master_driver_include"
      "${includes[@]}"
    )
    defines+=(EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX=1 EXPERIMENT_SLAVE_IBI_DATA=0xA5)
  fi

  sources+=(
    "$MASTER_SDK_DIR/board/board.c"
    "$MASTER_SDK_DIR/board/clock_config.c"
    "$MASTER_SDK_DIR/board/hardware_init.c"
    "$MASTER_SDK_DIR/board/pin_mux.c"
    "$MASTER_SDK_DIR/component/uart/fsl_adapter_usart.c"
    "$MASTER_SDK_DIR/device/system_MIMXRT595S_cm33.c"
    "$MASTER_SDK_DIR/flash_config/flash_config.c"
    "$master_driver_source"
    "$master_features_source"
    "$master_app_source"
    "$MASTER_SDK_DIR/source/semihost_hardfault.c"
    "$MASTER_SDK_DIR/startup/startup_mimxrt595s_cm33.c"
    "$MASTER_SDK_DIR/utilities/debug_console_lite/fsl_debug_console.c"
    "$MASTER_SDK_DIR/utilities/fsl_assert.c"
    "$MASTER_SDK_DIR/utilities/fsl_memcpy.S"
    "$MASTER_SDK_DIR/utilities/str/fsl_str.c"
  )

  while IFS= read -r src; do
    sources+=("$src")
  done < <(find "$MASTER_SDK_DIR/drivers" -maxdepth 1 -type f -name '*.c' ! -name 'fsl_i3c_smartdma.c' | sort)

  for src in "${sources[@]}"; do
    if [[ ! -f "$src" ]]; then
      echo "missing master source: $src" >&2
      exit 1
    fi

    if [[ $src == "$MASTER_SDK_DIR"/* ]]; then
      rel="sdk/${src#$MASTER_SDK_DIR/}"
    elif [[ $src == "$LOCAL_MASTER_DIR"/* ]]; then
      rel="local/${src#$LOCAL_MASTER_DIR/}"
    else
      rel=${src#$REPO_DIR/}
    fi

    obj="$MASTER_BUILD_DIR/${rel%.*}.o"
    mkdir -p "$(dirname "$obj")"

    if [[ $src == *.S ]]; then
      local -a cmd=("$CC" -c -x assembler-with-cpp "${CPU_FLAGS[@]}")
      for def in "${defines[@]}"; do
        cmd+=("-D$def")
      done
      for inc in "${includes[@]}"; do
        cmd+=("-I$inc")
      done
      cmd+=("-include" "$source_root/mcux_config.h" "-include" "$source_root/mcuxsdk_version.h" "-o" "$obj" "$src")
      run_with_eintr_retry "${cmd[@]}"
    else
      local -a cmd=("$CC" -c "${CPU_FLAGS[@]}" "${cflags[@]}")
      for def in "${defines[@]}"; do
        cmd+=("-D$def")
      done
      for inc in "${includes[@]}"; do
        cmd+=("-I$inc")
      done
      cmd+=("-include" "$source_root/mcux_config.h" "-include" "$source_root/mcuxsdk_version.h" "-o" "$obj" "$src")
      run_with_eintr_retry "${cmd[@]}"
    fi

    objects+=("$obj")
  done

  run_with_eintr_retry "$CC" \
    "${CPU_FLAGS[@]}" \
    -L"$link_dir" \
    -T "$link_script" \
    -Wl,-Map,"$MASTER_BUILD_DIR/evkmimxrt595_ezhb.map" \
    -Wl,--gc-sections \
    -Wl,--sort-section=alignment \
    -Wl,--cref \
    -Wl,--print-memory-usage \
    --specs=nano.specs \
    --specs=rdimon.specs \
    -o "$MASTER_BUILD_DIR/evkmimxrt595_ezhb.axf" \
    "${objects[@]}"

  run_with_eintr_retry "$OBJCOPY" -O binary \
    "$MASTER_BUILD_DIR/evkmimxrt595_ezhb.axf" \
    "$MASTER_BUILD_DIR/evkmimxrt595_ezhb.bin"
}

compile_slave() {
  local link_script=$1
  local link_dir
  local slave_app_source="$SLAVE_SDK_DIR/i3c_interrupt_b2b_transfer_slave_standalone.c"
  local src
  local rel
  local obj
  local -a includes
  local -a defines
  local -a cflags=("${COMMON_CFLAGS[@]}")
  local -a objects=()
  local -a sources=()

  rm -rf "$SLAVE_BUILD_DIR"
  mkdir -p "$SLAVE_BUILD_DIR"
  link_dir=$(dirname "$link_script")

  includes=(
    "$REPO_DIR"
    "$SLAVE_SDK_DIR"
    "$MASTER_SDK_DIR"
    "$MASTER_SDK_DIR/flash_config"
    "$MASTER_SDK_DIR/CMSIS"
    "$MASTER_SDK_DIR/CMSIS/m-profile"
    "$MASTER_SDK_DIR/device"
    "$MASTER_SDK_DIR/device/periph"
    "$MASTER_SDK_DIR/drivers"
    "$MASTER_SDK_DIR/drivers/rt500"
    "$MASTER_SDK_DIR/utilities"
    "$MASTER_SDK_DIR/utilities/str"
    "$MASTER_SDK_DIR/utilities/debug_console_lite"
    "$MASTER_SDK_DIR/component/uart"
  )

  defines=("${COMMON_DEFINES[@]}" SDK_DEBUGCONSOLE=1)

  if [[ -n "${RT595_EXTRA_SLAVE_DEFINES:-}" ]]; then
    read -r -a extra_slave_defines <<<"${RT595_EXTRA_SLAVE_DEFINES}"
    defines+=("${extra_slave_defines[@]}")
  fi

  if [[ "$EXPERIMENT_NAME" == "master_i3c_dma_seed_tail_ibi_probe" || "$EXPERIMENT_NAME" == "master_i3c_sdma_seed_tail_len_sweep" || "$EXPERIMENT_NAME" == "master_interrupt" ]]; then
    defines+=(EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX=1 EXPERIMENT_SLAVE_IBI_DATA=0xA5)
  fi

  if [[ "$EXPERIMENT_NAME" == "master_i3c_dma_seed_tail_ibi_probe" || "$EXPERIMENT_NAME" == "master_i3c_sdma_seed_tail_len_sweep" ]]; then
    defines+=(EXPERIMENT_SKIP_SLAVE_BOOT_LED_TEST=1)
  fi

  if [[ "$EXPERIMENT_NAME" == "master_interrupt" ]]; then
    defines+=(EXPERIMENT_SLAVE_MIN_ECHO_COUNT=8)
  fi

  sources+=(
    "$SLAVE_SDK_DIR/board.c"
    "$SLAVE_SDK_DIR/clock_config.c"
    "$SLAVE_SDK_DIR/hardware_init.c"
    "$slave_app_source"
    "$SLAVE_SDK_DIR/pin_mux.c"
    "$MASTER_SDK_DIR/component/uart/fsl_adapter_usart.c"
    "$MASTER_SDK_DIR/device/system_MIMXRT595S_cm33.c"
    "$MASTER_SDK_DIR/flash_config/flash_config.c"
    "$MASTER_SDK_DIR/startup/startup_mimxrt595s_cm33.c"
    "$MASTER_SDK_DIR/utilities/debug_console_lite/fsl_debug_console.c"
    "$MASTER_SDK_DIR/utilities/fsl_assert.c"
    "$MASTER_SDK_DIR/utilities/fsl_memcpy.S"
    "$MASTER_SDK_DIR/utilities/str/fsl_str.c"
  )

  while IFS= read -r src; do
    sources+=("$src")
  done < <(find "$MASTER_SDK_DIR/drivers" -maxdepth 1 -type f -name '*.c' | sort)

  for src in "${sources[@]}"; do
    if [[ ! -f "$src" ]]; then
      echo "missing slave build source: $src" >&2
      exit 1
    fi

    if [[ $src == "$MASTER_SDK_DIR"/* ]]; then
      rel="shared/${src#$MASTER_SDK_DIR/}"
    elif [[ $src == "$SLAVE_SDK_DIR"/* ]]; then
      rel="sdk/${src#$SLAVE_SDK_DIR/}"
    else
      rel=${src#$REPO_DIR/}
    fi

    obj="$SLAVE_BUILD_DIR/${rel%.*}.o"
    mkdir -p "$(dirname "$obj")"

    if [[ $src == *.S ]]; then
      local -a cmd=("$CC" -c -x assembler-with-cpp "${CPU_FLAGS[@]}")
      for def in "${defines[@]}"; do
        cmd+=("-D$def")
      done
      for inc in "${includes[@]}"; do
        cmd+=("-I$inc")
      done
      cmd+=("-include" "$SLAVE_SDK_DIR/mcux_config.h" "-include" "$SLAVE_SDK_DIR/mcuxsdk_version.h" "-o" "$obj" "$src")
      run_with_eintr_retry "${cmd[@]}"
    else
      local -a cmd=("$CC" -c "${CPU_FLAGS[@]}" "${cflags[@]}")
      for def in "${defines[@]}"; do
        cmd+=("-D$def")
      done
      for inc in "${includes[@]}"; do
        cmd+=("-I$inc")
      done
      cmd+=("-include" "$SLAVE_SDK_DIR/mcux_config.h" "-include" "$SLAVE_SDK_DIR/mcuxsdk_version.h" "-o" "$obj" "$src")
      run_with_eintr_retry "${cmd[@]}"
    fi

    objects+=("$obj")
  done

  run_with_eintr_retry "$CC" \
    "${CPU_FLAGS[@]}" \
    -L"$link_dir" \
    -T "$link_script" \
    -Wl,-Map,"$SLAVE_BUILD_DIR/slave.map" \
    -Wl,--gc-sections \
    -Wl,--sort-section=alignment \
    -Wl,--cref \
    -Wl,--print-memory-usage \
    --specs=nano.specs \
    --specs=nosys.specs \
    -o "$SLAVE_BUILD_DIR/slave.axf" \
    "${objects[@]}"

  run_with_eintr_retry "$OBJCOPY" -O binary \
    "$SLAVE_BUILD_DIR/slave.axf" \
    "$SLAVE_BUILD_DIR/slave.bin"
}

skip_master_linkserver_run() {
  echo "Skipping master LinkServer run because RT595_MASTER_RUN_MODE=$MASTER_RUN_MODE"
  echo "Master ELF ready at $MASTER_ELF"
}

validate_master_output() {
  local experiment_name=$1
  local output_file=$2

  case "$experiment_name" in
    master_polling|master_interrupt)
      grep -q 'I3C master transfer successful in I3C SDR mode' "$output_file"
      ;;
    master_dma_irq_probe)
      grep -q 'DMA0 IRQ to SmartDMA proof successful' "$output_file"
      ;;
    master_i3c_dma_irq_probe)
      grep -q 'I3C DMA request to DMA0 IRQ to SmartDMA proof successful' "$output_file"
      ;;
    master_i3c_dma_byte_bridge)
      grep -q 'I3C DMA byte bridge to SmartDMA proof successful' "$output_file"
      ;;
    master_i3c_dma_seed_chain_probe)
      grep -q 'I3C DMA seed chain probe successful' "$output_file"
      ;;
    master_i3c_dma_seed_tail_ibi_probe)
      grep -q 'I3C DMA seed tail IBI probe successful' "$output_file"
      ;;
    master_i3c_sdma_seed_tail_len_sweep)
      grep -q 'I3C SDMA seed tail length sweep successful' "$output_file"
      ;;
    master_led_smoke)
      grep -q 'LED smoke starting: raw GPIO forever' "$output_file" && grep -q 'readback:' "$output_file"
      ;;
    *)
      echo "unknown experiment: $experiment_name" >&2
      return 1
      ;;
  esac
}

if [[ -n "${RT595_MASTER_LINK_SCRIPT:-}" ]]; then
  MASTER_LINK_SCRIPT="$RT595_MASTER_LINK_SCRIPT"
else
  MASTER_LINK_SCRIPT="$MASTER_SDK_DIR/evkmimxrt595_ezhb_HelloRam.ld"
fi
SLAVE_LINK_SCRIPT="${RT595_SLAVE_LINK_SCRIPT:-$SLAVE_SDK_DIR/evkmimxrt595_ezhb_Debug.ld}"

echo "Building $EXPERIMENT_NAME master"
compile_master "$MASTER_LINK_SCRIPT"

MASTER_ELF="$MASTER_BUILD_DIR/evkmimxrt595_ezhb.axf"
MASTER_LOG="$BUILD_DIR/master_run.log"

if [[ "$EXPERIMENT_NAME" == "master_led_smoke" ]]; then
  if [[ "$MASTER_RUN_MODE" != "linkserver" ]]; then
    skip_master_linkserver_run
    exit 0
  fi

  echo "Running master LED smoke only on $MASTER_PROBE"
  set +e
  "$LINKSERVER" run -p "$MASTER_PROBE" --exit-timeout "$EXIT_TIMEOUT" "$DEVICE" "$MASTER_ELF" 2>&1 | tee "$MASTER_LOG"
  MASTER_STATUS=${PIPESTATUS[0]}
  set -e

  if ! validate_master_output "$EXPERIMENT_NAME" "$MASTER_LOG"; then
    echo "$EXPERIMENT_NAME FAILED validation" >&2
    exit 1
  fi

  echo "master_led_smoke raw GPIO probe finished or timed out with status $MASTER_STATUS"
  exit 0
fi

echo "Building $EXPERIMENT_NAME slave"
compile_slave "$SLAVE_LINK_SCRIPT"

SLAVE_ELF="$SLAVE_BUILD_DIR/slave.axf"
SLAVE_LOG="$BUILD_DIR/slave_run.log"
SLAVE_ELF_HASH=$(file_hash "$SLAVE_ELF")
SLAVE_FLASH_STAMP=$(slave_flash_stamp_file)

if slave_flash_required "$SLAVE_ELF_HASH" "$SLAVE_FLASH_STAMP"; then
  echo "Flashing slave on $SLAVE_PROBE"
  run_with_eintr_retry "$LINKSERVER" flash -p "$SLAVE_PROBE" "$DEVICE" load -e "$SLAVE_ELF"
  run_with_eintr_retry "$LINKSERVER" flash -p "$SLAVE_PROBE" "$DEVICE" verify "$SLAVE_ELF"
  record_slave_flash "$SLAVE_ELF_HASH" "$SLAVE_FLASH_STAMP"
else
  echo "Skipping slave flash on $SLAVE_PROBE; image unchanged"
fi

reset_master_board_quiet

USE_LIVE_SLAVE_RUN=0
if [[ ( "$MASTER_RUN_MODE" == "linkserver" && ( "$EXPERIMENT_NAME" == "master_i3c_dma_seed_tail_ibi_probe" || "$EXPERIMENT_NAME" == "master_i3c_sdma_seed_tail_len_sweep" ) ) || "${RT595_SLAVE_LIVE_RUN:-0}" == 1 ]]; then
  USE_LIVE_SLAVE_RUN=1
fi

if [[ $USE_LIVE_SLAVE_RUN -eq 1 ]]; then
  echo "Running slave on $SLAVE_PROBE and waiting for ready"
  start_slave_run "$SLAVE_LOG"
  if [[ "$MASTER_RUN_MODE" == "linkserver" ]]; then
    trap cleanup_slave_run EXIT
  fi
else
  echo "Resetting slave on $SLAVE_PROBE"
  reset_slave_board "$SLAVE_LOG"
fi

if [[ "$MASTER_RUN_MODE" != "linkserver" ]]; then
  skip_master_linkserver_run
  if [[ $USE_LIVE_SLAVE_RUN -eq 1 ]]; then
    echo "Slave live run started on $SLAVE_PROBE"
  else
    echo "Slave reset complete on $SLAVE_PROBE"
  fi
  exit 0
fi

echo "Running master on $MASTER_PROBE"
set +e
"$LINKSERVER" run -p "$MASTER_PROBE" --exit-timeout "$EXIT_TIMEOUT" "$DEVICE" "$MASTER_ELF" 2>&1 | tee "$MASTER_LOG"
MASTER_STATUS=${PIPESTATUS[0]}
set -e

if [[ $MASTER_STATUS -ne 0 ]]; then
  echo "master exit code: $MASTER_STATUS" >&2
  exit "$MASTER_STATUS"
fi

if ! validate_master_output "$EXPERIMENT_NAME" "$MASTER_LOG"; then
  echo "$EXPERIMENT_NAME FAILED validation" >&2
  exit 1
fi

echo "$EXPERIMENT_NAME PASS"
echo "master exit code: $MASTER_STATUS"
