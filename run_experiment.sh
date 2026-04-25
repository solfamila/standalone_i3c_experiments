#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  run_experiment.sh <experiment-dir>
  run_experiment.sh clean [experiment-dir]
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

LINKSERVER=${LINKSERVER_BIN:-$REPO_DIR/.local/linkserver/extracted/flatten_LinkServer_25.12.83.pkg/Payload/dist/LinkServer}
TOOLCHAIN_ROOT=${RT595_TOOLCHAIN_ROOT:-$REPO_DIR/.local/toolchains/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi}
CC=${RT595_CC:-$TOOLCHAIN_ROOT/bin/arm-none-eabi-gcc}
OBJCOPY=${RT595_OBJCOPY:-$TOOLCHAIN_ROOT/bin/arm-none-eabi-objcopy}
DEVICE=${RT595_DEVICE:-MIMXRT595S:EVK-MIMXRT595}
MASTER_PROBE=${RT595_MASTER_PROBE:-${RT595_PROBE:-PRASAQKQ}}
SLAVE_PROBE=${RT595_SLAVE_PROBE:-GRA1CQLQ}
EXIT_TIMEOUT=${RT595_EXIT_TIMEOUT:-20}
SLAVE_READY_TIMEOUT=${RT595_SLAVE_READY_TIMEOUT:-20}
SLAVE_READY_PATTERN=${RT595_SLAVE_READY_PATTERN:-'slave: waiting for master traff|slave: armed'}
SLAVE_POST_MASTER_WAIT=${RT595_SLAVE_POST_MASTER_WAIT:-1}

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

start_slave_run() {
  local log_file=$1

  : >"$log_file"
  "$LINKSERVER" run -p "$SLAVE_PROBE" --exit-timeout "$EXIT_TIMEOUT" "$DEVICE" "$SLAVE_ELF" >"$log_file" 2>&1 &
  SLAVE_RUN_PID=$!

  if wait_for_slave_ready "$log_file" "$SLAVE_READY_PATTERN" "$SLAVE_READY_TIMEOUT"; then
    return 0
  fi

  echo "slave did not become ready before timeout" >&2
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
  local src
  local rel
  local obj
  local -a includes
  local -a defines
  local -a cflags=("${COMMON_CFLAGS[@]}")
  local -a objects=()
  local -a sources=()

  rm -rf "$MASTER_BUILD_DIR"
  mkdir -p "$MASTER_BUILD_DIR"
  link_dir=$(dirname "$link_script")

  includes=(
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

echo "Building $EXPERIMENT_NAME slave"
compile_slave "$SLAVE_LINK_SCRIPT"

MASTER_ELF="$MASTER_BUILD_DIR/evkmimxrt595_ezhb.axf"
SLAVE_ELF="$SLAVE_BUILD_DIR/slave.axf"
MASTER_LOG="$BUILD_DIR/master_run.log"
SLAVE_LOG="$BUILD_DIR/slave_run.log"

echo "Flashing slave on $SLAVE_PROBE"
run_with_eintr_retry "$LINKSERVER" flash -p "$SLAVE_PROBE" "$DEVICE" load -e "$SLAVE_ELF"
run_with_eintr_retry "$LINKSERVER" flash -p "$SLAVE_PROBE" "$DEVICE" verify "$SLAVE_ELF"

echo "Running slave on $SLAVE_PROBE"
start_slave_run "$SLAVE_LOG"

trap cleanup_slave_run EXIT

echo "Running master on $MASTER_PROBE"
set +e
"$LINKSERVER" run -p "$MASTER_PROBE" --exit-timeout "$EXIT_TIMEOUT" "$DEVICE" "$MASTER_ELF" 2>&1 | tee "$MASTER_LOG"
MASTER_STATUS=${PIPESTATUS[0]}
set -e

sleep "$SLAVE_POST_MASTER_WAIT"
cleanup_slave_run
trap - EXIT

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
