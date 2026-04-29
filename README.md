# Standalone I3C Experiments


## Layout

- `master_polling/` and `master_interrupt/` contain the experiment-specific
  master source files directly at the top level of each experiment folder,
  including the experiment-specific `fsl_i3c_smartdma.c` override.
- `sdk/` contains the vendored RT595 SDK payloads used by the GCC runner,
  including a shared `sdk/master/` tree for the master build and a shared
  `sdk/slave/` tree for the slave build. The live standalone slave wrapper and
  slave base implementation both now live under `sdk/slave/`.
- `sdk/master/evkmimxrt595_ezhb/evkmimxrt595_ezhb_HelloRam.ld` is the default
  master linker script, so the master is linked for RAM execution.
- `sdk/slave/evkmimxrt595_ezhb_Debug.ld` is the default slave linker script,
  so the slave is linked for flash, flashed, and run from flash.
- `.local/` can still hold local caches such as a private toolchain install,
  LinkServer unpacking, and flash-state stamps, but the passing sweep source,
  runner settings, and TRACE32 validation scripts are now versioned in this repo.
- `Makefile` is the primary entry point for the common workflows.
- `run_experiment.sh` is the backend used by the Makefile targets. It keeps the
  compile, flash, log-watch, timeout, and process-cleanup flow in one place.

## Prerequisites

- A macOS or other Unix-like host with the shell and basic command-line tools
  used by `run_experiment.sh`. The runner directly uses `bash`, `find`, `grep`,
  `make`, `mktemp`, and `sleep`. Only `bash` and `make` are version-pinned by
  this repo; the other utilities come from the host OS. The projects were validated using:
- GNU Make `3.81` from `/usr/bin/make` on the validation host.
- GNU bash `3.2.57(1)-release` on the validation host.
- Arm GNU Toolchain `15.2.Rel1 (Build arm-15.86)` under
  `.local/toolchains/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi`.
- `arm-none-eabi-gcc` `15.2.1 20251203`.
- `arm-none-eabi-objcopy` `2.45.1.20251203`.
- NXP LinkServer `v25.12.83 [Build 83] [2025-12-09 17:25:54]` under
  `.local/linkserver/extracted/flatten_LinkServer_25.12.83.pkg/Payload/dist/LinkServer`.
- Two powered EVK-MIMXRT595 boards with accessible debug probes: one for the
  master image and one for the slave image.
- The runner first honors `RT595_CC`, `RT595_OBJCOPY`, `RT595_GDB`,
  `RT595_TOOLCHAIN_ROOT`, and `LINKSERVER_BIN`. If those are unset, it falls
  back to the repo-local `.local/` install and then to the host `PATH`.
- The repo already vendors the linker scripts and RT595 SDK payloads needed by
  the build, so no files outside this folder are required.

## Passing Sweep Reproduction

The validated passing checkpoint for `master_i3c_sdma_seed_tail_len_sweep`
includes the experiment source, the shared-driver wiring in `run_experiment.sh`,
and the TRACE32 settle scripts under `trace32/`.

Validated build/run settings:

- `RT595_MASTER_RUN_MODE=none`
- `RT595_SLAVE_LIVE_RUN=1`
- Arm GNU Toolchain `15.2.Rel1`
- LinkServer `25.12.83`

Validated reproduction flow:

```bash
cd standalone_i3c_experiments
RT595_MASTER_RUN_MODE=none RT595_SLAVE_LIVE_RUN=1 ./run_experiment.sh master_i3c_sdma_seed_tail_len_sweep
TRACE32_WRAPPER=/path/to/t32cmd_nostop ./trace32/run_master_i3c_sdma_seed_tail_len_sweep_long_settle.sh
```

If `t32cmd_nostop` is already on your `PATH`, you can omit `TRACE32_WRAPPER`.

Passing settle signature:

- `longRun= pc=20281C10 logical=100 chunk=8 lenSweepStage=5 dmaStage=0B dmaResult=0 roundStage=5 roundResult=0 ibiCount=1 chunkValidateReason=0`
- `0x20281C10` resolves to `set_success_led`

## Experiment Differences

The exact live file differences between `master_polling` and
`master_interrupt` are:

- Local experiment files:
  - `master_polling/ezh_features_standalone.c`
  - `master_interrupt/ezh_features_standalone.c`
  - `master_polling/ezh_test_standalone.c`
  - `master_interrupt/ezh_test_standalone.c`
  - `master_polling/fsl_i3c_smartdma.c`
  - `master_interrupt/fsl_i3c_smartdma.c`

Behaviorally, the important difference is in the live
`ezh_features_standalone.c` files: `master_interrupt` contains the more
advanced SmartDMA TX/RX state machine with the extra `first_round` handling,
additional jump-table entries, and explicit RX pending loop handling. The live
`ezh_test_standalone.c` files currently use the same validated roundtrip app
logic and differ only in the banner comment at the top of the file.

The important difference in the local `fsl_i3c_smartdma.c` copies is not
whether SmartDMA is used: both variants still call `SMARTDMA_Boot()` for their
transfers. The difference is how the I3C peripheral side is synchronized with
that SmartDMA program. `master_polling` keeps the older I3C DMA-trigger path by
manipulating `MDMACTRL` and calling `I3C_MasterEnableDMA(...)`, while
`master_interrupt` switches that handoff to I3C ready interrupts. In the
interrupt variant, the driver enables `kI3C_MasterTxReadyFlag` and
`kI3C_MasterRxReadyFlag` before `SMARTDMA_Boot()`, then disables those same
interrupts in the SmartDMA callback when the last-byte handoff completes. The
old `I3C_MasterEnableDMA(...)` call is commented out there because the updated
interrupt experiment is synchronized through IRQ-ready events instead of the
earlier I3C DMA trigger bits. The interrupt-side driver also adds an explicit
NVIC priority assignment and comments out one defensive `assert(false)` path,
but the main behavioral change is the shift from I3C DMA-bit coordination to
IRQ-driven coordination around the same SmartDMA engine.

Cleanup note: the repo now keeps one shared `sdk/master/` tree plus the live
top-level `master_polling` and `master_interrupt` build inputs:
`ezh_features_standalone.c`, `ezh_test_standalone.c`, and the local
`fsl_i3c_smartdma.c` override for each experiment. The older archive-era
wrapper files, the unused vendored `source/ezh_features.c` and
`source/ezh_test.c` copies, and the duplicate per-experiment master SDK tree
were removed because `run_experiment.sh` never needed two full copies.

## Usage

```bash
cd standalone_i3c_experiments
make master_polling
make master_interrupt
make clean
make clean-master_polling
```

Direct script usage is still available when needed:

```bash
./run_experiment.sh master_polling
./run_experiment.sh clean master_polling
```

Useful overrides:

- `RT595_MASTER_PROBE`
- `RT595_SLAVE_PROBE`
- `RT595_DEVICE`
- `LINKSERVER_BIN`
- `RT595_TOOLCHAIN_ROOT`
- `RT595_MASTER_LINK_SCRIPT`
- `RT595_SLAVE_LINK_SCRIPT`
