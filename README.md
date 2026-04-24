# Standalone I3C Experiments


## Layout

- `master_polling/` and `master_interrupt/` contain the experiment-specific
  master source files directly at the top level of each experiment folder.
- `slave/` contains the shared standalone slave wrapper used by both
  experiments.
- `common/` contains the shared master roundtrip helper and the local slave base
  implementation used by both experiments.
- `sdk/` contains the vendored RT595 SDK payloads used by the GCC runner,
  including a shared `sdk/slave/` tree for the slave build.
- `linker/` contains the local `HelloRam` and `Debug` linker scripts.
- `.local/` is the local default location for the Arm GNU toolchain and
  LinkServer binaries used by the runner. It is ignored by git.
- `run_experiment.sh` compiles, flashes, runs, and validates each experiment.

## Prerequisites

- A Unix-like shell environment with the standard command-line tools used by
  `run_experiment.sh`, including `bash`, `find`, `grep`, `mktemp`, and `sleep`.
- Arm GNU Toolchain for bare-metal Cortex-M builds, providing at least
  `arm-none-eabi-gcc` and `arm-none-eabi-objcopy`.
- NXP LinkServer, used to flash the slave image and run the master and slave
  ELFs on the EVK-MIMXRT595 boards.
- Two powered EVK-MIMXRT595 boards with accessible debug probes: one for the
  master image and one for the slave image.
- By default the runner uses a local toolchain and LinkServer under `.local/`
  inside this repository. You can override those paths with `LINKSERVER_BIN`
  and `RT595_TOOLCHAIN_ROOT`.
- The repo already vendors the linker scripts and RT595 SDK payloads needed by
  the build, so no files outside this folder are required.

## Usage

```bash
cd standalone_i3c_experiments
./run_experiment.sh master_polling
./run_experiment.sh master_interrupt
```

Useful overrides:

- `RT595_MASTER_PROBE`
- `RT595_SLAVE_PROBE`
- `RT595_DEVICE`
- `LINKSERVER_BIN`
- `RT595_TOOLCHAIN_ROOT`
- `RT595_MASTER_LINK_SCRIPT`
- `RT595_SLAVE_LINK_SCRIPT`
