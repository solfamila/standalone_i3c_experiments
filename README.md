# Standalone I3C Experiments

This repository keeps only the experiment-specific files for the passing
no-Bazel RT595 I3C board-to-board runs.

The repository now vendors the required RT595 SDK payloads, linker scripts, and
local tool defaults so it does not rely on files outside this folder.

## Layout

- `02_i3c_new/` and `03_i3c_new_interrupt/` contain the local wrapper sources.
- `common/` contains the shared master roundtrip helper and the local slave base
  implementation used by both experiments.
- `sdk/` contains the vendored per-experiment RT595 SDK payloads used by the
  GCC runner.
- `linker/` contains the local `HelloRam` and `Debug` linker scripts.
- `.local/` is the local default location for the Arm GNU toolchain and
  LinkServer binaries used by the runner. It is ignored by git.
- `run_experiment.sh` compiles, flashes, runs, and validates each experiment.

## Prerequisites

- By default the runner uses tools from `.local/` inside this repository.
- You can override those paths with `LINKSERVER_BIN` and
  `RT595_TOOLCHAIN_ROOT`.

## Usage

```bash
cd standalone_i3c_experiments
./run_experiment.sh 02_i3c_new
./run_experiment.sh 03_i3c_new_interrupt
```

Useful overrides:

- `RT595_MASTER_PROBE`
- `RT595_SLAVE_PROBE`
- `LINKSERVER_BIN`
- `RT595_TOOLCHAIN_ROOT`
- `RT595_MASTER_LINK_SCRIPT`
- `RT595_SLAVE_LINK_SCRIPT`
