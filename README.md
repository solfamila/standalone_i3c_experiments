# Standalone I3C Experiments

This repository keeps only the experiment-specific files for the passing
no-Bazel RT595 I3C board-to-board runs.

It builds against the checked-out `tts` source tree instead of copying the full
SDK payload into this repo.

## Layout

- `02_i3c_new/` and `03_i3c_new_interrupt/` contain only local wrappers.
- `common/` contains the shared master roundtrip helper and the local slave base
  implementation used by both experiments.
- `run_experiment.sh` compiles, flashes, runs, and validates each experiment.

## Prerequisites

- This repo is expected to live inside the `tts` workspace root by default.
- If you move it elsewhere, set `TTS_ROOT=/absolute/path/to/tts`.
- The Arm GNU toolchain and LinkServer defaults still come from
  `TTS_ROOT/.local/...`.

## Usage

```bash
cd standalone_i3c_experiments
./run_experiment.sh 02_i3c_new
./run_experiment.sh 03_i3c_new_interrupt
```

Useful overrides:

- `TTS_ROOT`
- `RT595_MASTER_PROBE`
- `RT595_SLAVE_PROBE`
- `LINKSERVER_BIN`
- `RT595_TOOLCHAIN_ROOT`
