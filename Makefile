.DEFAULT_GOAL := help

RUNNER := ./run_experiment.sh
EXPERIMENTS := master_polling master_interrupt

.PHONY: help all clean $(EXPERIMENTS) clean-%

help:
	@printf '%s\n' \
	  'Targets:' \
	  '  make master_polling      Build, flash, run, and validate master_polling' \
	  '  make master_interrupt    Build, flash, run, and validate master_interrupt' \
	  '  make clean               Remove all experiment _build directories' \
	  '  make clean-master_polling    Remove only master_polling/_build' \
	  '  make clean-master_interrupt  Remove only master_interrupt/_build'

all: $(EXPERIMENTS)

$(EXPERIMENTS):
	$(RUNNER) $@

clean:
	$(RUNNER) clean

clean-%:
	$(RUNNER) clean $*