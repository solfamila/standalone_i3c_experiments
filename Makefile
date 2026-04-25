.DEFAULT_GOAL := help

RUNNER := ./run_experiment.sh
EXPERIMENTS := master_polling master_interrupt master_dma_irq_probe master_i3c_dma_irq_probe

.PHONY: help all clean $(EXPERIMENTS) clean-%

help:
	@printf '%s\n' \
	  'Targets:' \
	  '  make master_polling      Build, flash, run, and validate master_polling' \
	  '  make master_interrupt    Build, flash, run, and validate master_interrupt' \
	  '  make master_dma_irq_probe Build, flash, run, and validate the DMA0 IRQ to SmartDMA probe' \
	  '  make master_i3c_dma_irq_probe Build, flash, run, and validate the I3C0->DMA0->SmartDMA probe' \
	  '  make clean               Remove all experiment _build directories' \
	  '  make clean-master_polling    Remove only master_polling/_build' \
	  '  make clean-master_interrupt  Remove only master_interrupt/_build' \
	  '  make clean-master_dma_irq_probe Remove only master_dma_irq_probe/_build' \
	  '  make clean-master_i3c_dma_irq_probe Remove only master_i3c_dma_irq_probe/_build'

all: $(EXPERIMENTS)

$(EXPERIMENTS):
	$(RUNNER) $@

clean:
	$(RUNNER) clean

clean-%:
	$(RUNNER) clean $*