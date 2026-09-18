# Interface tests

Run `make` for RTL or use the official GF workflow for gate-level tests.
The tests use external pins only, an independent 2048-sample area queue, and a peak candidate model.
Levels 0, 1, and 5 use real 80 MHz clocks and the default 80,000-clock handoff.
All output pins are checked from the first reset edge; X/Z fails.
The GF testbench connects VPWR/VGND. Functional simulation and physical timing are checked separately.

The v5 model captures a sample, updates statistics one clock later, and exposes them to the output on the next clock. Checks include a single-clock reset, pending-sample discard, zero/max magnitude and unchanged handoff timing.
