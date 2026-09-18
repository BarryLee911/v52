## How it works

At 80 MHz, the design counts sign matches over the last 2048 samples and estimates the peak absolute distance from ADC midpoint 128 over 1024 samples. The peak uses two candidates and a previous-sample buffer; it is approximate. `ena` is retained for compatibility and ignored internally.

## How to test

Apply an 80 MHz clock and hold `rst_n=0` for four rising edges before reading results. Set `uio[4:0]` before releasing reset. The first legal level (0–23) is latched once; reset is required to reconfigure. Levels 24–31 wait for a legal value.

Levels 0–22 sample every `2^level` clocks, giving a reference frequency of `80,000,000 / (2048 * 2^level)` Hz. Level 23 samples every 78,125,000 clocks for a 0.5 mHz reference. Sampling begins one full divider interval after latching.

Drive ADC codes on `ui[7:0]`. The reference sign is low for 1024 samples, then high for 1024. Area is the count of equal signs, clipped to 2047. Peak is `abs(ADC-128)` in ADC counts, with a maximum code of 128.

Release external drive on `uio[0]` during the 80,000-clock (1 ms) wait after configuration. Its output enable then changes from input to output automatically; no handshake is needed. The first valid frame is area (`uio[0]=0`), followed by peak (`uio[0]=1`), alternating every clock. Read the 11-bit value as `{uio[7:5], uo[7:0]}` after each edge. At sampling edge N, magnitude and the sign-match bit are captured using the reference sign at that same instant. Statistics update at N+1; the output can use them at N+2. Since output types alternate, the corresponding frame can appear at N+2 or N+3. The reference phase and sampling schedule are unchanged. Before handoff, the data pins show area and `uio[0]` is an input.

`uio[1:4]` remain inputs after configuration; subsequent changes are ignored. `uio[7:5]` and `uo[7:0]` are always outputs. Reset clears statistics, restarts configuration, and releases `uio[0]`.

## External hardware

An 8-bit ADC-code source, an 80 MHz clock, and a controller that releases `uio[0]` before handoff and reads the multiplexed result.
