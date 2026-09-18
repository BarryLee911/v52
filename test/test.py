# SPDX-License-Identifier: Apache-2.0
"""Pin-only checks shared by RTL and gate-level simulation."""
from collections import Counter, deque
import json
import math
from pathlib import Path
import time
import cocotb
from cocotb.triggers import Timer


class Reference:
    def __init__(self):
        self.level = None
        self.elapsed = self.samples = self.processed = 0
        self.pending = None
        self.history = deque(maxlen=2048)
        self.candidates = []
        self.previous = None
        self.area = self.peak = self.kind = self.data = 0
        self.ready = False
        self.cover = Counter()

    def edge(self, reset, level, adc):
        if reset:
            self.__init__()
            return
        # Output reads old backend state, then the backend processes a saved sample.
        self.kind = int(self.ready and not self.kind)
        self.data = self.peak if self.kind else self.area
        pending = self.pending
        self.pending = None
        if pending is not None:
            self.sample(pending)
        if self.level is None:
            if level <= 23:
                self.level = level
                self.cover['latch_without_sample'] += 1
            else:
                self.cover['illegal_level'] += 1
        else:
            self.elapsed += 1
            divisor = 78125000 if self.level == 23 else 2 ** self.level
            if self.elapsed % divisor == 0:
                self.pending = adc
                self.samples += 1
            if self.elapsed == 80000:
                self.ready = True
                self.cover['default_handoff'] += 1

    def sample(self, adc):
        index = self.processed
        self.history.append(int((adc >= 128) == (index % 2048 >= 1024)))
        total = sum(self.history)
        previous_area = self.area
        self.area = min(total, 2047)
        if total == 2048:
            self.cover['saturation'] += 1
        if previous_area == 2047 and self.area == 2046:
            self.cover['leave_saturation'] += 1
        # Full sample ages and a ranked list avoid RTL position/promotion logic.
        live = [item for item in self.candidates if index - item[1] < 1024]
        if len(live) < len(self.candidates):
            self.cover['expiry'] += 1
        if len(live) < 2 and self.previous is not None and self.previous not in live:
            live.append(self.previous)
            self.cover['buffer_refill'] += 1
        magnitude = abs(adc - 128)
        if any(value == magnitude for value, _ in live):
            self.cover['newer_tie'] += 1
        self.candidates = sorted(live + [(magnitude, index)], reverse=True)[:2]
        old_peak = self.peak
        self.peak = self.candidates[0][0]
        self.previous = (magnitude, index)
        if self.ready and not self.kind and self.peak != old_peak:
            self.cover['processed_peak_change'] += 1
        self.processed += 1
        if self.processed in (1, 1024, 2047, 2048, 2049):
            self.cover[f'sample_{self.processed}'] += 1


class Driver:
    def __init__(self, dut):
        self.dut = dut
        self.ref = Reference()
        self.cycles = 0
        self.half = Timer(6250, unit='ps')

    async def cycle(self, adc=127, level=0, ena=1, reset=False, drive=True):
        d = self.dut
        d.clk.value = 0
        d.rst_n.value = int(not reset)
        d.ena.value = ena
        d.ui_in.value = adc
        d.uio_in.value = level
        d.external_drive0.value = int(drive)
        await self.half
        d.clk.value = 1
        self.ref.edge(reset, level, adc)
        await self.half
        self.cycles += 1
        # Reject X/Z and check all output pins from the first reset edge.
        r = self.ref
        expected = (r.data & 255, ((r.data >> 8) << 5) | r.kind,
                    0xe1 if r.ready else 0xe0)
        actual = (int(d.uo_out.value), int(d.uio_out.value), int(d.uio_oe.value))
        assert actual == expected, (self.cycles, r.level, r.samples, actual, expected)
        if self.cycles == 1:
            assert reset, 'First edge must assert reset'
            d._log.info('First reset edge passed: data=0, type=0, output-enable=0xe0')
        assert not (drive and actual[2] & 1), 'External driver overlaps DUT uio[0]'
        if r.ready:
            assert int(d.pad0.value) == r.kind


def stimulus(index, after_handoff):
    if after_handoff < 0:
        # Match initially so the first sampling delay is visible at the pins.
        magnitude = 1 if index < 64 else abs(round(127 * math.sin(index * math.tau / 2048)))
        magnitude = max(1, magnitude)
        matched = True
    elif after_handoff < 2050:
        magnitude, matched = 37, True
    elif after_handoff < 4100:
        magnitude, matched = 37, False
    else:
        # Adjacent large candidates expire, refill, and follow lower amplitude.
        offset = after_handoff - 4100
        magnitude = 127 if offset == 0 else 126 if offset == 1 else 5 + offset % 7
        matched = bool(offset & 1)
    positive = (index % 2048 >= 1024) == matched
    return 128 + magnitude if positive else 128 - magnitude


@cocotb.test()
async def pin_interface_real_dividers(dut):
    started = time.perf_counter()
    driver = Driver(dut)
    results = []
    combined = Counter()
    for level in (0, 1, 5):
        ena_mode = lambda cycle: 1 if level == 0 else 0 if level == 1 else (cycle // 17) % 2
        for _ in range(4):
            await driver.cycle(reset=True, level=31, ena=ena_mode(driver.cycles), drive=False)
        for invalid in range(24, 32):
            for _ in range(3):
                await driver.cycle(level=invalid, ena=ena_mode(driver.cycles))
        await driver.cycle(level=level, ena=ena_mode(driver.cycles))
        assert driver.ref.samples == 0
        start_samples = None
        while start_samples is None or driver.ref.samples - start_samples < 6300:
            r = driver.ref
            if r.ready and start_samples is None:
                start_samples = r.samples
            offset = -1 if start_samples is None else r.samples - start_samples
            # Change level pins after latching; release bit 0 before handoff.
            drive = r.elapsed < 16
            other_level = (driver.cycles * 7) & 31
            await driver.cycle(stimulus(r.samples, offset), other_level,
                               ena_mode(driver.cycles), drive=drive)
        r = driver.ref
        for key in ('illegal_level', 'latch_without_sample', 'default_handoff',
                    'saturation', 'leave_saturation', 'expiry', 'buffer_refill',
                    'newer_tie', 'sample_1', 'sample_1024', 'sample_2047',
                    'sample_2048', 'sample_2049'):
            assert r.cover[key], (level, 'missing coverage', key)
        assert r.samples >= 2200 and r.ready
        combined.update(r.cover)
        results.append({'level': level, 'mode': 'real clock division',
                        'samples': r.samples, 'coverage': dict(r.cover)})
        dut._log.info('Level %d passed: %d real samples', level, r.samples)
    assert combined['processed_peak_change']
    # Reset an active design and configure again with ena low.
    for _ in range(1):
        await driver.cycle(reset=True, level=31, ena=0, drive=False)
    for _ in range(20):
        await driver.cycle(level=0, ena=0)
    assert driver.ref.samples == 19
    assert driver.ref.processed == 18
    # Flush a pending sample, immediately configure, and exercise zero/max inputs.
    await driver.cycle(reset=True, level=31, ena=0, drive=False)
    await driver.cycle(adc=128, level=0, ena=0)
    assert driver.ref.samples == 0
    for _ in range(24):
        await driver.cycle(adc=128, level=31, ena=0, drive=False)
    assert driver.ref.peak == 0
    await driver.cycle(adc=0, level=31, ena=0, drive=False)
    assert driver.ref.peak == 0
    await driver.cycle(adc=255, level=31, ena=0, drive=False)
    assert driver.ref.peak == 128
    await driver.cycle(adc=128, level=31, ena=0, drive=False)
    output = Path('output')
    output.mkdir(exist_ok=True)
    (output / 'pin_checks.json').write_text(json.dumps({
        'status': 'PASS', 'clock_hz': 80000000, 'clock_period_ns': 12.5,
        'handoff_cycles': 80000, 'cycles': driver.cycles, 'levels': results,
        'first_reset_edge': 'PASS; all output pins checked, no X/Z exception',
        'seconds': time.perf_counter() - started,
        'scope': 'v5: capture at N, process at N+1, output available N+2; unchanged sample phase.',
        'single_cycle_reset': 'PASS', 'zero_max_pipeline': 'PASS'
    }, indent=2), encoding='utf-8')
