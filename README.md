# High speed SPI DMA bit banging with Orange Pi Zero LTS and MCP3204/3208

This code contains a PoC demonstration of a software implementation of SPI protocol for communication
with MCP3204/MCP3208 and similar ADC chips using direct access to the GPIO registers of Allwinner H2+
(as well as other CPUs, provided that correct register addresses are used: look them up in their
respective datasheets).

On my particular instance of OPi I was able to achieve only about 23k samples/s using the SPI functions
of the WiringOP library (there's too much overhead between the sampling cycles, so even setting the SPI clock
frequency to 2 MHz doesn't help) and about 46k samples/s controlling the pins directly using its digitalRead() /
digitalWrite() calls.

Direct register access via `mmap()`'ed `/dev/mem` turns out to be much more efficient, allowing
to achieve clock frequency of about 2.8 MHz (at 1.368 GHz CPU clock) and the sampling rate of about 140 ksps,
exceeding the MCP320x specifications, at which point data can still be read, but the values read at this
out-of-spec sampling rate are obviously off.

This leaves plenty of room to decrease the OPi CPU clock frequency to save power, and the difference can
be quite significant. I was able to set it to 648 MHz, at which point the OPi plus the ADC circuit are drawing
about 1.5 W, at which point the sampling rate, with all the delays in the sampling loop removed, reaches
just over 100 ksps, and the readings look good and no different from what they look at a lower sampling rate.

At full 1.368 GHz CPU clock the measured power consumption was about 2.6 W, therefore switching to 648 MHz
means a 43% power draw reduction.

It will also work with the MCP3302/3304 13-bit ADC chip family, provided that the obvious necessary changes are added
to read the extra sign bit and, if fully differential mode is used, to employ the necessary arithmetics
to treat the result as a signed integer. See the respective datasheet.

Tested on Armbian/Linux. Not portable. Possibly not even compiler-portable, maybe not even across versions:
some gcc-specific features are used, mainly related to disabling optimizations to prevent it from optimizing out the direct
memory access instructions.

See the code for further details.
