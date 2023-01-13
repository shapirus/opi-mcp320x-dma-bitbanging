// PoC code that reads ADC samples from MCP3204/MCP3208 using software
// implementation of SPI protocol, setting individual pin
// logic levels using direct memory access.

// not portable, tested only under Armbian on Orange Pi Zero LTS.

// DMA bit banging method on the Orange Pi Zero LTS is at least 3.5 times
// as fast as bit banging using the wiringOP digitalRead() / digitalWrite()
// calls and 6.5 times as fast as using the wiringOP SPI interface, allowing
// to exploit full potential of the MCP3204/3208 ADC chip, getting over 100 ksps
// at clock frequency over 2 MHz, while leaving plenty of room for a significant
// downclocking of the CPU at the same time.


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>

// busy loop waiting has to be used, because no system call offers
// a sufficient (tens of nanoseconds) resolution, not even using
// the H2+ high speed timer registers, therefore we have no other
// choice than spend CPU cycles and fine-tune their number manually.

// this, however, can be replaced with no-op if the resulting SPI clock
// frequency does not exceed the ADC chip specifications and the readings
// stay correct;
// in my case this state is reached when the CPU clock frequency is set
// to 648 MHz: sampling rate reaches just over 100 ksps, and the readings
// still look good

#define BUSYWAIT_CYCLES		40
#define BUSYWAIT()		for (int z = 0; z < BUSYWAIT_CYCLES; z++) {}


// addresses defined below are described in the H2+ datasheet

// PIO base address is really 0x01C20800, but mmap() takes multiples of 4096,
// the page size, so we have to use base+offset
#define PIO_BASE	0x01C20000
#define PIO_OFFSET	0x800

// any desired set of pins can be used, but this example uses the designated
// SPI pins

// GPIO numbers, known as PAxx in the CPU datasheet
#define PIN_CS		13
#define PIN_CLK		14
#define PIN_MISO	16
#define PIN_MOSI	15

// this one is optional, we can use it to trigger a pulse
// at the beginning of each sampling cycle on PA12 aka physical pin 3
// for the oscilloscope to synchronize on
#define PIN_SYNC	12

// pin configuration bit offsets in their respective config registers
#define PIN_CS_BIT	20
#define PIN_CLK_BIT	24
#define PIN_MISO_BIT	0
#define PIN_MOSI_BIT	28
#define PIN_SYNC_BIT	16

// config register addresses
#define PIN_CS_CFG	0x04
#define PIN_CLK_CFG	0x04
#define PIN_MISO_CFG	0x08
#define PIN_MOSI_CFG	0x04
#define PIN_SYNC_CFG	0x04

// pin data register address
// all the pins we need are in the PA segment
#define PA_DATA		0x10


#define MMAP_WRITE(addr, offset, value)					\
	(*((uint32_t *)((unsigned char*)addr + offset)) = value)

#define MMAP_READ(addr, offset)						\
	(*((uint32_t *)((unsigned char*)addr + offset)))


#define PIN_HIGH(reg_value, pin)					\
	(reg_value | (1 << pin))

#define PIN_LOW(reg_value, pin)						\
	(reg_value & (~(1 << pin)))

#define PIN_CHANGE(reg_value, pin)					\
	(reg_value ^ (1 << pin))


#define PIN_VALUE(reg_value, pin)					\
	((reg_value & (1 << pin)) >> pin)


void *mmap_setup(off_t base_addr, uint64_t offset)
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		fprintf(stderr, "open()'ing /dev/mem failed: %s\n",
			strerror(errno));
		exit(1);
	}

	unsigned char *addr = mmap(NULL, 8192, PROT_READ | PROT_WRITE,
		MAP_SHARED, fd, base_addr);

	if (addr == MAP_FAILED) {
		fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
		exit(1);
	}

	return addr + offset;
}

// updates respective bits in the register value, refer to the H2+ datasheet
uint32_t pin_setup(uint32_t reg_value, int offset, uint32_t value)
{
	uint32_t mask_clr = ~(0x7 << offset); // 0b111
	reg_value &= mask_clr;
	uint32_t mask_set = value << offset;
	reg_value = reg_value | mask_set;
	return reg_value;
}


// set up input/output pins
// optimizations must be disabled, otherwise compilers
// can optimize our memory operations out
__attribute__((optimize("O0"))) void *spi_init()
{
	void *base = mmap_setup(PIO_BASE, PIO_OFFSET);
	uint32_t reg;

	reg = MMAP_READ(base, PIN_SYNC_CFG);
	reg = pin_setup(reg, PIN_SYNC_BIT, 1);
	MMAP_WRITE(base, PIN_SYNC_CFG, reg);

	reg = MMAP_READ(base, PIN_CS_CFG);
	reg = pin_setup(reg, PIN_CS_BIT, 1);
	MMAP_WRITE(base, PIN_CS_CFG, reg);

	reg = MMAP_READ(base, PIN_CLK_CFG);
	reg = pin_setup(reg, PIN_CLK_BIT, 1);
	MMAP_WRITE(base, PIN_CLK_CFG, reg);

	reg = MMAP_READ(base, PIN_MOSI_CFG);
	reg = pin_setup(reg, PIN_MOSI_BIT, 1);
	MMAP_WRITE(base, PIN_MOSI_CFG, reg);

	reg = MMAP_READ(base, PIN_MISO_CFG);
	reg = pin_setup(reg, PIN_MISO_BIT, 0);
	MMAP_WRITE(base, PIN_MISO_CFG, reg);

	reg = MMAP_READ(base, PA_DATA);
	reg = PIN_HIGH(reg, PIN_CS);	// CS must be high in initial state
	reg = PIN_HIGH(reg, PIN_CLK);	// initial clock is also high
	MMAP_WRITE(base, PA_DATA, reg);

	return base;
}


// see FIGURE 5-1 at http://ww1.microchip.com/downloads/en/devicedoc/21298e.pdf
__attribute__((optimize("O0"))) uint32_t getsample(register void *base)
{
	register uint32_t result = 0, reg;

//	session start: clk->low, cs->low, MOSI->high
	reg = MMAP_READ(base, PA_DATA);
	reg = PIN_LOW(reg, PIN_SYNC);	// for debugging with an oscilloscope

	reg = PIN_LOW(reg, PIN_CS);
	reg = PIN_LOW(reg, PIN_CLK);
	// start bit
	reg = PIN_HIGH(reg, PIN_MOSI);
	MMAP_WRITE(base, PA_DATA, reg);
	BUSYWAIT();

	reg = MMAP_READ(base, PA_DATA);
	reg = PIN_HIGH(reg, PIN_CLK);
	MMAP_WRITE(base, PA_DATA, reg);
	// start bit is received by the ADC at this point and the
	// sampling cycle begins, see Section 5.0
	BUSYWAIT();

	for (register int i = 0; i < 5; i++) {
	// send zero values for:
	// SGL/DIFF
	// D2
	// D1
	// D0
	// extra_cycle_after_D0
	// see TABLE 5-1/TABLE 5-2 and FIGURE 5-1
		reg = MMAP_READ(base, PA_DATA);
		reg = PIN_LOW(reg, PIN_CLK);
		reg = PIN_LOW(reg, PIN_MOSI);
		MMAP_WRITE(base, PA_DATA, reg);
		BUSYWAIT();

		reg = MMAP_READ(base, PA_DATA);
		reg = PIN_HIGH(reg, PIN_CLK);
		MMAP_WRITE(base, PA_DATA, reg);
		BUSYWAIT();
	}

	// read 13 bits, which includes the first null bit
	for (register int i = 13; i > 0; i--) {
		reg = MMAP_READ(base, PA_DATA);

		reg = PIN_LOW(reg, PIN_CLK);	// clock falling edge:
						// on its reception the ADC IC
						// sets next bit value at
		MMAP_WRITE(base, PA_DATA, reg);	// MISO (Dout)
		BUSYWAIT();

		reg = PIN_HIGH(reg, PIN_CLK);
		MMAP_WRITE(base, PA_DATA, reg);
		// delaying the following read until
		// after setting clock high allows to meet the
		// timing requirements of the ADC chip
		// and (apparently) gives the OPi time
		// to reflect the MISO pin value change
		// in the respective register;
		// if we try to read right after writing
		// the falling clock edge value, then readings
		// become garbage after reaching about 75-80k samples/sec

		// reading after setting clock high allows to get
		// reliable readings all the way up to 100 ksps and even beyond

		reg = MMAP_READ(base, PA_DATA);
		if (i != 13) {	// ignore the starting null bit
			result <<= 1;
			result += PIN_VALUE(reg, PIN_MISO);
		}

		BUSYWAIT();
	}

	// after reading B0 and setting CLK high, we must satisfy the t_{CSH} timing
	// which is 500ns minimum, meaning one full clock cycle at 2MHz, with CS set to high
	reg = MMAP_READ(base, PA_DATA);
	reg = PIN_HIGH(reg, PIN_SYNC);		// for debugging with an oscilloscope
	reg = PIN_HIGH(reg, PIN_CS);		// CS high
	reg = PIN_LOW(reg, PIN_CLK);		// CLK low
	MMAP_WRITE(base, PA_DATA, reg);
	BUSYWAIT();

	reg = MMAP_READ(base, PA_DATA);
	reg = PIN_HIGH(reg, PIN_CLK);		// CLK high, ready for the next session
	MMAP_WRITE(base, PA_DATA, reg);
	BUSYWAIT();

	return result;
}

__attribute__((optimize("O0"))) int main(void)
{
	void *base = spi_init();
	uint_fast64_t counter = 1, counter_prev = 0, ts_prev_nsec, ts_nsec, sample_sum = 0;
	uint_fast32_t sample, sample_min = ~0, sample_max = 0;
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	ts_prev_nsec = (uint_fast64_t)ts.tv_sec * 1000000000 + (uint_fast64_t)ts.tv_nsec;

	while (1) {
		sample = getsample(base);

		if (sample < sample_min)
			sample_min = sample;
		if (sample > sample_max)
			sample_max = sample;

		if (counter % 100000 == 0) {
			clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
			ts_nsec = (uint_fast64_t)ts.tv_sec * 1000000000 + (uint_fast64_t)ts.tv_nsec;

			uint_fast64_t sps = (counter - counter_prev) * 1000000000 / (ts_nsec - ts_prev_nsec);

			printf("counter: %llu, samples/sec: %llu\n", counter, sps);
			printf("	adc_min: %u, adc_max: %u, adc_avg: %llu\n", sample_min, sample_max, sample_sum / (counter - counter_prev));

			ts_prev_nsec = ts_nsec;
			counter_prev = counter;
			sample_min = ~0;
			sample_max = 0;
			sample_sum = 0;
		}
		sample_sum += sample;
		counter++;
	}
	return 0;
}
