/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <debug.h>
#include <plat_config.h>
#include <mmio.h>
#include <sys/errno.h>
#include "sunxi_def.h"
#include "sunxi_private.h"

#define BIT(n) (1U << (n))

#define RATE_OSC24M	(24 * 1000 * 1000)
#define RATE_PERIPH0X2	(1200 * 1000 * 1000)

struct scpi_clock {
	uint32_t min_freq;
	uint32_t max_freq;
	uint32_t (*getter)(uint32_t);
	uint32_t (*setter)(uint32_t, uint32_t);
	uint32_t reg_addr;
	const char *name;
};

static uint32_t get_div_clk_rate(uint32_t reg_addr)
{
	uint32_t clkreg = mmio_read_32(reg_addr);
	uint32_t freq;

	if (!(clkreg & BIT(31)))
		return 0;

	switch ((clkreg >> 24) & 0x3) {
	case 0: freq = RATE_OSC24M; break;
	case 1: freq = RATE_PERIPH0X2; break;
	case 2: freq = RATE_PERIPH0X2; break;
	case 3: return 0;
	}

	freq >>= (clkreg >> 16) & 0x3;
	freq /= (clkreg & 0xf) + 1;

	return freq;
}

static uint32_t set_div_clk(uint32_t base_rate, int div)
{
	uint32_t reg = BIT(31);
	int p, m;

	switch (base_rate) {
	case RATE_OSC24M: reg |= 0 << 24; break;
	case RATE_PERIPH0X2: reg |= 1 << 24; break;
	default: return 0;
	}

	m = div;
	for (p = 0; p < 4; p++) {
		if (m <= 16)
			break;
		m >>= 1;
	}

	reg |= (p & 0x03) << 16;
	reg |= (m - 1) & 0x0f;

	return reg;
}

static uint32_t find_matching_freq(uint32_t freq, uint32_t base_freq)
{
	int div, p;

	if (freq == 0)
		return 0;

	if (freq >= base_freq)
		return base_freq;

	div = (base_freq + freq - 1) / freq;
	for (p = 0; p < 4; p++) {
		if (div <= 16)
			break;
		div >>= 1;
	}
	if (div > 16)
		div = 16;

	return (base_freq >> p) / div;
}

static uint32_t set_best_divider(uintptr_t reg_addr,
				 uint32_t freq, uint32_t base_freq)
{
	uint32_t new_freq = find_matching_freq(freq, base_freq);
	uint32_t div = base_freq / new_freq;

	mmio_write_32(reg_addr, set_div_clk(base_freq, div));
	return new_freq;
}

static uint32_t set_div_clk_rate(uint32_t reg_addr, uint32_t freq)
{
	uint32_t new_freq;

	if (freq == 0) {
		uint32_t reg = mmio_read_32(reg_addr);

		mmio_write_32(reg_addr, reg & ~BIT(31));
		return 0;
	}

	if (freq >= RATE_PERIPH0X2) {
		mmio_write_32(reg_addr, set_div_clk(RATE_PERIPH0X2, 1));
		return RATE_PERIPH0X2;
	}

	if (freq > RATE_OSC24M)
		return set_best_divider(reg_addr, freq, RATE_PERIPH0X2);

	if (freq < RATE_PERIPH0X2 / 8 / 16)
		return set_best_divider(reg_addr, freq, RATE_OSC24M);

	new_freq = find_matching_freq(freq, RATE_OSC24M);

	if (new_freq < find_matching_freq(freq, RATE_PERIPH0X2))
		return set_best_divider(reg_addr, freq, RATE_PERIPH0X2);
	else
		return set_best_divider(reg_addr, freq, RATE_OSC24M);
}

#define MMC_CLK_DESC(nr)						\
	{.min_freq = RATE_OSC24M / 8 / 16, .max_freq= RATE_PERIPH0X2,	\
	 .getter = get_div_clk_rate, .setter = set_div_clk_rate,	\
	 .reg_addr = 0x1c20088 + (nr * 4),				\
	 .name = "mmc" __STRING(nr) "_clk", }

struct scpi_clock sunxi_clocks[] = {
	MMC_CLK_DESC(0),
	MMC_CLK_DESC(1),
	MMC_CLK_DESC(2),
};

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static struct scpi_clock *get_sunxi_clock(int clocknr)
{
	if (clocknr < 0 || clocknr >= ARRAY_SIZE(sunxi_clocks))
		return NULL;

	return &sunxi_clocks[clocknr];
}

uint32_t sunxi_clock_get_min_rate(int clocknr)
{
	struct scpi_clock *clk = get_sunxi_clock(clocknr);

	if (!clk)
		return ~0;

	return clk->min_freq;
}

uint32_t sunxi_clock_get_max_rate(int clocknr)
{
	struct scpi_clock *clk = get_sunxi_clock(clocknr);

	if (!clk)
		return ~0;

	return clk->max_freq;
}

const char* sunxi_clock_get_name(int clocknr)
{
	struct scpi_clock *clk = get_sunxi_clock(clocknr);

	if (!clk)
		return NULL;

	return clk->name;
}

uint32_t sunxi_clock_get_rate(int clocknr)
{
	struct scpi_clock *clk = get_sunxi_clock(clocknr);

	if (!clk)
		return ~0;

	return clk->getter(clk->reg_addr);
}

int sunxi_clock_set_rate(int clocknr, uint32_t freq)
{
	struct scpi_clock *clk = get_sunxi_clock(clocknr);

	if (!clk)
		return ~0;

	return clk->setter(clk->reg_addr, freq);
}

int sunxi_clock_nr_clocks(void)
{
	return ARRAY_SIZE(sunxi_clocks);
}
