/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
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

#ifndef __SUNXI_PRIVATE_H__
#define __SUNXI_PRIVATE_H__

#include <bl_common.h>
#include <platform_def.h>

/*******************************************************************************
 * Forward declarations
 ******************************************************************************/
struct meminfo;

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/
void sunxi_configure_mmu_el1(unsigned long total_base,
			   unsigned long total_size,
			   unsigned long,
			   unsigned long,
			   unsigned long,
			   unsigned long);
void sunxi_configure_mmu_el3(unsigned long total_base,
			   unsigned long total_size,
			   unsigned long,
			   unsigned long,
			   unsigned long,
			   unsigned long);
int sunxi_config_setup(void);

uint16_t sunxi_get_socid(void);

/* Declarations for sunxi_topology.c */
int plat_setup_topology(void);

/* Declarations for sunxi_io_storage.c */
void sunxi_io_setup(void);

/* Declarations for sunxi_security.c */
void sunxi_security_setup(void);

/* Declarations for sunxi_power.c */
int sunxi_pmic_setup(void);
int sunxi_pmic_read(uint8_t address);
int sunxi_pmic_write(uint8_t address, uint8_t value);
int sunxi_power_setup(uint16_t socid);
int sunxi_power_set_cpu_voltage(int millivolt);
unsigned int sunxi_pstate_get(uint16_t device);
int sunxi_pstate_set(uint16_t device, uint8_t state);

void udelay(unsigned int delay);
int sunxi_setup_clocks(uint16_t socid);

/* Declarations for sunxi_scpi.c */
uint32_t sunxi_trigger_scpi(uint32_t x1, uint32_t x2, uint32_t x3, uint32_t x4);

/* Declarations for sunxi_clocks.c */
int sunxi_clock_nr_clocks(void);
uint32_t sunxi_clock_get_min_rate(int clocknr);
uint32_t sunxi_clock_get_max_rate(int clocknr);
const char* sunxi_clock_get_name(int clocknr);
uint32_t sunxi_clock_get_rate(int clocknr);
int sunxi_clock_set_rate(int clocknr, uint32_t freq);

int sunxi_clock_set_cpu_clock(uint32_t freq_mhz, int enable);

/* Declarations for sunxi_rsb.c */
int sunxi_rsb_init(void);
int sunxi_rsb_read(uint8_t address);
int sunxi_rsb_write(uint8_t address, uint8_t value);
void sunxi_rsb_wait(const char *desc);
int sunxi_rsb_configure(uint16_t hw_addr, uint8_t rt_addr);

/* Declarations for sunxi_dvfs.c */
uint32_t sunxi_dvfs_get_get_opp_voltage(int oppnr);
uint32_t sunxi_dvfs_get_get_opp_frequency(int oppnr);
int sunxi_dvfs_set_index(int index);
int sunxi_dvfs_get_index(void);
int sunxi_dvfs_get_nr_opp(void);

/* Declarations for sunxi_sensors.c */
int sunxi_setup_sensors(void);
const char* sunxi_sensor_get_name(int sensornr);
uint32_t sunxi_sensor_get_value(int sensornr);
int sunxi_sensors_nr_sensors(void);

/* Declarations for sunxi_temp.c */
int sunxi_ths_setup(void);
int sunxi_ths_read_temp(int sensornr);

/* Gets the SPSR for BL33 entry */
uint32_t sunxi_get_spsr_for_bl33_entry(int aarch);


#endif /* __SUNXI_PRIVATE_H__ */
