/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/clock.h>
#include <debug.h>
#include <assert.h>
#include <sys/types.h>

#include <tinyara/board.h>
#include <arch/board/board.h>
#include <tinyara/i2c.h>
#include <arch/board/1100T_es8388_i2c.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ES8388_ADDR 0x11
#define CONFIG_ES8388_I2C_PORT 1
#define CONFIG_ES8388_I2C_FREQ 100000
#define CONFIG_ES8388_I2C_ADDRLEN 7
/****************************************************************************
 * Private Functions prototypes
 ****************************************************************************/
static int es8388_write(uint8_t addr, uint8_t data);
static int es8388_read(uint8_t addr, uint8_t *data);
static int es8388_modify(uint8_t addr, uint8_t clear, uint8_t set);

/************************************************************************************
 * Private Types
 ************************************************************************************/
typedef struct {
	uint8_t addr;
	uint8_t val;
	unsigned int delay;
} t_codec_init_script_entry;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct i2c_dev_s *i2c_dev;
static struct i2c_config_s configs;

static i2c_es i2c_es8388 = {
	.write = es8388_write,
	.read = es8388_read,
	.modify = es8388_modify,
};

/*This init script enables codec audio output */
static t_codec_init_script_entry codec_reset_script[] = {
	{ES8388_CONTROL1, SCPReset(1), 0},
	{ES8388_CONTROL1, SCPReset(0), 0},
	{ES8388_CONTROL1, SCPReset(0) | LRCM(1) | DACMCLK(1) | SameFs(1) | SeqEn(0) | EnRef(1) | VMIDSEL(0x2), 0},
	{ES8388_CONTROL2, LPVcmMod(0) | LPVrefBuf(0) | PdnAna(0) | PdnIbiasgen(0) | VrefrLo(0) | PdnVrefbuf(0), 0},
	{ES8388_MASTERMODE, MSC(1) | MCLKDIV2(1) | BCLK_INV(0) | BCLKDIV(0), 0},
	{ES8388_DACCONTROL21, slrck(1)|Lrck_sel(0)|offset_dis(0)|mclk_dis(0)|Adc_dll_pwd(0)|Dac_dll_pwd(0), 0},
	{ES8388_CHIPPOWER,  adc_DigPDN(0) | dac_DigPDN(0) | adc_stm_rst(0) | dac_stm_rst(0) |  ADCDLL_PDN(0) |  DACDLL_PDN(0) |  adcVref_PDN(0) |  dacVref_PDN(0), 0},
	{ES8388_ADCPOWER, PdnAINL(0) | PdnAINR(0) | PdnADCL(0) | PdnADCR(0) | PdnMICB(0) | PdnADCBiasgen(0) | flashLP(0) | int1LP(0), 0},
	{ES8388_DACPOWER, PdnDACL(0) | PdnDACR(0) | LOUT1(0) | ROUT1(0) | LOUT2(0) | ROUT2(0), 0},

};

/*This init script enables codec audio output */
/*
static t_codec_init_script_entry codec_init_script[] = {
	{0x00, 0x00, 0},
};
*/
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int es8388_write(uint8_t addr, uint8_t data)
{
	int32_t ret;
	uint8_t reg[2];

	reg[0] = addr;
	reg[1] = data;

	ret = i2c_write(i2c_dev, &configs, reg, 2);

	return ret;
}

static int es8388_read(uint8_t addr, uint8_t *data)
{
	int32_t ret;
	uint8_t reg[1];

	reg[0] = addr;

	ret = i2c_write(i2c_dev, &configs, reg, 1);
	ret = i2c_read(i2c_dev, &configs, reg, 1);

	*data = reg[0];

	return *data;
}

static int es8388_modify(uint8_t addr, uint8_t clear, uint8_t set)
{
	uint8_t data;

	es8388_read(addr, &data);
	data &= ~clear;
	data |= set;

	es8388_write(addr, data);
	es8388_read(addr, &data);
	return data;
}

static void delay(unsigned int mS)
{
	volatile systime_t start = clock_systimer();

	mS = mS / MSEC_PER_TICK + 1;

	while (1)
		if ((start + mS) < clock_systimer()) {
			return;
		}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_i2c_initialize
 *
 * Description:
 *   Initialize i2c interface to access es8388 codec.
 *   Returns methods to communicate with codec
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   pointer to i2c_es operations structure, or NULL in case of errors
 *
 ****************************************************************************/

i2c_es *es8388_i2c_initialize(void)
{
	i2c_es *i2c;
	int i;
	int ret;

	i2c_dev = up_i2cinitialize(CONFIG_ES8388_I2C_PORT);

	if (i2c_dev == NULL) {
		i2cerr("es8388 up_i2cinitialize(port: %d) failed\n", CONFIG_ES8388_I2C_PORT);
		return NULL;
	}

	configs.frequency = CONFIG_ES8388_I2C_FREQ;
	configs.address = ES8388_ADDR;
	configs.addrlen = CONFIG_ES8388_I2C_ADDRLEN;

        i2c = &i2c_es8388;
	for (i = 0; i < sizeof(codec_reset_script) / sizeof(t_codec_init_script_entry); i++) {
		ret = i2c->modify(codec_reset_script[i].addr, 0xff, codec_reset_script[i].val);
		audvdbg("es %x <- %x\n", codec_reset_script[i].addr, ret);
		delay(codec_reset_script[i].delay);
	}
	return i2c;
}
