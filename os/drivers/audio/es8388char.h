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
 * drivers/audio/es8388char.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_AUDIO_ES8388CHAR_H
#define __DRIVERS_AUDIO_ES8388CHAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/compiler.h>

#include <pthread.h>
#include <mqueue.h>

#include <tinyara/wqueue.h>
#include <tinyara/fs/ioctl.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
//Reg. 00		
#define ES8388_CONTROL1         0x00
#define SCPReset(x)		((x & 1) << 7)
#define LRCM(x)			((x & 1) << 6)
#define DACMCLK(x)		((x & 1) << 5)
#define SameFs(x)		((x & 1) << 4)
#define SeqEn(x)		((x & 1) << 4)
#define EnRef(x)		((x & 1) << 2)
#define VMIDSEL(x)		((x & 3) << 0)
//Reg. 01
#define ES8388_CONTROL2         0x01
#define LPVcmMod(x)		((x & 1) << 5)
#define LPVrefBuf(x)		((x & 1) << 4)
#define PdnAna(x)		((x & 1) << 3)
#define PdnIbiasgen(x)		((x & 1) << 2)
#define VrefrLo(x)		((x & 1) << 1)
#define PdnVrefbuf(x)		((x & 1) << 0)
//Reg. 02
#define ES8388_CHIPPOWER        0x02
#define adc_DigPDN(x)		((x & 1) << 7)
#define dac_DigPDN(x)		((x & 1) << 6)
#define adc_stm_rst(x)		((x & 1) << 5)
#define dac_stm_rst(x)		((x & 1) << 4)
#define ADCDLL_PDN(x)		((x & 1) << 3)
#define DACDLL_PDN(x)		((x & 1) << 2)
#define adcVref_PDN(x)		((x & 1) << 1)
#define dacVref_PDN(x)		((x & 1) << 0)
//Reg. 03
#define ES8388_ADCPOWER         0x03
#define PdnAINL(x)		((x & 1) << 7)
#define PdnAINR(x)		((x & 1) << 6)
#define PdnADCL(x)		((x & 1) << 5)
#define PdnADCR(x)		((x & 1) << 4)
#define PdnMICB(x)		((x & 1) << 3)
#define PdnADCBiasgen(x)	((x & 1) << 2)
#define flashLP(x)		((x & 1) << 1)
#define int1LP(x)		((x & 1) << 0)
//Reg. 04
#define ES8388_DACPOWER         0x04
#define PdnDACL(x)		((x & 1) << 7)
#define PdnDACR(x)		((x & 1) << 6)
#define LOUT1(x)		((x & 1) << 5)
#define ROUT1(x)		((x & 1) << 4)
#define LOUT2(x)		((x & 1) << 3)
#define ROUT2(x)		((x & 1) << 2)
//Reg. 05
#define ES8388_CHIPLOPOW1       0x05
#define LPDACL(x)		((x & 1) << 7)
#define LPDACR(x)		((x & 1) << 6)
#define LPLOUT1(x)		((x & 1) << 5)
#define LPLOUT2(x)		((x & 1) << 3)
//Reg. 06
#define ES8388_CHIPLOPOW2       0x06
#define LPPGA(x)		((x & 1) << 7)
#define LPLMIX(x)		((x & 1) << 6)
#define LPADCvrp(x)		((x & 1) << 1)
#define LPDACvrp(x)		((x & 1) << 0)
//Reg. 07
#define ES8388_ANAVOLMANAG      0x07
#define VSEL(x)			((x & 0x7f) << 0)
//Reg. 08	
#define ES8388_MASTERMODE       0x08
#define MSC(x)			((x & 1) << 7)
#define MCLKDIV2(x)		((x & 1) << 6)
#define BCLK_INV(x)		((x & 1) << 5)
#define BCLKDIV(x)		((x & 0x1F) << 0)
//Reg. 09	
#define ES8388_ADCCONTROL1      0x09
#define MicAmpL(x)		((x & 0xF) << 4)
#define MicAmpR(x)		((x & 0xF) << 0)
//Reg. 10	
#define ES8388_ADCCONTROL2      0x0a
#define LINSEL(x)		((x & 3) << 6)
#define RINSEL(x)		((x & 3) << 4)
#define DSSEL(x)		((x & 1) << 3)
#define DSR(x)			((x & 1) << 2)
//Reg. 11	
#define ES8388_ADCCONTROL3      0x0b
#define DS(x)			((x & 1) << 7)
#define MONOMIX(x)		((x & 3) << 3)
#define TRI(x)			((x & 1) << 2)
//Reg. 12	
#define ES8388_ADCCONTROL4      0x0c
#define DATSEL(x)		((x & 3) << 6)
#define ADCLRP(x)		((x & 1) << 5)
#define ADCWL(x)		((x & 7) << 2)
#define ADCFORMAT(x)		((x & 3) << 0)
//Reg. 13	
#define ES8388_ADCCONTROL5      0x0d
#define ADCFsMode(x)		((x & 1) << 5)
#define ADCFsRatio(x)		((x & 0x1F) << 0)
//Reg. 14	
#define ES8388_ADCCONTROL6      0x0e
#define ADC_invL(x)		((x & 1) << 7)
#define ADC_invR(x)		((x & 1) << 6)
#define ADC_HPF_L(x)		((x & 1) << 5)
#define ADC_HPF_R(x)		((x & 1) << 4)
//Reg. 15	
#define ES8388_ADCCONTROL7      0x0f
#define ADCRampRate(x)		((x & 3) << 6)
#define ADCSoftRamp(x)		((x & 1) << 5)
#define ADCLeR(x)		((x & 1) << 3)
#define ADCMute(x)		((x & 1) << 2)
//Reg. 16	
#define ES8388_ADCCONTROL8      0x10
#define LADCVOL(x)		((x & 0xFF) << 0)
//Reg. 17	
#define ES8388_ADCCONTROL9      0x11
#define RADCVOL(x)		((x & 0xFF) << 0)
//Reg. 18	
#define ES8388_ADCCONTROL10     0x12
#define ALCSEL(x)		((x & 3) << 6)
#define MAXGAIN(x)		((x & 7) << 3)
#define MINGAIN(x)		((x & 7) << 0)
//Reg. 19	
#define ES8388_ADCCONTROL11     0x13
#define ALCLVL(x)		((x & 0xf) << 4)
#define ALCHLD(x)		((x & 0xf) << 0)
//Reg. 20	
#define ES8388_ADCCONTROL12     0x14
#define ALCDCY(x)		((x & 0xf) << 4)
#define ALCATK(x)		((x & 0xf) << 0)
//Reg. 21	
#define ES8388_ADCCONTROL13     0x15
#define ALCMODE(x)		((x & 1) << 7)
#define ALCZC(x)		((x & 1) << 6)
#define TIME_OUT(x)		((x & 1) << 5)
#define WIN_SIZE(x)		((x & 0x1f) << 0)
//Reg. 22	
#define ES8388_ADCCONTROL14     0x16
#define NGTH(x)			((x & 0x1f) << 3)
#define NGG(x)			((x & 3) << 1)
#define NGAT(x)			((x & 1) << 0)

//Reg. 23	
#define ES8388_DACCONTROL1      0x17
#define DACLRSWAP(x)		((x & 1) << 7)
#define DACLRP(x)		((x & 1) << 6)
#define DACWL(x)		((x & 7) << 3)
#define DACFORMAT(x)		((x & 3) << 1)
//Reg. 24	
#define ES8388_DACCONTROL2      0x18
#define DACFsMode(x)		((x & 1) << 5)
#define DACFsRatio(x)		((x & 0x1f) << 0)
//Reg. 25	
#define ES8388_DACCONTROL3      0x19
#define DACRampRate(x)		((x & 3) << 6)
#define DACSoftRamp(x)		((x & 1) << 5)
#define DACLeR(x)		((x & 1) << 3)
#define DACMute(x)		((x & 1) << 2)
//Reg. 26	
#define ES8388_DACCONTROL4      0x1a
#define DACVolumeL(x) 		((x & 0x7f) << 0)
//Reg. 27	
#define ES8388_DACCONTROL5      0x1b
#define DACVolumeR(x)		((x & 0x7f) << 0)
//Reg. 28	
#define ES8388_DACCONTROL6      0x1c
#define DeemphasisMode(x)	((x & 3) << 6)
#define DAC_invL(x)		((x & 1) << 5)
#define DAC_invR(x)		((x & 1) << 4)
#define ClickFree(x)		((x & 1) << 3)
//Reg. 29	
#define ES8388_DACCONTROL7      0x1d
#define ZeroL(x)		((x & 1) << 7)
#define ZeroR(x)		((x & 1) << 6)
#define Mono(x)			((x & 1) << 5)
#define SE(x)			((x & 7) << 2)
#define Vpp_scale(x)		((x & 3) << 0)
//Reg. 30	
#define ES8388_DACCONTROL8      0x1e
#define Shelving_a_29_24(x)	((x & 0x3F) << 0)
//Reg. 31	
#define ES8388_DACCONTROL9      0x1f
#define Shelving_a_23_16(x)	((x & 0xff) << 0)
//Reg. 32	
#define ES8388_DACCONTROL10     0x20
#define Shelving_a_15_8(x)	((x & 0xff) << 0)
//Reg. 33	
#define ES8388_DACCONTROL11     0x21
#define Shelving_a_7_0(x)	((x & 0xff) << 0)
//Reg. 34	
#define ES8388_DACCONTROL12     0x22
#define Shelving_b_29_24(x)	((x & 0x3f) << 0)
//Reg. 35	
#define ES8388_DACCONTROL13     0x23
#define Shelving_b_23_16(x)	((x & 0xff) << 0)
//Reg. 36	
#define ES8388_DACCONTROL14     0x24
#define Shelving_b_15_8(x)	((x & 0xff) << 0)
//Reg. 37	
#define ES8388_DACCONTROL15     0x25
#define Shelving_b_7_0(x)	((x & 0xff) << 0)
//Reg. 38	
#define ES8388_DACCONTROL16     0x26
#define LMIXSEL(x)		((x & 7) << 3)
#define RMIXSEL(x)		((x & 7) << 0)
//Reg. 39	
#define ES8388_DACCONTROL17     0x27
#define LD2LO(x)		((x & 1) << 7)
#define LI2LO(x)		((x & 1) << 6)
#define LI2LOVOL(x)		((x & 7) << 3)


#define ES8388_DACCONTROL18     0x28
#define ES8388_DACCONTROL19     0x29

//Reg. 42	
#define ES8388_DACCONTROL20     0x2a
#define RD2RO(x)		((x & 1) << 7)
#define RI2RO(x)		((x & 1) << 6)
#define RI2ROVOL(x)		((x & 7) << 3)
//Reg. 43	
#define ES8388_DACCONTROL21     0x2b
#define slrck(x)		((x & 1) << 7)
#define Lrck_sel(x)		((x & 1) << 6)
#define offset_dis(x)		((x & 1) << 5)
#define mclk_dis(x)		((x & 1) << 4)
#define Adc_dll_pwd(x)		((x & 1) << 3)
#define Dac_dll_pwd(x)		((x & 1) << 2)
//Reg. 44	
#define ES8388_DACCONTROL22     0x2c
#define DAC_offset(x)		((x & 0xff) << 0)
//Reg. 45	
#define ES8388_DACCONTROL23     0x2d
#define VROI(x)			((x & 1) << 4)
//Reg. 46	
#define ES8388_DACCONTROL24     0x2e
#define LOUT1VOL(x)		((x & 0x3f) << 0)
//Reg. 47	
#define ES8388_DACCONTROL25     0x2f
#define ROUT1VOL(x)		((x & 0x3f) << 0)
//Reg. 48	
#define ES8388_DACCONTROL26     0x30
#define LOUT2VOL(x)		((x & 0x3f) << 0)
//Reg. 49	
#define ES8388_DACCONTROL27     0x31
#define ROUT2VOL(x)		((x & 0x3f) << 0)

#define ES8388_DACCONTROL28     0x32

//Reg. 51	
#define ES8388_DACCONTROL29     0x33
#define hpLout1_ref1(x)		((x & 1) << 7)
#define hpLout1_ref2(x)		((x & 1) << 6)
//Reg. 52
#define ES8388_DACCONTROL30     0x34
#define spkLout2_ref1(x)	((x & 1) << 7)
#define spkLout2_ref2(x)	((x & 1) << 6)
#define mixer_ref1(x)		((x & 1) << 3)
#define mixer_ref2(x)		((x & 1) << 2)
#define MREF1(x)		((x & 1) << 1)
#define MREF2(x)		((x & 1) << 0)





#define ES8388_LADC_VOL         ES8388_ADCCONTROL8
#define ES8388_RADC_VOL         ES8388_ADCCONTROL9

#define ES8388_LDAC_VOL         ES8388_DACCONTROL4
#define ES8388_RDAC_VOL         ES8388_DACCONTROL5

#define ES8388_LOUT1_VOL        ES8388_DACCONTROL24
#define ES8388_ROUT1_VOL        ES8388_DACCONTROL25
#define ES8388_LOUT2_VOL        ES8388_DACCONTROL26
#define ES8388_ROUT2_VOL        ES8388_DACCONTROL27

#define ES8388_ADC_MUTE         ES8388_ADCCONTROL7
#define ES8388_DAC_MUTE         ES8388_DACCONTROL3



#define ES8388_IFACE            ES8388_MASTERMODE

#define ES8388_ADC_IFACE        ES8388_ADCCONTROL4
#define ES8388_ADC_SRATE        ES8388_ADCCONTROL5

#define ES8388_DAC_IFACE        ES8388_DACCONTROL1
#define ES8388_DAC_SRATE        ES8388_DACCONTROL2




/* Registers Addresses ******************************************************/

typedef struct {
	uint8_t addr;
	uint8_t val;
	unsigned int delay;
} t_codec_init_script_entry;



t_codec_init_script_entry codec_reset_script[] = {
	/* Reset, release reset*/
	{ES8388_CONTROL1, SCPReset(1), 0},
	{ES8388_CONTROL1, SCPReset(0), 0},
};


t_codec_init_script_entry codec_init_script[] = {
	/* Reset, release reset*/
	{ES8388_CONTROL1, SCPReset(1), 0},
	{ES8388_CONTROL1, SCPReset(0), 0},
	/*DACMCLK chip master*/
	{ES8388_CONTROL1, SCPReset(0) | LRCM(1) | DACMCLK(1) | SameFs(1) | SeqEn(0) | EnRef(1) | VMIDSEL(0x2), 0},
	/*All powers are normal*/
	{ES8388_CONTROL2, LPVcmMod(0) | LPVrefBuf(0) | PdnAna(0) | PdnIbiasgen(0) | VrefrLo(0) | PdnVrefbuf(0), 0},
	/* Master, MCLK/2, BCLK generated based on clock table*/
	{ES8388_MASTERMODE, MSC(1) | MCLKDIV2(1) | BCLK_INV(0) | BCLKDIV(4), 0},
	/* DACLRC and ADCLRC same, use DAC LRCK, MCLK input from PAD, */
	{ES8388_DACCONTROL21, slrck(1)|Lrck_sel(0)|offset_dis(0)|mclk_dis(0)|Adc_dll_pwd(0)|Dac_dll_pwd(0), 0},
	/* Keep all pwr on*/
	{ES8388_CHIPPOWER,  adc_DigPDN(0) | dac_DigPDN(0) | adc_stm_rst(0) | dac_stm_rst(0) |  ADCDLL_PDN(0) |  DACDLL_PDN(0) |  adcVref_PDN(0) |  dacVref_PDN(0), 0},
	/* */
	{ES8388_ADCPOWER, PdnAINL(0) | PdnAINR(0) | PdnADCL(0) | PdnADCR(0) | PdnMICB(0) | PdnADCBiasgen(0) | flashLP(0) | int1LP(0), 0},
	{ES8388_DACPOWER, PdnDACL(0) | PdnDACR(0) | LOUT1(1) | ROUT1(1) | LOUT2(1) | ROUT2(1), 0},

	{ES8388_ADCCONTROL1, MicAmpL(8) | MicAmpR(8), 0},		
	{ES8388_ADCCONTROL2, LINSEL(3) | RINSEL(3) | DSSEL(1) | DSR(0), 0},			
	{ES8388_ADCCONTROL3, DS(1) | MONOMIX(0) | TRI(0), 0},		
	/*ADC I2S 16 bit*/
	{ES8388_ADCCONTROL4, DATSEL(0) | ADCLRP(0) |  ADCWL(3) |  ADCFORMAT(0), 0},
	{ES8388_ADCCONTROL5, ADCFsMode(0) | ADCFsRatio(2), 0},

	{ES8388_ADCCONTROL8, LADCVOL(0), 0}, 
	{ES8388_ADCCONTROL9, RADCVOL(0), 0},		


	/*DAC I2S 16 bit*/
	{ES8388_DACCONTROL1, DACLRSWAP(0) | DACLRP(0) | DACWL(3) | DACFORMAT(0), 0},	
	{ES8388_DACCONTROL2, DACFsMode(0) | DACFsRatio(2), 0},	
		
	{ES8388_DACCONTROL4, DACVolumeL(0), 0}, 		
	{ES8388_DACCONTROL5, DACVolumeR(0), 0}, 		

	{ES8388_DACCONTROL16, LMIXSEL(0) | RMIXSEL(0), 0},	
	{ES8388_DACCONTROL17, LD2LO(1) | LI2LO(1) | LI2LOVOL(0), 0},
	{ES8388_DACCONTROL20, RD2RO(1) | RI2RO(1) | RI2ROVOL(0), 0},		


	{ES8388_DACCONTROL24, LOUT1VOL((0x21 - 5)), 0},		
	{ES8388_DACCONTROL25, ROUT1VOL((0x21 - 5)), 0},		
	{ES8388_DACCONTROL26, LOUT2VOL((0x21 - 5)), 0},		
	{ES8388_DACCONTROL27, ROUT2VOL((0x21 - 5)), 0},		
};


/* Commonly defined and redefined macros */

#ifndef MIN
#define MIN(a, b)                   (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)                   (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: es8388char_readreg
 *
 * Description
 *    Read the specified 16-bit register from the ES8388CHAR device.
 *
 ****************************************************************************/


#endif							/* CONFIG_AUDIO */
#endif							/* __DRIVERS_AUDIO_ES8388CHAR_H */
