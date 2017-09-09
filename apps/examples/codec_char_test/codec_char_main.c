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
 * apps/examples/nxplayer/nxplayer_main.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/audio/audio.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include <fcntl.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
//#define BPSMPL	1
//#define BPSMPL	2
#define BPSMPL	3

#define BUFFSIZE (1024 * 16)

#ifndef CONFIG_EXAMPLES_ALCCHAR_RXSTACKSIZE
#define CONFIG_EXAMPLES_ALCCHAR_RXSTACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_ALCCHAR_TXSTACKSIZE
#define CONFIG_EXAMPLES_ALCCHAR_TXSTACKSIZE 2048
#endif

#define DEVICE_PATH "/dev/es8388char0"
/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
#if BPSMPL == 3
static int lut24_sin[];
#else 
static unsigned short int lut_sin[];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: i2schar_receiver()
 *
 * Description:
 *   This is the entry point for the receiver thread.
 *
 ****************************************************************************/

pthread_addr_t alcchar_receiver(pthread_addr_t arg)
{

	int fd;
	int errcode;
	FAR struct audio_buf_desc_s bufdesc;
	int i;

	/* Open the alc character device */
	fd = open(DEVICE_PATH, O_RDWR);
	if (fd < 0) {
		errcode = errno;
		printf("failed to open file, %d \n", errcode);
		return NULL;
	}

	/* This is example loop for only 1000 RX transactions. */
	i = 100;
	while (i--) {
		/* Define buffer size, if not defined 16KB buffer will be used */
		bufdesc.numbytes = BUFFSIZE;
		bufdesc.u.ppBuffer = NULL;

		/* Initiate Audio IN, read audio data */
		ioctl(fd, AUDIOIOC_DEQUEUEBUFFER, (unsigned long)&bufdesc);

		/* Do something with received data here.
		 *  APB buffer pointer is here: bufdesc.u.pBuffer
		 *  After processing buffer will be freed below.
		 */

		ioctl(fd, AUDIOIOC_FREEBUFFER, (unsigned long)&bufdesc);
	}

	close(fd);
	return NULL;
}

static unsigned int lut24_sin_sample(unsigned int sample)
{
	sample = sample % (1023*4);

	if (sample <= 1023) 
		return lut24_sin[sample];
	else if (sample <= 1023*2)
		return lut24_sin[1023*2 - sample];
	else if (sample <= 1023*3)
		return -lut24_sin[sample - 1023*2];
	else 
		return -lut24_sin[1023*4 - sample];
}

pthread_addr_t alcchar_transmitter(pthread_addr_t arg)
{

	int fd;
	int errcode;
	FAR struct audio_buf_desc_s bufdesc;
	struct ap_buffer_s *apb;
	unsigned int abs_smpl = 0;

	int i, smpl;

	/* Open the alc character device */

	fd = open(DEVICE_PATH, O_RDWR);
	if (fd < 0) {
		errcode = errno;
		printf("failed to open file, %d \n", errcode);
		return NULL;
	}

	/* This is example loop for only 1000 RX transactions. */
	i = 100;
	while (i--) {
		/* Define APB size and allocate */
		bufdesc.numbytes = BUFFSIZE;
		bufdesc.u.ppBuffer = &apb;
		ioctl(fd, AUDIOIOC_ALLOCBUFFER, (unsigned long)&bufdesc);
		printf(" + %p\n", apb);

		/* Fill up APB buffer with test SIN data */
		bufdesc.u.pBuffer = apb;
		apb->nbytes = apb->nmaxbytes;

#if (BPSMPL == 3)
		for (smpl = 0; smpl < apb->nbytes / 8; smpl++) {

			*(uint32_t *)(apb->samp + smpl * 8) = lut24_sin_sample(abs_smpl*64) >> 1;
			*(uint32_t *)(apb->samp + smpl * 8 + 4) = lut24_sin_sample(abs_smpl*167) >> 1;
			abs_smpl ++;
		}
#endif

#if (BPSMPL == 2)
		for (smpl = 0; smpl < apb->nbytes / 4; smpl++) {

			*(uint16_t *)(apb->samp + smpl * 4) = ((int32_t) 0x8000 + lut_sin[(smpl & 0x3F) << 4]);
			*(uint16_t *)(apb->samp + smpl * 4 + 2) = ((int32_t) 0x8000 + lut_sin[(smpl & 0x7F) << 3]);
		}
#endif

#if (BPSMPL == 1)
		for (smpl = 0; smpl < apb->nbytes / 4; smpl++) {

			*(uint8_t *)(apb->samp + smpl * 4) = ((int32_t) 0x8000 + (uint16_t) lut_sin[(smpl & 0x3F) << 4]) / 256;
			*(uint8_t *)(apb->samp + smpl * 4 + 2) = ((int32_t) 0x8000 + (uint16_t) lut_sin[((smpl) & 0x7F) << 3]) / 256;
		}
#endif

		/* Enqueue buffer to be sent */
		ioctl(fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&bufdesc);
	}

	close(fd);
	return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int es_test_main(int argc, char *argv[])
#endif
{
	pthread_attr_t attr;
	pthread_addr_t result;
	pthread_t transmitter;
	pthread_t receiver;
	int ret;

	int fd;
	int errcode;

	struct audio_caps_s caps;

	/* Open the alc character device */

	fd = open(DEVICE_PATH, O_RDWR);
	if (fd < 0) {
		errcode = errno;
		printf("failed to open file, %d \n", errcode);
		return EXIT_FAILURE;
	}

	printf("HELLO!!! ALC_TEST\n");

	/* Input will be the same since it is the same I2S channel */
	caps.ac_len = sizeof(struct audio_caps_s);
	caps.ac_type = AUDIO_TYPE_OUTPUT;
	caps.ac_channels = 2;

	caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_48K;
	caps.ac_controls.b[2] = 8 * BPSMPL;

	ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);

	/* Here we have to diverge on 3 threads - RX and TX and main*/

	sched_lock();
	printf("alcchar_main: Start receiver thread\n");
	pthread_attr_init(&attr);

	/* Set the receiver stack size */
	(void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_ALCCHAR_RXSTACKSIZE);

	/* Start the receiver */

	ret = pthread_create(&receiver, &attr, alcchar_receiver, NULL);
	if (ret != OK) {
		sched_unlock();
		printf("alcchar_main: ERROR: failed to Start receiver thread: %d\n", ret);
		return EXIT_FAILURE;
	}

	pthread_setname_np(receiver, "receiver");

	/* Start the transmitter thread */
	printf("alcchar_main: Start transmitter thread\n");
	pthread_attr_init(&attr);

	/* Set the transmitter stack size */
	(void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_ALCCHAR_TXSTACKSIZE);

	ret = pthread_create(&transmitter, &attr, alcchar_transmitter, NULL);
	if (ret != OK) {
		sched_unlock();
		printf("alcchar_main: ERROR: failed to Start transmitter thread: %d\n", ret);
		printf("alcchar_main: Waiting for the receiver thread\n");
		(void)pthread_join(receiver, &result);
		return EXIT_FAILURE;
	}

	pthread_setname_np(transmitter, "transmitter");
	sched_unlock();

	ioctl(fd, AUDIOIOC_START, 0);

	/* Set Volume */
	caps.ac_type = AUDIO_TYPE_FEATURE;

	caps.ac_format.hw = AUDIO_FU_VOLUME;
	caps.ac_controls.hw[0] = 800;
	ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);

	caps.ac_format.hw = AUDIO_FU_MUTE;
	caps.ac_controls.b[0] = false;
	ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);

	caps.ac_format.hw = AUDIO_FU_BALANCE;
	caps.ac_controls.hw[0] = -500;
	ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);

	/* Values -16 ~ 0 ~ 53 equal to -12dB ~ 0dB ~ 39.75 */
	caps.ac_format.hw = AUDIO_FU_MICGAIN;
	caps.ac_controls.hw[0] = 0;
	ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);


	printf("alcchar_main: Waiting for the transmitter thread\n");
	ret = pthread_join(transmitter, &result);
	if (ret != OK) {
		printf("alcchar_main: ERROR: pthread_join failed: %d\n", ret);
	}

	printf("alcchar_main: Waiting for the receiver thread\n");
	ret = pthread_join(receiver, &result);
	if (ret != OK) {
		printf("alcchar_main: ERROR: pthread_join failed: %d\n", ret);
	}

	ioctl(fd, AUDIOIOC_STOP, 0);
	close(fd);

	return EXIT_SUCCESS;
}

#if BPSMPL == 3
static int lut24_sin[1024] = {
	0, 25736, 51472, 77208, 102943, 128678, 154413, 180148,
	205882, 231616, 257349, 283082, 308814, 334545, 360275, 386005,
	411733, 437461, 463188, 488913, 514638, 540361, 566083, 591803,
	617523, 643240, 668957, 694671, 720384, 746096, 771805, 797513,
	823219, 848923, 874625, 900325, 926022, 951718, 977411, 1003102,
	1028791, 1054477, 1080161, 1105842, 1131521, 1157197, 1182870, 1208541,
	1234208, 1259873, 1285535, 1311194, 1336849, 1362502, 1388151, 1413797,
	1439440, 1465079, 1490715, 1516348, 1541976, 1567601, 1593223, 1618841,
	1644455, 1670065, 1695671, 1721273, 1746871, 1772465, 1798055, 1823640,
	1849222, 1874799, 1900371, 1925939, 1951503, 1977062, 2002616, 2028165,
	2053710, 2079250, 2104785, 2130315, 2155840, 2181360, 2206875, 2232385,
	2257889, 2283389, 2308882, 2334371, 2359854, 2385331, 2410802, 2436268,
	2461729, 2487183, 2512632, 2538074, 2563511, 2588942, 2614367, 2639785,
	2665197, 2690603, 2716003, 2741396, 2766783, 2792163, 2817537, 2842904,
	2868264, 2893618, 2918965, 2944305, 2969638, 2994964, 3020283, 3045595,
	3070900, 3096197, 3121487, 3146770, 3172046, 3197314, 3222574, 3247827,
	3273072, 3298310, 3323540, 3348762, 3373976, 3399182, 3424380, 3449570,
	3474752, 3499926, 3525091, 3550249, 3575398, 3600538, 3625670, 3650794,
	3675909, 3701015, 3726112, 3751201, 3776281, 3801352, 3826414, 3851467,
	3876512, 3901546, 3926572, 3951589, 3976596, 4001594, 4026582, 4051561,
	4076531, 4101491, 4126441, 4151381, 4176312, 4201233, 4226144, 4251045,
	4275936, 4300817, 4325688, 4350549, 4375399, 4400239, 4425069, 4449888,
	4474697, 4499496, 4524283, 4549061, 4573827, 4598583, 4623328, 4648062,
	4672785, 4697497, 4722198, 4746888, 4771566, 4796234, 4820890, 4845535,
	4870168, 4894790, 4919401, 4944000, 4968587, 4993163, 5017727, 5042279,
	5066819, 5091347, 5115863, 5140367, 5164860, 5189340, 5213807, 5238263,
	5262706, 5287137, 5311555, 5335961, 5360354, 5384735, 5409103, 5433458,
	5457801, 5482130, 5506447, 5530751, 5555042, 5579319, 5603584, 5627835,
	5652074, 5676298, 5700510, 5724708, 5748893, 5773064, 5797221, 5821365,
	5845495, 5869612, 5893714, 5917803, 5941878, 5965938, 5989985, 6014018,
	6038036, 6062041, 6086031, 6110007, 6133968, 6157915, 6181847, 6205765,
	6229668, 6253557, 6277431, 6301290, 6325134, 6348964, 6372778, 6396578,
	6420362, 6444132, 6467886, 6491625, 6515348, 6539057, 6562750, 6586427,
	6610089, 6633736, 6657366, 6680982, 6704581, 6728165, 6751733, 6775285,
	6798821, 6822341, 6845845, 6869333, 6892804, 6916260, 6939699, 6963122,
	6986528, 7009918, 7033292, 7056649, 7079989, 7103313, 7126620, 7149910,
	7173184, 7196440, 7219680, 7242903, 7266108, 7289297, 7312468, 7335622,
	7358759, 7381878, 7404981, 7428065, 7451133, 7474182, 7497215, 7520229,
	7543226, 7566205, 7589166, 7612110, 7635035, 7657943, 7680832, 7703704,
	7726557, 7749392, 7772209, 7795008, 7817788, 7840550, 7863293, 7886018,
	7908724, 7931412, 7954081, 7976732, 7999363, 8021976, 8044570, 8067145,
	8089701, 8112238, 8134756, 8157254, 8179734, 8202194, 8224635, 8247057,
	8269459, 8291842, 8314205, 8336549, 8358873, 8381177, 8403462, 8425727,
	8447972, 8470197, 8492402, 8514587, 8536753, 8558898, 8581023, 8603128,
	8625212, 8647277, 8669320, 8691344, 8713347, 8735330, 8757292, 8779233,
	8801154, 8823054, 8844933, 8866792, 8888629, 8910446, 8932242, 8954016,
	8975770, 8997503, 9019214, 9040904, 9062573, 9084220, 9105847, 9127451,
	9149035, 9170596, 9192136, 9213655, 9235152, 9256627, 9278080, 9299512,
	9320921, 9342309, 9363675, 9385018, 9406340, 9427639, 9448916, 9470171,
	9491404, 9512615, 9533803, 9554968, 9576111, 9597232, 9618330, 9639405,
	9660458, 9681488, 9702495, 9723479, 9744441, 9765379, 9786295, 9807187,
	9828057, 9848903, 9869726, 9890526, 9911303, 9932056, 9952786, 9973492,
	9994175, 10014835, 10035471, 10056083, 10076672, 10097237, 10117778, 10138295,
	10158789, 10179259, 10199704, 10220126, 10240524, 10260897, 10281247, 10301572,
	10321873, 10342149, 10362402, 10382630, 10402833, 10423012, 10443167, 10463297,
	10483402, 10503483, 10523539, 10543570, 10563576, 10583558, 10603514, 10623446,
	10643353, 10663234, 10683091, 10702922, 10722728, 10742509, 10762265, 10781995,
	10801700, 10821380, 10841034, 10860662, 10880266, 10899843, 10919395, 10938921,
	10958421, 10977896, 10997345, 11016767, 11036164, 11055535, 11074880, 11094199,
	11113492, 11132759, 11151999, 11171213, 11190401, 11209563, 11228698, 11247807,
	11266889, 11285945, 11304974, 11323977, 11342953, 11361902, 11380825, 11399720,
	11418589, 11437431, 11456247, 11475035, 11493796, 11512530, 11531237, 11549917,
	11568570, 11587196, 11605794, 11624365, 11642908, 11661425, 11679913, 11698375,
	11716808, 11735215, 11753593, 11771944, 11790267, 11808563, 11826831, 11845070,
	11863282, 11881467, 11899623, 11917751, 11935851, 11953923, 11971967, 11989982,
	12007970, 12025929, 12043860, 12061763, 12079637, 12097483, 12115300, 12133089,
	12150849, 12168581, 12186284, 12203958, 12221604, 12239221, 12256809, 12274368,
	12291898, 12309400, 12326872, 12344316, 12361730, 12379115, 12396472, 12413799,
	12431096, 12448365, 12465604, 12482814, 12499995, 12517146, 12534267, 12551360,
	12568422, 12585455, 12602459, 12619433, 12636377, 12653291, 12670176, 12687031,
	12703856, 12720651, 12737416, 12754151, 12770856, 12787531, 12804176, 12820791,
	12837376, 12853930, 12870454, 12886948, 12903412, 12919845, 12936248, 12952621,
	12968963, 12985274, 13001555, 13017805, 13034025, 13050214, 13066372, 13082500,
	13098596, 13114662, 13130697, 13146701, 13162675, 13178617, 13194528, 13210408,
	13226258, 13242075, 13257862, 13273618, 13289342, 13305035, 13320697, 13336328,
	13351927, 13367495, 13383031, 13398535, 13414009, 13429450, 13444860, 13460239,
	13475585, 13490900, 13506184, 13521435, 13536655, 13551843, 13566999, 13582123,
	13597215, 13612275, 13627303, 13642299, 13657263, 13672194, 13687094, 13701961,
	13716796, 13731599, 13746370, 13761108, 13775813, 13790487, 13805128, 13819736,
	13834312, 13848855, 13863366, 13877844, 13892290, 13906703, 13921083, 13935430,
	13949744, 13964026, 13978275, 13992491, 14006674, 14020824, 14034941, 14049025,
	14063076, 14077094, 14091079, 14105030, 14118949, 14132834, 14146686, 14160505,
	14174290, 14188042, 14201760, 14215446, 14229097, 14242716, 14256300, 14269852,
	14283369, 14296853, 14310303, 14323720, 14337103, 14350452, 14363768, 14377049,
	14390297, 14403511, 14416691, 14429838, 14442950, 14456028, 14469072, 14482082,
	14495058, 14508000, 14520908, 14533782, 14546621, 14559427, 14572197, 14584934,
	14597637, 14610305, 14622938, 14635537, 14648102, 14660632, 14673128, 14685590,
	14698016, 14710408, 14722766, 14735089, 14747377, 14759631, 14771849, 14784033,
	14796183, 14808297, 14820377, 14832421, 14844431, 14856406, 14868346, 14880251,
	14892121, 14903956, 14915755, 14927520, 14939250, 14950944, 14962603, 14974227,
	14985816, 14997370, 15008888, 15020371, 15031818, 15043231, 15054607, 15065949,
	15077255, 15088525, 15099760, 15110960, 15122123, 15133252, 15144344, 15155401,
	15166423, 15177408, 15188358, 15199273, 15210151, 15220994, 15231801, 15242572,
	15253307, 15264006, 15274669, 15285297, 15295888, 15306444, 15316963, 15327446,
	15337894, 15348305, 15358680, 15369019, 15379322, 15389588, 15399819, 15410013,
	15420171, 15430292, 15440378, 15450427, 15460439, 15470415, 15480355, 15490259,
	15500126, 15509956, 15519750, 15529507, 15539228, 15548913, 15558560, 15568172,
	15577746, 15587284, 15596785, 15606250, 15615677, 15625068, 15634423, 15643740,
	15653021, 15662265, 15671472, 15680642, 15689775, 15698871, 15707930, 15716953,
	15725938, 15734887, 15743798, 15752672, 15761509, 15770310, 15779073, 15787798,
	15796487, 15805139, 15813753, 15822330, 15830870, 15839373, 15847838, 15856267,
	15864657, 15873011, 15881327, 15889606, 15897847, 15906051, 15914218, 15922347,
	15930438, 15938493, 15946509, 15954488, 15962430, 15970334, 15978200, 15986029,
	15993820, 16001574, 16009290, 16016968, 16024609, 16032212, 16039777, 16047304,
	16054794, 16062246, 16069660, 16077036, 16084374, 16091675, 16098938, 16106162,
	16113349, 16120498, 16127609, 16134683, 16141718, 16148715, 16155674, 16162595,
	16169478, 16176323, 16183130, 16189899, 16196630, 16203323, 16209977, 16216594,
	16223172, 16229712, 16236214, 16242678, 16249103, 16255491, 16261840, 16268150,
	16274423, 16280657, 16286853, 16293010, 16299130, 16305210, 16311253, 16317257,
	16323223, 16329150, 16335039, 16340889, 16346701, 16352475, 16358210, 16363906,
	16369564, 16375184, 16380765, 16386307, 16391811, 16397277, 16402703, 16408091,
	16413441, 16418752, 16424024, 16429258, 16434453, 16439609, 16444726, 16449805,
	16454846, 16459847, 16464810, 16469734, 16474619, 16479465, 16484273, 16489042,
	16493772, 16498463, 16503116, 16507730, 16512304, 16516840, 16521337, 16525796,
	16530215, 16534595, 16538937, 16543239, 16547503, 16551728, 16555913, 16560060,
	16564168, 16568237, 16572267, 16576258, 16580210, 16584123, 16587996, 16591831,
	16595627, 16599384, 16603101, 16606780, 16610419, 16614020, 16617581, 16621103,
	16624587, 16628031, 16631435, 16634801, 16638128, 16641415, 16644663, 16647873,
	16651043, 16654173, 16657265, 16660317, 16663330, 16666304, 16669239, 16672135,
	16674991, 16677808, 16680586, 16683324, 16686024, 16688684, 16691304, 16693886,
	16696428, 16698931, 16701395, 16703819, 16706204, 16708550, 16710856, 16713123,
	16715351, 16717539, 16719688, 16721798, 16723868, 16725900, 16727891, 16729843,
	16731756, 16733630, 16735464, 16737259, 16739014, 16740730, 16742407, 16744044,
	16745642, 16747201, 16748720, 16750199, 16751639, 16753040, 16754402, 16755724,
	16757006, 16758249, 16759453, 16760617, 16761742, 16762827, 16763873, 16764880,
	16765847, 16766774, 16767662, 16768511, 16769320, 16770090, 16770820, 16771511,
	16772162, 16772774, 16773346, 16773879, 16774373, 16774827, 16775241, 16775616,
	16775952, 16776248, 16776504, 16776722, 16776899, 16777037, 16777136, 16777195,
};
#else
static unsigned short int lut_sin[1024] = {
	0x0, 0x2, 0x5, 0x9, 0xe, 0x14, 0x1c, 0x25,
	0x2f, 0x3b, 0x47, 0x55, 0x64, 0x75, 0x86, 0x99,
	0xad, 0xc3, 0xd9, 0xf1, 0x10a, 0x124, 0x13f, 0x15c,
	0x17a, 0x199, 0x1b9, 0x1db, 0x1fe, 0x221, 0x247, 0x26d,
	0x295, 0x2bd, 0x2e8, 0x313, 0x33f, 0x36d, 0x39c, 0x3cc,
	0x3fd, 0x42f, 0x463, 0x498, 0x4ce, 0x505, 0x53e, 0x577,
	0x5b2, 0x5ee, 0x62b, 0x669, 0x6a9, 0x6e9, 0x72b, 0x76e,
	0x7b2, 0x7f8, 0x83e, 0x886, 0x8cf, 0x919, 0x964, 0x9b0,
	0x9fd, 0xa4c, 0xa9b, 0xaec, 0xb3e, 0xb91, 0xbe5, 0xc3b,
	0xc91, 0xce9, 0xd41, 0xd9b, 0xdf6, 0xe52, 0xeaf, 0xf0d,
	0xf6c, 0xfcc, 0x102e, 0x1090, 0x10f4, 0x1158, 0x11be, 0x1225,
	0x128d, 0x12f6, 0x1360, 0x13cb, 0x1437, 0x14a4, 0x1512, 0x1581,
	0x15f1, 0x1662, 0x16d4, 0x1748, 0x17bc, 0x1831, 0x18a7, 0x191f,
	0x1997, 0x1a10, 0x1a8a, 0x1b05, 0x1b82, 0x1bff, 0x1c7d, 0x1cfc,
	0x1d7c, 0x1dfd, 0x1e7f, 0x1f02, 0x1f85, 0x200a, 0x2090, 0x2116,
	0x219e, 0x2226, 0x22b0, 0x233a, 0x23c5, 0x2451, 0x24de, 0x256c,
	0x25fa, 0x268a, 0x271a, 0x27ab, 0x283d, 0x28d0, 0x2964, 0x29f9,
	0x2a8e, 0x2b24, 0x2bbc, 0x2c53, 0x2cec, 0x2d86, 0x2e20, 0x2ebb,
	0x2f57, 0x2ff4, 0x3091, 0x312f, 0x31ce, 0x326e, 0x330e, 0x33b0,
	0x3451, 0x34f4, 0x3598, 0x363c, 0x36e0, 0x3786, 0x382c, 0x38d3,
	0x397b, 0x3a23, 0x3acc, 0x3b76, 0x3c20, 0x3ccb, 0x3d77, 0x3e23,
	0x3ed0, 0x3f7d, 0x402b, 0x40da, 0x4189, 0x4239, 0x42ea, 0x439b,
	0x444d, 0x44ff, 0x45b2, 0x4666, 0x471a, 0x47ce, 0x4883, 0x4939,
	0x49ef, 0x4aa6, 0x4b5d, 0x4c15, 0x4ccd, 0x4d85, 0x4e3f, 0x4ef8,
	0x4fb2, 0x506d, 0x5128, 0x51e4, 0x52a0, 0x535c, 0x5419, 0x54d6,
	0x5594, 0x5652, 0x5710, 0x57cf, 0x588f, 0x594e, 0x5a0e, 0x5acf,
	0x5b8f, 0x5c50, 0x5d12, 0x5dd4, 0x5e96, 0x5f58, 0x601b, 0x60de,
	0x61a1, 0x6265, 0x6329, 0x63ed, 0x64b2, 0x6576, 0x663b, 0x6701,
	0x67c6, 0x688c, 0x6952, 0x6a18, 0x6ade, 0x6ba5, 0x6c6c, 0x6d33,
	0x6dfa, 0x6ec1, 0x6f89, 0x7051, 0x7118, 0x71e0, 0x72a8, 0x7371,
	0x7439, 0x7501, 0x75ca, 0x7693, 0x775b, 0x7824, 0x78ed, 0x79b6,
	0x7a7f, 0x7b48, 0x7c11, 0x7cdb, 0x7da4, 0x7e6d, 0x7f36, 0x8000,
	0x8000, 0x80c9, 0x8192, 0x825b, 0x8324, 0x83ee, 0x84b7, 0x8580,
	0x8649, 0x8712, 0x87db, 0x88a4, 0x896c, 0x8a35, 0x8afe, 0x8bc6,
	0x8c8e, 0x8d57, 0x8e1f, 0x8ee7, 0x8fae, 0x9076, 0x913e, 0x9205,
	0x92cc, 0x9393, 0x945a, 0x9521, 0x95e7, 0x96ad, 0x9773, 0x9839,
	0x98fe, 0x99c4, 0x9a89, 0x9b4d, 0x9c12, 0x9cd6, 0x9d9a, 0x9e5e,
	0x9f21, 0x9fe4, 0xa0a7, 0xa169, 0xa22b, 0xa2ed, 0xa3af, 0xa470,
	0xa530, 0xa5f1, 0xa6b1, 0xa770, 0xa830, 0xa8ef, 0xa9ad, 0xaa6b,
	0xab29, 0xabe6, 0xaca3, 0xad5f, 0xae1b, 0xaed7, 0xaf92, 0xb04d,
	0xb107, 0xb1c0, 0xb27a, 0xb332, 0xb3ea, 0xb4a2, 0xb559, 0xb610,
	0xb6c6, 0xb77c, 0xb831, 0xb8e5, 0xb999, 0xba4d, 0xbb00, 0xbbb2,
	0xbc64, 0xbd15, 0xbdc6, 0xbe76, 0xbf25, 0xbfd4, 0xc082, 0xc12f,
	0xc1dc, 0xc288, 0xc334, 0xc3df, 0xc489, 0xc533, 0xc5dc, 0xc684,
	0xc72c, 0xc7d3, 0xc879, 0xc91f, 0xc9c3, 0xca67, 0xcb0b, 0xcbae,
	0xcc4f, 0xccf1, 0xcd91, 0xce31, 0xced0, 0xcf6e, 0xd00b, 0xd0a8,
	0xd144, 0xd1df, 0xd279, 0xd313, 0xd3ac, 0xd443, 0xd4db, 0xd571,
	0xd606, 0xd69b, 0xd72f, 0xd7c2, 0xd854, 0xd8e5, 0xd975, 0xda05,
	0xda93, 0xdb21, 0xdbae, 0xdc3a, 0xdcc5, 0xdd4f, 0xddd9, 0xde61,
	0xdee9, 0xdf6f, 0xdff5, 0xe07a, 0xe0fd, 0xe180, 0xe202, 0xe283,
	0xe303, 0xe382, 0xe400, 0xe47d, 0xe4fa, 0xe575, 0xe5ef, 0xe668,
	0xe6e0, 0xe758, 0xe7ce, 0xe843, 0xe8b7, 0xe92b, 0xe99d, 0xea0e,
	0xea7e, 0xeaed, 0xeb5b, 0xebc8, 0xec34, 0xec9f, 0xed09, 0xed72,
	0xedda, 0xee41, 0xeea7, 0xef0b, 0xef6f, 0xefd1, 0xf033, 0xf093,
	0xf0f2, 0xf150, 0xf1ad, 0xf209, 0xf264, 0xf2be, 0xf316, 0xf36e,
	0xf3c4, 0xf41a, 0xf46e, 0xf4c1, 0xf513, 0xf564, 0xf5b3, 0xf602,
	0xf64f, 0xf69b, 0xf6e6, 0xf730, 0xf779, 0xf7c1, 0xf807, 0xf84d,
	0xf891, 0xf8d4, 0xf916, 0xf956, 0xf996, 0xf9d4, 0xfa11, 0xfa4d,
	0xfa88, 0xfac1, 0xfafa, 0xfb31, 0xfb67, 0xfb9c, 0xfbd0, 0xfc02,
	0xfc33, 0xfc63, 0xfc92, 0xfcc0, 0xfcec, 0xfd17, 0xfd42, 0xfd6a,
	0xfd92, 0xfdb8, 0xfdde, 0xfe01, 0xfe24, 0xfe46, 0xfe66, 0xfe85,
	0xfea3, 0xfec0, 0xfedb, 0xfef5, 0xff0e, 0xff26, 0xff3c, 0xff52,
	0xff66, 0xff79, 0xff8a, 0xff9b, 0xffaa, 0xffb8, 0xffc4, 0xffd0,
	0xffda, 0xffe3, 0xffeb, 0xfff1, 0xfff6, 0xfffa, 0xfffd, 0xffff,
	0xffff, 0xfffe, 0xfffc, 0xfff8, 0xfff4, 0xffee, 0xffe7, 0xffdf,
	0xffd5, 0xffca, 0xffbe, 0xffb1, 0xffa2, 0xff93, 0xff82, 0xff6f,
	0xff5c, 0xff47, 0xff31, 0xff1a, 0xff02, 0xfee8, 0xfece, 0xfeb1,
	0xfe94, 0xfe76, 0xfe56, 0xfe35, 0xfe13, 0xfdf0, 0xfdcb, 0xfda5,
	0xfd7e, 0xfd56, 0xfd2d, 0xfd02, 0xfcd6, 0xfca9, 0xfc7b, 0xfc4b,
	0xfc1b, 0xfbe9, 0xfbb6, 0xfb82, 0xfb4c, 0xfb16, 0xfade, 0xfaa5,
	0xfa6b, 0xfa2f, 0xf9f3, 0xf9b5, 0xf976, 0xf936, 0xf8f5, 0xf8b2,
	0xf86f, 0xf82a, 0xf7e4, 0xf79d, 0xf755, 0xf70c, 0xf6c1, 0xf675,
	0xf629, 0xf5db, 0xf58c, 0xf53b, 0xf4ea, 0xf498, 0xf444, 0xf3ef,
	0xf399, 0xf342, 0xf2ea, 0xf291, 0xf237, 0xf1db, 0xf17f, 0xf121,
	0xf0c3, 0xf063, 0xf002, 0xefa0, 0xef3d, 0xeed9, 0xee74, 0xee0e,
	0xeda6, 0xed3e, 0xecd5, 0xec6a, 0xebff, 0xeb92, 0xeb24, 0xeab6,
	0xea46, 0xe9d6, 0xe964, 0xe8f1, 0xe87d, 0xe809, 0xe793, 0xe71c,
	0xe6a4, 0xe62c, 0xe5b2, 0xe537, 0xe4bc, 0xe43f, 0xe3c1, 0xe343,
	0xe2c3, 0xe243, 0xe1c1, 0xe13f, 0xe0bc, 0xe037, 0xdfb2, 0xdf2c,
	0xdea5, 0xde1d, 0xdd94, 0xdd0a, 0xdc80, 0xdbf4, 0xdb68, 0xdada,
	0xda4c, 0xd9bd, 0xd92d, 0xd89c, 0xd80b, 0xd778, 0xd6e5, 0xd651,
	0xd5bc, 0xd526, 0xd48f, 0xd3f8, 0xd35f, 0xd2c6, 0xd22c, 0xd192,
	0xd0f6, 0xd05a, 0xcfbd, 0xcf1f, 0xce80, 0xcde1, 0xcd41, 0xcca0,
	0xcbff, 0xcb5c, 0xcab9, 0xca16, 0xc971, 0xc8cc, 0xc826, 0xc77f,
	0xc6d8, 0xc630, 0xc588, 0xc4de, 0xc434, 0xc38a, 0xc2de, 0xc232,
	0xc186, 0xc0d9, 0xc02b, 0xbf7c, 0xbecd, 0xbe1e, 0xbd6d, 0xbcbd,
	0xbc0b, 0xbb59, 0xbaa6, 0xb9f3, 0xb940, 0xb88b, 0xb7d6, 0xb721,
	0xb66b, 0xb5b5, 0xb4fe, 0xb446, 0xb38e, 0xb2d6, 0xb21d, 0xb164,
	0xb0aa, 0xafef, 0xaf34, 0xae79, 0xadbd, 0xad01, 0xac45, 0xab88,
	0xaaca, 0xaa0c, 0xa94e, 0xa88f, 0xa7d0, 0xa711, 0xa651, 0xa591,
	0xa4d0, 0xa40f, 0xa34e, 0xa28c, 0xa1ca, 0xa108, 0xa045, 0x9f83,
	0x9ebf, 0x9dfc, 0x9d38, 0x9c74, 0x9bb0, 0x9aeb, 0x9a26, 0x9961,
	0x989c, 0x97d6, 0x9710, 0x964a, 0x9584, 0x94bd, 0x93f7, 0x9330,
	0x9269, 0x91a1, 0x90da, 0x9012, 0x8f4b, 0x8e83, 0x8dbb, 0x8cf3,
	0x8c2a, 0x8b62, 0x8a99, 0x89d1, 0x8908, 0x883f, 0x8776, 0x86ad,
	0x85e4, 0x851b, 0x8452, 0x8389, 0x82c0, 0x81f7, 0x812d, 0x8064,
	0x7f9b, 0x7ed2, 0x7e08, 0x7d3f, 0x7c76, 0x7bad, 0x7ae4, 0x7a1b,
	0x7952, 0x7889, 0x77c0, 0x76f7, 0x762e, 0x7566, 0x749d, 0x73d5,
	0x730c, 0x7244, 0x717c, 0x70b4, 0x6fed, 0x6f25, 0x6e5e, 0x6d96,
	0x6ccf, 0x6c08, 0x6b42, 0x6a7b, 0x69b5, 0x68ef, 0x6829, 0x6763,
	0x669e, 0x65d9, 0x6514, 0x644f, 0x638b, 0x62c7, 0x6203, 0x6140,
	0x607c, 0x5fba, 0x5ef7, 0x5e35, 0x5d73, 0x5cb1, 0x5bf0, 0x5b2f,
	0x5a6e, 0x59ae, 0x58ee, 0x582f, 0x5770, 0x56b1, 0x55f3, 0x5535,
	0x5477, 0x53ba, 0x52fe, 0x5242, 0x5186, 0x50cb, 0x5010, 0x4f55,
	0x4e9b, 0x4de2, 0x4d29, 0x4c71, 0x4bb9, 0x4b01, 0x4a4a, 0x4994,
	0x48de, 0x4829, 0x4774, 0x46bf, 0x460c, 0x4559, 0x44a6, 0x43f4,
	0x4342, 0x4292, 0x41e1, 0x4132, 0x4083, 0x3fd4, 0x3f26, 0x3e79,
	0x3dcd, 0x3d21, 0x3c75, 0x3bcb, 0x3b21, 0x3a77, 0x39cf, 0x3927,
	0x3880, 0x37d9, 0x3733, 0x368e, 0x35e9, 0x3546, 0x34a3, 0x3400,
	0x335f, 0x32be, 0x321e, 0x317f, 0x30e0, 0x3042, 0x2fa5, 0x2f09,
	0x2e6d, 0x2dd3, 0x2d39, 0x2ca0, 0x2c07, 0x2b70, 0x2ad9, 0x2a43,
	0x29ae, 0x291a, 0x2887, 0x27f4, 0x2763, 0x26d2, 0x2642, 0x25b3,
	0x2525, 0x2497, 0x240b, 0x237f, 0x22f5, 0x226b, 0x21e2, 0x215a,
	0x20d3, 0x204d, 0x1fc8, 0x1f43, 0x1ec0, 0x1e3e, 0x1dbc, 0x1d3c,
	0x1cbc, 0x1c3e, 0x1bc0, 0x1b43, 0x1ac8, 0x1a4d, 0x19d3, 0x195b,
	0x18e3, 0x186c, 0x17f6, 0x1782, 0x170e, 0x169b, 0x1629, 0x15b9,
	0x1549, 0x14db, 0x146d, 0x1400, 0x1395, 0x132a, 0x12c1, 0x1259,
	0x11f1, 0x118b, 0x1126, 0x10c2, 0x105f, 0xffd, 0xf9c, 0xf3c,
	0xede, 0xe80, 0xe24, 0xdc8, 0xd6e, 0xd15, 0xcbd, 0xc66,
	0xc10, 0xbbb, 0xb67, 0xb15, 0xac4, 0xa73, 0xa24, 0x9d6,
	0x98a, 0x93e, 0x8f3, 0x8aa, 0x862, 0x81b, 0x7d5, 0x790,
	0x74d, 0x70a, 0x6c9, 0x689, 0x64a, 0x60c, 0x5d0, 0x594,
	0x55a, 0x521, 0x4e9, 0x4b3, 0x47d, 0x449, 0x416, 0x3e4,
	0x3b4, 0x384, 0x356, 0x329, 0x2fd, 0x2d2, 0x2a9, 0x281,
	0x25a, 0x234, 0x20f, 0x1ec, 0x1ca, 0x1a9, 0x189, 0x16b,
	0x14e, 0x131, 0x117, 0xfd, 0xe5, 0xce, 0xb8, 0xa3,
	0x90, 0x7d, 0x6c, 0x5d, 0x4e, 0x41, 0x35, 0x2a,
	0x20, 0x18, 0x11, 0xb, 0x7, 0x3, 0x1, 0x0,
};


#endif


