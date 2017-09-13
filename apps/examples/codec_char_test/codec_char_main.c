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
unsigned int  BPSMPL = 3;
unsigned int  T_DUR;

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
static int lut24_sin[];

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
	i = T_DUR;
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
	switch((sample >> 10) & 0x3) {
		case 0: 
			return lut24_sin[sample & 0x3ff];
		case 1: 
			return lut24_sin[0x3ff - (sample & 0x3ff)];
		case 2: 
			return -lut24_sin[sample & 0x3ff];
		case 3: 
			return -lut24_sin[0x3ff - (sample & 0x3ff)];
	}

return 0;
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
	i = T_DUR;
	while (i--) {
		/* Define APB size and allocate */
		bufdesc.numbytes = BUFFSIZE;
		bufdesc.u.ppBuffer = &apb;
		ioctl(fd, AUDIOIOC_ALLOCBUFFER, (unsigned long)&bufdesc);
		printf(" + %p\n", apb);

		/* Fill up APB buffer with test SIN data */
		bufdesc.u.pBuffer = apb;
		apb->nbytes = apb->nmaxbytes;

if (BPSMPL == 3)
		for (smpl = 0; smpl < apb->nbytes / 8; smpl++) {

			*(uint32_t *)(apb->samp + smpl * 8) = lut24_sin_sample(abs_smpl*64) >> 1;
			*(uint32_t *)(apb->samp + smpl * 8 + 4) = lut24_sin_sample(abs_smpl*167) >> 1;
			abs_smpl ++;
		}
if (BPSMPL == 2)
		for (smpl = 0; smpl < apb->nbytes / 4; smpl++) {

			*(uint16_t *)(apb->samp + smpl * 4) = lut24_sin_sample(abs_smpl*64) >> 9;
			*(uint16_t *)(apb->samp + smpl * 4 + 2) = lut24_sin_sample(abs_smpl*167) >> 9;
			abs_smpl ++;

		}
if (BPSMPL == 1)
		for (smpl = 0; smpl < apb->nbytes / 4; smpl++) {

			*(uint8_t *)(apb->samp + smpl * 4) = lut24_sin_sample(abs_smpl*64) >> 17;
			*(uint8_t *)(apb->samp + smpl * 4 + 2) = lut24_sin_sample(abs_smpl*167) >> 17;
			abs_smpl ++;
		}

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
	int s_rate, s_size;
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

	printf("HELLO!!! CODEC_TEST\n");

	for(s_rate = 0; s_rate < 7; s_rate ++)
	for(s_size = 1; s_size < 4; s_size ++) {
		BPSMPL = s_size;
	
		/* Input will be the same since it is the same I2S channel */
		caps.ac_len = sizeof(struct audio_caps_s);
		caps.ac_type = AUDIO_TYPE_OUTPUT;
		caps.ac_channels = 2;
	
		caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_96K;
	
		T_DUR = 20 * (s_rate + 1);	
		switch(s_rate) {
		case 0:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_8K;
			printf("Rate 8K, Size %d \n", BPSMPL);
		break;
		case 1:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_12K;
			printf("Rate 12K, Size %d \n", BPSMPL);
		break;
		case 2:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_16K;
			printf("Rate 16K, Size %d \n", BPSMPL);
		break;
		case 3:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_24K;
			printf("Rate 24K, Size %d \n", BPSMPL);
		break;
		case 4:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_32K;
			printf("Rate 32K, Size %d \n", BPSMPL);
		break;
		case 5:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_48K;
			printf("Rate 48K, Size %d \n", BPSMPL);
		break;
		case 6:
			caps.ac_controls.hw[0] = AUDIO_SAMP_RATE_96K;
		printf("Rate 96K, Size %d \n", BPSMPL);
		break;
		}	
		
		caps.ac_controls.b[2] = 8 * BPSMPL;
        	
		ret = ioctl(fd, AUDIOIOC_CONFIGURE, &caps);
        	
		/* Here we have to diverge on 3 threads - RX and TX and main*/
        	
		sched_lock();
		printf("codec_char_main: Start receiver thread\n");
		pthread_attr_init(&attr);
        	
		/* Set the receiver stack size */
		(void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_ALCCHAR_RXSTACKSIZE);
        	
		/* Start the receiver */
        	
		ret = pthread_create(&receiver, &attr, alcchar_receiver, NULL);
		if (ret != OK) {
			sched_unlock();
			printf("codec_char_main: ERROR: failed to Start receiver thread: %d\n", ret);
			return EXIT_FAILURE;
		}
        	
		pthread_setname_np(receiver, "receiver");
        	
		/* Start the transmitter thread */
		printf("codec_char_main: Start transmitter thread\n");
		pthread_attr_init(&attr);
        	
		/* Set the transmitter stack size */
		(void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_ALCCHAR_TXSTACKSIZE);
        	
		ret = pthread_create(&transmitter, &attr, alcchar_transmitter, NULL);
		if (ret != OK) {
			sched_unlock();
			printf("codec_char_main: ERROR: failed to Start transmitter thread: %d\n", ret);
			printf("codec_char_main: Waiting for the receiver thread\n");
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
        	
        	
		printf("codec_char_main: Waiting for the transmitter thread\n");
		ret = pthread_join(transmitter, &result);
		if (ret != OK) {
			printf("codec_char_main: ERROR: pthread_join failed: %d\n", ret);
		}
        	
		printf("codec_char_main: Waiting for the receiver thread\n");
		ret = pthread_join(receiver, &result);
		if (ret != OK) {
			printf("codec_char_main: ERROR: pthread_join failed: %d\n", ret);
		}
	}       	
	ioctl(fd, AUDIOIOC_STOP, 0);
	close(fd);
       	
	return EXIT_SUCCESS;
}

static int lut24_sin[1024] = {
12855, 38566, 64277, 89987, 115698, 141408, 167118, 192827, 
218536, 244244, 269952, 295659, 321366, 347071, 372776, 398480, 
424183, 449885, 475586, 501286, 526985, 552683, 578379, 604074, 
629767, 655459, 681149, 706838, 732525, 758211, 783894, 809576, 
835256, 860934, 886610, 912284, 937956, 963625, 989292, 1014957, 
1040620, 1066280, 1091937, 1117592, 1143245, 1168895, 1194541, 1220186, 
1245827, 1271465, 1297101, 1322733, 1348362, 1373988, 1399611, 1425231, 
1450847, 1476460, 1502069, 1527675, 1553277, 1578875, 1604470, 1630061, 
1655649, 1681232, 1706811, 1732387, 1757958, 1783525, 1809088, 1834647, 
1860202, 1885752, 1911297, 1936839, 1962375, 1987907, 2013435, 2038957, 
2064475, 2089988, 2115496, 2140999, 2166497, 2191990, 2217478, 2242961, 
2268438, 2293910, 2319377, 2344838, 2370294, 2395744, 2421188, 2446627, 
2472060, 2497488, 2522909, 2548325, 2573734, 2599137, 2624535, 2649926, 
2675311, 2700690, 2726062, 2751428, 2776787, 2802140, 2827487, 2852826, 
2878159, 2903486, 2928805, 2954118, 2979423, 3004722, 3030014, 3055298, 
3080575, 3105845, 3131108, 3156364, 3181612, 3206852, 3232085, 3257310, 
3282528, 3307738, 3332941, 3358135, 3383322, 3408500, 3433671, 3458833, 
3483988, 3509134, 3534272, 3559402, 3584523, 3609636, 3634740, 3659836, 
3684924, 3710002, 3735072, 3760133, 3785186, 3810229, 3835264, 3860289, 
3885306, 3910313, 3935311, 3960300, 3985279, 4010250, 4035211, 4060162, 
4085104, 4110036, 4134958, 4159871, 4184774, 4209667, 4234551, 4259424, 
4284288, 4309141, 4333984, 4358817, 4383640, 4408453, 4433255, 4458046, 
4482828, 4507598, 4532359, 4557108, 4581847, 4606575, 4631292, 4655999, 
4680694, 4705378, 4730052, 4754714, 4779365, 4804005, 4828634, 4853251, 
4877857, 4902451, 4927034, 4951605, 4976165, 5000713, 5025249, 5049774, 
5074286, 5098787, 5123276, 5147753, 5172217, 5196670, 5221110, 5245538, 
5269953, 5294357, 5318748, 5343126, 5367492, 5391845, 5416185, 5440513, 
5464828, 5489130, 5513420, 5537696, 5561960, 5586210, 5610447, 5634671, 
5658882, 5683079, 5707263, 5731434, 5755591, 5779735, 5803865, 5827982, 
5852085, 5876174, 5900249, 5924311, 5948358, 5972392, 5996411, 6020417, 
6044408, 6068385, 6092348, 6116297, 6140231, 6164151, 6188056, 6211947, 
6235823, 6259684, 6283531, 6307363, 6331181, 6354983, 6378771, 6402543, 
6426301, 6450043, 6473770, 6497482, 6521179, 6544860, 6568526, 6592177, 
6615812, 6639432, 6663036, 6686624, 6710197, 6733754, 6757295, 6780820, 
6804330, 6827823, 6851300, 6874761, 6898207, 6921635, 6945048, 6968444, 
6991824, 7015188, 7038535, 7061865, 7085179, 7108477, 7131757, 7155021, 
7178268, 7201498, 7224711, 7247908, 7271087, 7294249, 7317394, 7340522, 
7363633, 7386726, 7409802, 7432861, 7455902, 7478926, 7501932, 7524920, 
7547891, 7570844, 7593779, 7616697, 7639596, 7662478, 7685341, 7708187, 
7731014, 7753824, 7776615, 7799388, 7822142, 7844878, 7867596, 7890295, 
7912976, 7935638, 7958282, 7980907, 8003513, 8026100, 8048668, 8071218, 
8093748, 8116260, 8138752, 8161226, 8183680, 8206115, 8228531, 8250927, 
8273304, 8295662, 8318000, 8340318, 8362617, 8384897, 8407156, 8429396, 
8451616, 8473817, 8495997, 8518157, 8540298, 8562418, 8584518, 8606598, 
8628658, 8650698, 8672717, 8694716, 8716695, 8738653, 8760590, 8782507, 
8804403, 8826279, 8848134, 8869968, 8891781, 8913574, 8935345, 8957096, 
8978825, 9000533, 9022221, 9043887, 9065531, 9087155, 9108757, 9130338, 
9151897, 9173435, 9194951, 9216446, 9237919, 9259370, 9280800, 9302208, 
9323594, 9344958, 9366300, 9387620, 9408918, 9430194, 9451448, 9472680, 
9493889, 9515076, 9536241, 9557383, 9578503, 9599601, 9620676, 9641728, 
9662757, 9683764, 9704749, 9725710, 9746649, 9767564, 9788457, 9809327, 
9830173, 9850997, 9871797, 9892575, 9913329, 9934060, 9954767, 9975451, 
9996112, 10016749, 10037363, 10057953, 10078519, 10099062, 10119581, 10140076, 
10160548, 10180995, 10201419, 10221819, 10242195, 10262546, 10282874, 10303177, 
10323457, 10343712, 10363942, 10384149, 10404331, 10424488, 10444621, 10464730, 
10484814, 10504873, 10524908, 10544918, 10564903, 10584863, 10604799, 10624710, 
10644595, 10664456, 10684292, 10704102, 10723887, 10743648, 10763383, 10783092, 
10802777, 10822436, 10842069, 10861677, 10881260, 10900817, 10920349, 10939854, 
10959334, 10978789, 10998217, 11017620, 11036997, 11056348, 11075673, 11094972, 
11114245, 11133492, 11152712, 11171907, 11191075, 11210217, 11229333, 11248422, 
11267485, 11286521, 11305531, 11324515, 11343471, 11362401, 11381305, 11400182, 
11419031, 11437855, 11456651, 11475420, 11494162, 11512878, 11531566, 11550227, 
11568862, 11587469, 11606048, 11624601, 11643126, 11661624, 11680094, 11698537, 
11716953, 11735341, 11753701, 11772034, 11790340, 11808617, 11826867, 11845089, 
11863283, 11881450, 11899588, 11917699, 11935781, 11953836, 11971862, 11989860, 
12007830, 12025772, 12043686, 12061571, 12079429, 12097257, 12115058, 12132829, 
12150573, 12168288, 12185974, 12203631, 12221260, 12238861, 12256432, 12273975, 
12291489, 12308974, 12326430, 12343857, 12361255, 12378625, 12395965, 12413276, 
12430557, 12447810, 12465034, 12482228, 12499392, 12516528, 12533634, 12550711, 
12567758, 12584775, 12601764, 12618722, 12635651, 12652550, 12669420, 12686259, 
12703069, 12719849, 12736599, 12753320, 12770010, 12786670, 12803301, 12819901, 
12836471, 12853011, 12869521, 12886001, 12902450, 12918869, 12935258, 12951616, 
12967944, 12984241, 13000508, 13016745, 13032951, 13049126, 13065270, 13081384, 
13097467, 13113520, 13129541, 13145532, 13161492, 13177421, 13193319, 13209186, 
13225022, 13240827, 13256601, 13272344, 13288056, 13303736, 13319385, 13335003, 
13350590, 13366145, 13381669, 13397161, 13412622, 13428051, 13443449, 13458816, 
13474150, 13489453, 13504725, 13519964, 13535172, 13550348, 13565493, 13580605, 
13595686, 13610734, 13625751, 13640736, 13655688, 13670609, 13685497, 13700353, 
13715178, 13729969, 13744729, 13759457, 13774152, 13788814, 13803445, 13818043, 
13832608, 13847141, 13861642, 13876109, 13890545, 13904947, 13919318, 13933655, 
13947960, 13962231, 13976470, 13990677, 14004850, 14018991, 14033098, 14047173, 
14061215, 14075223, 14089199, 14103141, 14117051, 14130927, 14144770, 14158580, 
14172356, 14186099, 14199809, 14213486, 14227129, 14240739, 14254315, 14267858, 
14281368, 14294843, 14308286, 14321694, 14335069, 14348411, 14361718, 14374992, 
14388232, 14401439, 14414611, 14427750, 14440855, 14453926, 14466963, 14479966, 
14492935, 14505870, 14518771, 14531637, 14544470, 14557269, 14570033, 14582763, 
14595459, 14608120, 14620748, 14633340, 14645899, 14658423, 14670913, 14683368, 
14695789, 14708175, 14720527, 14732844, 14745127, 14757375, 14769588, 14781767, 
14793910, 14806020, 14818094, 14830133, 14842138, 14854108, 14866043, 14877943, 
14889808, 14901638, 14913434, 14925194, 14936919, 14948609, 14960264, 14971884, 
14983468, 14995018, 15006532, 15018011, 15029455, 15040863, 15052236, 15063574, 
15074876, 15086143, 15097375, 15108571, 15119731, 15130856, 15141946, 15153000, 
15164018, 15175001, 15185948, 15196859, 15207735, 15218575, 15229380, 15240148, 
15250881, 15261578, 15272239, 15282864, 15293454, 15304007, 15314525, 15325006, 
15335452, 15345861, 15356235, 15366572, 15376873, 15387139, 15397368, 15407561, 
15417717, 15427838, 15437922, 15447970, 15457982, 15467957, 15477896, 15487799, 
15497666, 15507496, 15517289, 15527046, 15536767, 15546451, 15556099, 15565710, 
15575285, 15584823, 15594324, 15603789, 15613217, 15622608, 15631963, 15641281, 
15650563, 15659807, 15669015, 15678186, 15687320, 15696417, 15705478, 15714502, 
15723488, 15732438, 15741351, 15750227, 15759065, 15767867, 15776632, 15785360, 
15794051, 15802704, 15811321, 15819900, 15828442, 15836947, 15845415, 15853846, 
15862239, 15870595, 15878914, 15887196, 15895440, 15903647, 15911816, 15919949, 
15928043, 15936101, 15944121, 15952103, 15960048, 15967956, 15975826, 15983659, 
15991454, 15999211, 16006931, 16014613, 16022258, 16029865, 16037435, 16044966, 
16052461, 16059917, 16067336, 16074717, 16082060, 16089365, 16096633, 16103863, 
16111055, 16118209, 16125325, 16132404, 16139444, 16146447, 16153411, 16160338, 
16167227, 16174078, 16180891, 16187666, 16194403, 16201101, 16207762, 16214385, 
16220970, 16227516, 16234025, 16240495, 16246927, 16253321, 16259677, 16265995, 
16272274, 16278515, 16284718, 16290883, 16297010, 16303098, 16309148, 16315160, 
16321133, 16327068, 16332964, 16338823, 16344643, 16350424, 16356167, 16361872, 
16367538, 16373166, 16378756, 16384307, 16389819, 16395293, 16400729, 16406126, 
16411484, 16416804, 16422085, 16427328, 16432532, 16437698, 16442825, 16447913, 
16452963, 16457974, 16462946, 16467880, 16472775, 16477632, 16482449, 16487228, 
16491969, 16496670, 16501333, 16505957, 16510542, 16515089, 16519597, 16524066, 
16528496, 16532887, 16537240, 16541553, 16545828, 16550064, 16554261, 16558419, 
16562539, 16566619, 16570661, 16574663, 16578627, 16582552, 16586437, 16590284, 
16594092, 16597861, 16601591, 16605282, 16608934, 16612547, 16616121, 16619656, 
16623151, 16626608, 16630026, 16633405, 16636744, 16640045, 16643307, 16646529, 
16649712, 16652857, 16655962, 16659028, 16662054, 16665042, 16667991, 16670900, 
16673771, 16676602, 16679394, 16682147, 16684860, 16687535, 16690170, 16692766, 
16695323, 16697840, 16700319, 16702758, 16705158, 16707519, 16709840, 16712123, 
16714366, 16716569, 16718734, 16720859, 16722945, 16724992, 16726999, 16728967, 
16730896, 16732786, 16734636, 16736447, 16738219, 16739951, 16741644, 16743298, 
16744912, 16746487, 16748023, 16749519, 16750977, 16752394, 16753773, 16755112, 
16756411, 16757672, 16758893, 16760074, 16761217, 16762320, 16763383, 16764407, 
16765392, 16766337, 16767244, 16768110, 16768937, 16769725, 16770474, 16771183, 
16771853, 16772483, 16773074, 16773626, 16774138, 16774611, 16775044, 16775438, 
16775793, 16776108, 16776384, 16776620, 16776817, 16776975, 16777093, 16777172, 
};



