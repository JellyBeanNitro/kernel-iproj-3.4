/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>

#include <linux/msm_adc.h>

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO	/*                                        */
#include "../../lge/include/lg_power_common.h"
#endif

#define KELVINMIL_DEGMIL	273160

#ifdef CONFIG_MACH_LGE_325_BOARD	/*                                               */
const struct adc_map_pt adcmap_batttherm[166] = {
	{2046,-40},
	{2036,-39},
	{2026,-38},
	{2015,-37},
	{2003,-36},
	{1993,-35},
	{1980,-34},
	{1967,-33},
	{1954,-32},
	{1940,-31},
	{1927,-30},
	{1912,-29},
	{1897,-28},
	{1881,-27},
	{1865,-26},
	{1850,-25},
	{1833,-24},
	{1816,-23},
	{1798,-22},
	{1780,-21},
	{1764,-20},
	{1745,-19},
	{1726,-18},
	{1707,-17},
	{1687,-16},
	{1670,-15},
	{1650,-14},
	{1631,-13},
	{1611,-12},
	{1591,-11},
	{1573,-10},
	{1553,-9},
	{1534,-8},
	{1514,-7},
	{1495,-6},
	{1477,-5},
	{1458,-4},
	{1439,-3},
	{1420,-2},
	{1402,-1},
	{1385,0},
	{1368,1},
	{1350,2},
	{1333,3},
	{1316,4},
	{1301,5},
	{1285,6},
	{1269,7},
	{1254,8},
	{1239,9},
	{1225,10},
	{1211,11},
	{1197,12},
	{1184,13},
	{1171,14},
	{1159,15},
	{1147,16},
	{1135,17},
	{1124,18},
	{1113,19},
	{1102,20},
	{1092,21},
	{1082,22},
	{1073,23},
	{1064,24},
	{1055,25},
	{1046,26},
	{1038,27},
	{1030,28},
	{1022,29},
	{1015,30},
	{1008,31},
	{1001,32},
	{995,33},
	{988,34},
	{982,35},
	{976,36},
	{971,37},
	{966,38},
	{960,39},
	{955,40},
	{951,41},
	{946,42},
	{942,43},
	{937,44},
	{933,45},
	{929,46},
	{926,47},
	{922,48},
	{919,49},
	{915,50},
	{912,51},
	{909,52},
	{906,53},
	{903,54},
	{901,55},
	{898,56},
	{896,57},
	{893,58},
	{891,59},
	{889,60},
	{886,61},
	{884,62},
	{882,63},
	{881,64},
	{879,65},
	{877,66},
	{875,67},
	{874,68},
	{872,69},
	{871,70},
	{869,71},
	{868,72},
	{867,73},
	{865,74},
	{864,75},
	{863,76},
	{862,77},
	{861,78},
	{860,79},
	{858,80},
	{857,81},
	{857,82},
	{856,83},
	{855,84},
	{854,85},
	{853,86},
	{852,87},
	{852,88},
	{851,89},
	{850,90},
	{849,91},
	{849,92},
	{848,93},
	{848,94},
	{847,95},
	{846,96},
	{846,97},
	{845,98},
	{845,99},
	{844,100},
	{844,101},
	{844,102},
	{843,103},
	{843,104},
	{842,105},
	{842,106},
	{841,107},
	{841,108},
	{841,109},
	{840,110},
	{840,111},
	{840,112},
	{839,113},
	{839,114},
	{839,115},
	{839,116},
	{838,117},
	{838,118},
	{838,119},
	{837,120},
	{837,121},
	{837,122},
	{837,123},
	{837,124},
	{836,125},
};
#else
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
const struct adc_map_pt adcmap_batttherm[THERM_LAST] = {
#if 1 // from AT&T 2011-05-31
	{1500,   -8},
	{1430,   -3},
#else
	{1574,  -10},
	{1477,   -5},
#endif
	{946,	 42},
	{933,	 45},
	{901,	 55},
	{896,	 57},
	{889,	 60},
	{878,	 64},
};
#else	/* original codes(JB codes) */
static const struct adc_map_pt adcmap_batttherm[] = {
	{2020,	-30},
	{1923,	-20},
	{1796,	-10},
	{1640,	  0},
	{1459,	 10},
	{1260,	 20},
	{1159,	 25},
	{1059,	 30},
	{871,	 40},
	{706,	 50},
	{567,	 60},
	{453,	 70},
	{364,	 80}
};
#endif
#endif	/*                                        */

static const struct adc_map_pt adcmap_msmtherm[] = {
	{2150,	-30},
	{2107,	-20},
	{2037,	-10},
	{1929,	  0},
	{1776,	 10},
	{1579,	 20},
	{1467,	 25},
	{1349,	 30},
	{1108,	 40},
	{878,	 50},
	{677,	 60},
	{513,	 70},
	{385,	 80},
	{287,	 90},
	{215,	100},
	{186,	110},
	{107,	120}
};

static const struct adc_map_pt adcmap_ntcg104ef104fb[] = {
	{696483,	-40960},
	{649148,	-39936},
	{605368,	-38912},
	{564809,	-37888},
	{527215,	-36864},
	{492322,	-35840},
	{460007,	-34816},
	{429982,	-33792},
	{402099,	-32768},
	{376192,	-31744},
	{352075,	-30720},
	{329714,	-29696},
	{308876,	-28672},
	{289480,	-27648},
	{271417,	-26624},
	{254574,	-25600},
	{238903,	-24576},
	{224276,	-23552},
	{210631,	-22528},
	{197896,	-21504},
	{186007,	-20480},
	{174899,	-19456},
	{164521,	-18432},
	{154818,	-17408},
	{145744,	-16384},
	{137265,	-15360},
	{129307,	-14336},
	{121866,	-13312},
	{114896,	-12288},
	{108365,	-11264},
	{102252,	-10240},
	{96499,		-9216},
	{91111,		-8192},
	{86055,		-7168},
	{81308,		-6144},
	{76857,		-5120},
	{72660,		-4096},
	{68722,		-3072},
	{65020,		-2048},
	{61538,		-1024},
	{58261,		0},
	{55177,		1024},
	{52274,		2048},
	{49538,		3072},
	{46962,		4096},
	{44531,		5120},
	{42243,		6144},
	{40083,		7168},
	{38045,		8192},
	{36122,		9216},
	{34308,		10240},
	{32592,		11264},
	{30972,		12288},
	{29442,		13312},
	{27995,		14336},
	{26624,		15360},
	{25333,		16384},
	{24109,		17408},
	{22951,		18432},
	{21854,		19456},
	{20807,		20480},
	{19831,		21504},
	{18899,		22528},
	{18016,		23552},
	{17178,		24576},
	{16384,		25600},
	{15631,		26624},
	{14916,		27648},
	{14237,		28672},
	{13593,		29696},
	{12976,		30720},
	{12400,		31744},
	{11848,		32768},
	{11324,		33792},
	{10825,		34816},
	{10354,		35840},
	{9900,		36864},
	{9471,		37888},
	{9062,		38912},
	{8674,		39936},
	{8306,		40960},
	{7951,		41984},
	{7616,		43008},
	{7296,		44032},
	{6991,		45056},
	{6701,		46080},
	{6424,		47104},
	{6160,		48128},
	{5908,		49152},
	{5667,		50176},
	{5439,		51200},
	{5219,		52224},
	{5010,		53248},
	{4810,		54272},
	{4619,		55296},
	{4440,		56320},
	{4263,		57344},
	{4097,		58368},
	{3938,		59392},
	{3785,		60416},
	{3637,		61440},
	{3501,		62464},
	{3368,		63488},
	{3240,		64512},
	{3118,		65536},
	{2998,		66560},
	{2889,		67584},
	{2782,		68608},
	{2680,		69632},
	{2581,		70656},
	{2490,		71680},
	{2397,		72704},
	{2310,		73728},
	{2227,		74752},
	{2147,		75776},
	{2064,		76800},
	{1998,		77824},
	{1927,		78848},
	{1860,		79872},
	{1795,		80896},
	{1736,		81920},
	{1673,		82944},
	{1615,		83968},
	{1560,		84992},
	{1507,		86016},
	{1456,		87040},
	{1407,		88064},
	{1360,		89088},
	{1314,		90112},
	{1271,		91136},
	{1228,		92160},
	{1189,		93184},
	{1150,		94208},
	{1112,		95232},
	{1076,		96256},
	{1042,		97280},
	{1008,		98304},
	{976,		99328},
	{945,		100352},
	{915,		101376},
	{886,		102400},
	{859,		103424},
	{832,		104448},
	{807,		105472},
	{782,		106496},
	{756,		107520},
	{735,		108544},
	{712,		109568},
	{691,		110592},
	{670,		111616},
	{650,		112640},
	{631,		113664},
	{612,		114688},
	{594,		115712},
	{577,		116736},
	{560,		117760},
	{544,		118784},
	{528,		119808},
	{513,		120832},
	{498,		121856},
	{483,		122880},
	{470,		123904},
	{457,		124928},
	{444,		125952},
	{431,		126976},
	{419,		128000}
};

static int32_t
	adc_map_linear(const struct adc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if ((pts == NULL) || (output == NULL))
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else
			i++;
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
	/* result is between search_index and search_index-1 */
	/* interpolate linearly */
	*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

int32_t scale_default(int32_t adc_code,
				const struct adc_properties *adc_properties,
				const struct chan_properties *chan_properties,
				struct adc_chan_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0;
	int32_t rawfromoffset = adc_code - chan_properties->adc_graph->offset;

	if (!chan_properties->gain_numerator ||
		!chan_properties->gain_denominator)
		return -EINVAL;

	adc_chan_result->adc_code = adc_code;
	if (rawfromoffset < 0) {
		if (adc_properties->bipolar) {
			rawfromoffset = (rawfromoffset ^ -1) +  1;
			negative_rawfromoffset = 1;
		} else
			rawfromoffset = 0;
	}

	if (rawfromoffset >= 1 << adc_properties->bitresolution)
		rawfromoffset = (1 << adc_properties->bitresolution) - 1;

	adc_chan_result->measurement = (int64_t)rawfromoffset*
					chan_properties->adc_graph->dx*
					chan_properties->gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement, chan_properties->adc_graph->dy*
					chan_properties->gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement =
		(adc_chan_result->measurement ^ -1) + 1;

	/* Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input */
	adc_chan_result->physical = (int32_t) adc_chan_result->measurement;

	return 0;
}

int32_t scale_batt_therm(int32_t adc_code,
				const struct adc_properties *adc_properties,
				const struct chan_properties *chan_properties,
				struct adc_chan_result *adc_chan_result)
{
	scale_default(adc_code, adc_properties, chan_properties,
			adc_chan_result);
	/* convert mV ---> degC using the table */
	return adc_map_linear(
			adcmap_batttherm,
			sizeof(adcmap_batttherm)/sizeof(adcmap_batttherm[0]),
			adc_chan_result->physical,
			&adc_chan_result->physical);
}

int32_t scale_msm_therm(int32_t adc_code,
		const struct adc_properties *adc_properties,
		const struct chan_properties *chan_properties,
		struct adc_chan_result *adc_chan_result)
{
	scale_default(adc_code, adc_properties, chan_properties,
			adc_chan_result);
	/* convert mV ---> degC using the table */
	return adc_map_linear(
			adcmap_msmtherm,
			sizeof(adcmap_msmtherm)/sizeof(adcmap_msmtherm[0]),
			adc_chan_result->physical,
			&adc_chan_result->physical);
}

int32_t scale_pmic_therm(int32_t adc_code,
				const struct adc_properties *adc_properties,
				const struct chan_properties *chan_properties,
				struct adc_chan_result *adc_chan_result)
{
	/* 2mV/K */
	int32_t rawfromoffset = adc_code - chan_properties->adc_graph->offset;

	if (!chan_properties->gain_numerator ||
		!chan_properties->gain_denominator)
		return -EINVAL;

	adc_chan_result->adc_code = adc_code;
	if (rawfromoffset > 0) {
		if (rawfromoffset >= 1 << adc_properties->bitresolution)
			rawfromoffset = (1 << adc_properties->bitresolution)
									- 1;
		adc_chan_result->measurement = (int64_t)rawfromoffset*
					chan_properties->adc_graph->dx*
					chan_properties->gain_denominator*1000;
		do_div(adc_chan_result->measurement,
			chan_properties->adc_graph->dy*
			chan_properties->gain_numerator*2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Note: adc_chan_result->measurement is in the unit of
		adc_properties.adc_reference */
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;
	/* Change to .001 deg C */
	adc_chan_result->physical -= KELVINMIL_DEGMIL;
	adc_chan_result->measurement <<= 1;

	return 0;
}

/* Scales the ADC code to 0.001 degrees C using the map
 * table for the XO thermistor.
 */
int32_t tdkntcgtherm(int32_t adc_code,
			const struct adc_properties *adc_properties,
			const struct chan_properties *chan_properties,
			struct adc_chan_result *adc_chan_result)
{
	int32_t offset = chan_properties->adc_graph->offset,
		dy = chan_properties->adc_graph->dy,
		dx = chan_properties->adc_graph->dx,
		fullscale_calibrated_adc_code;

	uint32_t rt_r25;
	uint32_t num1, num2, denom;

	adc_chan_result->adc_code = adc_code;
	fullscale_calibrated_adc_code = dy + offset;
	/* The above is a short cut in math that would reduce a lot of
	   computation whereas the below expression
		(adc_properties->adc_reference*dy+dx*offset+(dx>>1))/dx
	   is a more generic formula when the 2 reference voltages are
	   different than 0 and full scale voltage. */

	if ((dy == 0) || (dx == 0) ||
			(offset >= fullscale_calibrated_adc_code)) {
		return -EINVAL;
	} else {
		if (adc_code >= fullscale_calibrated_adc_code) {
			rt_r25 = (uint32_t)-1;
		} else if (adc_code <= offset) {
			rt_r25 = 0;
		} else {
	/* The formula used is (adc_code of current reading - offset)/
	 * (the calibrated fullscale adc code - adc_code of current reading).
	 * For this channel, at this time, chan_properties->gain_numerator =
	 * chan_properties->gain_denominator = 1, so no need to incorporate
	 * into the formula even though we could and multiply/divide by 1
	 * which yields the same result but expensive on computation. */
		num1 = (adc_code - offset) << 14;
		num2 = (fullscale_calibrated_adc_code - adc_code) >> 1;
		denom = fullscale_calibrated_adc_code - adc_code;

			if ((int)denom <= 0)
				rt_r25 = 0x7FFFFFFF;
			else
				rt_r25 = (num1 + num2) / denom;
		}

		if (rt_r25 > 0x7FFFFFFF)
			rt_r25 = 0x7FFFFFFF;

		adc_map_linear(adcmap_ntcg104ef104fb,
		sizeof(adcmap_ntcg104ef104fb)/sizeof(adcmap_ntcg104ef104fb[0]),
		(int32_t)rt_r25, &adc_chan_result->physical);
	}

	return 0;
}

int32_t scale_xtern_chgr_cur(int32_t adc_code,
			const struct adc_properties *adc_properties,
			const struct chan_properties *chan_properties,
			struct adc_chan_result *adc_chan_result)
{
	int32_t rawfromoffset = adc_code - chan_properties->adc_graph->offset;

	if (!chan_properties->gain_numerator ||
		!chan_properties->gain_denominator)
		return -EINVAL;

	adc_chan_result->adc_code = adc_code;
	if (rawfromoffset > 0) {
		if (rawfromoffset >= 1 << adc_properties->bitresolution)
			rawfromoffset = (1 << adc_properties->bitresolution)
									- 1;
		adc_chan_result->measurement = ((int64_t)rawfromoffset * 5)*
						chan_properties->adc_graph->dx*
					chan_properties->gain_denominator;
		do_div(adc_chan_result->measurement,
					chan_properties->adc_graph->dy*
					chan_properties->gain_numerator);
	} else {
		adc_chan_result->measurement = 0;
	}
	adc_chan_result->physical = (int32_t) adc_chan_result->measurement;

	return 0;
}
