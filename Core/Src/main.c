/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SSD1331.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PI 3.14159265

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
//zmienne do przerwań
static uint8_t waveformCounter = 0; //zmienna do zmiany kształtu przebiegu
static uint8_t multiply = 0; //mnożnik częstotliwości

//zmienne dla adc
uint16_t adc_value = 0;
float voltage_conversion_num = 0;
float voltage_conversion_denum = 0;
float maximum_num = 0;
float maximum_denum = 0;
float nearest = 0.0;
const float supply_voltage = 3.0;
const float adc_resolution = 4095.0;
uint16_t i, j;
static uint8_t adc_counter = 0;
int d, adc_array_num[10], adc_array_denum[10], location;
uint8_t size = 10;

uint16_t changed_freq = 0;
int actual_freq = 1;
int last_freq = 0;

uint32_t switch_waveform_table[1024];

uint32_t squareTable[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void freq_change(uint8_t multiply);
int find_max(int[], int);

const uint32_t sineTable[1024] = {2048,2060,2073,2085,2098,2110,2123,2136,2148,2161,
		2173,2186,2198,2211,2223,2236,2248,2261,2273,2286,
		2298,2311,2323,2336,2348,2361,2373,2385,2398,2410,
		2423,2435,2447,2460,2472,2484,2497,2509,2521,2533,
		2545,2558,2570,2582,2594,2606,2618,2630,2642,2654,
		2666,2678,2690,2702,2714,2726,2738,2750,2762,2773,
		2785,2797,2808,2820,2832,2843,2855,2866,2878,2889,
		2901,2912,2924,2935,2946,2958,2969,2980,2991,3002,
		3014,3025,3036,3047,3058,3069,3079,3090,3101,3112,
		3123,3133,3144,3154,3165,3176,3186,3196,3207,3217,
		3227,3238,3248,3258,3268,3278,3288,3298,3308,3318,
		3328,3338,3347,3357,3367,3376,3386,3395,3405,3414,
		3424,3433,3442,3451,3460,3470,3479,3487,3496,3505,
		3514,3523,3532,3540,3549,3557,3566,3574,3582,3591,
		3599,3607,3615,3623,3631,3639,3647,3655,3663,3670,
		3678,3686,3693,3701,3708,3715,3723,3730,3737,3744,
		3751,3758,3765,3772,3778,3785,3792,3798,3805,3811,
		3818,3824,3830,3836,3842,3848,3854,3860,3866,3872,
		3877,3883,3889,3894,3899,3905,3910,3915,3920,3925,
		3930,3935,3940,3945,3949,3954,3959,3963,3968,3972,
		3976,3980,3984,3988,3992,3996,4000,4004,4008,4011,
		4015,4018,4022,4025,4028,4031,4034,4037,4040,4043,
		4046,4049,4051,4054,4056,4059,4061,4063,4065,4067,
		4069,4071,4073,4075,4077,4078,4080,4081,4083,4084,
		4085,4087,4088,4089,4090,4091,4091,4092,4093,4093,
		4094,4094,4094,4095,4095,4095,4095,4095,4095,4095,
		4094,4094,4093,4093,4092,4092,4091,4090,4089,4088,
		4087,4086,4085,4084,4082,4081,4079,4078,4076,4074,
		4072,4070,4068,4066,4064,4062,4060,4057,4055,4052,
		4050,4047,4044,4042,4039,4036,4033,4030,4026,4023,
		4020,4016,4013,4009,4006,4002,3998,3994,3990,3986,
		3982,3978,3974,3970,3965,3961,3956,3952,3947,3942,
		3938,3933,3928,3923,3918,3913,3907,3902,3897,3891,
		3886,3880,3875,3869,3863,3857,3851,3845,3839,3833,
		3827,3821,3814,3808,3802,3795,3788,3782,3775,3768,
		3761,3755,3748,3740,3733,3726,3719,3712,3704,3697,
		3689,3682,3674,3667,3659,3651,3643,3635,3627,3619,
		3611,3603,3595,3587,3578,3570,3561,3553,3544,3536,
		3527,3518,3510,3501,3492,3483,3474,3465,3456,3447,
		3438,3428,3419,3410,3400,3391,3381,3372,3362,3352,
		3343,3333,3323,3313,3303,3293,3283,3273,3263,3253,
		3243,3233,3222,3212,3202,3191,3181,3170,3160,3149,
		3139,3128,3117,3106,3096,3085,3074,3063,3052,3041,
		3030,3019,3008,2997,2986,2975,2963,2952,2941,2929,
		2918,2907,2895,2884,2872,2861,2849,2838,2826,2814,
		2803,2791,2779,2767,2756,2744,2732,2720,2708,2696,
		2684,2672,2660,2648,2636,2624,2612,2600,2588,2576,
		2564,2552,2539,2527,2515,2503,2490,2478,2466,2453,
		2441,2429,2416,2404,2392,2379,2367,2354,2342,2330,
		2317,2305,2292,2280,2267,2255,2242,2230,2217,2205,
		2192,2179,2167,2154,2142,2129,2117,2104,2092,2079,
		2066,2054,2041,2029,2016,2003,1991,1978,1966,1953,
		1941,1928,1916,1903,1890,1878,1865,1853,1840,1828,
		1815,1803,1790,1778,1765,1753,1741,1728,1716,1703,
		1691,1679,1666,1654,1642,1629,1617,1605,1592,1580,
		1568,1556,1543,1531,1519,1507,1495,1483,1471,1459,
		1447,1435,1423,1411,1399,1387,1375,1363,1351,1339,
		1328,1316,1304,1292,1281,1269,1257,1246,1234,1223,
		1211,1200,1188,1177,1166,1154,1143,1132,1120,1109,
		1098,1087,1076,1065,1054,1043,1032,1021,1010,999,
		989,978,967,956,946,935,925,914,904,893,
		883,873,862,852,842,832,822,812,802,792,
		782,772,762,752,743,733,723,714,704,695,
		685,676,667,657,648,639,630,621,612,603,
		594,585,577,568,559,551,542,534,525,517,
		508,500,492,484,476,468,460,452,444,436,
		428,421,413,406,398,391,383,376,369,362,
		355,347,340,334,327,320,313,307,300,293,
		287,281,274,268,262,256,250,244,238,232,
		226,220,215,209,204,198,193,188,182,177,
		172,167,162,157,153,148,143,139,134,130,
		125,121,117,113,109,105,101,97,93,89,
		86,82,79,75,72,69,65,62,59,56,
		53,51,48,45,43,40,38,35,33,31,
		29,27,25,23,21,19,17,16,14,13,
		11,10,9,8,7,6,5,4,3,3,
		2,2,1,1,0,0,0,0,0,0,
		0,1,1,1,2,2,3,4,4,5,
		6,7,8,10,11,12,14,15,17,18,
		20,22,24,26,28,30,32,34,36,39,
		41,44,46,49,52,55,58,61,64,67,
		70,73,77,80,84,87,91,95,99,103,
		107,111,115,119,123,127,132,136,141,146,
		150,155,160,165,170,175,180,185,190,196,
		201,206,212,218,223,229,235,241,247,253,
		259,265,271,277,284,290,297,303,310,317,
		323,330,337,344,351,358,365,372,380,387,
		394,402,409,417,425,432,440,448,456,464,
		472,480,488,496,504,513,521,529,538,546,
		555,563,572,581,590,599,608,616,625,635,
		644,653,662,671,681,690,700,709,719,728,
		738,748,757,767,777,787,797,807,817,827,
		837,847,857,868,878,888,899,909,919,930,
		941,951,962,972,983,994,1005,1016,1026,1037,
		1048,1059,1070,1081,1093,1104,1115,1126,1137,1149,
		1160,1171,1183,1194,1206,1217,1229,1240,1252,1263,
		1275,1287,1298,1310,1322,1333,1345,1357,1369,1381,
		1393,1405,1417,1429,1441,1453,1465,1477,1489,1501,
		1513,1525,1537,1550,1562,1574,1586,1598,1611,1623,
		1635,1648,1660,1672,1685,1697,1710,1722,1734,1747,
		1759,1772,1784,1797,1809,1822,1834,1847,1859,1872,
		1884,1897,1909,1922,1934,1947,1959,1972,1985,1997,
		2010,2022,2035,2048};

const uint32_t triangleTable[1024] = {8,16,24,32,40,48,56,64,72,80,
		88,96,104,112,120,128,136,144,152,160,
		168,176,184,192,200,208,216,224,232,240,
		248,256,264,272,280,288,296,304,312,320,
		328,336,344,352,360,368,376,384,392,400,
		408,416,424,432,440,448,456,464,472,480,
		488,496,504,512,520,528,536,544,552,560,
		568,576,584,592,600,608,616,624,632,640,
		648,656,664,672,680,689,697,705,713,721,
		729,737,745,753,761,769,777,785,793,801,
		809,817,825,833,841,849,857,865,873,881,
		889,897,905,913,921,929,937,945,953,961,
		969,977,985,993,1001,1009,1017,1025,1033,1041,
		1049,1057,1065,1073,1081,1089,1097,1105,1113,1121,
		1129,1137,1145,1153,1161,1169,1177,1185,1193,1201,
		1209,1217,1225,1233,1241,1249,1257,1265,1273,1281,
		1289,1297,1305,1313,1321,1329,1337,1345,1353,1361,
		1369,1377,1385,1393,1401,1409,1417,1425,1433,1441,
		1449,1457,1465,1473,1481,1489,1497,1505,1513,1521,
		1529,1537,1545,1553,1561,1569,1577,1585,1593,1601,
		1609,1617,1625,1633,1641,1649,1657,1665,1673,1681,
		1689,1697,1705,1713,1721,1729,1737,1745,1753,1761,
		1769,1777,1785,1793,1801,1809,1817,1825,1833,1841,
		1849,1857,1865,1873,1881,1889,1897,1905,1913,1921,
		1929,1937,1945,1953,1961,1969,1977,1985,1993,2001,
		2009,2017,2025,2033,2041,2050,2058,2066,2074,2082,
		2090,2098,2106,2114,2122,2130,2138,2146,2154,2162,
		2170,2178,2186,2194,2202,2210,2218,2226,2234,2242,
		2250,2258,2266,2274,2282,2290,2298,2306,2314,2322,
		2330,2338,2346,2354,2362,2370,2378,2386,2394,2402,
		2410,2418,2426,2434,2442,2450,2458,2466,2474,2482,
		2490,2498,2506,2514,2522,2530,2538,2546,2554,2562,
		2570,2578,2586,2594,2602,2610,2618,2626,2634,2642,
		2650,2658,2666,2674,2682,2690,2698,2706,2714,2722,
		2730,2738,2746,2754,2762,2770,2778,2786,2794,2802,
		2810,2818,2826,2834,2842,2850,2858,2866,2874,2882,
		2890,2898,2906,2914,2922,2930,2938,2946,2954,2962,
		2970,2978,2986,2994,3002,3010,3018,3026,3034,3042,
		3050,3058,3066,3074,3082,3090,3098,3106,3114,3122,
		3130,3138,3146,3154,3162,3170,3178,3186,3194,3202,
		3210,3218,3226,3234,3242,3250,3258,3266,3274,3282,
		3290,3298,3306,3314,3322,3330,3338,3346,3354,3362,
		3370,3378,3386,3394,3402,3410,3419,3427,3435,3443,
		3451,3459,3467,3475,3483,3491,3499,3507,3515,3523,
		3531,3539,3547,3555,3563,3571,3579,3587,3595,3603,
		3611,3619,3627,3635,3643,3651,3659,3667,3675,3683,
		3691,3699,3707,3715,3723,3731,3739,3747,3755,3763,
		3771,3779,3787,3795,3803,3811,3819,3827,3835,3843,
		3851,3859,3867,3875,3883,3891,3899,3907,3915,3923,
		3931,3939,3947,3955,3963,3971,3979,3987,3995,4003,
		4011,4019,4027,4035,4043,4051,4059,4067,4075,4083,
		4091,4095,4091,4083,4075,4067,4059,4051,4043,4035,
		4027,4019,4011,4003,3995,3987,3979,3971,3963,3955,
		3947,3939,3931,3923,3915,3907,3899,3891,3883,3875,
		3867,3859,3851,3843,3835,3827,3819,3811,3803,3795,
		3787,3779,3771,3763,3755,3747,3739,3731,3723,3715,
		3707,3699,3691,3683,3675,3667,3659,3651,3643,3635,
		3627,3619,3611,3603,3595,3587,3579,3571,3563,3555,
		3547,3539,3531,3523,3515,3507,3499,3491,3483,3475,
		3467,3459,3451,3443,3435,3427,3419,3410,3402,3394,
		3386,3378,3370,3362,3354,3346,3338,3330,3322,3314,
		3306,3298,3290,3282,3274,3266,3258,3250,3242,3234,
		3226,3218,3210,3202,3194,3186,3178,3170,3162,3154,
		3146,3138,3130,3122,3114,3106,3098,3090,3082,3074,
		3066,3058,3050,3042,3034,3026,3018,3010,3002,2994,
		2986,2978,2970,2962,2954,2946,2938,2930,2922,2914,
		2906,2898,2890,2882,2874,2866,2858,2850,2842,2834,
		2826,2818,2810,2802,2794,2786,2778,2770,2762,2754,
		2746,2738,2730,2722,2714,2706,2698,2690,2682,2674,
		2666,2658,2650,2642,2634,2626,2618,2610,2602,2594,
		2586,2578,2570,2562,2554,2546,2538,2530,2522,2514,
		2506,2498,2490,2482,2474,2466,2458,2450,2442,2434,
		2426,2418,2410,2402,2394,2386,2378,2370,2362,2354,
		2346,2338,2330,2322,2314,2306,2298,2290,2282,2274,
		2266,2258,2250,2242,2234,2226,2218,2210,2202,2194,
		2186,2178,2170,2162,2154,2146,2138,2130,2122,2114,
		2106,2098,2090,2082,2074,2066,2058,2050,2041,2033,
		2025,2017,2009,2001,1993,1985,1977,1969,1961,1953,
		1945,1937,1929,1921,1913,1905,1897,1889,1881,1873,
		1865,1857,1849,1841,1833,1825,1817,1809,1801,1793,
		1785,1777,1769,1761,1753,1745,1737,1729,1721,1713,
		1705,1697,1689,1681,1673,1665,1657,1649,1641,1633,
		1625,1617,1609,1601,1593,1585,1577,1569,1561,1553,
		1545,1537,1529,1521,1513,1505,1497,1489,1481,1473,
		1465,1457,1449,1441,1433,1425,1417,1409,1401,1393,
		1385,1377,1369,1361,1353,1345,1337,1329,1321,1313,
		1305,1297,1289,1281,1273,1265,1257,1249,1241,1233,
		1225,1217,1209,1201,1193,1185,1177,1169,1161,1153,
		1145,1137,1129,1121,1113,1105,1097,1089,1081,1073,
		1065,1057,1049,1041,1033,1025,1017,1009,1001,993,
		985,977,969,961,953,945,937,929,921,913,
		905,897,889,881,873,865,857,849,841,833,
		825,817,809,801,793,785,777,769,761,753,
		745,737,729,721,713,705,697,689,680,672,
		664,656,648,640,632,624,616,608,600,592,
		584,576,568,560,552,544,536,528,520,512,
		504,496,488,480,472,464,456,448,440,432,
		424,416,408,400,392,384,376,368,360,352,
		344,336,328,320,312,304,296,288,280,272,
		264,256,248,240,232,224,216,208,200,192,
		184,176,168,160,152,144,136,128,120,112,
		104,96,88,80,72,64,56,48,40,32,
		24,16,8,0};

void square_fill()
{
	for(int i = 0 ; i<1024; i++)
	{
		if(i<512){
			squareTable[i] = 0xfff; //4095
		}
		else
			squareTable[i] = 0;
	}
}

int find_max(int a[], int n) {
	int index = 0;

	for (int d = 1; d < n; d++)
		    if (a[d] > a[index])
		      index = d;

	return index;
}

//procedura obsługi przerwania dla przycisków od wybrania przebiegu oraz zmiany mnożnika częstotliwości
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin == SWITCH_WAVEFORM_Pin){
			//generacja przebiegu sinusoidalnego
			if(waveformCounter == 0){
				ssd1331_clear_shape(BLACK);
				for (uint16_t i = 0; i < 1024; i++){
					switch_waveform_table[i] = sineTable[i];
					ssd1331_display_string(30, 45, (const uint8_t*)"SINE", FONT_1608, GREEN);
				}
			}
			//generacja przebiegu prostokątnego
			else if(waveformCounter == 1){
				ssd1331_clear_shape(BLACK);
				for (uint16_t i = 0; i < 1024; i++){
					switch_waveform_table[i] = squareTable[i];
					ssd1331_display_string(30, 45, (const uint8_t*)"SQUARE", FONT_1608, GREEN);
				}
			}
			//generacja przebiegu trójkątnego
			else if(waveformCounter == 2){
				ssd1331_clear_shape(BLACK);
				for (uint16_t i = 0; i < 1024; i++){
					switch_waveform_table[i] = triangleTable[i];
					ssd1331_display_string(30, 45, (const uint8_t*)"TRIANGLE", FONT_1608, GREEN);
				}
			}

			waveformCounter++;
			if(waveformCounter > 2){
				waveformCounter = 0;
			}
		}

		if(GPIO_Pin == SWITCH_MULTIPLY_Pin){
			if(multiply == 0){
				ssd1331_clear_freq(BLACK);
				ssd1331_clear_multiply(BLACK);
				freq_change(multiply); //zmiana nastaw timera6 -> zmiana częstotliwości przebiegu
				ssd1331_display_num(40, 15, 1, 5, FONT_1608, GREEN);
				ssd1331_display_num(50, 30, multiply, 1 , FONT_1608, GREEN);
			}
			else if(multiply == 1){
				ssd1331_clear_freq(BLACK);
				ssd1331_clear_multiply(BLACK);
				freq_change(multiply); //zmiana nastaw timera6 -> zmiana częstotliwości przebiegu
				ssd1331_display_num(40, 15, 10, 5, FONT_1608, GREEN);
				ssd1331_display_num(50, 30, multiply, 1 , FONT_1608, GREEN);
			}
			else if(multiply == 2){
				ssd1331_clear_freq(BLACK);
				ssd1331_clear_multiply(BLACK);
				freq_change(multiply); //zmiana nastaw timera6 -> zmiana częstotliwości przebiegu
				ssd1331_display_num(40, 15, 100, 5, FONT_1608, GREEN);
				ssd1331_display_num(50, 30, multiply, 1 , FONT_1608, GREEN);
			}
			else if(multiply == 3){
				ssd1331_clear_freq(BLACK);
				ssd1331_clear_multiply(BLACK);
				freq_change(multiply); //zmiana nastaw timera6 -> zmiana częstotliwości przebiegu
				ssd1331_display_num(40, 15, 1000, 5, FONT_1608, GREEN);
				ssd1331_display_num(50, 30, multiply, 1 , FONT_1608, GREEN);
			}
			multiply++;
			if(multiply == 4){
				multiply = 0;
			}
		}
		if(GPIO_Pin == VOLTAGE_MEASURE_Pin){
			ssd1331_clear_amp(BLACK);

			if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) { // Oczekiwanie na zakonczenie konwersji
				adc_value = HAL_ADC_GetValue(&hadc1);// Pobranie zmierzonej wartosci
				voltage_conversion_num = (supply_voltage*adc_value)/adc_resolution;// Przeliczenie wartosci zmierzonej na napiecie
				nearest = roundf(voltage_conversion_num * 100) / 100;
				voltage_conversion_denum = (uint16_t)(nearest*100) % 100;
				HAL_ADC_Start(&hadc1);// Rozpoczecie nowej konwersji
			}

			adc_array_num[adc_counter] = voltage_conversion_num;
			adc_array_denum[adc_counter] = voltage_conversion_denum;

			adc_counter++;
			if(adc_counter == 10){
				adc_counter = 0;

				location = find_max(adc_array_num, size);
				maximum_num  = adc_array_num[location];

				location = find_max(adc_array_denum, size);
				maximum_denum  = adc_array_denum[location];

				ssd1331_display_num(32, 0, maximum_num, 2 , FONT_1608, GREEN);
				ssd1331_display_char(48, 0, '.', FONT_1608, GREEN);
				ssd1331_display_num(51, 0, maximum_denum, 2 , FONT_1608, GREEN);
				ssd1331_display_char(70, 0, 'V', FONT_1608, GREEN);

				for(int i=0; i<10; i++){
					adc_array_num[i] = 0;
					adc_array_denum[i] = 0;
				}

			}
		}

}

void freq_change(uint8_t multiply)
{
	switch (multiply)
	{
	case 0: //1 Hz
		TIM6->PSC = 6;
		TIM6->ARR = 10085;
		TIM6->CNT = 0;
		break;
	case 1: //10 Hz
		TIM6->PSC = 7;
		TIM6->ARR = 878;
		TIM6->CNT = 0;
		break;
	case 2: //100 Hz
		TIM6->PSC = 7;
		TIM6->ARR = 87;
		TIM6->CNT = 0;
		break;
	case 3: //1000 Hz
		TIM6->PSC = 6;
		TIM6->ARR = 9;
		TIM6->CNT = 0;
		break;
	default:
		break;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  square_fill();

  HAL_TIM_Base_Start(&htim6); //uruchomienie timer6 od DAC

  ssd1331_init();
  ssd1331_clear_screen(BLACK);
  ssd1331_display_string(0, 0, (const uint8_t*)"VPP: ", FONT_1608, GREEN);
  ssd1331_display_string(0, 15, (const uint8_t*)"FREQ: ", FONT_1608, GREEN);
  ssd1331_display_string(0, 30, (const uint8_t*)"MUL: ", FONT_1608, GREEN);
  ssd1331_display_string(0, 45, (const uint8_t*)"SW: ", FONT_1608, GREEN);
  ssd1331_display_string(80, 15, (const uint8_t*)"Hz ", FONT_1608, GREEN);

  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)switch_waveform_table, 1023, DAC_ALIGN_12B_R); //generacja określonego przebiegu o określonej częstotliwości

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VOLTAGE_MEASURE_Pin SWITCH_MULTIPLY_Pin */
  GPIO_InitStruct.Pin = VOLTAGE_MEASURE_Pin|SWITCH_MULTIPLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_Pin */
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_Pin */
  GPIO_InitStruct.Pin = RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_WAVEFORM_Pin */
  GPIO_InitStruct.Pin = SWITCH_WAVEFORM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_WAVEFORM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
