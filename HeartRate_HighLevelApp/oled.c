/***************************************************************************************************
   Name: OLED.c
   Sphere OS: 19.05
****************************************************************************************************/

#include "oled.h"
#include <math.h>
#include <applibs/log.h>
#include "hr4.h"

//#define DISABLE_OLED

// externs to access data from other files
extern network_var network_data;
extern bool RTCore_status;
extern int i2cFd;
uint8_t oled_state;

uint8_t rtOledMsg[6][RTAPP_MAX_OLED_MSG_SIZE + 1] = {
	// Title font can display 10 characters max
	//"123456789A"
	{"RTApp Disp"},
	// Lines 1 - 5 can display 21 characters max
   //"123456789A123456789A12"
	{"     Azure Sphere   "},
	{"     This screen    "},
	{"    can be driven   "},
	{"      from a        "},
	{"   RTApplication    "}
};

/**
  * @brief  OLED initialization.
  * @param  None.
  * @retval Positive if was unsuccefully, zero if was succefully.
  */
uint8_t oled_init()
{
	return sd1306_init();
}

// State machine to change the OLED status
void update_oled()
{
#ifndef DISABLE_OLED

	uint8_t string_data[10];

	switch (oled_state)
	{
		case BUS_STATUS:
			oled_i2c_bus_status(I2C_INIT);
			Log_Debug("BUS_STATUS Screen\n");
			break;
		case RTAPP_MESSAGE:
		{
			clear_oled_buffer();
			
			for (int i = 0; i < 6; i++) {
				if (i == 0) {
					sd1306_draw_string(OLED_TITLE_X, OLED_TITLE_Y, rtOledMsg[i], FONT_SIZE_TITLE, white_pixel);
				}
				else {
					sd1306_draw_string(0, (OLED_LINE_1_Y + ((i-1) * 10)), rtOledMsg[i], FONT_SIZE_LINE, white_pixel);
				}
			}

			sd1306_refresh();
			break;
		}
		case NETWORK_STATUS:
			update_network();
			break;
		case LOGO:
			oled_draw_logo();
			break;
		case HR4_STATUS:
			oled_draw_hr4();
			break;
		default:
			break;
	}
#endif 
}


/**
  * @brief  Template to show I2C bus status
  * @param  sensor_number: Sensor number
  * @param  sensor_status: Sensor status
  * @retval None.
  */
void oled_i2c_bus_status(uint8_t sensor_number)
{
#ifndef DISABLE_OLED	
	// Strings for labels
	uint8_t str_bus_sta[]   = "I2C Bus Status:";
	uint8_t str_oled_sta[] = "OLED:";
	uint8_t str_rtcore_sta[] = "Real Time Core:";

	switch (sensor_number)
	{
    	case CLEAR_BUFFER:
		{
			// Clear OLED buffer
			clear_oled_buffer();
		}
		break;

		case I2C_INIT:
		{
			// If we are here I2C is working well

			// Clear OLED buffer
			clear_oled_buffer();

			// Draw the title
			sd1306_draw_string(OLED_TITLE_X, OLED_TITLE_Y, " I2C Init", FONT_SIZE_TITLE, white_pixel);

			// Draw a label at line 1
			sd1306_draw_string(OLED_LINE_1_X, OLED_LINE_1_Y, str_bus_sta, FONT_SIZE_LINE, white_pixel);

			// I2C bus OK, if not OLED doesn't show a image
			sd1306_draw_string(sizeof(str_bus_sta) * 6, OLED_LINE_1_Y, "OK", FONT_SIZE_LINE, white_pixel);

			// Draw a label at line 2
			sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_oled_sta, FONT_SIZE_LINE, white_pixel);

			// If we're  using the OLED then it's installed and working
     		sd1306_draw_string(sizeof(str_oled_sta) * 6, OLED_LINE_2_Y, "OK", FONT_SIZE_LINE, white_pixel);
			
			// Draw a label at line 3
			sd1306_draw_string(OLED_LINE_3_X, OLED_LINE_3_Y, str_rtcore_sta, FONT_SIZE_LINE, white_pixel);

			// Show RTcore status
			if (i2cFd > 0)
			{
				sd1306_draw_string(sizeof(str_rtcore_sta) * 6, OLED_LINE_3_Y, "OK", FONT_SIZE_LINE, white_pixel);
			}
			else
			{
				sd1306_draw_string(sizeof(str_rtcore_sta) * 6, OLED_LINE_3_Y, "ERROR", FONT_SIZE_LINE, white_pixel);
			}
		}
		default:
			break;
	}

	// Send the buffer to OLED RAM
	sd1306_refresh();
#endif 
}

/**
  * @brief  Get the channel at given frequency
  * @param  freq_MHz: Frequency in MHz
  * @retval Channel.
  */
uint16_t get_channel(uint16_t freq_MHz)
{
	if (freq_MHz < 5000 && freq_MHz > 2400)
	{
		// channel of in 2.4 GHz band
		freq_MHz -= 2407;
	}
	else if(freq_MHz > 5000)
	{
		// channel of in 5 GHz band
		freq_MHz -= 5000;
	}
	else
	{
		// channel not in 2.4 or 5 GHz bands
		freq_MHz = 0;
	}

	freq_MHz /= 5;

	return freq_MHz;
}

/**
  * @brief  Template to show Network status
  * @param  None
  * @retval None.
  */
void update_network()
{
#ifndef DISABLE_OLED
	uint8_t string_data[10];
	uint16_t channel;
	uint8_t aux_size;
	
	// Strings for labels
	uint8_t str_SSID[] = "SSID:";
	uint8_t str_freq[] = "Freq:";
	uint8_t str_RSSI[] = "RSSI:";
	uint8_t str_chan[] = "Chan:";

	uint8_t aux_data_str[] = "%d";

	// Clear oled buffer
	clear_oled_buffer();

	// Draw the title
	sd1306_draw_string(OLED_TITLE_X, OLED_TITLE_Y, "  Network", FONT_SIZE_TITLE, white_pixel);

	// Draw a label at line 1
	sd1306_draw_string(OLED_LINE_1_X, OLED_LINE_1_Y, str_SSID, FONT_SIZE_LINE, white_pixel);
	// Draw SSID string
	sd1306_draw_string(sizeof(str_SSID)*6, OLED_LINE_1_Y, network_data.SSID, FONT_SIZE_LINE, white_pixel);

	// Draw a label at line 2
	sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_freq, FONT_SIZE_LINE, white_pixel);

	// Convert frequency value to string
	intToStr(network_data.frequency_MHz, string_data, 1);

	// Draw frequency value
	sd1306_draw_string(sizeof(str_freq) * 6, OLED_LINE_2_Y, string_data, FONT_SIZE_LINE, white_pixel);
	// Draw the units
	//sd1306_draw_string(sizeof(str_freq) * 6 + (get_str_size(string_data)+1) * 6, OLED_LINE_2_Y, "MHz", FONT_SIZE_LINE, white_pixel);

	// Draw channel label at line 2
	sd1306_draw_string(sizeof(str_freq) * 6 + (get_str_size(string_data) + 1) * 6, OLED_LINE_2_Y, str_chan, FONT_SIZE_LINE, white_pixel);

	channel = get_channel(network_data.frequency_MHz);

	aux_size = get_str_size(string_data);

	// Convert frequency value to string
	intToStr(channel, string_data, 1);

	// Draw channel value
	sd1306_draw_string(sizeof(str_freq) * 6 + (aux_size + sizeof(str_chan)+ 1) * 6, OLED_LINE_2_Y, string_data, FONT_SIZE_LINE, white_pixel);

	// Draw a label at line 3
	sd1306_draw_string(OLED_LINE_3_X, OLED_LINE_3_Y, str_RSSI, FONT_SIZE_LINE, white_pixel);

	// Convert RSSI value to string (Currently RSSI is always zero)
	intToStr(network_data.rssi, string_data, 1);

	strcpy(string_data, "%d");
	snprintf(string_data, 10, aux_data_str, network_data.rssi);

	// Draw RSSI value
	sd1306_draw_string(sizeof(str_RSSI) * 6, OLED_LINE_3_Y, string_data, FONT_SIZE_LINE, white_pixel);

	// Draw dBm unit
	sd1306_draw_string(sizeof(str_freq) * 6 + (get_str_size(string_data) + 1) * 6, OLED_LINE_3_Y, "dBm", FONT_SIZE_LINE, white_pixel);

	// Send the buffer to OLED RAM
	sd1306_refresh();
#endif 
}

/**
  * @brief  Template to show Acceleration data
  * @param  x: Acceleration in axis X
  * @param  y: Acceleration in axis Y
  * @param  z: Acceleration in axis Z
  * @retval None.
  */

/**
  * @brief  Template to show a logo
  * @param  None.
  * @retval None.
  */
void oled_draw_logo(void)
{
#ifndef DISABLE_OLED

	// Copy image_avnet to OLED buffer
	sd1306_draw_img(Image_avnet_bmp);

	// Send the buffer to OLED RAM
	sd1306_refresh();
#endif 
}

void oled_draw_hr4(void)
{

#ifndef DISABLE_OLED
	uint8_t string_data[10];
	uint8_t aux_size;

	// Strings for labels
	uint8_t str_heartRate[] =   "Heart Rate: ";
	uint8_t str_sp02[] =        "      SP02: ";
	uint8_t str_idle[] =        "    Press button A       to start test";
	uint8_t str_calibrating[] = "Calibrating . . .";
	uint8_t str_reposition[] =  "  Reposition finger        on sensor";
	
	uint8_t str_finger[] =      "     Place finger          on sensor";
	uint8_t str_readingData[] = "  Collecting data      points . . . ";

	// Clear oled buffer
	clear_oled_buffer();

	// Draw the title
	sd1306_draw_string(OLED_TITLE_X, OLED_TITLE_Y, " HealthCare", FONT_SIZE_TITLE, white_pixel);

	switch (currentState) {
	case IDLE:
		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_idle, FONT_SIZE_LINE, white_pixel);
		break;

	case HR4_CALIBRATE:
		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_calibrating, FONT_SIZE_LINE, white_pixel);
		break;

	case REPOSITION_FINGER:
		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_reposition, FONT_SIZE_LINE, white_pixel);
		break;

	case DATA_COLLECTION:
		// Draw a label at line 1
		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_readingData, FONT_SIZE_LINE, white_pixel);
		break;

	case DETECT_FINGER:
		// Draw a label at line 1
		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_finger, FONT_SIZE_LINE, white_pixel);
		break;

	case DISPLAY_RESULTS:
		// Convert heartRate value to string
		ftoa(median_hr, string_data, 1);

		sd1306_draw_string(OLED_LINE_2_X, OLED_LINE_2_Y, str_heartRate, FONT_SIZE_LINE, white_pixel);
		sd1306_draw_string(sizeof(str_heartRate) * 6, OLED_LINE_2_Y, string_data, FONT_SIZE_LINE, white_pixel);

		// Draw the units
		sd1306_draw_string(sizeof(str_heartRate) * 6 + (strlen(string_data) + 1) * 6, OLED_LINE_2_Y, "BPM", FONT_SIZE_LINE, white_pixel);

		// Convert sp02 value to string, add the % symbol
		ftoa(median_spo2, string_data, 1);
		string_data[strlen(string_data) + 1] = '\0';
		string_data[strlen(string_data)] = '%';

		sd1306_draw_string(OLED_LINE_3_X, OLED_LINE_3_Y, str_sp02, FONT_SIZE_LINE, white_pixel);
		sd1306_draw_string(sizeof(str_sp02) * 6, OLED_LINE_3_Y, string_data, FONT_SIZE_LINE, white_pixel);
		break;

	default:
		break;
	};

	// Send the buffer to OLED RAM
	sd1306_refresh();
#endif 
}

// reverses a string 'str' of length 'len' 
static void reverse(uint8_t *str, int32_t len)
{
	int32_t i = 0;
	int32_t j = len - 1;
	int32_t temp;

	while (i < j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

/**
  * @brief  Converts a given integer x to string uint8_t[]
  * @param  x: x integer input
  * @param  str: uint8_t array output
  * @param  d: Number of zeros added
  * @retval i: number of digits
  */
int32_t intToStr(int32_t x, uint8_t str[], int32_t d)
{
	int32_t i = 0;
	uint8_t flag_neg = 0;

	if (x < 0)
	{
		flag_neg = 1;
		x *= -1;
	}
	while (x)
	{
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then 
	// add 0s at the beginning 
	while (i < d)
	{
		str[i++] = '0';
	}

	if (flag_neg)
	{
		str[i] = '-';
		i++;
	}

	reverse(str, i);
	str[i] = '\0';
	return i;
}

/**
  * @brief  Converts a given integer x to string uint8_t[]
  * @param  n: float number to convert
  * @param  res:
  * @param  afterpoint:
  * @retval None.
  */
void ftoa(float n, uint8_t *res, int32_t afterpoint)
{
	// Extract integer part 
	int32_t ipart = (int32_t)n;

	// Extract floating part 
	float fpart = n - (float)ipart;

	int32_t i;

	if (ipart < 0)
	{
		res[0] = '-';
		res++;
		ipart *= -1;
	}

	if (fpart < 0)
	{
		fpart *= -1;

		if (ipart == 0)
		{
			res[0] = '-';
			res++;
		}
	}

	// convert integer part to string 
	i = intToStr(ipart, res, 1);

	// check for display option after point 
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot 

		// Get the value of fraction part upto given no. 
		// of points after dot. The third parameter is needed 
		// to handle cases like 233.007 
		fpart = fpart * (float)pow(10, afterpoint);

		intToStr((int32_t)fpart, res + i + 1, afterpoint);
	}
}

// AVNET logo

const unsigned char Image_avnet_bmp[BUFFER_SIZE_OLED] =
{
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,128,240,240,240,240, 48,  0,  0,112,
  240,240,240,224,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,112,
  240,240,240,192,  0,  0,  0,  0,  0,  0,  0,  0,  0,224,240,240,
  240, 16,  0,  0,  0,  0,  0,  0,  0,  0,240,240,240,240,224,128,
	0,  0,  0,  0,  0,  0,  0,  0,240,240,240,240,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,240,240,240,240,112,112,112,112,112,112,
  112,112,112,112,112,  0,  0,  0,  0,  0,  0,  0,  0,112,112,112,
  112,112,112,112,240,240,240,240,112,112,112,112,112,112,  0,  0,
	0,  0,  0,  0,  0,224,252,255,255,255, 15,  1,  0,  0,  0,  0,
	3, 15,127,255,255,248,128,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	7, 31,255,255,254,240,  0,  0,  0,  0,224,248,255,255,127,  7,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,255,255,255,255, 15, 31,
  127,252,248,224,224,128,  0,  0,255,255,255,255,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,255,255,255,255,224,224,224,224,224,224,
  224,224,224,224,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,255,255,255,255,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,128,240,254,255,127, 15,  1,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  3, 31,255,255,252,224,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  7, 63,255,255,248,240,254,255,255, 31,  3,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,255,255,255,255,  0,  0,
	0,  1,  3, 15, 15, 63,126,252,255,255,255,255,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,255,255,255,255,129,129,129,129,129,129,
  129,129,129,129,128,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,255,255,255,255,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  7,  7,  7,  3,  0,  0,  0, 12, 14, 14, 14, 14, 14, 14,
   14, 14, 12,  0,  0,  0,  7,  7,  7,  7,  4,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  1,  7,  7,  7,  7,  1,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  7,  7,  7,  7,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  1,  3,  7,  7,  7,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,
	7,  7,  7,  7,  7,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  7,  7,  7,  7,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};


uint8_t get_str_size(uint8_t * str)
{
	uint8_t legth = 0;
	while (*(str) != NULL)
	{
		str++;
		legth++;
	}
	return legth;
}