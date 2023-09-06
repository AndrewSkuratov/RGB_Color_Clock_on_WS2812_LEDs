/*
 * WS2812_RGB_Clock.c
 *
 * Created: 13.07.2022 9:04:56
 * Author : Andrew
 */ 
 

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "light_ws2812.h"
//#include <avr/pgmspace.h>

#define F_CPU 16000000UL

// CKSEL3:0 = 0100 --> 8 MHz, (default = 0001 --> 1 MHz)
// SUT1:0 = 10 (default) = 65 ms Start-up

#include <util/delay.h>

#define Q_DEL _delay_loop_2(3)
#define H_DEL _delay_loop_2(5)

//#define RTC_PENDULUM_LED 2

//#define TIMER_CAL 34287
//31250 <-- 0,5 sec @ 256 prescaller @ 8 MHz, or 0,5 sec @ 1024 prescaller @ 16 MHz
//#define TIMER_CAL 58200//58415
//#define TIMER_CAL 58200//1.064 Hz
//#define TIMER_CAL 57700// 0.99556 Hz
//#define TIMER_CAL 57750// 1.0020  Hz
//#define TIMER_CAL 57725// T= 1.000133 s
//#define TIMER_CAL 57739// F = 1.001 Hz 
//#define TIMER_CAL 34352// 
//#define TIMER_CAL 49943//
//#define TIMER_CAL 49923// @ MODE=5 (div 1024)
//#define TIMER_CAL 26500// @ MODE=2 (div 1) = 42000 --> 42 Hz
//#define TIMER_CAL 41552// @ MODE=1 (div 1) = 42000 --> 42 Hz
//#define TIMER_CAL 53544// @ MODE=1 (div 1) = 42000 --> 42 Hz
//#define TIMER_CAL 63973// @ MODE=1 (div 1) = 42000 --> 42 Hz
#define TIMER_CAL 63973// @ MODE=5 (div 1024) = 42000 --> 42 Hz



//

#define LED_LINE1_STRIP_PORT PORTD
#define LED_LINE1_STRIP_DDR DDRD
#define LED_LINE1_STRIP_PIN 0

#define LED_LINE2_STRIP_PORT PORTD
#define LED_LINE2_STRIP_DDR DDRD
#define LED_LINE2_STRIP_PIN 1

#define LED_LINE3_STRIP_PORT PORTD
#define LED_LINE3_STRIP_DDR DDRD
#define LED_LINE3_STRIP_PIN 2

#define LED_LINE4_STRIP_PORT PORTD
#define LED_LINE4_STRIP_DDR DDRD
#define LED_LINE4_STRIP_PIN 3

#define LED_LINE5_STRIP_PORT PORTD
#define LED_LINE5_STRIP_DDR DDRD
#define LED_LINE5_STRIP_PIN 4
//
// Vibor cifri 1=HR_D, 2=HR_E, 3=MIN_D, 4=MIN_E, 5=SEC_D, 6=SEC_E, 7=MS_x100
#define HR_D 1
#define HR_E 2
#define MIN_D 4
#define MIN_E 5
#define SEC_D 6
#define SEC_E 7
#define MS_x100 8
#define SEC_DOTS 3
#define RTC_Time_MSEC_xDOTS_ON 11
#define RTC_Time_MSEC_xDOTS_OFF 12
#define DIGIT_BLINK 1

#define TIME_MODE_ALL 0
#define TIME_MODE_HR_MIN 1

//
//ADC channel
#define ADC_PORT PORTC
#define ADC_DDR DDRC
#define ADC_CH0 0

// -------------- GLOBAL VARIABLES -----------------

// ADC Converter
volatile unsigned char ADC_RESULT_H=0x00; // ADCi result (hex)

// Control variables
volatile unsigned int BTN0_CNT = 0;
volatile unsigned int BTN1_CNT = 0;
volatile unsigned int BTN2_CNT = 0;

volatile unsigned char CMD_COLOR_SET = 17;
volatile unsigned char CMD_ADJUST = 0;
volatile unsigned char CMD_MODE = 0;


// ----Array---

// Firmware
//unsigned char DT_Firmware_Ln1[16] PROGMEM = {0x66,0x69,0x72,0x6D,0x77,0x61,0x72,0x65,0x3A,0x20,0x76, 0x31,0x2E,0x30, 0x20,0xE9}; //firmware: v1.0 ~
//unsigned char DT_Firmware_Ln2[16] PROGMEM = {0x20,0x20,0x20,0x20,0x20,  0x37,0x37,0x32,0x30,  0x20,0x62,0x79,0x74,0x65,0x73,0x20}; // 7720 bytes

// -------------- GLOBAL VARIABLES for CLOCK -----------------

volatile unsigned char RTC_Time_pendulum = 0;
volatile unsigned char RTC_Time_HR_D = 0;
volatile unsigned char RTC_Time_HR_E = 0;
volatile unsigned char RTC_Time_MIN_D = 0;
volatile unsigned char RTC_Time_MIN_E = 0;
volatile unsigned char RTC_Time_SEC_D = 0;
volatile unsigned char RTC_Time_SEC_E = 0;
volatile unsigned char RTC_Time_MSEC_x100 = 0;
volatile unsigned char Digit_Type = 0;
volatile unsigned char digit_view_mode = 0;
//
//volatile uint16_t CNTx1ms = 0;
//volatile uint16_t CNTx16Bit = 0;
//volatile uint32_t CNTx32BIT = 0;
#define COUNT_TO 0.5
#define TIMER2_COMPARE_VALUE  194//194
//#define CORRECTION 0.00001257501 //0.000597501; < --------------------- 0.50169; 
volatile float SEC_FLOAT = 0; // < ---------------------
//#define TIM1_ONE_STEP 0.100523625// Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//#define TIM1_ONE_STEP 0.107500//0.94 trend. Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//#define TIM1_ONE_STEP 0.10062500// 1.0016 s trend. Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//#define TIM1_ONE_STEP 0.10063500// <1.0016 s trend. Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//#define TIM1_ONE_STEP 0.10064500// Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//#define TIM1_ONE_STEP 0.10068500// Middle from Scanning by Saleae16 (MIN + MAX) \ 2 = 24.9 ms ~ 40 Hz
//
// ~+8..10 min / week.
//#define TIM1_ONE_STEP 0.10078500// ~ 1.000 s trend. from Scanning by Saleae16. [0.10078500] --> ~+8..10 min / week.
//
//#define TIM1_ONE_STEP 0.10035500// ~ -15..20 min / week. from Scanning by Saleae16. [0.10035500] --> ~ -15..20 min / week.
//
//#define TIM1_ONE_STEP 0.10057// +10 min / month. from Scanning by Saleae16. [0.10057] --> +10 min / month.
//
#define TIM1_ONE_STEP 0.10049// ??? min / month. from Scanning by Saleae16. [0.10049] --> ~ ??? min / month.
//
//
// Colors for Line Digits
volatile unsigned char CNT_DELAY = 5, State_m = 0;
volatile unsigned char Red, Current_Red, Prepare_Red;
volatile unsigned char Green, Current_Green, Prepare_Green;
volatile unsigned char Blue, Current_Blue, Prepare_Blue;
volatile unsigned char BackGround_Red, BackGround_Green, BackGround_Blue;
volatile unsigned char	R_DIR, G_DIR, B_DIR;
volatile unsigned char 	Green_MAX_LIGHT, Red_MAX_LIGHT, Blue_MAX_LIGHT;
volatile unsigned char Red_MAX_Brightness, Green_MAX_Brightness, Blue_MAX_Brightness;
//
volatile unsigned char CMD_RUN_RTC_unit_pendulum = 0;
// reserve functions
void RTC_unit_pendulum_v2_RGB_LED_Clock (void);
void BUTTONS_FUNCTIONS (void);

// -- ???????????? ???????????? ??????????

// ???????????? ?????????? ??????? ???????????? T/C1


//-----------------------------
//-----------FUNCTIONS---------
//-----------------------------


void RTC_unit_pendulum_v2_RGB_LED_Clock (void)
{
	// --------------------------
	// RTC - Real Time Clock unit
	// --------------------------
	// ---------- variables: ------------
	// RTC_pendulum - time pendulum with period = 1 sec;
	// RTC_hr - Time "hours" register
	// RTC_min - Time "minutes" register
	// RTC_sec - Time "seconds" register
	RTC_Time_pendulum = RTC_Time_pendulum ^ 0x01; // Trigger Toggle mode
	//PORTB = PORTB ^ (1 << 1); // toggle RB1 (RTC Pendulum LED) for Calibration by Saleae16
	
	if (RTC_Time_pendulum == 1)
	{

		RTC_Time_SEC_E++; // increment (+1) "Second" register
		if (RTC_Time_SEC_E >= 10)
		{
			RTC_Time_SEC_E = 0; // clear Seconds register
			RTC_Time_SEC_D++;
			//
			if (RTC_Time_SEC_D >= 6)
			{
				RTC_Time_SEC_D = 0;
				RTC_Time_MIN_E++; // increment (+1) "Minutes" register
				//
				if (RTC_Time_MIN_E >= 10)
				{
					RTC_Time_MIN_E = 0; // clear minutes register
					RTC_Time_MIN_D++;
					//
					if (RTC_Time_MIN_D >= 6)
					{
						RTC_Time_MIN_D = 0;
						RTC_Time_HR_E++; // increment (+1) "Hours" register
						if (RTC_Time_HR_E >= 10)
						{
							RTC_Time_HR_E = 0; // reset Time "Hours" register
							RTC_Time_HR_D++;
							//
						}
						if ((RTC_Time_HR_D >= 2) & (RTC_Time_HR_E >= 4))
						{
							RTC_Time_HR_E = 0;
							RTC_Time_HR_D = 0;
						} 
					
					}
				}
			}
		}
	}
}


void BUTTONS_FUNCTIONS (void)
{
  // 1 PIND7 -KEY4			BTN HARDWARE RESET
  // 2 PINB0 -KEY1			BTN0 (MODE)
  // 3 PIND5 -KEY2			BTN1 (ADJUST)
  // 4 PIND6 -KEY3			BTN2 (COLLOR SET)
	//----------------------------
	//-----------MODE Button-----------------
	if (~PIND &(1<<5))
	{
	 	if (BTN0_CNT < 65534)
	  	{
	    	 BTN0_CNT++;
	  	}
		if (BTN0_CNT >= 64)
		{
			CMD_MODE++;
			if (CMD_MODE >= 5)
			{
				CMD_MODE = 0;
			}
			BTN0_CNT = 0;
		}
	}
	else
	{
		BTN0_CNT = 0;
	}
	//-----------ADJUST Button----------------
	if (~PIND &(1<<6))
	{
	 	if (BTN1_CNT < 65534)
	  	{
	    	 BTN1_CNT++;
	  	}
		if (BTN1_CNT >= 64)
		{
			CMD_ADJUST = 1;
			BTN1_CNT = 0;
			//
			if(CMD_MODE == 1)// SET HR_D
			{
				RTC_Time_HR_D++;
				if (RTC_Time_HR_D == 2)
				{
					if (RTC_Time_HR_E >= 4)
					{
						RTC_Time_HR_E = 0;
					}
				}
				if (RTC_Time_HR_D >=3)
				{
					RTC_Time_HR_D = 0;
				}
			}
			if(CMD_MODE == 2)// SET HR_D
			{
				RTC_Time_HR_E++;
				if(RTC_Time_HR_D < 2)
				{
					if (RTC_Time_HR_E >= 10)
					{
						RTC_Time_HR_E = 0;
					}
				}
				else
				{
					if (RTC_Time_HR_E >= 4)
					{
						RTC_Time_HR_E = 0;
					}
				}
			}
			if(CMD_MODE == 3)// SET HR_D
			{
				RTC_Time_MIN_D++;
				if(RTC_Time_MIN_D >= 6)
				{
					RTC_Time_MIN_D = 0;
				}
			}
			if(CMD_MODE == 4)// SET HR_D
			{
				RTC_Time_MIN_E++;
				if(RTC_Time_MIN_E >= 10)
				{
					RTC_Time_MIN_E = 0;
				}
			}
		}
	}
	else
	{
		BTN1_CNT = 0;
	}
	//-----------COLOR SET Button-----------------
	if (~PIND &(1<<7))
	{
	 	if (BTN2_CNT < 65534)
	  	{
	    	 BTN2_CNT++;
	  	}
		if (BTN2_CNT >= 64)
		{
			CMD_COLOR_SET++;
			if (CMD_COLOR_SET >= 19)
			{
				CMD_COLOR_SET = 1;
			}
			BTN2_CNT = 0;
		}
	}
	else
	{
		BTN2_CNT = 0;
	}
}

//--------------
void SHOW_BYTE (unsigned char RES_data)
{
	// before writing data - disable 10 KHz GENERATOR for
	unsigned char result_mas[3]={0,0,0};
	unsigned char tmp=0;
	unsigned char d=0, k=0;

	k = 0;
	tmp = RES_data;
	// ???????? ????? ?? ????? ?????
	
	while (tmp > 0)
	{
		d = tmp % 10;
		// (1023 % 10 = 3, 102 % 10 = 2, 10 % 10 = 0, 1 % 10 = 1)
		if (k <= 2)
		result_mas[2-k] = d;
		tmp = tmp / 10;
		k++;
	}
	RTC_Time_HR_D = result_mas[0];
	RTC_Time_HR_E = result_mas[1];
	RTC_Time_MIN_D = result_mas[2];
}
//---------------

// --- ADC  Analog to Digital converter--
void ADC_Initialization (unsigned char ADC_pin)
{
	ADC_DDR = ADC_DDR & ~(1 << ADC_pin); // set ADC_pin to input
	
	ADC_PORT = ADC_PORT & ~(1 << ADC_pin); // ADC_pin set Z-condition

	// ----------ADC setup-------
	ADMUX = 0b11100000; // [7,6] - REFS1=1,REFS0=1 --> ??? ?????????? = 2,56 ?;
	// [5] - ADLAR = 1 (????? ????????????
	//					?????????? ? ADCH, ADCL )
	// [4..0] - ????? ????? ADC0
	
	ADMUX = ADMUX | ADC_pin; // ????? ?????? MUX

	// ??????? ?????????? ? ????????? (ADC control & status register)
	ADCSRA = 0b10000100; // 0b10001100
	//   ||||||||
	//  [76543210]

	// [7] - ?????????? (?????????) ??? (1=ON, 0=OFF)
	// [6] - ?????? ?????????????? (1 = Start)
	// [5] - ????? ?????? (0 - ????????? ??????????????)
	// [4] - ???? ?????????? (?????? ??????)
	// [3] - ?????????? ?????????? ??????????? ???
	// [2..0] - Frequency divider (100 --> div 16)


	// ????????????? ??? (?????? ?????)
	ADCSRA = ADCSRA | (1 << 6); // ????????? ???
	// ???????? ?????????? (???? ????????? ????? ?????????? ADIF

	while (~ADCSRA & (1<<4))
	{
	}

	ADCSRA = ADCSRA | (1 << 4); // ??????? ???? ?????????? (?????? "1")

}


void ADC_convert (unsigned char ADC_pin)
{
	// ----------ADC setup-------
	ADMUX = 0b11100000; // [7,6] - REFS1=1,REFS0=1 --> ??? ?????????? = 2,56 ?;
	// [5] - ADLAR = 1 (????? ????????????
	//					?????????? ? ADCH, ADCL )
	// [4..0] - ????? ????? ADC0
	
	ADMUX = ADMUX | ADC_pin; // ????? ?????? ADC


	//	if (~ADCSRA & (1<<6)) // ???? ?? ??????? ???
	//	{
	ADCSRA = ADCSRA | (1<<6); // ????????? ???
	//	}
	// ???????? ?????????? (???? ????????? ????? ?????????? ADIF
	while (~ADCSRA & (1<<4))
	{
	}

	ADCSRA = ADCSRA | (1 << 4); // ??????? ???? ?????????? (?????? "1")

	ADC_RESULT_H = ADCH; // ?????????? ???????? ???????? ????? ?? ???
}

// The rgb_color struct represents the color for an 8-bit RGB LED.
// Examples:
//   Black:      (rgb_color){ 0, 0, 0 }
//   Pure red:   (rgb_color){ 255, 0, 0 }
//   Pure green: (rgb_color){ 0, 255, 0 }
//   Pure blue:  (rgb_color){ 0, 0, 255 }
//   White:      (rgb_color){ 255, 255, 255}
typedef struct rgb_color
{
	uint8_t red, green, blue;
} rgb_color;

// led_strip_write sends a series of colors to the LED strip, updating the LEDs.
// The colors parameter should point to an array of rgb_color structs that hold
// the colors to send.
// The count parameter is the number of colors to send.
// This function takes about 1.1 ms to update 30 LEDs.
// Interrupts must be disabled during that time, so any interrupt-based library
// can be negatively affected by this function.
// Timing details at 20 MHz:
//   0 pulse  = 400 ns
//   1 pulse  = 850 ns
//   "period" = 1300 ns
// Timing details at 16 MHz:
//   0 pulse  = 375 ns
//   1 pulse  = 812.5 ns
//   "period" = 1500 ns

/*
void __attribute__((noinline)) led_strip_write(rgb_color * X_colors, uint16_t count)
{
	cli();   // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
	
	// Set the pin to be an output driving low.
	LED_STRIP_PORT &= ~(1<<LED_STRIP_PIN);
	LED_STRIP_DDR |= (1<<LED_STRIP_PIN);
	
	while (count--)
	{
		// Send a color to the LED strip.
		// The assembly below also increments the 'colors' pointer,
		// it will be pointing to the next color at the end of this loop.
		asm volatile (
		"ld __tmp_reg__, %a0+\n"
		"ld __tmp_reg__, %a0\n"
		"rcall send_led_strip_byte%=\n"  // Send red component.
		"ld __tmp_reg__, -%a0\n"
		"rcall send_led_strip_byte%=\n"  // Send green component.
		"ld __tmp_reg__, %a0+\n"
		"ld __tmp_reg__, %a0+\n"
		"ld __tmp_reg__, %a0+\n"
		"rcall send_led_strip_byte%=\n"  // Send blue component.
		"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

		// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
		"send_led_strip_byte%=:\n"
		"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
		"ret\n"

		// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
		// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
		// but this function always takes the same time (2 us).
		"send_led_strip_bit%=:\n"
		#if F_CPU == 8000000
		"rol __tmp_reg__\n"                      // Rotate left through carry.
		#endif
		"sbi %2, %3\n"                           // Drive the line high.

		#if F_CPU != 8000000
		"rol __tmp_reg__\n"                      // Rotate left through carry.
		#endif

		#if F_CPU == 16000000
		"nop\n" "nop\n"
		#elif F_CPU == 20000000
		"nop\n" "nop\n" "nop\n" "nop\n"
		#elif F_CPU != 8000000
		#error "Unsupported F_CPU"
		#endif

		"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

		#if F_CPU == 8000000
		"nop\n" "nop\n"
		#elif F_CPU == 16000000
		"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
		#elif F_CPU == 20000000
		"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
		"nop\n" "nop\n"
		#endif

		"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

		"ret\n"
		"led_strip_asm_end%=: "
		: "=b" (X_colors)
		: "0" (X_colors),         // %a0 points to the next color to display
		"I" (_SFR_IO_ADDR(LED_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
		"I" (LED_STRIP_PIN)     // %3 is the pin number (0-8)
		);

		// Uncomment the line below to temporarily enable interrupts between each color.
		//sei(); asm volatile("nop\n"); cli();
	}
	sei();          // Re-enable interrupts now that we are done.
	_delay_us(50);  // Send the reset signal. 80
}
*/

void __attribute__((noinline)) led_Line_strip_write(rgb_color * X_colors, uint16_t count, uint8_t Line_Number)
{
	cli();   // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
	
	// Set the pin to be an output driving low.
	switch (Line_Number)
	{
		case 1: 
			LED_LINE1_STRIP_PORT &= ~(1<<LED_LINE1_STRIP_PIN);
			LED_LINE1_STRIP_DDR |= (1<<LED_LINE1_STRIP_PIN);
			while (count--)
			{
				// Send a color to the LED strip.
				// The assembly below also increments the 'colors' pointer,
				// it will be pointing to the next color at the end of this loop.
				asm volatile (
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0\n"
				"rcall send_led_strip_byte%=\n"  // Send red component.
				"ld __tmp_reg__, -%a0\n"
				"rcall send_led_strip_byte%=\n"  // Send green component.
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"rcall send_led_strip_byte%=\n"  // Send blue component.
				"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

				// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
				"send_led_strip_byte%=:\n"
				"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
				"ret\n"

				// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
				// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
				// but this function always takes the same time (2 us).
				"send_led_strip_bit%=:\n"
				#if F_CPU == 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif
				"sbi %2, %3\n"                           // Drive the line high.

				#if F_CPU != 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif

				#if F_CPU == 16000000
				"nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU != 8000000
				#error "Unsupported F_CPU"
				#endif

				"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

				#if F_CPU == 8000000
				"nop\n" "nop\n"
				#elif F_CPU == 16000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				"nop\n" "nop\n"
				#endif

				"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

				"ret\n"
				"led_strip_asm_end%=: "
				: "=b" (X_colors)
				: "0" (X_colors),         // %a0 points to the next color to display
				"I" (_SFR_IO_ADDR(LED_LINE1_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
				"I" (LED_LINE1_STRIP_PIN)     // %3 is the pin number (0-8)
				);

				// Uncomment the line below to temporarily enable interrupts between each color.
				//sei(); asm volatile("nop\n"); cli();
			}
		break;
		case 2:
			LED_LINE2_STRIP_PORT &= ~(1<<LED_LINE2_STRIP_PIN);
			LED_LINE2_STRIP_DDR |= (1<<LED_LINE2_STRIP_PIN);
			while (count--)
			{
				// Send a color to the LED strip.
				// The assembly below also increments the 'colors' pointer,
				// it will be pointing to the next color at the end of this loop.
				asm volatile (
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0\n"
				"rcall send_led_strip_byte%=\n"  // Send red component.
				"ld __tmp_reg__, -%a0\n"
				"rcall send_led_strip_byte%=\n"  // Send green component.
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"rcall send_led_strip_byte%=\n"  // Send blue component.
				"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

				// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
				"send_led_strip_byte%=:\n"
				"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
				"ret\n"

				// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
				// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
				// but this function always takes the same time (2 us).
				"send_led_strip_bit%=:\n"
				#if F_CPU == 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif
				"sbi %2, %3\n"                           // Drive the line high.

				#if F_CPU != 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif

				#if F_CPU == 16000000
				"nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU != 8000000
				#error "Unsupported F_CPU"
				#endif

				"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

				#if F_CPU == 8000000
				"nop\n" "nop\n"
				#elif F_CPU == 16000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				"nop\n" "nop\n"
				#endif

				"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

				"ret\n"
				"led_strip_asm_end%=: "
				: "=b" (X_colors)
				: "0" (X_colors),         // %a0 points to the next color to display
				"I" (_SFR_IO_ADDR(LED_LINE2_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
				"I" (LED_LINE2_STRIP_PIN)     // %3 is the pin number (0-8)
				);

				// Uncomment the line below to temporarily enable interrupts between each color.
				//sei(); asm volatile("nop\n"); cli();
			}
		break;
		case 3:
			LED_LINE3_STRIP_PORT &= ~(1<<LED_LINE3_STRIP_PIN);
			LED_LINE3_STRIP_DDR |= (1<<LED_LINE3_STRIP_PIN);
			while (count--)
			{
				// Send a color to the LED strip.
				// The assembly below also increments the 'colors' pointer,
				// it will be pointing to the next color at the end of this loop.
				asm volatile (
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0\n"
				"rcall send_led_strip_byte%=\n"  // Send red component.
				"ld __tmp_reg__, -%a0\n"
				"rcall send_led_strip_byte%=\n"  // Send green component.
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"rcall send_led_strip_byte%=\n"  // Send blue component.
				"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

				// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
				"send_led_strip_byte%=:\n"
				"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
				"ret\n"

				// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
				// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
				// but this function always takes the same time (2 us).
				"send_led_strip_bit%=:\n"
				#if F_CPU == 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif
				"sbi %2, %3\n"                           // Drive the line high.

				#if F_CPU != 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif

				#if F_CPU == 16000000
				"nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU != 8000000
				#error "Unsupported F_CPU"
				#endif

				"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

				#if F_CPU == 8000000
				"nop\n" "nop\n"
				#elif F_CPU == 16000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				"nop\n" "nop\n"
				#endif

				"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

				"ret\n"
				"led_strip_asm_end%=: "
				: "=b" (X_colors)
				: "0" (X_colors),         // %a0 points to the next color to display
				"I" (_SFR_IO_ADDR(LED_LINE3_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
				"I" (LED_LINE3_STRIP_PIN)     // %3 is the pin number (0-8)
				);

				// Uncomment the line below to temporarily enable interrupts between each color.
				//sei(); asm volatile("nop\n"); cli();
			}
		break;
		case 4:
			LED_LINE4_STRIP_PORT &= ~(1<<LED_LINE4_STRIP_PIN);
			LED_LINE4_STRIP_DDR |= (1<<LED_LINE4_STRIP_PIN);
			while (count--)
			{
				// Send a color to the LED strip.
				// The assembly below also increments the 'colors' pointer,
				// it will be pointing to the next color at the end of this loop.
				asm volatile (
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0\n"
				"rcall send_led_strip_byte%=\n"  // Send red component.
				"ld __tmp_reg__, -%a0\n"
				"rcall send_led_strip_byte%=\n"  // Send green component.
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"rcall send_led_strip_byte%=\n"  // Send blue component.
				"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

				// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
				"send_led_strip_byte%=:\n"
				"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
				"ret\n"

				// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
				// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
				// but this function always takes the same time (2 us).
				"send_led_strip_bit%=:\n"
				#if F_CPU == 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif
				"sbi %2, %3\n"                           // Drive the line high.

				#if F_CPU != 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif

				#if F_CPU == 16000000
				"nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU != 8000000
				#error "Unsupported F_CPU"
				#endif

				"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

				#if F_CPU == 8000000
				"nop\n" "nop\n"
				#elif F_CPU == 16000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				"nop\n" "nop\n"
				#endif

				"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

				"ret\n"
				"led_strip_asm_end%=: "
				: "=b" (X_colors)
				: "0" (X_colors),         // %a0 points to the next color to display
				"I" (_SFR_IO_ADDR(LED_LINE4_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
				"I" (LED_LINE4_STRIP_PIN)     // %3 is the pin number (0-8)
				);

				// Uncomment the line below to temporarily enable interrupts between each color.
				//sei(); asm volatile("nop\n"); cli();
			}
		break;
		case 5:
			LED_LINE5_STRIP_PORT &= ~(1<<LED_LINE5_STRIP_PIN);
			LED_LINE5_STRIP_DDR |= (1<<LED_LINE5_STRIP_PIN);
			while (count--)
			{
				// Send a color to the LED strip.
				// The assembly below also increments the 'colors' pointer,
				// it will be pointing to the next color at the end of this loop.
				asm volatile (
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0\n"
				"rcall send_led_strip_byte%=\n"  // Send red component.
				"ld __tmp_reg__, -%a0\n"
				"rcall send_led_strip_byte%=\n"  // Send green component.
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"ld __tmp_reg__, %a0+\n"
				"rcall send_led_strip_byte%=\n"  // Send blue component.
				"rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

				// send_led_strip_byte subroutine:  Sends a byte to the LED strip.
				"send_led_strip_byte%=:\n"
				"rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"
				"rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
				"ret\n"

				// send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
				// high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
				// but this function always takes the same time (2 us).
				"send_led_strip_bit%=:\n"
				#if F_CPU == 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif
				"sbi %2, %3\n"                           // Drive the line high.

				#if F_CPU != 8000000
				"rol __tmp_reg__\n"                      // Rotate left through carry.
				#endif

				#if F_CPU == 16000000
				"nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU != 8000000
				#error "Unsupported F_CPU"
				#endif

				"brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

				#if F_CPU == 8000000
				"nop\n" "nop\n"
				#elif F_CPU == 16000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				#elif F_CPU == 20000000
				"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
				"nop\n" "nop\n"
				#endif

				"brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

				"ret\n"
				"led_strip_asm_end%=: "
				: "=b" (X_colors)
				: "0" (X_colors),         // %a0 points to the next color to display
				"I" (_SFR_IO_ADDR(LED_LINE5_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
				"I" (LED_LINE5_STRIP_PIN)     // %3 is the pin number (0-8)
				);

				// Uncomment the line below to temporarily enable interrupts between each color.
				//sei(); asm volatile("nop\n"); cli();
			}
		break;
		default:
		break;
	}
	sei();          // Re-enable interrupts now that we are done.
	_delay_us(50);  // Send the reset signal. 80
}


#define LED_COUNT 27
rgb_color X_colors[LED_COUNT];
rgb_color all_LEDs_colors_OFF[LED_COUNT];


void Function_Create_Lines_by_RTC (uint8_t time_mode, uint8_t Line_Nr, uint8_t F_Digit_Type, uint8_t Digit_Display_Mode)
{
	uint8_t Current_Digit = 0;
	uint8_t X_i = 0; // cifrovoj ukazatelj na x0,x1,x2...x25,x26
	//
	//----------------------
	// Vibor cifri 1=HR_D, 2=HR_E, 3=MIN_D, 4=MIN_E, 5=SEC_D, 6=SEC_E, 7=MS_x100, 8=SEC_DOTS
	/* 
	//vines vne funkcii
	Digit_Type++;
	if (Digit_Type >= 8)
	{
		Digit_Type = 1;
	}
	*/
	//----------------------
	// Prorisovka po linijam
	if (Line_Nr == 1) // top
	{
		if (F_Digit_Type == HR_D) // 1 - desatki chasov
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			if (Digit_Display_Mode == DIGIT_BLINK)
			{
				if(RTC_Time_pendulum == 1)
				{
					Red = 2;
					Green = 2;
					Blue = 2;
				}
			}
			//
			Current_Digit = RTC_Time_HR_D; // [0:2]
			if (time_mode == TIME_MODE_ALL)
				X_i = 0; // 0,1,2
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 4; // 4,5,6
			
		}
		if (F_Digit_Type == HR_E) // 2-cifra - edenici chasov {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if (Digit_Display_Mode == DIGIT_BLINK)
			{
				if(RTC_Time_pendulum == 1)
				{
					Red = 2;
					Green = 2;
					Blue = 2;
				}
			}
			//
			Current_Digit = RTC_Time_HR_E; // [4:6]
			if (time_mode == TIME_MODE_ALL)
				X_i = 4; // 4,5,6
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 8; // 8,9,10
			
		}
		if (F_Digit_Type == MIN_D) // 3-cifra - desjatki minut {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if (Digit_Display_Mode == DIGIT_BLINK)
			{
				if(RTC_Time_pendulum == 1)
				{
					Red = 2;
					Green = 2;
					Blue = 2;
				}
			}
			//
			Current_Digit = RTC_Time_MIN_D; // [8:10]

			if (time_mode == TIME_MODE_ALL)
				X_i = 8; // 8,9,10
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 16; // 16,17,18			
		}
		if (F_Digit_Type == MIN_E) // 4-cifra - edinici minut {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if (Digit_Display_Mode == DIGIT_BLINK)
			{
				if(RTC_Time_pendulum == 1)
				{
					Red = 2;
					Green = 2;
					Blue = 2;
				}
			}
			//
			Current_Digit = RTC_Time_MIN_E; // [12:14]
			if (time_mode == TIME_MODE_ALL)
				X_i = 12; // 12,13,14
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 20; // 20,21,22			
			
		}
		if (F_Digit_Type == SEC_D) // 5-cifra - desjtki sekund {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_D; // [16:18]
			if (time_mode == TIME_MODE_ALL)
				X_i = 16; // 16,17,18
		}
		if (F_Digit_Type == SEC_E) // 6-cifra - edinici sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_E; // [20:22]
			if (time_mode == TIME_MODE_ALL)
				X_i = 20; // 20,21,22			
		}
		if (F_Digit_Type == MS_x100) // 7-cifra - desjatki mili sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MSEC_x100; // [24:26]
			if (time_mode == TIME_MODE_ALL)
				X_i = 24; // 24,25,26
			
		}
		if (F_Digit_Type == SEC_DOTS) // 8-cifra - sekundnie tochki ":"
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if(RTC_Time_pendulum == 1)
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_ON; // '11' address: [14]
			}
			else
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_OFF; // '12' address: [14]
			}
			if (time_mode == TIME_MODE_HR_MIN)
			X_i = 13; // 20,21,22
		}
		// create X_colors array
		switch (Current_Digit)
		{
			// Cifra 0 imeet 3 tochki v line_1 --> xi,xi+1,xi+2
			case 0:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 1 imeet 1 tochku v line_1 --> x5
			case 1:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue }; // ON
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 2 imeet 3 tochki v line_1 --> x4,x5,x6
			case 2:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 3 imeet 3 tochki v line_1 --> x4,x5,x6
			case 3:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 4 imeet 2 tochki v line_1 --> x4,x6
			case 4:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 5 imeet 3 tochki v line_1 --> x4,x5,x6
			case 5:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 6 imeet 2 tochki v line_1 --> x5,x6
			case 6:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 7 imeet 3 tochki v line_1 --> x4,x5,x6
			case 7:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 8 imeet 3 tochki v line_1 --> x4,x5,x6
			case 8:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 9 imeet 3 tochki v line_1 --> x4,x5,x6
			case 9:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// ON DOTS
			case 11:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			// OFF DOTS
			case 12:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			default:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
		}
	}
	if (Line_Nr == 2) // middle
	{
		if (F_Digit_Type == HR_D) // 1 - desatki chasov
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_D; // [0:2]
			if (time_mode == TIME_MODE_ALL)
				X_i = 0; // 0,1,2
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 4; // 4,5,6
			
		}
		if (F_Digit_Type == HR_E) // 2-cifra - edenici chasov {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_E; // [4:6]
			if (time_mode == TIME_MODE_ALL)
				X_i = 4; // 4,5,6
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 8; // 8,9,10
			
		}
		if (F_Digit_Type == MIN_D) // 3-cifra - desjatki minut {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_D; // [8:10]

			if (time_mode == TIME_MODE_ALL)
				X_i = 8; // 8,9,10
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 16; // 16,17,18			
		}
		if (F_Digit_Type == MIN_E) // 4-cifra - edinici minut {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_E; // [12:14]
			if (time_mode == TIME_MODE_ALL)
				X_i = 12; // 12,13,14
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 20; // 20,21,22			
			
		}
		if (F_Digit_Type == SEC_D) // 5-cifra - desjtki sekund {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_D; // [16:18]
			if (time_mode == TIME_MODE_ALL)
				X_i = 16; // 16,17,18
		}
		if (F_Digit_Type == SEC_E) // 6-cifra - edinici sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_E; // [20:22]
			if (time_mode == TIME_MODE_ALL)
				X_i = 20; // 20,21,22			
		}
		if (F_Digit_Type == MS_x100) // 7-cifra - desjatki mili sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MSEC_x100; // [24:26]
			if (time_mode == TIME_MODE_ALL)
				X_i = 24; // 24,25,26
			
		}
		if (F_Digit_Type == SEC_DOTS) // 8-cifra - sekundnie tochki ":"
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if(RTC_Time_pendulum == 1)
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_ON; // '11' address: [14]
			}
			else
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_OFF; // '12' address: [14]
			}
			if (time_mode == TIME_MODE_HR_MIN)
			X_i = 13; // 20,21,22
		}
		// Create X_colors array
		switch (Current_Digit)
		{
			// Cifra 0 imeet 3 tochki v line_2 --> xi,xi+1,xi+2
			case 0:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 1 imeet 2 tochku v line_2 --> x_i,x_i+1
			case 1:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue }; // ON
				X_colors[X_i+1] = (rgb_color){ Red, Green, Blue }; // ON
				X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 2 imeet 1 tochki v line_2 --> x_i+2
			case 2:
				X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 3 imeet 1 tochki v line_1 --> x_i+2
			case 3:
				X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 4 imeet 2 tochki v line_1 --> x_i,x_i+2
			case 4:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 5 imeet 1 tochki v line_1 --> x_i
			case 5:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 6 imeet 1 tochki v line_1 --> x_i
			case 6:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 7 imeet 2 tochki v line_1 --> x_i,x_i+2
			case 7:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 8 imeet 3 tochki v line_1 --> x_i,x_i+2
			case 8:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 9 imeet 3 tochki v line_1 --> x_i,x_i+2
			case 9:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// ON DOTS
			case 11:
				X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// OFF DOTS
			case 12:
				X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			default:
				X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
				X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
		}
	}
	if (Line_Nr == 3) // middle
	{
		if (F_Digit_Type == HR_D) // 1 - desatki chasov
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_D; // [0:2]
			if (time_mode == TIME_MODE_ALL)
				X_i = 0; // 0,1,2
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 4; // 4,5,6
			
		}
		if (F_Digit_Type == HR_E) // 2-cifra - edenici chasov {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_E; // [4:6]
			if (time_mode == TIME_MODE_ALL)
				X_i = 4; // 4,5,6
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 8; // 8,9,10
			
		}
		if (F_Digit_Type == MIN_D) // 3-cifra - desjatki minut {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_D; // [8:10]

			if (time_mode == TIME_MODE_ALL)
				X_i = 8; // 8,9,10
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 16; // 16,17,18			
		}
		if (F_Digit_Type == MIN_E) // 4-cifra - edinici minut {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_E; // [12:14]
			if (time_mode == TIME_MODE_ALL)
				X_i = 12; // 12,13,14
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 20; // 20,21,22			
			
		}
		if (F_Digit_Type == SEC_D) // 5-cifra - desjtki sekund {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_D; // [16:18]
			if (time_mode == TIME_MODE_ALL)
				X_i = 16; // 16,17,18
		}
		if (F_Digit_Type == SEC_E) // 6-cifra - edinici sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_E; // [20:22]
			if (time_mode == TIME_MODE_ALL)
				X_i = 20; // 20,21,22			
		}
		if (F_Digit_Type == MS_x100) // 7-cifra - desjatki mili sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MSEC_x100; // [24:26]
			if (time_mode == TIME_MODE_ALL)
				X_i = 24; // 24,25,26
			
		}
		if (F_Digit_Type == SEC_DOTS) // 8-cifra - sekundnie tochki ":"
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if(RTC_Time_pendulum == 1)
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_ON; // '11' address: [14]
			}
			else
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_OFF; // '12' address: [14]
			}
			if (time_mode == TIME_MODE_HR_MIN)
			X_i = 13; // 20,21,22
		}
		// create X_colors array
		switch (Current_Digit)
		{
			// Cifra 0 imeet 2 tochki v line_2 --> xi,xi+2
			case 0:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 1 imeet 1 tochku v line_2 --> x_i+1
			case 1:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue }; // ON
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 2 imeet 3 tochki v line_2 --> x_i,x_i+1,x_i+2
			case 2:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 3 imeet 1 tochki v line_1 --> x_i+2
			case 3:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 4 imeet 3 tochki v line_1 --> x_i,x_i+1,x_i+2
			case 4:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 5 imeet 3 tochki v line_1 --> x_i,x_i+1,x_i+2
			case 5:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 6 imeet 2 tochki v line_1 --> x_i,x_i+1
			case 6:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 7 imeet 1 tochki v line_1 --> x_i+2
			case 7:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 8 imeet 3 tochki v line_1 --> x_i,x_i+1,x_i+2
			case 8:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 9 imeet 3 tochki v line_1 --> x_i,x_i+1,x_i+2
			case 9:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// ON DOTS
			case 11:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			// OFF DOTS
			case 12:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			default:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
		}
	}
	if (Line_Nr == 4) // middle
	{
		if (F_Digit_Type == HR_D) // 1 - desatki chasov
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_D; // [0:2]
			if (time_mode == TIME_MODE_ALL)
				X_i = 0; // 0,1,2
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 4; // 4,5,6
			
		}
		if (F_Digit_Type == HR_E) // 2-cifra - edenici chasov {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_E; // [4:6]
			if (time_mode == TIME_MODE_ALL)
				X_i = 4; // 4,5,6
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 8; // 8,9,10
			
		}
		if (F_Digit_Type == MIN_D) // 3-cifra - desjatki minut {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_D; // [8:10]

			if (time_mode == TIME_MODE_ALL)
				X_i = 8; // 8,9,10
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 16; // 16,17,18			
		}
		if (F_Digit_Type == MIN_E) // 4-cifra - edinici minut {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_E; // [12:14]
			if (time_mode == TIME_MODE_ALL)
				X_i = 12; // 12,13,14
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 20; // 20,21,22			
			
		}
		if (F_Digit_Type == SEC_D) // 5-cifra - desjtki sekund {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_D; // [16:18]
			if (time_mode == TIME_MODE_ALL)
				X_i = 16; // 16,17,18
		}
		if (F_Digit_Type == SEC_E) // 6-cifra - edinici sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_E; // [20:22]
			if (time_mode == TIME_MODE_ALL)
				X_i = 20; // 20,21,22			
		}
		if (F_Digit_Type == MS_x100) // 7-cifra - desjatki mili sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MSEC_x100; // [24:26]
			if (time_mode == TIME_MODE_ALL)
				X_i = 24; // 24,25,26
			
		}
		if (F_Digit_Type == SEC_DOTS) // 8-cifra - sekundnie tochki ":"
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if(RTC_Time_pendulum == 1)
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_ON; // '11' address: [14]
			}
			else
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_OFF; // '12' address: [14]
			}
			if (time_mode == TIME_MODE_HR_MIN)
			X_i = 13; // 20,21,22
		}
		// create X_colors array
		switch (Current_Digit)
		{
			// Cifra 0 imeet 2 tochki v line_2 --> xi,xi+2
			case 0:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 1 imeet 1 tochku v line_2 --> x_i+1
			case 1:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue }; // ON
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 2 imeet 1 tochki v line_2 --> x_i
			case 2:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 3 imeet 1 tochki v line_1 --> x_i+2
			case 3:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 4 imeet 1 tochki v line_1 --> x_i+2
			case 4:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 5 imeet 1 tochki v line_1 --> x_i+2
			case 5:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 6 imeet 2 tochki v line_1 --> x_i,x_i+2
			case 6:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 7 imeet 1 tochki v line_1 --> x_i+2
			case 7:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 8 imeet 2 tochki v line_1 --> x_i,x_i+2
			case 8:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 9 imeet 1 tochki v line_1 --> x_i+2
			case 9:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// ON DOTS
			case 11:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// OFF DOTS
			case 12:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			default:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
		}
	}
	if (Line_Nr == 5) // bottom
	{
		if (F_Digit_Type == HR_D) // 1 - desatki chasov
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_D; // [0:2]
			if (time_mode == TIME_MODE_ALL)
				X_i = 0; // 0,1,2
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 4; // 4,5,6
			
		}
		if (F_Digit_Type == HR_E) // 2-cifra - edenici chasov {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_HR_E; // [4:6]
			if (time_mode == TIME_MODE_ALL)
				X_i = 4; // 4,5,6
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 8; // 8,9,10
			
		}
		if (F_Digit_Type == MIN_D) // 3-cifra - desjatki minut {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_D; // [8:10]

			if (time_mode == TIME_MODE_ALL)
				X_i = 8; // 8,9,10
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 16; // 16,17,18			
		}
		if (F_Digit_Type == MIN_E) // 4-cifra - edinici minut {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MIN_E; // [12:14]
			if (time_mode == TIME_MODE_ALL)
				X_i = 12; // 12,13,14
			if (time_mode == TIME_MODE_HR_MIN)
				X_i = 20; // 20,21,22			
			
		}
		if (F_Digit_Type == SEC_D) // 5-cifra - desjtki sekund {0:5}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_D; // [16:18]
			if (time_mode == TIME_MODE_ALL)
				X_i = 16; // 16,17,18
		}
		if (F_Digit_Type == SEC_E) // 6-cifra - edinici sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_SEC_E; // [20:22]
			if (time_mode == TIME_MODE_ALL)
				X_i = 20; // 20,21,22			
		}
		if (F_Digit_Type == MS_x100) // 7-cifra - desjatki mili sekund {0:9}
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			Current_Digit = RTC_Time_MSEC_x100; // [24:26]
			if (time_mode == TIME_MODE_ALL)
				X_i = 24; // 24,25,26
			
		}
		if (F_Digit_Type == SEC_DOTS) // 8-cifra - sekundnie tochki ":"
		{
			Red = Current_Red;
			Green = Current_Green;
			Blue = Current_Blue;
			//
			if(RTC_Time_pendulum == 1)
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_ON; // '11' address: [14]
			}
			else
			{
				Current_Digit = RTC_Time_MSEC_xDOTS_OFF; // '12' address: [14]
			}
			if (time_mode == TIME_MODE_HR_MIN)
			X_i = 13; // 20,21,22
		}
		// create X_colors array
		switch (Current_Digit)
		{
			// Cifra 0 imeet 3 tochki v line_2 --> xi,x_i+1,xi+2
			case 0:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 1 imeet 3 tochki v line_2 --> xi,x_i+1,xi+2
			case 1:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 2 imeet 3 tochki v line_2 --> xi,x_i+1,xi+2
			case 2:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 3 imeet 3 tochki v line_2 --> xi,x_i+1,xi+2
			case 3:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 4 imeet 1 tochki v line_1 --> x_i+2
			case 4:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 5 imeet 1 tochki v line_1 --> x_i+2
			case 5:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 6 imeet 1 tochki v line_1 --> x_i+1
			case 6:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
			// Cifra 7 imeet 1 tochki v line_1 --> x_i+2
			case 7:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 8 imeet 2 tochki v line_1 --> x_i,x_i+2
			case 8:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// Cifra 9 imeet 1 tochki v line_1 --> x_i+2
			case 9:
			X_colors[X_i] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+1] = (rgb_color){ Red, Green, Blue };// ON
			X_colors[X_i+2] = (rgb_color){ Red, Green, Blue };// ON
			break;
			// ON DOTS
			case 11:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			// OFF DOTS
			case 12:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };// OFF
			break;
			default:
			X_colors[X_i] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+1] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			X_colors[X_i+2] = (rgb_color){ BackGround_Red, BackGround_Green, BackGround_Blue };
			break;
		}
	}
}

// ---------------------------------------MAIN program ------------------------------------
int main (void)
{
	//--------------------------------------
	// 1 PIND7 -KEY4			BTN HARDWARE RESET
  	// 2 PINB0 -KEY1			BTN0 (MODE)
  	// 3 PIND5 -KEY2			BTN1 (ADJUST)
  	// 4 PIND6 -KEY3			BTN2 (COLLOR SET)
 	//--------------------------------------
// set to input
	DDRB = DDRB &~(1<<0);
	//
	DDRB = DDRB |(1<<1); // Set to output for calibration PINB1
	//
	DDRD = DDRD &~(1<<5);
	DDRD = DDRD &~(1<<6);
	DDRD = DDRD &~(1<<7);
// pull-up enable
	PORTB = PORTB |(1<<0);
	PORTD = PORTD |(1<<5);
	PORTD = PORTD |(1<<6);
	PORTD = PORTD |(1<<7);
	//--------------------------------------
	uint8_t time_mode = TIME_MODE_HR_MIN;
// info
// DDRn = 0 <-- input
	//PORTxn =0 -- Z condition
	//PORTxn =1 -- Pull_up enable
		//PINxn =0/1 (default setup)

// DDRn = 1 --> output (pull_up disabled)
	//PORTxn =0 -- out='0'
	//PORTxn =1 -- out='1'
//-----------------------------------
	OSCCAL = 128; // 128 kalibrovka RC-oscillator

//	short int SPI_start=0;
	//unsigned int SPI_DATA=0

		  // [76543210]
	LED_LINE1_STRIP_DDR = LED_LINE1_STRIP_DDR | (1<<LED_LINE1_STRIP_PIN); // SET to Output
	LED_LINE1_STRIP_PORT = LED_LINE1_STRIP_PORT &~(1<<LED_LINE1_STRIP_PIN); // RESET to 0

// -------------- Timer 2 initialization ---------------------------------------------
	// TIMER2 with prescaler clkT2S/1024
//	#define TIMER2_PRESCALER      (1 << CS22) | (1 << CS21) | (1 << CS20)
	#define TIMER2_PRESCALER					(1 << CS21)


	// TIMER2 output compare value
	// --> value 98 is 25.088ms (4MHz@1024 prescale factor)

	DDRB |= (1<<3);
	TIMSK |= (1 << OCIE2);                    // set output compare interrupt enable
	TCCR2 |= (1<< FOC2) |(1<< COM20)  | (1 << WGM21) | TIMER2_PRESCALER; // set CTC mode
	OCR2   = TIMER2_COMPARE_VALUE;            // set compare value for interrupt

// -------------- Timer 1 initialization ---------------------------------------------
//
//CS12 CS11 CS10 Description
//0 0 0 No clock source. (Timer/Counter stopped)
//0 0 1 clkI/O/1 (No prescaling)
//0 1 0 clkI/O/8 (From prescaler)
//0 1 1 clkI/O/64 (From prescaler)
//1 0 0 clkI/O/256 (From prescaler)
//1 0 1 clkI/O/1024 (From prescaler)

//
	TIMSK = TIMSK | (1<<2); // set mask TOIE1:=1 (COUNTER1)
	//TIMSK = TIMSK | (1<<0); // set mask TOIE0:=1 (COUNTER1)
	//TIMSK |= (1<<TOIE1); // set mask TOIE0:=1 (COUNTER0)
	TCNT1 = TIMER_CAL; // ???????? ???????????? ????????
	//TCNT0 = 0; // ???????? ???????????? ????????
//	TCCR1B = 0x03; // RUN TC1, TIMER MODE=3 (DIV 64)
	//TCCR1B = 0x05; // RUN TC1, TIMER MODE=5 (DIV 1024)
	//
	TCCR1B = 0x05; // RUN TC1, TIMER MODE=4 (DIV 256)
	//TCCR0 = 0x02; // RUN TC0, TIMER MODE=2 (DIV 8)
	//TCCR1B |= (1<<CS10); // RUN TC1, TIMER MODE=1 (DIV 1)
	//
	sei();
// -------------- Timer 1 initialization end ---------------------------------------------


	
//	SREG ^= 0x80; // enable all interrupt (I:=1)
	SREG |= 0x80;

	// init black colors
	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		all_LEDs_colors_OFF[i] = (rgb_color){ 0, 0, 0 };
	}

	// init colors array with black color
	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		X_colors[i] = (rgb_color){ 0, 0, 0 };
	}

	uint32_t CNT_MS_x100 = 0;
	//R_DIR = 1; 
	//G_DIR = 2;
	//B_DIR = 3;
	uint32_t CNT_BKLIGHT = 0;

	Prepare_Red = 0;
	Prepare_Green = 32;
	Prepare_Blue = 0;
	//
	Green_MAX_LIGHT = 32;
	Red_MAX_LIGHT = 1;
	Blue_MAX_LIGHT = 1;
	//
	R_DIR = 1;
	G_DIR = 1;
	B_DIR = 1;
	//
	ADC_Initialization(0);
	// main program
	while (1)
	{
		ADC_convert(0);
		//
		if (ADC_RESULT_H >= 50)
		{
			CNT_DELAY = 40;
			Red_MAX_Brightness = 4;
			Green_MAX_Brightness = 4;
			Blue_MAX_Brightness = 4;
		}
		if (ADC_RESULT_H >= 60)
		{
			CNT_DELAY = 20;
			Red_MAX_Brightness = 8;
			Green_MAX_Brightness = 8;
			Blue_MAX_Brightness = 8;
		}
		if (ADC_RESULT_H >= 80)
		{
			CNT_DELAY = 10;
			Red_MAX_Brightness = 16;
			Green_MAX_Brightness = 16;
			Blue_MAX_Brightness = 16;
		}
		if (ADC_RESULT_H >= 100)
		{
			CNT_DELAY = 7;
			Red_MAX_Brightness = 24;
			Green_MAX_Brightness = 24;
			Blue_MAX_Brightness = 24;
		}
		if (ADC_RESULT_H >= 120)
		{
			CNT_DELAY = 5;
			Red_MAX_Brightness = 32;
			Green_MAX_Brightness = 32;
			Blue_MAX_Brightness = 32;
		}
		//
		// Color Chose:
		// RED
		if (CMD_COLOR_SET == 1)
		{
			CMD_COLOR_SET++;
			Red_MAX_LIGHT = Red_MAX_Brightness;
			Green_MAX_LIGHT = 0;
			Blue_MAX_LIGHT = 0;
			//
			Prepare_Red = 32;
			Prepare_Green = 0;
			Prepare_Blue = 0;
			//
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;
		}
		// GREEN
		if (CMD_COLOR_SET == 3)
		{
			Red_MAX_LIGHT = 0;
			Green_MAX_LIGHT = Green_MAX_Brightness;
			Blue_MAX_LIGHT = 0;
			//
			Prepare_Red = 0;
			Prepare_Green = 32;
			Prepare_Blue = 0;
			CMD_COLOR_SET++;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;

		}
		// BLUE
		if (CMD_COLOR_SET == 5)
		{
			Red_MAX_LIGHT = 0;
			Green_MAX_LIGHT = 0;
			Blue_MAX_LIGHT = Blue_MAX_Brightness;
			//
			Prepare_Red = 0;
			Prepare_Green = 0;
			Prepare_Blue = 32;
			CMD_COLOR_SET++;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;
		}
		// ORANGE
		if (CMD_COLOR_SET == 7)
		{
			Red_MAX_LIGHT = Red_MAX_Brightness;
			Green_MAX_LIGHT = Green_MAX_Brightness;
			Blue_MAX_LIGHT = 0;
			//
			Prepare_Red = 32;
			Prepare_Green = 32;
			Prepare_Blue = 0;
			CMD_COLOR_SET++;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;
		}
		// VIOLET
		if (CMD_COLOR_SET == 9)
		{
			Red_MAX_LIGHT = Red_MAX_Brightness;
			Green_MAX_LIGHT = 0;
			Blue_MAX_LIGHT = Blue_MAX_Brightness;
			//
			Prepare_Red = 32;
			Prepare_Green = 0;
			Prepare_Blue = 32;
			CMD_COLOR_SET++;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;
		}
		// CYAN
		if (CMD_COLOR_SET == 11)
		{
			Red_MAX_LIGHT = 0;
			Green_MAX_LIGHT = Green_MAX_Brightness;
			Blue_MAX_LIGHT = Blue_MAX_Brightness;
			//
			Prepare_Red = 0;
			Prepare_Green = 32;
			Prepare_Blue = 32;
			CMD_COLOR_SET++;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;
		}
		if (CMD_COLOR_SET == 13)
		{
			Red_MAX_LIGHT = Red_MAX_Brightness;
			Green_MAX_LIGHT = Green_MAX_Brightness;
			Blue_MAX_LIGHT = Blue_MAX_Brightness;
			//
			Prepare_Red = 32;
			Prepare_Green = 32;
			Prepare_Blue = 32;
			CMD_COLOR_SET = 16;
			R_DIR = 1;
			G_DIR = 1;
			B_DIR = 1;

		}
		//if (CMD_COLOR_SET == 15)
		//{
			//Red_MAX_LIGHT = ADC_RESULT_H;
			//Green_MAX_LIGHT = ADC_RESULT_H;
			//Blue_MAX_LIGHT = ADC_RESULT_H;
			////
			//Prepare_Red = 32;
			//Prepare_Green = 32;
			//Prepare_Blue = 32;
			//CMD_COLOR_SET++;
			//R_DIR = 1;
			//G_DIR = 1;
			//B_DIR = 1;
		//}
		if (CMD_COLOR_SET == 17)
		{
			// Start Rainbow Colors
			Prepare_Red = 128;
			Prepare_Green = 0;
			Prepare_Blue = 0;
			CMD_COLOR_SET++;
			State_m = 0;
		}
		if (CMD_COLOR_SET == 18)
		{
			CNT_DELAY = 7;
			// Rainbow Colors
			if(CNT_BKLIGHT >= CNT_DELAY)
			{
				CNT_BKLIGHT = 0;
				//------------------------
				// RED
				
				
				// Step 1.(Red), RGB=128,0,0
				if ((Prepare_Red >= 51) & (State_m == 0))
				{
					Prepare_Red--;
				}
				else
				{
					if (State_m == 0)
						State_m = 1;
				}
				//
				// Step 2.(weak Red) RGB=50,0,0
				if ((Prepare_Green <= 31) & (State_m == 1))
				{
					Prepare_Green++;
				}
				else
				{
					if (State_m == 1)
						State_m = 2;
				}
				//
				// Step 3.(Orange) R=50,G=32,B=0
				if ((Prepare_Red >= 33) & (State_m == 2))
				{
					Prepare_Red--;
				}
				else
				{
					if (State_m == 2)
						State_m = 3;
				}
				//
				// Step 4.(Red-Yellow) R=32,G=32,B=0
				if ((Prepare_Green <= 49) & (State_m == 3))
				{
					Prepare_Green++;
				}
				else
				{
					if (State_m == 3)
						State_m = 4;
				}
				//
				// Step 5.(Yellow) R=32,G=50,B=0
				if ((Prepare_Red >= 1) & (State_m == 4))
				{
					Prepare_Red--;
				}
				else
				{
					if (State_m == 4)
						State_m = 5;
				}
				//
				// Step 6.(Green) R=0,G=50,B=0
				if ((Prepare_Blue <= 31) & (State_m == 5))
				{
					Prepare_Blue++;
				}
				else
				{
					if (State_m == 5)
						State_m = 6;
				}
				//
				// Step 7.(Green-Blue) R=0,G=50,B=32
				if ((Prepare_Green >= 33) & (State_m == 6))
				{
					Prepare_Green--;
				}
				else
				{
					if (State_m == 6)
						State_m = 7;
				}
				//
				// Step 8.(Green-Blue) R=0,G=32,B=32
				if ((Prepare_Blue <= 49) & (State_m == 7))
				{
					Prepare_Blue++;
				}
				else
				{
					if (State_m == 7)
						State_m = 8;
				}
				//
				// Step 9.(Green-Blue) R=0,G=32,B=50
				if ((Prepare_Green >= 1) & (State_m == 8))
				{
					Prepare_Green--;
				}
				else
				{
					if (State_m == 8)
						State_m = 9;
				}
				//
				// Step 10.(Blue) R=0,G=0,B=50
				if ((Prepare_Red <= 31) & (State_m == 9))
				{
					Prepare_Red++;
				}
				else
				{
					if (State_m == 9)
						State_m = 10;
				}
				//
				// Step 11.(Blue-Red) R=32,G=0,B=50
				if ((Prepare_Blue >= 33) & (State_m == 10))
				{
					Prepare_Blue--;
				}
				else
				{
					if (State_m == 10)
						State_m = 11;
				}
				//
				// Step 12.(Blue-Red) R=32,G=0,B=32
				if ((Prepare_Blue >= 1) & (State_m == 11))
				{
					Prepare_Blue--;
				}
				else
				{
					if (State_m == 11)
						State_m = 12;
				}
				//
				// Step 13.(Red) R=32,G=0,B=0
				if ((Prepare_Red <= 63) & (State_m == 12))
				{
					Prepare_Red++;
				}
				else
				{
					if (State_m == 12)
						State_m = 0;
				}
				//------------------------
			}
			CNT_BKLIGHT++;
			// End Rainbow Light
		}
		else
		{
			// IF others and NOT 18
			//
			// Soft Back lighting Clock time
			CNT_BKLIGHT++;
			if(CNT_BKLIGHT >= CNT_DELAY)
			{
				CNT_BKLIGHT = 0;
				//------------------------
				// RED
				
				Prepare_Red = Prepare_Red + R_DIR;
				
				if (Red_MAX_LIGHT == 0)
				{
					R_DIR = 0;
				}
				else
				{
					if (Prepare_Red >= Red_MAX_LIGHT)
					R_DIR = -1;
					else
					if(Prepare_Red <= 1)
					R_DIR = 1;
				}
				//------------------------
				// GREEN
				Prepare_Green = Prepare_Green + G_DIR;

				if (Green_MAX_LIGHT == 0)
				{
					G_DIR = 0;
				}
				else
				{
					if (Prepare_Green >= Green_MAX_LIGHT)
					G_DIR = -1;
					else
					if(Prepare_Green <= 1)
					G_DIR = 1;
				}
				//------------------------
				//------------------------
				// BLUE
				Prepare_Blue = Prepare_Blue + B_DIR;
				
				if (Blue_MAX_LIGHT == 0)
				{
					B_DIR = 0;
				}
				else
				{
					if (Prepare_Blue >= Blue_MAX_LIGHT)
					B_DIR = -1;
					else
					if(Prepare_Blue <= 1)
					B_DIR = 1;
				}
			}
			// End Soft Back lighting Clock time
		}
		//----------------------------------------------
		if (CMD_RUN_RTC_unit_pendulum >= 1)
		{
			CMD_RUN_RTC_unit_pendulum--;
			
			//
			
			//
			//if (SEC_FLOAT >= COUNT_TO)
			//{
			//	SEC_FLOAT = SEC_FLOAT - COUNT_TO;
				//PORTB = PORTB ^ (1 << 1); // toggle RB1 (RTC Pendulum LED) for Calibration by Saleae16
				RTC_unit_pendulum_v2_RGB_LED_Clock();
			//}
			//
			//
		}
		//
		

		CNT_MS_x100++;

		BUTTONS_FUNCTIONS ();
		//
		for(uint8_t Line=1; Line <=5; Line++)
		{
				Current_Red = Prepare_Red;
				Current_Green = Prepare_Green;
				Current_Blue = Prepare_Blue;
			for(uint8_t Digit_Type = 1; Digit_Type <= 5; Digit_Type++)
			{
				time_mode = TIME_MODE_HR_MIN;
				// Setup Time:
				if (CMD_MODE == 0)
				{
					digit_view_mode = 0;
				}
				if (CMD_MODE == 1)
				{
					if (Digit_Type == 1)
					{
						digit_view_mode = 1;
					}
					else
					{
						digit_view_mode = 0;
					}
				}
				//
				if (CMD_MODE == 2)
				{
					if (Digit_Type == 2)
					{
						digit_view_mode = 1;
					}
					else
					{
						digit_view_mode = 0;
					}
				}
				if (CMD_MODE == 3)
				{
					if (Digit_Type == 4)
					{
						digit_view_mode = 1;
					}
					else
					{
						digit_view_mode = 0;
					}
				}
				if (CMD_MODE == 4)
				{
					if (Digit_Type == 5)
					{
						digit_view_mode = 1;
					}
					else
					{
						digit_view_mode = 0;
					}
				}
				Function_Create_Lines_by_RTC (time_mode, Line, Digit_Type, digit_view_mode);
			}
			led_Line_strip_write(X_colors, LED_COUNT,Line);
			_delay_us (90);
			led_Line_strip_write(all_LEDs_colors_OFF, LED_COUNT,Line);
			_delay_us (10);
		}
		//
	} // ----- end while (1) -----	
}

//ISR(TIMER1_OVF_vect, ISR_NAKED)

ISR(TIMER1_OVF_vect)
{
//-->	//cli(); // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
//-->	TIFR = TIFR | (1<<2); // clear TOV1 bit
	//	TIMSK = TIMSK & ~(1 <<2); // clear mask TOIE1:=0
	//TCCR1B = 0x04; // 4 = div 256
	//TCCR1B = 0x01; // 5 = div 1024
	TCNT1 = TIMER_CAL;
	//TCCR1B = 0x00; // STOP Timer 1
	//TCNT1L = 0;
	//TCNT1H = 255;
	//TCCR1B = 0x01; // RUN TC1, TIMER MODE=4 (DIV 256)
	//CMD_RUN_RTC_unit_pendulum = 1;
	//
	//CMD_RUN_RTC_unit_pendulum++;

//-->	//sei();          // Re-enable interrupts now that we are done.
	//reti();
	
	SEC_FLOAT = SEC_FLOAT + TIM1_ONE_STEP;
	if (SEC_FLOAT >= 0.5)
	{
		SEC_FLOAT = SEC_FLOAT - 0.5;
		PORTB = PORTB ^ (1 << 1); // toggle RB1 (RTC Pendulum LED) for Calibration by Saleae16
		CMD_RUN_RTC_unit_pendulum++;
	}
}


/*
ISR(TIMER0_OVF_vect)
{
	//-->	//cli(); // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
	//-->	TIFR = TIFR | (1<<2); // clear TOV1 bit
	//	TIMSK = TIMSK & ~(1 <<2); // clear mask TOIE1:=0
	//TCCR1B = 0x04; // 4 = div 256
	//TCCR1B = 0x01; // 5 = div 1024
	//TCNT0 = 5;
	//TCCR1B = 0x00; // STOP Timer 1
	//TCNT1L = 0;
	//TCNT1H = 255;
	//TCCR1B = 0x01; // RUN TC1, TIMER MODE=4 (DIV 256)
	//CMD_RUN_RTC_unit_pendulum = 1;
	CNTx32BIT++;
	PORTB ^= (1<<1);
	//-->	//sei();          // Re-enable interrupts now that we are done.
	//reti();
}
*/

ISR(TIMER2_COMP_vect)
{
	//cli();
	
	//CMD_RUN_RTC_unit_pendulum++;
	//SEC_FLOAT = SEC_FLOAT + TIM1_ONE_STEP;
	//CMD_RUN_RTC_unit_pendulum = 0;
	//SEC_FLOAT = SEC_FLOAT + CORRECTION;
	//sei();
}
