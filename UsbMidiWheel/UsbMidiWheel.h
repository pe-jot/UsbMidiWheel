#pragma once

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

/* Timer preload values for 16 MHz /64 */
#define TIMER_PRELOAD_10MS		0xF63C
#define TIMER_PRELOAD_20MS		0xEC78
#define TIMER_PRELOAD_50MS		0xCF2C
#define TIMER_PRELOAD_100MS		0x9E58

#define RX_LED_PIN		(1 << PB0)
#define RX_LED_ON		PORTB |=  LED_RX_PIN 
#define RX_LED_OFF		PORTB &= ~LED_RX_PIN
#define RX_LED_TOGGLE	PORTB ^=  LED_RX_PIN

#define TX_LED_PIN		(1 << PD5)
#define TX_LED_ON		PORTD |=  LED_TX_PIN
#define TX_LED_OFF		PORTD &= ~LED_TX_PIN
#define TX_LED_TOGGLE	PORTD ^=  LED_TX_PIN

/* User LED is high-side driven */
#define USER_LED_PIN	(1 << PC7)
#define USER_LED_ON		PORTC |=  USER_LED_PIN
#define USER_LED_OFF	PORTC &= ~USER_LED_PIN
#define USER_LED_TOGGLE	PORTC ^=  USER_LED_PIN

/* Inputs have internal pull-up resistor enabled */
#define ENCODER1_A_PIN				(1 << PB2)	/* MOSI */
#define ENCODER1_B_PIN				(1 << PD3)	/* D1/TX */
#define ENCODER1_A_VALUE			((PINB & ENCODER1_A_PIN) ? 0 : 0x01)
#define ENCODER1_B_VALUE			((PIND & ENCODER1_B_PIN) ? 0 : 0x02)
#define ENCODER1_SW_PIN				(1 << PD2)	/* D0/RX */
#define ENCODER1_SW_PIN_PRESSED		(!(PIND & ENCODER1_SW_PIN))

#define ENCODER2_A_PIN				(1 << PD0)	/* D3/SCL */
#define ENCODER2_B_PIN				(1 << PD1)	/* D2/SDA */
#define ENCODER2_A_VALUE			((PIND & ENCODER2_A_PIN) ? 0 : 0x01)
#define ENCODER2_B_VALUE			((PIND & ENCODER2_B_PIN) ? 0 : 0x02)
#define ENCODER2_SW_PIN				(1 << PD4)	/* D4 */
#define ENCODER2_SW_PIN_PRESSED		(!(PIND & ENCODER2_SW_PIN))

#define BUTTONGND_PIN				(1 << PB5)	/* D9 */
#define BUTTON1_PIN					(1 << PC6)	/* D5 */
#define BUTTON1_PRESSED				(!(PINC & BUTTON1_PIN))
#define BUTTON2_PIN					(1 << PD7)	/* D6 */
#define BUTTON2_PRESSED				(!(PIND & BUTTON2_PIN))
#define BUTTON3_PIN					(1 << PE6)	/* D7 */
#define BUTTON3_PRESSED				(!(PINE & BUTTON3_PIN))
#define BUTTON4_PIN					(1 << PB4)	/* D8 */
#define BUTTON4_PRESSED				(!(PINB & BUTTON4_PIN))

#define UNUSED_SCK_PIN				(1 << PB1)
#define UNUSED_MISO_PIN				(1 << PB3)
#define UNUSED_IO12_PIN				(1 << PC6)
#define UNUSED_A5_PIN				(1 << PF0)
#define UNUSED_A4_PIN				(1 << PF1)
#define UNUSED_A3_PIN				(1 << PF4)
#define UNUSED_A2_PIN				(1 << PF5)
#define UNUSED_A1_PIN				(1 << PF6)
#define UNUSED_A0_PIN				(1 << PF7)

typedef enum EncoderState {
	ENCODER_NONE = 0,
	ENCODER_CLOCKWISE = 1,
	ENCODER_COUNTERCLOCKWISE = -1
} EncoderState;
	
typedef enum PushbuttonState {
	PUSHBUTTON_NONE = 0,
	PUSHBUTTON_PRESSED = 1,
	PUSHBUTTON_RELEASED = -1
} PushbuttonState;

typedef struct Command_type 
{
	EncoderState Encoder1_Rotation;
	PushbuttonState Encoder1_Button;
	
	EncoderState Encoder2_Rotation;
	PushbuttonState Encoder2_Button;
	
	PushbuttonState Button1;
	PushbuttonState Button2;
	PushbuttonState Button3;
	PushbuttonState Button4;
} Command_type;

void EVENT_USB_Device_Connect (void);
void EVENT_USB_Device_Disconnect (void);
void EVENT_USB_Device_ConfigurationChanged (void);
