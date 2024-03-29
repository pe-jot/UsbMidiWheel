#include "UsbMidiWheel.h"
#include <stdlib.h>


/* Based on:
 * Reading the encoder switch - http://web.engr.oregonstate.edu/%7Etraylor/ece473/student_projects/ReadingEncoderSwitches.pdf
 */
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 2
const int8_t encoderStates[] = { 0, 1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1, 0 };

static volatile uint8_t cmdReadEncoder1 = 0;
static volatile uint8_t cmdReadEncoder2 = 0;
static volatile uint8_t cmdReadEncoder3 = 0;
static volatile Command_type gCommand;


/* Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) 
{}


/* Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{}


/* Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;
	
	ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_IN_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_OUT_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);
}


/* Task to handle the generation of MIDI note change events and send them to the host. */
void MIDI_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	{
		return;
	}

	Endpoint_SelectEndpoint(MIDI_STREAM_IN_EPADDR);

	if (Endpoint_IsINReady())
	{
		uint8_t MIDICommand = 0;
		uint8_t DataByte1;
		uint8_t DataByte2 = MIDI_STANDARD_VELOCITY;
		
		/* Encoder 1 */
		if (gCommand.Encoder1_Button != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Encoder1_Button == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 1;
			gCommand.Encoder1_Button = PUSHBUTTON_NONE;
		}
		else if (gCommand.Encoder1_Rotation != ENCODER_NONE)
		{
			/* SDR Console seems to expect Control Change commands as sent by the Behringer CMD PL-1 */
			/* Data spec. see: https://www.midi.org/specifications-old/item/table-3-control-change-messages-data-bytes-2 */
			MIDICommand = MIDI_COMMAND_CONTROL_CHANGE;
			DataByte1 = 0x10; /* General Purpose Controller 1 */
			DataByte2 = (gCommand.Encoder1_Rotation & 0x7F);
			gCommand.Encoder1_Rotation = ENCODER_NONE;
		}
		
		/* Encoder 2 */
		else if (gCommand.Encoder2_Button != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Encoder2_Button == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 2;
			gCommand.Encoder2_Button = PUSHBUTTON_NONE;
		}
		else if (gCommand.Encoder2_Rotation != ENCODER_NONE)
		{
			MIDICommand = MIDI_COMMAND_CONTROL_CHANGE;
			DataByte1 = 0x11; /* General Purpose Controller 2 */
			DataByte2 = (gCommand.Encoder2_Rotation & 0x7F);
			gCommand.Encoder2_Rotation = ENCODER_NONE;
		}
		
		/* Encoder 3 */
		else if (gCommand.Encoder3_Button != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Encoder3_Button == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 3;
			gCommand.Encoder3_Button = PUSHBUTTON_NONE;
		}
		else if (gCommand.Encoder3_Rotation != ENCODER_NONE)
		{
			MIDICommand = MIDI_COMMAND_CONTROL_CHANGE;
			DataByte1 = 0x12; /* General Purpose Controller 3 */
			DataByte2 = (gCommand.Encoder3_Rotation & 0x7F);
			gCommand.Encoder3_Rotation = ENCODER_NONE;
		}
		
		/* Buttons */
		else if (gCommand.Button1 != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Button1 == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 11;
			gCommand.Button1 = PUSHBUTTON_NONE;
		}
		else if (gCommand.Button2 != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Button2 == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 12;
			gCommand.Button2 = PUSHBUTTON_NONE;
		}
		else if (gCommand.Button3 != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Button3 == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 13;
			gCommand.Button3 = PUSHBUTTON_NONE;
		}
		else if (gCommand.Button4 != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Button4 == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			DataByte1 = 14;
			gCommand.Button4 = PUSHBUTTON_NONE;
		}

		/* Check if a MIDI command is to be sent */
		if (MIDICommand)
		{
			MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t) {
				.Event       = MIDI_EVENT(0, MIDICommand),
				.Data1       = MIDICommand,
				.Data2       = DataByte1,
				.Data3       = DataByte2,
			};

			/* Write the MIDI event packet to the endpoint */
			Endpoint_Write_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);

			/* Send the data in the endpoint to the host */
			Endpoint_ClearIN();
		}
	}

	/* Select the MIDI OUT stream */
	Endpoint_SelectEndpoint(MIDI_STREAM_OUT_EPADDR);

	/* Check if a MIDI command has been received */
	if (Endpoint_IsOUTReceived())
	{
		MIDI_EventPacket_t MIDIEvent;

		/* Read the MIDI event packet from the endpoint */
		Endpoint_Read_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);

		/* If the endpoint is now empty, clear the bank */
		if (!(Endpoint_BytesInEndpoint()))
		{
			/* Clear the endpoint ready for new packet */
			Endpoint_ClearOUT();
		}
	}
}

/***********************************************************************************************************************************************************************/

ISR(INT1_vect)
{
	cmdReadEncoder3 = 1;
}


ISR(INT2_vect)
{
	cmdReadEncoder2 = 1;
}


ISR(INT3_vect)
{
	cmdReadEncoder1 = 1;
}


inline static void ReadDebounced(volatile uint8_t* history, volatile PushbuttonState* action, uint8_t inputState)
{
	/* Button debouncing - based on:
	 * Elliot Williams - Debounce Your Noisy Buttons, Part II - http://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
	 */
	
	*history <<= 1;
	*history |= inputState;
	if ((*history & 0b11000111) == 0b00000111)
	{
		/* Button pressed... */
		*action = PUSHBUTTON_PRESSED;
		*history = 0b11111111;
	}
	else if ((*history & 0b11100011) == 0b11100000)
	{
		/* Button released... */
		*action = PUSHBUTTON_RELEASED;
		*history = 0b00000000;
	}	
}


ISR(TIMER1_OVF_vect) /* 10ms Timer */
{
	TCNT1 = TIMER_PRELOAD_10MS;
	uint8_t tempSREG = SREG;
	
	static uint8_t encoder1ButtonHistory = 0;
	static uint8_t encoder2ButtonHistory = 0;
	static uint8_t encoder3ButtonHistory = 0;
	static uint8_t button1History = 0;
	static uint8_t button2History = 0;
	static uint8_t button3History = 0;
	static uint8_t button4History = 0;
	
	ReadDebounced(&encoder1ButtonHistory, &(gCommand.Encoder1_Button), ENCODER1_SW_PIN_PRESSED);
	ReadDebounced(&encoder2ButtonHistory, &(gCommand.Encoder2_Button), ENCODER2_SW_PIN_PRESSED);
	ReadDebounced(&encoder3ButtonHistory, &(gCommand.Encoder3_Button), ENCODER3_SW_PIN_PRESSED);
	ReadDebounced(&button1History, &(gCommand.Button1), BUTTON1_PRESSED);
	ReadDebounced(&button2History, &(gCommand.Button2), BUTTON2_PRESSED);
	ReadDebounced(&button3History, &(gCommand.Button3), BUTTON3_PRESSED);
	ReadDebounced(&button4History, &(gCommand.Button4), BUTTON4_PRESSED);
	
	SREG = tempSREG;
}


/***********************************************************************************************************************************************************************/
/*                  !!! This project is currently configured for Arduino Micro @ 16 MHz clock !!! Hardware change needs change of F_CPU and F_USB !!!                  */
/***********************************************************************************************************************************************************************/

int main(void)
{
	uint8_t encoder1Value = 0;
	uint8_t encoder1Count = 0;
	uint8_t encoder1Direction = 0;
	uint8_t encoder2Value = 0;
	uint8_t encoder2Count = 0;
	uint8_t encoder2Direction = 0;
	uint8_t encoder3Value = 0;
	uint8_t encoder3Count = 0;
	uint8_t encoder3Direction = 0;
	
	memset((void*)&gCommand, 0, sizeof(Command_type));
	
	/* Configure I/O Ports */
    DDRB |= RX_LED_PIN | LED3_PIN | LED2_PIN | LED1_PIN;
    PORTB = UNUSED_SCK_PIN | UNUSED_MISO_PIN | BUTTON4_PIN | ENCODER2_SW_PIN;
	
    DDRC |= USER_LED_PIN;
    PORTC = BUTTON1_PIN;
	
    DDRD |= TX_LED_PIN | LED4_PIN;
    PORTD = BUTTON2_PIN | ENCODER2_B_PIN | ENCODER1_A_PIN | ENCODER2_A_PIN | ENCODER3_A_PIN | ENCODER1_B_PIN;
	
	DDRE = 0;
	PORTE = BUTTON3_PIN;
	
	DDRF = LED8_PIN | LED7_PIN | LED6_PIN | LED5_PIN;
	PORTF = ENCODER3_SW_PIN | ENCODER3_B_PIN;
	
	// #TODO: Enter bootloader if any HW button is being pressed during startup & switch on some HW LEDs.
	/* Start Bootloader if �C is started via reset button */
	if (MCUSR & (1 << EXTRF))
	{
		USER_LED_ON;
		/*
		 * Bootloader section starts at address 0x3800 (as configured in fuses)
		 * BOOTRST fuse is unprogrammed to directly start into application
		 */
		asm volatile ("jmp 0x3800");
	}

	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	
	/* INT1, INT2, INT3 on any edge for encoder reading - Encoder_A creates an interrupt (B is being polled) */
	EICRA = (1 << ISC30) | (1 << ISC20) | (1 << ISC10);
	EIMSK = (1 << INT3) | (1 << INT2) | (1 << INT1);
	
	/* Timer1 at fosc/64 */
	TCNT1 = TIMER_PRELOAD_10MS;
	TCCR1B = (3 << CS10);
	TIMSK1 = (1 << TOIE1);

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	USB_Init();
	GlobalInterruptEnable();
	
    while(1)
    {
		if (cmdReadEncoder1)
		{
			/* Get 2 new encoder bits */
			encoder1Value |= (ENCODER1_B_VALUE | ENCODER1_A_VALUE);
			encoder1Direction = encoderStates[(encoder1Value) & 0xF];
					
			if (encoder1Direction == CLOCKWISE)
			{
				encoder1Count++;
			}
			else if (encoder1Direction == COUNTERCLOCKWISE)
			{
				encoder1Count--;
			}
					
			if ((encoder1Value & 3) == 3)
			{
				/* Stretch valid limits for debouncing reasons */
				if ((encoder1Count >= 0x01) && (encoder1Count < 0x70))
				{
					/* Clockwise turn... */
					gCommand.Encoder1_Rotation = ENCODER_CLOCKWISE;
				}
				else if ((encoder1Count <= 0xFF) && (encoder1Count > 0x90))
				{
					/* Counterclockwise turn... */
					gCommand.Encoder1_Rotation = ENCODER_COUNTERCLOCKWISE;
				}
				encoder1Count = 0;
				cmdReadEncoder1 = 0;
			}
					
			/* Make place for the next 2 encoder bits */
			encoder1Value = (encoder1Value << 2) & 0xFC;
		}

		if (cmdReadEncoder2)
		{
			/* Get 2 new encoder bits */
			encoder2Value |= (ENCODER2_B_VALUE | ENCODER2_A_VALUE);
			encoder2Direction = encoderStates[(encoder2Value) & 0xF];
			
			if (encoder2Direction == CLOCKWISE)
			{
				encoder2Count++;
			}
			else if (encoder2Direction == COUNTERCLOCKWISE)
			{
				encoder2Count--;
			}
			
			if ((encoder2Value & 3) == 3)
			{
				/* Stretch valid limits for debouncing reasons */
				if ((encoder2Count >= 0x01) && (encoder2Count < 0x70))
				{
					/* Clockwise turn... */
					gCommand.Encoder2_Rotation = ENCODER_CLOCKWISE;
				}
				else if ((encoder2Count <= 0xFF) && (encoder2Count > 0x90))
				{
					/* Counterclockwise turn... */
					gCommand.Encoder2_Rotation = ENCODER_COUNTERCLOCKWISE;
				}
				encoder2Count = 0;
				cmdReadEncoder2 = 0;
			}
			
			/* Make place for the next 2 encoder bits */
			encoder2Value = (encoder2Value << 2) & 0xFC;
		}

		if (cmdReadEncoder3)
		{
			/* Get 2 new encoder bits */
			encoder3Value |= (ENCODER3_B_VALUE | ENCODER3_A_VALUE);
			encoder3Direction = encoderStates[(encoder3Value) & 0xF];
		
			if (encoder3Direction == CLOCKWISE)
			{
				encoder3Count++;
			}
			else if (encoder3Direction == COUNTERCLOCKWISE)
			{
				encoder3Count--;
			}
		
			if ((encoder3Value & 3) == 3)
			{
				/* Stretch valid limits for debouncing reasons */
				if ((encoder3Count >= 0x01) && (encoder3Count < 0x70))
				{
					/* Clockwise turn... */
					gCommand.Encoder3_Rotation = ENCODER_CLOCKWISE;
				}
				else if ((encoder3Count <= 0xFF) && (encoder3Count > 0x90))
				{
					/* Counterclockwise turn... */
					gCommand.Encoder3_Rotation = ENCODER_COUNTERCLOCKWISE;
				}
				encoder3Count = 0;
				cmdReadEncoder3 = 0;
			}
		
			/* Make place for the next 2 encoder bits */
			encoder3Value = (encoder3Value << 2) & 0xFC;
		}
		
		/* PTT - operate ON-AIR LED */
		if (gCommand.Button4 == PUSHBUTTON_PRESSED)
		{
			LED4_ON;
		}
		else
		{
			LED4_OFF;
		}
		
		MIDI_Task();
		USB_USBTask();
    }
}
