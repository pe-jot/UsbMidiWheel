#include "UsbMidiWheel.h"
#include <stdlib.h>


/* Based on:
 * Reading the encoder switch - http://web.engr.oregonstate.edu/%7Etraylor/ece473/student_projects/ReadingEncoderSwitches.pdf
 */
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 2
const int8_t encoderStates[] = { 0, 1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1, 0 };

static volatile uint8_t cmdReadEncoder = 0;
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
		uint8_t MIDIPitch;
		
		if (gCommand.Enter != PUSHBUTTON_NONE)
		{
			MIDICommand = (gCommand.Enter == PUSHBUTTON_PRESSED) ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF;
			MIDIPitch = 1;
			gCommand.Enter = PUSHBUTTON_NONE;
		}
		else if (gCommand.Rotation != ENCODER_NONE)
		{
			MIDICommand = MIDI_COMMAND_PROGRAM_CHANGE;
			MIDIPitch = (gCommand.Rotation & 0x7F);
			gCommand.Rotation = ENCODER_NONE;
		}

		/* Check if a MIDI command is to be sent */
		if (MIDICommand)
		{
			MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
				{
					.Event       = MIDI_EVENT(0, MIDICommand),

					.Data1       = MIDICommand,
					.Data2       = MIDIPitch,
					.Data3       = MIDI_STANDARD_VELOCITY,
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

ISR(INT0_vect)
{
	cmdReadEncoder = 1;
}


ISR(TIMER1_OVF_vect) // 10ms Timer
{
	TCNT1 = TIMER_PRELOAD_10MS;
	uint8_t tempSREG = SREG;
	static uint8_t encoderButtonHistory = 0;
	
	/* Button debouncing - based on:
	 * Elliot Williams - Debounce Your Noisy Buttons, Part II - http://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
	 */
	encoderButtonHistory <<= 1;
	encoderButtonHistory |= ENCODER_SW_PIN_PRESSED;
	
	if ((encoderButtonHistory & 0b11000111) == 0b00000111)
	{
		/* Encoder button pressed... */
		gCommand.Enter = PUSHBUTTON_PRESSED;
		encoderButtonHistory = 0b11111111;
	}
	else if ((encoderButtonHistory & 0b11100011) == 0b11100000)
	{
		/* Encoder button released... */
		gCommand.Enter = PUSHBUTTON_RELEASED;
		encoderButtonHistory = 0b00000000;
	}
	
	SREG = tempSREG;
}


/***********************************************************************************************************************************************************************/
/*                  !!! This project is currently configured for Arduino Micro @ 16 MHz clock !!! Hardware change needs change of F_CPU and F_USB !!!                  */
/***********************************************************************************************************************************************************************/

int main(void)
{
	uint8_t encoderValue = 0;
	uint8_t encoderCount = 0;
	uint8_t encoderDirection = 0;
	
	memset((void*)&gCommand, 0, sizeof(Command_type));
	
	/* Configure I/O Ports */
    DDRB |= (1<<0); /* PB0 = RXLED */
    PORTB = (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5);
    DDRC |= (1<<7); /* PC7 = USER_LED */
    PORTC = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6);
    DDRD |= (1<<5); /* PD5 = TXLED */
    PORTD = (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<6) | (1<<7);
	
	/* Start Bootloader if µC is started via reset button */
	if (MCUSR & (1 << EXTRF) || (ENCODER_SW_PIN_PRESSED))
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
	
	/* INT0 on any edge for encoder reading - Encoder_A creates an interrupt (B is being polled) */
	EICRA = (1 << ISC00);
	EIMSK = (1 << INT0);
	
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
		if (cmdReadEncoder)
		{
			// Get 2 new encoder bits
			encoderValue |= (ENCODER_B_VALUE | ENCODER_A_VALUE);
			encoderDirection = encoderStates[(encoderValue) & 0xF];
					
			if (encoderDirection == CLOCKWISE)
			{
				encoderCount++;
			}
			else if (encoderDirection == COUNTERCLOCKWISE)
			{
				encoderCount--;
			}
					
			if ((encoderValue & 3) == 3)
			{
				/* Stretch valid limits for debouncing reasons */
				if ((encoderCount >= 0x01) && (encoderCount < 0x70))
				{
					/* Clockwise turn... */
					gCommand.Rotation = 1;
				}
				else if ((encoderCount <= 0xFF) && (encoderCount > 0x90))
				{
					/* Counterclockwise turn... */
					gCommand.Rotation = -1;
				}
				encoderCount = 0;
				cmdReadEncoder = 0;
			}
					
			/* Make place for the next 2 encoder bits */
			encoderValue = (encoderValue << 2) & 0xFC;
		}
		
		MIDI_Task();
		USB_USBTask();
    }
}
