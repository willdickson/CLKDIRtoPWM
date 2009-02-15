/*
	CLKDIRtoPWM 

    who when        what
    --- ----        ----
    pjp 06/24/08    version 1.0
*/
/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/
#define INCLUDE_FROM_CLKDIRTOPWM_C
#include "CLKDIRtoPWM.h"

/* Project Tags, for reading out using the ButtLoad project */
BUTTLOADTAG(ProjName,     "CLKDIRtoPWM");
BUTTLOADTAG(BuildTime,    __TIME__);
BUTTLOADTAG(BuildDate,    __DATE__);
BUTTLOADTAG(MyUSBVersion, "MyUSB V" MYUSB_VERSION_STRING);

/* Scheduler Task List */
TASK_LIST
{
	{ Task: USB_USBTask          , TaskStatus: TASK_STOP },    { Task: USB_ProcessPacket    , TaskStatus: TASK_STOP },
};

/* DFU Bootloader Declarations */
uint32_t  boot_key __attribute__ ((section (".noinit")));
void (*start_bootloader) (void)=(void (*)(void))0xf000;

int main(void)
{
    /* After reset start bootloader? */
    if ((AVR_IS_WDT_RESET()) && (boot_key == DFU_BOOT_KEY_VAL)) 
    { 
        boot_key = 0;
        (*start_bootloader)();           
    } 

	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable Clock Division */
	SetSystemClockPrescaler(0);

	/* Hardware Initialization */
	LEDs_Init();
	
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
	
	/* Initialize Scheduler so that it can be used */
	Scheduler_Init();

	/* Initialize USB Subsystem */
	USB_Init();

    /* Initialize ServoArray Address data */
    Servo_Address_Init();

    /* Initialize ServoArray PulseWidth data */
    Servo_PulseWidth_Init();

    /* Initialize timers, I/O lines, and interrupts */
    IO_Init();	

    /* Turn on PWM channels */
    PWM_On();

	/* Scheduling - routine never returns, so put this last in the main function */
	Scheduler_Start();
}

EVENT_HANDLER(USB_Connect)
{
	/* Start USB management task */
	Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);

    /* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED4);
}

EVENT_HANDLER(USB_Disconnect)
{
	/* Stop running ProcessPacket and USB management tasks */
	Scheduler_SetTaskMode(USB_ProcessPacket, TASK_STOP);
	Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);

    /* Stop the timers and reset I/O lines to reduce current draw */
    IO_Disconnect();

	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
}

EVENT_HANDLER(USB_CreateEndpoints)
{
	/* Setup USB In and Out Endpoints */
	Endpoint_ConfigureEndpoint(CLKDIR_PWM_IN_EPNUM, EP_TYPE_BULK,
		                       ENDPOINT_DIR_IN, CLKDIR_PWM_IN_EPSIZE,
	                           ENDPOINT_BANK_DOUBLE);

	Endpoint_ConfigureEndpoint(CLKDIR_PWM_OUT_EPNUM, EP_TYPE_BULK,
		                       ENDPOINT_DIR_OUT, CLKDIR_PWM_OUT_EPSIZE,
	                           ENDPOINT_BANK_DOUBLE);

	/* Indicate USB connected and ready */
	LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
	
	/* Start ProcessPacket task */
	Scheduler_SetTaskMode(USB_ProcessPacket, TASK_RUN);
}

TASK(USB_ProcessPacket)
{
    /* Check if the USB System is connected to a Host */    if (USB_IsConnected)
    {
        /* Select the Data Out Endpoint */
        Endpoint_SelectEndpoint(CLKDIR_PWM_OUT_EPNUM);

        /* Check to see if a command from the host has been issued */
        if (Endpoint_ReadWriteAllowed())
        {	
            /* Indicate busy */
            LEDs_TurnOnLEDs(LEDS_LED3 | LEDS_LED4);

            /* Read USB packet from the host */
            USBPacket_Read();

            /* Return the same CommandID that was received */           
            USBPacketIn.Header.CommandID = USBPacketOut.Header.CommandID;

            /* Process USB packet */
            switch (USBPacketOut.Header.CommandID)
            {
                case USB_CMD_AVR_RESET:     
                    {   
                        USBPacket_Write();
                        AVR_RESET();    
                    }
                    break;
                case USB_CMD_AVR_DFU_MODE:     
                    {   
                        USBPacket_Write();
                        boot_key = DFU_BOOT_KEY_VAL;
                        AVR_RESET();    
                    }
                    break;
                case USB_CMD_SERVO_READ:     
                    {      
                    }
                    break;
                case USB_CMD_TEST:     
                    {   
                        USBPacketIn.Header.ControlByte  = sizeof(USBPacketIn.PulseWidthArray[0]);
                        USBPacketIn.PulseWidthArray[0].Value = 200;
                        USBPacketIn.PulseWidthArray[1].Default = 69;    
                    }
                    break;
                default:    
                    {
                    }
            }
            
            /* Write the return USB packet */
            USBPacketIn.Header.ControlByte  = 77;
            USBPacketIn.Header.ActiveServos = ServoArray.Header.ActiveServos;
            USBPacketIn.Header.PWMOnOff     = ServoArray.Header.PWMOnOff;
            for( uint8_t i=0; i<=5; i++) 
            { 
                USBPacketIn.PulseWidthArray[i] = ServoArray.Servo[i].PulseWidth;
            }          
            USBPacket_Write();

            /* Indicate ready */
            LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
        }
    }
}

static void USBPacket_Read(void)
{
	uint8_t* USBPacketOutPtr = (uint8_t*)&USBPacketOut;

	/* Select the Data Out endpoint */
	Endpoint_SelectEndpoint(CLKDIR_PWM_OUT_EPNUM);

    /* Read in USB packet header */
    Endpoint_Read_Stream_LE(USBPacketOutPtr, sizeof(USBPacketOut));
  
	/* Clear the endpoint */
	Endpoint_FIFOCON_Clear();
}

static void USBPacket_Write(void)
{
	uint8_t* USBPacketInPtr = (uint8_t*)&USBPacketIn;

	/* Select the Data Out endpoint */
	Endpoint_SelectEndpoint(CLKDIR_PWM_OUT_EPNUM);

	/* While data pipe is stalled, process control requests */
	while (Endpoint_IsStalled())
	{
		USB_USBTask();
		Endpoint_SelectEndpoint(CLKDIR_PWM_OUT_EPNUM);
	}

	/* Select the Data In endpoint */
	Endpoint_SelectEndpoint(CLKDIR_PWM_IN_EPNUM);

	/* While data pipe is stalled, process control requests */
	while (Endpoint_IsStalled())
	{
		USB_USBTask();
		Endpoint_SelectEndpoint(CLKDIR_PWM_IN_EPNUM);
	}
	
	/* Wait until read/write to IN data endpoint allowed */
	while (!(Endpoint_ReadWriteAllowed()));

	/* Write the return data to the endpoint */
	Endpoint_Write_Stream_LE(USBPacketInPtr, sizeof(USBPacketIn));
	
	/* Send the CSW */
	Endpoint_FIFOCON_Clear();
}

static void Servo_Address_Init(void)
{
    ServoArray.Servo[0].Address.Interrupt = INT0;
    ServoArray.Servo[0].Address.Timer = &OCR1A;
    ServoArray.Servo[0].Address.DirReg = &PINA;
    ServoArray.Servo[0].Address.DirPin = PINA0;

    ServoArray.Servo[1].Address.Interrupt = INT1;
    ServoArray.Servo[1].Address.Timer = &OCR1B;
    ServoArray.Servo[1].Address.DirReg = &PINA;
    ServoArray.Servo[1].Address.DirPin = PINA1;

    ServoArray.Servo[2].Address.Interrupt = INT2;
    ServoArray.Servo[2].Address.Timer = &OCR1C;
    ServoArray.Servo[2].Address.DirReg = &PINA;
    ServoArray.Servo[2].Address.DirPin = PINA2;

    ServoArray.Servo[3].Address.Interrupt = INT3;
    ServoArray.Servo[3].Address.Timer = &OCR3A;
    ServoArray.Servo[3].Address.DirReg = &PINA;
    ServoArray.Servo[3].Address.DirPin = PINA3;

    ServoArray.Servo[4].Address.Interrupt = INT4;
    ServoArray.Servo[4].Address.Timer = &OCR3B;
    ServoArray.Servo[4].Address.DirReg = &PINA;
    ServoArray.Servo[4].Address.DirPin = PINA4;

    ServoArray.Servo[5].Address.Interrupt = INT5;
    ServoArray.Servo[5].Address.Timer = &OCR3C;
    ServoArray.Servo[5].Address.DirReg = &PINA;
    ServoArray.Servo[5].Address.DirPin = PINA5;
}

static void Servo_PulseWidth_Init(void)
{
    ServoArray.Servo[0].PulseWidth.Default = 1500;
    ServoArray.Servo[0].PulseWidth.Max = 2500;
    ServoArray.Servo[0].PulseWidth.Min = 500;
    ServoArray.Servo[0].PulseWidth.Inc = 1;

    ServoArray.Servo[1].PulseWidth.Default = 1500;
    ServoArray.Servo[1].PulseWidth.Max = 2500;
    ServoArray.Servo[1].PulseWidth.Min = 500;
    ServoArray.Servo[1].PulseWidth.Inc = 1;

    ServoArray.Servo[2].PulseWidth.Default = 1500;
    ServoArray.Servo[2].PulseWidth.Max = 2500;
    ServoArray.Servo[2].PulseWidth.Min = 500;
    ServoArray.Servo[2].PulseWidth.Inc = 1;

    ServoArray.Servo[3].PulseWidth.Default = 1500;
    ServoArray.Servo[3].PulseWidth.Max = 2500;
    ServoArray.Servo[3].PulseWidth.Min = 500;
    ServoArray.Servo[3].PulseWidth.Inc = 1;

    ServoArray.Servo[4].PulseWidth.Default = 1500;
    ServoArray.Servo[4].PulseWidth.Max = 2500;
    ServoArray.Servo[4].PulseWidth.Min = 500;
    ServoArray.Servo[4].PulseWidth.Inc = 1;

    ServoArray.Servo[5].PulseWidth.Default = 1500;
    ServoArray.Servo[5].PulseWidth.Max = 2500;
    ServoArray.Servo[5].PulseWidth.Min = 500;
    ServoArray.Servo[5].PulseWidth.Inc = 1;
}

static void IO_Init(void)
{
    /* Input lines initialization */
        /* Set data direction of pins 0:5 on PORTA to input for DIR lines */
        DDRA &= ~((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3) | (1<<DDA4) | (1<<DDA5));

        /* Switch off pull-up resistors on pins 0:5 on PORTA */
        PORTA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2) | (1<<PA3) | (1<<PA4) | (1<<PA5));   

        /* Set data direction of pins 0:3 on PORTD to input for CLK lines */
        DDRD &= ~((1<<DDD0) | (1<<DDD1) | (1<<DDD2) | (1<<DDD3));

        /* Switch off pull-up resistors on pins 0:3 on PORTD */
        PORTD &= ~((1<<PD0) | (1<<PD1) | (1<<PD2) | (1<<PD3)); 

        /* Set data direction of pins 4:5 on PORTE to input for CLK lines */
        DDRE &= ~((1<<DDE4) | (1<<DDE5));

        /* Switch off pull-up resistors on pins 4:5 on PORTE */
        PORTE &= ~((1<<PE4) | (1<<PE5)); 

    /* Output lines initialization */
        /* Set data direction of Timer 1 pwm pins to output (PORTB pins 5:7) */
        DDRB |= ((1<<DDB5) | (1<<DDB6) | (1<<DDB7));

        /* Set Timer 1 pwm pins (PORTB pins 5:7) low to start */
        PORTB &= ~((1<<PB5) | (1<<PB6) | (1<<PB7));

        /* Set data direction of Timer 3 pwm pins to output (PORTC pins 4:6) */
        DDRC |= ((1<<DDC4) | (1<<DDC5) | (1<<DDC6));

        /* Set Timer 3 pwm pins (PORTC pins 4:6) low to start */
        PORTC &= ~((1<<PC4) | (1<<PC5) | (1<<PC6));

    /* External interrupts initialization */
        /* Disable external interrupt pins 0:5 before changing interrupt sense control */ 
        EIMSK &= ~((1<<INT0) | (1<<INT1) | (1<<INT2) | (1<<INT3) | (1<<INT4) | (1<<INT5));  

        /* Set external interrupt pins 0:3 to rising edge */
        EICRA = 0xFF; 

        /* Set external interrupt pins 4:5 to rising edge */  
        EICRB |= ((1<<ISC40) | (1<<ISC41) | (1<<ISC50) | (1<<ISC51));

        /* Enable external interrupt pins 0:5 */
        EIMSK |= ((1<<INT0) | (1<<INT1) | (1<<INT2) | (1<<INT3) | (1<<INT4) | (1<<INT5)); 

        /* All servo external interrupts are disabled initially */
        ServoArray.Header.ActiveServos = 0;

    /* Timers initialization */
        /* Clear OC1A/OC1B/OC1C on compare match */
        /* Set Fast PWM Mode on Timer 1, Top @ IRC1, Update of OCR1x @ TOP, TOV1 Flag Set on TOP */
        TCCR1A = ((1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1) | (1<<WGM11));
        TCCR1B = ((1<<WGM13) | (1<<WGM12));

        /* Clear OC3A/OC3B/OC3C on compare match */
        /* Set Fast PWM Mode on Timer 3, Top @ IRC3, Update of OCR3x @ TOP, TOV3 Flag Set on TOP */
        TCCR3A = ((1<<COM3A1) | (1<<COM3B1) | (1<<COM3C1) | (1<<WGM31));
        TCCR3B = ((1<<WGM33) | (1<<WGM32));

        /* Both 16-bit timers are stopped, PWM is turned off */
        ServoArray.Header.PWMOnOff = 0;
    
        /* Set PWM frequency */
        REG_16bit_Write(&ICR1,(TIMER_RES*1000000)/F_PWM);
        REG_16bit_Write(&ICR3,(TIMER_RES*1000000)/F_PWM);

        /* Initialize pulse width values to the default */
        for( uint8_t i=0; i<=5; i++) 
        { 
            REG_16bit_Write(ServoArray.Servo[i].Address.Timer,TIMER_RES*ServoArray.Servo[i].PulseWidth.Default);
            ServoArray.Servo[i].PulseWidth.Value = ServoArray.Servo[i].PulseWidth.Default; 
        }       
}

static void IO_Disconnect(void)
{
    /* Turn off PWM channels */
    PWM_Off();
    /* Set data direction of Timer 1 pwm pins to input (PORTB pins 5:7) to reduce current draw */
    DDRB &= ~((1<<DDB5) | (1<<DDB6) | (1<<DDB7));

    /* Set data direction of Timer 3 pwm pins to input (PORTC pins 4:6) to reduce current draw */
    DDRC &= ~((1<<DDC4) | (1<<DDC5) | (1<<DDC6));
}

static void PWM_On(void)
{
    /* Set prescaler value for both 16-bit timers (starts timers) */
    switch (PRESCALER)
    {
        case 1:     
            {
                TCCR1B |= (1<<CS10);
                TCCR3B |= (1<<CS30);
            }
            break;
        case 8:     
            {
                TCCR1B |= (1<<CS11);
                TCCR3B |= (1<<CS31);
            }
            break;
        case 64:    
            {
                TCCR1B |= ((1<<CS11)|(1<<CS10));
                TCCR3B |= ((1<<CS31)|(1<<CS30));
            }
            break;
        case 256:   
            {
                TCCR1B |= (1<<CS12);
                TCCR3B |= (1<<CS32);
            }
            break;
        case 1024:  
            {
                TCCR1B |= ((1<<CS12)|(1<<CS10));
                TCCR3B |= ((1<<CS32)|(1<<CS30));
            }
            break;
        default:    
            {
                TCCR1B |= (1<<CS11);
                TCCR3B |= (1<<CS31);
            }
    }

    /* Both 16-bit timers are started, PWM is turned on */
    ServoArray.Header.PWMOnOff = 1;	
}

static void PWM_Off(void)
{
    /* Stop both 16-bit timers */
	TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
    TCCR3B &= ~((1<<CS32)|(1<<CS31)|(1<<CS30));

    /* Both 16-bit timers are stopped, PWM is turned off */
    ServoArray.Header.PWMOnOff = 0;	
}

static void REG_16bit_Write(volatile uint16_t *reg, volatile uint16_t val)
{
    /* See "Accessing 16-bit Registers" of the AT90USB1287 datasheet */
    uint8_t sreg;
    /* Save global interrupt flag */
    sreg = SREG;
    /* Disable interrupts */
    cli();  
    *reg = val;
    /* Restore global interrupt flag */
    SREG = sreg;
}

static void PWM_Update(volatile uint8_t ServoAddress)
{
    uint8_t Dir;
    uint16_t PW;
  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 

        /* Get direction */
        Dir = *ServoArray.Servo[ServoAddress].Address.DirReg & (1 << ServoArray.Servo[ServoAddress].Address.DirPin);
    
        if (Dir == REVERSE) {
            ServoArray.Servo[ServoAddress].PulseWidth.Value -= ServoArray.Servo[ServoAddress].PulseWidth.Inc;
        }
        else {
            ServoArray.Servo[ServoAddress].PulseWidth.Value += ServoArray.Servo[ServoAddress].PulseWidth.Inc;
        }
    
        /* Clamp output between min and max values */
        if (ServoArray.Servo[ServoAddress].PulseWidth.Value < ServoArray.Servo[ServoAddress].PulseWidth.Min) {
            PW = ServoArray.Servo[ServoAddress].PulseWidth.Min;
        }
        else if (ServoArray.Servo[ServoAddress].PulseWidth.Value > ServoArray.Servo[ServoAddress].PulseWidth.Max) {
            PW = ServoArray.Servo[ServoAddress].PulseWidth.Max;
        }
        else {
            PW = ServoArray.Servo[ServoAddress].PulseWidth.Value;
        }
    }

    /* Set new pulse width value */
    REG_16bit_Write(ServoArray.Servo[ServoAddress].Address.Timer,TIMER_RES*PW);
}

ISR(INT0_vect) {
    PWM_Update(0);
    return;
}

ISR(INT1_vect) {
    PWM_Update(1);
    return;
}

ISR(INT2_vect) {
    PWM_Update(2);
    return;
}

ISR(INT3_vect) {
    PWM_Update(3);
    return;
}

ISR(INT4_vect) {
    PWM_Update(4);
    return;
}

ISR(INT5_vect) {
    PWM_Update(5);
    return;
}
