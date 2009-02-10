#ifndef _CLKDIRtoPWM_H_
#define _CLKDIRtoPWM_H_

	/* Includes: */
		#include <avr/io.h>
        #include <avr/interrupt.h>
		#include <avr/wdt.h>

		#include "Descriptors.h"
		
		#include <MyUSB/Version.h>                    // Library Version Information
		#include <MyUSB/Common/ButtLoadTag.h>         // PROGMEM tags readable by the ButtLoad project
		#include <MyUSB/Drivers/USB/USB.h>            // USB Functionality
		#include <MyUSB/Drivers/Board/LEDs.h>         // LEDs driver
		#include <MyUSB/Scheduler/Scheduler.h>        // Simple scheduler for task management
	
	/* Macros: */
        /* PWM frequency in Hz */
        #define F_PWM   50
        
        /* Timer prescaler (1,8,64,256,1024) 
           (F_CPU/PRESCALER)/F_PWM must be <= (2^16-1) (65535 the max count for a 16 bit timer) 
                and F_CPU/PRESCALER must be >= 1000000 (so TIMER_RES will be an integer) 
           Example: F_CPU=8000000, PRESCALER=8, F_PWM=50 
                    8000000/(8*50) = 20000 <= 65535 
                    8000000/8 = 1000000 >= 1000000 */
        #define PRESCALER   8
            
        /* Timer resolution (counts per microsecond) */
        #define TIMER_RES   ((F_CPU/PRESCALER)/1000000) 

        /* Servo direction */        
        #define REVERSE 0

        /* USB Command IDs */
        #define USB_CMD_SERVO_READ      1
        #define USB_CMD_SERVO_WRITE     2
        #define USB_CMD_SERVO_ACTIVATE  3
        #define USB_CMD_PWM_TOGGLE      4
        #define USB_CMD_AVR_RESET       200
        #define USB_CMD_AVR_DFU_MODE    201
        #define USB_CMD_TEST            251

        /* Software reset */
        #define AVR_RESET() wdt_enable(WDTO_30MS); while(1) {}
        #define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? 1:0)
        #define DFU_BOOT_KEY_VAL 0xAA55AA55

	/* Type Defines: */
        typedef struct
		{
            uint16_t    Value;
            uint16_t    Default; 
            uint16_t    Max;
            uint16_t    Min;
            uint16_t    Inc;      	
	
		} PulseWidthWrapper_t;  

        typedef struct
		{
            struct
            {
                uint8_t             Interrupt;
                volatile uint16_t   *Timer;
                volatile uint8_t    *DirReg;
                uint8_t             DirPin;
            } Address;	

            PulseWidthWrapper_t PulseWidth;

		} ServoWrapper_t;   

        typedef struct
		{
            struct
            {
                uint8_t     ActiveServos;
                uint8_t     PWMOnOff;
            } Header;	
    
            ServoWrapper_t Servo[6];

		} ServoArrayWrapper_t; 

        typedef struct
		{
			struct
			{
                uint8_t     CommandID;
                uint8_t     ControlByte;		
			} Header;

            PulseWidthWrapper_t PulseWidthArray[6];
			
		} USBPacketOutWrapper_t;

		typedef struct
		{
            struct
			{
                uint8_t     CommandID;
                uint8_t     ControlByte;
                uint8_t     ActiveServos; 
                uint8_t     PWMOnOff;     		
			} Header;

            PulseWidthWrapper_t PulseWidthArray[6];
	
		} USBPacketInWrapper_t;
		
	/* Enums: */

    /* Global Variables: */
        volatile ServoArrayWrapper_t    ServoArray;
        USBPacketOutWrapper_t           USBPacketOut;
        USBPacketInWrapper_t            USBPacketIn;
		
	/* Task Definitions: */
		TASK(USB_ProcessPacket);

	/* Event Handlers: */
		HANDLES_EVENT(USB_Connect);
		HANDLES_EVENT(USB_Disconnect);
        HANDLES_EVENT(USB_CreateEndpoints);

	/* Function Prototypes: */
		#if defined(INCLUDE_FROM_CLKDIRTOPWM_C)
            static void USBPacket_Read(void);
            static void USBPacket_Write(void);
            static void Servo_Address_Init(void);
            static void Servo_PulseWidth_Init(void);
			static void IO_Init(void);
			static void IO_Disconnect(void);
            static void PWM_On(void);
            static void PWM_Off(void);
            static void PWM_Update(volatile uint8_t ServoNum);
            static void REG_16bit_Write(volatile uint16_t *reg, volatile uint16_t val);
		#endif

#endif
