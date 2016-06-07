/*
 * board.c - tiva-c-connected launchpad configuration
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "simplelink.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "sensorlib/hw_tmp006.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/tmp006.h"
#include "sensorlib/bmp180.h"
#include "board.h"

#define TMP006_I2C_ADDRESS      0x41
#define BMP180_I2C_ADDRESS      0x77

P_EVENT_HANDLER        pIrqEventHandler = 0;

_u8 IntIsMasked;
_u32 g_SysClock=120000000;
//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the TMP006 sensor driver.
//
//*****************************************************************************
tTMP006 g_sTMP006Inst;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor driver.
//
//*****************************************************************************
tBMP180 g_sBMP180Inst;

int sensorTurn=0;


void initClk()
{
    /*The FPU should be enabled because some compilers will use floating-
    * point registers, even for non-floating-point code.  If the FPU is not
    * enabled this will cause a fault.  This also ensures that floating-
    * point operations could be added to this application and would work
    * correctly and use the hardware floating-point unit.  Finally, lazy
    * stacking is enabled for interrupt handlers.  This allows floating-
    * point instructions to be used within interrupt handlers, but at the
    * expense of extra stack usage. */
    FPUEnable();
    FPULazyStackingEnable();

    g_SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
						 SYSCTL_OSC_MAIN |
						 SYSCTL_USE_PLL |
						 SYSCTL_CFG_VCO_480), 120000000);
		
		
		
}

void initI2C(void)
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
		
		GPIOPinConfigure(GPIO_PD0_I2C7SCL);
		
    GPIOPinConfigure(GPIO_PD1_I2C7SDA);
	
		
	
		GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
		
		
	
		GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOH);
	
		
		I2CMInit(&g_sI2CInst, I2C7_BASE, INT_I2C7, 0xff, 0xff, g_SysClock);
	
		//
    // Initialize the TMP006
    //
    TMP006Init(&g_sTMP006Inst, &g_sI2CInst, TMP006_I2C_ADDRESS,
               TMP006AppCallback, &g_sTMP006Inst);
		
		SysCtlDelay(g_SysClock / (100 * 3));
		
		//
    // Initialize the BMP180.
    //
    BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
               BMP180AppCallback, &g_sBMP180Inst);
					 
		SysCtlDelay(g_SysClock / (100 * 3));
		
}

void initSystick(void)
{
		sensorTurn=0;
		SysTickPeriodSet(g_SysClock);
    SysTickIntEnable();
    SysTickEnable();
}

void SysTickIntHandler(void)
{
	
	if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0))
	{
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, PIN_LOW);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, PIN_HIGH);
	}
	switch(sensorTurn)
	{
		case 0:
		{
				TMP006DataRead(&g_sTMP006Inst, TMP006AppCallback, &g_sTMP006Inst);
				SysTickDisable();
				break;
		}
		case 1:
		{
				BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
				SysTickDisable();
				break;
		}	
	}
		
			
}

//void
//IntHandlerGPIOPortH(void)
//{
//    uint32_t ui32Status;

//    ui32Status = GPIOIntStatus(GPIO_PORTH_BASE, 1);

//    //
//    // Clear all the pin interrupts that are set
//    //
//    GPIOIntClear(GPIO_PORTH_BASE, ui32Status);

//    if(ui32Status & GPIO_PIN_2)
//    {
//        //
//        // This interrupt indicates a conversion is complete and ready to be
//        // fetched.  So we start the process of getting the data.
//        //
//				
//        TMP006DataRead(&g_sTMP006Inst, TMP006AppCallback, &g_sTMP006Inst);
//    }
//}

void BMP180AppCallback(void* pvCallbackData, unsigned     int ui8Status)
{
	
		float fTemperature, fPressure, fAltitude;
    int_fast32_t i32IntegerPart;
    int_fast32_t i32FractionPart;
		unsigned char tempString[30]={0};
		
		if(ui8Status == I2CM_STATUS_SUCCESS&&sensorTurn==1)
    {
				//
        // Get a local copy of the latest temperature and pressure data in
        // float format.
        //
        BMP180DataTemperatureGetFloat(&g_sBMP180Inst, &fTemperature);
        BMP180DataPressureGetFloat(&g_sBMP180Inst, &fPressure);

        //
        // Convert the temperature to an integer part and fraction part for
        // easy print.
        //
        i32IntegerPart = (int32_t) fTemperature;
        i32FractionPart =(int32_t) (fTemperature * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }

        //
        // Print temperature with three digits of decimal precision.
        //
        sprintf(tempString,"Temperature %3d.%03d\t", i32IntegerPart,
                   i32FractionPart);
				CLI_Write(tempString);
				

        //
        // Convert the pressure to an integer part and fraction part for
        // easy print.
        //
        i32IntegerPart = (int32_t) fPressure;
        i32FractionPart =(int32_t) (fPressure * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }
				
				 //
        // Print Pressure with three digits of decimal precision.
        //
        sprintf(tempString,"Pressure %3d.%03d\t", i32IntegerPart, i32FractionPart);
				CLI_Write(tempString);

        //
        // Calculate the altitude.
        //
        fAltitude = 44330.0f * (1.0f - powf(fPressure / 101325.0f,
                                            1.0f / 5.255f));

        //
        // Convert the altitude to an integer part and fraction part for easy
        // print.
        //
        i32IntegerPart = (int32_t) fAltitude;
        i32FractionPart =(int32_t) (fAltitude * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }

        //
        // Print altitude with three digits of decimal precision.
        //
        sprintf(tempString,"Altitude %3d.%03d", i32IntegerPart, i32FractionPart);
				CLI_Write(tempString);

        //
        // Print new line.
        //
        CLI_Write("\n\r");
				sensorTurn=(sensorTurn+1)%2;
				sprintf(tempString,"Systick Value: %ld\n\r",SysTickValueGet());
				CLI_Write(tempString);
				SysTickEnable();
			}
}

void
I2CIntHandler(void)
{
		
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
		
}

void
TMP006AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
		float fAmbient, fObject;
    int_fast32_t i32IntegerPart;
    int_fast32_t i32FractionPart;
		unsigned char tempString[20]={0};
		
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS&&sensorTurn==0)
    {
				//
				// Get a local copy of the latest data in float format.
				//
				TMP006DataTemperatureGetFloat(&g_sTMP006Inst, &fAmbient, &fObject);

				//
				// Convert the floating point ambient temperature  to an integer part
				// and fraction part for easy printing.
				//
				i32IntegerPart = (int32_t)fAmbient;
				i32FractionPart = (int32_t)(fAmbient * 1000.0f);
				i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
				if(i32FractionPart < 0)
				{
						i32FractionPart *= -1;
				}
				sprintf(tempString,"Ambient %3d.%03d\t", i32IntegerPart, i32FractionPart);
				CLI_Write(tempString);
				//
				// Convert the floating point ambient temperature  to an integer part
				// and fraction part for easy printing.
				//
				i32IntegerPart = (int32_t)fObject;
				i32FractionPart = (int32_t)(fObject * 1000.0f);
				i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
				if(i32FractionPart < 0)
				{
						i32FractionPart *= -1;
				}
				sprintf(tempString,"Object %3d.%03d\n\r", i32IntegerPart, i32FractionPart);
				CLI_Write(tempString);
				sensorTurn=(sensorTurn+1)%2;
		
				SysTickEnable();
		}
}
	

void initTimer()
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		IntMasterEnable();
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		TimerLoadSet(TIMER0_BASE, TIMER_A, g_SysClock/5);
		IntPriorityGroupingSet(4);
    IntPrioritySet(INT_TIMER0A, 0xE0);
		IntEnable(INT_TIMER0A);
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		TimerEnable(TIMER0_BASE, TIMER_A);
}
void
Timer0IntHandler(void)
{
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4))
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4, PIN_LOW);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4, PIN_HIGH);
		}
}
void DisableTimer0(void)
{
		TimerDisable(TIMER0_BASE, TIMER_A);
}

void EnableTimer0(void)
{
		TimerEnable(TIMER0_BASE, TIMER_A);
}

void stopWDT()
{
}

int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue)
{
    pIrqEventHandler = InterruptHdl;

    return 0;
}

void CC3100_disable()
{
    ROM_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_4, PIN_LOW);
}

void CC3100_enable()
{
    ROM_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_4, PIN_HIGH);
}

void CC3100_InterruptEnable()
{
    GPIOIntEnable(GPIO_PORTM_BASE,GPIO_PIN_7);
}

void CC3100_InterruptDisable()
{
     GPIOIntDisable(GPIO_PORTM_BASE,GPIO_PIN_7);
}

void MaskIntHdlr()
{
    IntIsMasked = TRUE;
}

void UnMaskIntHdlr()
{
    IntIsMasked = FALSE;
}

void UARTRxTxHandler()
{
}

void GPIOM_intHandler()
{
    unsigned long intStatus;
    intStatus = GPIOIntStatus(GPIO_PORTM_BASE, 0);
    GPIOIntClear(GPIO_PORTM_BASE,intStatus);

    if(intStatus & GPIO_PIN_7)
    {
    	if(pIrqEventHandler)
        {
            pIrqEventHandler(0);
        }
    }
}

void initLEDs()
{
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_LOW);
	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, PIN_LOW);
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, PIN_LOW);
	  GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4, PIN_LOW);
}

void turnLedOn(char ledNum)
{
    switch(ledNum)
    {
      case LED1:
    	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_HIGH);
        break;
      case LED2:
    	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, PIN_HIGH);
        break;
    }
}

void turnLedOff(char ledNum)
{
    switch(ledNum)
    {
      case LED1:
    	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_LOW);
        break;
      case LED2:
    	  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, PIN_LOW);
        break;
    }
}


void toggleLed(char ledNum)
{
    switch(ledNum)
    {
      case LED1:
    	  if(GPIOPinRead(GPIO_PORTN_BASE,GPIO_PIN_0))
    	  {
    		  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_LOW);
    	  }
    	  else
    	  {
    		  GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_HIGH);
    	  }
        break;
      case LED2:
    	  if(GPIOPinRead(GPIO_PORTN_BASE,GPIO_PIN_1))
    	  {
    	     GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, PIN_LOW);
    	  }
    	  else
    	  {
    	     GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0, PIN_HIGH);
    	  }
        break;
    }

}

unsigned char GetLEDStatus()
{
  unsigned char status = 0;

  if(GPIOPinRead(GPIO_PORTN_BASE,GPIO_PIN_0))
    status |= (1 << 0);
  if(GPIOPinRead(GPIO_PORTN_BASE,GPIO_PIN_1))
    status |= (1 << 1);

  return status;
}


