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
#include "sensorlib/hw_isl29023.h"
#include "sensorlib/hw_sht21.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/tmp006.h"
#include "sensorlib/bmp180.h"
#include "sensorlib/isl29023.h"
#include "sensorlib/sht21.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "board.h"

#define SHT21_I2C_ADDRESS  0x40
#define TMP006_I2C_ADDRESS      0x41
#define ISL29023_I2C_ADDRESS    0x44
#define MPU9150_I2C_ADDRESS     0x68
#define BMP180_I2C_ADDRESS      0x77

#define NumberOfSensor 5

P_EVENT_HANDLER        pIrqEventHandler = 0;

_u8 IntIsMasked;
_u32 g_SysClock=120000000;
_u8 timer0_status=0;


int_fast32_t i32IPart[16], i32FPart[16];

int_fast32_t SHT21_i32IntegerPart1;
int_fast32_t SHT21_i32FractionPart1;
int_fast32_t SHT21_i32IntegerPart2;
int_fast32_t SHT21_i32FractionPart2;

int_fast32_t ISL290_i32IntegerPart, ISL290_i32FractionPart;

int_fast32_t BMP180_i32IntegerPart1;
int_fast32_t BMP180_i32FractionPart1;
int_fast32_t BMP180_i32IntegerPart2;
int_fast32_t BMP180_i32FractionPart2;
int_fast32_t BMP180_i32IntegerPart3;
int_fast32_t BMP180_i32FractionPart3;

int_fast32_t TMP006_i32IntegerPart1;
int_fast32_t TMP006_i32FractionPart1;
int_fast32_t TMP006_i32IntegerPart2;
int_fast32_t TMP006_i32FractionPart2;

//*****************************************************************************
//
// Constants to hold the floating point version of the thresholds for each
// range setting. Numbers represent an 81% and 19 % threshold levels. This
// creates a +/- 1% hysteresis band between range adjustments.
//
//*****************************************************************************
const float g_fThresholdHigh[4] =
{
    810.0f, 3240.0f, 12960.0f, 64000.0f
};
const float g_fThresholdLow[4] =
{
    0.0f, 760.0f, 3040.0f, 12160.0f
};

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

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tISL29023 g_sISL29023Inst;

//*****************************************************************************
//
// Global instance structure for the SHT21 sensor driver.
//
//*****************************************************************************
tSHT21 g_sSHT21Inst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

volatile unsigned long g_vui8IntensityFlag;

volatile uint_fast32_t ui32CompDCMStarted;

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
		uint8_t ui8Mask;
	
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
		
		GPIOPinConfigure(GPIO_PD0_I2C7SCL);
		
    GPIOPinConfigure(GPIO_PD1_I2C7SDA);
	
		
	
		GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
		//
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // ISL29023
    //
	
		
		GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOE);
		
		I2CMInit(&g_sI2CInst, I2C7_BASE, INT_I2C7, 0xff, 0xff, g_SysClock);
		
		//
    // Initialize the SHT21.
    //
    SHT21Init(&g_sSHT21Inst, &g_sI2CInst, SHT21_I2C_ADDRESS,
            SHT21AppCallback, &g_sSHT21Inst);
						
		SysCtlDelay(g_SysClock / (100 * 3));
	
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
		
		IntPrioritySet(INT_I2C7, 0x00);
		IntPrioritySet(INT_GPIOM, 0x00);
		IntPrioritySet(INT_TIMER0A, 0x80);
		IntPrioritySet(INT_TIMER1A, 0x40);
    IntPrioritySet(INT_GPIOE, 0x80);
    IntPrioritySet(INT_UART0, 0x80);
		
		//
    // Initialize the ISL29023 Driver.
    //
    ISL29023Init(&g_sISL29023Inst, &g_sI2CInst, ISL29023_I2C_ADDRESS,
                 ISL29023AppCallback, &g_sISL29023Inst);
								 
		SysCtlDelay(g_SysClock / (100 * 3));
		
		//
    // Configure the ISL29023 to measure ambient light continuously. Set a 8
    // sample persistence before the INT pin is asserted. Clears the INT flag.
    // Persistence setting of 8 is sufficient to ignore camera flashes.
    //
    ui8Mask = (ISL29023_CMD_I_OP_MODE_M | ISL29023_CMD_I_INT_PERSIST_M |
               ISL29023_CMD_I_INT_FLAG_M);
    ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_I, ~ui8Mask,
                            (ISL29023_CMD_I_OP_MODE_ALS_CONT |
                             ISL29023_CMD_I_INT_PERSIST_8),
                            ISL29023AppCallback, &g_sISL29023Inst);
														
		//
    // Configure the upper threshold to 80% of maximum value
    //
    g_sISL29023Inst.pui8Data[1] = 0xCC;
    g_sISL29023Inst.pui8Data[2] = 0xCC;
    ISL29023Write(&g_sISL29023Inst, ISL29023_O_INT_HT_LSB,
                  g_sISL29023Inst.pui8Data, 2, ISL29023AppCallback,
                  &g_sISL29023Inst);

    //
    // Wait for transaction to complete
    //
    SysCtlDelay(g_SysClock / (100 * 3));

    //
    // Configure the lower threshold to 20% of maximum value
    //
    g_sISL29023Inst.pui8Data[1] = 0x33;
    g_sISL29023Inst.pui8Data[2] = 0x33;
    ISL29023Write(&g_sISL29023Inst, ISL29023_O_INT_LT_LSB,
                  g_sISL29023Inst.pui8Data, 2, ISL29023AppCallback,
                  &g_sISL29023Inst);
    //
    // Wait for transaction to complete
    //
    SysCtlDelay(g_SysClock / (100 * 3));
		
		//
		// Write the command to start a humidity measurement.
		//
		SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_RH, g_sSHT21Inst.pui8Data, 0,
						SHT21AppCallback, &g_sSHT21Inst);
						
		//
    // Wait for transaction to complete
    //
    SysCtlDelay(g_SysClock / (100 * 3));
		
		//
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);
								
		SysCtlDelay(g_SysClock / (100 * 3));
		
		//
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);
								 
		SysCtlDelay(g_SysClock / (100 * 3));
								 
		//
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);
		
		SysCtlDelay(g_SysClock / (100 * 3));
		
		ui32CompDCMStarted = 0;

		//
    // Print the basic outline of our data table. Done once and then kept as we
    // print only the data.
    //
		
//    CLI_Write("\033[2J\033[H");
//    CLI_Write("MPU9150 9-Axis Simple Data Application Example\n\r\n\r");
//    CLI_Write("\033[20GX\033[31G|\033[43GY\033[54G|\033[66GZ\n\r\n\r");
//    CLI_Write("Accel\033[8G|\033[31G|\033[54G|\n\r\n\r");
//    CLI_Write("Gyro\033[8G|\033[31G|\033[54G|\n\r\n\r");
//    CLI_Write("Mag\033[8G|\033[31G|\033[54G|\n\r\n\r");
//    CLI_Write("\n\033[20GRoll\033[31G|\033[43GPitch\033[54G|\033[66GYaw\n\r\n\r");
//    CLI_Write("Eulers\033[8G|\033[31G|\033[54G|\n\r\n\r");

//    CLI_Write("\n\033[17GQ1\033[26G|\033[35GQ2\033[44G|\033[53GQ3\033[62G|"
//               "\033[71GQ4\n\r\n\r");
//    CLI_Write("Q\033[8G|\033[26G|\033[44G|\033[62G|\n\r\n\r");
		
		TimerEnable(TIMER1_BASE, TIMER_A);
}

void initTimer1(void)
{
		sensorTurn=0;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		IntMasterEnable();
		TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		TimerLoadSet(TIMER1_BASE, TIMER_A, g_SysClock);
		IntPriorityGroupingSet(4);
    IntPrioritySet(INT_TIMER0A, 0xF0);
		IntEnable(INT_TIMER1A);
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		
}

void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
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
				TimerDisable(TIMER1_BASE, TIMER_A);
				
				break;
		}
		case 1:
		{
				BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
				TimerDisable(TIMER1_BASE, TIMER_A);
				
				break;
		}
		case 2:
		{
				ISL29023DataRead(&g_sISL29023Inst, ISL29023AppCallback, &g_sISL29023Inst);
				TimerDisable(TIMER1_BASE, TIMER_A);
				
				break;
		}
		case 3:
		{						
				SHT21DataRead(&g_sSHT21Inst, SHT21AppCallback, &g_sSHT21Inst);
				TimerDisable(TIMER1_BASE, TIMER_A);
				break;
		}
		case 4:
		{
				MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
				TimerDisable(TIMER1_BASE, TIMER_A);
				
				break;
		}
	}
		
			
}

void
GPIOPortEIntHandler(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTE_BASE, 1);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTE_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_5)
    {
        //
        // ISL29023 has indicated that the light level has crossed outside of
        // the intensity threshold levels set in INT_LT and INT_HT registers.
        //
        g_vui8IntensityFlag = 1;
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

void MPU9150AppCallback(void *pvCallbackData, unsigned     int ui8Status)
{
		
    uint_fast32_t ui32Idx, ui32CompDCMStarted;
    float pfData[16];
    float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
		//unsigned char tempString[30]={0};
		
		if(ui8Status == I2CM_STATUS_SUCCESS&&sensorTurn==4)
		{
	
				//
				// Initialize convenience pointers that clean up and clarify the code
				// meaning. We want all the data in a single contiguous array so that
				// we can make our pretty printing easier later.
				//
				pfAccel = pfData;
				pfGyro = pfData + 3;
				pfMag = pfData + 6;
				pfEulers = pfData + 9;
				pfQuaternion = pfData + 12;
			
				//
				// Get floating point version of the Accel Data in m/s^2.
				//
				MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
																 pfAccel + 2);

				//
				// Get floating point version of angular velocities in rad/sec
				//
				MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
																pfGyro + 2);

				//
				// Get floating point version of magnetic fields strength in tesla
				//
				MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
																	 pfMag + 2);
																	 
				//
				// Check if this is our first data ever.
				//
				if(ui32CompDCMStarted == 0)
				{
						//
						// Set flag indicating that DCM is started.
						// Perform the seeding of the DCM with the first data set.
						//
						ui32CompDCMStarted = 1;
						CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
																 pfMag[2]);
						CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
															 pfAccel[2]);
						CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
															pfGyro[2]);
						CompDCMStart(&g_sCompDCMInst);
				}
				else
				{
						//
						// DCM Is already started.  Perform the incremental update.
						//
						CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
																 pfMag[2]);
						CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
															 pfAccel[2]);
						CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
															-pfGyro[2]);
						CompDCMUpdate(&g_sCompDCMInst);
				}
				
				//
				// Get Euler data. (Roll Pitch Yaw)
				//
				CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
														 pfEulers + 2);

				//
				// Get Quaternions.
				//
				CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

				//
				// convert mag data to micro-tesla for better human interpretation.
				//
				pfMag[0] *= 1e6;
				pfMag[1] *= 1e6;
				pfMag[2] *= 1e6;

				//
				// Convert Eulers to degrees. 180/PI = 57.29...
				// Convert Yaw to 0 to 360 to approximate compass headings.
				//
				pfEulers[0] *= 57.295779513082320876798154814105f;
				pfEulers[1] *= 57.295779513082320876798154814105f;
				pfEulers[2] *= 57.295779513082320876798154814105f;
				if(pfEulers[2] < 0)
				{
						pfEulers[2] += 360.0f;
				}

				//
				// Now drop back to using the data as a single array for the
				// purpose of decomposing the float into a integer part and a
				// fraction (decimal) part.
				//
				for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
				{
						//
						// Conver float value to a integer truncating the decimal part.
						//
						i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

						//
						// Multiply by 1000 to preserve first three decimal values.
						// Truncates at the 3rd decimal place.
						//
						i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

						//
						// Subtract off the integer part from this newly formed decimal
						// part.
						//
						i32FPart[ui32Idx] = i32FPart[ui32Idx] -
																(i32IPart[ui32Idx] * 1000);

						//
						// make the decimal part a positive number for display.
						//
						if(i32FPart[ui32Idx] < 0)
						{
								i32FPart[ui32Idx] *= -1;
						}
				}

//				//
//				// Print the acceleration numbers in the table.
//				//
//				sprintf(tempString,"\033[5;17H%3d.%03d", i32IPart[0], i32FPart[0]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[5;40H%3d.%03d", i32IPart[1], i32FPart[1]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[5;63H%3d.%03d", i32IPart[2], i32FPart[2]);
//				CLI_Write(tempString);

//				//
//				// Print the angular velocities in the table.
//				//
//				sprintf(tempString,"\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);
//				CLI_Write(tempString);

//				//
//				// Print the magnetic data in the table.
//				//
//				sprintf(tempString,"\033[9;17H%3d.%03d", i32IPart[6], i32FPart[6]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[9;40H%3d.%03d", i32IPart[7], i32FPart[7]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[9;63H%3d.%03d", i32IPart[8], i32FPart[8]);
//				CLI_Write(tempString);

//				//
//				// Print the Eulers in a table.
//				//
//				sprintf(tempString,"\033[14;17H%3d.%03d", i32IPart[9], i32FPart[9]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[14;40H%3d.%03d", i32IPart[10], i32FPart[10]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[14;63H%3d.%03d", i32IPart[11], i32FPart[11]);
//				CLI_Write(tempString);

//				//
//				// Print the quaternions in a table format.
//				//
//				sprintf(tempString,"\033[19;14H%3d.%03d", i32IPart[12], i32FPart[12]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[19;32H%3d.%03d", i32IPart[13], i32FPart[13]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[19;50H%3d.%03d", i32IPart[14], i32FPart[14]);
//				CLI_Write(tempString);
//				sprintf(tempString,"\033[19;68H%3d.%03d", i32IPart[15], i32FPart[15]);
//				CLI_Write(tempString);
//				CLI_Write("\n\r");
				sensorTurn=(sensorTurn+1)%NumberOfSensor;
				TimerEnable(TIMER1_BASE, TIMER_A);
	 }

        
}
unsigned char flag=0;
void SHT21AppCallback(void *pvCallbackData, unsigned     int ui8Status)
{
		
		float fTemperature, fHumidity;
    
		unsigned char tempString[30]={0};
		
		if(ui8Status == I2CM_STATUS_SUCCESS&&sensorTurn==3)
		{
			
				if(flag==0)
				{
					
						
						//
						// Get the most recent temperature result as a float in celcius.
						//
						SHT21DataTemperatureGetFloat(&g_sSHT21Inst, &fTemperature);
						flag=(flag+1)%3;
					
						//
						// Write the command to start a humidity measurement.
						//
						SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_RH, g_sSHT21Inst.pui8Data, 0,
										SHT21AppCallback, &g_sSHT21Inst);
						
						SysCtlDelay(g_SysClock / (100 * 3));
						
						//
						// Perform the conversion from float to a printable set of integers.
						//
						SHT21_i32IntegerPart2 = (int32_t) fTemperature;
						SHT21_i32FractionPart2 = (int32_t) (fTemperature * 1000.0f);
						SHT21_i32FractionPart2 = SHT21_i32FractionPart2 - (SHT21_i32IntegerPart2 * 1000);
						if(SHT21_i32FractionPart2 < 0)
						{
								SHT21_i32FractionPart2 *= -1;
						}
						
//						//
//						// Print the temperature as integer and fraction parts.
//						//
//						sprintf(tempString,"Temperature %3d.%03d\n\r", SHT21_i32IntegerPart2, SHT21_i32FractionPart2);
//						CLI_Write(tempString);
				}
				else
				{
						SHT21DataRead(&g_sSHT21Inst, SHT21AppCallback, &g_sSHT21Inst);
					
						SysCtlDelay(g_SysClock / (100 * 3));	
						//
						// Get a copy of the most recent raw data in floating point format.
						//
						SHT21DataHumidityGetFloat(&g_sSHT21Inst, &fHumidity);
						
					
					
						//
						// Convert the floats to an integer part and fraction part for easy
						// print. Humidity is returned as 0.0 to 1.0 so multiply by 100 to get
						// percent humidity.
						//
						fHumidity *= 100.0f;
						SHT21_i32IntegerPart1 = (int32_t) fHumidity;
						SHT21_i32FractionPart1 = (int32_t) (fHumidity * 1000.0f);
						SHT21_i32FractionPart1 = SHT21_i32FractionPart1 - (SHT21_i32IntegerPart1 * 1000);
						if(SHT21_i32FractionPart1 < 0)
						{
								SHT21_i32FractionPart1 *= -1;
						}

						
						
//						
//						//
//						// Print the humidity value using the integers we just created.
//						//
						//sprintf(tempString,"Humidity %3d.%03d\n\r", SHT21_i32IntegerPart1, SHT21_i32FractionPart1);
						//CLI_Write(tempString);
						
						sensorTurn=(sensorTurn+1)%NumberOfSensor;
						TimerEnable(TIMER1_BASE, TIMER_A);
						
						flag=(flag+1)%3;
						
						//
						// Write the command to start a temperature measurement.
						//
						SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_T, g_sSHT21Inst.pui8Data, 0,
										SHT21AppCallback, &g_sSHT21Inst);
			}
		
		}
}




void ISL29023AppCallback(void *pvCallbackData, unsigned     int ui8Status)
{
	
		float fAmbient;
    
		unsigned char tempString[30]={0};
		
		float tempfAmbient;
		uint8_t ui8NewRange;
		
		if(ui8Status == I2CM_STATUS_SUCCESS&&sensorTurn==2)
		{
				//
				// Get a local floating point copy of the latest light data
				//
				ISL29023DataLightVisibleGetFloat(&g_sISL29023Inst, &fAmbient);
			
				//
				// Perform the conversion from float to a printable set of integers
				//
				ISL290_i32IntegerPart = (int32_t)fAmbient;
				ISL290_i32FractionPart = (int32_t)(fAmbient * 1000.0f);
				ISL290_i32FractionPart = ISL290_i32FractionPart - (ISL290_i32IntegerPart * 1000);
				if(ISL290_i32FractionPart < 0)
				{
						ISL290_i32FractionPart *= -1;
				}

				//
				// Print the temperature as integer and fraction parts.
				//
				//sprintf(tempString,"Visible Lux: %3d.%03d\n\r", ISL290_i32IntegerPart,ISL290_i32FractionPart);
				//CLI_Write(tempString);

				if(g_vui8IntensityFlag)
				{
					IntPriorityMaskSet(0x40);
					//
					// Reset the intensity trigger flag.
					//
					g_vui8IntensityFlag = 0;

					//
					// Adjust the lux range.
					//
					
					ui8NewRange = g_sISL29023Inst.ui8Range;

					//
					// Get a local floating point copy of the latest light data
					//
					ISL29023DataLightVisibleGetFloat(&g_sISL29023Inst, &tempfAmbient);

					//
					// Check if we crossed the upper threshold.
					//
					if(tempfAmbient > g_fThresholdHigh[g_sISL29023Inst.ui8Range])
					{
							//
							// The current intensity is over our threshold so adjsut the range
							// accordingly
							//
							if(g_sISL29023Inst.ui8Range < ISL29023_CMD_II_RANGE_64K)
							{
									ui8NewRange = g_sISL29023Inst.ui8Range + 1;
							}
					}

					//
					// Check if we crossed the lower threshold
					//
					if(tempfAmbient < g_fThresholdLow[g_sISL29023Inst.ui8Range])
					{
							//
							// If possible go to the next lower range setting and reconfig the
							// thresholds.
							//
							if(g_sISL29023Inst.ui8Range > ISL29023_CMD_II_RANGE_1K)
							{
									ui8NewRange = g_sISL29023Inst.ui8Range - 1;
							}
					}
					
					//
					// If the desired range value changed then send the new range to the sensor
					//
					if(ui8NewRange != g_sISL29023Inst.ui8Range)
					{
							ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II,
																			~ISL29023_CMD_II_RANGE_M, ui8NewRange,
																			ISL29023AppCallback, &g_sISL29023Inst);
					}
					
					SysCtlDelay(g_SysClock / (100 * 3));
					//
					// Now we must manually clear the flag in the ISL29023
					// register.
					//
					ISL29023Read(&g_sISL29023Inst, ISL29023_O_CMD_I,
											 g_sISL29023Inst.pui8Data, 1, ISL29023AppCallback,
											 &g_sISL29023Inst);

					//
					// Wait for transaction to complete
					//
					SysCtlDelay(g_SysClock / (100 * 3));
					
					//
					// Disable priority masking so all interrupts are enabled.
					//
					
					IntPriorityMaskSet(0);
				}
				sensorTurn=(sensorTurn+1)%NumberOfSensor;
				TimerEnable(TIMER1_BASE, TIMER_A);
		}
		
}

void BMP180AppCallback(void* pvCallbackData, unsigned     int ui8Status)
{
	
		float fTemperature, fPressure, fAltitude;
    
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
        BMP180_i32IntegerPart1 = (int32_t) fTemperature;
        BMP180_i32FractionPart1 =(int32_t) (fTemperature * 1000.0f);
        BMP180_i32FractionPart1 = BMP180_i32FractionPart1 - (BMP180_i32IntegerPart1 * 1000);
        if(BMP180_i32FractionPart1 < 0)
        {
            BMP180_i32FractionPart1 *= -1;
        }

        //
        // Print temperature with three digits of decimal precision.
        //
        //sprintf(tempString,"Temperature %3d.%03d\t", BMP180_i32IntegerPart1,
         //          BMP180_i32FractionPart1);
				//CLI_Write(tempString);
				

        //
        // Convert the pressure to an integer part and fraction part for
        // easy print.
        //
        BMP180_i32IntegerPart2 = (int32_t) fPressure;
        BMP180_i32FractionPart2 =(int32_t) (fPressure * 1000.0f);
        BMP180_i32FractionPart2 = BMP180_i32FractionPart2 - (BMP180_i32IntegerPart2 * 1000);
        if(BMP180_i32FractionPart2 < 0)
        {
            BMP180_i32FractionPart2 *= -1;
        }
				
				 //
        // Print Pressure with three digits of decimal precision.
        //
        //sprintf(tempString,"Pressure %3d.%03d\t", BMP180_i32IntegerPart2, BMP180_i32FractionPart2);
				//CLI_Write(tempString);

        //
        // Calculate the altitude.
        //
        fAltitude = 44330.0f * (1.0f - powf(fPressure / 101325.0f,
                                            1.0f / 5.255f));

        //
        // Convert the altitude to an integer part and fraction part for easy
        // print.
        //
        BMP180_i32IntegerPart3 = (int32_t) fAltitude;
        BMP180_i32FractionPart3 =(int32_t) (fAltitude * 1000.0f);
        BMP180_i32FractionPart3 = BMP180_i32FractionPart3 - (BMP180_i32IntegerPart3 * 1000);
        if(BMP180_i32FractionPart3 < 0)
        {
            BMP180_i32FractionPart3 *= -1;
        }

        //
        // Print altitude with three digits of decimal precision.
        //
        //sprintf(tempString,"Altitude %3d.%03d", BMP180_i32IntegerPart3, BMP180_i32FractionPart3);
				//CLI_Write(tempString);

        //
        // Print new line.
        //
        //CLI_Write("\n\r");
				//sensorTurn=3;
				sensorTurn=(sensorTurn+1)%NumberOfSensor;
				TimerEnable(TIMER1_BASE, TIMER_A);
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
				TMP006_i32IntegerPart1 = (int32_t)fAmbient;
				TMP006_i32FractionPart1 = (int32_t)(fAmbient * 1000.0f);
				TMP006_i32FractionPart1 = TMP006_i32FractionPart1 - (TMP006_i32IntegerPart1 * 1000);
				if(TMP006_i32FractionPart1 < 0)
				{
						TMP006_i32FractionPart1 *= -1;
				}
				//sprintf(tempString,"Ambient %3d.%03d\t", TMP006_i32IntegerPart1, TMP006_i32FractionPart1);
				//CLI_Write(tempString);
				//
				// Convert the floating point ambient temperature  to an integer part
				// and fraction part for easy printing.
				//
				TMP006_i32IntegerPart2 = (int32_t)fObject;
				TMP006_i32FractionPart2 = (int32_t)(fObject * 1000.0f);
				TMP006_i32FractionPart2= TMP006_i32FractionPart2 - (TMP006_i32IntegerPart2 * 1000);
				if(TMP006_i32FractionPart2 < 0)
				{
						TMP006_i32FractionPart2 *= -1;
				}
				//sprintf(tempString,"Object %3d.%03d\n\r", TMP006_i32IntegerPart2, TMP006_i32FractionPart2);
				//CLI_Write(tempString);
				sensorTurn=(sensorTurn+1)%NumberOfSensor;
				//sensorTurn=4;
				TimerEnable(TIMER1_BASE, TIMER_A);
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
		timer0_status=1;
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
		timer0_status=0;
}
void DisableTimer1(void)
{
		TimerDisable(TIMER1_BASE, TIMER_A);
}

void EnableTimer0(void)
{
		TimerEnable(TIMER0_BASE, TIMER_A);
		timer0_status=1;
}

void EnableTimer1(void)
{
		TimerEnable(TIMER1_BASE, TIMER_A);
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

unsigned char ToggleTimer0()
{
		if(timer0_status==1)
		{
			DisableTimer0();
		}
		else
		{
			EnableTimer0();
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

unsigned char GetTimer0Status()
{
		return timer0_status;
}





