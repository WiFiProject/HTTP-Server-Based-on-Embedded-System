/*
 * board.h - tiva-c-connected launchpad configuration
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

#ifndef _BOARD_H
#define _BOARD_H


#define PIN_HIGH                              0xFF
#define PIN_LOW                              (!PIN_HIGH)


extern signed int i32IPart[16], i32FPart[16];

extern signed int SHT21_i32IntegerPart1;
extern signed int SHT21_i32FractionPart1;
extern signed int SHT21_i32IntegerPart2;
extern signed int SHT21_i32FractionPart2;

extern signed int ISL290_i32IntegerPart, ISL290_i32FractionPart;

extern signed int BMP180_i32IntegerPart1;
extern signed int BMP180_i32FractionPart1;
extern signed int BMP180_i32IntegerPart2;
extern signed int BMP180_i32FractionPart2;
extern signed int BMP180_i32IntegerPart3;
extern signed int BMP180_i32FractionPart3;

extern signed int TMP006_i32IntegerPart1;
extern signed int TMP006_i32FractionPart1;
extern signed int TMP006_i32IntegerPart2;
extern signed int TMP006_i32FractionPart2;



typedef void (*P_EVENT_HANDLER)(void* pValue);

/*!
    \brief register an interrupt handler for the host IRQ

    \param[in]      InterruptHdl    -    pointer to interrupt handler function

    \param[in]      pValue          -    pointer to a memory strcuture that is 
                    passed to the interrupt handler.

    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa
    \note           If there is already registered interrupt handler, the 
                    function should overwrite the old handler with the new one

    \warning
*/
int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue);

/*!
    \brief          GPIOM interrupt handler

    \param[in]      none

    \return         none

    \note

    \warning
*/
void GPIOM_intHandler();

/*!
    \brief             Enables the CC3100

    \param[in]         none

    \return            none

    \note

    \warning
*/
void CC3100_enable();

/*!
    \brief             Disables the CC3100

    \param[in]         none

    \return            none

    \note

    \warning
*/
void CC3100_disable();

/*!
    \brief          Enables the interrupt from the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_InterruptEnable();

/*!
    \brief          Disables the interrupt from the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_InterruptDisable();

/*!
    \brief          Stops the Watch Dog timer

    \param[in]      none

    \return         none

    \note

    \warning
*/
void stopWDT();

/*!
    \brief          Initialize the system clock of MCU

    \param[in]      none

    \return         none

    \note

    \warning
*/
void initClk();
void initTimer();
void
Timer0IntHandler(void);

void
SysTickIntHandler();

void DisableTimer0(void);
void EnableTimer0(void);

void initI2C(void);

void I2CIntHandler(void);

void initTimer1(void);

void Timer1IntHandler(void);

void MPU9150AppCallback(void *pvCallbackData, unsigned     int ui8Status);

void SHT21AppCallback(void *pvCallbackData, unsigned     int ui8Status);

void ISL29023AppCallback(void *pvCallbackData, unsigned     int ui8Status);

void GPIOPortEIntHandler(void);

void BMP180AppCallback(void* pvCallbackData, unsigned     int ui8Status);

//the function related to the tmp006
//void IntHandlerGPIOPortH(void);

void TMP006AppCallback(void *pvCallbackData, unsigned     int ui8Status);

/*!
    \brief      Masks the Host IRQ

	\param[in]      none

    \return         none

    \warning
*/
void MaskIntHdlr();

/*!
    \brief     Unmasks the Host IRQ
	
	\param[in]      none

    \return         none

    \warning
*/
void UnMaskIntHdlr();

/*!
    \brief          UART Rx and Tx handler. Handles the traffic that otherwise 
                    goes unhandled.

    \param[in]      none

    \return         none

    \note

    \warning
*/
void UARTRxTxHandler();

/*!
    \brief     Produce delay in ms

    \param[in]         interval - Time in ms

    \return            none

    \note

    \warning
*/
void Delay(unsigned long interval);
typedef enum
{
    NO_LED,
    LED1,
    LED2
} ledEnum;
void initLEDs();

/*!
    \brief          Turn on the LED on MCU

    \param[in]      ledNum - LED to be turned on

    \return         none

    \note

    \warning
*/
void turnLedOn(char ledNum);

/*!
    \brief          Turn off the LED on MCU

    \param[in]      ledNum - LED to be turned of

    \return         none

    \note

    \warning
*/
void turnLedOff(char ledNum);

/*!
    \brief          Toggle the LED

    \param[in]      ledNum - LED to be toggled

    \return         none

    \note

    \warning
*/
void toggleLed(char ledNum);

/*!
    \brief          Get the LED status

    \param[in]      none

    \return         unsigned char - byte containing the status of LED

    \note

    \warning
*/

unsigned char ToggleTimer0(void);
	
unsigned char GetLEDStatus(void);

unsigned char GetTimer0Status(void);


#endif
