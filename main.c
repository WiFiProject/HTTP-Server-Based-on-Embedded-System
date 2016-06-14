/*
 * main.c - HTTP server sample example
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

/*
 * Application Name     -   HTTP Server
 * Application Overview -   This is a sample application demonstrating the
 *                          capability of CC3100 device to work as a
 *                          web-server and allowing the end-users to
 *                          communicate w/ it using standard web-browsers.
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_HTTP_Server
 *                          doc\examples\http_server.pdf
 */

#include <stdio.h>
#include "simplelink.h"
#include "protocol.h"
#include "sl_common.h"

#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"

#define APPLICATION_VERSION "1.2.0"

#define SL_STOP_TIMEOUT        0xFF

#define BUF_SIZE        2000

//_u8 POST_token[] = "__SL_P_U01";
//_u8 POST_token1[] = "__SL_P_UT1";
//_u8 POST_token2[] = "__SL_P_UT2";
//_u8 GET_token[]  = "__SL_G_ULD";

_u8 LED_token[] = "__SL_P_UL1";
_u8 FTP1_token[] = "__SL_P_UF1";
_u8 UDP_token[] = "__SL_P_UUS";

_u8 FTPINFO_token[]  = "__SL_G_UFT";
_u8 LED1_token[]  = "__SL_G_UL1";
_u8 LED2_token[]  = "__SL_G_UL2";
_u8 LED3_token[]  = "__SL_G_UL3";
_u8 LED4_token[]  = "__SL_G_UL4";
_u8 TMP1_token[]  = "__SL_G_UT1";
_u8 TMP2_token[]  = "__SL_G_UT2";
_u8 BMP1_token[]  = "__SL_G_UB1";
_u8 BMP2_token[]  = "__SL_G_UB2";
_u8 BMP3_token[]  = "__SL_G_UB3";
_u8 ISL_token[]  = "__SL_G_UIS";
_u8 SHT1_token[]  = "__SL_G_US1";
_u8 SHT2_token[]  = "__SL_G_US2";
_u8 AX_token[]  = "__SL_G_UAX";
_u8 AY_token[]  = "__SL_G_UAY";
_u8 AZ_token[]  = "__SL_G_UAZ";
_u8 GX_token[]  = "__SL_G_UGX";
_u8 GY_token[]  = "__SL_G_UGY";
_u8 GZ_token[]  = "__SL_G_UGZ";
_u8 MX_token[]  = "__SL_G_UMX";
_u8 MY_token[]  = "__SL_G_UMY";
_u8 MZ_token[]  = "__SL_G_UMZ";
_u8 ER_token[]  = "__SL_G_UER";
_u8 EP_token[]  = "__SL_G_UEP";
_u8 EY_token[]  = "__SL_G_UEY";
_u8 Q1_token[]  = "__SL_G_UQ1";
_u8 Q2_token[]  = "__SL_G_UQ2";
_u8 Q3_token[]  = "__SL_G_UQ3";
_u8 Q4_token[]  = "__SL_G_UQ4";



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


#define CONFIG_IP       SL_IPV4_VAL(192,168,1,45)       /* Static IP to be configured */
#define AP_MASK         SL_IPV4_VAL(255,255,255,0)      /* Subnet Mask for the station */
#define AP_GATEWAY      SL_IPV4_VAL(192,168,1,1)        /* Default Gateway address */
#define AP_DNS          SL_IPV4_VAL(202,120,2,101)            /* DNS Server Address */

/* Application specific status/error codes */
typedef enum{
    LAN_CONNECTION_FAILED = -0x7D0,        /* Choosing this number to avoid overlap w/ host-driver's error codes */
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,
		TCP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    TCP_RECV_ERROR = TCP_SEND_ERROR -1,
		BSD_UDP_CLIENT_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;
/*
 * GLOBAL VARIABLES -- Start
 */
_u32  g_Status = 0;
char tempc[1000];
union
{
    _u8 BsdBuf[BUF_SIZE];
    _u32 demobuf[BUF_SIZE/4];
} uBuf;
char s[10] = {0}, temps[10] = {0};

_u32 port1,port2;
_i16          SockCliCID = 0;
_i16          SockCliDID = 0;
int count, N;
float data[200], temp, tdata[200];

/* IP addressed of server side socket. Should be in long format,
 * E.g: 0xc0a8010a == 192.168.1.10 */
_u32 IP_ADDR=         0xc0a8017e;
_u16 PORT_NUM=        21;           /* Port number to be used */
_u16 PORT_NUM_UDP=        5001; 
unsigned char filename[20];
unsigned char FTPPassword[20];
unsigned char FTPUsername[20];

unsigned long IP_ADDR_1 = 0;
unsigned long IP_ADDR_2 = 0;

unsigned char FTPInfoBuff[2000]={0};
unsigned char InfoPointer[51]={0};
_u32 InfoLen=0;
_u8 FTPLinkStatus=0;

_u8 EventSelector=0;

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */

_u8 filterdata[BUF_SIZE];

static _i32 BsdTcpClient(_u16 Port);
static _i32 BsdUdpServer(_u16 Port);

static _i32 configureSimpleLinkToDefaultState();
static _i32 establishConnectionWithAP();
static _i32 initializeAppVariables();
static void displayBanner();
static void establishConnectionWithFTP();
static void UDPServerRoute();
_i32 set_port_number(_u16 num);
/*
 * STATIC FUNCTION DEFINITIONS -- End
 */
 
 
 
 static void bufclean()
{
    int i;
    for (i=0; i<BUF_SIZE; i++)
        uBuf.BsdBuf[i] = '\0';
}
static void delay(int time)
{
	int clock;
    for (clock = time; clock>0; clock--);
}

/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvents is the event passed to the handler

    \return         none

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
        return;
    }
    
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write((_u8 *)" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pEvent - Contains the relevant event information
    \param[in]      pResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pEvent,
                                  SlHttpServerResponse_t *pResponse)
{
    if(pEvent == NULL || pResponse == NULL)
    {
        CLI_Write(" [HTTP EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch (pEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            _u8 status = 0;
            _u8 *ptr = 0;
						unsigned char tempString[30]={0};

            ptr = pResponse->ResponseData.token_value.data;
            pResponse->ResponseData.token_value.len = 0;
            if(pal_Memcmp(pEvent->EventData.httpTokenName.data, LED1_token,
                                         pal_Strlen(LED1_token)) == 0)
            {
                status = GetLEDStatus();
//                pal_Memcpy(ptr, "LED1_", pal_Strlen("LED1_"));
//                ptr += 5;
//                pResponse->ResponseData.token_value.len += 5;
                if(status & 0x01)
                {
                    pal_Memcpy(ptr, "On", pal_Strlen("On"));
                    ptr += 2;
                    pResponse->ResponseData.token_value.len += 2;
                }
                else
                {
                    pal_Memcpy(ptr, "Off", pal_Strlen("Off"));
                    ptr += 3;
                    pResponse->ResponseData.token_value.len += 3;
                }
                

                
            }else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, LED2_token,
                                         pal_Strlen(LED2_token)) == 0)
						{
								status = GetLEDStatus();
                if(status & 0x02)
                {
                    pal_Memcpy(ptr, "On", pal_Strlen("On"));
                    ptr += 2;
                    pResponse->ResponseData.token_value.len += 2;
                }
                else
                {
                    pal_Memcpy(ptr, "Off", pal_Strlen("Off"));
                    ptr += 3;
                    pResponse->ResponseData.token_value.len += 3;
                }
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, LED3_token,
                                         pal_Strlen(LED3_token)) == 0)
						{
								status=GetTimer0Status();
								if(status==1)
								{
										pal_Memcpy(ptr, "Toggle On", pal_Strlen("Toggle On"));
                    ptr += 9;
                    pResponse->ResponseData.token_value.len += 9;
								}
								else
								{
										pal_Memcpy(ptr, "Toggle Off", pal_Strlen("Toggle Off"));
                    ptr += 10;
                    pResponse->ResponseData.token_value.len += 10;
								}
						
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, LED4_token,
                                         pal_Strlen(LED4_token)) == 0)
						{
								pal_Memcpy(ptr, "valid", pal_Strlen("valid"));
                ptr += 5;
                pResponse->ResponseData.token_value.len += 5;
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, TMP1_token,
                                         pal_Strlen(TMP1_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", TMP006_i32IntegerPart1, TMP006_i32FractionPart1);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, TMP2_token,
                                         pal_Strlen(TMP2_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", TMP006_i32IntegerPart2, TMP006_i32FractionPart2);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, BMP1_token,
                                         pal_Strlen(BMP1_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", BMP180_i32IntegerPart1,BMP180_i32FractionPart1);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, BMP2_token,
                                         pal_Strlen(BMP2_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", BMP180_i32IntegerPart2, BMP180_i32FractionPart2);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, BMP3_token,
                                         pal_Strlen(BMP3_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", BMP180_i32IntegerPart3, BMP180_i32FractionPart3);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, ISL_token,
                                         pal_Strlen(ISL_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", ISL290_i32IntegerPart,ISL290_i32FractionPart);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);              
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, SHT2_token,
                                         pal_Strlen(SHT2_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", SHT21_i32IntegerPart1, SHT21_i32FractionPart1);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);      
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, SHT1_token,
                                         pal_Strlen(SHT1_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", SHT21_i32IntegerPart2, SHT21_i32FractionPart2);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, AX_token,
                                         pal_Strlen(AX_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[0], i32FPart[0]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, AY_token,
                                         pal_Strlen(AY_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[1], i32FPart[1]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, AZ_token,
                                         pal_Strlen(AZ_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[2], i32FPart[2]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, GX_token,
                                         pal_Strlen(GX_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[3], i32FPart[3]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, GY_token,
                                         pal_Strlen(GY_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[4], i32FPart[4]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, GZ_token,
                                         pal_Strlen(GZ_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[5], i32FPart[5]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
								ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, MX_token,
                                         pal_Strlen(MX_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[6], i32FPart[6]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, MY_token,
                                         pal_Strlen(MY_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[7], i32FPart[7]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, MZ_token,
                                         pal_Strlen(MZ_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[8], i32FPart[8]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, ER_token,
                                         pal_Strlen(ER_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[9], i32FPart[9]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, EP_token,
                                         pal_Strlen(EP_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[10], i32FPart[10]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, EY_token,
                                         pal_Strlen(EY_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[11], i32FPart[11]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, Q1_token,
                                         pal_Strlen(Q1_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[12], i32FPart[12]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, Q2_token,
                                         pal_Strlen(Q2_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[13], i32FPart[13]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, Q3_token,
                                         pal_Strlen(Q3_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[14], i32FPart[14]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, Q4_token,
                                         pal_Strlen(Q4_token)) == 0)
						{
								sprintf(tempString,"%3d.%03d", i32IPart[15], i32FPart[15]);
								pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
                ptr += pal_Strlen(tempString);
                pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
						}
						else if(pal_Memcmp(pEvent->EventData.httpTokenName.data, FTPINFO_token,
                                         pal_Strlen(FTPINFO_token)) == 0)
						{
								//tempString=?
								
								if(FTPLinkStatus==0)
								{									
										sprintf(tempString,"waiting...");
										pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
										ptr += pal_Strlen(tempString);
										pResponse->ResponseData.token_value.len += pal_Strlen(tempString);
								}
								else
								{		
										if(FTPLinkStatus==3 || FTPLinkStatus==-3)
										{
											if(FTPLinkStatus==3)
											{
													sprintf(tempString,"END With Success\n");	
													pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
													ptr += pal_Strlen(tempString);
													pResponse->ResponseData.token_value.len += pal_Strlen(tempString);	
											}	
											else
											{
													sprintf(tempString,"END With Error\n");	
													pal_Memcpy(ptr, tempString, pal_Strlen(tempString));
													ptr += pal_Strlen(tempString);
													pResponse->ResponseData.token_value.len += pal_Strlen(tempString);	
											}
										}
										else
										{
												if(FTPLinkStatus==1)
												{
													//sprintf(tempString,"SU CCESS\n");
													InfoLen=pal_Strlen(FTPInfoBuff);
													FTPLinkStatus=2;
													//InfoPointer=FTPInfoBuff;
												}
												else if(FTPLinkStatus==-1) 
												{
													//sprintf(tempString,"ER ROR\n");
													InfoLen=pal_Strlen(FTPInfoBuff);
													FTPLinkStatus=-2;
													//InfoPointer=FTPInfoBuff;
												}
												if(InfoLen>50)
												{
													for(int i=0;i<50;i++)
													{
														InfoPointer[i]=FTPInfoBuff[pal_Strlen(FTPInfoBuff)-InfoLen+i];
													}
													InfoPointer[50]='\0';
													pal_Memcpy(ptr, InfoPointer,  pal_Strlen(InfoPointer));
													ptr  +=pal_Strlen(InfoPointer);
													pResponse->ResponseData.token_value.len += pal_Strlen(InfoPointer);
													InfoLen-=50;
												}
												else
												{
													for(int i=0;i<InfoLen+1;i++)
													{
														InfoPointer[i]=FTPInfoBuff[pal_Strlen(FTPInfoBuff)-InfoLen+i];
													}
													InfoPointer[InfoLen+1]='\0';
													pal_Memcpy(ptr, InfoPointer,  pal_Strlen(InfoPointer));
													ptr  +=pal_Strlen(InfoPointer);
													pResponse->ResponseData.token_value.len += pal_Strlen(InfoPointer);
													FTPInfoBuff[0]='\0';
													if(FTPLinkStatus==2) FTPLinkStatus=3;
													else if(FTPLinkStatus==-2) FTPLinkStatus=-3;
												}		
										}											
								}
						}
						*ptr = '\0';
        }
        break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {
						_u32 IPAddress=0;
						_u16 port=0;
            _u8 led = 0;
            _u8 *ptr = pEvent->EventData.httpPostData.token_name.data;

            if(pal_Memcmp(ptr, LED_token, pal_Strlen(LED_token)) == 0)
            {
                ptr = pEvent->EventData.httpPostData.token_value.data;
                if(pal_Memcmp(ptr, "LED", 3) != 0)
                    break;

                ptr += 3;
                led = *ptr;
                ptr += 1;
                if(led == '1')
                {
                    if(pal_Memcmp(ptr, "ON", 2) == 0)
                    {
                        turnLedOn(LED1);
                    }
                    else
                    {
                        turnLedOff(LED1);
                    }
                }
                else if(led == '2')
                {
                    if(pal_Memcmp(ptr, "ON", 2) == 0)
                    {
                        turnLedOn(LED2);
                    }
                    else
                    {
                        turnLedOff(LED2);
                    }
                }
								else if(led == '3')
								{
										if(pal_Memcmp(ptr, "TO", 2) == 0)
                    {
                        ToggleTimer0();
                    }
								}
            }
						else if(pal_Memcmp(ptr, FTP1_token, pal_Strlen(FTP1_token)) == 0)
						{
								ptr = pEvent->EventData.httpPostData.token_value.data;
								//get FTP IP
								_u8 IPlen=0;
								unsigned char strbuf[8];
								_u16 buf=0;
								
								while((*(ptr+IPlen))!='.') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									strbuf[i]=(*(ptr+i));
								}
								strbuf[IPlen]='\0';

								sscanf(strbuf,"%d",&buf);
								IPAddress+=buf;
								IPAddress=IPAddress<<8;
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								while((*(ptr+IPlen))!='.') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									strbuf[i]=(*(ptr+i));
								}
								strbuf[IPlen]='\0';
								sscanf(strbuf,"%d",&buf);
								IPAddress+=buf;
								IPAddress=IPAddress<<8;
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								while((*(ptr+IPlen))!='.') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									strbuf[i]=(*(ptr+i));
								}
								strbuf[IPlen]='\0';
								sscanf(strbuf,"%d",&buf);
								IPAddress+=buf;
								IPAddress=IPAddress<<8;
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								while((*(ptr+IPlen))!=':') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									strbuf[i]=(*(ptr+i));
								}
								strbuf[IPlen]='\0';
								sscanf(strbuf,"%d",&buf);
								IPAddress+=buf;
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								//get FTP Port
								while((*(ptr+IPlen))!=',') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									strbuf[i]=(*(ptr+i));
								}
								strbuf[IPlen]='\0';
								sscanf(strbuf,"%d",&port);
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								IP_ADDR=IPAddress;
								PORT_NUM= port;
								//getFileName
								while((*(ptr+IPlen))!=',') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									filename[i]=(*(ptr+i));
								}
								filename[IPlen]='\0';
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								//getUsername
								while((*(ptr+IPlen))!=',') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									FTPUsername[i]=(*(ptr+i));
								}
								FTPUsername[IPlen]='\0';
								ptr+=IPlen+1;
								IPlen=0;
								buf=0;
								//getPassword
								while((*(ptr+IPlen))!='\0') IPlen++;
								for(int i=0;i<IPlen;i++)
								{
									FTPPassword[i]=(*(ptr+i));
								}
								FTPPassword[IPlen]='\0';
								//if IP Port valid
								EventSelector=1;
								
						}
						else if(pal_Memcmp(ptr, UDP_token, pal_Strlen(UDP_token)) == 0)
						{
								EventSelector=2;
						}
						
        }
        break;

        default:
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
        return;
    }
 
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)	
{
    if(pSock == NULL)
    {
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
        return;
    }
    
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            /*
             * TX Failed
             *
             * Information about the socket descriptor and status will be
             * available in 'SlSockEventData_t' - Applications can use it if
             * required
             *
            * SlSockEventData_u *pEventData = NULL;
            * pEventData = & pSock->socketAsyncEvent;
             */
            switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation, failed to transmit all queued packets\n\r");
                    break;
                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                    break;
            }
            break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
            break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */


/*
 * Application's entry point
 */
int main(int argc, char** argv)
{
    _u8   SecType = 0;
    _i32   retVal = -1;
    _i32   mode = ROLE_STA;
	
		SlNetCfgIpV4Args_t ipV4;
	
		retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);
	
		/* Stop WDT and initialize the system-clock of the MCU */
    stopWDT();
    initClk();
		initTimer();
		
		/* Configure command line interface */
    CLI_Configure();
		
		CLI_Write(" Device is configured in default state \n\r");
    displayBanner();
		
    initLEDs();
		
		initTimer1();
		
		initI2C();
		
		
    

    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
		 

		
    //retVal = configureSimpleLinkToDefaultState();
		retVal=0;
    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");
		
		ipV4.ipV4 = CONFIG_IP;
    ipV4.ipV4Mask = AP_MASK;
    ipV4.ipV4Gateway = AP_GATEWAY;
    ipV4.ipV4DnsServer = AP_DNS;

    CLI_Write(" Configuring device to connect using static IP \n\r");

    /* After calling this API device will be configure for static IP address.*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_STATIC_ENABLE,1,
                    sizeof(SlNetCfgIpV4Args_t), (_u8 *)&ipV4);
    if(retVal < 0)
        LOOP_FOREVER();
		
		/* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is configured \n\r");
    /* Process the async events from the NWP */
    while(1)
    {
        _SlNonOsMainLoopTask();
				if(EventSelector==1) establishConnectionWithFTP();
				if(EventSelector==2)	UDPServerRoute();
    }
}

static void establishConnectionWithFTP()
{
		_i32   retVal = -1;
		
		CLI_Write(" Establishing connection with FTP server \n\r");
    /*Before proceeding, please make sure to have a server waiting on PORT_NUM*/
		retVal = BsdTcpClient(PORT_NUM);
	
		FTPLinkStatus=1;
		EventSelector=0;
}

static void UDPServerRoute()
{
		_i32 retVal = -1;
		DisableTimer1();
		DisableTimer0();
		CLI_Write(" Turn to UDP server \n\r");
		retVal = BsdUdpServer(PORT_NUM_UDP);
    if(retVal < 0)
        CLI_Write(" Failed to read data from the UDP client \n\r");
    else
        CLI_Write(" Successfully received data from UDP client \n\r");
		EventSelector=0;
		EnableTimer0();
		EnableTimer1();
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = pal_Strlen(PASSKEY);
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

static _i32 getPortNum(_i8 *buf)
{
    _u32 i,d,dec = 1;
		unsigned char tempString[20];
    port1 = 0;
    port2 = 0;
    for (d = pal_Strlen(buf)-1; d>=0; d--)
    {
        if (buf[d] == ',') break;
    }
    for (i = d-1; buf[i] != ','; i--)
    {
        port1 += dec*(buf[i]-48);
        dec *= 10;
    }
    dec = 1;
    for (d = pal_Strlen(buf)-1; d>=0; d--)
    {
        if (buf[d] == ')') break;
    }
    for (i = d-1; buf[i] != ','; i--)
    {
        port2 += dec*(buf[i]-48);
        dec *= 10;
    }
    return 0;
		sprintf(tempString,"%d %d",port1,port2);
    CLI_Write(tempString);
}


static _i32 BsdTcpClient(_u16 Port)
{
    int i = 0, j = 0;
    //getwave(Port);
    SlSockAddrIn_t  Addr;
    _u16 PortD;
    _u16          idx = 0;
    _u16          AddrSize = 0;
    _i16          Status = 0;
    _u16          LoopCount = 0;
    strcpy(uBuf.BsdBuf,"whatever");
    
    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((_u16)Port);
    Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);
    AddrSize = sizeof(SlSockAddrIn_t);


    //create Command Socket
    SockCliCID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( SockCliCID < 0 )
    {		
        CLI_Write(" [FTP Client] Create socket Error \n\r");
				strcat(FTPInfoBuff," [FTP Client] Create socket Error \n");
				FTPLinkStatus=-1;
				EventSelector=0;
        ASSERT_ON_ERROR(SockCliCID);
    }

    //Connect to the server
    Status = sl_Connect(SockCliCID, ( SlSockAddr_t *)&Addr, AddrSize);
    if( Status < 0 )
    {
        sl_Close(SockCliCID);
        CLI_Write(" [FTP Client]  FTP connection Error \n\r");
				strcat(FTPInfoBuff," [FTP Client]  FTP connection Error \n");
				FTPLinkStatus=-1;
				EventSelector=0;
        ASSERT_ON_ERROR(Status);
    }

    //Read welcome message
    _u32 recvSize = BUF_SIZE;
    _u16 len;
    bufclean();
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    if( Status <= 0 )
    {
        sl_Close(SockCliCID);
        CLI_Write(" [FTP Server] Data recv Error \n\r");
				strcat(FTPInfoBuff," [FTP Server] Data recv Error \n");
				FTPLinkStatus=-1;
				EventSelector=0;
        ASSERT_ON_ERROR(TCP_RECV_ERROR);
    }
    for (i=1; i<recvSize; i++)
    {
        if ((uBuf.BsdBuf[i] == '\n')&&(uBuf.BsdBuf[i-1]=='\r')) break;
    }

    //Print the welcome message
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //send User name and password
		sprintf(uBuf.BsdBuf,"USER %s\r\n",FTPUsername);
    //pal_Strcpy(uBuf.BsdBuf,"USER hyacinth\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );;
		Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
		CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
		sprintf(uBuf.BsdBuf,"PASS %s\r\n",FTPPassword);
    //pal_Strcpy(uBuf.BsdBuf,"PASS 123\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
		Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //send PASV
    bufclean();
    pal_Strcpy(uBuf.BsdBuf,"PASV\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    //delay(40000);
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //get the Port number
    getPortNum(uBuf.BsdBuf);
    CLI_Write("Data port received\n\r");
		strcat(FTPInfoBuff,"Data port received\n");
    //create data socket
    PortD = 256 * port1 + port2;
    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((_u16)PortD);
    Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);
    AddrSize = sizeof(SlSockAddrIn_t);
    SockCliDID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);

    //connect to server
    Status = sl_Connect(SockCliDID, ( SlSockAddr_t *)&Addr, AddrSize);

    if( SockCliDID < 0 )
    {
        CLI_Write(" [FTP Client] Create socket Error \n\r");
				strcat(FTPInfoBuff," [FTP Client] Create socket Error \n");
				FTPLinkStatus=-1;
				EventSelector=0;
        ASSERT_ON_ERROR(SockCliDID);
    }

    //check file size
		sprintf(uBuf.BsdBuf,"SIZE %s\r\n",filename);
    //pal_Strcpy(uBuf.BsdBuf,"SIZE wave.txt\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //download file
		sprintf(uBuf.BsdBuf,"RETR %s\r\n",filename);
    //pal_Strcpy(uBuf.BsdBuf,"RETR wave.txt\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    Status = sl_Recv(SockCliDID, uBuf.BsdBuf, recvSize, 0);
    pal_Strcpy(filterdata,uBuf.BsdBuf);
    bufclean();
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //close data socket
    Status = sl_Close(SockCliDID);
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //delay(50000);


    //CLI_Write(filterdata);
    //waveFilter();
    float *p;
    int jj = 0;
    for (i = 0; i <pal_Strlen(filterdata); i++)
    {
        if (((i == 0) || (filterdata[i - 1] == ' ')))
        {
            p = &(data[jj]);
            sscanf(&(filterdata[i]),"%f", p);
            jj++;
        }
    }

    N = jj;

    for (i=0; i<N; i++)
    {
        tdata[i] = data[i];
        if ((i-2>=0)&&(i+2<N))
        {
            tdata[i] = (data[i-2]+data[i-1]+data[i]+data[i+1]+data[i+2])/5.0;
        }
    }

    pal_Strcpy(filterdata, "");
    for (i=0; i<N; i++)
    {
        if (i == 120)
        {
            CLI_Write("now");
						strcat(FTPInfoBuff,"now");
        }
        int temp = 0;
        if (tdata[i]<0)
        {
            tdata[i] = -tdata[i];
            pal_Strcat(filterdata,"-");
        }
        temp = ((int)tdata[i]);
        len = 0;
        while (temp>0)
        {
            temps[len] = 48 + (temp % 10);
            temp = temp / 10;
            len++;
        }
        for (j = 0; j<len; j++) s[j] = temps[len - j - 1];
        s[len] = '\0';
        if (len == 0) pal_Strcpy(s,"0");
        //CLI_Write(s);
        pal_Strcat(filterdata, s);

        pal_Strcat(filterdata, ".");
        temp = (int)((tdata[i]*1000))%1000;
        len = 0;
        while (temp>0)
        {
            temps[len] = 48 + (temp % 10);
            temp = temp / 10;
            len++;
        }
        for (j = 0; j<len; j++) s[j] = temps[len - j - 1];
        s[len] = '\0';
        if (len == 0) pal_Strcpy(s,"0");
        //CLI_Write(s);
        pal_Strcat(filterdata, s);
        pal_Strcat(filterdata, " ");
    }
    CLI_Write("Wave data receiving OK\n\r");
		strcat(FTPInfoBuff,"Wave data receiving OK\n");
    CLI_Write(filterdata);
		//strcat(FTPInfoBuff,filterdata);

    //send PASV
    //delay(400000);
		
    pal_Strcpy(uBuf.BsdBuf,"PASV\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    delay(400000);
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);

    //get the Port number
    getPortNum(uBuf.BsdBuf);
    CLI_Write("Data port received\r\n");
		strcat(FTPInfoBuff,"Data port received\n");
    //create data socket
    PortD = 256 * port1 + port2;
    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((_u16)PortD);
    Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);
    AddrSize = sizeof(SlSockAddrIn_t);
    SockCliDID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);

    //connect to server
    Status = sl_Connect(SockCliDID, ( SlSockAddr_t *)&Addr, AddrSize);

    if( SockCliDID < 0 )
    {
        CLI_Write(" [FTP Client] Create socket Error \n\r");
				strcat(FTPInfoBuff," [FTP Client] Create socket Error \n");
				FTPLinkStatus=-1;
				EventSelector=0;
        ASSERT_ON_ERROR(SockCliDID);
    }

    //upload file
		sprintf(uBuf.BsdBuf,"STOR %s\r\n",filename);
    //pal_Strcpy(uBuf.BsdBuf,"STOR wave.txt\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    //delay(50000);

    for (i=0; i<BUF_SIZE; i++) uBuf.BsdBuf[i] = '\0';
    pal_Strcpy(uBuf.BsdBuf,filterdata);
    pal_Strcat(uBuf.BsdBuf,"\n");

    Status = sl_Send(SockCliDID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //close data socket
    Status = sl_Close(SockCliDID);
    //delay(50000);
    
    //send quit
    pal_Strcpy(uBuf.BsdBuf,"QUIT\r\n");
    Status = sl_Send(SockCliCID, uBuf.BsdBuf, pal_Strlen(uBuf.BsdBuf), 0 );
    bufclean();
    Status = sl_Recv(SockCliCID, uBuf.BsdBuf, recvSize, 0);
    pal_Strcat(uBuf.BsdBuf,"");
    CLI_Write(uBuf.BsdBuf);
		strcat(FTPInfoBuff,uBuf.BsdBuf);
    //close command socket
    Status = sl_Close(SockCliCID);  

    CLI_Write("All is done\n\r");
		strcat(FTPInfoBuff,"All is done\n");
		
    return 0;
    //connect to the new data socket
}

/*!
    \brief Opening a UDP server side socket and receiving data

    This function opens a UDP socket in Listen mode and waits for incoming
    UDP packets from the connected client.

    \param[in]      port number on which the server will be listening on

    \return         0 on success, Negative value on Error.

    \note

    \warning
*/
static _i32 BsdUdpServer(_u16 Port)
{
    SlSockAddrIn_t  Addr;
    SlSockAddrIn_t  LocalAddr;
    _u16          idx = 0;
    _u16          AddrSize = 0;
    _i16          SockID = 0;
    _i16          SockIDC = 0;
    _i16          Status = 0;
    _u16          LoopCount = 0;
    _u16          recvSize = 0;

    for (idx=0 ; idx<BUF_SIZE ; idx++)
    {
        uBuf.BsdBuf[idx] = (_u8)(idx % 10);
    }

    LocalAddr.sin_family = SL_AF_INET;
    LocalAddr.sin_port = sl_Htons((_u16)Port);
    LocalAddr.sin_addr.s_addr = 0;

    AddrSize = sizeof(SlSockAddrIn_t);
		CLI_Write("1\n\r");
    SockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    //SockIDC = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( SockID < 0 )
    {
        ASSERT_ON_ERROR(SockID);
    }
    if( SockIDC < 0 )
    {
        ASSERT_ON_ERROR(SockIDC);
    }

    Status = sl_Bind(SockID, (SlSockAddr_t *)&LocalAddr, AddrSize);
    if( Status < 0 )
    {
        Status = sl_Close(SockID);
        ASSERT_ON_ERROR(Status);
    }
    recvSize = BUF_SIZE;
    int i,cnt = 0;
    do
    {
        //cleanbuf();
				CLI_Write("2\n\r");
        for (i=0 ; i<BUF_SIZE; i++){ uBuf.BsdBuf[i] = '\0'; }

        Status = sl_RecvFrom(SockID, &uBuf.BsdBuf, recvSize, 0,
                                (SlSockAddr_t *)&Addr, (SlSocklen_t*)&AddrSize );

        if ((uBuf.BsdBuf[0] == '-')&&(uBuf.BsdBuf[1] == 'e')&&(uBuf.BsdBuf[2] == 'x'))
        {
            if (Addr.sin_addr.s_addr == IP_ADDR_1) pal_Strcpy(tempc,"Client 1 stops the chat...\n\r");
            else if (Addr.sin_addr.s_addr == IP_ADDR_2) pal_Strcpy(tempc,"Client 2 stops the chat...\n\r");
            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_1;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_2;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
            break;
        }
        if(Status < 0)
        {
            sl_Close(SockID);
            ASSERT_ON_ERROR(Status);
        }
        if (Status > 50) continue;
        if (IP_ADDR_1 == 0)
        {
            IP_ADDR_1 = Addr.sin_addr.s_addr;
            //IP_ADDR_2 = Addr.sin_addr.s_addr;
            CLI_Write("Client 1 connected\n\r");
            //CLI_Write("Client 2 connected\n\r");
            pal_Strcpy(tempc,"Client 1 comes in\n\r");
            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_1;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_2;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
        }
        else if ((IP_ADDR_2 == 0)&&(IP_ADDR_1 != Addr.sin_addr.s_addr))
        {
            IP_ADDR_2 = Addr.sin_addr.s_addr;
            CLI_Write("Client 2 connected\n\r");
            pal_Strcpy(tempc,"Client 2 comes in\n\r");

            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_1;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
            Addr.sin_family = SL_AF_INET;
            Addr.sin_port = sl_Htons((_u16)4001);
            Addr.sin_addr.s_addr = IP_ADDR_2;
            Status = sl_SendTo(SockID, tempc, 500, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);   
        }
        else if ((IP_ADDR_2 != 0)&&(IP_ADDR_1 != 0)){
            //if (Addr.sin_addr.s_addr == IP_ADDR_1)
            {

                if (Addr.sin_addr.s_addr == IP_ADDR_1) sprintf(tempc,"Client_1 said: %s",uBuf.BsdBuf);
                else if (Addr.sin_addr.s_addr == IP_ADDR_2) sprintf(tempc,"Client_2 said: %s",uBuf.BsdBuf);
                if ((Addr.sin_addr.s_addr == IP_ADDR_1) || (Addr.sin_addr.s_addr == IP_ADDR_2)) 
                {
                    Addr.sin_family = SL_AF_INET;
                    Addr.sin_port = sl_Htons((_u16)4001);
                    Addr.sin_addr.s_addr = IP_ADDR_1;
                    Status = sl_SendTo(SockID, tempc, 500, 0,
                                       (SlSockAddr_t *)&Addr, AddrSize);   
                    Addr.sin_family = SL_AF_INET;
                    Addr.sin_port = sl_Htons((_u16)4001);
                    Addr.sin_addr.s_addr = IP_ADDR_2;
                    Status = sl_SendTo(SockID, tempc, 500, 0,
                                       (SlSockAddr_t *)&Addr, AddrSize);   
                }
            }
            /*else if (Addr.sin_addr.s_addr == IP_ADDR_2)
            {
                Addr.sin_addr.s_addr = IP_ADDR_1;
                Addr.sin_family = SL_AF_INET;
                Addr.sin_port = sl_Htons((_u16)4001);
                sprintf(uBuf.BsdBuf,"Client_2 said: %s",uBuf.BsdBuf);
                Status = sl_SendTo(SockID, uBuf.BsdBuf, 500, 0,
                                   (SlSockAddr_t *)&Addr, AddrSize);      
            }*/
        }
        delay(1600000);
    } while(1);

    Status = sl_Close(SockID);
    ASSERT_ON_ERROR(Status);
    CLI_Write("Chat exits successfully...\n\r");
    return SUCCESS;
}

/*!
    \brief Set the HTTP port

    This function can be used to change the default port (80) for HTTP request

    \param[in]      num- contains the port number to be set

    \return         None

    \note

    \warning
*/
_i32 set_port_number(_u16 num)
{
    _NetAppHttpServerGetSet_port_num_t port_num;
    _i32 status = -1;

    port_num.port_number = num;

    /*Need to restart the server in order for the new port number configuration
     *to take place */
    status = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
    ASSERT_ON_ERROR(status);

    status  = sl_NetAppSet (SL_NET_APP_HTTP_SERVER_ID, NETAPP_SET_GET_HTTP_OPT_PORT_NUMBER,
                  sizeof(_NetAppHttpServerGetSet_port_num_t), (_u8 *)&port_num);
    ASSERT_ON_ERROR(status);

    status = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
    ASSERT_ON_ERROR(status);

    return SUCCESS;
}



/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;
		
    mode = sl_Start(0, 0, 0);
		
    ASSERT_ON_ERROR(mode);
		
    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }
		
    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);
		
    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);
		
    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);
		
    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
		EventSelector=0;
    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" HTTP Server application - By Guyao and cxy ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
