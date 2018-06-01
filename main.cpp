/*
 * MCS03.cpp
 *
 * Created: 5/31/2018 5:33:21 PM
 * Author : Amir
 */ 

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include "ports.h"
#include "define.h"
#include "CommandGet.h"
#include "SFlash.h"
#include "CMDExecution.h"
#include <util/delay.h>

char StopTime=0;

// System Clocks initialization
void system_clocks_init(void)
{
	unsigned char n,s;

	// Optimize for speed
	#pragma optsize-
	// Save interrupts enabled/disabled state
	s=SREG;
	// Disable interrupts
	asm("cli");

	// Internal 32 kHz RC oscillator initialization
	// Enable the internal 32 kHz RC oscillator
	OSC.CTRL|=OSC_RC32KEN_bm;
	// Wait for the internal 32 kHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);

	// Internal 32 MHz RC oscillator initialization
	// Enable the internal 32 MHz RC oscillator
	OSC.CTRL|=OSC_RC32MEN_bm;

	// System Clock prescaler A division factor: 1
	// System Clock prescalers B & C division factors: B:1, C:1
	// ClkPer4: 32000.000 kHz
	// ClkPer2: 32000.000 kHz
	// ClkPer:  32000.000 kHz
	// ClkCPU:  32000.000 kHz
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;

	// Internal 32 MHz RC osc. calibration reference clock source: 32.768 kHz Internal Osc.
	OSC.DFLLCTRL&= ~(OSC_RC32MCREF_bm | OSC_RC2MCREF_bm);
	// Enable the auto-calibration of the internal 32 MHz RC oscillator
	DFLLRC32M.CTRL|=DFLL_ENABLE_bm;

	// Wait for the internal 32 MHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);

	// Select the system clock source: 32 MHz Internal RC Osc.
	n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=n;

	// Disable the unused oscillators: 2 MHz, external clock/crystal oscillator, PLL
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);

	// ClkPer output disabled
	PORTCFG.CLKEVOUT&= ~PORTCFG_CLKOUT_gm;
	// Restore interrupts enabled/disabled state
	SREG=s;
	// Restore optimization for size if needed
	#pragma optsize_default
}

// PORTH interrupt 0 service routine
ISR (PORTH_INT0_vect)
{
	if(JogPos)
	{
		printf("Jog+\r\n");
		if(!JogNeg)
		{
			MaxSpeed=10000;
			IACC=5000;
			IDEC=5000;
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			
			ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			ACCSpeedINT = MaxSpeed / ACCSpeedINT;
			
			DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			DECSpeedINT= MaxSpeed / DECSpeedINT;
			switch(SelectedAxis)
			{
				case 1:
				{
					
					if(!LIM_POS1 && Axes[0].HardwareLimitPosIsActive)
					break;
					MaxSpeed=Axes[0].MaxSpeed;
					IACC=Axes[0].ACC;
					IDEC=Axes[0].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					SET_DIR1;
					FreeJog=1;
					Ma1=1000;
					DistanceToGo1=0x7FFFFFFF;
					break;
					
				}
				case 2:
				{
					if(!LIM_POS2 && Axes[1].HardwareLimitPosIsActive)
					break;
					MaxSpeed=Axes[1].MaxSpeed;
					IACC=Axes[1].ACC;
					IDEC=Axes[1].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					SET_DIR2;
					FreeJog=2;
					Ma2=1000;
					DistanceToGo2=0x7FFFFFFF;
					break;
					
				}
				case 3:
				{
					if(!LIM_POS3 && Axes[2].HardwareLimitPosIsActive)
					break;
					MaxSpeed=Axes[2].MaxSpeed;
					IACC=Axes[2].ACC;
					IDEC=Axes[2].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					SET_DIR3;
					FreeJog=3;
					Ma3=1000;
					DistanceToGo3=0x7FFFFFFF;
					break;
					
				}
			}
			
			ACC=1;
		}
	}
	else if((FreeJog==1 && DIR1)||(FreeJog==2 && DIR2)||(FreeJog==3 && DIR3))
	{
		ACC=0;
		DEC=1;
	}
}

// PORTH interrupt 0 service routine
ISR (PORTD_INT0_vect)
{
	if(JogNeg)
	{
		if(!JogPos)
		{
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			
			ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			ACCSpeedINT = MaxSpeed / ACCSpeedINT;
			
			DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			DECSpeedINT= MaxSpeed / DECSpeedINT;
			switch(SelectedAxis)
			{
				case 1:
				{
					if(!LIM_NEG1 && Axes[0].HardwareLimitNegIsActive)
					break;
					MaxSpeed=Axes[0].MaxSpeed;
					IACC=Axes[0].ACC;
					IDEC=Axes[0].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					CLR_DIR1;
					FreeJog=1;
					Ma1=1000;
					DistanceToGo1=0x7FFFFFFF;
					break;
					
				}
				case 2:
				{
					if(!LIM_NEG2 && Axes[1].HardwareLimitNegIsActive)
					break;
					MaxSpeed=Axes[1].MaxSpeed;
					IACC=Axes[1].ACC;
					IDEC=Axes[1].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					CLR_DIR2;
					FreeJog=2;
					Ma2=1000;
					DistanceToGo2=0x7FFFFFFF;
					break;
					
				}
				case 3:
				{
					if(!LIM_NEG3 && Axes[2].HardwareLimitNegIsActive)
					break;
					MaxSpeed=Axes[2].MaxSpeed;
					IACC=Axes[2].ACC;
					IDEC=Axes[2].DEC;
					
					ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
					ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
					DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
					DECSpeedINT= MaxSpeed / DECSpeedINT;
					
					CLR_DIR3;
					FreeJog=3;
					Ma3=1000;
					DistanceToGo3=0x7FFFFFFF;
					break;
				}
			}
			
			ACC=1;
		}
	}
	else if((FreeJog==1 && !DIR1)||(FreeJog==2 && !DIR2)||(FreeJog==3 && !DIR3))
	{
		ACC=0;
		DEC=1;
	}
}

ISR (PORTD_INT1_vect)
{
	//printf("PortD INT1\r\n");
	if(RunProgram)
	{
		//     //printf("Start Program...\r\n");
		//     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
		//     if (!ProgramRun)
		//     {
		//       //printf("Get Run Command\r\n");
		//       while (AxisMoving == 1)
		//       {
		//         AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
		//       }
		//       ContinuousRead=0;
		//       PRGEXEindex = 0;
		//       ProgramRun = 1;
		//     }
		//     else if(IsPause==1)
		//     {
		//         PRGExe();
		//         IsPause=0;
		//     }
	}
	if(StopProgram)
	{
		if(ProgramRun)
		{
			IsPause=1;
			DistanceToGo1=0;
			DistanceToGo2=0;
			DistanceToGo3=0;
			sprintf(BufferSend,"!4100\r\n");
			Responsing=1;
		}
		
		StopTime=0;
		while(StopProgram)
		{
			_delay_ms(100);
			StopTime++;
			if(StopTime>30)
			{


				DO1_CLR;

				AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
				while (AxisMoving == 1)
				{
					AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
				}
				if(SoftRef1<AxisPosition1) CLR_DIR1;
				if(SoftRef1>AxisPosition1) SET_DIR1;
				
				if(SoftRef2<AxisPosition2) CLR_DIR2;
				if(SoftRef2>AxisPosition2) SET_DIR2;
				
				if(SoftRef3<AxisPosition3) CLR_DIR3;
				if(SoftRef3>AxisPosition3) SET_DIR3;
				
				//printf("SoftRef1: %d\tSoftRef2: %d\tSoftRef3: %d\r\n",SoftRef1,SoftRef2,SoftRef3);
				DistanceToGo1=labs(AxisPosition1-SoftRef1);
				DistanceToGo2=labs(AxisPosition2-SoftRef2);
				DistanceToGo3=labs(AxisPosition3-SoftRef3);
				//printf("DistanceToGo1: %d\tDistanceToGo2: %d\tDistanceToGo3: %d\r\n",DistanceToGo1,DistanceToGo2,DistanceToGo3);
				if(DistanceToGo1 > DistanceToGo2 && DistanceToGo1 > DistanceToGo3)
				{
					MaxSpeed = Axes[0].MaxSpeed;
					IACC=Axes[0].ACC;
					IDEC=Axes[0].DEC;
					MaxDistanceToGo=labs(AxisPosition1-SoftRef1);
				}
				if(DistanceToGo2 > DistanceToGo1 && DistanceToGo2 > DistanceToGo3)
				{
					MaxSpeed = Axes[1].MaxSpeed;
					IACC=Axes[1].ACC;
					IDEC=Axes[1].DEC;
					MaxDistanceToGo=labs(AxisPosition2-SoftRef2);
				}
				if(DistanceToGo3 > DistanceToGo1 && DistanceToGo3 > DistanceToGo2)
				{
					MaxSpeed = Axes[2].MaxSpeed;
					IACC=Axes[2].ACC;
					IDEC=Axes[2].DEC;
					MaxDistanceToGo=labs(AxisPosition3-SoftRef3);
				}
				//printf("MaxDTG: %d\r\n",MaxDistanceToGo);
				if(MaxSpeed>0)
				{
					Ma1=(unsigned int)((float)((float)DistanceToGo1/(float)MaxDistanceToGo)*1000);
					Ma2=(unsigned int)((float)((float)DistanceToGo2/(float)MaxDistanceToGo)*1000);
					Ma3=(unsigned int)((float)((float)DistanceToGo3/(float)MaxDistanceToGo)*1000);
					//printf("MaxSpeed: %d\tMa1: %d\tMa2: %d\tMa3: %d\r\n",MaxSpeed,Ma1,Ma2,Ma3);
				}
				CurSpeedFrq=MaxSpeed;
				ACC = 0;
				DEC = 0;
				TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
				tcc0_init();
				TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
				ProgramRun=0;
				//DistanceToGo1=0;
				//DistanceToGo2=0;
				//DistanceToGo3=0;
			}
		}
		
	}
}
// USARTC0 initialization
void usartc0_init(void)
{
	// Note: The correct PORTC direction for the RxD, TxD and XCK signals
	// is configured in the ports_init function.

	// Transmitter is enabled
	// Set TxD=1
	PORTC.OUTSET=0x08;

	// Communication mode: Asynchronous USART
	// Data bits: 8
	// Stop bits: 1
	// Parity: Disabled
	USARTC0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

	// Receive complete interrupt: Medium Level
	// Transmit complete interrupt: Disabled
	// Data register empty interrupt: Disabled
	USARTC0.CTRLA=(USARTC0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
	USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;

	// Required Baud rate: 115200
	// Real Baud Rate: 115211.5 (x1 Mode), Error: 0.0 %
	USARTC0.BAUDCTRLA=0x2E;
	USARTC0.BAUDCTRLB=((0x09 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x08;

	// Receiver: On
	// Transmitter: On
	// Double transmission speed mode: Off
	// Multi-processor communication mode: Off
	USARTC0.CTRLB=(USARTC0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_RXEN_bm | USART_TXEN_bm;
}

// USARTC0 Receiver buffer
#define RX_BUFFER_SIZE_USARTC0 8
char rx_buffer_usartc0[RX_BUFFER_SIZE_USARTC0];

#if RX_BUFFER_SIZE_USARTC0 <= 256
unsigned char rx_wr_index_usartc0=0,rx_rd_index_usartc0=0;
#else
unsigned int rx_wr_index_usartc0=0,rx_rd_index_usartc0=0;
#endif

#if RX_BUFFER_SIZE_USARTC0 < 256
unsigned char rx_counter_usartc0=0;
#else
unsigned int rx_counter_usartc0=0;
#endif

// This flag is set on USARTC0 Receiver buffer overflow
bool rx_buffer_overflow_usartc0=0;

// USARTC0 Receiver interrupt service routine
ISR (USARTC0_RXC_vect)
{
	unsigned char status;
	char data;

	status=USARTC0.STATUS;
	data=USARTC0.DATA;
	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		rx_buffer_usartc0[rx_wr_index_usartc0++]=data;
		#if RX_BUFFER_SIZE_USARTC0 == 256
		// special case for receiver buffer size=256
		if (++rx_counter_usartc0 == 0) rx_buffer_overflow_usartc0=1;
		#else
		if (rx_wr_index_usartc0 == RX_BUFFER_SIZE_USARTC0) rx_wr_index_usartc0=0;
		if (++rx_counter_usartc0 == RX_BUFFER_SIZE_USARTC0)
		{
			rx_counter_usartc0=0;
			rx_buffer_overflow_usartc0=1;
		}
		#endif
	}
}

// Receive a character from USARTC0
// USARTC0 is used as the default input device by the 'getchar' function
#define _ALTERNATE_GETCHAR_

#pragma used+
//char getchar(void)
//{
	//char data;
//
	//while (rx_counter_usartc0==0);
	//data=rx_buffer_usartc0[rx_rd_index_usartc0++];
	//#if RX_BUFFER_SIZE_USARTC0 != 256
	//if (rx_rd_index_usartc0 == RX_BUFFER_SIZE_USARTC0) rx_rd_index_usartc0=0;
	//#endif
	//#asm("cli")
	//--rx_counter_usartc0;
	//#asm("sei")
	//return data;
//}
#pragma used-

// Write a character to the USARTC0 Transmitter
// USARTC0 is used as the default output device by the 'putchar' function
#define _ALTERNATE_PUTCHAR_

#pragma used+
//void putchar(char c)
//{
	//while ((USARTC0.STATUS & USART_DREIF_bm) == 0);
	//USARTC0.DATA=c;
//}
#pragma used-

int main(void)
{
    // Declare your local variables here
    unsigned char n;

    // Interrupt system initialization
    // Optimize for speed
    #pragma optsize-
    // Make sure the interrupts are disabled
    asm("cli");
    // Low level interrupt: On
    // Round-robin scheduling for low level interrupt: Off
    // Medium level interrupt: On
    // High level interrupt: On
    // The interrupt vectors will be placed at the start of the Application FLASH section
    n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
    PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
    CCP=CCP_IOREG_gc;
    PMIC.CTRL=n;
    // Set the default priority for round-robin scheduling
    PMIC.INTPRI=0x00;
    // Restore optimization for size if needed
    #pragma optsize_default

    // System clocks initialization
    system_clocks_init();

    // Ports initialization
    ports_init();

    // Virtual Ports initialization
    //vports_init();
    spif_init();


    // Globally enable interrupts
    asm("sei");

    tcc0_init();
    tcc1_init();
    tcd1_init();
    spie_init();
    SG17_SET;
    usartc0_init();
    SETSFRST;
    SETSFWP;
    _delay_ms(250);
    printf("Reading Config...\r\n");
    ReadConfig();

    while (1)
    {
	    
	    if(JogNeg) SETBIT(PORTR.OUT,0);
	    if(JogNeg==0) CLRBIT(PORTR.OUT,0);
	    
	    if(JogPos) SETBIT(PORTR.OUT,1);
	    if(JogPos==0) CLRBIT(PORTR.OUT,1);
	    
	    if(RefCmd==0)
	    {
		    //TODO : 
	    }
	    AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
	    if(ProgramRun && !AxisMoving)
	    {
		    if(IsPause==0)
		    {
			    if(RunSubProgram)
			    {
				    printf("Read Sub Program.\r\n");
				    
				    ReadSubProgram(0);
			    }
			    else
			    {
				    printf("Read Main Program.\r\n");
				    readRam();
			    }
			    printf("Program Block: %u\tMode: %u\r\n",CMDPRGlist.PRGLine,CMDPRGlist.Mode);
			    
				PORTR.OUTTGL=0x01; //TGLBIT(PORTR.OUT,0);
			    PRGExe();
			    if(PRGEXEindex < PRGSize) PRGEXEindex++;
		    }
	    }
	    /**********************************************/
	    if(cmdIsReady==0) continue;
	    for (j = 0 ; j < 21 ; j++)
	    {
		    cmdTemp[j] = EXECMD[j];
		    EXECMD[j] = 0;
	    }
	    cmdExe();
	    for (j=0;j<21;j++) cmdTemp[j] = 0;
	    cmdIsReady=0;
    }
}

