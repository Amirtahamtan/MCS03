#define F_CPU 32000000UL

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#include "define.h"
#include "AxisCommands.h"
#include "ports.h"
#include "CommandGet.h"
#include "SFlash.h"
#include "CMDExecution.h"

char StopTime=0; //Counter to count stop down key time


void system_clocks_init(void) // System Clocks initialization
{
	unsigned char n,s;
	s = SREG;
	asm("cli");
	OSC.CTRL|=OSC_RC32KEN_bm;
	while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);
	OSC.CTRL|=OSC_RC32MEN_bm;
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;
	OSC.DFLLCTRL&= ~(OSC_RC32MCREF_bm | OSC_RC2MCREF_bm);
	DFLLRC32M.CTRL|=DFLL_ENABLE_bm;
	while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);
	n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=n;
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
	PORTCFG.CLKEVOUT&= ~PORTCFG_CLKOUT_gm;
	SREG = s;
}

// PORTH interrupt 0 service routine
// JOG Interrupt routine for positive direction
ISR (PORTH_INT0_vect)
{
	if(JogPosInput)
	{
		printf("Jog+\r\n");
		if(!JogNegInput)
		{
			MaxSpeed=10000;
			IACC=5000;
			IDEC=5000;
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			
			//ACCSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			//ACCSpeedINT = MaxSpeed / ACCSpeedINT;
			//
			//DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			//DECSpeedINT= MaxSpeed / DECSpeedINT;

			if(!LIM_POS1 && Axes[SelectedAxis].HardwareLimitPosIsActive)
				MaxSpeed=Axes[SelectedAxis].MaxSpeed;
				
			IACC=Axes[SelectedAxis].ACC;
			IDEC=Axes[SelectedAxis].DEC;
					
			ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
			DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			DECSpeedINT= MaxSpeed / DECSpeedINT;
					
			SET_DIR1;
			FreeJog=SelectedAxis;
			Ma[SelectedAxis] = 1000;
			DistanceToGo[SelectedAxis] = 0x7FFFFFFF;
			
			ACC=1;
		}
	}
	else if((FreeJog == 0 && DIR1)||(FreeJog == 1 && DIR2)||(FreeJog == 2 && DIR3))
	{
		ACC=0;
		DEC=1;
	}
}

// PORTH interrupt 0 service routine
// Interrupt for Negative jog key
ISR (PORTD_INT0_vect)
{
	if(JogNegInput)
	{
		if(!JogPosInput)
		{
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			
			//ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			//ACCSpeedINT = MaxSpeed / ACCSpeedINT;
			//
			//DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			//DECSpeedINT= MaxSpeed / DECSpeedINT;

			if(!LIM_NEG1 && Axes[SelectedAxis].HardwareLimitNegIsActive)
				MaxSpeed=Axes[SelectedAxis].MaxSpeed;
				
			IACC=Axes[SelectedAxis].ACC;
			IDEC=Axes[SelectedAxis].DEC;
					
			ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
			ACCSpeedINT = MaxSpeed / ACCSpeedINT;
					
			DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
			DECSpeedINT= MaxSpeed / DECSpeedINT;
					
			CLR_DIR1;
			FreeJog=SelectedAxis;
			Ma[SelectedAxis] = 1000;
			DistanceToGo [SelectedAxis] = 0x7FFFFFFF;

			ACC=1;
		}
	}
	else if((FreeJog==0 && !DIR1)||(FreeJog==1 && !DIR2)||(FreeJog==2 && !DIR3))
	{
		ACC=0;
		DEC=1;
	}
}

//Port D interrupt
//Start Stop Key Interrupt
ISR (PORTD_INT1_vect)
{
	//printf("PortD INT1\r\n");
	if(RunProgramInput) //If Run Program Input pin is pressed
	{
		
		//     because start is done in Raspberry then the part of code is disable here
		
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
	if(StopProgramInput) //Stop Program Input pin
	{
		if(ProgramRun) //if the program is in running mode
		{
			IsPause=1;
			for (int ax = 0 ; ax < AxisNumber ; ax++)
			{
				DistanceToGo[ax] =0;
			}
			sprintf(BufferSend,"!4100\r\n");
			Responsing=1;
		}
		
		StopTime=0;
		while(StopProgramInput)
		{
			_delay_ms(100);
			StopTime++;
			if(StopTime>30)
			{
				DO1_CLR; //Pen UP

				while (IsAnyAxisMoving() == 1);
				long int Tempd = 0;
				int Tempa = 0;
				for (int ax = 0 ; ax < AxisNumber ; ax++)
				{
					if (SoftRefrence[ax] < AxisPosition[ax]) SetAxisDir(ax,0);
					if (SoftRefrence[ax] >= AxisPosition[ax]) SetAxisDir(ax,1);
					DistanceToGo[ax] = labs(AxisPosition[ax]-SoftRefrence[ax]);
					if (Tempd < DistanceToGo[ax])
					{
						Tempd = DistanceToGo[ax];
						Tempa = ax;
					}
				}
				MaxSpeed = Axes[Tempa].MaxSpeed;
				IACC=Axes[Tempa].ACC;
				IDEC=Axes[Tempa].DEC;
				MaxDistanceToGo=labs(AxisPosition[Tempa]-SoftRefrence[Tempa]);
				//printf("MaxDTG: %d\r\n",MaxDistanceToGo);
				for (int ax = 0 ; ax < AxisNumber ; ax++)
				{
					Ma[ax] = (unsigned int)((float)((float)DistanceToGo[ax]/(float)MaxDistanceToGo)*1000);
				}
				CurSpeedFrq=MaxSpeed;
				ACC = 0;
				DEC = 0;
				TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
				tcc0_init();
				TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
				ProgramRun=0;
			}
		}
	}
}


// USARTC0 initialization
// USART PORT C  for testing and trace in debugging  putchar and getchar is working with this port

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
// interrupt definition for receive of data in USARTC0 port C

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
// Debug port receive interrupt
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

int main(void)
{
	// Declare your local variables here
	unsigned char n;

	// Interrupt system initialization
	// Optimize for speed
	//#pragma optsize-
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
	//#pragma optsize_default

	// System clocks initialization
	system_clocks_init();

	// Ports initialization
	ports_init();

	// Virtual Ports initialization
	//vports_init();
	spif_init();


	// Globally enable interrupts
	asm("sei");

	tcc0_init();  // Axis interpolation counter for level 0
	tcc1_init();  // Axis interpolation counter for level 1
	tcd1_init();  // PLC interpolation timer
	spie_init();  // SPI Flash initialization
	SG17_SET;     // says to 7 segment board and says that Xmega is loaded
	usartc0_init();
	SETSFRST; //set Serial Flash Reset means Xmega is able to read from SPI Flash
	SETSFWP;  //set Serial Flash Write protect to allow raspberry to write on it
	_delay_ms(250);
	printf("Reading Config...\r\n");
	ReadConfig();  // read configuration from Serail Flash

	while (1)
	{
		// if jog is pushed the LED on the board turns on
		if(JogNegInput == 1) SETBIT(PORTR.OUT,0);
		if(JogNegInput == 0) CLRBIT(PORTR.OUT,0);
		if(JogPosInput == 1) SETBIT(PORTR.OUT,1);
		if(JogPosInput == 0) CLRBIT(PORTR.OUT,1);
		
		if(ProgramRun && !IsAnyAxisMoving())
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
				
				PORTR.OUTTGL=0x01; //program running will flash the LED
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

