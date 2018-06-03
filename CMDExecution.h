#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timers.h"

char ProgramRun;

float TimerMainPeriod;

int j;
int k;
char cmdTemp[21];
char temp[21];
char EchoON;

long int AccDTG;
long int DecDTG;
long int MaxDistanceToGo;
long int MaxSpeed;
long int IACC;
long int IDEC;
long int SumAccDecDTG = 0;
long int CurSpeedFrq;
long int ACCSpeedINT,DECSpeedINT;
char ACC,DEC;

char PRGIsFinished; //IS program finished
int PRGEXEindex; //index of execution command
int PRGSize; //Size of program
char RefCmd; //if reference command is executed

int FreeJog = -1;
int SelectedAxis = 0;

char IsSync; //For Select Sync Mode or A sync mode: 0 is A sync mode , 1 is Sync mode.
//Sync Means send current situation to control a Sync means reply will not send to control

unsigned int LastPLine;
unsigned long int CurrentSerial;
char SerialCh[10];    // serial characters array
int SerChIndex;       // index of characters in serial string
char RunSubProgram;   //means the sub program is running
char IsPause=0;

// Timer/Counter TCC0 Compare/Capture A interrupt service routine
// timer to make pulses level 0

ISR (TCC0_CCA_vect)
{
	if (TCC0.INTFLAGS & TC0_CCAIF_bm) TCC0.INTFLAGS |= TC0_CCAIF_bm;
	
	if(MaxDistanceToGo>0) MaxDistanceToGo--;
	
	for (int ax = 0 ; ax < AxisNumber ; ax++)
	{
		if (DistanceToGo[ax] > 0)
		{
			Pi[ax] += Ma[ax];
			if (Pi[ax] >= 1000)
			{
				AxisPulse(ax); ///should be correct
				Pi[ax] -= 1000;
				DistanceToGo[ax]--;
				if (AxisDir[ax] == 0)
				AxisPosition[ax]--;
				else
				AxisPosition[ax]++;
			}
			AxisIsMoving[ax] = 1;
		}
		else
		{
			AxisIsMoving[ax] = 0;
		}
	}
}

// Timer/Counter TCC1 Overflow/Underflow interrupt service routine
// Timer to set speed according to acceleration and deceleration Level 1
ISR (TCC1_OVF_vect)
{
	float PER;
	j++;
	if(MaxDistanceToGo>0)
	{
		if(MaxDistanceToGo >= AccDTG)
		{
			CurSpeedFrq += ACCSpeedINT;
			PER=(float)(16000.0/CurSpeedFrq);
			TCC0.CCA=(unsigned int)(PER*1000);
		}
		if(MaxDistanceToGo <= DecDTG)
		{
			CurSpeedFrq -= ACCSpeedINT;
			PER=(float)(16000.0/CurSpeedFrq);
			TCC0.CCA=(unsigned int)(PER*1000);
		}
	}
	else
	{
		if(ACC==1)
		{
			CurSpeedFrq += ACCSpeedINT;
			if(CurSpeedFrq >= MaxSpeed)
			{
				CurSpeedFrq = MaxSpeed;
				ACC = 0;
			}
			PER=(float)(16000.0/CurSpeedFrq);
			
			TCC0.CCA=(unsigned int)(PER*1000);
		}
		if(DEC==1)
		{
			CurSpeedFrq -= DECSpeedINT;
			if(CurSpeedFrq <= DECSpeedINT)
			{
				CurSpeedFrq=DECSpeedINT;
				DEC=0;
				if (FreeJog != -1)
				{
					DistanceToGo[FreeJog] = 0;
					FreeJog = -1;
				}
			}
			PER=(float)(16000.0/CurSpeedFrq);
			
			TCC0.CCA=(unsigned int)(PER*1000);
		}
	}
	//printf("%u\r\n",TCC0.CCA);
}

// Timer/Counter TCD1 Overflow/Underflow interrupt service routine
// PLC INterpolation in this time level 4
ISR (TCD1_OVF_vect)
{
	for (int ax = 0 ; ax < AxisNumber ; ax++)
	{
		if(Axes[ax].HardwareLimitNegIsActive)
		{
			//printf("Hardware Limit Axis X is Active.(From PLC Timer routine)\r\n");
			//printf("LIM POS X: %u\r\n",LIM_NEG1);
			if(!AxisDir[ax] && !AxisLimitNeg(ax))
			{
				if(AxisIsMoving[ax])
				{
					DistanceToGo[ax] = 0;
					if(!RefCmd)
					{
						FreeJog = ax;
						MaxDistanceToGo = 0;
						ResetDistanceToGoEXP(ax);
					}
				}
			}
		}
		if(Axes[0].HardwareLimitPosIsActive)
		{
			//printf("Hardware Limit Axis X is Active.(From PLC Timer routine)\r\n");
			//printf("LIM POS X: %u\r\n",LIM_NEG1);
			if(AxisDir[ax] && !AxisLimitPos(ax))
			{
				//printf("Axis X PLimit is Active.\r\n");
				if(AxisIsMoving[ax])
				{
					DistanceToGo[ax] = 0;
					if(!RefCmd)
					{
						FreeJog = ax;
						MaxDistanceToGo = 0;
						ResetDistanceToGoEXP(ax);
					}
				}
			}
		}
	}
}


// run a single block the command will be in the CMDPRGList structure
// the structure will load in Main program
void PRGExe()
{
	switch (CMDPRGlist.Mode)
	{
		case 0 :  //Acceleration Mode
		{
			tc0_disable(&TCC0);
			for (int ax = 0 ; ax < CMDPRGlist.AxisNum ; ax++)
			{
				Ma[ax] = CMDPRGlist.SpeedMa[ax];
				SetAxisDir(ax,CMDPRGlist.Direction[ax]);
				DistanceToGo[ax] = CMDPRGlist.Move[ax];
			}
			
			MaxSpeed = CMDPRGlist.MaxSpeed;
			ACCSpeedINT = CMDPRGlist.ACCSpeedINT;
			DECSpeedINT = CMDPRGlist.DECSpeedINT;
			MaxDistanceToGo=CMDPRGlist.MaxDistanceToGo;
			AccDTG=CMDPRGlist.ACCDTG;
			DecDTG=CMDPRGlist.DECDTG;
			
			CurSpeedFrq = 0;
			tcc0_init();
			TCC0.CCA = 0XFFFF;
			//        ACC = 1;
			
			break;
		}
		case 1 : // With out ACC Mode
		{
			tc0_disable(&TCC0);
			for (int ax = 0 ; ax < CMDPRGlist.AxisNum ; ax++)
			{
				Ma[ax] = CMDPRGlist.SpeedMa[ax];
				SetAxisDir(ax,CMDPRGlist.Direction[ax]);
				DistanceToGo[ax] = CMDPRGlist.Move[ax];
			}
			
			MaxSpeed = CMDPRGlist.MaxSpeed;
			AccDTG = 0;
			DecDTG = 0;
			
			CurSpeedFrq = MaxSpeed;
			ACC = 0;
			DEC = 0;
			TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
			tcc0_init();
			TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
			break;
		}
		case 12: //Go to Refrence
		{
			while (IsAnyAxisMoving() == 1);
			
			for (int ax = 0 ; ax < AxisNumber ; ax++)
			{
				SetAxisDir(ax,Axes[ax].RefrenceDir);
				Ma[ax] = 1000;
				DistanceToGo[ax] = 0x7FFFFFFF;
			}
			
			MaxSpeed=8000;
			
			CurSpeedFrq = MaxSpeed;
			ACC = 0;
			DEC = 0;
			TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
			RefCmd=1;
			
			tcc0_init();
			
			TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
			
			sprintf(BufferSend,"!1200\r\n");
			Responsing = 1;
			SetDataReady; //there is some data for raspberry to receive
		}
		case 13: //set this position as reference
		{
			for (int ax = 0 ; ax < AxisNumber ; ax++)
			{
				SoftRefrence[ax] = AxisPosition[ax];
			}
			break;
		}
		case 14: //goto software refrence
		{
			//printf("Back to Software reference.\r\n");
			while (IsAnyAxisMoving() == 1);
			unsigned int temp = 0;
			int tempa = 0;
			for (int ax = 0 ; ax < AxisNumber ; ax++)
			{
				if(SoftRefrence[ax]<AxisPosition[ax]) SetAxisDir(ax,0);
				if(SoftRefrence[ax]>=AxisPosition[ax]) SetAxisDir(ax,1);
				DistanceToGo[ax] = 	labs(AxisPosition[ax]-SoftRefrence[ax]);
				if (DistanceToGo[ax] > temp)
				{
					temp = DistanceToGo[ax];
					tempa = ax;
				}
			}
			
			//printf("SoftRef1: %d\tSoftRef2: %d\tSoftRef3: %d\r\n",SoftRef1,SoftRef2,SoftRef3);
			//printf("DistanceToGo1: %d\tDistanceToGo2: %d\tDistanceToGo3: %d\r\n",DistanceToGo1,DistanceToGo2,DistanceToGo3);
			MaxSpeed = Axes[tempa].MaxSpeed;
			IACC = Axes[tempa].ACC;
			IDEC = Axes[tempa].DEC;
			MaxDistanceToGo = temp;
			//printf("MaxDTG: %d\r\n",MaxDistanceToGo);
			if(MaxSpeed == 0)
			{
				MaxSpeed = 500;
				ACC = 0;
				DEC = 0;
			}
			for (int ax = 0 ; ax < AxisNumber ; ax++)
			{
				Ma[ax] = (unsigned int)((float)((float)DistanceToGo[ax]/(float)MaxDistanceToGo)*1000);
			}
			//printf("MaxSpeed: %d\tMa1: %d\tMa2: %d\tMa3: %d\r\n",MaxSpeed,Ma1,Ma2,Ma3);
			
			CurSpeedFrq=MaxSpeed;

			TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
			tcc0_init();
			TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
			break;
		}
		case 15:
		{
			break;
		}
		case 81 : //pen Down
		{
			DO1_SET;
			
			break;
		}
		case 82 : //Pen UP
		{
			DO1_CLR;
			break;
		}
		case 90: //Serial number mode
		{
			char digit;
			int di=0;
			LastPLine = CMDPRGlist.PRGLine;
			
			CurrentSerial=ReadSerial();
			_delay_ms(100);
			if(((CurrentSerial <= CMDPRGlist.SerialTo) && (CMDPRGlist.SerialCountType == 0)) || ((CurrentSerial >= CMDPRGlist.SerialTo) && (CMDPRGlist.SerialCountType == 1)))
			{
				ltoa(CurrentSerial,SerialCh,10);
				if(CMDPRGlist.FillWithZero)
				{
					digit = CurrentSerial / 10;
					digit++;
					if(CMDPRGlist.DigitNumber>digit)
					{
						for(di=digit-1;di>=0;di--)
						{
							SerialCh[(di+(CMDPRGlist.DigitNumber-digit))]=SerialCh[di];
						}
						for(di=0;di<(CMDPRGlist.DigitNumber-digit);di++)
						{
							SerialCh[di] = '0';
						}
						SerialCh[CMDPRGlist.DigitNumber] = '0';
					}
				}
				if(CMDPRGlist.SerialCountType == 0)
				{
					CurrentSerial++;
				}
				else
				{
					CurrentSerial--;
				}
				
				SerChIndex = 0;
				ContinuousRead = 0;
				WriteSerial(CurrentSerial);
				_delay_ms(500);
				CurrentSerial = ReadSerial();
				_delay_ms(100);
				printf("Current Serial is %lu\r\n",CurrentSerial);
				ReadSubProgram(((SerialCh[SerChIndex]-48)*10)+100);
				printf("Program Block: %u\tMode: %u\r\n",CMDPRGlist.PRGLine,CMDPRGlist.Mode);
				printf("Move1:%lu\tMove2:%lu\tMove3:%lu\r\n",CMDPRGlist.Move[0],CMDPRGlist.Move[1],CMDPRGlist.Move[2]);
				PRGExe();
				RunSubProgram=1;
			}
			else
			{
				RunSubProgram=0;
				ContinuousRead=0;
				readRam();
				while(CMDPRGlist.PRGLine != LastPLine)
				{
					readRam();
				}
			}
			break;
		}
		case 100 :
		{
			printf("End Of Program.\r\n");
			ProgramRun = 0;
			ContinuousRead = 0;
			break;
		}
		case 101:
		{
			SerChIndex++;
			
			if(SerialCh[SerChIndex]!=0)
			{
				RunSubProgram = 1;
				ContinuousRead = 0;
				ReadSubProgram(((SerialCh[SerChIndex]-48)*10)+100);
				PRGExe();
			}
			else
			{
				RunSubProgram=0;
				ContinuousRead=0;
				readRam();
				while(CMDPRGlist.PRGLine != LastPLine)
				{
					readRam();
				}
			}
			break;
		}
	}
}

// run command that comes from raspberry the command will be in cmdtemp
void cmdExe()
{
	int ax = cmdTemp[4] - 1;
	if(strncmp(cmdTemp,"?0000",3)==0)
	{
		sprintf(BufferSend,"!0000\n\r");
		Responsing=1;
		SetDataReady;
	}
	if(strncmp(cmdTemp,"?01",3)==0)
	{
		if(cmdTemp[3]=='0') //Read Axis Inputs
		{
			switch(cmdTemp[5])
			{
				case '0': //ALARM
				{
					if(EchoON) printf("!010%d0\t%u\n\r",ax,AxisAlarm(ax));
					break;
				}
				case '1': //INPOS
				{
					if(EchoON) printf("!010%d1\t%u\n\r",ax,AxisInPos(ax));
					break;
				}
				case '2': //REF
				{
					if(EchoON) printf("!010%d2\t%u\n\r",ax,AxisRef(ax));
					break;
				}
				case '3': //LIMIT+
				{
					if(EchoON) printf("!010%d3\t%u\n\r",ax,AxisLimitPos(ax));
					break;
				}
				case '4': //LIMIT-
				{
					if(EchoON) printf("!010%d4\t%u\n\r",ax,AxisLimitNeg(ax));
					break;
				}
				case '5':
				{
					break;
				}
				case '6':
				{
					break;
				}
				case '7':
				{
					break;
				}
				case '8':
				{
					break;
				}
				case '9':
				{
					break;
				}
			}
		}
	}
	else if(strncmp(cmdTemp,"#02",3)==0)
	{
		if(cmdTemp[3]=='0')//Write Axis Outputs
		{
			switch(cmdTemp[5])
			{
				case '0': //Enable
				{
					SetAxisEnable(ax,cmdTemp[6]);
					break;
				}
				case '1': //EMG
				{
					SetAxisEMG(ax,cmdTemp[6]);
					break;
				}
				case '2':
				{
					break;
				}
				case '3':
				{
					break;
				}
				case '4':
				{
					break;
				}
				case '5':
				{
					break;
				}
				case '6':
				{
					break;
				}
				case '7':
				{
					break;
				}
				case '8':
				{
					break;
				}
				case '9':
				{
					break;
				}
			}
		}
	}
	else if(strncmp(cmdTemp,"?03",3)==0)
	{
		if(cmdTemp[3]=='0') //Read Axis Output
		{
			switch(cmdTemp[5])
			{
				case '0': //Enable
				{
					if(EchoON) printf("!030%d0\t%u\n\r",ax,AxisEnable[ax]);
					break;
				}
				case '1': //EMG
				{
					if(EchoON) printf("!030%d1\t%u\n\r",ax,AxisEMG[ax]);
					break;
				}
				case '2':
				{
					break;
				}
				case '3':
				{
					break;
				}
				case '4':
				{
					break;
				}
				case '5':
				{
					break;
				}
				case '6':
				{
					break;
				}
				case '7':
				{
					break;
				}
				case '8':
				{
					break;
				}
				case '9':
				{
					break;
				}
			}
		}
	}
	else if(strncmp(cmdTemp,"#04",3)==0) // Jog Each Axis
	{
		if (!ProgramRun)
		{
			if(cmdTemp[3]=='0')//SET JOG FOR AXIS
			{
				for(k = 0 ; k < 14 ; k++)
				{
					temp[k]=cmdTemp[k+5];
				}
				temp[20]=0;
				SetJog[ax]=atol(temp);
				sprintf(BufferSend,"!040%d\t%s\n\r",ax,temp);
				Responsing = 1;
				SetDataReady;
			}
		}
	}
	else if(strncmp(cmdTemp,"?05",3)==0)
	{
		if(cmdTemp[3]=='0')//READ JOG FOR AXIS
		{
			for(k=0;k<10;k++)
			{
				temp[k]=0;
			}
			temp[10]=0;
			ltoa(SetJog[ax],temp,10);
			if(EchoON) printf("!050%d\t%s\n\r",ax,temp);
		}
	}
	else if(strncmp(cmdTemp,"#06",3)==0)
	{
		if (!ProgramRun)
		{
			while (IsAnyAxisMoving() == 1);
			if(cmdTemp[3]=='0')//RUN JOG
			{
				if (SetJog[ax] < AxisPosition[ax])
				SetAxisDir(ax,0);
				else
				SetAxisDir(ax,1);
				MaxSpeed = SetSpeed[ax];
				IACC=AxACC[ax];
				IDEC=AxDEC[ax];
				MaxDistanceToGo=labs(AxisPosition[ax]-SetJog[ax]);
				Ma[ax]=1000;
				CurSpeedFrq=0;
				TCC0.CCA=0xFFFF;
				
				if (IACC == 0) IACC = 100;
				if (IDEC == 0) IDEC = 100;
								
				ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
				ACCSpeedINT = MaxSpeed / ACCSpeedINT;
				
				DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
				DECSpeedINT= MaxSpeed / DECSpeedINT;
				
				DecDTG = (long int) (MaxSpeed * MaxSpeed)/(long int) (IDEC);
				AccDTG = (long int) (MaxSpeed * MaxSpeed)/(long int) (IACC);
				SumAccDecDTG = DecDTG + AccDTG;
				if (MaxDistanceToGo <= SumAccDecDTG)
				{
					DecDTG = (long int) ((float) DecDTG * ((float) ((float)MaxDistanceToGo / (float)SumAccDecDTG)));
					AccDTG = MaxDistanceToGo - DecDTG;
				}
				else
				{
					AccDTG = MaxDistanceToGo - AccDTG;
				}
				
				DistanceToGo[ax] = labs(AxisPosition[ax]-SetJog[ax]);
				ACC=1;
				sprintf(BufferSend,"!060%d\r\n",ax);
				Responsing=1;
				SetDataReady;
			}
			
			else if(cmdTemp[3]=='1') //Axis Selected
			{
				SelectedAxis = ax;
				sprintf(BufferSend,"!061%d\r\n",ax);
				Responsing=1;
				SetDataReady;
			}
			else if(cmdTemp[3]=='2')//Free Jog  to Positive ?062X
			{
				SetAxisDir(ax,1);
				MaxSpeed=SetSpeed[ax];
				IACC=AxACC[ax];
				IDEC=AxDEC[ax];
				CurSpeedFrq=0;
				TCC0.CCA=0xFFFF;
				
				ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
				ACCSpeedINT = MaxSpeed / ACCSpeedINT;
				
				DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
				DECSpeedINT= MaxSpeed / DECSpeedINT;
				FreeJog = ax;
				ACC=1;
				sprintf(BufferSend,"!062%d\r\n",ax);
				Responsing=1;
				SetDataReady;
			}
			else if(cmdTemp[3]=='3')//Free Jog   STOP ?63X
			{
				DEC=1;
				sprintf(BufferSend,"!063%d\r\n",ax);
				Responsing=1;
				SetDataReady;
			}
			else if (cmdTemp[3] =='4') // free jog to negative
			{
				SetAxisDir(ax,0);
				MaxSpeed=SetSpeed[ax];
				IACC=AxACC[ax];
				IDEC=AxDEC[ax];
				CurSpeedFrq=0;
				TCC0.CCA=0xFFFF;
				
				ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a
				ACCSpeedINT = MaxSpeed / ACCSpeedINT;
				
				DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a
				DECSpeedINT= MaxSpeed / DECSpeedINT;
				FreeJog = ax;
				ACC=1;
				sprintf(BufferSend,"!064%d\r\n",ax);
				Responsing=1;
				SetDataReady;
			}
		}
	}
	else if(strncmp(cmdTemp,"#07",3)==0)
	{
		if(!ProgramRun)
		{
			if(cmdTemp[3]=='0')//STOP JOG
			{
				DistanceToGo[ax] = 0;
				if(EchoON) printf("!070%d\n\r",ax);
			}
		}
	}
	else if(strncmp(cmdTemp,"?08",3)==0)
	{
		if(cmdTemp[3]=='0')//Read DistanceToGo
		{
			ltoa(DistanceToGo[ax],temp,10);
			if(EchoON) printf("!080%d\t%s\n\r",ax,temp);
		}
	}
	else if(strncmp(cmdTemp,"?09",3)==0)
	{
		if(cmdTemp[3]=='0')//Read Axis Position
		{
			ltoa(AxisPosition[ax],temp,10);
			sprintf(BufferSend,"!090%d\t%s\r\n",ax,temp);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#10",3)==0)
	{
		if(!ProgramRun)//Clear JOG
		{
			if(cmdTemp[3]=='0')
			{
				DistanceToGo[ax] = 0;
				SetJog[ax] = AxisPosition[ax];
				if(EchoON) printf("!100%d\n\r",ax);
			}
		}
	}
	else if(strncmp(cmdTemp,"#11",3)==0)
	{
		if (!ProgramRun)
		{
			while (IsAnyAxisMoving() == 1);
			if(cmdTemp[3]=='0')//Set AXIS Position
			{
				for(k=0;k<10;k++)
				{
					temp[k]=cmdTemp[k+5];
				}
				temp[10]=0;
				AxisPosition[ax]=atol(temp);
				if(EchoON) printf("!110%d\n\r",ax);
			}
		}
	}
	else if(strncmp(cmdTemp,"#12",3)==0)
	{//Go To REF
		if (!ProgramRun)
		{
			while (IsAnyAxisMoving() == 1);
			if(cmdTemp[3]=='0')
			{
				// TODO: It should be implemented for going to reference
			}
		}
	}
	else if(strncmp(cmdTemp,"#13",3)==0)
	{// Set Speed
		if (!ProgramRun & !IsAnyAxisMoving())
		{
			if(cmdTemp[3]=='0')
			{
				for(k=0;k<10;k++)
				{
					temp[k]=cmdTemp[k+5];
				}
				temp[10]=0;
				SetSpeed[ax]=atof(temp);
				sprintf(BufferSend,"!130%d\t%s\r\n",ax,temp);
				Responsing=1;
				SetDataReady;
			}
		}
	}
	else if(strncmp(cmdTemp,"?14",3)==0)
	{//AxisIsMoving
		if(cmdTemp[3]=='0')
		{
			if(EchoON) printf("!140%d\t%u\n\r",ax,AxisIsMoving[ax]);
		}
	}
	else if(strncmp(cmdTemp,"?15",3)==0)
	{//Read Axis Speed
		if(cmdTemp[3]=='0')
		{
			if(EchoON) printf("!150%d\t%lu",ax,SetSpeed[ax]);
		}
	}
	else if(strncmp(cmdTemp,"#16",3)==0)
	{// Set ACC
		if (!ProgramRun & !IsAnyAxisMoving())
		{
			if(cmdTemp[3]=='0')
			{
				for(k=0;k<10;k++)
				{
					temp[k]=cmdTemp[k+5];
				}
				temp[10]=0;
				AxACC[ax]=atof(temp);
				sprintf(BufferSend,"!160%d\t%s\r\n",ax,temp);
				Responsing=1;
				SetDataReady;

			}
		}
	}
	else if(strncmp(cmdTemp,"#17",3)==0)
	{// Set DEC
		if (!ProgramRun & !IsAnyAxisMoving())
		{
			if(cmdTemp[3]=='0')
			{
				for(k=0;k<10;k++)
				{
					temp[k]=cmdTemp[k+5];
				}
				temp[10]=0;
				AxDEC[ax]=atof(temp);
				sprintf(BufferSend,"!170%d\t%s\r\n",ax,temp);
				Responsing=1;
				SetDataReady;
			}
		}
	}
	else if(strncmp(cmdTemp,"#1800",5)==0)
	{//Go To Refrence
		if (!ProgramRun)
		{
			while (IsAnyAxisMoving() == 1);
			printf("RefDir : %u\r\n",Axes[ax].RefrenceDir);
			for(int gx = 0 ; gx < AxisNumber ; gx++)
			{
				SetAxisDir(gx,Axes[gx].RefrenceDir);
				Ma[gx] = 1000;
				DistanceToGo[gx] = 0x7FFFFFFF;
				AxisPosition[gx] = 0;
			}
			MaxSpeed=8000;
			
			CurSpeedFrq = MaxSpeed;
			ACC = 0;
			DEC = 0;
			TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);
			RefCmd=1;
			
			tcc0_init();
			
			TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);
			
			sprintf(BufferSend,"!1800\r\n");
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#19",3)==0)
	{
		if(cmdTemp[3]=='0') //Reset Reference Direction
		{
			RefDIR[ax] = 0;
			sprintf(BufferSend,"!190%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#20",3)==0)
	{
		if(cmdTemp[3]=='0') //Set Reference Direction
		{
			RefDIR[ax] = 1;
			sprintf(BufferSend,"!200%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#21",3)==0)
	{
		if (cmdTemp[3] == 0)//Disable Hardware Positive Limit
		{
			PLimitIsActiveA[ax]=0;
			sprintf(BufferSend,"!210%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
		else if (cmdTemp[3] == 1)//Enable Hardware Positive Limit
		{
			PLimitIsActiveA[ax] = 1;
			sprintf(BufferSend,"!211%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#22",5)==0)
	{
		if (cmdTemp[3] == 0) //Disable Hardware Negative Limit
		{
			NLimitIsActiveA[ax] = 0;
			sprintf(BufferSend,"!220%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
		else if (cmdTemp[3] == 1)//Enable Hardware Negative Limit
		{
			NLimitIsActiveA[ax] = 1;
			sprintf(BufferSend,"!221%d\r\n",ax);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#2800",5)==0)
	{
		EchoON=1;
		if(EchoON) printf("!2800\n\r");
	}
	else if(strncmp(cmdTemp,"#2900",5)==0)
	{
		EchoON=0;
		if(EchoON) printf("!2900\n\r");
	}
	else if(strncmp(cmdTemp,"#3000",5)==0)
	{//Run 4Axis Interpolation
		if (!ProgramRun)
		{
			while (IsAnyAxisMoving() == 1);
			unsigned int TempS = 0;
			int tempa = 0;
			for (int gx = 0 ; gx < AxisNumber ; gx++)
			{
				if (SetJog[gx] < AxisPosition[gx])
				SetAxisDir(gx,0);
				else
				SetAxisDir(gx,1);
				if (TempS < SetSpeed[gx])
				{
					TempS = SetSpeed[gx];
					tempa = gx;
				}
				DistanceToGo[gx] = labs(AxisPosition[gx]-SetJog[gx]);
			}
			MaxSpeed = TempS;
			for (int gx = 0 ; gx < AxisNumber ; gx++)
			{
				Ma[gx]=(unsigned int)((float)(SetSpeed[gx]/MaxSpeed)*1000);
			}
			IACC=AxACC[tempa];
			IDEC=AxDEC[tempa];
			MaxDistanceToGo=labs(AxisPosition[tempa]-SetJog[tempa]);
			Ma[tempa]=1000;
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			
			CurSpeedFrq=0;
			TCC0.CCA=0xFFFF;
			if(IACC>0)
			{
				ACCSpeedINT = (long int) (100 * (float)((float)MaxSpeed / (float)IACC));// t = V/a
				ACCSpeedINT = (long int)(MaxSpeed / ACCSpeedINT);
			}
			if(IDEC>0)
			{
				DECSpeedINT = (long int)(100 * (float)((float)MaxSpeed / (float)IDEC));// t = V/a
				DECSpeedINT = (long int)(MaxSpeed / DECSpeedINT);
			}
			
			DecDTG = pow(MaxSpeed,2)/(IDEC);
			AccDTG = MaxDistanceToGo - pow(MaxSpeed,2)/(IACC);
			
			ACC=1;
			sprintf(BufferSend,"!30\tMaxS:%ld\tIACC:%ld\tIDEC:%ld\n\rMa1:%u\tMa2:%u\tBACC:%ld\tBDEC:%ld\tMAXDTG:%ld\n\r",MaxSpeed,IACC,IDEC,Ma[0],Ma[1],ACCSpeedINT,DECSpeedINT,MaxDistanceToGo);
			Responsing=1;
			SetDataReady;
		}
	}
	else if(strncmp(cmdTemp,"#4000",5)==0) //Stop Running Interpolation program , Stop All axis
	{
		ProgramRun=0;
		for (int gx = 0 ; gx < AxisNumber ; gx++)
		{
			DistanceToGo[gx] =0;
		}
		DO1_CLR; //Pen UP
		sprintf(BufferSend,"!4000\r\n");
		Responsing=1;
		//}
	}
	else if(strncmp(cmdTemp,"#4100",5)==0) //pause the axis in moving interplation
	{
		if(ProgramRun)
		{
			IsPause=1;
			for (int gx = 0 ; gx < AxisNumber ; gx++)
			{
				DistanceToGo[gx] =0;
				//TODO : it should be implemented
			}
			sprintf(BufferSend,"!4100\r\n");
			Responsing=1;
		}
	}
	else if(strncmp(cmdTemp,"#9800",5)==0)
	{
		if (!ProgramRun)//wait until all axis stopped
		{
			printf("Get Run Command\r\n");
			while (IsAnyAxisMoving() == 1);
	    	ContinuousRead = 0;
			PRGEXEindex = 0;
			ProgramRun = 1;
			printf("Program is running\r\n");
			sprintf(BufferSend,"!9800\r\n");
			Responsing=1;
			SetDataReady;
		}
		else if(IsPause==1)
		{
			PRGExe();
			IsPause=0;
		}
	}
	else if(strncmp(cmdTemp,"#9900",5)==0) //
	{
		ContinuousRead=0;
		readRam();
		readRam();
		readRam();
		sprintf(BufferSend,"!9800\r\nPL:%u\tMD:%u\tMv1:%ld\tMaxDTG:%ld\tMaxSpeed:%ld\tACCDTG:%ld\r\n",CMDPRGlist.PRGLine,CMDPRGlist.Mode,CMDPRGlist.Move[0],CMDPRGlist.MaxDistanceToGo,CMDPRGlist.MaxSpeed,CMDPRGlist.ACCDTG);
		Responsing=1;
		SetDataReady;
	}
	else
	{
		ClrDataReady;
	}
}

