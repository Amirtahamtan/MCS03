
#define AxisNumber 3 //the maximum number of Axis

long int SetJog[AxisNumber];
long int AxisPosition[AxisNumber];
long int DistanceToGo[AxisNumber];
long int SetSpeed[AxisNumber];
long int AxACC[AxisNumber];
long int AxDEC[AxisNumber];
char RefDIR[AxisNumber];

char AxisIsMoving[AxisNumber];
unsigned int Pi[AxisNumber];
unsigned int Ma[AxisNumber];
char AxisDir[AxisNumber];
char AxisEMG[AxisNumber];
char AxisEnable[AxisNumber];

unsigned int SoftRefrence[AxisNumber];

char PLimitIsActiveA[AxisNumber]; //if positive limit is activated
char NLimitIsActiveA[AxisNumber];//if negative limit is activated


// it uses in when the axis is not going to reference 
// then the axis see the limits
// all the axis should stop there 
void ResetDistanceToGoEXP(char a) 
{
	for (int ax = 0 ; ax < AxisNumber ; ax++)
	{
		if	(a != ax)
		{
			DistanceToGo[ax] = 0;
		}
	}
}

char IsAnyAxisMoving(void)
{
	char res = 0;
	for (int ax = 0 ; ax < AxisNumber ; ax++)
	{
		res |= AxisIsMoving[ax];
	}
	return res;
}

void AxisPulse (char ax)
{
	switch (ax)
	{
		case 0: //Axis 1
		PULSE1;
		break;
		
		case 1: //Axis 2
		PULSE2;
		break;
		
		case 2: //Axis 3
		PULSE3;
		break;
	}
}

void SetAxisDir(int ax,char Direction)
{
	AxisDir[ax] = Direction;
	switch(Direction)
	{
		case 0: //Direction should be zero
		switch(ax)
		{
			case 0: //Axis 1
			CLR_DIR1;
			break;
			
			case 1: //Axis 2
			CLR_DIR2;
			break;
			
			case 2: //Axis 3
			CLR_DIR3;
			break;
		}
		break;
		case 1: //Direction should be set to one
		switch(ax)
		{
			case 0: //Axis 1
			SET_DIR1;
			break;
			
			case 1: //Axis 2
			SET_DIR2;
			break;
			
			case 2: //Axis 3
			SET_DIR3;
			break;
		}
		break;
	}
}

void SetAxisEMG(int ax,char emg)
{
	AxisEMG[ax] = emg;
	switch(emg)
	{
		case 0: //Direction should be zero
		switch(ax)
		{
			case 0: //Axis 1
			CLR_EMG1;
			break;
			
			case 1: //Axis 2
			CLR_EMG2;
			break;
			
			case 2: //Axis 3
			CLR_EMG3;
			break;
		}
		break;
		case 1: //Direction should be set to one
		switch(ax)
		{
			case 0: //Axis 1
			SET_EMG1;
			break;
			
			case 1: //Axis 2
			SET_EMG2;
			break;
			
			case 2: //Axis 3
			SET_EMG3;
			break;
		}
		break;
	}
}

void SetAxisEnable(int ax,char enb)
{
	AxisEnable[ax] = enb;
	switch(enb)
	{
		case 0: //Direction should be zero
		switch(ax)
		{
			case 0: //Axis 1
			CLR_ENABLE1;
			break;
			
			case 1: //Axis 2
			CLR_ENABLE2;
			break;
			
			case 2: //Axis 3
			CLR_ENABLE3;
			break;
		}
		break;
		case 1: //Direction should be set to one
		switch(ax)
		{
			case 0: //Axis 1
			SET_ENABLE1;
			break;
			
			case 1: //Axis 2
			SET_ENABLE2;
			break;
			
			case 2: //Axis 3
			SET_ENABLE3;
			break;
		}
		break;
	}
}

char AxisReady(char ax)
{
	char temp= 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = READY1;
		break;
		case 1:
		temp = READY2;
		break;
		case 2:
		temp = READY3;
		break;
	}
	return temp;
}

char AxisAlarm(char ax)
{
	char temp = 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = ALARM1;
		break;
		case 1:
		temp = ALARM2;
		break;
		case 2:
		temp = ALARM3;
		break;
	}
	return temp;
}

char AxisInPos(char ax)
{
	char temp = 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = INPOS1;
		break;
		case 1:
		temp = INPOS2;
		break;
		case 2:
		temp = INPOS3;
		break;
	}
	return temp;
}

char AxisLimitNeg(char ax)
{
	char temp = 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = LIM_NEG1;
		break;
		case 1:
		temp = LIM_NEG2;
		break;
		case 2:
		temp = LIM_NEG3;
		break;
	}
	return temp;
}

char AxisLimitPos(char ax)
{
	char temp = 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = LIM_POS1;
		break;
		case 1:
		temp = LIM_POS2;
		break;
		case 2:
		temp = LIM_POS3;
		break;
	}
	return temp;
}
char AxisRef(char ax)
{
	char temp = 0;
	switch (ax)
	{
		case 0: //Axis 1
		temp = REF1;
		break;
		case 1:
		temp = REF2;
		break;
		case 2:
		temp = REF3;
		break;
	}
	return temp;
}
