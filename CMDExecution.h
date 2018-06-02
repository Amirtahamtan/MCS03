#include <stdlib.h> 
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timers.h"

char ProgramRun;

long int SetJog1;
long int SetJog2;
long int SetJog3;

long int AxisPosition1;
long int AxisPosition2;
long int AxisPosition3;

long int SoftRef1;
long int SoftRef2;
long int SoftRef3;

long int MaxDistanceToGo;

long int DistanceToGo1;
long int DistanceToGo2;
long int DistanceToGo3;

long int AccDTG;
long int DecDTG;

char RefDir1;
char RefDir2;
char RefDir3;

long int SetSpeed1;
long int SetSpeed2;
long int SetSpeed3;

long int MaxSpeed;

long int ACC1;
long int ACC2;
long int ACC3;

long int DEC1;
long int DEC2;
long int DEC3;


long int IACC;
long int IDEC;

long int SumAccDecDTG = 0;
char AxisIsMoving1;
char AxisIsMoving2;
char AxisIsMoving3;


long int CurSpeedFrq;

long int ACCSpeedINT,DECSpeedINT;

char ACC,DEC;

char IsSync;//For Select Sync Mode or Async mode: 0 is Async mode , 1 is Sync mode.
unsigned int Pi1,Pi2,Pi3;
unsigned int Ma1,Ma2,Ma3;

float TimerMainPeriod;

int j;
int k;
char cmdTemp[21];
char temp[21];
char EchoON;

char AxisMoving; //if any axis is moving
char PRGIsFinished; //IS program finished 
int PRGEXEindex; //index of execution command
int PRGSize; //Size of program 
char RefCmd; //if reference command is executed

char PLimitIsActiveA1=0;
char NLimitIsActiveA1=0;
char PLimitIsActiveA2=0;
char NLimitIsActiveA2=0;
char PLimitIsActiveA3=0;
char NLimitIsActiveA3=0;
char FreeJog=0;
char SelectedAxis=1;

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
    if (TCC0.INTFLAGS & TC0_CCAIF_bm) TCC0.INTFLAGS|=TC0_CCAIF_bm;   
     
    if(MaxDistanceToGo>0) MaxDistanceToGo--;
     
    if(DistanceToGo1>0)
    {              
      Pi1+=Ma1; 
      if(Pi1>=1000)
      {                   
        PULSE1;
        Pi1-=1000;
        DistanceToGo1--;   
        if(DIR1==0) AxisPosition1--;  
        else AxisPosition1++; 
      } 
      AxisIsMoving1=1;
    } 
    else
    {
      AxisIsMoving1=0;
    }   
                            
    if(DistanceToGo2>0)
    {
      Pi2+=Ma2; 
      if(Pi2>=1000)
      {                   
        PULSE2;
        Pi2-=1000;
        DistanceToGo2--;   
        if(DIR2==0) AxisPosition2--;  
        else AxisPosition2++; 
      }
     AxisIsMoving2=1;
    } 
    else
    {
      AxisIsMoving2=0;
    }
        
    if(DistanceToGo3>0)
    {
      Pi3+=Ma3; 
      if(Pi3>=1000)
      {                   
        PULSE3;
        Pi3-=1000;
        DistanceToGo3--;   
        if(DIR3==0) AxisPosition3--;  
        else AxisPosition3++; 
      }
     AxisIsMoving3=1;
    } 
    else
    {
      AxisIsMoving3=0;
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
    CurSpeedFrq+= ACCSpeedINT; 
    PER=(float)(16000.0/CurSpeedFrq);        
    TCC0.CCA=(unsigned int)(PER*1000);
    }
    if(MaxDistanceToGo <= DecDTG)
    {
    CurSpeedFrq-= ACCSpeedINT; 
    PER=(float)(16000.0/CurSpeedFrq);        
    TCC0.CCA=(unsigned int)(PER*1000);
    }  
  }
  else
  {          
      if(ACC==1)
      {
         CurSpeedFrq+= ACCSpeedINT;
         if(CurSpeedFrq>=MaxSpeed)
         {
           CurSpeedFrq=MaxSpeed;   
           ACC=0;
         } 
         PER=(float)(16000.0/CurSpeedFrq);   
            
        TCC0.CCA=(unsigned int)(PER*1000); 
      } 
      if(DEC==1)
      { 
         CurSpeedFrq-= DECSpeedINT;
         if(CurSpeedFrq<=DECSpeedINT)
         {
           CurSpeedFrq=DECSpeedINT;   
           DEC=0;  
           if(FreeJog==1)
           {
            FreeJog=0;  
            DistanceToGo1=0;
           }
           if(FreeJog==2)
           {
            FreeJog=0;  
            DistanceToGo2=0;
           }
           if(FreeJog==3)
           {
            FreeJog=0;  
            DistanceToGo3=0;
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
   if(Axes[0].HardwareLimitNegIsActive)
   {  
       //printf("Hardware Limit Axis X is Active.(From PLC Timer routine)\r\n");  
       //printf("LIM POS X: %u\r\n",LIM_NEG1);            
       if(!DIR1 && !LIM_NEG1)       
       {
         if(AxisIsMoving1) 
         {
          DistanceToGo1=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo2=0;
            DistanceToGo3=0;  
          } 
         }
       }  
   }
     
   if(Axes[0].HardwareLimitPosIsActive)
   { 
       //printf("Hardware Limit Axis X is Active.(From PLC Timer routin)\r\n");    
       //printf("LIM POS X: %u\r\n",LIM_NEG1);
       if(DIR1 && !LIM_POS1)       
       {
         //printf("Axis X PLimit is Active.\r\n");
         if(AxisIsMoving1) 
         {
          DistanceToGo1=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo2=0;
            DistanceToGo3=0;  
          } 
         }
       } 
   }
  //------------------------------------------- 
   if(Axes[1].HardwareLimitNegIsActive)  
   {
       if(!DIR2 && !LIM_NEG2)       
       { 
         if(AxisIsMoving2)
         {
          DistanceToGo2=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo1=0;
            DistanceToGo3=0;  
          }
         }
       }    
   }
   
   if(Axes[1].HardwareLimitPosIsActive)
   {
       if(DIR2 && !LIM_POS2)       
       { 
         if(AxisIsMoving2)
         {
          DistanceToGo2=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo1=0;
            DistanceToGo3=0;  
          }
         }
       } 
   }  
   //----------------------------------------------- 
   if(Axes[2].HardwareLimitNegIsActive)
   {
       if(!DIR3 && !LIM_NEG3)       
       {
         if(AxisIsMoving3)
         {
          DistanceToGo3=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo2=0;
            DistanceToGo1=0;  
          }
         }
       }
   }
   
   if(Axes[2].HardwareLimitPosIsActive)
   {
       if(DIR3 && !LIM_POS3)       
       {
         if(AxisIsMoving3)
         {
          DistanceToGo3=0;      
          if(!RefCmd)
          {
            FreeJog=0;
            MaxDistanceToGo=0;        
            DistanceToGo2=0;
            DistanceToGo1=0;  
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
            
        Ma1 = CMDPRGlist.Speed1Ma1;                
        Ma2 = CMDPRGlist.Speed2Ma2;   
        Ma3 = CMDPRGlist.Speed3Ma3;
                    
                    
        MaxSpeed = CMDPRGlist.MaxSpeed;
        ACCSpeedINT = CMDPRGlist.ACCSpeedINT;
        DECSpeedINT = CMDPRGlist.DECSpeedINT;
                    
          if(CMDPRGlist.Direction1==1)
          { 
            SET_DIR1;
          }
          else
          {
            CLR_DIR1;
          } 
          
          if(CMDPRGlist.Direction2==1)
          { 
            SET_DIR2;
          }
          else
          {
            CLR_DIR2;
          } 
          
          if(CMDPRGlist.Direction3==1)
          { 
            SET_DIR3;
          }
          else
          {
            CLR_DIR3;
          }
                            
//         if(CMDPRGlist.Speed1Ma1==1000) MaxDistanceToGo=CMDPRGlist.Move1;
//         if(CMDPRGlist.Speed2Ma2==1000) MaxDistanceToGo=CMDPRGlist.Move2;
//         if(CMDPRGlist.Speed3Ma3==1000) MaxDistanceToGo=CMDPRGlist.Move3;
         MaxDistanceToGo=CMDPRGlist.MaxDistanceToGo;
         
//         IACC=(long int)((float)(MaxSpeed) / (float)((float) ACCSpeedINT/100.0)); 
//         IDEC=(long int)((float)(MaxSpeed) / (float)((float) DECSpeedINT/100.0));         
//         DecDTG = (long int) (MaxSpeed * MaxSpeed)/(long int) (IDEC);  
//         AccDTG = (long int) (MaxSpeed * MaxSpeed)/(long int) (IACC);  
//         SumAccDecDTG = DecDTG + AccDTG;
//         if (MaxDistanceToGo <= SumAccDecDTG)   
//         {        
//            DecDTG = (long int) ((float) DecDTG * ((float) ((float)MaxDistanceToGo / (float)SumAccDecDTG)));
//            AccDTG = MaxDistanceToGo - DecDTG;
//         }
//         else
//         {
//              AccDTG = MaxDistanceToGo - AccDTG;
//         }
            
        AccDTG=CMDPRGlist.ACCDTG;
        DecDTG=CMDPRGlist.DECDTG;
                    
        DistanceToGo1 = CMDPRGlist.Move1;
        DistanceToGo2 = CMDPRGlist.Move2;
        DistanceToGo3 = CMDPRGlist.Move3;
                    
        CurSpeedFrq = 0;                
        tcc0_init(); 
        TCC0.CCA = 0XFFFF;
//        ACC = 1;      
        
        break;                
    }
    case 1 : // With out ACC Mode
    {        
        tc0_disable(&TCC0);
        Ma1 = CMDPRGlist.Speed1Ma1;
        Ma2 = CMDPRGlist.Speed2Ma2;
        Ma3 = CMDPRGlist.Speed3Ma3;
          
        MaxSpeed = CMDPRGlist.MaxSpeed;
                    
        if(CMDPRGlist.Direction1==1)
        { 
         SET_DIR1;
        }
        else
        {
         CLR_DIR1;
        } 
              
        if(CMDPRGlist.Direction2==1)
        { 
         SET_DIR2;
        }
        else
        {
          CLR_DIR2;
        }  
              
        if(CMDPRGlist.Direction3==1)
        { 
         SET_DIR3;
        }
        else
        {
         CLR_DIR3;
        }                     
    				
        AccDTG = 0;
        DecDTG = 0;
                    
        DistanceToGo1 = CMDPRGlist.Move1;
        DistanceToGo2 = CMDPRGlist.Move2;
        DistanceToGo3 = CMDPRGlist.Move3;
                    
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
        AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;           
        while (AxisMoving == 1) 
        {
         AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
        }   
      
        if(RefDir1==0)
        {
          CLR_DIR1;
        }
        else if(RefDir1==1)
        {
          SET_DIR1;
        } 
      
        if(RefDir2==0)
        {
          CLR_DIR2;
        }
        else if(RefDir2==1)
        {
          SET_DIR2;
        }
      
        if(RefDir3==0)
        {
          CLR_DIR3;
        }
        else if(RefDir3==1)
        {
          SET_DIR3;
        }       
                             
        MaxSpeed=8000;                           
        
        CurSpeedFrq = MaxSpeed;
        ACC = 0;
        DEC = 0;
        TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);          
        RefCmd=1;
        Ma1=1000; 
        Ma2=1000;
        Ma3=1000;
        DistanceToGo1=0x7FFFFFFF;
        DistanceToGo2=0x7FFFFFFF;
        DistanceToGo3=0x7FFFFFFF;   
         
        tcc0_init();     
      
        TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);                    
                
        sprintf(BufferSend,"!1800\r\n"); 
        Responsing=1; 
        SetDataReady;  
    }
    case 13:
    {
      SoftRef1=AxisPosition1; 
      SoftRef2=AxisPosition2;
      SoftRef3=AxisPosition3;
      break;
    }       
    case 14:
    {
      //printf("Back to Software refrence.\r\n");
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
      break;
    }       
    case 15:
    {
      break;
    }
    case 81 :
    {
        DO1_SET;
        
        break;
    } 
    case 82 :
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
        
        SerChIndex=0; 
        ContinuousRead=0;       
        WriteSerial(CurrentSerial); 
        _delay_ms(500);
        CurrentSerial=ReadSerial(); 
        _delay_ms(100);                     
        printf("Current Serial is %lu\r\n",CurrentSerial);
        ReadSubProgram(((SerialCh[SerChIndex]-48)*10)+100); 
        printf("Program Block: %u\tMode: %u\r\n",CMDPRGlist.PRGLine,CMDPRGlist.Mode); 
        printf("Move1:%lu\tMove2:%lu\tMove3:%lu\r\n",CMDPRGlist.Move1,CMDPRGlist.Move2,CMDPRGlist.Move3);
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
        RunSubProgram=1;
        ContinuousRead=0;
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
    if(strncmp(cmdTemp,"?0000",3)==0)         
    {
      sprintf(BufferSend,"!0000\n\r");       
      Responsing=1; 
      SetDataReady;
    }  
    if(strncmp(cmdTemp,"?01",3)==0)
    {//Read Axis Inputs
      if(cmdTemp[3]=='0')
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
           switch(cmdTemp[5])
           {
             case '0': //ALARM
             {
               if(EchoON) printf("!01010\t%u\n\r",ALARM1);
               break;
             }
             case '1': //INPOS
             {
               if(EchoON) printf("!01011\t%u\n\r",INPOS1);
               break;
             }
             case '2': //REF
             {
               if(EchoON) printf("!01012\t%u\n\r",REF1);
               break;
             }
             case '3': //LIMIT+
             {
               if(EchoON) printf("!01013\t%u\n\r",LIM_POS1);
               break;
             }
             case '4': //LIMIT-
             {
               if(EchoON) printf("!01014\t%u\n\r",LIM_NEG1);
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
           break;
         }
         case '2': //Axis 2
         {
           switch(cmdTemp[5])
           {
             case '0': //ALARM
             {
               if(EchoON) printf("!01020\t%u\n\r",ALARM2);
               break;
             }
             case '1': //INPOS
             {
               if(EchoON) printf("!01021\t%u\n\r",INPOS2);
               break;
             }
             case '2': //REF
             {
               if(EchoON) printf("!01032\t%u\n\r",REF2);
               break;
             }
             case '3': //LIMIT+
             {
               if(EchoON) printf("!01023\t%u\n\r",LIM_POS2);
               break;
             }
             case '4': //LIMIT-
             {
               if(EchoON) printf("!01024\t%u\n\r",LIM_NEG2);
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
           break;
         }
         case '3': //Axis 3
         {
           switch(cmdTemp[5])
           {
             case '0': //ALARM
             {
               if(EchoON) printf("!01030\t%u\n\r",ALARM3);
               break;
             }
             case '1': //INPOS
             {
               if(EchoON) printf("!01031\t%u\n\r",INPOS3);
               break;
             }
             case '2': //REF
             {
               if(EchoON) printf("!01032\t%u\n\r",REF3);
               break;
             }
             case '3': //LIMIT+
             {
               if(EchoON) printf("!01033\t%u\n\r",LIM_POS3);
               break;
             }
             case '4': //LIMIT-
             {
               if(EchoON) printf("!01034\t%u\n\r",LIM_NEG3);
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
           break;
         }
       }
      }
    }      
    else if(strncmp(cmdTemp,"#02",3)==0)
    {//Write Axis Outputs
      if(cmdTemp[3]=='0')
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {
               if(cmdTemp[6]=='0')
               {
                 CLR_ENABLE1;
                 if(EchoON) printf("!020100\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 SET_ENABLE1;  
                 if(EchoON) printf("!020101\n\r");
               }                       
               break;
             }
             case '1': //EMG
             {
               if(cmdTemp[6]=='0')
               {
                 CLR_EMG1;
                 if(EchoON) printf("!020110\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 SET_EMG1;  
                 if(EchoON) printf("!020111\n\r");
               }  
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
           break;
         }
         case '2': //Axis 2
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {
               if(cmdTemp[6]=='0')
               {
                 CLR_ENABLE2;
                 if(EchoON) printf("!020200\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 SET_ENABLE2;  
                 if(EchoON) printf("!020201\n\r");
               }                       
               break;
             }
             case '1': //EMG
             {
               if(cmdTemp[6]=='0')
               {
                 CLR_EMG2;
                 if(EchoON) printf("!020210\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 SET_EMG2;  
                 if(EchoON) printf("!020211\n\r");
               }  
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
           break;
         }
         case '3': //Axis 3
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {
               if(cmdTemp[6]=='0')
               {
                 CLR_ENABLE3;
                 if(EchoON) printf("!020300\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 SET_ENABLE3;  
                 if(EchoON) printf("!020301\n\r");
               }                       
               break;
             }
             case '1': //EMG
             {
               if(cmdTemp[6]=='0')
               {
                 //CLR_EMG3;
                 if(EchoON) printf("!020310\n\r");  
               }
               if(cmdTemp[6]=='1')
               {
                 //SET_EMG3;  
                 if(EchoON) printf("!020311\n\r");
               }  
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
           break;
         }         
       }
                                  
      }
    }
    else if(strncmp(cmdTemp,"?03",3)==0)
    {//Read Axis Output
      if(cmdTemp[3]=='0')
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {                       
               if(EchoON) printf("!03010\t%u\n\r",ENABLE1);                                      
               break;
             }
             case '1': //EMG
             {
               if(EchoON) printf("!03011\t%u\n\r",EMG1);                                      
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
           break;
         }
         case '2': //Axis 2
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {                       
               if(EchoON) printf("!03020\t%u\n\r",ENABLE2);                                      
               break;
             }
             case '1': //EMG
             {
               if(EchoON) printf("!03021\t%u\n\r",EMG2);                                      
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
           break;
         }
         case '3': //Axis 3
         {
           switch(cmdTemp[5])
           {
             case '0': //Enable
             {                       
               if(EchoON) printf("!03030\t%u\n\r",ENABLE3);                                      
               break;
             }
             case '1': //EMG
             {
               //if(EchoON) printf("!03031\t%u\n\r",EMG3);                                      
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
           break;
         }         
       }
                                  
      }
    }
    else if(strncmp(cmdTemp,"#04",3)==0)
    {//SET JOG FOR AXIS  
      if (!ProgramRun)
      {
        if(cmdTemp[3]=='0')
         {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {
                for(k=0;k<14;k++)
                {
                  temp[k]=cmdTemp[k+5];
                }
                temp[20]=0;
                SetJog1=atol(temp);
                sprintf(BufferSend,"!0401\t%s\n\r",temp); 
                Responsing=1;
                SetDataReady;   
                break;
             }
             case '2': //Axis 2
             {
                for(k=0;k<10;k++)
                {
                  temp[k]=cmdTemp[k+5];
                }
                temp[10]=0;
                SetJog2=atol(temp);
                sprintf(BufferSend,"!0402\t%s\n\r",temp); 
                Responsing=1; 
                SetDataReady; 
                break;
             }                          
             case '3': //Axis 3
             {
                for(k=0;k<10;k++)
                {
                  temp[k]=cmdTemp[k+5];
                }
                temp[10]=0;
                SetJog3=atol(temp);
                sprintf(BufferSend,"!0403\t%s\n\r",temp); 
                Responsing=1; 
                SetDataReady; 
                break;
             }              
           }
         }
      }          
    }
    else if(strncmp(cmdTemp,"?05",3)==0)
    {//READ JOG FOR AXIS  
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {
                for(k=0;k<10;k++)
                {
                  temp[k]=0;
                }
                temp[10]=0;
                ltoa(SetJog1,temp,10);
                if(EchoON) printf("!0501\t%s\n\r",temp);  
                break;
             }
             case '2': //Axis 2
             {
                for(k=0;k<10;k++)
                {
                  temp[k]=0;
                }
                temp[10]=0;
                ltoa(SetJog2,temp,10);
                if(EchoON) printf("!0502\t%s\n\r",temp);  
                break;
             }                          
             case '3': //Axis 3
             {
                for(k=0;k<10;k++)
                {
                  temp[k]=0;
                }
                temp[10]=0;
                ltoa(SetJog3,temp,10);
                if(EchoON) printf("!0503\t%s\n\r",temp);  
                break;
             }              
           }
      }
    }
    else if(strncmp(cmdTemp,"#06",3)==0)
    {//RUN JOG  
    AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
         
     if (!ProgramRun)
     {
      while (AxisMoving == 1) 
      {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
      }
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {
                if(SetJog1<AxisPosition1) CLR_DIR1;
                if(SetJog1>AxisPosition1) SET_DIR1;
                        
                MaxSpeed=SetSpeed1; 
                IACC=ACC1; 
                IDEC=DEC1; 
                MaxDistanceToGo=labs(AxisPosition1-SetJog1);               
                Ma1=1000;          
                CurSpeedFrq=0;
                TCC0.CCA=0xFFFF;                                                   
                                 
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
                            
                DistanceToGo1=labs(AxisPosition1-SetJog1);  
                ACC=1;    
                sprintf(BufferSend,"!0601\r\n"); 
                Responsing=1; 
                SetDataReady; 
                break;
             }
             case '2': //Axis 2
             {
                if(SetJog2<AxisPosition2) CLR_DIR2;
                if(SetJog2>AxisPosition2) SET_DIR2;
                        
                MaxSpeed=SetSpeed2;
                IACC=ACC2;
                IDEC=DEC2;
                MaxDistanceToGo=labs(AxisPosition2-SetJog2); 
                              
                Ma2=1000;           
                CurSpeedFrq=0;
                TCC0.CCA=0xFFFF;                                                   
                                 
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
                            
                DistanceToGo2=labs(AxisPosition2-SetJog2);  
                ACC=1;    
                sprintf(BufferSend,"!0602\r\n");
                Responsing=1;  
                SetDataReady;
                break;
             } 
            case '3': //Axis 3
            {
            if(SetJog3<AxisPosition3) CLR_DIR3;
            if(SetJog3>AxisPosition3) SET_DIR3;
                        
            MaxSpeed=SetSpeed3;
            IACC=ACC3;
            IDEC=DEC3;
            MaxDistanceToGo = labs(AxisPosition3-SetJog3);  
                         
            Ma3=1000;           
            CurSpeedFrq=0;
            TCC0.CCA=0xFFFF;                                                   
                                 
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
                            
                DistanceToGo3=labs(AxisPosition3-SetJog3);  
                ACC=1;    
                sprintf(BufferSend,"!0603\r\n");
            Responsing=1;  
            SetDataReady;
            break;
            }                
           }
      }
      else if(cmdTemp[3]=='1')//Axis Selected
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
            SelectedAxis=1;
            sprintf(BufferSend,"!0611\r\n"); 
            Responsing=1; 
            SetDataReady;
            break;
         }
         case '2': //Axis 2
         {
            SelectedAxis=2;
            sprintf(BufferSend,"!0612\r\n"); 
            Responsing=1; 
            SetDataReady;
            break;
         } 
        case '3': //Axis 3
        {        
            SelectedAxis=3;
            sprintf(BufferSend,"!0613\r\n"); 
            Responsing=1; 
            SetDataReady;
            break;
        }     
      }
     }
      else if(cmdTemp[3]=='2')//Free Jog   to Positive
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
            SET_DIR1;                        
            MaxSpeed=SetSpeed1; 
            IACC=ACC1; 
            IDEC=DEC1;         
            CurSpeedFrq=0;
            TCC0.CCA=0xFFFF;                                                   
                                 
            ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a                        
            ACCSpeedINT = MaxSpeed / ACCSpeedINT; 
                        
            DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a                           
            DECSpeedINT= MaxSpeed / DECSpeedINT;                  
            FreeJog=1; 
            ACC=1;    
            sprintf(BufferSend,"!0601\r\n"); 
            Responsing=1; 
            SetDataReady; 
            break;
         }
         case '2': //Axis 2
         {
            SET_DIR2;            
            MaxSpeed=SetSpeed2;
            IACC=ACC2;
            IDEC=DEC2;                     
            CurSpeedFrq=0;
            TCC0.CCA=0xFFFF;  
                                           
            ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a                        
            ACCSpeedINT = MaxSpeed / ACCSpeedINT; 
                                   
            DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a                           
            DECSpeedINT= MaxSpeed / DECSpeedINT;                       
            FreeJog=2;   
            ACC=1;    
            sprintf(BufferSend,"!0602\r\n");
            Responsing=1;  
            SetDataReady;
            break;
         } 
        case '3': //Axis 3
        {        
          SET_DIR3;                
          MaxSpeed=SetSpeed3;
          IACC=ACC3;
          IDEC=DEC3;          
          CurSpeedFrq=0;
          TCC0.CCA=0xFFFF;                                                   
                                 
          ACCSpeedINT =(long int)(100 *(float)MaxSpeed/(float)IACC);// t = V/a                        
          ACCSpeedINT = MaxSpeed / ACCSpeedINT; 
                        
          DECSpeedINT = (long int)(100 *(float)MaxSpeed/(float)IDEC);// t = V/a                           
          DECSpeedINT= MaxSpeed / DECSpeedINT;                 
          FreeJog=3;    
          ACC=1;    
          sprintf(BufferSend,"!0603\r\n");
          Responsing=1;  
          SetDataReady;
          break;
        }     
      }
     }
      else if(cmdTemp[3]=='3')//Free Jog   STOP
      {
       switch(cmdTemp[4])
       {
         case '1': //Axis 1
         {
            DEC=1;
            sprintf(BufferSend,"!0631\r\n"); 
            Responsing=1; 
            SetDataReady; 
            break;
         }
         case '2': //Axis 2
         {
            DEC=1;   
            sprintf(BufferSend,"!0632\r\n");
            Responsing=1;  
            SetDataReady;
            break;
         } 
        case '3': //Axis 3
        {        
          DEC=1;
          sprintf(BufferSend,"!0633\r\n");
          Responsing=1;  
          SetDataReady;
          break;
        }     
      }
     }
    } 
    }
    else if(strncmp(cmdTemp,"#07",3)==0)
    {//STOP JOG
    if(!ProgramRun)
    {
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {                                                
                DistanceToGo1 = 0;                       
                if(EchoON) printf("!0701\n\r");
                break;
             }
             case '2': //Axis 2
             {
                DistanceToGo2 = 0;                  
                if(EchoON) printf("!0702\n\r");
                break;
             }                          
             case '3': //Axis 3
             {
                DistanceToGo3 = 0;                  
                if(EchoON) printf("!0703\n\r");
                break;
             }               
           }
        }
      } 
    }
    else if(strncmp(cmdTemp,"?08",3)==0)
    {//Read DistanceToGo  
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {                   
                ltoa(DistanceToGo1,temp,10);            
                if(EchoON) printf("!0801\t%s\n\r",temp);
                break;
             }
             case '2': //Axis 2
             {
                ltoa(DistanceToGo2,temp,10);            
                if(EchoON) printf("!0802\t%s\n\r",temp);
                break;
             }                          
             case '3': //Axis 3
             {
                ltoa(DistanceToGo3,temp,10);            
                if(EchoON) printf("!0803\t%s\n\r",temp);
                break;
             }              
           }
      }
    } 
    else if(strncmp(cmdTemp,"?09",3)==0)
    {//Read Axis Position  
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {                   
                ltoa(AxisPosition1,temp,10);            
                sprintf(BufferSend,"!0901\t%s\r\n",temp);  
                Responsing=1;
                SetDataReady; 
                break;
             }
             case '2': //Axis 2
             {
                ltoa(AxisPosition2,temp,10);            
                sprintf(BufferSend,"!0902\t%s\r\n",temp); 
                Responsing=1;  
                SetDataReady;
                break;
             }                          
             case '3': //Axis 3
             {
                ltoa(AxisPosition3,temp,10);            
                sprintf(BufferSend,"!0903\t%s\r\n",temp);
                Responsing=1; 
                SetDataReady;
                break;
             }             
           }
      }
    }
     else if(strncmp(cmdTemp,"#10",3)==0)
    {//Clear JOG  
     if(!ProgramRun)
    {
      if(cmdTemp[3]=='0')
      {
           switch(cmdTemp[4])
           {
             case '1': //Axis 1
             {                                                
                DistanceToGo1 = 0;
                SetJog1 = AxisPosition1;
                if(EchoON) printf("!0701\n\r");
                break;
             }
             case '2': //Axis 2
             {
                DistanceToGo2 = 0;   
                SetJog2 = AxisPosition2;
                if(EchoON) printf("!0702\n\r");
                break;
             }                          
             case '3': //Axis 3
             {
                DistanceToGo3 = 0; 
                SetJog3 = AxisPosition3;                    
                if(EchoON) printf("!0703\n\r");
                break;
             }              
           }
        }
      } 
    }
    else if(strncmp(cmdTemp,"#11",3)==0)
    {//Set AXIS Position
    AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
         
    if (!ProgramRun)
     {
     while (AxisMoving == 1) 
      {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
      }
      if(cmdTemp[3]=='0')
      { 
        switch(cmdTemp[4])
        {
          case '1': //Axis 1
          {                   
            for(k=0;k<10;k++)
            {
              temp[k]=cmdTemp[k+5];
            }
            temp[10]=0;
            AxisPosition1=atol(temp);
            if(EchoON) printf("!1101\n\r");  
            break;                        
          }
          case '2'://Axis 2
          {                   
            for(k=0;k<10;k++)
            {
              temp[k]=cmdTemp[k+5];
            }
            temp[10]=0;
            AxisPosition2=atol(temp);
            if(EchoON) printf("!1102\n\r");  
            break;                        
          }
          case '3'://Axis 3
          {                   
            for(k=0;k<10;k++)
            {
              temp[k]=cmdTemp[k+5];
            }
            temp[10]=0;
            AxisPosition3=atol(temp);
            if(EchoON) printf("!1103\n\r");  
            break;                        
          }                                      
        }
      }            
     }
    } 
    else if(strncmp(cmdTemp,"#12",3)==0)
    {//Go To REF  
    AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;       
    if (!ProgramRun)
     {
     while (AxisMoving == 1) 
     {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
     }
       if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              break; 
            }
            case '2': //Axis 2
            {
              break; 
            }
            case '3': //Axis 3
            {
              break; 
            }
            case '4': //Axis 4
            {
              break; 
            }
         }
       }
     }
    }   
    else if(strncmp(cmdTemp,"#13",3)==0)
    {// Set Speed 
     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
     if (!ProgramRun & !AxisMoving)
     {
       if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              SetSpeed1=atof(temp);  
              sprintf(BufferSend,"!1301\t%s\r\n",temp); 
              Responsing=1; 
              SetDataReady;
              break; 
            }
            case '2': //Axis 2
            {
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              SetSpeed2=atof(temp);   
              sprintf(BufferSend,"!1302\t%s\r\n",temp); 
              Responsing=1;
              SetDataReady;
              break; 
            }
            case '3': //Axis 3
            {
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              SetSpeed3=atof(temp);   
              sprintf(BufferSend,"!1303\t%s\r\n",temp); 
              Responsing=1;
              SetDataReady;
              break; 
            }            
         }
       }
     }
    }
    else if(strncmp(cmdTemp,"?14",3)==0)
    {//AxisIsMoving
       if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              if(EchoON) printf("!1401\t%u\n\r",AxisIsMoving1);                      
              break; 
            }
            case '2': //Axis 2
            {
              if(EchoON) printf("!1402\t%u\n\r",AxisIsMoving2);
              break; 
            }
            case '3': //Axis 3
            {
              if(EchoON) printf("!1403\t%u\n\r",AxisIsMoving3);
              break; 
            }            
         }
       }
    }
    else if(strncmp(cmdTemp,"?15",3)==0)
    {//Read Axis Speed
       if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              if(EchoON) printf("!1501\t%lu",SetSpeed1);                      
              break; 
            }
            case '2': //Axis 2
            {
              if(EchoON) printf("!1502\t%lu",SetSpeed2);
              break; 
            }
            case '3': //Axis 3
            {
              if(EchoON) printf("!1503\t%lu",SetSpeed3);
              break; 
            }            
         }
       }
    }
    else if(strncmp(cmdTemp,"#16",3)==0)
    {// Set ACC 
     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
     if (!ProgramRun & !AxisMoving)
     {
      if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              ACC1=atof(temp);  
              sprintf(BufferSend,"!1601\t%s\r\n",temp); 
              Responsing=1;    
              SetDataReady;
              break; 
            }
            case '2': //Axis 2
            { 
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              ACC2=atof(temp);   
              sprintf(BufferSend,"!1602\t%s\r\n",temp); 
              Responsing=1;  
              SetDataReady;
              break; 
            }
            case '3': //Axis 3
            {
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              ACC3=atof(temp);   
              sprintf(BufferSend,"!1603\t%s\r\n",temp); 
              Responsing=1;
              SetDataReady;
              break; 
            }
            
         }
       }          
      }
    }
    else if(strncmp(cmdTemp,"#17",3)==0)
    {// Set DEC 
     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
     if (!ProgramRun & !AxisMoving)
     {     
       if(cmdTemp[3]=='0')
       {
         switch(cmdTemp[4])
         {
            case '1': //Axis 1
            {
              while(AxisIsMoving1)
              {
                 /*wait until Is Moving*/
              } 
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              DEC1=atof(temp);  
              sprintf(BufferSend,"!1701\t%s\r\n",temp); 
              Responsing=1;  
              SetDataReady;
              break; 
            }
            case '2': //Axis 2
            {
              while(AxisIsMoving2)
              {
                 /*wait until Is Moving*/
              }
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              DEC2=atof(temp);  
              sprintf(BufferSend,"!1702\t%s\r\n",temp); 
              Responsing=1;
              SetDataReady;
              break; 
            }
            case '3': //Axis 3
            {
              while(AxisIsMoving3)
              {
                 /*wait until Is Moving*/
              } 
              for(k=0;k<10;k++)
              {
                temp[k]=cmdTemp[k+5];
              }
              temp[10]=0;
              DEC3=atof(temp);  
              sprintf(BufferSend,"!1703\t%s\r\n",temp); 
              Responsing=1;    
              SetDataReady;
              break; 
            }            
         }
       }
     }
    }                          
    else if(strncmp(cmdTemp,"#1800",5)==0)
    {//Go To Refrence 
     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;           
     if (!ProgramRun)
     {
      while (AxisMoving == 1) 
      {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
      }   
      
      printf("RefDir : %u\r\n",Axes[0].RefrenceDir);
      if(Axes[0].RefrenceDir==1)
      {
        
        CLR_DIR1;
      }
      else if(Axes[0].RefrenceDir==0)
      {
        SET_DIR1;
      } 
      
      if(Axes[1].RefrenceDir==1)
      {
        CLR_DIR2;
      }
      else if(Axes[1].RefrenceDir==0)
      {
        SET_DIR2;
      }
      
      if(Axes[2].RefrenceDir==1)
      {
        CLR_DIR3;
      }
      else if(Axes[2].RefrenceDir==0)
      {
        SET_DIR3;
      }       
                             
      MaxSpeed=8000;                         
        
      CurSpeedFrq = MaxSpeed;
      ACC = 0;
      DEC = 0;
      TimerMainPeriod = (float)((float)16000.0 / (float) CurSpeedFrq);          
      RefCmd=1;
      Ma1=1000; 
      Ma2=1000;
      Ma3=1000;
      DistanceToGo1=0x7FFFFFFF;
      DistanceToGo2=0x7FFFFFFF;
      DistanceToGo3=0x7FFFFFFF;   
         
      tcc0_init();     
      
      TCC0.CCA = (unsigned int)(TimerMainPeriod*1000);                    
                
      sprintf(BufferSend,"!1800\r\n"); 
      AxisPosition1=0;
      AxisPosition2=0;
      AxisPosition3=0;
      Responsing=1; 
      SetDataReady;                      
     }
    }     
    else if(strncmp(cmdTemp,"#1901",5)==0)
    {
        RefDir1=0;  
        sprintf(BufferSend,"!1901\r\n"); 
        Responsing=1; 
        SetDataReady;                    
    }
    else if(strncmp(cmdTemp,"#1902",5)==0)
    {             
        RefDir2=0;    
        sprintf(BufferSend,"!1902\r\n"); 
        Responsing=1; 
        SetDataReady; 
    }
    else if(strncmp(cmdTemp,"#1903",5)==0)
    {             
        RefDir3=0; 
        sprintf(BufferSend,"!1903\r\n"); 
        Responsing=1; 
        SetDataReady; 
    }
    else if(strncmp(cmdTemp,"#2001",5)==0)
    {
      RefDir1=1;    
      sprintf(BufferSend,"!2001\r\n"); 
      Responsing=1; 
      SetDataReady; 
    } 
    else if(strncmp(cmdTemp,"#2002",5)==0)
    {
        RefDir2=1;  
        sprintf(BufferSend,"!2002\r\n"); 
        Responsing=1; 
        SetDataReady; 
    }
    else if(strncmp(cmdTemp,"#2003",5)==0)
    {
        RefDir3=1;
        sprintf(BufferSend,"!2003\r\n"); 
        Responsing=1; 
        SetDataReady; 
    }   
    else if(strncmp(cmdTemp,"#2110",5)==0)  
    {
        //Disable Hardware Positive Limit 1 
        PLimitIsActiveA1=0;
        sprintf(BufferSend,"!2110\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2111",5)==0)
    {     
        //Enable Hardware Positive Limit 1
        PLimitIsActiveA1=1; 
        sprintf(BufferSend,"!2111\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2120",5)==0)
    {
        //Disable Hardware Positive Limit 2
        PLimitIsActiveA2=0;
        sprintf(BufferSend,"!2120\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2121",5)==0)
    {
        //Enable Hardware Positive Limit 2
        PLimitIsActiveA2=1;
        sprintf(BufferSend,"!2121\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2130",5)==0)
    { 
        //Enable Hardware Positive Limit 3
        PLimitIsActiveA3=0;
        sprintf(BufferSend,"!2130\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2131",5)==0)
    { 
        //Enable Hardware Positive Limit 3 
        PLimitIsActiveA3=1;
        sprintf(BufferSend,"!2131\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2210",5)==0)  
    {  
        NLimitIsActiveA1=0;
        sprintf(BufferSend,"!2210\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2211",5)==0)
    { 
        NLimitIsActiveA1=1; 
        sprintf(BufferSend,"!2211\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2220",5)==0)
    {
        NLimitIsActiveA2=0;
        sprintf(BufferSend,"!2220\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2221",5)==0)
    {
        NLimitIsActiveA2=1;
        sprintf(BufferSend,"!2221\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2230",5)==0)
    { 
        NLimitIsActiveA3=0;
        sprintf(BufferSend,"!2230\r\n"); 
        Responsing=1; 
        SetDataReady;
    }
    else if(strncmp(cmdTemp,"#2231",5)==0)
    {  
        NLimitIsActiveA3=1;
        sprintf(BufferSend,"!2231\r\n"); 
        Responsing=1; 
        SetDataReady;
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
    AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
         
    if (!ProgramRun)
     {
     while (AxisMoving == 1) 
     {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
     }      
        if(SetJog1<AxisPosition1) CLR_DIR1;
        if(SetJog1>AxisPosition1) SET_DIR1; 
                
        if(SetJog2<AxisPosition2) CLR_DIR2;
        if(SetJog2>AxisPosition2) SET_DIR2;  
            
        if(SetJog3<AxisPosition3) CLR_DIR3;
        if(SetJog3>AxisPosition3) SET_DIR3;           
              
        if(SetSpeed1 >= SetSpeed2 && SetSpeed1 >= SetSpeed3)
          { 
            MaxSpeed = SetSpeed1; 
            IACC=ACC1; 
            IDEC=DEC1; 
            MaxDistanceToGo=labs(AxisPosition1-SetJog1);
          }
        if(SetSpeed2 >= SetSpeed1 && SetSpeed2 >= SetSpeed3)
          { 
            MaxSpeed = SetSpeed2; 
            IACC=ACC2; 
            IDEC=DEC2; 
            MaxDistanceToGo=labs(AxisPosition2-SetJog2);
          }
        if(SetSpeed3 >= SetSpeed2 && SetSpeed3 >= SetSpeed1)
        { 
          MaxSpeed = SetSpeed3; 
          IACC=ACC3; 
          IDEC=DEC3; 
          MaxDistanceToGo=labs(AxisPosition3-SetJog3);
        }
        if(MaxSpeed>0)
        {
          Ma1=(unsigned int)((float)(SetSpeed1/MaxSpeed)*1000);
          Ma2=(unsigned int)((float)(SetSpeed2/MaxSpeed)*1000);  
          Ma3=(unsigned int)((float)(SetSpeed3/MaxSpeed)*1000);          		  
        }
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
                 
        DistanceToGo1=labs(AxisPosition1-SetJog1);  
        DistanceToGo2=labs(AxisPosition2-SetJog2);        
        ACC=1; 
        sprintf(BufferSend,"!30\tMaxS:%ld\tIACC:%ld\tIDEC:%ld\n\rMa1:%u\tMa2:%u\tBACC:%ld\tBDEC:%ld\tMAXDTG:%ld\n\r",MaxSpeed,IACC,IDEC,Ma1,Ma2,ACCSpeedINT,DECSpeedINT,MaxDistanceToGo); 
        Responsing=1;                     
        SetDataReady;
     }     
    }            
    else if(strncmp(cmdTemp,"#4000",5)==0)
    { 
      //if(ProgramRun)
      //{
          ProgramRun=0;
          DistanceToGo1=0;
          DistanceToGo2=0;
          DistanceToGo3=0;   
          DO1_CLR;
          sprintf(BufferSend,"!4000\r\n"); 
          Responsing=1;      
      //}
    } 
    else if(strncmp(cmdTemp,"#4100",5)==0)
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
    }
    else if(strncmp(cmdTemp,"#9800",5)==0)
    {  
          
     AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;       
     if (!ProgramRun)
     { 
      printf("Get Run Command\r\n");
      while (AxisMoving == 1) 
      {
       AxisMoving = AxisIsMoving1 || AxisIsMoving2 || AxisIsMoving3;
      }
       ContinuousRead=0; 
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
    else if(strncmp(cmdTemp,"#9900",5)==0)
    { 
       ContinuousRead=0; 
       readRam(); 
       readRam(); 
       readRam(); 
       sprintf(BufferSend,"!9800\r\nPL:%u\tMD:%u\tMv1:%ld\tMaxDTG:%ld\tMaxSpeed:%ld\tACCDTG:%ld\r\n",CMDPRGlist.PRGLine,CMDPRGlist.Mode,CMDPRGlist.Move1,CMDPRGlist.MaxDistanceToGo,CMDPRGlist.MaxSpeed,CMDPRGlist.ACCDTG); 
       //sprintf(BufferSend,"!9900\r\nID:%u\tMaxSpeed:%d\tACC:%d\r\n",Axes[0].ID,Axes[0].MaxSpeed,Axes[0].ACC);
       Responsing=1;
       SetDataReady;
    }    
    else
    {
      ClrDataReady;
    }
}

