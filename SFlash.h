//save program into flash
//read program from flash
#define FPRGpage 1000  

char ContinuousRead=0;
long int fk = 0;
int ft = 0;
char fadd1 = 0;
char fadd2 = 0;
struct CMDPRGType
{
  unsigned int PRGLine; 
  char BlockSize;
  char Mode; 
  char AxisNum;
  long int Move1;
  long int Move2;
  long int Move3;

  char Direction1;
  char Direction2;
  char Direction3;
  
  long int Speed1Ma1;
  long int Speed2Ma2;
  long int Speed3Ma3;
 
  long int MaxSpeed; 
  
  long int MaxDistanceToGo;
   
  long int ACCSpeedINT;
  long int DECSpeedINT;    
  
  long int ACCDTG;
  long int DECDTG;   
  
  char SerialCountType;
  
  unsigned long int SerialFrom;
  unsigned long int SerialTo; 
  
  char FillWithZero;
  char DigitNumber;
  
};
struct CMDPRGType CMDPRGlist;

struct Axis
{
  unsigned int ID;
  char Name;
  char Type;
  char ReversDir;
  long int MaxSpeed;
  long int ACC;
  long int DEC;
  long int PSLimit;//Positive Software Limit 
  long int NSLimit;//Negative Software Limit          
  unsigned int BreakTime;
  char RefrenceType;
  char RefrenceDir;
  long int RefrenceSpeed1;
  long int RefrenceSpeed2;
  char AlarmIsActive;
  char InpositionIsActive;
  char EmergencyIsActive;
  char BreakIsActive;
  char HardwareLimitPosIsActive;
  char HardwareLimitNegIsActive;
};
struct Axis Axes[3];

// SPIE initialization
void spie_init(void)
{
// SPIE is enabled
// SPI mode: 0
// Operating as: Master
// Data order: MSB First
// SCK clock prescaler: 128
// SCK clock doubled: Off
// SCK clock frequency: 250.000 kHz
SPIE.CTRL=SPI_ENABLE_bm | SPI_MODE_0_gc | (1<<SPI_MASTER_bp) | (0<<SPI_DORD_bp) |
	SPI_PRESCALER_DIV128_gc | (0<<SPI_CLK2X_bp);

// SPIE interrupt: Disabled
SPIE.INTCTRL=SPI_INTLVL_OFF_gc;

// Note: The MOSI (PORTE Pin 5), SCK (PORTE Pin 7) and
// /SS (PORTE Pin 4) signals are configured as outputs in the ports_init function.
}

// Macro used to drive the SPIE /SS signal low in order to select the slave
#define SET_SPIE_SS_LOW {PORTE.OUTCLR=SPI_SS_bm;}
// Macro used to drive the SPIE /SS signal high in order to deselect the slave
#define SET_SPIE_SS_HIGH {PORTE.OUTSET=SPI_SS_bm;}

// SPIE transmit/receive function in Master mode
// c - data to be transmitted
// Returns the received data

#pragma used+
unsigned char spie_master_tx_rx(unsigned char c)
{
// Transmit data in Master mode
SPIE.DATA=c;
// Wait for the data to be transmitted/received
while ((SPIE.STATUS & SPI_IF_bm)==0);
// Return the received data
return SPIE.DATA;
}
#pragma used-

void ltob(long int li,char *chs)
{
   long int temp = li;
   
   chs[0] = temp & 0xff;
   temp = temp >> 8;
   chs[1] = temp & 0xff;
   temp = temp >> 8;
   chs[2] = temp & 0xff;
   temp = temp >> 8;
   chs[3] = temp & 0xff;
   temp = temp >> 8;    
}

long int btol (char *chs)
{
     long int res;
     
     res = (long int)((long int)chs[3] << 24) + (long int)((long int)chs[2] << 16) + (long int)((long int)chs[1] << 8) + (long int) chs[0] ;  
     return res;
}


void utob (unsigned int ui , char *chs)
{
   unsigned int temp = ui;
   
   chs[0] = temp & 0xff;
   temp = temp >> 8;
   chs[1] = temp & 0xff;
   temp = temp >> 8;  
}

unsigned int btou (char *chs)
{
    unsigned int res;
    res = (chs[1] << 8) + chs[0] ; 
    return res;
}

int btoi (char *chs)
{
    int res;
    res = (chs[1] << 8) + chs[0] ; 
    return res;
}

void ReadConfig()
{
   char b[4];
   char ax; 
  
   //How make FlashPge address 
   // the flash page is : 0
   //Data stat from start   
   fk = 0; 
   ft = fk << 1;
   fadd1 = ft & 0xff;
   fadd2 = (ft>>8)& 0xff; 
   
   SETSFCS;
   CLRSFCS;      

   spie_master_tx_rx(0x03);  //Read Continues opcode
   spie_master_tx_rx(fadd2); //address
   spie_master_tx_rx(fadd1); //address
   spie_master_tx_rx(0x00);  //00  
   
   //Reading configuration data from flash just for 3 axis
   for(ax=0;ax<3;ax++)
   {
        b[1]=spie_master_tx_rx(0x00); //push clock to read
        b[0]=spie_master_tx_rx(0x00); //push clock to read
        Axes[ax].ID = btou(b);  
        
        Axes[ax].Name = spie_master_tx_rx(0x00);
        Axes[ax].Type = spie_master_tx_rx(0x00); 
        Axes[ax].ReversDir = spie_master_tx_rx(0x00); 
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].MaxSpeed = btol(b); 
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].ACC = btol(b);     
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].DEC = btol(b);   
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].PSLimit = btol(b);
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].NSLimit = btol(b);           
       
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].BreakTime = btou(b); 
        
        Axes[ax].RefrenceType = spie_master_tx_rx(0x00);  
        
        Axes[ax].RefrenceDir = spie_master_tx_rx(0x00);    
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].RefrenceSpeed1 = btol(b);     
        
        b[3]=spie_master_tx_rx(0x00);
        b[2]=spie_master_tx_rx(0x00);
        b[1]=spie_master_tx_rx(0x00);
        b[0]=spie_master_tx_rx(0x00);
        Axes[ax].RefrenceSpeed2 = btol(b); 
        
        Axes[ax].AlarmIsActive = spie_master_tx_rx(0x00);  
        
        Axes[ax].InpositionIsActive = spie_master_tx_rx(0x00);  
        
        Axes[ax].EmergencyIsActive = spie_master_tx_rx(0x00);  
        
        Axes[ax].BreakIsActive = spie_master_tx_rx(0x00);  
        
        Axes[ax].HardwareLimitPosIsActive = spie_master_tx_rx(0x00);  
        printf("Axes : %u\t HLP Activation : %u\r\n",ax,Axes[ax].HardwareLimitPosIsActive);
        Axes[ax].HardwareLimitNegIsActive = spie_master_tx_rx(0x00); 
        printf("Axes : %u\t HLN Activation : %u\r\n",ax,Axes[ax].HardwareLimitNegIsActive);        
   }  
}

//read one block of program from ram and save it into CMDPRGList
void readRam(void)
{ 

 char b[4];
 int bi;   
 printf("Start readRam\r\n");
 if(ContinuousRead==0)
 {
   printf("Reading First block\r\n");
   fk = FPRGpage;    
   printf("First Page Address: %u\r\n",fk);
   ft = fk << 1;
   fadd1 = ft & 0xff;
   fadd2 = (ft>>8)& 0xff; 
   
   SETSFCS;
   CLRSFCS;      

   spie_master_tx_rx(0x03);
   spie_master_tx_rx(fadd2);
   spie_master_tx_rx(fadd1);
   spie_master_tx_rx(0x00);
   ContinuousRead=1; 
 }                             
 
 //Read PRG Line 
 b[1]=spie_master_tx_rx(0x00);
 b[0]=spie_master_tx_rx(0x00);
 CMDPRGlist.PRGLine=btou(b);   
 //------------------------------------------
 CMDPRGlist.BlockSize=spie_master_tx_rx(0x00);
 CMDPRGlist.Mode=spie_master_tx_rx(0x00);

 switch(CMDPRGlist.Mode)
 {
   case 0:
   {
      CMDPRGlist.AxisNum=spie_master_tx_rx(0x00); 
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move1=btol(b);
      }
      
      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move2=btol(b); 
      }
      
      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move3=btol(b);\
      }
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed1Ma1=btol(b);
      }

      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed2Ma2=btol(b);
      }

      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed3Ma3=btol(b);
      } 
      
      if (CMDPRGlist.AxisNum > 0) CMDPRGlist.Direction1=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 1) CMDPRGlist.Direction2=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 2) CMDPRGlist.Direction3=spie_master_tx_rx(0x00); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxSpeed=btol(b); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxDistanceToGo = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.ACCSpeedINT = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.DECSpeedINT = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.ACCDTG = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.DECDTG = btol(b); 
      break;                    
      
   }
   case 1:
   {  
      CMDPRGlist.AxisNum=spie_master_tx_rx(0x00); 
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move1=btol(b);
      }
      
      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move2=btol(b); 
      }
      
      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move3=btol(b);\
      }
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed1Ma1=btol(b);
      }

      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed2Ma2=btol(b);
      }

      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed3Ma3=btol(b);
      } 
      
      if (CMDPRGlist.AxisNum > 0) CMDPRGlist.Direction1=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 1) CMDPRGlist.Direction2=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 2) CMDPRGlist.Direction3=spie_master_tx_rx(0x00); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxSpeed=btol(b); 
      break;                    

   } 
   case 90:
   {
     CMDPRGlist.SerialCountType=spie_master_tx_rx(0x00);
     
     b[3]=spie_master_tx_rx(0x00);
     b[2]=spie_master_tx_rx(0x00);
     b[1]=spie_master_tx_rx(0x00);
     b[0]=spie_master_tx_rx(0x00);
     CMDPRGlist.SerialFrom=btol(b);
     
     b[3]=spie_master_tx_rx(0x00);
     b[2]=spie_master_tx_rx(0x00);
     b[1]=spie_master_tx_rx(0x00);
     b[0]=spie_master_tx_rx(0x00);
     CMDPRGlist.SerialTo=btol(b);
     
     CMDPRGlist.FillWithZero = spie_master_tx_rx(0x00);
     CMDPRGlist.DigitNumber = spie_master_tx_rx(0x00);
     break;
   }
 }
}

//Read Serial Number from Flash this is just one serial number 
// Serial Number is in page : 999
unsigned long int ReadSerial()
{  
   char b[4];
   printf("Reading Serial...\r\n");
   fk = 999; 
   ft = fk << 1;
   fadd1 = ft & 0xff;
   fadd2 = (ft>>8)& 0xff; 
   SETSFCS;
   CLRSFCS;      

   spie_master_tx_rx(0x03);
   spie_master_tx_rx(fadd2);
   spie_master_tx_rx(fadd1);
   spie_master_tx_rx(0x00); 
   
   b[3]=spie_master_tx_rx(0x00);
   b[2]=spie_master_tx_rx(0x00);
   b[1]=spie_master_tx_rx(0x00);
   b[0]=spie_master_tx_rx(0x00);  
   
   printf("B0: %u\tB1: %u\tB2: %u\tB3: %u\r\n",b[0],b[1],b[2],b[3]);
   return btol(b);
   
}

//Read sub program for each characters 
// normally sub programs start from page 100 and it will contains just 10 sub program each sub program have
// 10 pages 
void ReadSubProgram(unsigned int StartPage)
{ 
 char b[4];
 int bi;   
 
 if(ContinuousRead==0)
 {
   fk = StartPage; 
   ft = fk << 1;
   fadd1 = ft & 0xff;
   fadd2 = (ft>>8)& 0xff; 
   SETSFCS;
   CLRSFCS;      

   spie_master_tx_rx(0x03);
   spie_master_tx_rx(fadd2);
   spie_master_tx_rx(fadd1);
   spie_master_tx_rx(0x00);
   ContinuousRead=1; 
 }                             
 
 //Read PRG Line 
 b[1]=spie_master_tx_rx(0x00);
 b[0]=spie_master_tx_rx(0x00);
 CMDPRGlist.PRGLine=btou(b);   
 //------------------------------------------
 CMDPRGlist.BlockSize=spie_master_tx_rx(0x00);
 CMDPRGlist.Mode=spie_master_tx_rx(0x00);

 switch(CMDPRGlist.Mode)
 {
   case 0:
   {
      CMDPRGlist.AxisNum=spie_master_tx_rx(0x00); 
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move1=btol(b);
      }
      
      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move2=btol(b); 
      }
      
      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move3=btol(b);\
      }
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed1Ma1=btol(b);
      }

      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed2Ma2=btol(b);
      }

      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed3Ma3=btol(b);
      } 
      
      if (CMDPRGlist.AxisNum > 0) CMDPRGlist.Direction1=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 1) CMDPRGlist.Direction2=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 2) CMDPRGlist.Direction3=spie_master_tx_rx(0x00); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxSpeed=btol(b); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxDistanceToGo = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.ACCSpeedINT = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.DECSpeedINT = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.ACCDTG = btol(b); 

      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.DECDTG = btol(b); 
      break;                    
      
   }
   case 1:
   {  
      CMDPRGlist.AxisNum=spie_master_tx_rx(0x00); 
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move1=btol(b);
      }
      
      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move2=btol(b); 
      }
      
      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Move3=btol(b);\
      }
      
      if (CMDPRGlist.AxisNum > 0)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed1Ma1=btol(b);
      }

      if (CMDPRGlist.AxisNum > 1)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed2Ma2=btol(b);
      }

      if (CMDPRGlist.AxisNum > 2)
      {
          for(bi=3;bi>=0;bi--)
          {
            b[bi]=spie_master_tx_rx(0x00);
          }  
          CMDPRGlist.Speed3Ma3=btol(b);
      } 
      
      if (CMDPRGlist.AxisNum > 0) CMDPRGlist.Direction1=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 1) CMDPRGlist.Direction2=spie_master_tx_rx(0x00);

      if (CMDPRGlist.AxisNum > 2) CMDPRGlist.Direction3=spie_master_tx_rx(0x00); 
      
      for(bi=3;bi>=0;bi--)
      {
        b[bi]=spie_master_tx_rx(0x00);
      }  
      CMDPRGlist.MaxSpeed=btol(b); 
      break;                    

   } 
   
   
 }
}

// write serial number 
// Page is : 999
void WriteSerial(long int serial)
{
 char ftemp1;
 char charTemp[4];
 fk=999;
 ft = fk << 1;
 fadd1 = ft & 0xff;
 fadd2 = (ft>>8)& 0xff; 
 
 SETSFCS;
 CLRSFCS; 
    
 spie_master_tx_rx(0x81);   //delette
 spie_master_tx_rx(fadd2);
 spie_master_tx_rx(fadd1);
 spie_master_tx_rx(0x00);
 SETSFCS;
 do
 {                       
   CLRSFCS;  //active   
   TGLBIT(PORTR.OUT,0);
   spie_master_tx_rx(0xD7);       //wait until busy
   ftemp1 = spie_master_tx_rx(0); 
   printf("status: %u\r\n",ftemp1);
   SETSFCS;  
 }
 while ((ftemp1 & 0x80)==0); 
 
 CLRSFCS;    //start writing  
 TGLBIT(PORTR.OUT,1); 
 spie_master_tx_rx(0x84);   // first buffer
 spie_master_tx_rx(fadd2);
 spie_master_tx_rx(fadd1); 
 spie_master_tx_rx(0x00);  
   
 ltob(serial,charTemp);
 printf("B0:%u\tB1:%u\tB2:%u\tB3:%u\r\n",charTemp[0],charTemp[1],charTemp[2],charTemp[3]);
          
 spie_master_tx_rx(charTemp[3]); 
 spie_master_tx_rx(charTemp[2]);
 spie_master_tx_rx(charTemp[1]);
 spie_master_tx_rx(charTemp[0]); 

 SETSFCS;
 CLRBIT(PORTE.OUT,4);
 spie_master_tx_rx(0x83);    //write into page from buffer 1
 spie_master_tx_rx(fadd2);
 spie_master_tx_rx(fadd1);
 spie_master_tx_rx(0x00); 
 SETSFCS; 
}       
