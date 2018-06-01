#define BIT(x) 			(1 << (x))
#define SETBITS(x,y) 	((x) |= (y))
#define CLEARBITS(x,y) 	((x) &= (~(y)))
#define SETBIT(x,y) 	SETBITS((x), (BIT((y))))			/* EXAMPLE SETBIT(PORTB,2) sets the 2 bit of PORTB */
#define CLRBIT(x,y) 	CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) 	((x) & (BIT(y)))
#define BITCLEAR(x,y) 	!BITSET((x), (y))
#define BITSSET(x,y) 	(((x) & (y)) == (y))
#define BITSCLEAR(x,y) 	(((x) & (y)) == 0)
#define TSTBIT(x,y) 	(((x)>>(y)) & 1)
#define TGLBIT(x,y)     (TSTBIT(x,y))?CLEARBITS(x,y):SETBIT(x,y)
//Outputs
#define DIR1         TSTBIT(PORTK.OUT,4) //Axis 1 Direction
#define SET_DIR1     SETBIT(PORTK.OUT,4) //Axis 1 Direction
#define CLR_DIR1     CLRBIT(PORTK.OUT,4) //Axis 1 Direction
#define PULSE1       PORTK.OUTTGL = 0x20 //TGLBIT(PORTK.OUT,5)
#define EMG1         TSTBIT(PORTA.OUT,6) //Axis 1 EMG output 
#define SET_EMG1     SETBIT(PORTA.OUT,6) //Axis 1 EMG output 
#define CLR_EMG1     CLRBIT(PORTA.OUT,6) //Axis 1 EMG output 
#define ENABLE1      TSTBIT(PORTA.OUT,7) //Axis 1 Enable Output
#define SET_ENABLE1  SETBIT(PORTA.OUT,7) //Axis 1 Enable Output
#define CLR_ENABLE1  CLRBIT(PORTA.OUT,7) //Axis 1 Enable Output

#define DIR2         TSTBIT(PORTK.OUT,2)
#define SET_DIR2     SETBIT(PORTK.OUT,2)
#define CLR_DIR2     CLRBIT(PORTK.OUT,2)
#define PULSE2       PORTK.OUTTGL=0x08   //TGLBIT(PORTK.OUT,3)
#define EMG2         TSTBIT(PORTB.OUT,4)
#define SET_EMG2     SETBIT(PORTB.OUT,4)
#define CLR_EMG2     CLRBIT(PORTB.OUT,4)
#define ENABLE2      TSTBIT(PORTB.OUT,7)
#define SET_ENABLE2  SETBIT(PORTB.OUT,7)
#define CLR_ENABLE2  CLRBIT(PORTB.OUT,7)

#define DIR3         TSTBIT(PORTK.OUT,0)
#define SET_DIR3     SETBIT(PORTK.OUT,0)
#define CLR_DIR3     CLRBIT(PORTK.OUT,0)
#define PULSE3       PORTK.OUTTGL=0x02   //TGLBIT(PORTK.OUT,1)
#define EMG3         TSTBIT(PORTC.OUT,3)
#define SET_EMG3     SETBIT(PORTC.OUT,3)
#define CLR_EMG3     CLRBIT(PORTC.OUT,3)
#define ENABLE3      TSTBIT(PORTC.OUT,7)
#define SET_ENABLE3  SETBIT(PORTC.OUT,7)
#define CLR_ENABLE3  CLRBIT(PORTC.OUT,7)

#define DO1_SET      SETBIT(PORTJ.OUT,2) // Auxiliary output on 25din pin 
#define DO1_CLR      CLRBIT(PORTJ.OUT,2) // Auxiliary output on 25din pin 
#define DO2_SET      SETBIT(PORTJ.OUT,3) // Auxiliary output on 25din pin 
#define DO2_CLR      CLRBIT(PORTJ.OUT,3) // Auxiliary output on 25din pin 
#define DO3_SET      SETBIT(PORTJ.OUT,4) // Auxiliary output on 25din pin 
#define DO3_CLR      CLRBIT(PORTJ.OUT,4) // Auxiliary output on 25din pin 
#define DO4_SET      SETBIT(PORTJ.OUT,5) // Auxiliary output on 25din pin 
#define DO4_CLR      CLRBIT(PORTJ.OUT,5) // Auxiliary output on 25din pin 
#define DO5_SET      SETBIT(PORTJ.OUT,7) // Auxiliary output on 25din pin 
#define DO5_CLR      CLRBIT(PORTJ.OUT,7) // Auxiliary output on 25din pin 
#define DO6_SET      SETBIT(PORTJ.OUT,6) // Auxiliary output on 25din pin 
#define DO6_CLR      CLRBIT(PORTJ.OUT,6) // Auxiliary output on 25din pin 

#define SG11_SET     SETBIT(PORTH.OUT,3) // seven SegmentBoard interlocks 
#define SG11_CLR     CLRBIT(PORTH.OUT,3) // seven SegmentBoard interlocks 
#define SG12_SET     SETBIT(PORTH.OUT,2) // seven SegmentBoard interlocks 
#define SG12_CLR     CLRBIT(PORTH.OUT,2) // seven SegmentBoard interlocks 
#define SG13_SET     SETBIT(PORTH.OUT,5) // seven SegmentBoard interlocks 
#define SG13_CLR     CLRBIT(PORTH.OUT,5) // seven SegmentBoard interlocks 
#define SG14_SET     SETBIT(PORTH.OUT,4) // seven SegmentBoard interlocks 
#define SG14_CLR     CLRBIT(PORTH.OUT,4) // seven SegmentBoard interlocks 
#define SG15_SET     SETBIT(PORTH.OUT,7) // seven SegmentBoard interlocks 
#define SG15_CLR     CLRBIT(PORTH.OUT,7) // seven SegmentBoard interlocks 
#define SG16_SET     SETBIT(PORTH.OUT,6) // seven SegmentBoard interlocks 
#define SG16_CLR     CLRBIT(PORTH.OUT,6) // seven SegmentBoard interlocks 
#define SG17_SET     SETBIT(PORTJ.OUT,1) // seven SegmentBoard interlocks 
#define SG17_CLR     CLRBIT(PORTJ.OUT,1) // seven SegmentBoard interlocks 
#define SG18_SET     SETBIT(PORTJ.OUT,0) // seven SegmentBoard interlocks 
#define SG18_CLR     CLRBIT(PORTJ.OUT,0) // seven SegmentBoard interlocks 

#define LED1         PORTR.OUTTGL=0x01  //TGLBIT(PORTR.OUT,0)
#define LED2         PORTR.OUTTGL=0x02  //TGLBIT(PORTR.OUT,1)

#define SETSFRST     SETBIT(PORTE.OUT,2) //set Serial Flash Reset
#define CLRSFRST     CLRBIT(PORTE.OUT,2) //Clear Serial Flash Reset

#define SETSFWP      SETBIT(PORTE.OUT,1) //Set Serial Flash Write protection
#define CLRSFWP      CLRBIT(PORTE.OUT,1) //Clear Serial Flash Write protection 

#define SETSFCS      SETBIT(PORTE.OUT,4); //Set serial Flash Chip Select
#define CLRSFCS      CLRBIT(PORTE.OUT,4); //Reset Serail Flash Chip Select

#define SetDataReady SETBIT(PORTF.OUT,0) //say to Raspberry that there is some data on SPI port to receive
#define ClrDataReady CLRBIT(PORTF.OUT,0) //say to Raspberry that there is not any data on SPI port to receive


/****************************************************/    
/*****Inputs*****************************************/

#define READY1       TSTBIT(PORTA.IN,0) //Axis 1 Ready input signal
#define INPOS1       TSTBIT(PORTA.IN,1) //Axis 1 Inpos Input signal
#define ALARM1       TSTBIT(PORTA.IN,2) //Axis 1 Alarm Input signal
#define LIM_POS1     TSTBIT(PORTA.IN,3) //Axis 1 Limit Positive input signal
#define LIM_NEG1     TSTBIT(PORTA.IN,4) //Axis 1 limit negative input signal
#define REF1         TSTBIT(PORTA.IN,5) //Axis 1 Reference Signal

#define READY2       TSTBIT(PORTB.IN,3)
#define INPOS2       TSTBIT(PORTB.IN,2)
#define ALARM2       TSTBIT(PORTB.IN,1)
#define LIM_POS2     TSTBIT(PORTB.IN,0)
#define LIM_NEG2     TSTBIT(PORTB.IN,6)
#define REF2         TSTBIT(PORTB.IN,5)

#define READY3       TSTBIT(PORTC.IN,2)
#define INPOS3       TSTBIT(PORTC.IN,1)
#define ALARM3       TSTBIT(PORTC.IN,0)
#define LIM_POS3     TSTBIT(PORTC.IN,5)
#define LIM_NEG3     TSTBIT(PORTC.IN,4)
#define REF3         TSTBIT(PORTC.IN,6)

#define JogPos       TSTBIT(PORTH.IN,1) //Positive Jog Key 
#define JogNeg       TSTBIT(PORTD.IN,3) //Negative Jog Key

#define RunProgram   TSTBIT(PORTD.IN,1) //Run Program Key

#define StopProgram  TSTBIT(PORTD.IN,2) //Stop Program Key
