// Disable a Timer/Counter type TC0
void tc0_disable(TC0_t *ptc)
{
// Timer/Counter off
ptc->CTRLA=TC_CLKSEL_OFF_gc;
// Issue a reset command
ptc->CTRLFSET=TC_CMD_RESET_gc;
}
/**************************************/
// Timer/Counter TCC0 initialization
void tcc0_init(void)
{
unsigned char s;
unsigned char n;

// Note: The correct PORTC direction for the Compare Channels
// outputs is configured in the ports_init function.

// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
asm("cli");

// Disable and reset the timer/counter just to be sure
tc0_disable(&TCC0);
// Clock source: ClkPer/1
TCC0.CTRLA=TC_CLKSEL_DIV1_gc;
// Mode: Frequency Waveform Gen., Overflow Int./Event on TOP
// Compare/Capture on channel A: Off
// Compare/Capture on channel B: Off
// Compare/Capture on channel C: Off
// Compare/Capture on channel D: Off
TCC0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (0<<TC0_CCAEN_bp) |
    TC_WGMODE_FRQ_gc;
// Capture event source: None
// Capture event action: None
TCC0.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

// Set Timer/Counter in Normal mode
TCC0.CTRLE=(0<<TC0_BYTEM_bp);

// Overflow interrupt: Disabled
// Error interrupt: Disabled
TCC0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;

// Compare/Capture channel A interrupt: High Level
// Compare/Capture channel B interrupt: Disabled
// Compare/Capture channel C interrupt: Disabled
// Compare/Capture channel D interrupt: Disabled
TCC0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_HI_gc;

// High resolution extension: Off
HIRESC.CTRLA&= ~HIRES_HREN0_bm;

// Advanced Waveform Extension initialization
// Optimize for speed
//#pragma optsize- 
// Disable locking the AWEX configuration registers just to be sure
n=MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
CCP=CCP_IOREG_gc;
MCU.AWEXLOCK=n;
// Restore optimization for size if needed
//#pragma optsize_default

// Pattern generation: Off
// Common waveform channel mode: Off
// Dead time insertion for compare channel A: Off
// Dead time insertion for compare channel B: Off
// Dead time insertion for compare channel C: Off
// Dead time insertion for compare channel D: Off
AWEXC.CTRL=(0<<AWEX_PGM_bp) | (0<<AWEX_CWCM_bp) | (0<<AWEX_DTICCDEN_bp) | (0<<AWEX_DTICCCEN_bp) | 
    (0<<AWEX_DTICCBEN_bp) | (0<<AWEX_DTICCAEN_bp);
// Low side dead time duration [ClkPer cycles]
AWEXC.DTLS=0;
// High side dead time duration [ClkPer cycles]
AWEXC.DTHS=0;
// PORTC output register override
AWEXC.OUTOVEN=0b00000000;

// Fault protection initialization
// Fault detection on OCD Break detection: On
// Fault detection restart mode: Latched Mode
// Fault detection action: None (Fault protection disabled)
AWEXC.FDCTRL=(AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) |
    (0<<AWEX_FDDBD_bp) | (0<<AWEX_FDMODE_bp) | AWEX_FDACT_NONE_gc;
// Fault detect events: 
// Event channel 0: Off
// Event channel 1: Off
// Event channel 2: Off
// Event channel 3: Off
// Event channel 4: Off
// Event channel 5: Off
// Event channel 6: Off
// Event channel 7: Off
AWEXC.FDEMASK=0b00000000;
// Make sure the fault detect flag is cleared
AWEXC.STATUS|=AWEXC.STATUS & AWEX_FDF_bm;

// Clear the interrupt flags
TCC0.INTFLAGS=TCC0.INTFLAGS;
// Set Counter register
TCC0.CNT=0x0000;
// Set Period register
// Not used in Frequency Waveform Generation mode
TCC0.PER=0x0000;
// Set channel A Compare/Capture register
// Controlls the period in Frequency Waveform Generation mode
TCC0.CCA=0xFFFF;
// Set channel B Compare/Capture register
TCC0.CCB=0x0000;
// Set channel C Compare/Capture register
TCC0.CCC=0x0000;
// Set channel D Compare/Capture register
TCC0.CCD=0x0000;

// Restore interrupts enabled/disabled state
SREG=s;
}

// Disable a Timer/Counter type TC1
void tc1_disable(TC1_t *ptc)
{
// Timer/Counter off
ptc->CTRLA=TC_CLKSEL_OFF_gc;
// Issue a reset command
ptc->CTRLFSET=TC_CMD_RESET_gc;
}

// Timer/Counter TCC1 initialization
void tcc1_init(void)
{
unsigned char s;

// Note: The correct PORTC direction for the Compare Channels
// outputs is configured in the ports_init function.

// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
asm("cli");

// Disable and reset the timer/counter just to be sure
tc1_disable(&TCC1);
// Clock source: ClkPer/8
TCC1.CTRLA=TC_CLKSEL_DIV8_gc;
// Mode: Normal Operation, Overflow Int./Event on TOP
// Compare/Capture on channel A: Off
// Compare/Capture on channel B: Off
TCC1.CTRLB=(0<<TC1_CCBEN_bp) | (0<<TC1_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
// Capture event source: None
// Capture event action: None
TCC1.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

// Set Timer/Counter in Normal mode
TCC1.CTRLE=(0<<TC1_BYTEM_bp);

// Overflow interrupt: Medium Level
// Error interrupt: Disabled
TCC1.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_MED_gc;

// Compare/Capture channel A interrupt: Disabled
// Compare/Capture channel B interrupt: Disabled
TCC1.INTCTRLB=TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

// High resolution extension: Off
HIRESC.CTRLA&= ~HIRES_HREN1_bm;

// Clear the interrupt flags
TCC1.INTFLAGS=TCC1.INTFLAGS;
// Set Counter register
TCC1.CNT=0x0000;
// Set Period register
TCC1.PER=0x9C3F;
// Set channel A Compare/Capture register
TCC1.CCA=0x0000;
// Set channel B Compare/Capture register
TCC1.CCB=0x0000;

// Restore interrupts enabled/disabled state
SREG=s;
}

void tcd1_init(void)
{
unsigned char s;

// Note: The correct PORTD direction for the Compare Channels
// outputs is configured in the ports_init function.

// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
asm("cli");

// Disable and reset the timer/counter just to be sure
tc1_disable(&TCD1);
// Clock source: ClkPer/64
TCD1.CTRLA=TC_CLKSEL_DIV64_gc;
// Mode: Normal Operation, Overflow Int./Event on TOP
// Compare/Capture on channel A: Off
// Compare/Capture on channel B: Off
TCD1.CTRLB=(0<<TC1_CCBEN_bp) | (0<<TC1_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
// Capture event source: None
// Capture event action: None
TCD1.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

// Set Timer/Counter in Normal mode
TCD1.CTRLE=(0<<TC1_BYTEM_bp);

// Overflow interrupt: Low Level
// Error interrupt: Disabled
TCD1.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_LO_gc;

// Compare/Capture channel A interrupt: Disabled
// Compare/Capture channel B interrupt: Disabled
TCD1.INTCTRLB=TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

// High resolution extension: Off
HIRESD.CTRLA&= ~HIRES_HREN1_bm;

// Clear the interrupt flags
TCD1.INTFLAGS=TCD1.INTFLAGS;
// Set Counter register
TCD1.CNT=0x0000;
// Set Period register
TCD1.PER=0xC34F;
// Set channel A Compare/Capture register
TCD1.CCA=0x0000;
// Set channel B Compare/Capture register
TCD1.CCB=0x0000;

// Restore interrupts enabled/disabled state
SREG=s;
}