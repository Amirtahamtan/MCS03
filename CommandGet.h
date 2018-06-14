#include <avr/interrupt.h>

char BufferSend[100];
int indexBS=0;
char BufferReceive[100];
char EXECMD[100];
int indexBR=0;
char Responsing = 0;
char cmdIsReady = 0;
int h;

// SPIF initialization
// Raspberry Communication port
void spif_init(void)
{
	// SPIF is enabled
	// SPI mode: 0
	// Operating as: Slave
	// Data order: MSB First
	SPIF.CTRL=SPI_ENABLE_bm | SPI_MODE_0_gc | (0<<SPI_MASTER_bp) | (0<<SPI_DORD_bp);

	// SPIF interrupt: High Level
	SPIF.INTCTRL=SPI_INTLVL_HI_gc;

	// Note: The MISO (PORTF Pin 6) signal is
	// configured as output in the ports_init function.
}

// SPIF interrupt service routine
ISR (SPIF_INT_vect)
{
	// SPIF status
	unsigned char status;
	// Received data
	unsigned char rx_data;
	// New data to be transmitted
	unsigned char tx_data;
	// Get SPIF status
	status=SPIF.STATUS;
	// Get received data
	rx_data=SPIF.DATA;
	tx_data = ' ';
	if(Responsing==0)
	{
		if(rx_data!='\r')
		{
			if (rx_data != 0)
			{
				if(indexBR<100)
				{
					BufferReceive[indexBR]=rx_data;
					indexBR++;
				}
			}
		}
		else
		{
			for (h = 0;h < 100;h++)
			{
				EXECMD[h] = BufferReceive[h];
				BufferReceive[h] = 0;
			}
			
			cmdIsReady=1;
			for(h=0;h<100;h++)
				BufferReceive[h]=0;
			indexBR=0;
		}
	}
	else
	{
		if(indexBS<100)
			tx_data=BufferSend[indexBS];
		indexBS++;
		if(indexBS==101)
		{
			ClrDataReady;
			Responsing=0;
			indexBS=0;
		}
	}
	SPIF.DATA=tx_data;
	// Place your code here
}
