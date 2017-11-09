#include "compiler_abstraction.h"
#include "main.h"
#include "nrf.h"
#include "./src/headers/message_table.h"
#include "./src/headers/network_characteristics.h"
#include <stdint.h>


#define MS_DELAY_1000	31250


uint8_t *tx_data_pointer;
uint8_t tx_ready_flag = 0;

extern uint8_t g_Beacon_RX;

struct __attribute__((packed)) MSG_BEACON {
	
	uint8_t S0;
	uint8_t Length;
	uint8_t S1;
	uint8_t MAC_ADDR[6];
	uint8_t iBeacon_prefix[9];
	uint8_t UUID[16];
	uint8_t Major[2];
	uint8_t Minor[2];
	uint8_t TXpower;	
	
};

void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
		//NRF_CLOCK->LFCLKSRC =0;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


void setup_timer()
{
	NRF_TIMER0->TASKS_STOP = 1;													//stop the timer to configure it
	NRF_TIMER0->PRESCALER = 9;													//set timer prescaler to 2^9 (16MHz/512 = 31250) for 32us ticks
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;						//set module to timer mode
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;	//make it a 24-bit timer so max = 536.87s
	NRF_TIMER0->CC[0] = MS_DELAY_1000;											//The period for desync algorithm
	NRF_TIMER0->CC[1] = 0;															//clear the RX capture time register
	NRF_TIMER0->INTENSET = 0x00010000;									//enable interrupt for compare0
	NVIC_EnableIRQ(TIMER0_IRQn);												//enable interrupt in NVIC
	NRF_TIMER0->TASKS_CLEAR = 1;												//clear the timer to 0 counts
	NRF_TIMER0->TASKS_START = 1;												//start the timer
}


void TIMER0_IRQHandler(void)
{
	NRF_TIMER0->TASKS_CLEAR = 1;												//clear the timer to 0 counts
	if(g_Beacon_RX == 1)
	{
		NRF_GPIO->OUTCLR= (1<< ORNG_LED);
	}
	else
	{
		NRF_GPIO->OUTSET= (1<< ORNG_LED);
	}
	g_Beacon_RX = 0;
  NRF_TIMER0->CC[0] = MS_DELAY_1000;											//set the next TX period
	
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;									//clear the compare flag
	
	NVIC_ClearPendingIRQ(TIMER0_IRQn);									//clear the interrupt in NVIC
}


int main (void)
{
		clock_initialization();  	//setup clocks 
		setup_timer();						//sets up timer0 for transmit/receive intervals
		setup_gpio();							//sets up all gpio pins
		hal_radio_config();				//configure the radio parameters
		hal_radio_start_rx();			//start receive mode
		while(1)
		{			

		}
}

