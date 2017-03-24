/*
* UART+BT.c
*
* Created: 3/6/2017 4:14:32 PM
* Author : youssefawny
*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

void uart_init();
void i2c_init(uint8_t address);


volatile char ID[7];
volatile uint8_t Char_Counter=0;
volatile int i=0;

int main(void)
{
	sei();
	uart_init();
	//i2c_init(3);
	while (1)
	{
		
	}

}

ISR(USART_RX_vect)
{
	ID[Char_Counter]=UDR0;

	if (ID[Char_Counter]==7)
	{
		Char_Counter=0;
	}
}



/*ISR(TWI_vect){
	if ((TWSR & 0xF8) == TW_ST_SLA_ACK)
	{
		TWDR= '*';
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	}
	else if ((TWSR & 0xF8) == TW_ST_DATA_ACK)
	{
		TWDR= ID[i++];
		if (i==7)
		{
			i=0;
		}
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
		TWCR &= ~(1<<TWEA);
	}
}

*/

void uart_init()
{
UBRR0H = 0;
UBRR0L = 103;
UCSR0B |= (1<<RXEN0) |/* (1<<TXEN0) |*/ (1<<RXCIE0);
UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
}

void i2c_init(uint8_t address)
{
// load address into TWI address register
TWAR = (address<<1);
// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}


