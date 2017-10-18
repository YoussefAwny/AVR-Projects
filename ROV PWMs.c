/*
 * ROV Final.c
 *
 * Created: 11/21/2016 1:17:29 PM
 * Author : youssefawny
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>





volatile int rxIndex =0;
unsigned char received[16];
void i2c_Init(uint8_t address)
{
	// load address into TWI address register
	TWAR = (address<<1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}



volatile uint16_t encoderCounter=0;
volatile uint16_t encoderCompareIndex=0;
void external_Interrupt(void)//encoder
{
	EICRA |= (1<<ISC11) | (1<<ISC10) ;
	EIMSK |= (1<<INT1);
	DDRD  |= (1<<5) | (1<<2) | (1<<1);
	PORTD |= (1<<5);
}



unsigned char counter =0;
unsigned int pwm[]={630,1000,1500,1750,2000,2250,2100}; //7 pwm int values for 6 brushless and servo needing char array of size 14
unsigned int pinIdentifingTick[]={532,3314,6096,8878,11660,14442,17224};
unsigned char pwmIndex =0;
unsigned int currentPwm = 0;
unsigned int currentPin = 532;

int main(void)
{ 
   	TCCR1A = 0x00; //setting the timer count mode as
   	TCCR1B = 0x19;
   	TIMSK1 = 0x07;
   	ICR1   = 19999;
   	DDRB  |= 0b01111111;
   	PORTB  = 0;
   	OCR1A  = currentPin;

    i2c_Init(3);
	external_Interrupt();
	sei();
	while (1)
	{

	}
	return 0 ;

}



ISR(TWI_vect){
   if ((TWSR & 0xF8) == TW_ST_SLA_ACK)
    {
    TWDR= "H";
	TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    }
	else if ((TWSR & 0xF8) == TW_ST_DATA_ACK)
	{ 
	 TWDR= "i";
     TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
	 TWCR &= ~(1<<TWEA);
	}
 	
	else if( (TWSR & 0xF8) == TW_SR_SLA_ACK )// own address has been acknowledged
	{
		rxIndex = 0;
		
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);// clear TWI interrupt flag, prepare to receive next byte and acknowledge
	}
	else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ) 	    // data has been received in slave receiver mode
	
	{

	    received[rxIndex++]=TWDR;
		pwm[0]=((received[0]<<8) | received[1]);
		pwm[1]=((received[2]<<8) | received[3]);
		pwm[2]=((received[4]<<8) | received[5]);
		pwm[3]=((received[6]<<8) | received[7]);
		pwm[4]=((received[8]<<8) | received[9]);
		pwm[5]=((received[10]<<8) | received[11]);
		pwm[6]=((received[12]<<8) | received[13]);
		encoderCompareIndex=(received[14]<<8 | received[15]);
		if (encoderCompareIndex != encoderCounter)
		{
		 PORTD |=  (1<<1);
		 PORTD &= ~(1<<2);
		}
	   	
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	}
	else if((TWSR & 0xF8)==TW_SR_STOP)
	{
		TWCR |= (1<<TWINT) | (1<<TWEA);
		TWCR &= ~( (1<<TWSTA) | (1<<TWSTO) );

	}
}
ISR(TIMER1_COMPA_vect)
{
	counter++;
	if (counter % 2 != 0)
	{
		pwmIndex = counter / 2;
		PORTB |= 1 << pwmIndex;
		currentPwm = pwm[pwmIndex] + pinIdentifingTick[pwmIndex];
		OCR1A = currentPwm;
	}
	else
	{
		PORTB = 0;
		if (counter != 14) //double the number of PWMs 
		{
			currentPin = pinIdentifingTick[pwmIndex+1];
			OCR1A = currentPin;
		}
	}
}

ISR(TIMER1_OVF_vect)
{
	counter =0;
	OCR1A = pinIdentifingTick[0];
	pwmIndex =0;
}

ISR(INT1_vect)
{
	encoderCounter++;
	if (encoderCounter==encoderCompareIndex)
	{
		PORTD |= (1<<2) | (1<<1) ;
		encoderCounter=0;
	}
	else 
	{
	   PORTD &= ~(1<<2);
	   PORTD |=  (1<<1);
	}

	
}