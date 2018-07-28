/*
 * GccApplication1.c
 *
 * Created: 10.01.2015 13:23:10
 *  Author: belenon
 */
#ifndef F_CPU
#define F_CPU 16000000
#endif

#define DevESC01  0x60
#define generalCall 1

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include <string.h>
#include "main.h"

volatile uint8_t sectorVolatile = 0; // commutation position
volatile uint16_t msVolatile = 0; // milli seconds
volatile uint16_t secondsVolatile = 0; // mission time
volatile uint16_t pwmPeriod = 500;
volatile uint16_t pwmOn = 35;
volatile uint8_t zeroCrossingFlag = 0;

void commutate(uint8_t sector)
{
    switch (sector)
    {
    case 0:
		// BEGIN - Working A=PWM C=LO B=float -> Sector 1
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;

		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_DDR_PWM;
		PHASE_A_SD_HI;

		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_LO;
		PHASE_C_SD_HI;

		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_HI;
		PHASE_B_SD_LO;

		// END - Working A=PWM C=LO B=float
		ADMUX &= ~(1<<MUX1);
		ADMUX |= (1<<MUX0);
        break;
    case 1:
		// BEGIN - Working B=PWM C=LO A=float -> Sector 2
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;
		
		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_DDR_PWM;
		PHASE_B_SD_HI;

		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_LO;
		PHASE_C_SD_HI;

		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_HI;
		PHASE_A_SD_LO;
		// END - Working B=PWM C=LO A=float

        ADMUX &= ~((1<<MUX0) | (1<<MUX1));
        break;
    case 2:
		// BEGIN - Working B=PWM A=LO C=float -> Sector 3
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;
		
		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_DDR_PWM;
		PHASE_B_SD_HI;

		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_LO;
		PHASE_A_SD_HI;

		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_HI;
		PHASE_C_SD_LO;
		// END - Working B=PWM A=LO C=float
        ADMUX |= (1<<MUX1);
        ADMUX &= ~(1<<MUX0);
        break;
    case 3:
		// BEGIN - Working C=PWM A=LO B=float -> Sector 4
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;
		
		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_DDR_PWM;
		PHASE_C_SD_HI;

		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_LO;
		PHASE_A_SD_HI;

		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_HI;
		PHASE_B_SD_LO;

		// END - Working C=PWM A=LO B=float
        ADMUX &= ~(1<<MUX1);
        ADMUX |= (1<<MUX0);
        break;
    case 4:
		// BEGIN - C=PWM A=float B=LO -> Sector 5
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;
		
		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_DDR_PWM;
		PHASE_C_SD_HI;

		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_LO;
		PHASE_B_SD_HI;

		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_HI;
		PHASE_A_SD_LO;

		// END - C=PWM A=float B=LO
        ADMUX &= ~((1<<MUX0) | (1<<MUX1));
        break;
    case 5:
		// BEGIN - Working A=PWM B=LO C=float -> Sector 6
		PHASE_A_SD_LO;
		PHASE_B_SD_LO;
		PHASE_C_SD_LO;
		
		PHASE_A_IN_DDR_OUTPUT;
		PHASE_A_IN_DDR_PWM;
		PHASE_A_SD_HI;

		PHASE_B_IN_DDR_OUTPUT;
		PHASE_B_IN_LO;
		PHASE_B_SD_HI;

		PHASE_C_IN_DDR_OUTPUT;
		PHASE_C_IN_HI;
		PHASE_C_SD_LO;

		// END - Working A=PWM B=LO C=float
		ADMUX |= (1<<MUX1);
		ADMUX &= ~(1<<MUX0);
        break;
    }
    //PORTD ^= (1<<PD2); // Status LED
	//PORTC ^= (1 << PC3);
}

void hard_commutation(uint16_t maxCount, uint16_t top, uint16_t pulse)
{
	ICR1 = top; // PWM Frequency
	OCR1A = ICR1 - pulse; // Pulse

	uint8_t sector = 0;
	uint16_t counter = 1;
	while(counter != maxCount)
	{
		commutate(sector);
		sector++;
		if (sector > 5)
		{
			sector = 0;
		}
		_delay_us(4000);
		counter++;
	}
	PHASE_A_SD_LO;
	PHASE_B_SD_LO;
	PHASE_C_SD_LO;
}

void start_up(uint16_t top, uint16_t pulse)
{
	int tmp1 = ICR1;
	int tmp2 = OCR1A;
	ICR1 = top; // PWM Frequency
	OCR1A = ICR1 - pulse; // Pulse

	PWM_OUTPUT;
	PWM_ON;
	PHASE_A_SD_DDR_OUTPUT;
	PHASE_B_SD_DDR_OUTPUT;
	PHASE_C_SD_DDR_OUTPUT;

	//commutate(0);
	//_delay_ms(3);
	//commutate(1);
	//_delay_ms(30);
	//commutate(2);
	//_delay_ms(15);
	
	//commutate(3);
	//_delay_ms(50);
	//commutate(4);
	//_delay_ms(10);
	//commutate(5);
	//_delay_ms(7);
	
	uint16_t o = 10;

	for (uint8_t i = 0; i < 50; i++)
	{
		commutate(0);

		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);
		
		commutate(1);
		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);

		commutate(2);
		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);

		commutate(3);
		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);

		commutate(4);
		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);

		commutate(5);
		for (uint8_t q = o; q > 0; q--)
			_delay_ms(1);

		if ((o > 5) && (i % 2))
			o -= 1;
	}

	hard_commutation(100, top, pulse);

	PHASE_A_SD_LO;
	PHASE_B_SD_LO;
	PHASE_C_SD_LO;
	ICR1 = tmp1;
	OCR1A = tmp2;
}

ISR(ANALOG_COMP_vect) // zero crossing occured
{
    zeroCrossingFlag = 1;
}

ISR(TIMER1_COMPA_vect) // this int is called when the pwm signal is toggled
{
	if (PINB & (1<<PB1)) // if pwm is hi
    {
		if ((ACSR & ACIE) == ACIE) // When changing the ACD bit, the Analog Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
		{
			ACSR &= ~(1<<ACIE); // analog comparator interrupt disable
			ACSR &= ~(1<<ACD); // clear analog comparator disable
			ACSR |= (1<<ACIE); // analog comparator interrupt enable
		}
		else
			ACSR &= ~(1<<ACD); // clear analog comparator disable
    }
	else // if pwm is lo
	{
		if ((ACSR & ACIE) == ACIE)
		{
			ACSR &= ~(1<<ACIE); // analog comparator interrupt disable
			ACSR |= (1<<ACD); // set analog comparator disable
			ACSR |= (1<<ACIE); // analog comparator interrupt enable
		}
		else
			ACSR |= (1<<ACD); // set analog comparator disable
	}
}

ISR(TIMER1_OVF_vect) // this int only active when pwm lo
{
	ICR1 = pwmPeriod;
	OCR1A = ICR1 - pwmOn;
	if (zeroCrossingFlag == 1)
	{
	    uint8_t sector = sectorVolatile;
	    commutate(sector);
        sector++;
        if (sector > 5)
        {
            sector = 0;
        }
        zeroCrossingFlag = 0; // clearing zero crossing flag
        sectorVolatile = sector; // updating volatile sector variable
	}
}

ISR(TIMER2_COMPA_vect)
{
    uint16_t ms = msVolatile;
	ms++;
	if (ms == 1000)
	{
		ms = 0;
		secondsVolatile++; // mission time
	}
	msVolatile = ms;
}

ISR(TWI_vect)
{
    switch(TWSR)
    {
        case 0x60: // own SLA W address
            TWCR &= ~((1<<TWSTO) | (1<<TWEA));
            break;
        case 0x70: // general call address W
            TWCR &= ~((1<<TWSTO) | (1<<TWEA));
            break;
        case 0x88: // own address, data received
            pwmOn = TWDR * 4;
            TWCR |= (1<<TWEA);
            break;
        case 0x98: // general call address, data received
            pwmOn = TWDR;
            TWCR |= (1<<TWEA);
            break;
        case 0xA0: // stop signal
            TWCR |= (1<<TWEA);
            break;
        case 0xA8: // SLA R has been received, ACK has been returned
            TWDR = pwmOn/2;
            TWCR &= ~((1<<TWSTO) | (1<<TWEA));
            break;
        case 0xC0: // data has been transmitted; NACK has been received
            TWCR &= ~((1<<TWSTA) | (1<<TWSTO));
            TWCR |= (1<<TWEA);
            break;
        default:
            break;
    }
    TWCR |= (1<<TWINT);
	LED1_ON;
}

int main(void)
{
	// ============== Port initialization ================
	//_delay_ms(500);
    DDRB = 0xff;
    PORTB = 0x00;
    DDRB &= ~((1<<PB0) | (1<<PB2));
	
    DDRC = 0x00;
    PORTC = 0x00;
	DDRC |= (1<< PC3);

    DDRD = 0xff;
    PORTD = 0x00;
    DDRD &= ~((1<<PD7) | (1<<PD6));

    //LED1_ON
    //_delay_ms(100);
    //LED1_OFF
	
	//USART_0_init();
	//_delay_us(100);
	//printf("esc program start!");

    // ================ TWI ====================
    //*
    TWAR = (DevESC01<<1);// + generalCall;
    TWCR &= ((1<<TWSTO) | (1<<TWSTA));
    TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
    TWCR |= ((1<<TWEA) | (1<<TWEN)| (1<<TWINT) | (1<<TWIE));
    //*/

    // ================ analog comparator ===========================
    PRR &= ~(1<<PRADC); // Power reduction disabled for adc
    ADCSRA &= ~(1<<ADEN); // ADC disabled
    ADCSRB |= (1<<ACME); // Analog comparator multiplexer enable
    ACSR |= (1<<ACIS1); // Interrupt on Falling Output Edge
    // ADMUX register must be handled!


    // ================ pwm stuff =====================
    TCCR1A |=  (1<<COM1A1) | (1<<COM1A0); // phase correct pwm, inv mode
    TCCR1B |= (1<<WGM13) | (1<<CS10); // phase correct pwm, inverted mode
    pwmPeriod = 2000;
    ICR1 = pwmPeriod; // PWM Period
    pwmOn = 100;
    OCR1A = ICR1 - pwmOn; // Pulse

    // =============== mission time ================
    //*
    TCCR2A |= (1<<WGM21); // CTC
    TCCR2B |= (1<<CS22);// 64 prescaler
    OCR2A = 249; // 250x64=16000MHz -> 1ms
    TIMSK2 |= (1<<OCIE2A);
    //*/
    // ================ delay =====================
    //for (uint8_t i=0; i<2; i++)
	//{
		//_delay_ms(1000);
	//}

    // =================== start up ================
    uint16_t top = 1000;
    uint16_t pulse = 200;
    start_up(top, pulse); // minpulse not working
    //hard_commutation(100, top, pulse);

    // ================ enable interrupts ===============

    TIMSK1 |= ((1<<OCIE1A) | (1<<TOIE1)); // interrupts for pwm toggle
    ACSR &= ~(1<<ACIE); // analog comparator interrupt disable
    ACSR |= (1<<ACD); // set analog comparator disable
	ACSR |= (1<<ACIE); // analog comparator interrupt enable
    sei();
    // =============== main programm ====================

	uint16_t counter = 0;

	while(1)
	{
		_delay_ms(1);

		counter += 0;
		if (counter > 2000)
		{
			PHASE_A_SD_LO;
			PHASE_B_SD_LO;
			PHASE_C_SD_LO;
			return 0;
		}
	}
}


