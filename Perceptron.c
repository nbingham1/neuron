// *********** Single Perceptron *************
// ** This program is for the Mega 16 and acts as a single perceptron
// ** 


//The program needs to do the following:
// 1) Take in parameters such as weight  (Threshold done, weights not)
// 2) Compute weighted sum of inputs	(done)
// 3) Apply threshold function			(done)
// 4) Update threshold unit marble lights	(done)
// 5) On next clock cycle, update all outputs and do IRL 'fire' animation  (done)

#define F_CPU 1000000UL
#define VCC 5

//__AVR_ATmega16__
//__AVR_ATmega32__

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define mask1(m0) (1 << m0)
#define mask2(m0, m1) ((1 << m0) | (1 << m1))
#define mask3(m0, m1, m2) ((1 << m0) | (1 << m1) | (1 << m2))
#define mask4(m0, m1, m2, m3) ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3))
#define mask5(m0, m1, m2, m3, m4) ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4))
#define mask6(m0, m1, m2, m3, m4, m5) ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5))
#define mask7(m0, m1, m2, m3, m4, m5, m6) ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5) | (1 << m6))
#define mask8(m0, m1, m2, m3, m4, m5, m6, m7) ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5) | (1 << m6) | (1 << m7))

#define unmask1(m0) (0xFF ^ (1 << m0))
#define unmask2(m0, m1) (0xFF ^ ((1 << m0) | (1 << m1)))
#define unmask3(m0, m1, m2) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2)))
#define unmask4(m0, m1, m2, m3) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3)))
#define unmask5(m0, m1, m2, m3, m4) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4)))
#define unmask6(m0, m1, m2, m3, m4, m5) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5)))
#define unmask7(m0, m1, m2, m3, m4, m5, m6) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5) | (1 << m6)))
#define unmask8(m0, m1, m2, m3, m4, m5, m6, m7) (0xFF ^ ((1 << m0) | (1 << m1) | (1 << m2) | (1 << m3) | (1 << m4) | (1 << m5) | (1 << m6) | (1 << m7)))

//Pinout defines:

#define WEIGHTPIN PINA
#define WEIGHTDDR DDRA
#define WEIGHT0NUM 0
#define WEIGHT1NUM 1
#define WEIGHT2NUM 2
#define WEIGHT3NUM 3

#define READENPORT PORTA
#define READENDDR DDRA
#define READENNUM 4

#define BUTTONFIREPIN PINA
#define BUTTONFIREDDR DDRA
#define BUTTONFIRENUM 5

#define INPUTPIN PINB
#define INPUTDDR DDRB

#define THRESHPIN PINC
#define THRESHPORT PORTC
#define THRESHDDR DDRC

#define ANIMATIONPORT PORTD
#define ANIMATIONDDR DDRD
#define SRR0NUM 5
#define SRR1NUM 4
#define SRR2NUM 3
#define SRR3NUM 2

#define CLOCKPORT PORTD
#define CLOCKDDR DDRD
#define CLOCKNUM 6

#define BUZZERPORT PORTD
#define BUZZERDDR DDRD
#define BUZZERNUM 7

#define ANIMFLAG 0
#define PAUSEFLAG 1
#define PAUSEFLAG1 2
#define INPUTFLAG 3

// State Variables
#define _pgmAccum_ 0
#define _pgmFire_  1

#define TIMEBETWEENANIMATION 50	// milliseconds
#define THRESHOLDPAUSELENGTH 5
#define TIMEBETWEENTHRESHOLD 200
#define TIMEBETWEENWEIGHTS	 200
volatile uint8_t programStatus = _pgmAccum_;

// Perceptron Variables
volatile uint8_t value = 0;
uint8_t threshold = 0;
uint8_t raw_threshold = 0xff;
uint8_t weight[4] = {0, 0, 0, 0};
uint8_t weight_to_anim_LED[4] = {0,0,0,0};
volatile uint16_t inputCount[8] = {0,0,0,0,0,0,0,0};		// High 12 bits represent the number of 1ms ticks between TIMEBETWEENANIMATION detections
												// Low 4 bit represent the number of TIMEBETWEENANIMATION detections

uint8_t animation_step = 0;
volatile uint8_t  flags = 0; //MAKE THIS GO HIGH WHEN THE ANIMATION CLOCK (PORTD6) GOES FROM HIGH TO LOW. I'll set it low again.
volatile uint16_t nextAnimTime = 0;
volatile uint8_t  pauseTime = 0;
volatile uint8_t  pauseTime1 = 0;

volatile uint8_t raw_input = 0;

uint8_t iter = 0;
volatile uint8_t isr_iter = 0;

volatile uint8_t buzzeriter = 0;
volatile uint8_t buzzermax = 0;


ISR (TIMER1_COMPA_vect)
{
	raw_input = INPUTPIN;

	//Cycle through all 8 inputs. Each time shift raw_input by one so it loses the LSB and the i'th input is on &0x01
	for (isr_iter = 0; isr_iter < 8; isr_iter++)   
	{
		//On the condition that the upper 12' of inputCount[i] have counted to higher than TIMEBETWEENANIMATIONS, we want to reset the
		//top 12 bits and increment the bottom four. Remember that the top 12' are just for counting ms and the bottom four are for keeping
		//track of how many times we have reached TIMEBETWEENANIMATIONS. If we haven't reached TIMEBETWEENANIMATIONS yet, increment the upper 12'
		// iff the current raw_input is high. This will stop the counting if the input goes low. 
		inputCount[isr_iter] = (((inputCount[isr_iter] & 0xfff0) > TIMEBETWEENANIMATION*16) ? (inputCount[isr_iter]+1) & 0x000f : inputCount[isr_iter]) + (raw_input & 0x01)*16;

		//If the input on the line we are currently looking at goes low and it has been high at least one ms, increment the value by the number
		//of animation cycles it was high minus four (pre-loading the shift registers)
		//Clear count
		if ((raw_input & 0x01) == 0 && inputCount[isr_iter] != 0)
		{
			value += (inputCount[isr_iter] & 0x000f) - 4;
			inputCount[isr_iter] = 0;
		}

		raw_input>>=1;
	}
	
	if (programStatus == _pgmFire_ && ++buzzeriter > buzzermax)
	{
		BUZZERPORT		 ^= mask1(BUZZERNUM); // Debugging	
		buzzeriter = 0;
	}


	if (++nextAnimTime > TIMEBETWEENANIMATION)
	{
		nextAnimTime	  = 0; 
		if (programStatus == _pgmFire_)
		{
			flags			 |= mask1(ANIMFLAG);
			CLOCKPORT		 &= unmask1(CLOCKNUM); //Get ready to shift again... 
		}
	}
	if ((flags & mask1(PAUSEFLAG)) > 0 && pauseTime-- <= 0)
		flags &= unmask1(PAUSEFLAG);
	if ((flags & mask1(PAUSEFLAG1)) > 0 && pauseTime1-- <= 0)
		flags &= unmask1(PAUSEFLAG1);
}

void init(void)
{
//The program will have the following pins (new): 
// 4 pin inputs for weight sensors for all of the axon's ouputs (PINA0-3)
// 1 pin of input that is a button you push to 'force a fire'	(PORTA5)
// 2 free pins. Yay! (PORTA6-7)
// 8 pins of input. This will be a single wire per input into the axon (PINB)
// 9 pins reserved for threshold. Both IO. (8 pins on PORTC, 1 on PINA4)
// 2 pins dedicated for UART (save functionality just in case (PORTD0-1)
// 5 pins of output for animations. Four for SRR on each shift register, one for a clock pin (PORTD2-6)
// 1 pin on output for buzzer(PORTD7)

	DDRA = 0x00;
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;

	// I/O Port Setup
	WEIGHTDDR		&= unmask4(WEIGHT0NUM,WEIGHT1NUM,WEIGHT2NUM,WEIGHT3NUM);	//4 weight sensors 
	READENDDR		|= mask1(READENNUM);
	ANIMATIONDDR	|= mask4(SRR0NUM,SRR1NUM,SRR2NUM,SRR3NUM);
	BUTTONFIREDDR	&= unmask1(BUTTONFIRENUM);
	CLOCKDDR		|= mask1(CLOCKNUM);
	BUZZERDDR		|= mask1(BUZZERNUM);
	INPUTDDR		 = 0x00;		// 8 inputs on port A (inputs into dendrite)
	THRESHDDR		 = 0x00; //MAKE SURE TO FUCKING UPDATE THESE WHEN SETTING THRESHOLD 

	//NOTE!!!! NEVER SET PORTC = 1 EVER!!! ESP IF CS IS ON. BUT EVER EVER EVER. 
	THRESHPORT = 0x00;
	
	// 1ms Timer Setup
	TIMSK = mask1(OCIE1A);     		// 8'b00000010 enable the timer 0 compare match a isr
	OCR1A  = 1000; 			      		 //            compare match after 250 ticks
	TCCR1A = 0;     			// 8'b00000010 turn on clear-on-match
	TCCR1B = mask2(CS10, WGM12); 		// 8'b00000011 set clock prescalar to 64
	
	ADMUX = mask1(ADLAR);
	ADCSRA = mask1(ADEN);


	#ifdef DEBUG
	uart_init();							// Initialize serial communication for
	stdout = stdin = stderr = &uarthdl;		// debug messages through PortD.0 and PortD.1
	#endif
		 
	sei();									// Enable interrupts
} //init

//This function will cycle through all the thresholds whenever called and will update the threshold 
//Note that 8 pins are on PORTC (read pins) and a ninth is on PINB4 (CS, Read enable)
//NOTE!!!! NEVER SET PORTC = 1 EVER!!! ESP IF CS IS ON. BUT EVER EVER EVER. 
void detectNumThresholds()
{
	uint8_t oldthresh = THRESHDDR;
	uint8_t temp_thresh = 0;

	if ((flags & mask1(PAUSEFLAG)) == 0)
	{
		THRESHDDR = 0x00;				//Change THRESHDDR so we can read
		READENPORT &= unmask1(READENNUM);	//Set CS low so we can read
	
		pauseTime = THRESHOLDPAUSELENGTH;
		flags |= mask1(PAUSEFLAG);
		while ((flags & mask1(PAUSEFLAG)) > 0);

		//PINC should now have either 1s (marbles are in the slots) or 0s (marbles not in the slot) for each bit. 
		//Sum the bits on THRESHPIN and update threshold
		threshold = 0;
		temp_thresh = THRESHPIN;
		raw_threshold = temp_thresh;

		/*threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;*/

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;
		temp_thresh>>=1;

		threshold += temp_thresh&0x01;

		READENPORT |= mask1(READENNUM);
		THRESHDDR = oldthresh;

		pauseTime = TIMEBETWEENTHRESHOLD;
		flags |= mask1(PAUSEFLAG);
	}
}

// This displays the current charge of the neuron through the threshold marbles
void displayThresholds() 
{
	/*uint8_t bitmask = raw_threshold;
	uint8_t set = 0;
	uint8_t bit = 0;
	
	while (set < (threshold - (value>>2)) && bit < 8)
	{
		set += ((bitmask & 0x80) > 0);
		bitmask <<= 1;
		bit++;
	}

	bitmask >>= bit;

	THRESHDDR = bitmask;*/

	uint8_t bitmask = 0;
	uint8_t bit = 0;
	
	while (bit < threshold)
		bitmask |= mask1(bit++);

	THRESHDDR = bitmask;
}

// This function will cycle through all of the weights whenever called and will update the weights
// Uses the ADC to grab values from voltage dividers
void detectWeights()
{
	if ((flags & mask1(PAUSEFLAG1)) == 0)
	{
		ADMUX &= 0xe0;
		ADMUX |= WEIGHT0NUM;
		ADCSRA |= mask1(ADSC);
		while ((ADCSRA & mask1(ADSC)) > 0);
		weight[0] = ((uint16_t)ADCH + 32)>>6;

		ADMUX &= 0xe0;
		ADMUX |= WEIGHT1NUM;
		ADCSRA |= mask1(ADSC);
		while ((ADCSRA & mask1(ADSC)) > 0);
		weight[1] = ((uint16_t)ADCH + 32)>>6;

		ADMUX &= 0xe0;
		ADMUX |= WEIGHT2NUM;
		ADCSRA |= mask1(ADSC);
		while ((ADCSRA & mask1(ADSC)) > 0);
		weight[2] = ((uint16_t)ADCH + 32)>>6;

		ADMUX &= 0xe0;
		ADMUX |= WEIGHT3NUM;
		ADCSRA |= mask1(ADSC);
		while ((ADCSRA & mask1(ADSC)) > 0);
		weight[3] = ((uint16_t)ADCH + 32)>>6;

		pauseTime1 = TIMEBETWEENWEIGHTS;
		flags |= mask1(PAUSEFLAG1);
	}
}

// This function will shift out the outputs to be sent.
// 5 pins of output for animations. Four for SRR on each shift register, one for a clock pin (PORTD2-6)
// Let PORTD6 be clock, PORTD2 be the leftmost output line (controlling the first four LEDs and output 0) and PORTD3-5 be out 1-3
void executeAnimation()
{
	if ((flags & mask1(ANIMFLAG)) > 0)
	{
		animation_step++;
		flags &= unmask1(ANIMFLAG);
		if ((animation_step < 4))
			ANIMATIONPORT |= mask4(SRR0NUM, SRR1NUM, SRR2NUM, SRR3NUM);
		else
		{
			if (weight_to_anim_LED[0] > 0)
			{	//If there are any LEDs left in the queue? #0
				ANIMATIONPORT |= mask1(SRR0NUM); //Send one next time clock goes high!
				weight_to_anim_LED[0]--;
				
			}
			else
				ANIMATIONPORT &= unmask1(SRR0NUM); //Don't send one. :(
			
			if (weight_to_anim_LED[1] > 0)
			{	//If there are any LEDs left in the queue? #1
				ANIMATIONPORT |= mask1(SRR1NUM); //Send one next time clock goes high!
				weight_to_anim_LED[1]--;
				
			}
			else
				ANIMATIONPORT &= unmask1(SRR1NUM); //Don't send one. :(
			
			if (weight_to_anim_LED[2] > 0)
			{	//If there are any LEDs left in the queue? #2
				ANIMATIONPORT |= mask1(SRR2NUM); //Send one next time clock goes high!
				weight_to_anim_LED[2]--;
				
			}
			else
				ANIMATIONPORT &= unmask1(SRR2NUM); //Don't send one. :(
			
			if (weight_to_anim_LED[3] > 0)
			{	//If there are any LEDs left in the queue? #3
				ANIMATIONPORT |= mask1(SRR3NUM); //Send one next time clock goes high!
				weight_to_anim_LED[3]--;
			}
			else
				ANIMATIONPORT &= unmask1(SRR3NUM); //Don't send one. :(
		}
		
		if (animation_step < 12)
			CLOCKPORT |= mask1(CLOCKNUM); //SHIFT THAT SHIT!!		
	}
}

int main()
{
	init();
	
	while(1)
	{
		// Read threshold value
		detectWeights();
		detectNumThresholds();
		displayThresholds();

		switch (programStatus)
		{
			case _pgmAccum_:
				animation_step = 0;

				if ((BUTTONFIREPIN & mask1(BUTTONFIRENUM)) > 0 || value > threshold/**4*/)
				{
					weight_to_anim_LED[0] = weight[0];
					weight_to_anim_LED[1] = weight[1];
					weight_to_anim_LED[2] = weight[2];
					weight_to_anim_LED[3] = weight[3];
					buzzermax = 4 - ((weight[0] + weight[1] + weight[2] + weight[3])>>2);
					programStatus = _pgmFire_;
				}
				break;
			case _pgmFire_:
				value = 0; //zero out the current value
				
				executeAnimation(animation_step);
				
				if (animation_step > 12)
				{
					ANIMATIONPORT &= unmask4(SRR0NUM,SRR1NUM,SRR2NUM,SRR3NUM);
					programStatus = _pgmAccum_;
				}
				break;
			default:
				programStatus = _pgmAccum_;
				break;
		}
		
	}
	
	return 0;
}
