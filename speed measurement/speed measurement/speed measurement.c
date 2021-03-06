/********************************************************************************
 Written by: Vinod Desai,Sachitanand Malewar NEX Robotics Pvt. Ltd.
 Edited by: e-Yantra team
 AVR Studio Version 6

 Date: 19th October 2012


 This experiment demonstrates use of position encoders.

 Concepts covered: External Interrupts, Position control
 
 Microcontroller pins used:
 PORTA3 to PORTA0: Robot direction control
 PL3, PL4: Robot velocity control. Currently set to 1 as PWM is not used
 PE4 (INT4): External interrupt for left motor position encoder 
 PE5 (INT5): External interrupt for the right position encoder

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
    Frequency: 14745600
 	Optimization: -O0  (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2.	It is observed that external interrupts does not work with the optimization level -Os

 3. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
#define F_CPU 14745600
#define SIZE 2
#define DIVIDER 1024
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned long int ShaftCountLeft = 0, timLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0, timRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char data, datal, datar, recv_commands[128] = {0}; //to store received data from UDR1
unsigned int recv_counter = 0;
unsigned long recv_times[128] = {0};
volatile int learn_flag = 0;
volatile int repeat_flag = 0, cend_flag = 0;


void forward_control(unsigned char ref)
{
	unsigned long val[SIZE], var[SIZE];
	double lvel = 0;
	double rvel = 0;
	char change = 0;
	
	for(int i = 0; i < SIZE; i++)
	{
		val[i] = 0;
		var[i] = 0;
	}
	
	velocity (ref, ref); // 195 and 178 gave equal speeds
	forward();
	
	while((cend_flag == 0) || (data == 'w'))
	{
		_delay_ms(50);
		
		val[0] = val[1];
		var[0] = var[1];
		
		cli();
		
		lvel = F_CPU / (timLeft * DIVIDER);
		rvel = F_CPU / (timRight * DIVIDER);
		
		sei();
		
		if(rvel < lvel)
		change++;
		if(rvel > lvel)
		change--;
		
		velocity(ref, ref + change);
		
		val[1] = lvel - (unsigned long) lvel > 0.5 ? ((unsigned long) lvel + 1) : (unsigned long) lvel ;
		var[1] = rvel - (unsigned long) rvel > 0.5 ? ((unsigned long) rvel + 1) : (unsigned long) rvel ;
		
		datal = (unsigned char) (val[1] + val[0]);
		datar = (unsigned char) (var[1] + var[0]);
		
	}
}


//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
/*
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}
*/


ISR(INT5_vect)
{
	timRight = TCNT3;  
	TCNT3 = 0x00;
}


//ISR for left position encoder
/*
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
*/


ISR(INT4_vect)
{
	timLeft = TCNT1;  
	TCNT1 = 0x00;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
		if(ShaftCountLeft > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void timer1_init()
{
	TCCR1B = 0x00;	//Stop
	TCCR1A = 0x00;	// no pwm, etc.
	TCCR1B = 0x05;  //Clk = basic clk / 1024
	
}

void timer3_init()
{
	TCCR3B = 0x00;	
	TCCR3A = 0x00;
	TCCR3B = 0x05;
	
}

void timer4_init()
{
	TCCR4B = 0x00;	
	TCCR4A = 0x00;
}

void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0;	//making copy of data from UDR0 in 'data' variable
	
	if(learn_flag == 1 && data != 1)
	{
		
		recv_commands[recv_counter] = data;
	
		recv_times[recv_counter] = TCNT4;
		
		recv_counter++;
		UDR0 = 'p';
	}
	
	if(data == '1')
	{
		
		learn_flag = 0;	
		UDR0 = 'n';
	}		
		
	if(data == '2')
	{
		
		repeat_flag = 1;
		UDR0 = 'm';
	}	
	
	if(data == 'w') //ASCII value of w
	{
		//PORTA=0x06;  //forward
		forward_control(180);
	}

	if(data == 'x') //ASCII value of x
	{
		PORTA=0x09; //back 
	}

	if(data == 'a') //ASCII value of a
	{
		PORTA=0x05;  //left
	}

	if(data == 'd') //ASCII value of d
	{
		PORTA=0x0A; //right
	}
	
	if(data == 0x6c) //ASCII value of l
	{
		PORTA=0x04;  //left
	}

	if(data == 0x72) //ASCII value of r
	{
		PORTA=0x02; //right
	}
	
	if(data == 's') //ASCII value of s
	{
		PORTA=0x00; //stop
	}


}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	timer5_init();
	uart0_init();
	sei();   // Enables the global interrupt
}

//Main Function
int main(void)
{
	init_devices();
	
	timer1_init();
	timer3_init();
	
	unsigned long t_in = 0;
	unsigned long del_t = 0;
	
	learn_flag = 1;
	TCCR4A = 0x05; // start timer 4 (to store timestamps of commands)
	
	while(repeat_flag != 1);
	
	UDR0 = 'o';
	
	//TCCR4A = 0x00; // stop timer 4. We'll use it here to generate delays
	
	for(int i = 0; i < recv_counter; i++)
	{
		TCCR4A = 0x00;
		
		UDR0 = recv_commands[i];
		
		if(recv_commands[i] == 'w') //ASCII value of w
		{
			//PORTA=0x06;  //forward
			forward_control(180);
		}

		if(recv_commands[i] == 'x') //ASCII value of x
		{
			PORTA=0x09; //back
		}

		if(recv_commands[i] == 'a') //ASCII value of a
		{
			PORTA=0x05;  //left
		}

		if(recv_commands[i] == 'd') //ASCII value of d
		{
			PORTA=0x0A; //right
		}
		
		if(recv_commands[i] == 0x6c) //ASCII value of l
		{
			PORTA=0x04;  //left
		}

		if(recv_commands[i] == 0x72) //ASCII value of r
		{
			PORTA=0x02; //right
		}
		
		if(recv_commands[i] == 's') //ASCII value of s
		{
			PORTA=0x00; //stop
		}
		
		if(recv_times[i] - t_in > 0)
			del_t = recv_times[i] - t_in;
			
		else
			del_t = recv_times[i] - t_in + 0xffff;
			
		t_in = recv_times[i];
		
		TCNT4 = 0x00;
		TCCR4A = 0x05;
		
		while(TCNT4 < del_t);
		
	}
	
}
