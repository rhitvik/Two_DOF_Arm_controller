/*
 * hand_project_simulation.cpp
 *
 * Created: 04-Apr-18 5:48:13 PM
 *  Author: RHITVIK
 */ 


/*
 * coinAcceptor_innterruptProgram.cpp
 *
 * Created: 26-Jul-16 7:34:29 PM
 *  Author: RHITVIK
 */ 

// #include <avr/io.h>
// 
// int main(void)
// {
// 	GICR |= (1<<INT1);
// 	MCUCR |= (1<<ISC10) | (1<<ISC11);
// 	
//     while(1)
//     {
//         //TODO:: Please write your application code 
//     }
// }

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define unlock		   0
#define lock           1

#define LightSwitch    7
#define ReadWrite      6
#define BiPolarMood    5

#define MrLCDsCrib            PORTB
#define DataDir_MrLCDsCrib	  DDRB

#define MrLCDsControl		  PORTC
#define DataDir_MrLCDsControl DDRC

void initialize_ADC(void);
void Kill_ADC(void);

void accept_coin(void);
void return_coin(void);
void initialise_PWM(void);
void initialise_PWM_1(void);
void initialise_PWM_2(void);
void stop_PWM_1(void);
void stop_PWM_2(void);
void Motor_1_Direction_Clockwise(void);
void Motor_1_Direction_Anticlockwise(void);
void Motor_1_Switch_OFF(void);
void Motor_2_Direction_Clockwise(void);
void Motor_2_Direction_Anticlockwise(void);
void Motor_2_Switch_OFF(void);

void initialise_counter(void);
void stop_counter(void);

void sw_debounce_pressed(void);
void sw_debounce_released(void);

void Check_IF_MrLCD_isBusy(void);
void Peek_A_Boo(void);
void Send_A_Command(char command);
void Send_A_Character(char character);
void Send_An_Integer(int integer);
void Send_A_String(char *stringOfCharacters);
void Send_A_String_inst(char *stringOfCharacters);

void Assign_motor_direction (void);
void converted_value_absolution(void);
void converted_value_preobfuscation (void);
void convert_PWM_value(void);
void Segregate_ADC_Vectors (void);

static volatile uint8_t count=0;  //for interrupt generation
static volatile uint8_t count1=0; //10 sec delay initializer
static volatile uint8_t FAIL = 0;

int pressed=0;
int pressed_1=0;
int pressed_2=0;

int metal_touch=0;
int dispatched=0;
int successful=0;
int aagaya=0;

int refresh = 0;
int refresh2 = 0;
volatile int a = 0;

int converted_value_1 = 0;
int converted_value_2 = 0;

int converted_absolute_value_1 = 0;
int converted_absolute_value_2 = 0;

int PWM_value_1 = 0;
int PWM_value_2 = 0;

bool channel_1_PWM_Lock = unlock;
bool channel_2_PWM_Lock = unlock;

volatile int ADC_Channnel = 0;
volatile int Channel_1_ADC_value = 0;
volatile int Channel_2_ADC_value = 0;

int main(void)
{
	MCUCSR |= (1<<JTD);
	MCUCSR |= (1<<JTD);
	
	DDRD = 0xFF;
	
	DataDir_MrLCDsControl |= (1<<LightSwitch) | (1<<ReadWrite) | (1<<BiPolarMood);	
	
	PORTD = 0x00;

	sei();
	
	_delay_ms(15);
	Send_A_Command(0x01);//clrscr();
	_delay_ms(2);
	Send_A_Command(0x38);
	_delay_us(50);
	Send_A_Command(0b00001100);
	_delay_us(50);
	
	char ValueSampled_channel_1[4];
	char Reduced_Value_channel_1[4];
	
	char ValueSampled_channel_2[4];
	char Reduced_Value_channel_2[4];	

	char OCR1A_value[6];
	char OCR1B_value[6];
	
	Send_A_Command(0x80);//clrscr();
	_delay_ms(2);
	Send_A_String("X");
	_delay_ms(2);
	
	Send_A_Command(0x80 + 10);//clrscr();
	_delay_ms(2);
	Send_A_String("Y");
	_delay_ms(2);
	
		Send_A_Command(0x94);//clrscr();
		_delay_ms(2);
		Send_A_String("A");
		_delay_ms(2);
		
		Send_A_Command(0x94+10);//clrscr();
		_delay_ms(2);
		Send_A_String("B");
		_delay_ms(2);
		
		Send_A_Command(0xD4);//clrscr();
		_delay_ms(2);
		Send_A_String("1A");
		_delay_ms(2);
		
		Send_A_Command(0xD4+10);//clrscr();
		_delay_ms(2);
		Send_A_String("1B");
		_delay_ms(2);
// 	
	initialize_ADC();
	initialise_PWM();
	
// 	Motor_1_Direction_Clockwise();
// 	Motor_2_Direction_Clockwise();
// 	while (2)
// 	{
// 		PORTA |= (1<<PINA4);
// 		_delay_ms(1000);
// 		PORTA &=~ (1<<PINA4);
// 		_delay_ms(1000);
// 	}
	
// 	while (2)
// 	{
// 		Send_A_Command(0x01);//clrscr();
// 		_delay_ms(2);		
// 		Send_A_String_inst("rhitvik");
// 		_delay_ms(1000);
// 		Send_A_Command(0x01);//clrscr();
// 		_delay_ms(2);
// 		Send_A_String_inst("kumawat");
// 		_delay_ms(1000);
// 	}	
	while (1)
	{	
		_delay_ms(20);		
		ADCSRA |= (1<<ADSC);
		_delay_ms(20);
			
		if (ADC_Channnel == 1)
		{
			itoa(Channel_1_ADC_value,ValueSampled_channel_1, 10);
			
			Send_A_Command(0x80 + 2);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0x80 + 2);//();
			_delay_ms(2);
			Send_A_String_inst(ValueSampled_channel_1);
			_delay_ms(2);
			
			//_delay_ms(20);
			Segregate_ADC_Vectors();
			_delay_ms(10);
			converted_value_preobfuscation();
			_delay_ms(10);
			Assign_motor_direction();
					
			itoa(converted_value_1,Reduced_Value_channel_1,10);
			
			Send_A_Command(0x94 + 2);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0x94 + 2);//();
			_delay_ms(2);
			Send_A_String_inst(Reduced_Value_channel_1);
			_delay_ms(2);
						
			_delay_ms(20);
			converted_value_absolution();
			convert_PWM_value();
			
			itoa(PWM_value_1,OCR1A_value,10);
			
			Send_A_Command(0xD4 + 3);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0xD4  + 3);//();
			_delay_ms(2);
			Send_A_String_inst(OCR1A_value);
			_delay_ms(2);
			
			initialise_PWM_1();	
		}	
		else
		if (ADC_Channnel == 2)
		{
			itoa(Channel_2_ADC_value,ValueSampled_channel_2, 10);
			
			Send_A_Command(0x80+ 10 + 2);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0x80+ 10 + 2);//();
			_delay_ms(2);
			Send_A_String_inst(ValueSampled_channel_2);
			_delay_ms(2);

			// _delay_ms(20);
			Segregate_ADC_Vectors();
			_delay_ms(10);
			converted_value_preobfuscation();
			_delay_ms(10);
			Assign_motor_direction();

			itoa(converted_value_2,Reduced_Value_channel_2,10);

			Send_A_Command(0x94+ 10 + 2);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0x94+ 10 + 2);//();
			_delay_ms(2);
			Send_A_String_inst(Reduced_Value_channel_2);
			_delay_ms(2);

			_delay_ms(20);
			converted_value_absolution();
			convert_PWM_value();

			itoa(PWM_value_2,OCR1B_value,10);

			Send_A_Command(0xD4+ 10 + 3);//();
			_delay_ms(2);
			Send_A_String("     ");
			_delay_ms(2);
			Send_A_Command(0xD4+ 10 + 3);//();
			_delay_ms(2);
			Send_A_String_inst(OCR1B_value);
			_delay_ms(2);

			initialise_PWM_2();
		}			
	}				
}	

void initialize_ADC(void)
{
	ADCSRA |= (1<<ADPS2) | (1<<ADIE);
	ADMUX  |= (1<<ADLAR) |(1<<REFS0);
 	ADCSRA |= (1<<ADEN);
 	ADCSRA |= (1<<ADSC);
}
void initialise_PWM(void)
{
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<COM1B1);
	ICR1=19999; //top value
}
void initialise_PWM_1(void)
{	
	OCR1A =	PWM_value_1;
}
void initialise_PWM_2(void)
{
	OCR1B = PWM_value_2;
}
void stop_PWM_1(void)
{
	//TCCR1A &=~ (1<<COM1A1);
	OCR1A = 0;
}
void stop_PWM_2(void)
{
	//TCCR1A &=~ (1<<COM1B1);
	OCR1B = 0;
}
void Motor_1_Direction_Clockwise(void)
{	
	_delay_ms(5);
	PORTD |= (1<<PIND6);
	PORTD &=~ (1<<PIND7);
	_delay_ms(5);
	/////////////
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("         ");
	_delay_ms(2);
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("SINISTRAL");
	
}
void Motor_1_Direction_Anticlockwise(void)
{
	//////////////
	_delay_ms(5);
	PORTD |= (1<<PIND7);
	PORTD &=~ (1<<PIND6);
	_delay_ms(5);
	
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("         ");
	_delay_ms(2);
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("DEXTRAL");
}
void Motor_1_Switch_OFF(void)
{
	//////////////
	_delay_ms(5);
	PORTD &=~ (1<<PIND6);
	PORTD &=~ (1<<PIND7);
	PORTD &=~ (1<<PIND5);
	_delay_ms(5);
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("         ");
	_delay_ms(2);
	Send_A_Command(0xC0);//();
	_delay_ms(2);
	Send_A_String("NEUTRAL");
}
void Motor_2_Direction_Clockwise(void)
{
	_delay_ms(5);
	PORTD |= (1<<PIND0);
	PORTD &=~ (1<<PIND1);
	_delay_ms(5);
	/////////////
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("          ");
	_delay_ms(2);
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("OPEN CLAW");
}
void Motor_2_Direction_Anticlockwise(void)
{
	_delay_ms(5);
	PORTD |= (1<<PIND1);
	PORTD &=~ (1<<PIND0);
	_delay_ms(5);
	////////////
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("          ");
	_delay_ms(2);
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("CLOSE CLAW");
}
void Motor_2_Switch_OFF(void)
{
	//////////////
	_delay_ms(5);
	PORTD &=~ (1<<PIND0);
	PORTD &=~ (1<<PIND1);
	PORTD &=~ (1<<PIND4);
	_delay_ms(5);
	
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("          ");
	_delay_ms(2);
	Send_A_Command(0xC0 + 10);//();
	_delay_ms(2);
	Send_A_String("NEUTRAL");
	
}

void initialise_counter(void)
{
	TCCR0 |= (1<<CS01)|(1<<CS00);
	TIMSK |= (1<<TOIE0);
	TCNT0 = 0;
}
void stop_counter(void)
{
	TCCR0 &=~ (1<<CS00);
	TCCR0 &=~ (1<<CS01);
	TIMSK &=~ (1<<TOIE0);
	_delay_ms(1);
	TCNT0=0;
}

void sw_debounce_pressed(void)
{
	//code
	unsigned int pressed_confidence_level=0;
	pressed_confidence_level++;
	if(pressed_confidence_level>=500 )//put a debounce value///////////////
	{
		pressed_confidence_level=0;
	}
}
void sw_debounce_released(void)
{
	//code
	unsigned int released_confidence_level=0;
	released_confidence_level++;

	if(released_confidence_level>=500 )//put a debounce value//////////////
	{
		released_confidence_level=0;
	}
}

void Check_IF_MrLCD_isBusy(void)
{
	DataDir_MrLCDsCrib=0;
	MrLCDsControl |= 1<<ReadWrite;
	MrLCDsControl &=~ 1<<BiPolarMood;
	
	while(MrLCDsCrib >= 0x80)
	{
		Peek_A_Boo();
	}
	
	DataDir_MrLCDsCrib=0xFF;
}
void Peek_A_Boo(void)
{
	MrLCDsControl |= 1<<LightSwitch;
	_delay_us(10);/////CALIBRATE IT FURTHER
	MrLCDsControl &=~ 1<<LightSwitch;
}
void Send_A_Command(char command)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = command;
	MrLCDsControl &=~ (1<<ReadWrite|1<<BiPolarMood);
	Peek_A_Boo();
	MrLCDsCrib = 0;	
}
void Send_A_Character(char character)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = character;
	MrLCDsControl &=~ (1<<ReadWrite);
	MrLCDsControl |= (1<<BiPolarMood);
	Peek_A_Boo();
	MrLCDsCrib = 0;
}
void Send_An_Integer(int integer)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = integer;
	MrLCDsControl &=~ (1<<ReadWrite);
	MrLCDsControl |= (1<<BiPolarMood);
	Peek_A_Boo();
	MrLCDsCrib = 0;
}
void Send_A_String(char *stringOfCharacters)
{
	while(*stringOfCharacters > 0)
	{
		Send_A_Character(*stringOfCharacters++);
		//_delay_ms(10);
	}
}
void Send_A_String_inst(char *stringOfCharacters)
{
	while(*stringOfCharacters > 0)
	{
		Send_A_Character(*stringOfCharacters++);// _delay_ms(50);
	}
}

void Assign_motor_direction (void)
{
	switch (ADC_Channnel)
	{
		case 1:
		if (converted_value_1 < 0)
		{
			Motor_1_Direction_Anticlockwise();
		}
		else
		if (converted_value_1 > 0)
		{
			Motor_1_Direction_Clockwise();			
		}
		else
		if (converted_value_1 == 0)
		{
			Motor_1_Switch_OFF();
		}
		break;
		
		case 2:
		if (converted_value_2 < 0)
		{
			Motor_2_Direction_Anticlockwise();
		}
		else
		if (converted_value_2 > 0)
		{
			Motor_2_Direction_Clockwise();
		}	
		else
		if (converted_value_2 == 0)
		{
			Motor_2_Switch_OFF();
		}
		break;	
	}		
}
void converted_value_preobfuscation(void)
{
	switch (ADC_Channnel)
	{
		case 1:
		if (converted_value_1 <= 20 && converted_value_1 >= -20)
		{
			converted_value_1 = 0;
		}
		
		case 2:
		if (converted_value_2 <= 20 && converted_value_2 >= -20)
		{
			converted_value_2 = 0;
		}
	}	
}
void converted_value_absolution (void)
{
	switch (ADC_Channnel)
	{
		case 1:
		
		if (converted_value_1 <= 20 && converted_value_1 >= -20)
		{
			converted_absolute_value_1 = 0;
		}
		else
		{
			if (converted_value_1 > 0)
			{
				converted_absolute_value_1 = converted_value_1;
			}
			else
			if (converted_value_1 < 0)
			{
				converted_absolute_value_1 = converted_value_1*(-1);
			}
		}
		
		break;
		
		case 2:
		
		if (converted_value_2 <= 20 && converted_value_2 >= -20)
		{
			converted_absolute_value_2 = 0;
		}
		else
		{
			if (converted_value_2 > 0)
			{
				converted_absolute_value_2 = converted_value_2;
			}
			else
			if (converted_value_2 < 0)
			{
				converted_absolute_value_2 = converted_value_2*(-1);
			}
		}
		
		break;	
	}
	
}

void convert_PWM_value(void)
{
	if (ADC_Channnel == 1)
	{
		PWM_value_1 = converted_absolute_value_1*38;
	}	
	else 
	if (ADC_Channnel == 2)
	{
		PWM_value_2 = converted_absolute_value_2*38;
	}
}

void Segregate_ADC_Vectors (void)
{
	if (ADC_Channnel == 1)
	{
		converted_value_1 = Channel_1_ADC_value - 515;
	}
	
	else
	if (ADC_Channnel == 2)
	{
		converted_value_2 = Channel_2_ADC_value - 513;
	}
}

ISR (ADC_vect)
{
	uint8_t the_low = ADCL;
	uint8_t the_high = ADCH;
	uint16_t ten_bit_val = the_high<<2 | the_low>>6;
	a = ten_bit_val;
	
	switch (ADMUX)
	{
		case 0x60:
		//b=1;
		Channel_1_ADC_value = a;	
		ADC_Channnel = 1;
		ADMUX = 0x61;		
		break;
		
 		case 0x61:
 		//b=0;
 		Channel_2_ADC_value = a;
		ADC_Channnel = 2;		
 		ADMUX = 0x60;
 		break;
	}
}
ISR(TIMER0_OVF_vect)
{		
	count++;
	if (count==61)
	{
		// 1 second has elapsed
		count=0;
				
		count1++;
		
		PORTC ^= (1<<PINC3);
		
		if (count1>=20)/////// calibrate it further
		{			
			//return coin if not dispatched  
			count1=0;
			FAIL=1;
		}		
	}
}


// if (ADMUX == 0x61)
	// {
	// 	itoa(a,ValueSampled, 10);
	// 	
	// 	
	// 	Send_A_Command(0x80 + 2);//();
	// 	_delay_ms(2);
	// 	Send_A_String("     ");
	// 	_delay_ms(2);
	// 	Send_A_Command(0x80 + 2);//();
	// 	_delay_ms(2);
	// 	Send_A_String_inst(ValueSampled);
	// 	_delay_ms(2);
	// 	
	// 	//_delay_ms(20);
	// 	Segregate_ADC_Vectors();
	// 	Assign_motor_direction();
	// 	
	// 	
	// 	itoa(converted_value,Reduced_Value,10);
	// 	
	// 	Send_A_Command(0x94 + 2);//();
	// 	_delay_ms(2);
	// 	Send_A_String("     ");
	// 	_delay_ms(2);
	// 	Send_A_Command(0x94 + 2);//();
	// 	_delay_ms(2);
	// 	Send_A_String_inst(Reduced_Value);
	// 	//Send_A_String("rhitvik");
	// 	_delay_ms(2);
	// 	
	// 	//_delay_ms(20);
	// 	converted_value_absolution();
	// 	convert_PWM_value();
	// 	
	// 	itoa(PWM_value_1,OCR1A_value,10);
	// 	
	// 	Send_A_Command(0xD4 + 3);//();
	// 	_delay_ms(2);
	// 	Send_A_String("     ");
	// 	_delay_ms(2);
	// 	Send_A_Command(0xD4  + 3);//();
	// 	_delay_ms(2);
	// 	Send_A_String_inst(OCR1A_value);
	// 	_delay_ms(2);
	// 	
	// 	
	// 	//initialise_PWM_1();
	// 	
	// 	//_delay_ms(100);
	// 	ADCSRA |= (1<<ADSC);
	// }
	// else
	// if (ADMUX == 0x60)// for y
	// {
	// 	
//}			