/*
 * self_control.c
 *
 * Created: 13/05/2011 12:22:10
 *  Author: Christo
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

/*!
 * Target : ATtiny24A
 * Clock : 8MHz
 *
 * Fuse Extended Byte:
 ** SELFPRGEN 0 -> 1
 *
 * Fuse High Byte:
 ** RSTDISBL 7 -> 1
 ** DWEN 6 -> 1
 ** SPIEN 5 -> 0
 ** WDTON 4 -> 1
 ** EESAVE 3 -> 1
 ** BODLEVEL [2:0] -> 111 (disabled)
 *
 * Fuse Low Byte:
 ** CKDIV8 7 -> 1 (divide clock by 8)
 ** CKOUT 6 -> 1
 ** SUT [5:4] -> 10 (startup time 6CK)
 ** CKSEL [3:0] : 0010 (internal 8MHz)
 */


/*!
 *
 */

#define BCD_DRIVER_A 0
#define BCD_DRIVER_B 0
#define BCD_DRIVER_C 0
#define BCD_DRIVER_D 0
#define BLANK_DISPLAY 15
#define BUZZER 0

#define START_IN 0 //
#define STOP_IN 0
#define SENSOR_IN 0
#define BUZZER_VOLUME_IN 0

#define DEBOUNCE_TIME 15
#define BOUND_TIME 2000


uint16_t millis()
{
	//TODO
}


/************************************************************************/
/* Contact                                                              */
/************************************************************************/
typedef struct
{
	uint8_t _port;
	uint8_t _state;
	uint16_t _time;
} Contact;
Contact bound0, bound1;
Contact sensor;

void Contact_init(Contact* self, uint8_t a_port)
{
	self->_port = a_port;
	self->_state = 0xff & a_port;
}

void Contact_update(Contact* self)
{
	const uint8_t state = (PINB & self->_port);
	if (state != self->_state)
	{
		self->_time = millis();
		self->_state = state;
	}
}

/**
 * \param a_time must be <65535
 */
uint8_t Contact_lastOver(Contact* self, uint16_t a_time)
{
	return ((millis() - self->_time) > a_time);
}



/************************************************************************/
/* GAME                                                                 */
/************************************************************************/
typedef enum
{
	PENDING,
	IN_PROGRESS,
	STOPPED
} GameStatus;

typedef struct 
{
	GameStatus _state;
	uint8_t _score;
	Contact* _start;
	Contact* _end;
	uint16_t _time;
} Game;
Game game;


void Game_init()
{
	game._state = PENDING;
	game._score = 0;
	game._start = game._end = 0;
}

void Game_reset(Contact* a_start)
{
	game._state = PENDING;
	game._score = 0;
	game._start = a_start;
	game._end = ((a_start == &bound0) ? &bound1 : &bound0);
}

void Game_start()
{
	game._state = IN_PROGRESS;
	game._time = millis();
}

void Game_terminate()
{
	game._state = STOPPED;
}


/************************************************************************/
/* MISC                                                                 */
/************************************************************************/
uint8_t Volume;

void update_display()
{
	//update display
	PINA &= 0b11110000;
	PINA |= game._score;
}	


/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* SETUP                                                                */
	/************************************************************************/
	//Digital I/O
	DDRB = 0;//digital inputs
	PORTB = (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);//{0 1 2} with internal pullup
	DDRA = (1<<DDA5) | (1<<DDA3) | (1<<DDA2) | (1<<DDA1) | (1<<DDA0);//digital outputs for buzzer and BCD driver (ABCD wires)
		
	//ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);//ADC prescaler 64
	ADCSRB = (1<<ADLAR);//ADC Left Adjust Result
		
	//PWM for buzzer
	OCR1A = 1211;//6.6KHz = fPWM = fT1/(N*(1+TOP))
	OCR1B = 0;
	TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);//non-inverted PWM -> COM1B[1:0] = 10 ; fast PWM with TOP=OCR1A -> WGM1[3:0] = 1111
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);//fT1 = fCLK_IO / 1
		
	
	/************************************************************************/
	/* BUZZER VOLUME                                                        */
	/************************************************************************/
	ADMUX = 7;//ADC channel 7
	ADCSRA |= (1<<ADSC);//start ADC conversion
	while (!(ADCSRA & (1<<ADIF)));//poll ADC Interrupt Flag
	ADCSRA |= (1<<ADIF);//clear ADC Interrupt Flag
	Volume = ADCH;//get value from high byte only (left adjust result)
	
	
	/************************************************************************/
	/* GAME                                                                 */
	/************************************************************************/
	Contact_init(&bound0, PINB0);
	Contact_init(&bound1, PINB1);
	Game_init();
	
	
	uint8_t start_pin = -1;
	uint8_t stop_pin = -1;
	
	Contact* start, end;
	
    while (1)
    {
		//si on touche un bord plus de 3 secondes, celui-ci devient start -> init_game(PIN_NBR)
		//si on ne touche plus le bord de départ depuis plus de 15ms c'est parti
		//si on touche le fil plus de 15ms c'est une touche
		//au bout de 30s, score 9
		//si on touche le bord de fin, c'est fini
		//si score 9 c'est fini
		
		//a la touche -> beep
		//a score 9 -> beep beep
		//durant le jeu -> display clignote
		
		Contact_update(&bound0);
		Contact_update(&bound1);
		
		//read at BOUND0_PIN
		if (!bound0._state && Contact_lastOver(&bound0, BOUND_TIME))
		{
			Game_reset(&bound0);
		}
				
		//read at BOUND1_PIN
		if (!bound1._state && Contact_lastOver(&bound1, BOUND_TIME))
		{
			Game_reset(&bound1);
		}			
						
		//read at SENSOR_PIN
		if (status == IN_PROGRESS)
		{
			Contact_update(&sensor);
			if (!sensor._state && is_contact_stable(&sensor))
			{
				if (score != 9)
				{
					++score;
				}
				
				if (score == 9)
				{
					terminate_game();
				}
			}
		}			
		//if (status == IN_PROGRESS)
		//{
			//const uint8_t sensor_state = (PINB & PINB2);
			//if (sensor_state != last_sensor_state)//touch sensor wire
			//{
				//last_touch_time = millis();
				//last_sensor_state = sensor_state;
			//}
			//
			//if (!last_sensor_state && (millis() - last_touch_time) > DEBOUNCE_TIME)
			//{
				//if (score != 15)
				//{
					//++score;
					//update_display();
				//}					
			//}
		//}
		
    }
}
