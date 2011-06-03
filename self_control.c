/*
 * self_control.c
 *
 * Created: 13/05/2011 12:22:10
 *  Author: Christo
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

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


/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/
#define BLANK_DISPLAY 15

#define DEBOUNCE_TIME 40
#define BOUND_TIME 1800

#define BUZZER_FREQUENCY 4000
#define BUZZER_EVENT 200
#define BUZZER_LONG_EVENT 750


/************************************************************************/
/* TIMER0 INTERRUPT                                                     */
/************************************************************************/
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
// the prescaler is set so that timer0 ticks every 8 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR (TIM0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;

	m += MILLIS_INC;
	timer0_fract += FRACT_INC;
	if (timer0_fract >= FRACT_MAX)
	{
		timer0_fract -= FRACT_MAX;
		++m;
	}

	timer0_millis = m;
}

unsigned long millis()
{
	unsigned long m;

	ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
		m = timer0_millis;

	return m;
}


/************************************************************************/
/* TIMER1 INTERRUPT                                                     */
/************************************************************************/
volatile unsigned long timer1_period_count = 0;

ISR (TIM1_OVF_vect)
{
	if (timer1_period_count)
	{
		--timer1_period_count;
	}
	else
	{
		TCCR1B &= ~(1<<CS10);//Stop Timer/Counter1
		TCNT1 = 0;//Reset Timer/Counter1 counter
		PORTA &= ~(1<<OC1B_PIN);//force out to LOW
	}
}


/************************************************************************/
/* Buzzer                                                               */
/************************************************************************/
void Buzzer_play(uint16_t a_duration)
{
	ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
		timer1_period_count = ((unsigned long)BUZZER_FREQUENCY * a_duration) / 1000;
	
	TCCR1B |= (1<<CS10);//Start Timer/Counter1
}


/************************************************************************/
/* Contact                                                              */
/************************************************************************/
typedef struct
{
	uint8_t _port_mask;
	uint8_t _state;
	uint32_t _time;
	uint8_t _used;
} Contact;
Contact bound0, bound1;
Contact sensor;

void Contact_init(Contact* self, uint8_t a_port)
{
	self->_port_mask = 1<<a_port;
	self->_state = self->_port_mask;
	self->_used = 0;
	self->_time = millis();
}

void Contact_update(Contact* self)
{
	const uint8_t state = (PINB & self->_port_mask);
	if (state != self->_state)
	{
		self->_state = state;
		self->_used = 0;
		self->_time = millis();
	}
}

/**
 * \param a_time must be <65535
 */
uint8_t Contact_last_over(Contact* self, uint16_t a_time)
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
	uint32_t _start_time;
	uint16_t _max_time;
} Game;
Game game;


void Game_init()
{
	game._state = PENDING;
	game._score = 0;
	game._start = game._end = 0;
	//Max time from ADC
	ADCSRA |= (1<<ADSC);//start ADC conversion
	while (!(ADCSRA & (1<<ADIF)));//poll ADC Interrupt Flag
	ADCSRA |= (1<<ADIF);//clear ADC Interrupt Flag
	game._max_time = 60 * ADC;
}

void Game_over()
{
	game._state = STOPPED;
	game._score = 9;
	Buzzer_play(BUZZER_LONG_EVENT);
}

void Game_check_time()
{
	if (millis() - game._start_time > game._max_time)
		Game_over();
}

void Game_reset(Contact* a_start)
{
	game._state = PENDING;
	game._score = 0;
	game._start = a_start;
	game._end = ((game._start == &bound0) ? &bound1 : &bound0);
	a_start->_used = 1;
	Buzzer_play(BUZZER_EVENT);
}

void Game_start()
{
	game._state = IN_PROGRESS;
	game._start_time = millis();
	Buzzer_play(BUZZER_EVENT);
}

void Game_terminate()
{
	game._state = STOPPED;
	Buzzer_play(BUZZER_LONG_EVENT);
}

void Game_score_up(Contact* a_sensor)
{
	if (game._score < 8)
	{
		++game._score;
		a_sensor->_used = 1;
		Buzzer_play(BUZZER_EVENT);
	}		
	else
		Game_over();
}


/************************************************************************/
/* DISPLAY                                                              */
/************************************************************************/
void Display_init()
{
	PORTA &= 0b11110000;
}	

void Display_update()
{
	if (game._state == IN_PROGRESS)
	{
		const unsigned long unit = millis() / 500;//1/2 second unit
		if (unit & 1)
		{
			PORTA = (PORTA & 0b11110000) | BLANK_DISPLAY;
		}
		else
		{
			PORTA = (PORTA & 0b11110000) | game._score;
		}
	}
	else
		PORTA = (PORTA & 0b11110000) | game._score;
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
	ADMUX = 7;//ADC channel 7
		
	//PWM for buzzer
	OCR1A = F_CPU / BUZZER_FREQUENCY - 1;//fPWM = fT1/(N*(1+TOP))
	OCR1B = OCR1A / 2;//Duty cyle 50%
	TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);//non-inverted PWM -> COM1B[1:0] = 10 ; fast PWM with TOP=OCR1A -> WGM1[3:0] = 1111
	TCCR1B = (1<<WGM13) | (1<<WGM12);
	TIMSK1 = (1<<TOIE1);//Timer/Counter1 Overflow Interrupt Enable
	
	//Timer to measure time with millis()
	TCCR0A = 0;//normal port operation, non-PWM mode
	TCCR0B = (1<<CS01);//fT0 = fCLK_IO / 8 -> CS0[2:0] = 010
	TIMSK0 = (1<<TOIE0);//Timer/Counter0 Overflow Interrupt Enable
	
	
	/************************************************************************/
	/* GAME                                                                 */
	/************************************************************************/
	sei();//enable global interrupt mask
	Contact_init(&bound0, PINB0);
	Contact_init(&bound1, PINB1);
	Contact_init(&sensor, PINB2);
	Game_init();
	Display_init();
		
    while (1)
    {
		//si on touche un bord plus de 3 secondes, celui-ci devient start -> init_game(PIN_NBR)
		//si on ne touche plus le bord de départ depuis plus de 15ms c'est parti
		//si on touche le fil plus de 15ms c'est une touche
		//au bout de 30s, score 9
		//si on touche le bord de fin, c'est fini
		//si on touche plus de N secondes c'est fini
		//si score 9 c'est fini
		
		//a la touche -> beep
		//a score 9 -> beep beep
		//durant le jeu -> display clignote
		
		//Check game time
		if (game._state == IN_PROGRESS)
			Game_check_time();
		
		//Update contact state
		Contact_update(&bound0);
		Contact_update(&bound1);
		Contact_update(&sensor);
		
		//read at bound0
		if (!bound0._used && !bound0._state && Contact_last_over(&bound0, BOUND_TIME))
			Game_reset(&bound0);
				
		//read at bound1
		if (!bound1._used && !bound1._state && Contact_last_over(&bound1, BOUND_TIME))
			Game_reset(&bound1);
		
		
		//check game is init
		if (game._start && game._end)
		{
			//read at start
			if (game._state == PENDING)
			{
				if (game._start->_state && Contact_last_over(game._start, DEBOUNCE_TIME))
					Game_start();
			}
		
			if (game._state == IN_PROGRESS)
			{
				//read at end
				if (!game._end->_state)
					Game_terminate();
			
				//read at sensor
				if (!sensor._state)
				{
					if (Contact_last_over(&sensor, BOUND_TIME))
						Game_over();
					else if (!sensor._used && Contact_last_over(&sensor, DEBOUNCE_TIME))
						Game_score_up(&sensor);
				}						
			}
		}
		
		Display_update();	
    }
	
	return 0;
}
