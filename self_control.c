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


/*!
 *
 */

#define BLANK_DISPLAY 15

#define DEBOUNCE_TIME 15
#define BOUND_TIME 2000


#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint16_t timer0_overflow_count = 0;
volatile uint16_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

ISR(TIM0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint16_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

uint16_t millis()
{
	uint16_t m;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		m = timer0_millis;

	return m;
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
	uint16_t _time;
} Game;
Game game;


void Game_init()
{
	game._state = PENDING;
	game._score = 0;
	game._start = game._end = 0;
}

void Game_over()
{
	game._state = STOPPED;
	game._score = 9;
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

void Game_score_up()
{
	if (game._score < 8)
		++game._score;
	else
		Game_over();
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
	sei();//enable global interrupt mask
	Contact_init(&bound0, PINB0);
	Contact_init(&bound1, PINB1);
	Game_init();
	
		
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
		
		//Update contact state
		Contact_update(&bound0);
		Contact_update(&bound1);
		Contact_update(&sensor);
		
		//read at bound0
		if (!bound0._state && Contact_last_over(&bound0, BOUND_TIME))
		{
			Game_reset(&bound0);
		}
				
		//read at bound1
		if (!bound1._state && Contact_last_over(&bound1, BOUND_TIME))
		{
			Game_reset(&bound1);
		}			
		
		
		//check game is init
		if (!game._start)
			continue;
		
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
			if (!sensor._state && Contact_last_over(&sensor, DEBOUNCE_TIME))
				Game_score_up();
		}			
    }
	
	return 0;
}
