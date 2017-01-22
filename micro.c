#include <avr/io.h>
#include <avr/interrupt.h>

/*

Assumptions:
- the CPU, by entering active mode will "steal" current from LEDs and in this way
  will make them "dim". 

- the control transistor output will be attached to PB3 (PDIP-2)

- the debug pin will be PB4 (PDIP-3)

- the sense input will be PB1 (or INT0) (PDIP-6)

*/

#define COUNTING_TIMEOUT_S (60 * 9)
#define WARNING_TIMEOUT_S  (60 * 1)


typedef enum
{
    STATE_COUNTING,
    STATE_WARNING,
    STATE_OFF,
} app_state_t;



register unsigned char timerForButtonFlag asm("r6"); /* flag to be used between interrupts */
register unsigned char savedR24 asm("r7");     /* interrupt helper to restore r24,r25 */
register unsigned char savedR25 asm("r8");     /* interrupt helper to restore r24,r25 */
register unsigned char appState asm("r9");      /* current fill of PWM. 0-31 */
register unsigned char counter1 asm("r10");
register unsigned char counter2 asm("r11");

void disableInt0(void);


/*
 *  PINS in PORTB setup
 */
#define PWRSUPPORT  3
#define DEBUGPIN    4
#define BUTTONSENSE 1

/*
 *  LOCAL DEFINITION OF SPECIAL REGISTERS -- the code is compiled for tiny2313 which may have different
 *  numbers. This makes sure we are using tiny12 registers
 */
#ifdef ACSR
#undef ACSR
#endif
#define ACSR    _SFR_IO8(0x08)

#ifdef TIFR
#undef TIFR
#endif
#define TIFR    _SFR_IO8(0x38)

#ifdef TCNT0
#undef TCNT0
#endif
#define TCNT0   _SFR_IO8(0x32)

#ifdef TCCR0
#undef TCCR0
#endif
#define TCCR0   _SFR_IO8(0x33)

#ifdef MCUCR
#undef MCUCR
#endif
#define MCUCR   _SFR_IO8(0x35)

#ifdef GIMSK
#undef GIMSK
#endif
#define GIMSK   _SFR_IO8(0x3B)

#ifdef GIFR
#undef GIFR
#endif
#define GIFR   _SFR_IO8(0x3A)



/*
 *  Using ACSR to return flags from Timer Interrupt.
 *  Register variables are non-volatile by default, so hacking is needed
 */
void clrFlagButton(void)
{
    ACSR &= ~_BV(0);
}

void setFlagButton(void)
{
    ACSR |= _BV(0);
}

uint8_t getFlagButton(void)
{
    return (ACSR & _BV(0));
}



void clrFlagCollectSamples(void)
{
    ACSR &= ~_BV(1);
}

void setFlagCollectSamples(void)
{
    ACSR |= _BV(1);
}

uint8_t getFlagCollectSamples(void)
{
    return (ACSR & _BV(1));
}



void clrFlag200ms(void)
{
    ACSR &= ~_BV(6);
}

void setFlag200ms(void)
{
    ACSR |= _BV(6);
}

uint8_t getFlag200ms(void)
{
    return (ACSR & _BV(6));
}




/* external input interrupt handler */
ISR(_VECTOR(1), ISR_NAKED)
{
    /* save r24, r25 */
    asm("mov r7, r24"::);
    asm("mov r8, r25"::);

    disableInt0();  /* disable the interrupt <= we need to debounce, will be done in
                       timer routine. Will ruin 200ms, but action will be taken anyway.
                       Timer will handle debouncing and re-enable INT0 */
    TCNT0 = 256 - (234 / 4); /* 200ms should do */
    TIFR |= _BV(1);  /* clear TOV0 interrupt flag in case the timer just fired */
    timerForButtonFlag = 1; /* inform timer interrupt about keyboard */

    /* restore r24, r25 and return */
    asm("mov r24, r7"::);
    asm("mov r25, r8"::);
    reti();
}

void disableInt0(void)
{
    GIMSK &= ~_BV(INT0); /* disable the interrupt for now */
}

void enableInt0(void)
{
    GIFR  |= _BV(INTF0); /* clear flag if already present */
    GIMSK |= _BV(INT0);  /* enable mask */
}

void setupInt0(void)
{
    MCUCR |= _BV(ISC01) | _BV(ISC00); /* The rising edge of INT0 generates an interrupt request */
    enableInt0();
}



/* timer interrupt handler */
ISR(_VECTOR(3), ISR_NAKED)
{
    /* save r24, r25 */
    asm("mov r7, r24"::);
    asm("mov r8, r25"::);

    /* rewind timer - must be first */
    TCNT0 = 256 - 234; /* TCNT0: target delay is 200ms */

    if (timerForButtonFlag == 0)
    {
        setFlag200ms();
        /* re-enable button interrupts when released */
        if ((PINB & _BV(BUTTONSENSE)) == 0)
        {
            enableInt0();
        }
    }
    else  /* timer fired because of button press */
    {
        timerForButtonFlag = 0; /* re-enable the button interrupt */
        if ((PINB & _BV(BUTTONSENSE)) != 0)
        {
            setFlagButton();
        }
    }

    /* restore r24, r25 and return */
    asm("mov r24, r7"::);
    asm("mov r25, r8"::);
    reti();
}

void setupTimer(void)
{
    TCCR0 = 5; /* TCCR0 set timer to CLK/1024 */
    TIMSK = _BV(TOIE0);
}

void calibrateOscillator(void)
{
    _SFR_IO8(0x31) = 0x40; /* some adjustable value */
}



void enableSleep(void)
{
    MCUCR |= _BV(SE);
}

void disableSleep(void)
{
    MCUCR &= ~_BV(SE);
}


void enterState(app_state_t newState)
{
    /* state exit procedure */
    switch (appState)
    {
        case STATE_WARNING:
            enableSleep();
            break;
        default:
            break;
    }

    /* state enter procedure */
    switch (newState)
    {
        case STATE_COUNTING:
            counter1 = 0; /* 5 Hz */
            counter2 = 0; /* 30 sec */
            appState = STATE_COUNTING;
            break;

        case STATE_WARNING:
            counter1 = 0; /* 5 Hz */
            counter2 = 0; /* 30 sec */
            appState = STATE_WARNING;
            break;

        case STATE_OFF:
            PORTB &= ~_BV(PWRSUPPORT);
            appState = STATE_OFF;
            break;
    }
}


/**
 *  Runs every 200ms. Actions depend on the app state
 */
void handleTimerTick(void)
{
    /* handle timer ticks */
    if (counter1 == (5 * 30))
    {
        counter1 = 0;
        ++counter2;
    }
    else
    {
        ++counter1;
    }

    switch (appState)
    {
        case STATE_COUNTING:
            if (counter2 == (COUNTING_TIMEOUT_S / 30))
            {
                enterState(STATE_WARNING);
            }
            break;

        case STATE_WARNING:
            if ((counter1 & 0x2) != 0) /* led dimming effect */
            {
                disableSleep();
                PORTB ^= _BV(DEBUGPIN);
            }
            else
            {
                PORTB ^= _BV(DEBUGPIN);
                enableSleep();
            }
            if (counter2 == (WARNING_TIMEOUT_S / 30))
            {
                enterState(STATE_OFF);
            }
            break;

        case STATE_OFF:  /* wait until switched off */
            break;
    }
}


/**
 *  Button press handler
 */
void handleButtonPressed(void)
{
    switch (appState)
    {
        case STATE_COUNTING:  /* button when counting = goto OFF */
            enterState(STATE_OFF);
            break;

        case STATE_WARNING:   /* button when warning = restart counting */
            enterState(STATE_COUNTING);
            break;

        case STATE_OFF: /* when in state OFF, do nothing, wait until power goes off */
            break;
    }
}


void showState(void)
{
    switch (appState)
    {
        case STATE_COUNTING:
            PORTB = _BV(PWRSUPPORT);
            break;
        case STATE_WARNING:
            PORTB = _BV(DEBUGPIN);
            break;
        case STATE_OFF:
            PORTB = _BV(PWRSUPPORT) | _BV(DEBUGPIN);
            break;
        default:
            PORTB = 0;
            break;
    }
}


/**
 *  main loop
 */
int main(void)
{
    /* hack to provide minimum C runtime */
    asm volatile("eor	r1, r1"::);

    /* initialise */
    DDRB = _BV(DEBUGPIN) | _BV(PWRSUPPORT);
    calibrateOscillator();
    setupTimer();
    setupInt0();
    enableSleep();
    PORTB = _BV(PWRSUPPORT);  /* enable external power supply */
    timerForButtonFlag = 0;
    counter1 = 0;
    counter2 = 0;
    appState = STATE_OFF;
    enterState(STATE_COUNTING);
    sei();

    /* Normal loop */
    for (;;)
    {

        if (appState == STATE_OFF)
        {
            asm("sleep"::);
            continue;
        }

#if 0
        showState();
#endif

        /* handle timer tick */
        if (getFlag200ms() != 0)
        {
            cli();
            clrFlag200ms();
            sei();
            handleTimerTick();
        }


        /* handle button press. We know it is pressed, call handler */
        if (getFlagButton() != 0)
        {
            cli();
            clrFlagButton();
            sei();
            handleButtonPressed();
        }

        asm("sleep"::);
    }
}

