#include <avr/io.h>
#include <avr/interrupt.h>

register unsigned char pwmFastCount asm("r6"); /* soft divider to help lower PFM freq to 300Hz */
register unsigned char savedR24 asm("r7");     /* interrupt helper to restore r24,r25 */
register unsigned char savedR25 asm("r8");     /* interrupt helper to restore r24,r25 */
register unsigned char pwmFill asm("r9");      /* current fill of PWM. 0-31 */
register unsigned char counter asm("r10");
register unsigned char counter2 asm("r11");
register unsigned char bitCounter asm("r12");   /* counter for received bits */
register unsigned char receivedChar asm("r13"); /* register for received character */

void disableInt0(void);


/*
 *  PINS in PORTB setup
 */
#define PWMOUTPIN1  4
#define PWMOUTPIN2  3
#define PWMOUTPIN3  0
#define PWMOUTPIN4  2

#define RS232PIN   1


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
void clrFlagReceivedChar(void)
{
    ACSR &= ~_BV(0);
}

void setFlagReceivedChar(void)
{
    ACSR |= _BV(0);
}

uint8_t getFlagReceivedChar(void)
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



void clrFlag300Hz(void)
{
    ACSR &= ~_BV(6);
}

void setFlag300Hz(void)
{
    ACSR |= _BV(6);
}

uint8_t getFlag300Hz(void)
{
    return (ACSR & _BV(6));
}




/* external input interrupt handler */
ISR(_VECTOR(1), ISR_NAKED)
{
    /* save r24, r25 */
    asm("mov r7, r24"::);
    asm("mov r8, r25"::);

    /* a new pin change has shown, start counting to finish at the middle and rewind the timer */
    TCNT0 = 256 - 20; /* TCNT0: next interrupt should come in the middle of a character */
    TIFR |= _BV(1);  /* clear TOV0 interrupt flag in case the timer just fired */

    setFlagCollectSamples();  /* start collecting bits */
    bitCounter = 9; /* 10 bits for 8N1 format, 0 also counts as bit */
    disableInt0();  /* disable the interrupt because in the frame there will be a few high bits */

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
    clrFlagCollectSamples();
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
    TCNT0 = 256 - 110/*117*/; /* TCNT0: need to count 25 ticks */

    /* handle PWM */
    ++pwmFastCount;
    if ((pwmFastCount & 32) == 32)  /* 9600 / 32 = 300 */
    {
        pwmFastCount = 0;
        /* enable PWM pin here */
        PORTB |= _BV(PWMOUTPIN1) | _BV(PWMOUTPIN2) | _BV(PWMOUTPIN3) | _BV(PWMOUTPIN4);
    }
    if (pwmFastCount >= pwmFill)
    {
        /* disable PWM pin when counter */
        PORTB &= ~(_BV(PWMOUTPIN1) | _BV(PWMOUTPIN2) | _BV(PWMOUTPIN3) | _BV(PWMOUTPIN4));
    }

    /* handle UART */
    if (getFlagCollectSamples())
    {
        if (bitCounter == 9)  {  --bitCounter;  receivedChar = 0;  }
        else
        {
             if (bitCounter > 0)
             {
                 --bitCounter;
                 receivedChar = (receivedChar >> 1);
                 if ((PINB & _BV(RS232PIN)) == 0)
                 {
                     receivedChar |= 0x80; /* set bit if something comes from RS */
                 }
             }
             else /* bitCounter == 0 */ 
            {  
                clrFlagCollectSamples(); /* re-enable reception of new characters */
                enableInt0();
                setFlagReceivedChar();  /* inform task that there is new character */
            }
        }
    }

    /* for delay loops */
    setFlag300Hz();

    /* restore r24, r25 and return */
    asm("mov r24, r7"::);
    asm("mov r25, r8"::);
    reti();
}



void setupTimer(void)
{
    TCCR0 = 1; /* TCCR0 set timer to CLK/1 */
    TIMSK = _BV(TOIE0);
}


void calibrateOscillator(void)
{
    _SFR_IO8(0x31) = 0x27; /* OSCCAL 0x27 read from avrdude -Ucalibrate */
}

void setupPwm(void)
{
    pwmFill = 0;
}

void setPwm(uint8_t newPwmFill)
{
    pwmFill = newPwmFill;
}

uint8_t getPwm(void)
{
    return pwmFill;
}

void incPwm(void)
{
    if (pwmFill < 32)
    {
        pwmFill = pwmFill + 1;
    }
    else
    {
        pwmFill = 0;
    }
}


void enableSleep(void)
{
    MCUCR |= _BV(SE);
}

int main(void)
{
    /* hack to provide minimum C runtime */
    asm volatile("eor	r1, r1"::);

    /* initialise */
    DDRB = _BV(PWMOUTPIN1) | _BV(PWMOUTPIN2) | _BV(PWMOUTPIN3) | _BV(PWMOUTPIN4) ;
    calibrateOscillator();
    setupTimer();
    setupPwm();
    setupInt0();
    enableSleep();
    sei();

    /* Soft start */
    for (counter2 = 0; counter2 < 64; ++counter2)
    {
        for (counter = 0; counter < 250; ++counter)
        {
            if (getFlag300Hz())
            {
                cli();
                clrFlag300Hz();
                sei();
                asm("sleep"::);
            }
        }
        setPwm(counter2 >> 2);
    }


    /* Normal loop */
    for (;;)
    {
        if (getFlagReceivedChar())
        {
            cli();
            clrFlagReceivedChar();
            sei();
            setPwm(receivedChar & 0x1F);
        }

        asm("sleep"::);
    }
}

