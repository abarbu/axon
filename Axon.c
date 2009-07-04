/****************************************************************************
 *
 *   Copyright (c) 2008 www.societyofrobots.com
 *   (please link back if you use this code!)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   Alternatively, this software may be distributed under the terms of BSD
 *   license.
 *
 ****************************************************************************/

//SoR Include
#include <avr/wdt.h>
#include "SoR_Utils.h" //includes all the technical stuff
#include "sensors.c" //sensor libraries for sonar, sharp IR, etc.
/* #include "misc.c" //includes libraries for various hardware and other useful stuff */
 /* #include "axon_DAQ.c" //use the Axon like a data acquisition device */
//#include "Blackfin_Axon.c" //files for Blackfin Robot camera
//#include "axon_test.c" //include this is doing a function test for the Axon
//#include "axon_oscope_test.c" //include this is doing a function test for the Axon

void control();

int main(void)
{
  /****************INITIALIZATIONS*******************/
  //other stuff Im experimenting with for SoR
  uartInit();  // initialize the UART (serial port)
  uartSetBaudRate(0, 38400); // set UARTE speed, for Bluetooth
  uartSetBaudRate(1, 115200); // set UARTD speed, for USB connection, up to 500k, try 115200 if it
  uartSetBaudRate(2, 38400); // set UARTH speed
  uartSetBaudRate(3, 38400); // set UARTJ speed, for Blackfin
  //G=Ground, T=Tx (connect to external Rx), R=Rx (connect to external Tx)

  // initialize rprintf system and configure uart1 (USB) for rprintf
  rprintfInit(uart1SendByte);

  configure_ports(); // configure which ports are analog, digital, etc.

  LED_on();

  rprintf("\r\nSystem Warming Up");

  // initialize the timer system (comment out ones you don't want)
  init_timer0(TIMER_CLK_1024);
  init_timer1(TIMER_CLK_64);
  init_timer2(TIMER2_CLK_64);
  init_timer3(TIMER_CLK_64);
  init_timer4(TIMER_CLK_64);
  init_timer5(TIMER_CLK_1024);
  //timer5Init();

  a2dInit(); // initialize analog to digital converter (ADC)
  a2dSetPrescaler(ADC_PRESCALE_DIV32); // configure ADC scaling
  a2dSetReference(ADC_REFERENCE_AVCC); // configure ADC reference voltage

  int i = 0, j = 0;
  //let system stabelize for X time
  for(i=0;i<16;i++)
  {
    j=a2dConvert8bit(i);//read each ADC once to get it working accurately
    delay_cycles(5000); //keep LED on long enough to see Axon reseting
    rprintf(".");
  }

  LED_off();

  rprintf("Initialization Complete \r\n");

  reset_timer0();
  reset_timer1();
  reset_timer2();
  reset_timer3();
  reset_timer4();
  reset_timer5();

  while(1)
  {
    control();
    delay_cycles(100);
    //an optional small delay to prevent crazy oscillations
  }

  return 0;
}

void bzero(char* x, int y)
{
  while(y--) *x++ = 0;
}

#define MAX_TASKS 8
#define MAX_QUANTUM 100

#define PRIORITY_RT 0
#define PRIORITY_P1 1
#define PRIORITY_P2 2
#define PRIORITY_NULL 7

// 32 user + status
#define NR_REGISTERS 35

struct task_t
{
  uint8_t registers[NR_REGISTERS];
  char* stack;
  uint8_t valid, unblocked, priority, quantum, sleeping;
  uint16_t wakeup;
  // todo andrei check for overflow in wakeup
} __attribute__((packed));

typedef void (*void_function_t)(void);

struct task_t tasks[MAX_TASKS];
struct task_t* current_task;
void* stack_top;
uint16_t time_out;

void initialize_scheduler()
{
  bzero((void*)tasks, sizeof(tasks));
  volatile uint8_t x;
  stack_top = (void*)&x;
  return;
}

int8_t new_task(void_function_t* main, uint8_t priority, uint8_t stack_size)
{
  int i = 0;
  while(i < MAX_TASKS)
  {
    if(tasks[i].valid) { i++; continue; }

    tasks[i].valid = 1;
    tasks[i].unblocked = 1;
    tasks[i].sleeping = 0;
    tasks[i].priority = priority;
    tasks[i].wakeup = 0;
    tasks[i].stack = (char*)malloc(stack_size)+stack_size;
    tasks[i].quantum = MAX_QUANTUM;
    asm("st %a0+, r30" "\n\t"
    	"st %a0, r31" "\n\t"
    	: : "x"(tasks[i].stack), "z"(main));
    tasks[i].stack += 2;
    return i;
  }
  return -1;
}

// wanted: usart task, servo control task

// take over timer 5

// 16k cycles per millisecond
// should set the length to 1ms

/* void disable_interrupts(); */
/* void enable_interrupts(); */

// todo andrei calculate quantum penalty
// todo andrei add priority inversion
// todo andrei how many clock cycles will this take?
// todo how expensive is indirection?
// todo watchdog reset task
// todo uart listen task
// todo a tool for code -> time
// no task that is not active can be unblocked

void switch_task(struct task_t* task);

// unlock the global interrupt disable

// ~200 cycles
__attribute__((noreturn, hot, naked)) void enter_scheduler()
{
  // decrement quantum by f_cpu*priority*constant; 0 if rt
  const int time_in = get_timer5_counter();
  if(current_task->priority != PRIORITY_RT)
  {
    // read timer 5 and decrement
    // todo andrei factor in overflow
    current_task->quantum -= time_out - time_in;
  }

  uint8_t best_priority = PRIORITY_NULL;
  int best_tid = 0;
  while(1)
  {
    // get_timer5_overflow?
    const uint16_t timer5 = get_timer5_counter();
    for(int i = 0; i < MAX_TASKS; ++i)
    {
      if(tasks[i].sleeping && tasks[i].wakeup < timer5)
      {
	tasks[i].unblocked = 1;
	tasks[i].sleeping = 0;
      }
      if(tasks[i].unblocked &&
	 (tasks[i].priority < best_priority) &&
	 tasks[i].quantum)
      {
	best_priority = tasks[i].priority;
	best_tid = i;
      }
    }
    if(best_priority == PRIORITY_NULL)
      for(int i = 1; i < MAX_TASKS; ++i)
      { tasks[i].quantum = MAX_QUANTUM; continue; }
    else
      switch_task(&tasks[best_tid]);
  }
}

// ~80 cycles
ISR(TIMER5_COMPA_vect, ISR_NAKED)
{
  asm("push r30");
  asm("push r31");
  asm("st %a0+, r0"  "\n\t"
      "st %a0+, r1"  "\n\t"
      "st %a0+, r2"  "\n\t"
      "st %a0+, r3"  "\n\t"
      "st %a0+, r4"  "\n\t"
      "st %a0+, r5"  "\n\t"
      "st %a0+, r6"  "\n\t"
      "st %a0+, r7"  "\n\t"
      "st %a0+, r8"  "\n\t"
      "st %a0+, r9"  "\n\t"
      "st %a0+, r10"  "\n\t"
      "st %a0+, r11"  "\n\t"
      "st %a0+, r12"  "\n\t"
      "st %a0+, r13"  "\n\t"
      "st %a0+, r14"  "\n\t"
      "st %a0+, r15"  "\n\t"
      "st %a0+, r16"  "\n\t"
      "st %a0+, r17"  "\n\t"
      "st %a0+, r18"  "\n\t"
      "st %a0+, r19"  "\n\t"
      "st %a0+, r20"  "\n\t"
      "st %a0+, r21"  "\n\t"
      "st %a0+, r22"  "\n\t"
      "st %a0+, r23"  "\n\t"
      "st %a0+, r24"  "\n\t"
      "st %a0+, r25"  "\n\t"
      "st %a0+, r26"  "\n\t"
      "st %a0+, r27"  "\n\t"
      "st %a0+, r28"  "\n\t"
      "st %a0+, r29"  "\n\t"
      "movw r28, r30"  "\n\t"
      "pop r31"  "\n\t"
      "pop r30"  "\n\t"
      "st Y+, r30"  "\n\t"
      "st Y+, r31"  "\n\t"
      "in r30, __SREG__"  "\n\t"
      "st Y+, r30"  "\n\t"
      /* "pop r30"  "\n\t" */ // pc
      /* "pop r31"  "\n\t" */
      /* "st Y+, r30"  "\n\t" */
      /* "st Y+, r31"  "\n\t" */
      "in r30, __SP_L__"  "\n\t"
      "in r31, __SP_H__"  "\n\t"
      "st Y+, r30"  "\n\t"
      "st Y+, r31"  "\n\t"
      : : "z"(current_task));
  enter_scheduler();
  // some sort of error
}

// ~80 cycles
__attribute__((naked)) void switch_task(struct task_t* task)
{
  asm("ld r0, %a0"  "\n\t"
      "ld r1, -%a0"  "\n\t"
      "out __SP_H__, r0"  "\n\t"
      "out __SP_L__, r1"  "\n\t"
      "ld r2, -%a0"  "\n\t"
      "out __SREG__, r2"  "\n\t"
      "ld r31, -%a0"  "\n\t"
      "ld r30, -%a0"  "\n\t"
      "push r30"  "\n\t"
      "push r31"  "\n\t"
      "movw r30, r26" "\n\t"
      "ld r29, -Z" "\n\t"
      "ld r28, -Z" "\n\t"
      "ld r27, -Z" "\n\t"
      "ld r26, -Z" "\n\t"
      "ld r25, -Z" "\n\t"
      "ld r24, -Z" "\n\t"
      "ld r23, -Z" "\n\t"
      "ld r22, -Z" "\n\t"
      "ld r21, -Z" "\n\t"
      "ld r20, -Z" "\n\t"
      "ld r19, -Z" "\n\t"
      "ld r18, -Z" "\n\t"
      "ld r17, -Z" "\n\t"
      "ld r16, -Z" "\n\t"
      "ld r15, -Z" "\n\t"
      "ld r14, -Z" "\n\t"
      "ld r13, -Z" "\n\t"
      "ld r12, -Z" "\n\t"
      "ld r11, -Z" "\n\t"
      "ld r10, -Z" "\n\t"
      "ld r9, -Z" "\n\t"
      "ld r8, -Z" "\n\t"
      "ld r7, -Z" "\n\t"
      "ld r6, -Z" "\n\t"
      "ld r5, -Z" "\n\t"
      "ld r4, -Z" "\n\t"
      "ld r3, -Z" "\n\t"
      "ld r2, -Z" "\n\t"
      "ld r1, -Z" "\n\t"
      "ld r0, -Z" "\n\t"
      "pop r30" "\n\t"
      "pop r31" "\n\t"
      "reti"
      : : "x"(&task->stack));
}

#define servo(port,number,position)   (PORT_ON(port,number), delay_us(position), PORT_OFF(port,number))
enum port_t
  {
    // Port A is free for use, direct battery powered heads, digital
    A = 0,
    // B0 is connected to the SPI header
    // B1, B2, and B3 are one side of the programmer port
    // B0 through 3 also for the SPI bus being respectively:
    //    SS, SCK, MOSI, and MISO
    // B4 and B5 are not connected
    // B6 is the LED
    // B7 is not connected
    B,
    // Port C is free for use, direct battery powered heads, digital
    C,
    // D0 is I2C SCL
    // D1 is I2C SCA
    // D2 is USBTX
    // D3 is USBRX
    // D4 not connected (ICP1?)
    // D5 not connected (XCK1?)
    // D6 is timer 1
    // D7 is timer 0 and connected to T0 pin
    D,
    // Port E is free for use, direct battery powered heads, digital
    E,
    // Port F is free for use, regulated 5V, numbered 0-7, analog/digital
    F,
    // G5 is the only connected port, button, digital
    G,
    // H0 is RX on UART2
    // H1 is TX on UART2
    // H2 is battery powered, digital
    // H3 though 6 are battery powered heads, digital
    //  connected to PWMs: 4A, 4B, 4C, 2B respectively
    // H7 is timer 4
    H,
    // No I port
    // I,
    // J0 is RX on UART3
    // J1 is TX on UART3
    // J2 through J5 are not connected
    // J6 is battery powered, digital
    // J7 is not connected
    J,
    // Port K is free for use, regulated 5V, numbered 8-15, analog/digital
    K,
    // L0 is not conneted (ICP4?)
    // L1 is not conneted (ICP5?)
    // L2 is timer 5
    // L3 through L7 are not connected
    L
  };

#ifdef __DEBUG_PORT_MISSES__
#define ERROR(fmt, ...)				\
  rprintf("Error encountered %s:%d in %s" fmt, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__);
#else
#define ERROR(fmt, ...)
#endif

#define PIN_VOLATILE

#define SWITCH_OVER_PORTS(port,obj)			\
    switch(port)					\
    {							\
    case A: SWITCH_OP(obj##A); break;			\
    case B: SWITCH_OP(obj##B); break;			\
    case C: SWITCH_OP(obj##C); break;			\
    case D: SWITCH_OP(obj##D); break;			\
    case E: SWITCH_OP(obj##E); break;			\
    case F: SWITCH_OP(obj##F); break;			\
    case G: SWITCH_OP(obj##G); break;			\
    case H: SWITCH_OP(obj##H); break;			\
    case J: SWITCH_OP(obj##J); break;			\
    case K: SWITCH_OP(obj##K); break;			\
    case L: SWITCH_OP(obj##L); break;			\
    }

#define SOME_PORT(port,obj)				\
  switch(port)						\
  {									\
  case A: SWITCH_OP(DDRA,PORTA,PINA,obj##A); break;			\
  case B: SWITCH_OP(DDRB,PORTB,PINB,obj##B); break;			\
  case C: SWITCH_OP(DDRC,PORTC,PINC,obj##C); break;			\
  case D: SWITCH_OP(DDRD,PORTD,PIND,obj##D); break;			\
  case E: SWITCH_OP(DDRE,PORTE,PINE,obj##E); break;			\
  case F: SWITCH_OP(DDRF,PORTF,PINF,obj##F); break;			\
  case G: SWITCH_OP(DDRG,PORTG,PING,obj##G); break;			\
  case H: SWITCH_OP(DDRH,PORTH,PINH,obj##H); break;			\
  case J: SWITCH_OP(DDRJ,PORTJ,PINJ,obj##J); break;			\
  case K: SWITCH_OP(DDRK,PORTK,PINK,obj##K); break;			\
  case L: SWITCH_OP(DDRL,PORTL,PINL,obj##L); break;			\
  }

#define SOME_DIGITAL_PORT(port,obj)					\
  switch(port)								\
  {									\
  case A: SWITCH_OP(DDRA,PORTA,PINA,obj##A); break;			\
  case C: SWITCH_OP(DDRC,PORTC,PINC,obj##C); break;			\
  case E: SWITCH_OP(DDRE,PORTE,PINE,obj##E); break;			\
  case H: SWITCH_OP(DDRH,PORTH,PINH,obj##H); break;			\
  case J: SWITCH_OP(DDRH,PORTH,PINH,obj##J); break;			\
  default: ERROR("SOME_DIGITAL_PORT %d", port);				\
  }

#define SOME_ANALOG_PORT(port,obj)					\
  switch(port)								\
  {									\
  case F: SWITCH_OP(DDRF,PORTF,PINF,obj##F); break;			\
  case K: SWITCH_OP(DDRK,PORTK,PINK,obj##K); break;			\
  default: ERROR("SOME_ANALOG_PORT %d", port);				\
  }

/* The digital-only pins are split into 2 types, the EH size for
   servos and the ACJ side for sensors */

#define SOME_SERVO_PORT(port,obj)					\
  switch(port)								\
  {									\
  case E: SWITCH_OP(DDRE,PORTE,PINE,obj##E); break;			\
  case H: SWITCH_OP(DDRH,PORTH,PINH,obj##H); break;			\
  default: ERROR("SOME_SERVO_PORT %d", port);				\
  }

#define SOME_DEVICE_PORT(port,obj)					\
  switch(port)								\
  {									\
  case A: SWITCH_OP(DDRA,PORTA,PINA,obj##A); break;			\
  case C: SWITCH_OP(DDRC,PORTC,PINC,obj##C); break;			\
  case J: SWITCH_OP(DDRH,PORTH,PINH,obj##J); break;			\
  default: ERROR("SOME_DEVICE_PORT %d", port);				\
  }

#define ALWAYS_INLINE_PIN_OPS

#ifdef INLINE_PIN_OPS
#define INLINE_PINS(r,f) inline r f
#elif defined(ALWAYS_INLINE_PIN_OPS)
#define INLINE_PINS(r,f) inline r __attribute__((always_inline)) f
#else
#define INLINE_PINS(r,f) r f
#endif


INLINE_PINS(void,
	    set_input_pin(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) arg &= ~_BV(pin); return;
  SWITCH_OVER_PORTS(port, DDR)
#undef SWITCH_OP
}

INLINE_PINS(void,
	    set_output_pin(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) arg |= _BV(pin); return;
  SWITCH_OVER_PORTS(port, DDR);
#undef SWITCH_OP
}

INLINE_PINS(uint8_t,
	    read_port(const PIN_VOLATILE enum port_t port))
{
#define SWITCH_OP(arg) return _SFR_BYTE(arg);
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d", port);
  return 0;
}

INLINE_PINS(uint8_t,
	    read_pin(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) return bit_is_set(arg, pin);
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d %d", port, pin);
  return 0;
}

INLINE_PINS(void ,
	    write_port(const PIN_VOLATILE enum port_t port, const uint8_t val))
{
#define SWITCH_OP(arg) arg = val; return;
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d %x", port, val);
}

INLINE_PINS(void,
	    pin_on(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) arg |= _BV(pin); return;
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d %d", port, pin);
}

INLINE_PINS(void,
	    pin_off(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) arg &= ~_BV(pin); return;
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d %d", port, pin);
}

INLINE_PINS(void,
	    pin_flip(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{
#define SWITCH_OP(arg) arg ^= (1 << pin); return;
  SWITCH_OVER_PORTS(port, PORT);
#undef SWITCH_OP
  ERROR("%d %d", port, pin);
}

INLINE_PINS(void,
	    wait_until_bit_is_set(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{do {} while(!read_pin(port, pin));}

INLINE_PINS(void,
	    wait_until_bit_is_clear(const PIN_VOLATILE enum port_t port, const uint8_t pin))
{do {} while(read_pin(port, pin));}

#if 0
// At least 200us must pass between reasds
int parallax_ping(const enum port_t port, const uint8_t pin)
{
  pin_off(port, pin);
  set_output_pin(port, pin);
  //delay_us(20);        //  Wait for 2 microseconds
  pin_on(port, pin);
  delay_us(5);       // Wait for 5 microseconds
  pin_off(port, pin);
  set_input_pin(port, pin);
  /* loop_until_bit_is_set(PINC, 0);     // Loop until the the PingPin goes high  (macro found in sfr_def.h) */
  wait_until_bit_is_set(port, pin);
  /* //clears timer, reset overflow counter */
  /* asm volatile ("/\*j*\/"); */
  reset_timer0();       //reset timer 0
  wait_until_bit_is_clear(port, pin);
  /* asm volatile ("/\*k*\/"); */
  /* loop_until_bit_is_clear(PINC, pin);     // Loop until the the PingPin goes low  (macro found in sfr_def.h) */
  /* asm volatile ("/\*l*\/"); */
  //read timer0's overflow counter
  //255 is count before overflow, dependent on clock
  return (get_timer0_overflow()*255+TCNT0) * 2.068965517;//elapsed time x conversion
}
#endif

// At least 200us must pass between reasds
int parallax_ping(const volatile enum port_t port_nr, const volatile uint8_t pin_nr)
{
#define SWITCH_OP(ddr, port, pin, arg)				\
  {								\
    PORT_ON(ddr, pin_nr);					\
    PORT_OFF(port, pin_nr);					\
    delay_us(2);						\
    PORT_ON(port, pin_nr);					\
    delay_us(5);						\
    PORT_OFF(port, pin_nr);					\
    FLIP_PORT(ddr, pin_nr);					\
    loop_until_bit_is_set(pin, pin_nr);				\
    reset_timer0();						\
    loop_until_bit_is_clear(pin, pin_nr);			\
    return (get_timer0_overflow()*255+TCNT0) * 2.068965517;	\
  }
  SOME_DEVICE_PORT(port_nr,PORT);
#undef SWITCH_OP
  ERROR("%d %d", port, pin);
  return 0;
}

void control(void)
{
  char str[100];
  int offst = 0;
  bzero(str, 99);
  wdt_disable();
  /* servo_controller(); */

  rprintf("Sizeof port %d\r\n", sizeof(enum port_t));
  /* int analog_pulse = 900, analog_nr = 6; */
  reset_timer5();
  rprintf("\r\n> ");
  uartFlushReceiveBuffer(1);

  pin_off(B, 6);
  /* LED_on(); */

  while(1)
  {
    /* rprintf("counter %d\r\n", get_timer5_counter()); */

    /* if(button_pressed()) */
    /*   LED_on(); */
    /* else */
    /*   LED_off(); */

    // this acts strangely
    /* if(get_timer5_counter() > 2) */
    /* { */
    /*   servo(PORTC,analog_nr,analog_pulse); */
    /*   reset_timer5(); */
    /* } */

    str[offst] = uart1GetByte();

    a2dSetReference(ADC_REFERENCE_256V);
    delay_us(200);

    if(str[offst] != 255)
    {
      uart1SendByte(str[offst]);

      if(str[offst] == 8)
      {
	if(offst > 0)
	  offst--;
	continue;
      }

      if(str[offst] == 13 || str[offst] == 10)
      {
	rprintf("\r\n");

	for(int i = 0; i < offst + 1; ++i)
	  uart1SendByte(str[i]);
	rprintf("\r\n");

	if(str[0] == 'P')
	{
	  int nr, pulse;
	  sscanf(str + 1, "%d %d", &nr, &pulse);
	  rprintf("Setting servo %d to %d\r\n", nr, pulse);
	  servo(PORTC,nr,pulse);
	}

	if(str[0] == 'A' && str[1] != 'T')
	{
	  int nr, pulse, time, times;
	  sscanf(str + 1, "%d %d %d %d", &nr, &pulse, &time, &times);
	  if(nr<0) {offst = 0; continue;}
	  rprintf("Analog servo %d to %d\r\n", nr, pulse);
	  while(times--)
	  {
	    delay_ms(time);
	    servo(PORTC,nr,pulse);
	  }
	}

	if(str[0] == 'S')
	{
	  int nr, data, time, times;
	  sscanf(str + 1, "%d %x %d %d", &nr, &data, &time, &times);
	  if(nr<0) {offst = 0; continue;}
	  rprintf("Servo serial %d to %x time do \r\n", nr, data, time, times);
	  while(times--)
	  {
	    int td = data;
	    for(int i = 0; i < 8; ++i)
	    {
	      if(td&1)
		PORT_ON(PORTC,nr);
	      else
		PORT_OFF(PORTC,nr);
	      delay_ms(time);
	      td = td >> 1;
	    }
	    FLIP_PORT(DDRC, nr);
	    for(int i = 0; i < 10; ++i)
	    {
	      rprintf("%x\r\n",bit_is_set(PINC,nr));
	      delay_ms(time);

	    }
	    rprintf("%x\r\n",bit_is_set(PINC,nr));
	  }
	}

	if(str[0] == 'Q')
	{
	  int times = 0;
	  sscanf(str + 1, "%d", &times);
	  for(int i = 0; i < times; ++i)
	  {
	    int ping = sonar_Ping();/* PINA, PORTA, DDRA, 3); */
	    rprintf("Ping %d\r\n", ping);
	    delay_ms(100);
	  }
	}

	if(str[0] == 'Z')
	{
	  int port, pin, times;
	  sscanf(str + 1, "%d %d %d", &port, &pin, &times);
	  for(int i = 0; i < times; ++i)
	  {
	    int ping = parallax_ping(port, pin);
	    rprintf("new Ping %d,%d %d\r\n", port, pin, ping);
	    delay_ms(100);
	  }
	}

	if(str[0] == 'I')
	{
	  int port, pin;
	  sscanf(str + 1, "%d %d", &port, &pin);
	  rprintf("Set %d,%d to input\r\n", port, pin);
	  set_input_pin(port, pin);
	}

	if(str[0] == 'O')
	{
	  int port, pin;
	  sscanf(str + 1, "%d %d", &port, &pin);
	  rprintf("Set %d,%d to output\r\n", port, pin);
	  set_output_pin(port, pin);
	}

	if(str[0] == 'R')
	{
	  int port, pin;
	  sscanf(str + 1, "%d %d", &port, &pin);
	  rprintf("Reading %d,%d\r\n", port, pin);
	  int val = read_pin(port, pin);
	  rprintf("Got %d\r\n", val);
	}

	if(str[0] == 'W')
	{
	  int port, pin, val;
	  sscanf(str + 1, "%d %d %d", &port, &pin, &val);
	  rprintf("Writing %d,%d val: %d\r\n", port, pin, val);
	  if(val)
	    pin_on(port, pin);
	  else
	    pin_off(port, pin);
	  printf("State is %d\r\n", read_pin(port, pin));
	}

	if(str[0] == 'F')
	{
	  int port, pin;
	  sscanf(str + 1, "%d %d", &port, &pin);
	  rprintf("Flipping %d,%d\r\n", port, pin);
	  pin_flip(port, pin);
	}

	if(str[0] == 'T')
	{
	  int times, port;
	  sscanf(str + 1, "%d %d", &times, &port);
	  for(int i = 0; i < times; ++i)
	  {
	    int touch = a2dConvert8bit(port);
	    rprintf("Touch %d\r\n", 255 - touch);
	    delay_ms(100);
	  }
	}

	offst = 0;
	bzero(str, 99);
	rprintf("> ");
      }
      else
      {
	if(offst > 90)
	  offst=0;
	else
	  offst++;
      }
    }
  }
}
