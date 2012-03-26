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
 #include "axon_DAQ.c" //use the Axon like a data acquisition device
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

void control(void)
{
  reset_timer5();
  uartFlushReceiveBuffer(1);

  a2dSetReference(ADC_REFERENCE_256V);
  delay_us(200);

  PORT_OFF(PORTB,7);

  while(1)
  {
    char read = uart1GetByte();

    if(read == 'Q') {
      /* Unimplemented */
#if 0
      rprintf("%d\r\n", sonar_Ping());
#endif
    } else if(read == 'L') {
      rprintf("%d\r\n", 255 - a2dConvert8bit(0));
    } else if(read == 'R') {
      rprintf("%d\r\n", 255 - a2dConvert8bit(1));
    } else if(read == 'G') {
      rprintf("%d %d %d\r\n", 255 - a2dConvert8bit(2), 255 - a2dConvert8bit(3), 255 - a2dConvert8bit(4));
    } else if(read == 'A') {
      rprintf("%d %d %d\r\n", 255 - a2dConvert8bit(5), 255 - a2dConvert8bit(6), 255 - a2dConvert8bit(7));
    } else if(read == 'H') {
      rprintf("%d %d %d\r\n", 255 - a2dConvert8bit(8), 255 - a2dConvert8bit(9), 255 - a2dConvert8bit(10));
    } else if(read == 'B') {
      rprintf("%d %d %d\r\n", 255 - a2dConvert8bit(11), 255 - a2dConvert8bit(12), 255 - a2dConvert8bit(13));
    } else if(read == 'P') {
      PORT_ON(PORTB,7);
    } else if(read == 'p') {
      PORT_OFF(PORTB,7);
    }

    delay_us(50);
  }
}
