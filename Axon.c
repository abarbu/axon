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
  uartSetBaudRate(0, 9600); // set UARTE speed, for Bluetooth
  uartSetBaudRate(1, 115200); // set UARTD speed, for USB connection, up to 500k, try 115200 if it
  uartSetBaudRate(2, 57600); // set UARTH speed
  uartSetBaudRate(3, 57600); // set UARTJ speed, for Blackfin
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
  delay_us(100);

  PORT_OFF(PORTB,7);

  while(1) {
    char read = uart1GetByte();
    if(read == 'U') { //Rangefinder only for the Hand! We only have 3 uart ports!
      uart0SendByte(0x22); //sends 4 bytes with the command to read distance (0x22)
      uart0SendByte(0x00); uart0SendByte(0x00);uart0SendByte(0x22);delay_us(50);
      uart0GetByte(); //we don't care about this byte
      rprintf("%d\r\n", 255*uart0GetByte() + uart0GetByte());
      uart0GetByte(); //we don't care about this byte
    } else if(read == 'L') {
      rprintf("%d\r\n", 255 - a2dConvert8bit(0));
    } else if(read == 'R') {
      rprintf("%d\r\n", 255 - a2dConvert8bit(1));
    } else if(read == 'D') {
      int buff='#';
      uart2SendByte('#'); uart2SendByte('o'); uart2SendByte('s');
      uart2SendByte('#'); uart2SendByte('f');
      delay_ms(60); // Do not change this delay or you'll get in trouble! 
      while(1) {
	buff = uart2GetByte();
	if(buff==10 || buff==13 || buff==-1) {
	  uart1SendByte('\r');
	  uart1SendByte('\n');
	  break;
	}
	uart1SendByte(buff);
      }
      while((buff=uart2GetByte()) != -1);
    }  else if(read == 'E') {
      int buff='#';
      uart3SendByte('#'); uart3SendByte('o'); uart3SendByte('s');
      uart3SendByte('#'); uart3SendByte('f');
      delay_ms(60); // Do not change this delay or you'll get in trouble! 
      while(1) {
	buff = uart3GetByte();
	if(buff==10 || buff==13 || buff==-1) {
	  uart1SendByte('\r');
	  uart1SendByte('\n');
	  break;
	}
	uart1SendByte(buff);
      }
      while((buff=uart3GetByte()) != -1);
    } else if(read == 'M') {
      int buff='#';
      uart2SendByte('#'); uart2SendByte('o'); uart2SendByte('t');
      uart2SendByte('#'); uart2SendByte('f');
      delay_ms(60); // Do not change this delay or you'll get in trouble! 
      while(1) {
	buff = uart2GetByte();
	if(buff==10 || buff==13 || buff==-1) {
	  uart1SendByte('\r');
	  uart1SendByte('\n');
	  break;
	}
	uart1SendByte(buff);
      }
      while((buff=uart2GetByte()) != -1);
    } else if(read == 'N') {
      int buff='#';
      uart3SendByte('#'); uart3SendByte('o'); uart3SendByte('t');
      uart3SendByte('#'); uart3SendByte('f');
      delay_ms(60); // Do not change this delay or you'll get in trouble! 
      while(1) {
	buff = uart3GetByte();
	if(buff==10 || buff==13 || buff==-1) {
	  uart1SendByte('\r');
	  uart1SendByte('\n');
	  break;
	}
	uart1SendByte(buff);
      }
      while((buff=uart3GetByte()) != -1);
    } else if(read == 'P') {
      PORT_ON(PORTB,6);
    } else if(read == 'p') {
      PORT_OFF(PORTB,6);
    } else if(read == 'Q') {
      PORT_ON(PORTB,7);
    } else if(read == 'q') {
      PORT_OFF(PORTB,7);
    } else if(read == 'X') {
      PORT_ON(PORTB,4);
    } else if(read == 'x') {
      PORT_OFF(PORTB,4);
    } else if(read == 'Y') {
      PORT_ON(PORTB,5);
    } else if(read == 'y') {
      PORT_OFF(PORTB,5);
    } 
    delay_us(50);
  }
}
