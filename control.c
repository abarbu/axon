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

//Add your code here

#include <stdlib.h>
#include <string.h>

unsigned int sensor_left=128;
unsigned int sensor_right=128;
int auto_calib_L=0;
int auto_calib_R=0;

void update_sensors(void);
void photovore(void);
void test_code(void);

void bzero(char* x, int y)
{
  while(y--) *x++ = 0;
}

//enter your code in control here
//photovore is just a default sample program and can be deleted
void control(void)
{
  char str[100];
  int offst = 0;
  bzero(str, 99);

  /* servo_controller(); */

  int analog_pulse = 900, analog_nr = 6;
  reset_timer5();
  rprintf("\r\n> ");

  while(1)
  {
    /* rprintf("counter %d\r\n", get_timer5_counter()); */

    if(button_pressed())
      LED_on();
    else
      LED_off();

    // this acts strangely
    /* if(get_timer5_counter() > 2) */
    /* { */
    /*   servo(PORTC,analog_nr,analog_pulse); */
    /*   reset_timer5(); */
    /* } */

    str[offst] = uart1GetByte();

    a2dSetReference(ADC_REFERENCE_256V);
    delay_us(200);

    if(str[offst] != 255 && str[offst] != -1)
    {
      uart1SendByte(str[offst]);

      if(str[offst] == 13)
      {
	rprintf("\r\n");

	for(int i = 0; i < offst + 1; ++i)
	  uart1SendByte(str[i]);
	rprintf("\r\n");

	/* if(str[0] == 'A') */
	/* { */
	/*   int nr, pulse; */
	/*   sscanf(str + 1, "%d %d", &nr, &pulse); */
	/*   rprintf("Setting analog servo %d to %d\r\n", nr, pulse); */
	/*   analog_nr = nr; */
	/*   analog_pulse = pulse; */
	/* } */
	if(str[0] == 'P')
	{
	  int nr, pulse;
	  sscanf(str + 1, "%d %d", &nr, &pulse);
	  rprintf("Setting servo %d to %d\r\n", nr, pulse);
	  servo(PORTC,nr,pulse);
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

	if(str[0] == 'T')
	{
	  int times = 0;
	  sscanf(str + 1, "%d", &times);
	  for(int i = 0; i < times; ++i)
	  {
	    int touch = a2dConvert8bit(7);
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

void update_sensors(void)
{
  sensor_right=a2dConvert8bit(2)+auto_calib_R;
  sensor_left=a2dConvert8bit(14)+auto_calib_L;//right sensor is off, so I fudged it . . . mmmmm fudge!
}

void photovore(void)
{
  //initialize values
  long unsigned int wheel_left=500;
  long unsigned int wheel_right=500;
  unsigned int threshold = 8;

  //autocalibrate makes both start at equal values (so must start in same lighting)
  update_sensors();
  if (sensor_left>sensor_right)
    auto_calib_R=sensor_left-sensor_right;
  else
    auto_calib_L=sensor_right-sensor_left;


  while(1)
  {
    //get sensor data
    update_sensors();

    //detects more light on left side of robot
    if(sensor_left > sensor_right && (sensor_left - sensor_right) > threshold)
    {//go left
      wheel_left=550;
      wheel_right=550;
    }

    //detects more light on right side of robot
    else if(sensor_right > sensor_left && (sensor_right - sensor_left) > threshold)
    {//go right
      wheel_left=850;
      wheel_right=850;
    }

    //light is about equal on both sides
    else
    {//go straight
      wheel_left=600;
      wheel_right=800;
    }

    //output data to USB (use hyperterminal to view it)
    rprintf("L_Sensor=%d, L_wheel=%d%d, R_Sensor=%d, R_wheel=%d%d\r\n",sensor_left, wheel_left, sensor_right, wheel_right);

    //command servos
    wheel_left(wheel_left);
    wheel_right(wheel_right);
  }
}
