/*
 Author:	Dr. G
*/

//#include <MIDI.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include "main.h"
#include "configuration.h"
#include "network.h"


unsigned long MillisecondTicks{};
unsigned long MicrosecondTicks{};
unsigned long LastMillisecondTicks{};//previous values
unsigned long LastMicrosecondTicks{};

//System clock
void GetTicks(void)
{
	LastMillisecondTicks = MillisecondTicks;
	LastMicrosecondTicks = MicrosecondTicks;

	MillisecondTicks = millis();
	MicrosecondTicks = micros();
}

//LED debugging
void BlinkLED(unsigned short time)
{
    //how much time until we turn the LED off
    static unsigned short timeout = 0;

    if(time != 0)
    {
        timeout = time;
    }
    else
        if(LastMillisecondTicks != MillisecondTicks &&
            timeout > 0)
            --timeout;


    if(timeout)
            digitalWrite(LED_BUILTIN, LED_ON_STATE);
        else
            digitalWrite(LED_BUILTIN, !LED_ON_STATE);


}



void setup() {
  
  initNetwork();
  pinMode(22, OUTPUT);

}

void loop() {
  
  GetTicks();
  handleNetwork();

  //BlinkLED(0);//process the blinking of the LED

  //if(MillisecondTicks % 1000 == 0 && LastMillisecondTicks != MillisecondTicks)
  //  BlinkLED(30);

  digitalWrite(22, getSwitchState());

}