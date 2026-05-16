#include <Arduino.h>
#include "driver/gpio.h"

#define laserswitch 33

void setup() {
pinMode(laserswitch, OUTPUT);
digitalWrite(laserswitch, LOW);
}

void loop() {
digitalWrite(laserswitch, HIGH);
delay(1000);

digitalWrite(laserswitch, LOW);
delay(2000);

}