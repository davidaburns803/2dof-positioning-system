#include "LaserSwitch.h"
#include "Pins.h"

void initLaser() {
    pinMode(laserswitch, OUTPUT);
    digitalWrite(laserswitch, LOW);
}

void laserOn() {
    digitalWrite(laserswitch, HIGH);
}

void laserOff() {
    digitalWrite(laserswitch, LOW);
}

void setLaser(bool state) {
    digitalWrite(laserswitch, state ? HIGH : LOW);
}