#include <stdio.h>
#include <wiringPi.h>
#include <errno.h>


#define LED 0; // define 17(0 for wiringPi package) to LED port
#define SENSOR 1; // define 18(1 for wiringPi package) to Sensor signal port
#define HIGH 1;
#define LOW 0;

void interrupt(void) // Turn LED on
{
	printf("Implement")
	digitalWrite(LED, HIGH);
	delay(10000); // Wating LED value as 1 for 10000ms
}

int main() {
	if (wiringPiSetup() == -1) return 1; // if wire does not setting, Finish

	pinMode(LED, OUTPUT); // setting Pin 17 port to Output
	pinMode(SENSOR, INPUT); // setting Pin 18 port to Input

	if (wiringPiISR(SENSOR, INT_EDGE_RIGING, &interrupt) < 0) { // when Detecting movement
		printf("Detecting something");
		return 1;
	}
	while (1) {
		digitalWrite(LED, LOW);
		delay(300);
	}
	return 0;
}