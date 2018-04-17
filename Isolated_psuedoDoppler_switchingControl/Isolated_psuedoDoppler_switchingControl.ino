/* Antenna Switching Control Unit
Created by: Marcel Stieber AI6MS 2012
Based on code from: David A. Mellis 2005 and Paul Stoffregen 2010 */

//Pin number assignments
const int refPin = 9;
const int ledPin1 = 10;
const int ledPin2 = 11;
const int ledPin3 = 12;
const int ledPin4 = 13;

long interval = 474; // switching delay used to set frequency (us)

void setup() {
	// set pins to output
	 pinMode(refPin, OUTPUT);
	 pinMode(ledPin1, OUTPUT);
	 pinMode(ledPin2, OUTPUT);
	 pinMode(ledPin3, OUTPUT);
	 pinMode(ledPin4, OUTPUT);
	 pinMode(refPin, OUTPUT);
	 //set initial pin states to LOW
	 digitalWrite(refPin, LOW);
	 digitalWrite(ledPin1, LOW);
	 digitalWrite(ledPin2, LOW);
	 digitalWrite(ledPin3, LOW);
	 digitalWrite(ledPin4, LOW);
}

void loop()
{
	 //Cycle through each pin progressively with delay between
	 //reference pin only switches every 2 cycles for 50% 500Hz wave
	 digitalWrite(refPin, HIGH);
   digitalWrite(ledPin4, LOW);
	 digitalWrite(ledPin1, HIGH);
	 digitalWrite(ledPin2, LOW);
	 digitalWrite(ledPin3, LOW);
	 delayMicroseconds(interval);

	 digitalWrite(refPin, LOW);
	 digitalWrite(ledPin1, LOW);
	 digitalWrite(ledPin2, HIGH);
	 digitalWrite(ledPin3, LOW);
	 digitalWrite(ledPin4, LOW);
	 delayMicroseconds(interval);

	 digitalWrite(refPin, LOW);
	 digitalWrite(ledPin1, LOW);
	 digitalWrite(ledPin2, LOW);
	 digitalWrite(ledPin3, HIGH);
	 digitalWrite(ledPin4, LOW);
	 delayMicroseconds(interval);
	 
	 digitalWrite(refPin, LOW);
	 digitalWrite(ledPin1, LOW);
	 digitalWrite(ledPin2, LOW);
	 digitalWrite(ledPin3, LOW);
	 digitalWrite(ledPin4, HIGH);
	 delayMicroseconds(interval);
}
