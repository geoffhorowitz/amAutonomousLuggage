/*
         Isolated_masterControl
            
         Program Objective:
             Program runs the master microcontroller, compiling information from object avoidace, course stabalization, and communicating with the  psuedo-doppler program to safely and effectively reach target beacon.

             NOTE: This program ALSO runs the IMU course stabalization control algorithm since that only involves the IMU input via I2C communication.
                ---> IMU: I2C communication via A4 & A5
                            //Connect SCL to analog 5 - A5 - black
                            //Connect SDA to analog 4 - A4 - Yellow
                ---> Object Avoidance: Digital communication via the pins listed below
                ---> Pseudo-Doppler: Serial communication via the Rx and Tx pins
          
            IMU Program Objective:
             Program runs the IMU and course stabalization control algorithms.  It uses the IMU to maintain a steady course

 
 */

 /*      Motor Driver notes:
            This code assumes the sabertooth2x25 motor driver is running in analog mode:
            ---switches 1 & 2 should be UP (direct drive, not boat-twin-screw-engine drive)
            ---switch 3 (depends on battery)
            ---switch 4 should be DOWN (independent drive mode)
            ---switch 5 should be DOWN (exponential response - softens control around low/0 speed)
            ---switch 6 should be UP (no limiting of sensitivity range)

            ---use low-pass filter on pwm signal

            Additionally, note that 2.5V input (50%pwm) corresponds to full stop
            ----> over 2.5V (>50%pwm) will make the motors go forward
            ----> under 2.5V will make the motors go backward

*/

// Inclusions and Definitions
#include <Wire.h>
#include <Math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

	//Pin Defns

      //User Input
#define moveButton 22      //system start activation switch

    	//ultrasonics
#define ultraTrig0 5      //ultrasonic sensor trigger - ahead - Ultra 3
#define ultra0echo 4      //ultrasonic sensor echo, 0 degrees (straight ahead) - Ultra 3

#define ultraTrigL30 3   	//ultrasonic sensor trigger - L30  - Ultra 2
#define ultraL30echo 2   	//ultrasonic sensor echo, 30 degrees left of driver  - Ultra 2

#define ultraTrigR30 7   	//ultrasonic sensor trigger - R30  - Ultra 1
#define ultraR30echo 6   	//ultrasonic sensor echo, 30 degrees right of driver  - Ultra 1

#define ir0read 23        //backup ir sensor -----------> IF USING INTERRUPT ON UNO, MAKE SURE ITS ATTACHED TO PIN 2 (INTERRUPT PIN)

		//LEDs
#define centerLED 10
#define leftLED 12
#define rightLED 11
#define flashingLightDelay 100

      //motor
#define leftMotor 8      //left side of driver motor  - S1 - Yellow
#define rightMotor 9     //right side of driver motor  - S2 - White

      //IMU Pins
//Connect SCL to analog 5 - A5 - black
//Connect SDA to analog 4 - A4 - Yellow

	//Variable Defns
		//motor
                         	//pwm 0-255 - note 50% is full stop, below 50% is reverse, above 50% is forward
#define pwmForward 210    	//produce motor forward voltage
#define pwmReverse 100    	//produce motor reverse voltage
#define pwmOff 127    		//produce motor stop voltage

	    //Delay specifications
#define program_start_delay_ms 2000 		//how long after the button push should the program start
#define object_removal_delay_ms 500 		//how long after an object is moved (or bot is reoriented) should the program wait to respond
#define BNO055_samplerate_delay_ms 100 	//Set the delay between fresh samples
#define debug_delay 0 					//delay in program at some debug points for easier visualization of data
#define serial_read_delay 100				//delay in reading after data becomes available - allows all serial sent to be received together

		//object avoidance parameters
#define cutoffDist 1 	 	//in meters - cutoff distance b 0 which object must be avoided
#define numMovingAvg 1   //number of items used to comput ultrasonic average

    	//Deviation from course parameters
#define max_deviation 2         		//in degrees - allows for maximum deviation from course by +- degrees
#define beaconOffValue 255

		//P-D/IMU calibration data
#define maxDeviation 2

	    //PID Control Variables
#define kp 1.2    //proportional gain
#define kd .5    //derivative gain
#define ki .2    //integral gain
#define maxGain 1
#define gainTimeout 3000

  	  //Baud Rates
//#define serial_baud 500000  //This is the same baud rate as the Psuedo-Doppler code
#define debugSerial Serial
#define debugSerial_baud 115200
#define commSerial Serial1
#define commSerial_baud 115200

#define debug true        //debug code
#define debugMotor true
#define debugUltra true
#define debugPD true
#define debugIMU false
#define makeSurePDReading false

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Program Setup Information

//create a BNO055 Object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Variable declarations
int buttonState;

    //course heading variables
int pdCalibratedZero = 0;
double currentHeading = 0; //stores IMU heading
double courseHeading = 0; //stores PD heading
double error = 0;
int headingCorrection = 0; //Resume course (after Object) flag
double headingRecall = 0; //Resume course (after Object) variable
int resetHeading = 0; //steady course flag
double steadyCourseHeading = 0; //steady course variable

    //serial communication variables
byte val = beaconOffValue;

    //PID variables
double previousError = 0;
double cumError = 0;
double pGain = 0;
double dGain = 0;
double iGain = 0;
double gain = 0;

    //ultrasonic variables
double tof = 0;  //time of flight for echo pulse
double dist0 = 0;
double distL30 = 0;
double distR30 = 0;
int ir = 0;

    //time variables
unsigned long currentTime;
unsigned long previousTime;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Function
void setup() {
	//set up pin modes
    pinMode(moveButton, INPUT_PULLUP);
    pinMode(ultra0echo, INPUT);
    pinMode(ultraR30echo, INPUT);
    pinMode(ultraL30echo, INPUT);
    pinMode(ultraTrig0, OUTPUT);
    pinMode(ultraTrigL30, OUTPUT);
    pinMode(ultraTrigR30, OUTPUT);
    pinMode(rightMotor, OUTPUT);
    pinMode(leftMotor, OUTPUT);
    pinMode(centerLED, OUTPUT);
    pinMode(leftLED, OUTPUT);
    pinMode(rightLED, OUTPUT);

    //initialize motors off
    setRightMotor(pwmOff);
    setLeftMotor(pwmOff);

    //set LEDs all on to indicate system on
    setLEDs(HIGH, HIGH, HIGH);
	
	//setup serial
    debugSerial.begin(debugSerial_baud);
    commSerial.begin(commSerial_baud);

    if(debug){
        debugSerial.println("Serial communications set up.");
        
        if(debugIMU && !bno.begin()) {
            debugSerial.println("Ooops, no BNO055 detected ... Check wiring!");
            while(1);
        }
    }//end if debug

    while(makeSurePDReading){
      getCourseHeading();
      if (debugSerial.read()==97){//escape while loop if 'a' is sent from debug serial
        break;
      }
    }
    
    //run Psuedo-Doppler/IMU calibration routine
    //pd_calibration_routine();

    //set LEDs all off to begin main loop
    setLEDs(LOW, LOW, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main Loop
void loop() {    
    //read button input
    buttonState = digitalRead(moveButton);
  	//buttonState = LOW;

    ledWaterfall(flashingLightDelay);

    if (debug) {
      //displayCalibrationStatus();
      debugSerial.println("Program ready.  Waiting for go button");
      debugSerial.println();
    }//end if debug

    //if start button is pressed, wait a few seconds for user to move if needed
    if (buttonState == LOW){
      	setLEDs(LOW, HIGH, LOW);
      	delay(program_start_delay_ms);

        if (debug && debugIMU) {
          displayCalibrationStatus();
          debugSerial.println();
          debugSerial.println("Program Enabled.  Lets get ready to rumble...");
          debugSerial.println();
        }//end if debug
    }//end if start button

  
    //main while loop
    while (buttonState == LOW) {

        //Find current heading
        currentHeading = getCurrentHeading();
                
        //Find course heading from Pseudo-Doppler
        courseHeading = getCourseHeading();
      
        dist0 = getUltraSonicDist(ultraTrig0, ultra0echo, numMovingAvg);  
        distR30 = getUltraSonicDist(ultraTrigR30, ultraR30echo, numMovingAvg);  
        distL30 = getUltraSonicDist(ultraTrigL30, ultraL30echo, numMovingAvg);  
                
        if(courseHeading == beaconOffValue){
                    //bot is not recieving a signal from the P-D --> Do not move
                    setRightMotor(pwmOff);
                    setLeftMotor(pwmOff);

                    if (debug && debugMotor) {
                        debugSerial.println("No Beacon Signal - Stay Put");
                    }//end if debug

                    resetHeading = 1;
                    steadyCourseHeading = currentHeading;
        }else if (dist0<=cutoffDist || distR30<=cutoffDist  || distL30<=cutoffDist ) {//if there is a object in front of the bot, enter object avoidance control loop
                ////////////////////Avoidance Control
                setLEDs(LOW, LOW, LOW);
                objectAvoidanceAlgorithm();
                
                headingCorrection = 1;
                resetHeading = 1;
                
                if(headingCorrection==0){
                  headingRecall = currentHeading;
                }
                
                //setLEDs(LOW, HIGH, LOW);
                
        } else if (headingCorrection == 1) { //if we hit an object, resume previous course heading pre-object

                ////////////////Course Resumption/Steady Course Algorithm
                currentHeading = getCurrentHeading();
                headingCorrection = resumePreviousCourse(currentHeading, headingRecall);
                
        } else { //if there isn't an object infront of the bot, have the bot keep a steady course or track beacon

                ///////////////PD Tracking/Steady Course Algorithm
                if (debug && debugUltra){
                    debugSerial.print("No Objects.  Steady Course Control");
                    delay(debug_delay);
                }// end if debug
   
                //Compare current heading to required course heading
                //error = courseHeading - pdCalibratedZero;
                error = courseHeading;
                
                if(abs(error)<=maxDeviation){
                  error = 0;
                }
                
                if (debug && debugIMU){
                    debugSerial.print("currentHeading: "); debugSerial.print(currentHeading);
                    debugSerial.print("\tcourseHeading: "); debugSerial.print(courseHeading);
                    //debugSerial.print("\tError: "); debugSerial.println(error);
                }//end if debug


                //Apply control algorithm to motor output
                if(courseHeading == beaconOffValue){
                    //bot is not recieving a signal from the P-D --> Do not move
                    setRightMotor(pwmOff);
			      	      setLeftMotor(pwmOff);

                    if (debug && debugMotor) {
                        debugSerial.println("No Beacon Signal - Stay Put");
                    }//end if debug

                    resetHeading = 1;
                    steadyCourseHeading = currentHeading;

                } else if (error > 0) {
                	  //Caluclate necessary control gains
                	  gain = calculatePIDgain(error);

                    //bot needs to turn right - add power to left motor
                    setRightMotor(pwmOff);
			      	      setLeftMotor(pwmForward+gain*pwmForward);

                    if (debug && debugMotor) {
                        debugSerial.print("Off Course - Turn Right");
                        debugSerial.print("\tGain: "); debugSerial.print(gain);
                        debugSerial.print("\tcourseHeading: "); debugSerial.print(courseHeading);
                        //debugSerial.print("\tError: "); debugSerial.println(error);
                    }//end if debug

                    resetHeading = 1;
                    steadyCourseHeading = currentHeading;
                      
                } else if(error < 0) {
            		    //Caluclate necessary control gains
            		    gain = calculatePIDgain(error);

                    //bot needs to turn left - add power to right motor
                    setRightMotor(pwmForward+gain*pwmForward);
		      		      setLeftMotor(pwmOff);
                  
                    if (debug && debugMotor) {
                        debugSerial.print("Off Course - Turn Left");
                        debugSerial.print("\tGain: "); debugSerial.print(gain);
                        debugSerial.print("\tcourseHeading: "); debugSerial.print(courseHeading);
                        //debugSerial.print("\tError: "); debugSerial.println(error);
                    }//end if debug

                    resetHeading = 1;
                    steadyCourseHeading = currentHeading;
                      
                } else {
                    if (debug && debugMotor) {
                        debugSerial.println("On Course");
                        debugSerial.print("\tcourseHeading: "); debugSerial.println(courseHeading);
                    }//end if debug

                    if(resetHeading == 1){
                        setRightMotor(pwmForward);
                        setLeftMotor(pwmForward);
                        
                        steadyCourseHeading = currentHeading;
                        resetHeading = 0;
                        
                    }else{
                        steadyCourseAlgorithm(currentHeading, steadyCourseHeading);
                    }

                }//end if PD tracking
        } // end if Object Avoidance/Course Correction/PD Tracking
    }//end main while (wait for button input)
}//end loop function


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Sub-functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setRightMotor(int pwmValue){
    analogWrite(rightMotor, pwmValue);

  	if (debug && debugMotor){
    	debugSerial.print("Right Motor: "); debugSerial.print(pwmValue);
  	}//end if debug
}

void setLeftMotor(int pwmValue){
  	analogWrite(leftMotor, pwmValue);

  	if (debug && debugMotor){
    	debugSerial.print("\tLeft Motor: "); debugSerial.println(pwmValue);
  	}//end if debug
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setLEDs(uint8_t right, uint8_t center, uint8_t left){
    digitalWrite(centerLED, center);
    digitalWrite(rightLED, right);
    digitalWrite(leftLED, left);
}

void ledWaterfall(int num){
    setLEDs(LOW,LOW,LOW);
    delay(num);
    setLEDs(HIGH,LOW,LOW);
    delay(num);
    setLEDs(HIGH,HIGH,LOW);
    delay(num);
    setLEDs(HIGH,HIGH,HIGH);
    delay(num);
    setLEDs(HIGH,HIGH,HIGH);
    delay(num);
    setLEDs(HIGH,HIGH,LOW);
    delay(num);
    setLEDs(HIGH,LOW,LOW);
    delay(num);
    setLEDs(LOW,LOW,LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getUltraSonicDist(int triggerPin, int echoPin, int n){
  //Function to get the distance measurement from the ultrasonic sensor

  double returnVal = 0;
  
  for(int i=0; i<n; i++){
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);
        tof = pulseIn(echoPin, HIGH, 2*6000);  //times out at 2m ==> 2*6000 ==>3000 us/m tof (there and back = 6000) therefore if no signal w/ 2* = 2m
        if(tof == 0){ // If we timed out
            //Serial.print("\t\t timing out:");
            tof = 57000;
            pinMode(echoPin, OUTPUT); // Then we set echo pin to output mode
            digitalWrite(echoPin, LOW); // We send a LOW pulse to the echo pin
            delayMicroseconds(200);
            pinMode(echoPin, INPUT); // And finaly we come back to input mode
        }
        returnVal += (tof/2)/2900;
  }
  returnVal = returnVal/n;
  return returnVal;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void objectAvoidanceAlgorithm(void){
    if (debug && debugUltra) {
            debugSerial.print("Objects detected"); 
            debugSerial.print("\tcenterUltra: "); debugSerial.println(dist0); 
            debugSerial.print("\trightUltra: "); debugSerial.print(distR30); 
            debugSerial.print("\tleftUltra: "); debugSerial.println(distL30);
            delay(debug_delay);
    }
  
    if(distR30>=cutoffDist){ //if the bot can turn right, turn right
        if (debug&& debugUltra) {
            Serial.print("Turning Right --> ");
        }
        setLEDs(HIGH, LOW, LOW);
        setRightMotor(pwmOff);
        setLeftMotor(pwmForward);

    } else if (distL30>=cutoffDist) { //if the bot cannot turn right but can turn left, turn left
        if (debug&& debugUltra) {
            Serial.print("Turning Left <-- ");
        }
        setLEDs(LOW, LOW, HIGH);
        setRightMotor(pwmForward);
        setLeftMotor(pwmOff);

    } else { //if the bot cannot turn in either direction, turn the motors off and reset the switch input
        if (debug&& debugUltra) {
            Serial.print("Staying Put -- ");
        }
        
        setRightMotor(pwmOff);
        setLeftMotor(pwmOff);
        buttonState = HIGH;
    } //end if ultraR30, ultraL30
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double getCurrentHeading(){
    sensors_event_t event;  //create sensor event
    bno.getEvent(&event); // get event from IMU

    currentHeading = event.orientation.x;

    if (currentHeading > 180){
          currentHeading = - 360 + currentHeading;
    }//end if current heading >180

    if (debug && debugIMU){
      Serial.print("Current IMU Heading: "); Serial.println(currentHeading);
    }

    return currentHeading;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getCourseHeading(void){
  if(commSerial.available()){
      val = commSerial.read();
      if (debug && debugPD){
        debugSerial.print("PD comm is working: ");
        debugSerial.println(val);
      }
  }
  //courseHeading = (double)val;
  //courseHeading = (courseHeading - 75)*2;
  
  //error>0 bot is turning right
  //error<0 bot is turning left
  //error=0 bot is going straight
  //error=255 bot is stopped
  
  if(val==1){
    courseHeading = -10;
    setLEDs(LOW, HIGH, HIGH); //LEFT
  }else if(val==2){
    courseHeading = 10;
    setLEDs(HIGH, HIGH, LOW);//RIGHT
  }else if (val==255){
    courseHeading = 255;
    setLEDs(HIGH, LOW, HIGH);
  }else if(val==0){
    courseHeading = 0;
    setLEDs(LOW, HIGH, LOW);
  }else{
     if (debug && debugPD){
        debugSerial.print("PD ERROR");
     }
  }
  
  return courseHeading;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Note: this function is exactly the same as the steadyCourseAlgorithm - only made two functions 
//        since variables being used are global (so don't want to overwrite) - can condence with cleaned up code

int resumePreviousCourse(double currentHeading, double headingRecall){
  
            error = currentHeading - headingRecall;
            gain = calculatePIDgain(error);
  
            if (abs(error) > maxDeviation && error < 0) {
              //bot needs to turn right - add power to left motor

                if (debug) {
                  Serial.print("Resuming Course - Turn Right");
                  Serial.print("\terror: "); Serial.print(error);Serial.print("\tGain: "); Serial.print(gain);
                }//end if debug
                
                setRightMotor(pwmForward);
                setLeftMotor(pwmForward+gain*pwmForward);

                return 1;
                
            }else if(abs(error) > max_deviation && error > 0){
                //bot needs to turn left - add power to right motor

                if (debug) {
                  Serial.print("Resuming Course - Turn Left");
                  Serial.print("\terror: "); Serial.print(error);Serial.print("\tGain: "); Serial.print(gain);                  
                }//end if debug
                
                setRightMotor(pwmForward+gain*pwmForward);
                setLeftMotor(pwmForward);

                return 1;
                
            } else {
                if (debug) {
                    Serial.print("On Course\t");
                }//end if debug
                
                setRightMotor(pwmForward);
                setLeftMotor(pwmForward);

                return 0;
                
            }//end if error
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int steadyCourseAlgorithm(double currentHeading, double steadyCourseHeading){
  
            error = currentHeading - steadyCourseHeading;
            gain = calculatePIDgain(error);
  
            if (abs(error) > maxDeviation && error < 0) {
              //bot needs to turn right - add power to left motor

                if (debug) {
                  Serial.print("Resuming Course - Turn Right");
                  Serial.print("\terror: "); Serial.print(error);Serial.print("\tGain: "); Serial.print(gain);
                }//end if debug
                
                setRightMotor(pwmForward);
                setLeftMotor(pwmForward+gain*pwmForward);

                return 1;
                
            }else if(abs(error) > max_deviation && error > 0){
                //bot needs to turn left - add power to right motor

                if (debug) {
                  Serial.print("Resuming Course - Turn Left");
                  Serial.print("\terror: "); Serial.print(error);Serial.print("\tGain: "); Serial.print(gain);                  
                }//end if debug
                
                setRightMotor(pwmForward+gain*pwmForward);
                setLeftMotor(pwmForward);

                return 1;
                
            } else {
                if (debug) {
                    Serial.print("On Course\t");
                }//end if debug
                
                setRightMotor(pwmForward);
                setLeftMotor(pwmForward);

                return 0;
                
            }//end if error
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calculatePIDgain(double error){

    currentTime = millis();

    if(currentTime - previousTime > gainTimeout){
      previousTime = currentTime;
    }

    cumError += (error * (currentTime - previousTime));
    
    pGain = kp * error;
    dGain = kd * (error - previousError) / (currentTime - previousTime);
    iGain = ki * cumError;

    previousError = error;
    previousTime = currentTime;
    
    gain = abs((pGain + dGain + iGain)/360);

   if(abs(gain) > maxGain){
        if(gain > maxGain){
          gain = maxGain;
        }else{
          gain = -maxGain;
        }
        if (debug  && debugMotor){
          debugSerial.println("Hit max gain (+17%% power)");
        }//end if debug
    }//end if gain

    return gain;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void pd_calibration_routine(void){
	//Flash LED to indicate ready to read beacon - wait for beacom to turn on
  courseHeading = getCourseHeading();
	while(courseHeading == 255){
		setLEDs(HIGH,LOW,LOW);
    delay(50);
		setLEDs(HIGH,HIGH,LOW);
    delay(50);
    setLEDs(HIGH,HIGH,HIGH);
    delay(50);
    setLEDs(HIGH,HIGH,HIGH);
    delay(50);
    setLEDs(HIGH,HIGH,LOW);
    delay(50);
    setLEDs(HIGH,LOW,LOW);

    courseHeading = getCourseHeading();

  	if (debug && debugPD){
  		debugSerial.print("Ready to calibrate: waiting for beacon\t PD Reading:"); debugSerial.println(courseHeading);
      delay(debug_delay);
  	}

   if (debug && debugIMU){
      getCurrentHeading();
   }
	}

  setLEDs(HIGH,HIGH,HIGH);

  //burn 2 values since the PD controller's 1st value is inaccurate
  delay(500);
  courseHeading = getCourseHeading();
  delay(500);
  courseHeading = getCourseHeading();
  delay(500);


	//beacon is now turned on.  Make LED HIGH and begin calibration routine
	setLEDs(HIGH, LOW, HIGH);
	
	if (debug && debugPD){
		debugSerial.println("Ready to calibrate: Calibration beginning");
		debugSerial.println();
    delay(debug_delay);
  }


	//check 0 degree
  courseHeading = getCourseHeading();
  pdCalibratedZero = courseHeading;

	currentHeading = getCurrentHeading();
    
    if (debug && debugPD){
    	debugSerial.print("Calibrating: "); debugSerial.print(pdCalibratedZero);
  		debugSerial.print(" degrees ---> maps to 0 ");
  		debugSerial.print("\t@ actual heading: "); debugSerial.print(currentHeading);
  		debugSerial.println();
  	}
 
    //Calibration is complete. flash a few times (for good measure), then turn off LED
    if (debug){
      debugSerial.println("Calibration Complete!");
    }
    
    for(int i = 0; i<5; i++){
      	setLEDs(LOW, HIGH, LOW);
    		delay(200);
    		setLEDs(LOW, LOW, LOW);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void displayCalibrationStatus(void) {
/*To see calibration Status
--calibration values (0-3)
--0 means not calibrated, though according to spec sheet, the IMU should work fine even uncalibrated
--3 means fully calibrated 
*/

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  debugSerial.print("\t");
  if (!system) {
    debugSerial.print("! ");
  }

  /* Display the individual values */
  debugSerial.print("Sys:"); debugSerial.print(system, DEC);
  debugSerial.print(" G:"); debugSerial.print(gyro, DEC);
  debugSerial.print(" A:"); debugSerial.print(accel, DEC);
  debugSerial.print(" M:"); debugSerial.print(mag, DEC);
}
