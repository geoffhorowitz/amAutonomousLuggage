/*
         Isolated_pseudoDoppler_analysis
         
         Program Objective:
            Program is meant to read the Psuedo-Doppler Data, analyze the phase shift (relative to the reference signal) and output to the master control

         general procedure to compute direction vector
              acos(dot(a,b)/(norm(a)norm(b)
              dot(a,b)=a1b1+a2b2+anbn
              norm(a)= (a[1]^2+a[2]^2+a[n}^2 )^.5
 */

// inclusions and definitions
#define AP_RS 10     // Reference Signal Analog Pin
#define AP_PDO 1     //PD Output Signal Analog Pin

#define sampleTime 2000    //how long to sample PD data
#define numAvgSamples 0  //how many samples to average before sending to controller

#define serial_write_delay 100
#define send_to_master_delay 400

#define debugSerial SerialUSB
#define debugBaud 115200
#define commSerial Serial1
#define commBaud 115200

#define beaconOffValue 255

#define debug false   //debug parameter

int VAL_RS[sampleTime];   //Arrays to Store PD and Ref Data     
int VAL_PDO[sampleTime];

double dot = 0;
double normR = 0;
double normP = 0;
double averagePhase = 0;
byte toSend;
int count;
double phase = 0;

bool beaconOn = false;

void setup()
{
  pinMode(AP_RS, INPUT);
  pinMode(AP_PDO, INPUT);

  commSerial.begin(debugBaud);              //  setup serialUSB over due NATIVE port for speed
  debugSerial.begin(commBaud);
  while(!commSerial){debugSerial.print("error connecting to commSerial");};
  if (debug){
      debugSerial.println("Ready to begin");
  }
}

void loop(){    
    dot = 0;
    normR = 0;
    normP = 0;
 
      
    for (int i = 0; i < sampleTime; i++) {
      VAL_RS[i] = analogRead(AP_RS);     // read the analogpins and store in array
      VAL_PDO[i] = analogRead(AP_PDO);
    }

    //while below loop could be included in above, breaking up to sample data as fast as possible.
    for (int i = 0; i < sampleTime; i++) {
         dot=dot+(VAL_RS[i]*VAL_PDO[i]);
         normR=normR+pow(VAL_RS[i],2);
         normP=normP+pow(VAL_PDO[i],2);
    }
    
    normR=pow(normR,.5);
    normP=pow(normP,.5);
    
    phase=60*acosfull(dot,(normR*normP));
    phase = (phase - 75)*2;
          
      if (!isnan(phase) && !(phase>=90)){
          averagePhase = averagePhase + phase;
          if (!beaconOn){
            averagePhase = 0;
            count = 0;
            beaconOn = true;
          }
      }else{
          beaconOn = false;
      }
    count++;

  //
   if(count>numAvgSamples){
      count = 0;
      
      if (beaconOn){
        if((averagePhase/numAvgSamples>10)){
          toSend = 1;
        }else if((averagePhase/numAvgSamples<-10)){
          toSend = 2;
        }else{
          toSend = 0;
        }
      }else{
          toSend =  beaconOffValue;
      }

      commSerial.write(toSend);
      delay(serial_write_delay);
      delay(send_to_master_delay);

      if (debug){
          debugSerial.print("\tAveragePhase: "); debugSerial.print(averagePhase/numAvgSamples);
          debugSerial.print("\tPhase: "); debugSerial.println(toSend);
      }//end if debug

      averagePhase = 0;
  }//end if count > numAvgSamples
}//end loop
  
    
double acosfull(double x, double y){
      //acossfull: find output angle of arccos over 0<theta<2*pi
      //find value of acos(z) in the correct quadrant over the full range [0 2pi]; 
      //acosfull requires as input x and y coordinates (length of x and y side of a rectangular triangle).
        
        double theta = acos(x/sqrt(x*x + y*y));
        if (y<0){
          theta += 3.1415;
        }
        
        return theta;
}
