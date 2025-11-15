// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 

//#include <ECE3_LCD7.h>

//uint16_t sensorValues[8]; // right -> left, 0 -> 7

#include <ECE3.h>

uint16_t sensorValues[8];

//left wheel pins 
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

//right wheel pins 
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int LED_RF = 41;

//Calibration Values for IR Sensor
long sensorMin[8] = {872, 710, 756, 664, 730, 688, 780, 849};
long sensorMax[8] = {1628, 1790, 1744, 1514, 1447, 1812, 1720, 1651};

// Weights for weighted-average error (right negative, left positive)
long weights[8] = {15, 14, 12, 8, -8, -12, -14, -15}; //Chose -15 -14 -12 -8 weighted sum scheme
long normalized[8];

// Initialize variables
float diffSum = 0;
int error = 0;
int prevError = 0;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;

///////////////////////////////////
void setup() 
{
  
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin, HIGH); // LOW for debugging, HIGH for testing

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin, HIGH); // LOW for debugging, HIGH for testing

  pinMode(LED_RF, OUTPUT);

  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600);
  // Serial.print(19200);
  delay(2000); //Wait 2 seconds before starting
  
}

void loop() 
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  //Subtract sensor to its respective min
  //Normalize each sensor to 1000 after subtracting the minimum (1000 * calculated / sensorMax)
  //Compute Weighted Avg (15-14-12-8)

  for (int j = 0; j < 8; j++)
  {
    normalized[j] = ((1000) * (sensorValues[j] - sensorMin[j]))/sensorMax[j];
    error += (normalized[j] * weights[j]);

  }
  
  

  error = error/8;

  // put your main code here, to run repeatedly: 
  float leftSpd = 15;
  float rightSpd = 15;

  /*DIFSUM FOR DERIVATIVE CONTROL*/
  diffSum = (error - prevError);

  // Initialize kP
  float kP = (7.5)/2181.0;

  // Initialize kD
  float kD = (15.0)/4362.0;

  prevError = error; // prevError for next loop

  // CALCULATE PIDSUM
  float PIDSum = (kP * error) + (kD * diffSum);
  leftSpd += PIDSum;
  rightSpd -= PIDSum;

  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

  // Donut section
  bool all2500 = true;  

  for (int i = 0; i < 8; i++) 
  {
    if (sensorValues[i] != 2500) 
    {
      all2500 = false;
      break;  // no need to keep checking
    }
  }

  if (all2500) 
  {
    leftSpd  = 30;
    rightSpd = 0;
    analogWrite(left_pwm_pin, leftSpd);
    analogWrite(right_pwm_pin, rightSpd);
    delay(1000);
    return;
  }

  // end of main loop
  // UNComment if DEBUGGING OR comment if not DEBUGGING

    Serial.print("PIDSUM: ");
    Serial.println(PIDSum);

    Serial.print("kP: ");
    Serial.println(kP);

    Serial.print("kD: ");
    Serial.println(kD);

    Serial.print("leftSpd: ");
    Serial.println(leftSpd);

    Serial.print("rightSpd: ");
    Serial.println(rightSpd);

    Serial.print("error: ");
    Serial.println(error);
   
    Serial.print("Sensor 1 Reading: ");
    Serial.println(sensorValues[0]);

    Serial.print("Sensor 2 Reading: ");
    Serial.println(sensorValues[1]);

    Serial.print("Sensor 3 Reading: ");
    Serial.println(sensorValues[2]);

    Serial.print("Sensor 4 Reading: ");
    Serial.println(sensorValues[3]);

    Serial.print("Sensor 5 Reading: ");
    Serial.println(sensorValues[4]);

    Serial.print("Sensor 6 Reading: ");
    Serial.println(sensorValues[5]);

    Serial.print("Sensor 7 Reading: ");
    Serial.println(sensorValues[6]);

    Serial.print("Sensor 8 Reading: ");
    Serial.println(sensorValues[7]);
  
}
