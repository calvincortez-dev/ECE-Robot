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

uint16_t sensorValues[8];

//left wheel pins declaration
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

//right wheel pins declaration
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int LED_RF = 41;

//Calibration Values for IR Sensor
long sensorMin[8] = {872, 710, 756, 687, 730, 688, 780, 849};
long sensorMax[8] = {1628, 1790, 1744, 1514, 1447, 1812, 1720, 1651};
long weights[8] = {-15, -14, -12, -8, 8, 12, 14, 15}; //Chose -15 -14 -12 -8 weighted sum scheme

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  
  //Change to LOW on tabletop
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);

  //Change to LOW on tabletop
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
//  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() 
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  

  // put your main code here, to run repeatedly: 
  int baseSpd = 70;

  analogWrite(left_pwm_pin,baseSpd);
  analogWrite(right_pwm_pin, baseSpd);

// 
  
//  ECE3_read_IR(sensorValues);

  digitalWrite(LED_RF, HIGH);
  delay(250);
  digitalWrite(LED_RF, LOW);
  delay(250);
    
  }
