#include <LiquidCrystal.h>
byte spark[8] = {     //For Lcd Special Character 
  B00010,
  B00110,
  B01100,
  B11111,
  B00110,
  B01100,
  B01000,
  
};
byte celcius[8] = {      //For Lcd Special Character
  B11100,
  B10100,
  B11100,
  B00011,
  B00100,
  B00100,
  B00100,
  B00011
};
byte line[8] = {       //For Lcd Special Character
  B01110,
  B01110,
  B01110,
  B01110,
  B01110,
  B01110,
  B01110,
  B01110
};
byte bullet[8] = {      //For Lcd Special Character
  B01000,
  B01100,
  B01110,
  B01111,
  B01110,
  B01100,
  B01000,
  B00000
};
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);      //Lcd Pinout (RS,E,D4,D5,D6,D7)
const int buttonUp =  A3;                   // Up button pin
const int buttonDown =  A2;                 // Down button pin 
const int buttonSel =  A1;                  // Menu Select button pin  
const int fancontrol =  6;                  // Fan Controll PWM pin
const int numReadings = 20;                 // for taking average of 20 reading of real temperature 
const int redLed = 13;                      // Warning LED


int firing_pin = 9;                         //TRIAC Firing Pin (D9) Connected with MOC3083
int zero_cross = 8;                         //Zero cross detector pin (D8) connected to optocoupler
int last_CH1_state = 0;                     //For interupt

long firing_delay = 7400;                   //TRIAC signal delay after zero cross detected
long maximum_firing_delay = 7400;          

unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
long temp_read_Delay = 500;
long real_temperature = 0;
long settemp = 100;                         // Initial Set Temp Of Gun when power on Set anything you want but not much


                                            //PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
long PID_value = 0;
                                            //PID constants
long kp = 203;   long ki= 7.2;   long kd = 1.04;
long PID_p = 0;    long PID_i = 0;    long PID_d = 0;
  
long readings[numReadings];
long readIndex = 0;
long total = 0;
long average = 0;
int upbuttonstate = 0;
int downbuttonstate = 0;
int selbuttonstate = 0;
int fanspeed = 100;                          //Initial Fan speed
//long stemp = 0;
long sensorvalRAW = 0;                      //Sensor value without calibration
long sensorvalCALI = 0;                     //Sensor value calibrated
int menustate = 0;                          //Menu state for choose menu bullet position
int fanpwm = 255;                           //Inital fan PWM for 100% speed 

int redLedstatus = LOW;

void setup() {

  
  pinMode (firing_pin,OUTPUT); 
  pinMode (zero_cross,INPUT);


  
  PCICR |= (1 << PCIE0);                   //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);                 //Set pin D8 (zero cross input) trigger an interrupt on state change.
  
  
  lcd.begin(16, 2);
  lcd.createChar(0, spark);
  lcd.createChar(1, celcius);
  lcd.createChar(2, line);
  lcd.createChar(3, bullet);
  pinMode(buttonUp, INPUT);
  pinMode(buttonDown, INPUT);
  pinMode(buttonSel, INPUT);
  pinMode(6 , OUTPUT);
 // Serial.begin(115200);
  lcd.setCursor(1,0);
  lcd.print("Temp=");
  lcd.setCursor(1,1);
  lcd.print("Fan =");
  lcd.setCursor(11,0);
  lcd.print("RealC");
  lcd.setCursor(9,1);            //Dispay Special Characters on LCD
  lcd.write(byte(0));
  lcd.setCursor(9,0);
  lcd.write(byte(1));
  lcd.setCursor(15,1);
  lcd.write(byte(1));
  lcd.setCursor(10,0);
  lcd.write(byte(2));
  lcd.setCursor(10,1);
  lcd.write(byte(2));
  
  

  for (long thisReading =  0; thisReading < numReadings; thisReading++){
    readings[thisReading] = 0;
  }
}

void loop() {
  


sensorvalRAW = analogRead(A5);              //Reading RAW sensor data

sensorvalCALI = (1.28571* sensorvalRAW - 0.714286);      //Calibrating Sensor data



total = total - readings[readIndex];                   
readings[readIndex] = sensorvalCALI;
total = total + readings[readIndex];
readIndex = readIndex + 1;

if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

average = total / numReadings;               //Average of 20 calibrated reading just to show on LCD for smooth transition

currentMillis = millis();

if(currentMillis - previousMillis >= temp_read_Delay){
    previousMillis += temp_read_Delay;                       //Increase the previous time for next loop

PID_error = settemp - sensorvalCALI ; 

if(PID_error > 30)                                           //integral constant will only affect errors below 30ºC             
    {PID_i = 0;}
    
    PID_p = kp * PID_error; 
    //Calculate the P value
    PID_i = PID_i + (ki * PID_error); 
    //Calculate the I value
    timePrev = Time;                                         // the previous time is stored before the actual time read
    Time = millis();                                         // actual time read
    elapsedTime = (Time - timePrev) / 1000;
     
   
    PID_d = kd*((PID_error - previous_error)/elapsedTime); 
                                                             //Calculate the D value
    PID_value = PID_p + PID_i + PID_d; 
    
    if(PID_value < 0)
    {      PID_value = 0;       }
    if(PID_value > 7400)
    {      PID_value = 7400;    }
    previous_error = PID_error;                              //Remember to store the previous error.
  }



  
                
lcd.setCursor(12,1);
lcd.print("   ");
lcd.setCursor(12,1);
lcd.print(average);
delay(20);
 
  
upbuttonstate = digitalRead(buttonUp);
downbuttonstate = digitalRead(buttonDown);
selbuttonstate = digitalRead(buttonSel);



if(downbuttonstate == HIGH)
{ 

  if (menustate == 1)
  {
    if(settemp != 0)
    {
     settemp -= 1;  
    }
  }

  else if (menustate == 2)
  {
  if(fanspeed != 0)
  {
    fanspeed -= 1;
  } 
  
  
}
delay(50);            // Delay for debounce
}
if(upbuttonstate == HIGH)
{ 
  if (menustate == 1)
  {
    if(settemp != 500)
    {
    settemp += 1;  
    }
  }
  else if (menustate == 2)
  {
  if(fanspeed != 100)
  {
    fanspeed += 1;
  }  
    
    
  }
  
  delay(50);
}
if(selbuttonstate == HIGH)
{ 
  
  
  menustate += 1;
  delay(100);
  
} 
 



updatesettemp();
updatefanspeed();

if(menustate == 1)
{
lcd.setCursor(0,1);
lcd.print(" ");
lcd.setCursor(0,0);
lcd.write(byte(3));
  
}

else if (menustate == 2)
{
lcd.setCursor(0,0);
lcd.print(" ");
lcd.setCursor(0,1);
lcd.write(byte(3));  
}

else if(menustate > 2)
{
lcd.setCursor(0,0);
lcd.print(" ");
lcd.setCursor(0,1);
lcd.print(" ");  
menustate = 0;  
}


fanpwm = map(fanspeed , 0, 100, 0, 255);      //Map O to 100 Fan speed into PWM
analogWrite(fancontrol, fanpwm);
int ledstatus = settemp - average;            //Red Warning LED controll, Activate when real temp is greater than set temp
bool compare = ledstatus > 0;                 


if(compare)
{
  
  redLedstatus = LOW;
 
}
else
{
 
  redLedstatus = HIGH;
}

digitalWrite(redLed, redLedstatus);        //On or Off the Red led


}




void updatesettemp()
{
lcd.setCursor(6,0);
lcd.print("   ");
lcd.setCursor(6,0);
lcd.print(settemp);

return 0;    
}

void updatefanspeed()
{

  lcd.setCursor(6,1);
  lcd.print("   ");
  lcd.setCursor(6,1);
  lcd.print(fanspeed);
  return 0;
  
}

                   //Interuupt Function for zero cross pin
ISR(PCINT0_vect){
  ///////////////////////////////////////Input from optocoupler
  if(PINB & B00000001){                                         //We make an AND with the state register, We verify if pin D8 is HIGH???
    if(last_CH1_state == 0 && redLedstatus == LOW){             //If the last state was 0, then we have a state change...and with red led status 
      
      
      delayMicroseconds(maximum_firing_delay - PID_value);      //This delay controls the power
                                                               
      digitalWrite(firing_pin,HIGH);
      delayMicroseconds(100);
      digitalWrite(firing_pin,LOW);
     
                                                                //We have detected a state change! We need both falling and rising edges
    }
  }
  else if(last_CH1_state == 1 && redLedstatus == LOW){    //If pin 8 is LOW and the last state was HIGH then we have a state change      
                                                          //We haev detected a state change!  We need both falling and rising edges.
    last_CH1_state = 0; 
    
      delayMicroseconds(maximum_firing_delay - PID_value);
     
      digitalWrite(firing_pin,HIGH);
      delayMicroseconds(100);
      digitalWrite(firing_pin,LOW);
      
                                           
    }
}
