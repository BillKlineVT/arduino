//LAST EDITED 22.10.16
//=====================
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
// ************************************************
// Pin definitions
// ************************************************

#define HLT_RelayPin 7
#define ONE_WIRE_BUS 12
#define BUTTON_FAR_L 8
#define BUTTON_MID_L 3
#define BUTTON_MID_R 4 
#define BUTTON_FAR_R 9

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double HLT_Setpoint;
double HLT_Temp;
double Mash_Temp;
double BK_Temp;
double Chill_Temp;
double HLT_Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp=100;
double Ki=0.2;
double Kd=5;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&HLT_Temp, &HLT_Output, &HLT_Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
unsigned int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&HLT_Temp, &HLT_Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

unsigned long lastInput = 0; // last button press

//byte degree[8] = // define the degree symbol 
//{ B00110, B01001, B01001, B00110, B00000,B00000, B00000, B00000 }; 

const int logInterval = 10000; // log every 10 seconds
unsigned long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, MAIN, SHOWPID, SETPID, TEMPS, SETPOINT, MONITOR, TUNE_P, TUNE_I, TUNE_D, AUTO_PID, TEMPS_HLT, TEMPS_MASH, TEMPS_BK, TEMPS_CHILL};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
//DeviceAddress tempSensor_HLT[8], tempSensor_Mash[8], tempSensor_BK[8], tempSensor_Chill[8];
DeviceAddress tempSensor_HLT = { 0x28, 0xC3, 0x65, 0xCD, 0x04, 0x00, 0x00, 0x21 };
DeviceAddress tempSensor_Mash = { 0x28, 0x23, 0x6F, 0xCD, 0x04, 0x00, 0x00, 0xB5 };
DeviceAddress tempSensor_BK = { 0x28, 0x07, 0x48, 0xCE, 0x04, 0x00, 0x00, 0xFE };
DeviceAddress tempSensor_Chill = { 0x28, 0x39, 0xA4, 0xCE, 0x04, 0x00, 0x00, 0xFB };

int button_far_left, button_mid_left, button_mid_right, button_far_right;
// ************************************************
// Setup and display initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

   // Initialize Relay Control:

  pinMode(HLT_RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(HLT_RelayPin, HIGH);  // make sure it is off to start
  pinMode(BUTTON_FAR_L, INPUT_PULLUP); 
  pinMode(BUTTON_MID_L, INPUT_PULLUP); 
  pinMode(BUTTON_MID_R, INPUT_PULLUP); 
  pinMode(BUTTON_FAR_R, INPUT_PULLUP);
  button_far_left=0; 
  button_mid_left=0; 
  button_mid_right=0; 
  button_far_right=0;

  // Initialize LCD Display 

  lcd.begin(16, 2);lcd.clear();lcd.backlight();lcd.setCursor(0, 0); //(Col, Row);
  lcd.print(F("Starting..."));

   // Start up the DS18B20 One Wire Temperature Sensor
   
   sensors.begin();

   
   if (!sensors.getAddress(tempSensor_HLT, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Err: HLT "));
      delay(1000);
   }
   if (!sensors.getAddress(tempSensor_Mash, 1)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Err: Mash"));
      delay(1000);
   }
   
   if (!sensors.getAddress(tempSensor_BK, 1)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Err: BK  "));
      delay(1000);
   }

   if (!sensors.getAddress(tempSensor_Chill, 1)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Err:Chill"));
      delay(1000);
   }
   sensors.setResolution(tempSensor_HLT, 12);
   sensors.setResolution(tempSensor_Mash, 12);
   sensors.setResolution(tempSensor_BK, 12);
   sensors.setResolution(tempSensor_Chill, 12);
   sensors.setWaitForConversion(false);
   Serial.println(tempSensor_HLT[7], HEX);
   Serial.println(tempSensor_Mash[7], HEX);
   Serial.println(tempSensor_BK[7], HEX);
   Serial.println(tempSensor_Chill[7], HEX);
   
   delay(3000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;

  TCCR1B = TCCR1B & B11111000 | B00000010;

}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(HLT_RelayPin, HIGH);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

//BUTTON INTERRUPT
void read_but()
{
  bool pin_status1;
  bool pin_status2;
  bool pin_status3;
  bool pin_status4;
  delay(3);

  pin_status1=digitalRead(BUTTON_FAR_L);  
  if((pin_status1 == LOW) && (button_far_left==128)) {button_far_left = 0;Serial.println("button far L 0");}
  if((pin_status1 == HIGH) && (button_far_left!=128)) {button_far_left = 128;Serial.println("button far L 1");lastInput=millis();}

  pin_status2=digitalRead(BUTTON_MID_L);
  if((pin_status2 == LOW) && (button_mid_left==128)) {button_mid_left = 0;Serial.println("button mid L 0");}
  if((pin_status2 == HIGH) && (button_mid_left!=128)) {button_mid_left=128;}//Serial.println("button mid L 1");lastInput=millis();}
  
  pin_status3=digitalRead(BUTTON_MID_R);
  if((pin_status3 == LOW) && (button_mid_right==128)) {button_mid_right = 0;Serial.println("button mid R 0");}
  if((pin_status3 == HIGH) && (button_mid_right!=128)) {button_mid_right=128;}//Serial.println("button mid R 1");lastInput=millis();}

  pin_status4=digitalRead(BUTTON_FAR_R);
  if((pin_status4 == LOW) && (button_far_right==128)) {button_far_right = 0;Serial.println("button far R 0");}
  if((pin_status4 == HIGH) && (button_far_right!=128)) {button_far_right=128;}//Serial.println("button far R 1");lastInput=millis();}
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   // wait for button release before changing state
   //while(enc_but != 1 ) { read_but();}
   //while(ReadButtons() != 0) {}

   lcd.clear();
//   enc_but = 0;
//OFF = 0, MAIN, SHOWPID, TEMPS, SETPOINT, MONITOR, SETPID, TUNE_P, TUNE_I, TUNE_D, AUTO_PID, TEMPS_HLT, TEMPS_MASH, TEMPS_BK, TEMPS_CHILL
   switch (opState)
   {
   case OFF:
      Off();       
      break;
    case MAIN:
      Main_Menu();         
      break;
   case SHOWPID:
      Show_PID();         
      break;
    case TEMPS:
      Show_Temps();        
      break;
   case SETPOINT:
      SetPoint();        
      break;
   case MONITOR:
      Monitor();        
      break;
   case SETPID:
      Set_PID();         
      break;
   case TUNE_P:
      TuneP();        
      break;
   case TUNE_I:
      TuneI();         
      break;
   case TUNE_D:
      TuneD();        
      break; 
   case AUTO_PID:
      AutoPID();        
      break; 
   case TEMPS_HLT:
      TempsHLT();        
      break;
   case TEMPS_MASH:
      TempsMash();         
      break;
   case TEMPS_BK:
      TempsBK();        
      break; 
   case TEMPS_CHILL:
      TempsChill();        
      break; 
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   digitalWrite(HLT_RelayPin, HIGH);  // make sure it is off
   lcd.print(F("POWER OFF"));
   lcd.setCursor(0, 1);
   lcd.print(F("START"));
   
   while(button_far_left != 0){read_but();}
   button_far_left = 1;button_mid_left = 1;button_mid_right = 1;button_far_right = 1;

   lcd.clear();
   lcd.print(F("Check Water Lvl"));
   lcd.setCursor(0,1);
   lcd.print(F("CONFIRM"));
   while(button_far_left != 0){read_but();}
   button_far_left = 1;
   lcd.clear();
   lcd.print(F("Heat Enabled..."));
   delay(1000);
   
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = MAIN; // start control
}

// ************************************************
// Main Menu State
// Far Left Button = PID menu
// Mid Left Button = TEMPS menu
// Mid Right Button = SET menu
// Far Right Button = MON menu
// ************************************************
void Main_Menu()
{
  lcd.clear();
  lcd.print(F("BILL BREW v1.0"));
  lcd.setCursor(0,1);
  lcd.print(F("PID TEMP SET MON"));

  while(true)
  {
      read_but();

      if (button_far_left == 0)
      {
        opState = SHOWPID;
        button_far_left = 1;
        Serial.println("Going to PID view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TEMPS;
        button_mid_left = 1;
        Serial.println("Going to Temps view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = SETPOINT;
        button_mid_right = 1;
        Serial.println("Going to SetPoint view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = MONITOR;
        button_far_right = 1;
        Serial.println("Going to Monitor view");
        delay(100);
        return;
      }
  }
}

// ************************************************
// Show PID State
// Far Left Button = BACK
// Mid Left Button = SET menu
// Mid Right Button = AUTO menu
// Far Right Button = (none)
// ************************************************
void Show_PID()
{
  lcd.clear();
  lcd.print(F("P:"));
  lcd.print(Kp,0);
  lcd.print(F("I:"));
  lcd.print(Ki,1);
  lcd.print(F("D:"));
  lcd.print(Kd,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK SET AUTO"));

  while(true)
  {
      read_but();

      if (button_far_left == 0)
      {
        opState = MAIN;
        button_far_left = 1;
        Serial.println("Going to Main view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = SETPID;
        button_mid_left = 1;
        Serial.println("Going to Set PID view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = AUTO_PID;
        button_mid_right = 1;
        Serial.println("Going to Auto PID view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        button_far_right = 1;
        Serial.println("no menu option for far right");
        delay(100);
        return;
      }
  }
}

// ************************************************
// Set PID State
// Far Left Button = BACK (to Show PID)
// Mid Left Button = P menu
// Mid Right Button = I menu
// Far Right Button = D menu
// ************************************************
void Set_PID()
{
  lcd.clear();
  lcd.print(F("P:"));
  lcd.print(Kp,0);
  lcd.print(F("I:"));
  lcd.print(Ki,1);
  lcd.print(F("D:"));
  lcd.print(Kd,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK P   I    D"));

  while(true)
  {
      read_but();

      if (button_far_left == 0)
      {
        opState = SHOWPID;
        button_far_left = 1;
        Serial.println("Going to Show PID view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TUNE_P;
        button_mid_left = 1;
        Serial.println("Going to Tune P view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = TUNE_I;
        button_mid_right = 1;
        Serial.println("Going to TUNE I view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = TUNE_D;
        button_far_right = 1;
        Serial.println("Going to Tune D view");
        delay(100);
        return;
      }
  }
}

// ************************************************
// Show Temps State
// Far Left Button = BACK
// Mid Left Button = MASH menu
// Mid Right Button = BK menu
// Far Right Button = CHILL menu
// ************************************************
void Show_Temps()
{
  lcd.clear();
  lcd.print(F("TEMPS HLT:"));
  lcd.print(HLT_Temp,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK MSH BK CHL"));

  while(true)
  {
      lcd.setCursor(0,0);
      lcd.print(F("                "));
      lcd.setCursor(0,0);
      lcd.print(F("TEMPS HLT:"));
      lcd.print(HLT_Temp,1);
      read_but();

      if (button_far_left == 0)
      {
        opState = MAIN;
        button_far_left = 1;
        Serial.println("Going to Main view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TEMPS_MASH;
        button_mid_left = 1;
        Serial.println("Going to TEMPS MASH view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = TEMPS_BK;
        button_mid_right = 1;
        Serial.println("Going to TEMPS BK view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = TEMPS_CHILL;
        button_far_right = 1;
        Serial.println("Going to TEMPS CHILL view");
        delay(100);
        return;
      }
      delay(100);
      DoControl();
  }
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void SetPoint()
{
   double increment = 1;
   lcd.clear();
   lcd.print(F("HLT SETP:"));
   lcd.print(HLT_Setpoint,1);
   lcd.setCursor(0,1);
   lcd.print(F("BACK UP DN BY1"));

   while(true)
   {
      read_but();

      if (button_far_left == 0)
      {
        opState = MAIN;
        button_far_left = 1;
        Serial.println("Going to Main view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        HLT_Setpoint = HLT_Setpoint + increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("HLT SETP:"));
        lcd.print(HLT_Setpoint,1);
        button_mid_left = 1;
        Serial.println("Increasing Set Point");
        delay(100);
        //return;
      }
      if (button_mid_right == 0)
      {
        HLT_Setpoint = HLT_Setpoint - increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("HLT SETP:"));
        lcd.print(HLT_Setpoint,1);
        button_mid_left = 1;
        Serial.println("Decreasing Set Point");
        delay(100);
        //return;
      }
      if (button_far_right == 0)
      {
        if (increment == 0.1) { increment = 1; }
        else if (increment == 1) { increment = 10; }
        else if (increment == 10) { increment = 0.1; }
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        lcd.setCursor(0,1);
        lcd.print(F("BACK UP DN BY"));
        if (increment == 0.1) { lcd.print(".1"); }
        else { lcd.print(increment,0); }
        button_far_right = 1;
        Serial.println("Toggling increment");
        delay(100);
        //return;
      }
      SaveParameters();   
      delay(100);
      DoControl();
   }
}

// ************************************************
// Monitor State
// Far Left Button = BACK
// Mid Left Button = (none)
// Mid Right Button = Increase Setpoint by 1
// Far Right Button = Decrease Setpoint by 1
// ************************************************
void Monitor()
{
  while(true)
  {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("HLT:"));
      lcd.print(HLT_Temp,1);
      lcd.print(F("S:"));
      lcd.print(HLT_Setpoint,1);
      lcd.setCursor(0,1);
      lcd.print(F("B PWM:"));
      float pct = map(HLT_Output, 0, WindowSize, 0, 1000);
      lcd.print((pct/10),0);
      lcd.print(F("M:"));
      lcd.print(Mash_Temp,1);
      
      read_but();

      if (button_far_left == 0)
      {
        opState = MAIN;
        button_far_left = 1;
        Serial.println("Going to Main view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        HLT_Setpoint = HLT_Setpoint + 1;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("HLT:"));
        lcd.print(HLT_Temp,1);
        lcd.print(F("S:"));
        lcd.print(HLT_Setpoint,1);
        button_mid_left = 1;
        Serial.println("Increasing Set Point");
        delay(100);
        //return;
      }
      if (button_far_right == 0)
      {
        HLT_Setpoint = HLT_Setpoint - 1;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("HLT:"));
        lcd.print(HLT_Temp,1);
        lcd.print(F("S:"));
        lcd.print(HLT_Setpoint,1);
        button_mid_left = 1;
        Serial.println("Decreasing Set Point");
        delay(100);
        //return;
      }
      DoControl();

      delay(100);
  }
}

// ************************************************
// Proportional Tuning State
// Far Left Button = BACK (to Show PID)
// Mid Left Button = UP (increase Kp)
// Mid Right Button = DN (decrease Kp)
// Far Right Button = I (go to Set I)
// ************************************************
void TuneP()
{
  lcd.clear();
  lcd.print(F("PID P: "));
  lcd.print(Kp,0);
  lcd.setCursor(0,1);
  lcd.print(F("BACK UP DN BY1"));
  double increment = 1;
  
  while(true)
  {
      read_but();
      if (button_far_left == 0)
      {
        opState = SHOWPID;
        button_far_left = 1;
        Serial.println("Going to Show PID view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        Kp = Kp + increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID P: "));
        lcd.print(Kp,0);
        lcd.print(F("     "));
        button_mid_left = 1;
        Serial.println("Increasing Kp");
        delay(100);
        //return;
      }
      if (button_mid_right == 0)
      {
        Kp = Kp - increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID P: "));
        lcd.print(Kp,0);
        lcd.print(F("     "));
        button_mid_left = 1;
        Serial.println("Decreasing Kp");
        delay(100);
        //return;
      }
      if (button_far_right == 0)
      {
        if (increment == 0.1) { increment = 1; }
        else if (increment == 1) { increment = 10; }
        else if (increment == 10) { increment = 0.1; }
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        lcd.setCursor(0,1);
        lcd.print(F("BACK UP DN BY"));
        if (increment == 0.1) { lcd.print(".1"); }
        else { lcd.print(increment,0); }
        button_far_right = 1;
        Serial.println("Toggling increment");
        delay(100);
      }
      SaveParameters();
      myPID.SetTunings(Kp,Ki,Kd);
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// Far Left Button = BACK (to Tune P)
// Mid Left Button = UP (increase Ki)
// Mid Right Button = DN (decrease Ki)
// Far Right Button = I (go to Set D)
// ************************************************
void TuneI()
{
  lcd.clear();
  lcd.print(F("PID I: "));
  lcd.print(Ki,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK UP DN BY.1"));
  double increment = 0.1;
  
  while(true)
  {
      read_but();
      if (button_far_left == 0)
      {
        opState = TUNE_P;
        button_far_left = 1;
        Serial.println("Going to Tune P view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        Ki = Ki + increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID I: "));
        lcd.print(Ki,1);
        button_mid_left = 1;
        Serial.println("Increasing Ki");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        Ki = Ki - increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID I: "));
        lcd.print(Ki,1);
        button_mid_left = 1;
        Serial.println("Decreasing Ki");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        //opState = TUNE_D;
        if (increment == 0.1) { increment = 1; }
        else if (increment == 1) { increment = 10; }
        else if (increment == 10) { increment = 0.1; }
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        lcd.setCursor(0,1);
        lcd.print(F("BACK UP DN BY"));
        if (increment == 0.1) { lcd.print(".1"); }
        else { lcd.print(increment,0); }
        button_far_right = 1;
        Serial.println("Toggling increment");
        delay(100);
      }
      SaveParameters();
      myPID.SetTunings(Kp,Ki,Kd);
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// Far Left Button = BACK (to Tune I)
// Mid Left Button = UP (increase Kd)
// Mid Right Button = DN (decrease Kd)
// Far Right Button = (none)
// ************************************************
void TuneD()
{
  lcd.clear();
  lcd.print(F("PID D: "));
  lcd.print(Kd,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK UP DN BY.1"));
  double increment = 0.1;
  
  while(true)
  {
      read_but();
      if (button_far_left == 0)
      {
        opState = TUNE_I;
        button_far_left = 1;
        Serial.println("Going to Tune I view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        Kd = Kd + increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID D: "));
        lcd.print(Kd,1);
        button_mid_left = 1;
        Serial.println("Increasing Kd");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        Kd = Kd - increment;
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,0);
        lcd.print(F("PID I: "));
        lcd.print(Kd,1);
        button_mid_left = 1;
        Serial.println("Decreasing Kd");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        if (increment == 0.1) { increment = 1; }
        else if (increment == 1) { increment = 10; }
        else if (increment == 10) { increment = 0.1; }
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        lcd.setCursor(0,1);
        lcd.print(F("BACK UP DN BY"));
        if (increment == 0.1) { lcd.print(".1"); }
        else { lcd.print(increment,0); }
        button_far_right = 1;
        Serial.println("Toggling increment");
        delay(100);
      }
      SaveParameters();
      myPID.SetTunings(Kp,Ki,Kd);
      
      DoControl();
   }
}

// ************************************************
// Auto PID Tuning State
// Far Left Button = BACK (to Show PID)
// Mid Left Button = START
// Mid Right Button = (none)
// Far Right Button = (none)
// ************************************************
void AutoPID()
{
  lcd.clear();
  lcd.print(F("AUTO PID: OFF  "));
  lcd.setCursor(0,1);
  lcd.print(F("BACK START      "));
  
  while(true)
  {
      read_but();
      if (button_far_left == 0)
      {
        opState = SHOWPID;
        button_far_left = 1;
        Serial.println("Going to Show PID view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        if (abs(HLT_Temp - HLT_Setpoint) < 0.5)// Should be at steady-state
        {
          StartAutoTune();
        }
        else
        {
          lcd.setCursor(0,0);
          lcd.print(F("AUTO PID: WAIT  "));
        }
        delay(100);
        //return;
      }
  }
}

// ************************************************
// Temps HLT state
// Far Left Button = BACK (to Main)
// Mid Left Button = MASH (to mash temp)
// Mid Right Button = BK (to BK temp)
// Far Right Button = CHILL (to chill temp)
// ************************************************
void TempsHLT()
{
  Show_Temps(); // Temps HLT view is same as the main Show Temps view
}

// ************************************************
// Temps Mash state
// Far Left Button = BACK
// Mid Left Button = HLT menu
// Mid Right Button = BK menu
// Far Right Button = CHILL menu
// ************************************************
void TempsMash()
{
  lcd.clear();
  lcd.print(F("TEMPS MASH:"));
  lcd.print(Mash_Temp,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK HLT BK CHL"));

  while(true)
  {
      lcd.setCursor(0,0);
      lcd.print(F("                "));
      lcd.setCursor(0,0);
      lcd.print(F("TEMPS MASH:"));
      lcd.print(Mash_Temp,1);
      read_but();

      if (button_far_left == 0)
      {
        opState = TEMPS;
        button_far_left = 1;
        Serial.println("Going to Show Temps view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TEMPS_HLT;
        button_mid_left = 1;
        Serial.println("Going to TEMPS HLT view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = TEMPS_BK;
        button_mid_right = 1;
        Serial.println("Going to TEMPS BK view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = TEMPS_CHILL;
        button_far_right = 1;
        Serial.println("Going to TEMPS CHILL view");
        delay(100);
        return;
      }
      delay(100);
      DoControl();
  }
}

// ************************************************
// Temps BK state
// Far Left Button = BACK
// Mid Left Button = HLT menu
// Mid Right Button = BK menu
// Far Right Button = CHILL menu
// ************************************************
void TempsBK()
{
  lcd.clear();
  lcd.print(F("TEMPS BK:"));
  lcd.print(BK_Temp,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK HLT MSH CHL"));

  while(true)
  {
      lcd.setCursor(0,0);
      lcd.print(F("                "));
      lcd.setCursor(0,0);
      lcd.print(F("TEMPS BK:"));
      lcd.print(BK_Temp,1);
      read_but();

      if (button_far_left == 0)
      {
        opState = TEMPS;
        button_far_left = 1;
        Serial.println("Going to Show Temps view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TEMPS_HLT;
        button_mid_left = 1;
        Serial.println("Going to TEMPS HLT view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = TEMPS_MASH;
        button_mid_right = 1;
        Serial.println("Going to TEMPS MASH view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = TEMPS_CHILL;
        button_far_right = 1;
        Serial.println("Going to TEMPS CHILL view");
        delay(100);
        return;
      }
      DoControl();
      delay(100);
  }
}

// ************************************************
// Temps Chill state
// Far Left Button = BACK
// Mid Left Button = HLT menu
// Mid Right Button = MASH menu
// Far Right Button = BK menu
// ************************************************
void TempsChill()
{
  lcd.clear();
  lcd.print(F("TEMPS CHL:"));
  lcd.print(Chill_Temp,1);
  lcd.setCursor(0,1);
  lcd.print(F("BACK HLT MASH BK"));

  while(true)
  {
      lcd.setCursor(0,0);
      lcd.print(F("                "));
      lcd.setCursor(0,0);
      lcd.print(F("TEMPS CHL:"));
      lcd.print(Chill_Temp,1);
      read_but();

      if (button_far_left == 0)
      {
        opState = TEMPS;
        button_far_left = 1;
        Serial.println("Going to Show Temps view");
        delay(100);
        return;
      }
      if (button_mid_left == 0)
      {
        opState = TEMPS_HLT;
        button_mid_left = 1;
        Serial.println("Going to TEMPS HLT view");
        delay(100);
        return;
      }
      if (button_mid_right == 0)
      {
        opState = TEMPS_MASH;
        button_mid_right = 1;
        Serial.println("Going to TEMPS MASH view");
        delay(100);
        return;
      }
      if (button_far_right == 0)
      {
        opState = TEMPS_BK;
        button_far_right = 1;
        Serial.println("Going to TEMPS BK view");
        delay(100);
        return;
      }
      DoControl();
      delay(100);
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    //double HLT_Temp;double Mash_Temp;double BK_Temp;double Chill_Temp;
    //tempSensor_HLT, tempSensor_Mash, tempSensor_BK, tempSensor_Chill
    HLT_Temp = sensors.getTempF(tempSensor_HLT);
    Mash_Temp= sensors.getTempF(tempSensor_Mash);
    BK_Temp = sensors.getTempF(tempSensor_BK);
    Chill_Temp= sensors.getTempF(tempSensor_Chill);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = HLT_Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(HLT_RelayPin,LOW);
  }
  else
  {
     digitalWrite(HLT_RelayPin,HIGH);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
//void setBacklight()
//{
//   if (tuning)
//   {
//      lcd.print("T "); // Tuning Mode
//   }
//   else if (abs(Input - Setpoint) > 1.0)  
//   {
//      lcd.print("H ");// High Alarm - off by more than 1 degree
//   }
//   else if (abs(Input - Setpoint) > 0.2)  
//   {
//      lcd.print("L ");// Low Alarm - off by more than 0.2 degrees
//   }
//   else
//   {
//      lcd.print("* "); // Lock on temp! 
//   }
//}


// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (HLT_Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, HLT_Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   HLT_Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(HLT_Setpoint))
   {
     HLT_Setpoint = 100;
   }
   if (isnan(Kp))
   {
     Kp = 100;
   }
   if (isnan(Ki))
   {
     Ki = 0.2;
   }
   if (isnan(Kd))
   {
     Kd = 5.0;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
