//Libraries

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "images.h"


//Final PCB pins

//buttons
#define POWER_BUTTON 2 //Press C/F on PCB
#define LOCK_BUTTON 3 //Press Light on PCB
#define UNITS_BUTTON 4 //Press On/Off on PCB
#define CAL_BUTTON 7 //Correct on PCB
#define LIGHT_BUTTON 12 //Press Lock on PCB
//Outputs
#define LED_LIGHT 10
//OLED Screen Macros
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//Temp Sensor Macros
#define POT_TEMP_INPUT A0
#define DIGITAL_TEMP_INPUT 13

//Timer Rollover Values
//Timer 1
#define TIMER_COMPARE_3S 46875
#define TIMER_COMPARE_1S 15625
//Timer 2
#define TIMER_COMPARE_250HZ 250

//Screen Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Temperature Sensor Objects
OneWire oneWire(DIGITAL_TEMP_INPUT);
DallasTemperature sensors(&oneWire);
DeviceAddress tempProbe;

//Gyro Object
MPU6050 accelgyro;

//Temp Store Variables
int gPotReading = 0;
float tempC = 0;

//Gyro Variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
char tilt = 1; // 0 is "upside down" and 1 is "right side up" (with regard to the text on the PCB)

//global flags and counters for interrupt routines
volatile char gPowerOnFlag = 0;
volatile char gLockFlag = 0;
volatile int g3sCounter = 0;
volatile int g250HzCounter = 0;
volatile char gReadTempFlag = 0;

//Misc button flags
char gUnits = 0; //0 = F, 1 = C
char gLightOn = 0;

//State machine variables
enum SM_STATES {
  IDLE_STATE,
  UNITS_PRESS_STATE,
  UNITS_CHANGE_STATE
};

enum SM_STATES SM_State = IDLE_STATE;

/////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //Pin Definitions
  pinMode(POWER_BUTTON, INPUT);
  pinMode(LOCK_BUTTON, INPUT);
  pinMode(UNITS_BUTTON, INPUT);
  pinMode(CAL_BUTTON, INPUT);
  pinMode(LIGHT_BUTTON,INPUT);
  pinMode(LED_LIGHT, OUTPUT);
  pinMode(POT_TEMP_INPUT, INPUT);
  pinMode(DIGITAL_TEMP_INPUT,INPUT);

  //Temperature Sensor Startup
  sensors.begin();
  if (!sensors.getAddress(tempProbe, 0)) {
    for(;;);
  }
  sensors.setResolution(tempProbe, 9);

  //OLED Screen Startup
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //If the test fails, loop forever
    for(;;); 
  }

  //Gyro Startup
  //Begin I2C communication with gyroscope
  Wire.begin();
  //initialize the gyroscope
  accelgyro.initialize();

  //Timers and Interrupts
  noInterrupts();
  
  //TIMERS

  //Timer 1 (3 seconds)
  TCCR1A = 0;
  TCCR1B = 0;
  
  //Timer rolls over at 46875 (happens every three seconds, since it counts 15625 every second)
  OCR1A = TIMER_COMPARE_3S;

  //CTC Mode (Compare to OCR1A)
  TCCR1B |= (1 << WGM12);

  //Reset Count
  TCNT1 = 0;

  //Prescales by 1024 (now runs at 16M/1024 = 15,625 Hz)
  //**STARTS WHEN THE UNITS BUTTON IS PRESSED**
  //TCCR1B |= (1 << CS10) | (1 << CS12);


  //Timer 2 (250 Hz --> 1s (handled in ISR))
  TCCR2A = 0;
  TCCR2B = 0;

  OCR2A = TIMER_COMPARE_250HZ;
  //CTC Mode (Compare to OCR2A)
  TCCR2A |= (1 << WGM21);
  //Prescales by 256 (now runs at 16M/256 = 62,500 Hz)
  TCCR2B |= (1 << CS21) | (1 << CS22); 
  
  //INTERRUPTS
  
  //Set interrupt when timer = OCR1A (aka when it rolls over)
  TIMSK1 |= (1 << OCIE1A);
  TIMSK2 |= (1 << OCIE2A);
  //Set external interrupts for buttons
  EICRA |= 0xF; //Rising edge for both INT0 and INT1, EICRA = 0b1111
  //Power button interrupt (pin 2)
  EIMSK |= (1 << INT0);
  //Lock button interrupt (pin 3)
  EIMSK |= (1 << INT1);

  interrupts();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/********************************************
               Main Loop
********************************************/
void loop() {
  //Main loop when the power has been turned on
  if(gPowerOnFlag) 
  {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //Runs the LED
    (gLightOn) ? digitalWrite(LED_LIGHT, HIGH) : digitalWrite(LED_LIGHT, LOW);
    switch(SM_State) 
    {
      case IDLE_STATE:
        //check buttons
        checkButtons();
        //read and display temperature
        readTemperature();
        displayTemperature();
        break;
      case UNITS_PRESS_STATE:
        //start timer        
        startTimer1();
        //leaves state if unit button is released
        if(!digitalRead(UNITS_BUTTON)) {
          SM_State = IDLE_STATE;
          resetTimer1();
        }
        //Display temperature
        displayTemperature();
        //When you remain in this state for 3 seconds, reset the timer go to UNITS_CHANGE_STATE
        if(g3sCounter)
        {
          resetTimer1();
          SM_State = UNITS_CHANGE_STATE;
        }
        break;
      case UNITS_CHANGE_STATE:
        //Change the units then go back to idle state
        gUnits = !gUnits;
        SM_State = IDLE_STATE;
        break;
    }
  } else {
    //If power isn't on, turn off screen
    display.clearDisplay();
    display.display();
  }
}


/*********************************************
          User-Defined Functions
*********************************************/



/********************
Temperature Functions
*********************/

void readPotTemperature() {
  gPotReading = analogRead(POT_TEMP_INPUT);
}

void readTemperature() {
  if(gReadTempFlag) {
    gReadTempFlag = 0;
    sensors.requestTemperatures();
    tempC = sensors.getTempC(tempProbe);
    if (tempC == DEVICE_DISCONNECTED_C)
    {
      tempC = 1000;
      return;
    } 
  }
}

void displayTemperature() {
  //clear and setup the screen
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  
  //Check orientation
  if(ax < -2000) tilt = 0;
  else if(ax > 2000) tilt = 1;
  else;

  (tilt) ? display.setRotation(2) : display.setRotation(0);

  //Print temperature
  display.setCursor(0,8);
  (gUnits) ? display.print(tempC) : display.print(DallasTemperature::toFahrenheit(tempC));

  //Print units
  (gUnits) ? display.drawBitmap(0,0,images[2],128,32,WHITE) : display.drawBitmap(0,0,images[3],128,32,WHITE);

  //Print locked/unlocked
  (gLockFlag) ? display.drawBitmap(0,0,images[0],128,32,WHITE) : display.drawBitmap(0,0,images[1],128,32,WHITE);


  display.display();
}

/***********************************
Timer Functions (See also interrupts)
***********************************/

void startTimer1() {
  //Assign CS bits - Starts timer prescaled at 1024 (15625 Hz)
  TCCR1B |= (1 << CS10) | (1 << CS12);
}
void resetTimer1() {
  //Reset CS bits (no longer selects a prescaler output)
  TCCR1B &= ~(1 << CS10) & ~(1 << CS12);
  //Reset count value to zero
  TCNT1 = 0;
  //Reset counter
  g3sCounter = 0;
}

/*************************************
Button Functions (See also interrupts)
*************************************/

void checkButtons() {
  //Read Units Button
  if(digitalRead(UNITS_BUTTON))
  {
    SM_State = UNITS_PRESS_STATE;
  }
  //Read Light Button
  if(digitalRead(LIGHT_BUTTON)) {
    delay(50);
    gLightOn = !gLightOn;
  }
  //Calibration button not included in the code
}

/***************************************
        Interrupt Routines
****************************************/

//Power button
ISR(INT0_vect) {
  gPowerOnFlag = !gPowerOnFlag;
};
//Lock button (Stops/starts TIMER2 which measures temperature)
ISR(INT1_vect) {
  //Flips locked state
  gLockFlag = !gLockFlag;
  if(gLockFlag) 
  {
    //Stops timer
    TCCR2B = 0;
  } else {
    //Starts timer with 62500 Hz
    TCCR2B |= (1 << CS21) | (1 << CS22);
  }
};

//3 second timer
ISR(TIMER1_COMPA_vect) {
  g3sCounter++;
};
//250Hz timer / 1 second timer
ISR(TIMER2_COMPA_vect) {
  g250HzCounter++;
  if(g250HzCounter == 250) {
    gReadTempFlag = 1;
    g250HzCounter = 0;
  }
};