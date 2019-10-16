 /* ___   _  _______  ___  ____   ___  ___  ____  ___    __________ _      _________    _________  ___  ____
  // _ | / |/ / _ \ \/ ( )/ __/  / _ \/ _ \/ __ \/ _ \  /_  __/ __ \ | /| / / __/ _ \  / ___/ __ \/ _ \/ __/
 // /_ |/    / // /\  /|/_\ \   / // / , _/ /_/ / ___/   / / / /_/ / |/ |/ / _// , _/ / /__/ /_/ / // / _/  
//_/ |_/_/|_/____/ /_/  /___/  /____/_/|_|\____/_/      /_/  \____/|__/|__/___/_/|_|  \___/\____/____/___/  
                                                                                                         */
#include <Adafruit_GFX.h>         // Core graphics library
#include <SPI.h>                  // this is needed for display
#include <Adafruit_ILI9341.h>
#include <Wire.h>                 // this is needed for FT6206
#include <Adafruit_FT6206.h>
#include <EEPROM.h>               // EEPROM will remember the cycle count in the event of power failure


// The display uses hardware SPI, plus #9 & #10
const byte TFT_CS        = 10;                            // TFT pins are related to the screen
const byte TFT_DC        =  9;                            
const byte in1           = 12;                            // H-bridge white cable
const byte in2           = 13;                            // H-bridge black cable
const byte contactPin    =  2;                            // Contact sensor pin
const byte gatePin       =  3;                            // Photogate pin
const byte electromagPin = 11;                            // controls electromagnet
const byte limitPin      = 18;                            // limit switch 
const byte estopPin      = 19;                            // e-stop
const byte limitIn       =  0;          //@@@@@@@@@@@@@@@@@@@                  // new limit switch

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

byte motorSpeed;                                   // variable for the speed of the motor
int desPos                       = 0;              // desired position in encoder count
int error                        = 0;              // position error - difference between desired and actual position                             
volatile boolean touch;                            // is the contact sensor touching?
float kp                         = 0.01;           // proportional control gain
float ki                         = 0.0005;         // integral control gain
long errorHistory[] = {                            // this is used to calculate integral error
  0, 0, 0, 0, 0, 0
};
int integralError;                                 // this is the sum of the error history
uint16_t count;                                    // number of cycles to be performed
uint16_t cycles;                                   // number of cycles performed so far
int height;                                        // desired height in mm
int currentHeight;                                 // current height in mm
int lastHeight;                                    // last height, used so screen is only updated when the value changes
byte page;                                         // what page is the screen on?
byte runMode       = 0;                            // 0 is setup, 1 is running test
byte state         = 4;                            // state the system is in
int limit;

/* States: 
 * 0 - slowly lower drive sled until it makes contact
 * 1 - lowers drive sled and engages electromagnet
 * 2 - raises sled to desired position
 * 3 - disengages electromagnet
 * 4 - do nothing at all! */
 
byte err           = 0;                             // 0 is no error, 1 is limit switch, 2 is estop, 3 is velocity
//note: err variable is to stop the test and show a relevant message; error is related to the controller

byte stopcount;
byte moving;               // 1 if drive sled is in motion, 0 if it is stopped
byte lastMoving;
byte armed = 0;

// stopcount, moving, lastMoving, and armed are related to the variable controller gains


byte button;
const byte buttonPin = 5;                           // this button is used to reset the screen if it goes white

byte estop      = 0;                                // 0 if no estop error, 1 if estop is pressed
byte resetTimer;                                    // how long the user has held the reset button, prevent accidental reset
volatile byte gateState;                            // is the photogate tripped?
volatile byte lastGateState = 0;                    // used for edge detection of photogate
float velErr;                                       // % difference between current velocity and first velocity
float sledVel;                                      // calculated sled velocity
float lastVel;                                      // previous velocity; only update the screen when the value changes
float firstVel;                                     // the velocity recorded from the first cycle, compare to this to find error
volatile unsigned long micros1;                     // micros are for the photogate
volatile unsigned long micros2;
unsigned long lastMillis;                           // millis are to time system steps
unsigned long currentMillis;
unsigned long interval;                             // how long the photogate was tripped for in microseconds

/* encoder chip */
// set pin numbers:
const byte SEL1 = 25;       // SEL1
const byte SEL2 = 26;       // SEL2
const byte OEN = 24;        // OEN
const byte RST = 23;        // Reset

const byte D0 = 27;
const byte D1 = 28;
const byte D2 = 29;
const byte D3 = 30;
const byte D4 = 31;
const byte D5 = 32;
const byte D6 = 33;
const byte D7 = 34;

byte new_msb  = B00000000;
byte new_smsb = B00000000;
byte new_tmsb = B00000000;
byte new_lsb  = B00000000;

byte old_msb  = B00000000;
byte old_smsb = B00000000;
byte old_tmsb = B00000000;
byte old_lsb  = B00000000;

unsigned long displacement;
unsigned long temp; 

boolean settle = true;
unsigned long mult = 1;
long sdisplacement = 0;

/*****SETUP*****/

void setup() 
{
  Serial.begin (38400);
  pinMode(in1, OUTPUT);                       // declare H-bridge/motor driver white wire as output
  pinMode(in2, OUTPUT);                       // declare H-bridge/motor driver black wire as output
  pinMode(contactPin, INPUT);                 // declare contact sensor as input
  pinMode(electromagPin, OUTPUT);             // declare electromagnet as output
  pinMode(gatePin, INPUT);                    // declare photogate as input
  pinMode(limitPin, INPUT_PULLUP);            // input goes low when limit switch is tripped (NO)
  pinMode(estopPin, INPUT_PULLUP);            // input goes high when estop is pressed (NC)
  pinMode(buttonPin, INPUT);                  // screen button is input
  pinMode(limitIn, INPUT); //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  // initialize encoder pins:
  pinMode(SEL1, OUTPUT);                      // SEL 1 and 2 are used to select which byte to read from the decoder chip
  pinMode(SEL2, OUTPUT);  
  pinMode(OEN, OUTPUT);                       // OEN tells the chip we're going to read values, so wait until we're done to update them
  pinMode(RST, OUTPUT);                       // this resets the encoder count
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);                         // D0-D7 are the 8 bits of the byte being read
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);  
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);  
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);  

  resetEncoder();                              // reset the encoder upon startup
  digitalWrite(in2, LOW);                      // send a stop command to the motor

  //set interrupts:
//  attachInterrupt(digitalPinToInterrupt(contactPin), contact, FALLING);   
  attachInterrupt(digitalPinToInterrupt(gatePin),    radarGun, CHANGE);       
  attachInterrupt(digitalPinToInterrupt(limitPin),   limitErr, FALLING);
  attachInterrupt(digitalPinToInterrupt(estopPin),   estopErr, RISING);
  
  tft.begin();                                // start screen
  tft.setRotation(3);                         // to orient the screen
  if (! ctp.begin(40))                        // pass in 'sensitivity' coefficient
  {
    Serial.println("Couldn't start FT6206 touchscreen controller");
    while (1);
  }
  Serial.println("Capacitive touchscreen started");
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(0,60);
  tft.setTextColor(ILI9341_GREEN);
  tft.println("  This tower made");
  tft.println("  operational by"); 
  tft.println("    Andy King");
  tft.println("   Summer 2018");
  delay(5000);
  tft.fillScreen(ILI9341_BLACK);  
  tft.setTextSize(2);
  err = 0; // there are no errors

  //THE EEPROM DOES NOT APPEAR TO WORK CURRENTLY!!!
  
  if (EEPROM.read(1) > 0)        // if there are cycles stored in the EEPROM, load the stored number
  {
    cycles = ((EEPROM.read(0)<<8) + EEPROM.read(1));
  }
}

//*********************************************************************************************************

void loop() 
{
  limit = analogRead(limitIn);
  Serial.println(limit);
  if (limit < 480) touch = 0;
  else touch = 1;
  if (limit > 600) err = 1;
  
  //@@@@@@@@@@@@@@@@@@@@@@
  button = digitalRead(buttonPin);      //reset the screen if the user presses the button!
  if (button) 
  {
    tft.begin();
    tft.setRotation(3);
    clearScreen();
  }
  /* data retrieval and calculations
     the desPos is in encoder count. The 21.032 number was calculated by measuring the diameter of the drive cog 
     and determining how much the drive sled raises per revolution, then multiplying by 800 pulses per revolution
     to determine how many pulses are counted per mm of vertical sled travel */
  
  desPos = 21.032 * height;                      // calculate desired position in pulse counts from desired height in mm
  currentHeight = (sdisplacement+10) / 21.032;   // calculate current height in mm; the +10 is to "center" truncated values when they round
  // touch = digita lRead(contactPin);               // read touch sensor   
  TS_Point p = ctp.getPoint();                   // Retrieve a point from touchscreen  
  p.x = map(p.x, 0, 240, 240, 0);                // flip it around to match the screen
//  p.y = map(p.y, 0, 320, 320, 0);              // this was unused because of screen orientation; leaving here just in case
  currentMillis = millis();                      // record the current time


  for (byte ii = 5; ii > 0; ii--)               // store last 6 error values 
  {
    errorHistory[ii] = errorHistory[ii-1];
  }
  integralError = 0;
  for (byte ii = 0; ii < 6 ; ii++)              // sum integral error array
  {
    integralError += errorHistory[ii];
  }
  
  if (errorHistory[0] == errorHistory[5])
  {
    moving = 0;
  }
  if (errorHistory[0] != errorHistory[5])
  {
    moving = 1;
  }
  
/* ON THE INTEGRAL ERROR:
 * The integral controller is useful for lifting varying weights. A proportional controller gives a 
 * larger command when the error is larger, slowing the sled down as it approaches the desired position.
 * Sometimes when the sled is very close to the desired position the small error yields a small command
 * which cannot overcome the weight of the drop sled. An integral controller sums the error terms over a
 * given period of time, which lets the controller overcome this "dead zone" and reach the desired position
 * even when there is a small error.
 */

/*______ _   _  _____ ____  _____  ______ _____  
 |  ____| \ | |/ ____/ __ \|  __ \|  ____|  __ \ 
 | |__  |  \| | |   | |  | | |  | | |__  | |__) |
 |  __| | . ` | |   | |  | | |  | |  __| |  _  / 
 | |____| |\  | |___| |__| | |__| | |____| | \ \ 
 |______|_| \_|\_____\____/|_____/|______|_|  \_\
*/

 
      PORTA |= 1 << PA1; //digitalWrite(OEN, HIGH); // disable OE
      delay(25);
      
      // GET Most significant byte
      PORTA &= ~(1 << PA3);    //digitalWrite(SEL1, LOW);
      PORTA |= 1 << PA4;       //digitalWrite(SEL2, HIGH);
      PORTA &= ~(1 << PA1);    //digitalWrite(OEN, LOW);  // enable OE     
      get_hi();
      
      PORTA |= 1 << PA3;       //digitalWrite(SEL1, HIGH);
      PORTA |= 1 << PA4;       //digitalWrite(SEL2, HIGH);
      get_2nd();
      
      PORTA &= ~(1 << PA3);    //digitalWrite(SEL1, LOW);
      PORTA &= ~(1 << PA4);    //digitalWrite(SEL2, LOW);
      get_3rd();      
  
      PORTA |= 1 << PA3;       //digitalWrite(SEL1, HIGH);
      PORTA &= ~(1 << PA4);    //digitalWrite(SEL2, LOW);
      get_low();
    
      PORTA |= 1 << PA1;       //digitalWrite(OEN, HIGH); // disable OE
      delay(25);
      
      mult = 1;
      temp = long(new_lsb)*mult;
      displacement = temp;
      
      mult = mult*256;
      temp = long(new_tmsb)*mult;
      displacement = displacement + temp;
      
      mult = mult*256;
      temp = long(new_smsb)*mult;
      displacement = displacement + temp;
      
      mult = mult*256;
      temp = long(new_msb)*mult;
      displacement = displacement + temp;
      
      if (displacement < 2147483648)
      {
        sdisplacement = displacement + 2147483648; 
      }  
      else
      {
        sdisplacement = displacement - 2147483648;
      }
 
      sdisplacement = sdisplacement - 2147483648;


/* _____  _____ _____  ______ ______ _   _ 
  / ____|/ ____|  __ \|  ____|  ____| \ | |
 | (___ | |    | |__) | |__  | |__  |  \| |
  \___ \| |    |  _  /|  __| |  __| | . ` |
  ____) | |____| | \ \| |____| |____| |\  |
 |_____/ \_____|_|  \_\______|______|_| \_|                                           
*/                                           

//The code between here and the RUNNING section is used to create the user interface for the various screens

/*****RUN PAGE*****/

if (err == 0)   // if there are no error messages (e-stop, limit, velocity) continue
{
if (page == 0)   //page 0 is the run page
{
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 30);
  tft.print("DROP:   ");      tft.print(height);        tft.println("mm");    //always show the user-set parameters
  tft.print("HEIGHT: ");      tft.print(currentHeight); tft.println("mm");
  tft.print("CYCLES: ");      tft.println(count);
  tft.print("COMPLETED: ");   tft.println(cycles);

  if (currentHeight != lastHeight)  //if the current height variable has changed, update the screen
  {
    lastHeight = currentHeight;
    clearHeight2();
  }
  
  if (runMode == 0)     //show the following in setup mode
  {
    digitalWrite(electromagPin, LOW);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(0, 0);               tft.println("SETUP");
    tft.setCursor (250, 213);          tft.print("SET P");                                 // set parameters button
    tft.setTextColor(ILI9341_BLUE);    tft.setCursor (158, 190);      tft.print("FIND");   // find button
    tft.setTextColor(ILI9341_GREEN);   tft.setCursor (40, 190);       tft.print("RUN");    // run button
    tft.setTextColor(ILI9341_RED);     tft.setCursor (250, 13);       tft.print("RESET");  // reset button

    tft.drawRect(120, 160, 120, 80, ILI9341_BLUE);                        // find button
    tft.drawRect(  0, 160, 120, 80, ILI9341_GREEN);                       // run button
    tft.drawRect(240, 200,  80, 40, ILI9341_YELLOW);                      // setup button
    tft.drawRect(240,   0,  80, 40, ILI9341_RED);                         // reset button
    tft.drawRect(240,  40,  80, 80, ILI9341_WHITE);                       // jog buttons
    tft.drawRect(240, 120,  80, 80, ILI9341_WHITE);
    tft.fillTriangle(280,  55, 260,  95, 300,  95, ILI9341_WHITE);        // jog up arrow
    tft.fillTriangle(280, 185, 260, 145, 300, 145, ILI9341_WHITE);        // jog down arrow

    if ((! ctp.touched()) && (state != 0)) 
    {
      analogWrite(in1, 190);             // stop the motor if screen is not being touched while in setup mode, unless finding home (state 0) **********
      resetTimer = 0;                    // prevent accidental resets by setting the reset timer to zero when screen not touched
      state = 4;
    }
    if (p.y >= 240)                      // this is the right side of the screen
    {
      if (p.x < 40)                      // reset count, height, etc to zero for new test
      {
        if (resetTimer >= 50)
        {
          clearScreen();
          count         = 0;
          cycles        = 0;
          height        = 0;
          velErr        = 0;
          firstVel      = 0;
          sledVel       = 0;
          resetTimer    = 0;
          integralError = 0;
          interval      = 0;
          kp            = 0.01;
          ki            = 0.0005;
          EEPROM.write(0, 0);
          EEPROM.write(1, 0);
        }
        else
        {
          resetTimer++;                  //increase reset timer while button is held down
          delayMicroseconds(250);
        }
      }
      else if (p.x < 120)                    // if jog up button is being pressed
      {
        if (sdisplacement < 16825)      // if the current height is less than the total height of the tower
        {
          analogWrite(in1, 114);        //jog up
        }
      }
      else if (p.x < 200)                   // if jog down button is pressed
      {
        if (touch == 1)          analogWrite(in1, 250);        // jog down
        if (touch == 0)          analogWrite(in1, 190);        // don't jog if in contact with drop sled
      }
      else if (p.x < 240)                                      //change to set parameter screen when SET P button is pressed
      {
        analogWrite(in1, 190);                                 //stop motor before switching screens
        page = 1;
        clearScreen();
        return;
      }      
    }
    if (p.x > 160 && p.x != 240 && p.y > 120 && p.y < 240)     // if the find button is pressed, find zero
    {
      if (state != 0)
      {
        state = 0;
      }
      
    }
    if (p.x > 160 && p.y < 120 && p.x != 240 && p.y != 0)      // if run button pressed, enter run mode
    {
      if (touch == 0)
      {
        clearScreen();                      // clear screen so running parameters are shown
        lastMillis = currentMillis;         // remember the time when the test is started
        runMode    = 1;
        state      = 2;                     // go straight to lifting state
        err        = 0;                     // clear error messages if user runs test
      }
      else
      {
        clearScreen();
        tft.setCursor(0, 100);
        tft.println("Press Find!");
      }
    }
  }
  else if (runMode == 1)
  {
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(0, 0);      tft.println("RUNNING...");
    tft.drawRect(200, 160, 120, 80, ILI9341_RED);                                       // draw stop button
    tft.setTextColor(ILI9341_RED);     tft.setCursor (235, 190);    tft.print("STOP");   // stop button - setup mode
    tft.setCursor(0, 94); tft.setTextColor(ILI9341_WHITE);                               // position display of running data below parameters
    tft.print("VELOCITY:  ");   tft.print(sledVel);    tft.println(" m/s");
    tft.print("VEL ERROR: ");   tft.print(velErr);     tft.print(" %");
    
    if (p.y > 200 && p.x > 160)         // if STOP pressed: stop motor, enter setup mode
    {
      digitalWrite(in1, LOW);           //stop motor
      state     = 4;                    
      runMode   = 0;                  //enter setup mode
      clearScreen();                  //clear for screen change
    }
  }
}

/*****SET PARAMETERS PAGE*****/

 if (page == 1)
 {
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("SET PARAMETERS");
  tft.drawRect(240, 0, 80, 40, ILI9341_GREEN);
  tft.setCursor(258,13);
  tft.print("DONE");

  /*****SET THE HEIGHT*****/
  tft.drawRect(  0, 100, 80, 40, ILI9341_WHITE);   //these are the buttons to add to the height
  tft.drawRect( 80, 100, 80, 40, ILI9341_WHITE);
  tft.drawRect(160, 100, 80, 40, ILI9341_WHITE);
  tft.drawRect(240, 100, 80, 40, ILI9341_WHITE);
  tft.drawRect(240,  60, 80, 40, ILI9341_RED);
  tft.drawFastHLine(0, 60, 240, ILI9341_WHITE);  

  tft.setCursor( 10, 113);    tft.print("+500");
  tft.setCursor( 95, 113);    tft.print("+100");
  tft.setCursor(180, 113);    tft.print("+10");
  tft.setCursor(270, 113);    tft.print("+1");
  tft.setCursor( 10,  73);    tft.print("HEIGHT(mm):");  
  tft.setCursor(150,  73);    tft.print(height);
  tft.setCursor(250,  73);    tft.print("RESET");

  if (p.x >= 100 && p.x <= 140 && p.y > 0)
  {
    if (p.y < 80)
    {
      height += 500;
      clearHeight();
      delay(200);
    }
    else if (p.y < 160)
    {
      height += 100;
      clearHeight();
      delay(200);
    }
    else if (p.y < 240)
    {
      height += 10;
      clearHeight();      
      delay(200);
    }
    else if (p.y < 320)
    {
      height += 1;
      clearHeight();
      delay(200);
    }
  }
  if (height > 800)
  {
    height = 800;   //don't let the user enter values that will definitely crash the machine
  }
  if (p.x > 60 && p.x < 100 && p.y > 240) //set to zero when reset is pressed
  {
    height = 0; clearHeight();
  }
  
  /*****SET THE COUNT*****/
  tft.drawRect(  0, 200, 80, 40, ILI9341_WHITE);
  tft.drawRect( 80, 200, 80, 40, ILI9341_WHITE);
  tft.drawRect(160, 200, 80, 40, ILI9341_WHITE);
  tft.drawRect(240, 200, 80, 40, ILI9341_WHITE);
  tft.drawRect(240, 160, 80, 40, ILI9341_RED);
  tft.drawFastHLine(0, 160, 240, ILI9341_WHITE);

  tft.setCursor( 10, 213);   tft.print("+1000");
  tft.setCursor( 95, 213);   tft.print("+100");
  tft.setCursor(180, 213);   tft.print("+10");
  tft.setCursor(270, 213);   tft.print("+1");
  tft.setCursor( 10, 173);   tft.print("# CYCLES:");  
  tft.setCursor(150, 173);   tft.print(count);
  tft.setCursor(250, 173);   tft.print("RESET");
  
  if (p.x > 200 && p.y > 0)
  {
    if (p.y < 80)
    {
      count += 1000;
      clearCount();
      delay(200);
    }
    else if (p.y < 160)
    {
      count += 100;
      clearCount();
      delay(200);
    }
    else if (p.y < 240)
    {
      count += 10;
      clearCount();      
      delay(200);
    }
    else if (p.y < 320)
    {
      count += 1;
      clearCount();
      delay(200);
    }
  }
  if (p.x > 160 && p.x < 200 && p.y > 240)
  {
    count = 0; clearCount();   //set to zero when reset is pressed
  }
  if (p.x < 40 && p.y > 240)   //switch to run page
  {
    clearScreen();
    page = 0;
  }  
}
}


/*_____  _    _ _   _ _   _ _____ _   _  _____ 
 |  __ \| |  | | \ | | \ | |_   _| \ | |/ ____|
 | |__) | |  | |  \| |  \| | | | |  \| | |  __ 
 |  _  /| |  | | . ` | . ` | | | | . ` | | |_ |
 | | \ \| |__| | |\  | |\  |_| |_| |\  | |__| |
 |_|  \_\\____/|_| \_|_| \_|_____|_| \_|\_____|
 */                                              

//calculate drop sled speed using photogate!
  if (micros2 > micros1 && state == 1)   // only count it when it drops, not when its being picked up during state 2
  {
    interval = micros2 - micros1;
    sledVel = 12700/(float)interval;    // the (float) is called casting - took lots of googling to figure this out
  }
  if (cycles == 1)
  {
    firstVel = sledVel;                 // remember the velocity from the first cycle to compare subsequent velocities to
  }
  if (cycles > 1)
  {
    velErr = (sledVel - firstVel)/(0.01 * firstVel);    //find % error of sled velocity after first cycle
  }
  if (err > 0)                                          // check for error messages
  {
    digitalWrite(in1, LOW);                             // if there's an error, immediately stop the motor
    if (err == 1)
    {
      doLimit();
    }
    if (err == 2)
    {
      doEstop();
    }
    if (err == 3)                //err 3 is a large velocity change
    {
      runMode = 0;                //stop test
      err     = 0;                //reset err so test can be restarted
      clearScreen();
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(0, 112); tft.println("ERROR:"); tft.print(">5% VELOCITY CHANGE");
    }
  }
  else          // move on to operation; THIS ELSE MAY UNNECESSARY; TEST WHEN MACHINE IS OPERATIONAL
  {
    if (runMode == 1)    //runmode 1 means the test is running
    {
      if (sledVel != lastVel)    //update screen if velocity variable has changed
      {
        lastVel = sledVel;       //setting the last vel equal to the current only triggers -
        clearStats();            // - this if statement if the value has changed
      }
      if (abs(velErr) > 5)
      {
        err = 3;
        runMode = 0;
      }
      if (cycles < count)       //we aren't done yet
      {
        switch(state)           //this switch statement issues the command for whichever state we're in
        {
          case 0:
            state0();           //state 0 slowly lowers the drive sled until it contacts the drop sled
            break;
          case 1:
            state1();           //state 1 lowers the sled and engages the electromagnet
            break;
          case 2:
            state2();           //state 2 raises the sled to the desired height with the electromagnet engaged       
            break;
          case 3:
            state3();           //state 3 releases the electromagnet
            break;
          case 4:
            motorSpeed = 190;   //state 4 stops the motor
            break;
        }
        analogWrite(in1, motorSpeed);                 //write speed to motor
      }
      if (cycles == count)       //when we're done
      {
        runMode = 0;
        clearScreen();
        tft.setCursor(0, 112); tft.setTextColor(ILI9341_GREEN); tft.println("TEST COMPLETE!");
        EEPROM.write(0, 0);   //clear EEPROM
        EEPROM.write(1, 0);
      }
    }
    
    //state 0 lowers the sled slowly until it makes contact and resets the encoder
    
    if (state == 0) 
    {
      state0();
    }
  }
    /* Serial prints */
//  Serial.print(stopcount); Serial.print("     ");
//  Serial.print(state); Serial.print("     ");
//  Serial.print(sledVel); Serial.print("     ");
//  Serial.print(velErr); Serial.print("     ");
//  Serial.println(interval);
}

/*************************************************************************/

void contact()
{
  if (sdisplacement < 100) touch = 0;  //sdisplacement condition is to prevent false readings upon descent
}

/*************************************************************************/

void radarGun() 
{
  /* The machine is in state 1 when the sled is falling; using state 1 as a condition for recording the reading
   * prevents the code from measuring velocity when the sled is being raised.
   * micros2 is the time when the leading edge of the flag passes the photogate. micros1 is when the
   * trailing edge of the flag passes the photogate. Taking the difference of these values, one can
   * calculate the velocity of the sled  */
  //gateState = digitalRead(gatePin); PINA & (1<<PE5)
  if (PINE & (1<<PE5) && state == 1) 
  {
    micros2 = micros();
  }
  else
  {
    micros1 = micros();
  }
}

/*************************************************************************/

void state0()
/* This is the function for state 0, which slowly lowers the sled until contact is made and resets the encoder 
   I chose to use the motorspeed variable instead of simply analogWrite(190) so that i could serial print and
   see what's happening when I'm debugging the code */
{
  if (touch == 0)   //touch = 0 when the magnet contacts the drop sled
  {
    motorSpeed = 190;                     // stop motor
    analogWrite(in1, motorSpeed);
    resetEncoder();                       // measure from this position by resetting encoder
    if (runMode == 0) state = 4;          // if this is done on setup page, simply stop the motor
    if (runMode == 1) state = 2;          // if this is done while the test is running, continue with test
  }
  else 
  {
    digitalWrite(electromagPin, 0);        // disengage electromagnet
    motorSpeed = 196;                      // lower sled slowly 
  }
  analogWrite(in1, motorSpeed);            // send command to motor
}

/*************************************************************************/

void state1()        // state 1 lowers the sled until it makes contact

/* I've had issues with false positives from the contact sensor causing this state to end prematurely, so there
   are many conditions in the if statement to counterract that. There is likely a better way to do this.
   Currently it checks the time since the last step, the contact sensor, and the absolute position and only 
   continues if there is contact, it's been more than 1.5s since the last step, and the sled is below ~2.5cm 
   I have a few ideas commented out; feel free to test them when the machine is working again 
   it currently uses 100 encoder count as the moveto argument because there has been encoder drift; this was
   a quick and dirty solution so that the ankle impaction test could continue. Ideally the first argument for
   moveto should be 0 when the sled is being lowered*/
{
//  if ((currentMillis - lastMillis) < 1500) return;  @@@@@@@@@POSSIBLY BUGGY, TRY THIS WHEN MACHINE IS OPERATIONAL @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  if (touch == 0 && (currentMillis - lastMillis) >= 1500 && sdisplacement < 500)
  {
    motorSpeed = 190;                        // stop sled
    analogWrite(in1, motorSpeed);            // write speed to motor
    digitalWrite(electromagPin, HIGH);       // engage electromagnet
    resetEncoder();
    delay(50);
    lastMillis = currentMillis;
    state = 2;
  }
  if (moving == 0 && touch == 1) state = 0; 
  // ^^if it's reached its destination but isn't touching, the part likely deformed; find zero again
  
//  else //@@@@@POSSIBLY BUGGY, TRY THIS WHEN MACHINE IS OPERATIONAL @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//  {
    moveto(0, 0.01, 0);                  //moveto is my P-I controller function
//  }
}

/*************************************************************************/

void state2()                              // state 2 raises the sled and weight to the desired height
// this function is well-behaved. It only moves on when the error is less than 10 encoder count or ~0.5mm
{
  digitalWrite(electromagPin, HIGH);      
  moveto(desPos, kp, ki);                  //moveto is my PI controller function
  if (moving == 0 && abs(error) < 10)       
  {
    motorSpeed = 190;
    analogWrite(in1, motorSpeed);          //stop motor when desired position is reached      
    lastMillis = currentMillis;
    state = 3;
  }
}

/*************************************************************************/

void state3()
{
    digitalWrite(electromagPin, LOW);                   //disengage electromagnet, add to count, reset to state 1
//    delay(50);
    clearStats();
    cycles++;
    lastMillis = currentMillis;
    if (cycles < count) moveto(0,0,0);
    state = 1;
    stopcount = 0;
    if (cycles <= 255)
    {
      EEPROM.update(1, cycles);
    }
    if (cycles > 255)
    {
      EEPROM.update(0, highByte(cycles));
      EEPROM.update(1, lowByte(cycles));
    }
}

/*************************************************************************/

void clearScreen()
{
  tft.fillScreen(ILI9341_BLACK);
}

/*************************************************************************/

void clearHeight()
{
  tft.fillRect(150, 73, 80, 14, ILI9341_BLACK);
}

/*************************************************************************/

void clearCount()
{
  tft.fillRect(150, 173, 80, 14, ILI9341_BLACK);
}

/*************************************************************************/

void clearStats()
{
  tft.fillRect(132, 62, 200, 80, ILI9341_BLACK);
}

/*************************************************************************/

void clearHeight2()
{
  tft.fillRect(90, 46, 100, 14, ILI9341_BLACK);
}

/*************************************************************************/

void moveto(int pos, float kpp, float kii)
{
  error = pos-sdisplacement;
  errorHistory[0] = error;
  if (abs(error) < 10) 
  {
    motorSpeed = 190;
  }
  else if (error > 0)
  {
    // make it go higher
    if (180 - (kpp*error + kii*integralError) < 114)       // to avoid command saturation   
    {         
      motorSpeed = 100;
    }
    else
    {
      motorSpeed = 180 - (kpp*error + kii*integralError);
    }
    if (state == 2 && error < 200 && error > 9 && moving == 0 && (currentMillis - lastMillis) > 1000)
    {
      motorSpeed = 176;
      if (lastMoving == 1)
      {
        stopcount++;
      }
      if (stopcount > 1)
      {
      if (kp < 0.0149)
      {
        kp = kp + 0.0025;
      }
      if (ki < 0.0007)
      {
        ki = ki + 0.0001;
      }
      stopcount = 0;
      }
    }

  }
  if (error < -9) 
  {
    // make it go lower
    if (195 - 0.01 * error > 250)                 // to avoid command saturation   
    {         
      motorSpeed = 245;
    }
    else
    {
      if (state == 2)
      {
        motorSpeed = 195;// - (0.001*error);
      }
      if (state == 1)
      {
        motorSpeed = 195 - (0.01 * error);
      }
    }
  }
  if (lastMoving != moving)
  {
    lastMoving = moving;
  }
  analogWrite(in1, motorSpeed);
}

/*************************************************************************/

void limitErr()
{
  err = 1;
  lastMillis = currentMillis;
}


/*************************************************************************/

void estopErr()
{
  err = 2;
  armed = 1;
}

/*************************************************************************/

void doLimit()
{
  if (currentMillis - lastMillis < 500)
  {
    analogWrite(in1, 200);
  }
  else
  {
    analogWrite(in1, 190);
    err = 0;
    clearScreen();
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(0, 112); tft.println("ERROR: LIMIT SWITCH");
    if (runMode == 0) 
    {
      tft.println("BE CAREFUL");
    }
    if (runMode == 1) 
    {
      tft.println("CHECK ENCODER"); runMode = 0;
    }
  }
}

/*************************************************************************/

void doEstop()
{
  analogWrite(in1, 190);
  if (armed == 1)
  {
    clearScreen();
    armed = 0;
  
  if (digitalRead(19) == 1)     // if e-stop button is pressed
  {
    tft.setCursor(0, 10);
    tft.setTextColor(ILI9341_WHITE);
    tft.println("E-STOP PRESSED");
    tft.println("STOPPING TEST");
    tft.println("RELEASE E-STOP TO RELEASE");
    tft.println("ELECTROMAGNET AND RESET");
  }
  }
  if (digitalRead(19) == 0)     //when e-stop is released
  {
    clearScreen();
    tft.setCursor(0, 112); tft.println("ERROR: E-STOP");
    runMode = 0;
    err = 0;
  }
  if (contact == 1)
  {
    digitalWrite(electromagPin, LOW);
  }
  if (contact == 0)
  {
    digitalWrite(electromagPin, HIGH);
  }
}

/*************************************************************************/

void get_hi(){

  settle = true;
  while(settle)
  {
  
    old_msb = B00000000;
    new_msb = B00000000;
  
    if (PINA & (1<<PA5)) {bitSet(old_msb,0);}
    if (PINA & (1<<PA6)) {bitSet(old_msb,1);}
    if (PINA & (1<<PA7)) {bitSet(old_msb,2);}
    if (PINC & (1<<PC7)) {bitSet(old_msb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(old_msb,4);}
    if (PINC & (1<<PC5)) {bitSet(old_msb,5);}
    if (PINC & (1<<PC4)) {bitSet(old_msb,6);}
    if (PINC & (1<<PC3)) {bitSet(old_msb,7);}
  
    if (PINA & (1<<PA5)) {bitSet(new_msb,0);}
    if (PINA & (1<<PA6)) {bitSet(new_msb,1);}
    if (PINA & (1<<PA7)) {bitSet(new_msb,2);}
    if (PINC & (1<<PC7)) {bitSet(new_msb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(new_msb,4);}
    if (PINC & (1<<PC5)) {bitSet(new_msb,5);}
    if (PINC & (1<<PC4)) {bitSet(new_msb,6);}
    if (PINC & (1<<PC3)) {bitSet(new_msb,7);}  
  
    if (old_msb == new_msb)
    {
    settle = false;
    }
  }
}


/*************************************************************************/


void get_2nd(){
  
  settle = true;
  while(settle)
  {
  
    old_smsb = B00000000;
    new_smsb = B00000000;
  
    if (PINA & (1<<PA5)) {bitSet(old_smsb,0);}
    if (PINA & (1<<PA6)) {bitSet(old_smsb,1);}
    if (PINA & (1<<PA7)) {bitSet(old_smsb,2);}
    if (PINC & (1<<PC7)) {bitSet(old_smsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(old_smsb,4);}
    if (PINC & (1<<PC5)) {bitSet(old_smsb,5);}
    if (PINC & (1<<PC4)) {bitSet(old_smsb,6);}
    if (PINC & (1<<PC3)) {bitSet(old_smsb,7);}
  
    if (PINA & (1<<PA5)) {bitSet(new_smsb,0);}
    if (PINA & (1<<PA6)) {bitSet(new_smsb,1);}
    if (PINA & (1<<PA7)) {bitSet(new_smsb,2);}
    if (PINC & (1<<PC7)) {bitSet(new_smsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(new_smsb,4);}
    if (PINC & (1<<PC5)) {bitSet(new_smsb,5);}
    if (PINC & (1<<PC4)) {bitSet(new_smsb,6);}
    if (PINC & (1<<PC3)) {bitSet(new_smsb,7);}  
  
    if (old_smsb == new_smsb)
    {
    settle = false;
    }
  }
}


/*************************************************************************/


void get_3rd(){
  
  settle = true;
  while(settle)
  {
  
    old_tmsb = B00000000;
    new_tmsb = B00000000;
  
    if (PINA & (1<<PA5)) {bitSet(old_tmsb,0);}
    if (PINA & (1<<PA6)) {bitSet(old_tmsb,1);}
    if (PINA & (1<<PA7)) {bitSet(old_tmsb,2);}
    if (PINC & (1<<PC7)) {bitSet(old_tmsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(old_tmsb,4);}
    if (PINC & (1<<PC5)) {bitSet(old_tmsb,5);}
    if (PINC & (1<<PC4)) {bitSet(old_tmsb,6);}
    if (PINC & (1<<PC3)) {bitSet(old_tmsb,7);}
  
    if (PINA & (1<<PA5)) {bitSet(new_tmsb,0);}
    if (PINA & (1<<PA6)) {bitSet(new_tmsb,1);}
    if (PINA & (1<<PA7)) {bitSet(new_tmsb,2);}
    if (PINC & (1<<PC7)) {bitSet(new_tmsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(new_tmsb,4);}
    if (PINC & (1<<PC5)) {bitSet(new_tmsb,5);}
    if (PINC & (1<<PC4)) {bitSet(new_tmsb,6);}
    if (PINC & (1<<PC3)) {bitSet(new_tmsb,7);}  
  
    if (old_tmsb == new_tmsb)
    {
    settle = false;
    }
  }  
}


/*************************************************************************/

void get_low(){
  
  settle = true;
  while(settle)
  {
  
    old_lsb = B00000000;
    new_lsb = B00000000;
  
    if (PINA & (1<<PA5)) {bitSet(old_lsb,0);}
    if (PINA & (1<<PA6)) {bitSet(old_lsb,1);}
    if (PINA & (1<<PA7)) {bitSet(old_lsb,2);}
    if (PINC & (1<<PC7)) {bitSet(old_lsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(old_lsb,4);}
    if (PINC & (1<<PC5)) {bitSet(old_lsb,5);}
    if (PINC & (1<<PC4)) {bitSet(old_lsb,6);}
    if (PINC & (1<<PC3)) {bitSet(old_lsb,7);}
  
    if (PINA & (1<<PA5)) {bitSet(new_lsb,0);}
    if (PINA & (1<<PA6)) {bitSet(new_lsb,1);}
    if (PINA & (1<<PA7)) {bitSet(new_lsb,2);}
    if (PINC & (1<<PC7)) {bitSet(new_lsb,3);}
  
    if (PINC & (1<<PC6)) {bitSet(new_lsb,4);}
    if (PINC & (1<<PC5)) {bitSet(new_lsb,5);}
    if (PINC & (1<<PC4)) {bitSet(new_lsb,6);}
    if (PINC & (1<<PC3)) {bitSet(new_lsb,7);}  
  
    if (old_lsb == new_lsb)
    {
    settle = false;
    }
  }  
}

/*************************************************************************/

void resetEncoder()
{
  digitalWrite (RST, LOW);
  delay(50);
  digitalWrite (RST, HIGH);
}


/*
    _   ______  _________________
   / | / / __ \/_  __/ ____/ ___/
  /  |/ / / / / / / / __/  \__ \ 
 / /|  / /_/ / / / / /___ ___/ / 
/_/ |_/\____/ /_/ /_____//____/  
                                
VERSION 2.0 includes port manipulation to read the decoder chip and controls for the stepservo motor
*/
