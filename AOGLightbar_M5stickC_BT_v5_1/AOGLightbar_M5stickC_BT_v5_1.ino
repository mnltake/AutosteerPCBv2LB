
  /*
  * USB Autosteer code For AgOpenGPS
  * 4 Feb 2021, Brian Tischler
  * Like all Arduino code - copied from somewhere else :)
  * So don't claim it as your own
  */
  
////////////////// User Settings /////////////////////////  

#include <Arduino.h>
#include <M5StickC.h>
#include <FastLED.h>
#include "BluetoothSerial.h"
  BluetoothSerial SerialAOG;
  #define NUMPIXELS   31                 // Odd number dont use =0 
  #define Neopixel_Pin 32                 //GPIO32:M5stickC  GPIO26:ATOMLite Set this to the pin number you are using for the Neopixel strip controll line
  #define cmPerLightbarPixel  2          // Must be a multiple of cmPerDistInt
  #define cmPerDistInt  2                // The number of centimeters represented by a change in 1 of the AOG cross track error byte
  
  CRGBArray<NUMPIXELS > leds;

  const uint8_t centerpixel = (NUMPIXELS-1)/2;
  uint8_t Neopixel_Brightness = 150;// default brightness value between 0 and 255



 
  //--------------------------- Switch Input Pins ------------------------
  #define STEERSW_PIN 26 //G26
  #define WORKSW_PIN 33  //G33
  #define REMOTE_PIN 0  //G0
  #define ANALOG_SENSOR_PIN 36 //G36

  #define CONST_180_DIVIDED_BY_PI 57.2957795130823

  //loop time variables in microseconds  
  const uint16_t LOOP_TIME = 20;  //50Hz    
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;
  
  const uint16_t WATCHDOG_THRESHOLD = 100;
  const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
  uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
  
   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;

  //show life in AgIO
  uint8_t helloAgIO[] = {0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };
  uint8_t helloCounter=0;

  //fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, Heading-7,8, 
        //Roll-9,10, SwitchByte-11, pwmDisplay-12, CRC 13
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
  int16_t AOGSize = sizeof(AOG);

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;


  float bno08xHeading = 0;
  double bno08xRoll = 0;
  double bno08xPitch = 0;

  int16_t bno08xHeading10x = 0;
  int16_t bno08xRoll10x = 0;

 
  //Relays
  bool isRelayActiveHigh = true;
  uint8_t relay = 0, relayHi = 0, uTurn = 0;
  uint8_t distanceFromLine = 255;  // cross track error - Autosteer PGN byte 10. Start at 255 so it is ignored untill a value is received from AOG
  uint8_t prevDistFromLine = 0;  // Used to only send the lightbar data if the distance has changed
  
  //Switches
  uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

  //On Off
  uint8_t guidanceStatus = 0;

  //speed sent as *10
  float gpsSpeed = 0;
  
  //steering variables
  float steerAngleActual = 0;
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  int16_t steeringPosition = 0; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  
  //pwm variables
  int16_t pwmDrive = 0, pwmDisplay = 0;
  float pValue = 0;
  float errorAbs = 0;
  float highLowPerDeg = 0; 
 
  //Steer switch button  ***********************************************************************************************************
  uint8_t currentState = 1, reading, previous = 0;
  uint8_t pulseCount = 0; // Steering Wheel Encoder
  bool encEnable = false; //debounce flag
  uint8_t thisEnc = 0, lastEnc = 0;

   //Variables for settings  
   struct Storage {
      uint8_t Kp = 40;  //proportional gain
      uint8_t lowPWM = 10;  //band of no action
      int16_t wasOffset = 0;
      uint8_t minPWM = 9;
      uint8_t highPWM = 60;//max PWM value
      float steerSensorCounts = 30;        
      float AckermanFix = 1;     //sent as percent
  };  Storage steerSettings;  //14 bytes

   //Variables for settings - 0 is false  
   struct Setup {
      uint8_t InvertWAS = 0;
      uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
      uint8_t MotorDriveDirection = 0;
      uint8_t SingleInputWAS = 1;
      uint8_t CytronDriver = 1;
      uint8_t SteerSwitch = 0;  //1 if switch selected
      uint8_t SteerButton = 0;  //1 if button selected
      uint8_t ShaftEncoder = 0;
      uint8_t PressureSensor = 0;
      uint8_t CurrentSensor = 0;
      uint8_t PulseCountMax = 5;
      uint8_t IsDanfoss = 0; 
  };  Setup steerConfig;          //9 bytes



void lightbar(uint8_t distanceFromLine ){
  int8_t cross_track_error = distanceFromLine - 127;
  const uint8_t cmPerLBPixel = cmPerLightbarPixel / cmPerDistInt;
  int8_t level = constrain((int8_t)(cross_track_error/cmPerLBPixel), -centerpixel, centerpixel);
  int8_t n = level + centerpixel;
  for (int i = 0 ;i < NUMPIXELS; i++){
    if (i == centerpixel && i == n){//Center
      leds[i] = CRGB::Yellow;
    }else if(level < 0 && i >= n && i < centerpixel && distanceFromLine != 255){ //Right Bar
      leds[i] = CRGB::Green;
    }else if (level > 0 && i <= n && i > centerpixel && distanceFromLine != 255){//Left Bar
      leds[i] = CRGB::Red;
    }else{
      leds[i] = CRGB::Black;//Clear
    }
    }
  FastLED.show();
  delayMicroseconds(50);  // tiny delay after sending the colours to allow the colours to latch in the LEDs
  if(distanceFromLine != 255){
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 15);
    M5.Lcd.setTextSize(5);
    if(cross_track_error > 0){
      M5.Lcd.setTextColor(RED);
      M5.Lcd.print("<");
    }else if(cross_track_error < 0){
      M5.Lcd.setTextColor(GREEN);
      M5.Lcd.print(">");
      cross_track_error*=-1;
    }else{
      M5.Lcd.setTextColor(WHITE);
    }
  M5.Lcd.setCursor(16, 15);
  M5.Lcd.printf("%4d",cross_track_error*cmPerDistInt);
  }
}

  void setup()
  { 

    //keep pulled high and drag low to activate, noise free safe   
    pinMode(WORKSW_PIN, INPUT_PULLUP); 
    pinMode(STEERSW_PIN, INPUT_PULLUP); 
    pinMode(REMOTE_PIN, INPUT_PULLUP); 

    
    //set up communication
    SerialAOG.begin("M5stickC LightBar"); //Bluetooth Serial

  M5.begin();
  M5.Lcd.setRotation(1);//1:PWR botton down 3:PWR button up
  FastLED.addLeds<WS2811,Neopixel_Pin,GRB>(leds, NUMPIXELS ).setCorrection(TypicalLEDStrip);
  M5.Lcd.fillScreen(TFT_BLACK);
  SerialAOG.write(helloAgIO,sizeof(helloAgIO));
  Serial.println("start");
  }// End of Setup

  void loop()
  {
    M5.update();
 
    // Loop triggers every 100 msec and sends back steer angle etc   
    currentTime = millis();
   
    if (currentTime - lastTime >= LOOP_TIME)
    {
      lastTime = currentTime;
  
      //reset debounce
      encEnable = true;
     
      //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
      if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;
  
      //read all the switches
      workSwitch = digitalRead(WORKSW_PIN);  // read work switch
      
      if (steerConfig.SteerSwitch == 1)         //steer switch on - off
      {
        steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
      }
      else if (steerConfig.SteerButton == 1)    //steer Button momentary
      {
        reading = digitalRead(STEERSW_PIN);      
        if (reading == LOW && previous == HIGH) 
        {
          if (currentState == 1)
          {
            currentState = 0;
            steerSwitch = 0;
          }
          else
          {
            currentState = 1;
            steerSwitch = 1;
          }
        }      
        previous = reading;
      }
      else                                      // No steer switch and no steer button
      {
        // So set the correct value. When guidanceStatus = 1, 
        // it should be on because the button is pressed in the GUI
        // But the guidancestatus should have set it off first
        if (guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
        {
          steerSwitch = 0;
          previous = 1;
        }

        // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
        if (guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
        {
          steerSwitch = 1;
          previous = 0;
        }
      }
      
      if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax) 
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }


      
      remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
      switchByte = 0;
      switchByte |= (remoteSwitch << 2); //put remote in bit 2
      switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
      switchByte |= workSwitch;   
    
    
      //send empty pgn to AgIO to show activity
      if (++helloCounter > 10)
      {
        SerialAOG.write(helloAgIO,sizeof(helloAgIO));
        helloCounter = 0;
      }
    } //end of timed loop
  
    //This runs continuously, not timed //// Serial Receive Data/Settings /////////////////
  
    // SerialAOG Receive
    //Do we have a match with 0x8081?    
    if (SerialAOG.available() > 1 && !isHeaderFound && !isPGNFound) 
    {
      uint8_t temp = SerialAOG.read();
      if (tempHeader == 0x80 && temp == 0x81) 
      {
        isHeaderFound = true;
        tempHeader = 0;        
      }
      else  
      {
        tempHeader = temp;     //save for next time
        return;    
      }
    }
  
    //Find Source, PGN, and Length
    if (SerialAOG.available() > 2 && isHeaderFound && !isPGNFound)
    {
      SerialAOG.read(); //The 7F or less
      pgn = SerialAOG.read();
      dataLength = SerialAOG.read();
      isPGNFound = true;
      idx=0;
    } 

    //The data package
    if (SerialAOG.available() > dataLength && isHeaderFound && isPGNFound)
    {
      if (pgn == 254) //FE AutoSteerData
      {
        //bit 5,6
        gpsSpeed = ((float)(SerialAOG.read()| SerialAOG.read() << 8 ))*0.1;
        
        //bit 7
        guidanceStatus = SerialAOG.read();

        //Bit 8,9    set point steer angle * 100 is sent
        steerAngleSetPoint = ((float)(SerialAOG.read()| SerialAOG.read() << 8 ))*0.01; //high low bytes
        
        if ((bitRead(guidanceStatus,0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1) )
        { 
          watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
        }
        else          //valid conditions to turn on autosteer
        {
          watchdogTimer = 0;  //reset watchdog
        }
        
        //Bit 10 Distance from line (cross track error)
        distanceFromLine = SerialAOG.read();
        
        //Bit 11 section 1 to 8
        relay = SerialAOG.read();
        
        //Bit 12 section 9 to 16
        relayHi = SerialAOG.read();
        
        
        //Bit 13 CRC
        SerialAOG.read();
        
        //reset for next pgn sentence
        isHeaderFound = isPGNFound = false;
        pgn=dataLength=0;      

                   //----------------------------------------------------------------------------
        //heading         
        AOG[7] = (uint8_t)9999;        
        AOG[8] = 9999 >> 8;
        
        //roll
        AOG[9] = (uint8_t)8888;  
        AOG[10] = 8888 >> 8;
        AOG[11] = switchByte;
        AOG[12] = (uint8_t)pwmDisplay;
        
        //add the checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < AOGSize - 1; i++)
        {
          CK_A = (CK_A + AOG[i]);
        }
        
        AOG[AOGSize - 1] = CK_A;
        SerialAOG.write(AOG, AOGSize);

        // Stop sending the helloAgIO message
        helloCounter = 0;
        //--------------------------------------------------------------------------              
      }
              
      
    
      //clean up strange pgns
      else
      {
          //reset for next pgn sentence
          isHeaderFound = isPGNFound = false;
          pgn=dataLength=0; 
      }        
  
    } //end if (SerialAOG.available() > dataLength && isHeaderFound && isPGNFound)      
  

    #if NUMPIXELS >0
      if (distanceFromLine != prevDistFromLine)  // only update the lightbar if it has changed
      {
        if(M5.BtnA.pressedFor(100))
        {
          Neopixel_Brightness = Neopixel_Brightness-32;
        }
        FastLED.setBrightness(Neopixel_Brightness);
        //Serial.println(Neopixel_Brightness);
        lightbar(distanceFromLine);
        prevDistFromLine = distanceFromLine;  // set the previous XTE value to the one we have just used for next time
      }
    #endif

  } // end of main loop
