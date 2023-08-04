/*
WELCOME TO THE CODE FOR CoralCam V3.1+

NOTE: The most recent tested camera for CoralCam boards is the SJCAM SJ4000 Air 4K 16MP
Wire as follows with camera lens facing you: 
- Mode positive from PCB to the bottom left prong of front button. 
- Mode negative from PCB to top left prong of this same button. 
- Shutter positive from PCb to the top button, left side prongs, prong closest to front face of camera.
- Shutter negative from PCB to rear left prong on this same button. 

This updated code enables users to do a couple things (new and old)...
- Set a specific start time and date for the camera to first boot up after deployment.
CoralCam will remain inactive and in a low-power sleep state until this start time/date occurs. 
- Set a sampling frequency (in seconds) of how often to collect data AFTER the start time/date has occurred.  
- Define two activity states a day time activity state with a specific sampling frequency, and a dark state activity with a specific frequency
- Define a range of hours, for example 09:00 (represented as "9") to 14:00 (represented as "14"), that represent
the times at which behvaiour changes between the two states. 

Instructions to program your CoralCam: 
1. Search for "STEP 1:", go to the following line: "RTC.setAlarm(ALM1_MATCH_HOURS, 15, 45, 20, 0);"

  Adjust the time/date above as needed. Above, 45 represents minutes, 20 represents hours, 
  and 15 represents seconds. So CoralCam will initiate its capture interval from the next time it encounters 
  20:45:15 (8:45:15 PM) after programming. If the sampling interval is set to 3600 (one hour in seconds), then the second 
  data capture will occur at 21:45:15 or (9:45:15 PM).

  Alternatively, replace the line with "RTC.setAlarm(ALM1_MATCH_DATE, 48, 14, 10);", which would allow you to set CoralCam to activate on
  the tenth day of the month, at 14 hours and 48 minutes. So if CoralCam is programmed on September 1st, it will not
  activate until September 10th at 14:48 (2:48 PM). This is useful if you want to deploy cameras well before data capture. 
  Note that the low-power sleep state of CoralCam is approximately 0.14 mA prior to the first activation and between scheduled data captures.

2. Search for "STEP 2:", go to the following line: "unsigned long alarmInterval = 3600;"

  Adjust the value to whatever sampling interval, in seconds, you want between data collections during the day period.
  For example, one hour is 3600 seconds, six hours is 21600 seconds, twelve hours is 43200 seconds, twenty-four hours is 86400 seconds.
  When writing your value, do not include any commas for values in the thousands etc. 
  Repeated sampling will occur at this interval AFTER the initial alarm time (Step 1). So, if 
  the initial alarm is set for September 3 at 14:00, and we have an interval of 180 seconds (3 minutes), then
  CoralCam will collect data at 14:00, 14:03, 14:06, 14:09, 15:02, etc until it runs out of camera battery. 

  NOTE: The shortest reccomended capture frequency is 60 seconds. 
  Cameras used with CoralCam require time and power to boot up. 

  Adjust the "unsigned long darkAlarmInterval = 3600" to the desired sampling frequency during the dark period, as above.

3. Search for "unsigned int Dawn =" or "unsigned int Dusk ="

  Adjust these values to represent the earliest and latest hours of the day (respectively) that you want 
  CoralCam to capture data. Setting Dawn = 0 and Dusk = 23 with a 3600 second sampling interval (1 hour) will 
  enable CoralCam to capture data every hour, 24 hours a day. Setting Dawn = 8 and Dusk = 18
  with a 7200 sampling interval will enable CoralCam to capture data every 2 hours from 08:00 to 18:00 each day.

  These times will reflect the time at which CoralCam behaviour changes. By default, dark actions are disabled unless
  the line defining them (see code) is uncommented. 

4. BEFORE UPLOADING! 
  In the Arduino IDE under "Tools" set the Board to "Arduino Pro or Pro Mini", set the Processor to "ATMEGA328P 3.3V 8MHz). 
  Plug in your USB FTDI programmer, and select the appropriate serial port under "Tools" in the Arduino IDE. 
  Set your FTDI programmer to 3.3V by adjusting the jumper, located near the pins on most FTDI boards. 
  Attach your FTDI programmer to the CoralCam PCB so that the GND and DTR pins on the programmer align with those marked on the PCB. 
  Hold the FTDI programmer gently in place to maintain a good connection, then hit "Upload" in the Arduino IDE. 
  Wait until the Arduino IDE reports "Done Uploading" before you disconnect the FTDI programmer from the KiloCam PCB.  
  
NOTES: 
- When programming CoralCam you should have a serial monitor open and set to 57600 baud. CoralCam will 
report back the current time, make sure this is correct before deploying. If it is not, use the "SetTime" or "SetTime_serial" 
examples from the Arduino RTCLib library to reprogram your PCB's real-time clock. Sketches from DS3232RTC.h work as well.      
- Power consumption of the CoralCam PCB is low, about 0.14 mA when sleeping and 5 mA when active, and 50 mA when capturing a photo. 
Be mindful of how long you can have CoralCam deployed based on your chosen power source and capture schedule (see above).
- Please make sure you have the DS3232RTC.h, Time.h, and LowPower.h libraries installed before attempting to load this
code onto your CoralCam. If you do not have these installed from the github directories linked below, the code will fail. 
- For more information on how to get more creative with your initial activation time or repeated 
sampling invervals, check out the DS3232RTC library repository on Github at the links below.
   
*/

#include <DS3232RTC.h>        // https://github.com/JChristensen/DS3232RTC
#include <LowPower.h>         // https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <SPI.h>

// Defined constants
#define RTC_ALARM_PIN 2 //pin for interrupt 

// Object instantiations
time_t          t, alarmTime;
tmElements_t    tm;

// Global variable declarations
volatile bool alarmIsrWasCalled = true;    // DS3231 RTC alarm interrupt service routine (ISR) flag. Set to true to allow first iteration of loop to take place and sleep the system.
unsigned long alarmInt;

// User defined global variable declarations
unsigned long alarmInterval = 300;          // STEP 2: Set sleep duration (in seconds) between data samples ... 300 = 5min
unsigned long darkAlarmInterval = 300;  // Set Dark period sleep duration 3600 = 1 hr
unsigned int Dawn = 0; // Specify the earliest hour (24 hour clock) you want captures to occur in. Or the time to move to dark action
unsigned int Dusk = 24; // Specify the latest hour (24 hour clock) you want captures to occur in Or the time to move to day action
//NOTE: The value you put for dusk is when the hour intervals will shift to darkalarminterval

// RTC Object
DS3232RTC RTC;

// Give pins names
int PowerMode = 6;
int Shutter = 5;
int LED = 13;

void setup()
{
  // DS3231 Real-time clock (RTC)
  RTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1);    // Initialize alarm 1 to known value
  RTC.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);    // Initialize alarm 2 to known value
  RTC.alarm(DS3232RTC::ALARM_1);                           // Clear alarm 1 interrupt flag
  RTC.alarm(DS3232RTC::ALARM_2);                           // Clear alarm 2 interrupt flag
  RTC.alarmInterrupt(DS3232RTC::ALARM_1, false);           // Disable interrupt output for alarm 1
  RTC.alarmInterrupt(DS3232RTC::ALARM_2, false);           // Disable interrupt output for alarm 2
  RTC.squareWave(DS3232RTC::SQWAVE_NONE);                  // Configure INT/SQW pin for interrupt operation by disabling default square wave output

  // Initializing camera/led pins
  pinMode(6, OUTPUT); //power and mode
  pinMode(5, OUTPUT); //shutter
  pinMode(13, OUTPUT); //LED indicator

  // Initialize serial monitor
  Serial.begin(57600);
  
  // Set interrupt pin
  pinMode(RTC_ALARM_PIN, INPUT);           // Enable internal pull-up resistor on external interrupt pin
  digitalWrite(RTC_ALARM_PIN, HIGH);
  attachInterrupt(0, alarmIsr, FALLING);       // Wake on falling edge of RTC_ALARM_PIN

  // Set initial RTC alarm
  RTC.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, 0, 0, 8, 0);  // STEP 1: Set initial alarm 1 here. Camera inactive until this time/date.
  RTC.alarm(DS3232RTC::ALARM_1);                             // Ensure alarm 1 interrupt flag is cleared
  RTC.alarmInterrupt(DS3232RTC::ALARM_1, true);              // Enable interrupt output for alarm

  // flash two short times to show KiloCam is powered on
  digitalWrite(13, HIGH);
  delay(250); 
  digitalWrite(13, LOW);   
  delay(250); 
  digitalWrite(13, HIGH);
  delay(250); 
  digitalWrite(13, LOW);   
  delay(250); 

  t = RTC.get(); // get time from RTC
  runCamera(); // do a demo photo cycle before going to sleep to verify all buttons work. Comment out this line with "//" to disable
}

// Loop
void loop()
{
  if (alarmIsrWasCalled)
  {
    Serial.println(F("Alarm ISR set to True! Waking up."));
    t = RTC.get();            // Read current date and time from RTC
    Serial.println(F("Current time is: "));
    Serial.print(year(t));
    Serial.print("/"); 
    Serial.print(month(t));
    Serial.print("/"); 
    Serial.print(day(t));
    Serial.print("  ");
    Serial.print(hour(t));
    Serial.print(":");
    Serial.print(minute(t));
    Serial.print(":");
    Serial.print(second(t));
    Serial.println("  ");

    if (RTC.alarm(DS3232RTC::ALARM_1))   // Check alarm 1 and clear flag if set
    { 
      if  (hour(t) >= Dawn && hour(t) <= Dusk) { //Only run camera for capture if current RTC time is between dawn and dusk
        Serial.println(F("Daytime alarm activated."));
        ACTION = 1; // Turns LED Flash off
        runCamera(); // DISABLING RUN CAMERA FOR DAYLIGHT HOURS
        alarmInt = alarmInterval;
      }
      else {
        // ********** Comment out the four lines below if you do not want dark action *********
        // Serial.println(F("Nighttime alarm activated."));
        // ACTION = 1; // Turns LED Flash on
        // runCamera();
        // alarmInt = darkAlarmInterval;
      }
      
      // Set alarm
      Serial.println(F("Setting new alarm."));
      alarmTime = t + alarmInt;  // Calculate next alarm
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, second(alarmTime), minute(alarmTime), hour(alarmTime), day(alarmTime));   // Set alarm
      Serial.println(F("Next alarm is at: "));
      Serial.print(year(alarmTime));
      Serial.print("/"); 
      Serial.print(month(alarmTime));
      Serial.print("/"); 
      Serial.print(day(alarmTime));
      Serial.print("  ");
      Serial.print(hour(alarmTime));
      Serial.print(":");
      Serial.print(minute(alarmTime));
      Serial.print(":");
      Serial.print(second(alarmTime));
      Serial.println("  ");

      // Check if alarm was set in the past
      if (RTC.get() >= alarmTime)
      {
        Serial.println(F("New alarm has already passed! Setting next one."));
        //t = RTC.get();                    // Read current date and time from RTC
        alarmTime = t + alarmInt;    // Calculate next alarm
        RTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, second(alarmTime), minute(alarmTime), hour(alarmTime), day(alarmTime));
        RTC.alarm(DS3232RTC::ALARM_1);               // Ensure alarm 1 interrupt flag is cleared
        Serial.println(F("Next alarm is at: "));
        Serial.print(year(alarmTime));
        Serial.print("/"); 
        Serial.print(month(alarmTime));
        Serial.print("/"); 
        Serial.print(day(alarmTime));
        Serial.print("  ");
        Serial.print(hour(alarmTime));
        Serial.print(":");
        Serial.print(minute(alarmTime));
        Serial.print(":");
        Serial.print(second(alarmTime));
        Serial.println("  ");
      }
    }
    
    alarmIsrWasCalled = false;  // Reset RTC ISR flag
    Serial.println(F("Alarm ISR set to False"));
    goToSleep();                // Sleep
  }
}

// Enable sleep and await RTC alarm interrupt
void goToSleep()
{
  Serial.println(F("Going to sleep..."));
  Serial.flush();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // Enter sleep and await external interrupt 
}

// Real-time clock alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmIsrWasCalled = true;
}


// Funtion to run the camera
void runCamera() {

    // Flash the LED to show cycle about to start
    digitalWrite(13, HIGH); 
    delay(500);
    digitalWrite(13, LOW);
    Serial.println(F("CoralCam awake. "));


    // Uncomment the lines below based on the action you want. 
    // Delay timing inside functions may need adjusting based on camera used.
    runCamera_Photo();
    // runCamera_Video(); // Adjust video length inside this function below

    // Flash the LED to show cycle has completed.
    digitalWrite(13, HIGH);  
    delay(500);
    digitalWrite(13, LOW);
    Serial.println(F("Capture finished."));

}

void runCamera_Photo() {

   // Press power button to turn camera on 
    Serial.println(F("Turning on camera")); 
    digitalWrite(PowerMode, HIGH);   // open power mosfet ("finger on button")
    delay(3000);                       // 3 second *button push* 
    digitalWrite(PowerMode, LOW);    // close power mosfet ("finger off button")
   
   // Wait 10 seconds to start because camera is slow to start up 
    Serial.println(F("Waiting 10 seconds for boot cycle")); 
    delay(10000); 

   // Press power (mode) button to change to photo mode
    Serial.println(F("Pressing mode button to change to photo mode"));
    digitalWrite(PowerMode, HIGH);   // // open power mosfet ("finger on button")
    delay(250);                       // 0.25 second *button push* - DO NOT CHANGE
    digitalWrite(PowerMode, LOW);    // close power mosfet ("finger off button")

   // Wait 3 secs because camera is slow
    Serial.println(F("Waiting 3 seconds")); 
    delay(3000);

   // Press shutter button to take photo 
    Serial.println(F("Taking photo")); 
    digitalWrite(Shutter, HIGH);   // open shutter mosfet ("finger on button")
    delay(250);                       // 0.25 second *button push* - DO NOT CHANGE
    digitalWrite(Shutter, LOW);    // close shutter mosfet ("finger off button")

    // Wait 8 seconds before powering off to give time to save file
    Serial.println(F("Waiting 8 seconds to save file"));
    delay(8000); 

   // Press power button to turn camera off
    Serial.println(F("Turning off camera"));
    digitalWrite(PowerMode, HIGH);   // open power mosfet ("finger on button")
    delay(3000);                       // 3 second *button push* DO NOT CHANGE
    digitalWrite(PowerMode, LOW);    // close power mosfet ("finger off button")

    delay(1000); // 1 second spacer delay. Adjust as needed

}

void runCamera_Video() {

   // Press power button to turn camera on 
    Serial.println(F("Turning on camera")); 
    digitalWrite(PowerMode, HIGH);   // open power mosfet ("finger on button")
    delay(3000);                       // 3 second *button push* 
    digitalWrite(PowerMode, LOW);    // close power mosfet ("finger off button")
   
   // Wait 10 seconds to start because camera is slow to start up 
    Serial.println(F("Waiting 10 seconds for boot cycle")); 
    delay(10000); 

   // Press shutter button to start video 
    Serial.println(F("Starting video")); 
    digitalWrite(Shutter, HIGH);   // open shutter mosfet ("finger on button")
    delay(250);                       // 0.25 second *button push* - DO NOT CHANGE
    digitalWrite(Shutter, LOW);    // close shutter mosfet ("finger off button")

    // Wait 60 seconds for video capture
    Serial.println(F("Waiting for 30 second video"));
    delay(30000); // This is how long your video is in milliseconds. Adjust as needed.
    
   // Press shutter button to wake camera (one press doesn't stop video if screen has gone dark)
   // If your video is longer than 30 seconds, uncomment the five lines below. 
    // Serial.println(F("Waking camera"));
    // digitalWrite(Shutter, HIGH);   // open shutter mosfet ("finger on button")
    // delay(250);                       // 0.25 second *button push* - DO NOT CHANGE
    // digitalWrite(Shutter, LOW);    // close shutter mosfet ("finger off button")
    // delay(3000); // wait

    // Press shutter button to stop video
    Serial.println(F("Stopping video"));
    digitalWrite(Shutter, HIGH);   // open shutter mosfet ("finger on button")
    delay(250);                       // 0.25 second *button push* - DO NOT CHANGE
    digitalWrite(Shutter, LOW);    // close shutter mosfet ("finger off button")

    // Wait 8 seconds before powering off to give time to save file
    Serial.println(F("Waiting 8 seconds to save file"));
    delay(8000); 

   // Press power button to turn camera off
    Serial.println(F("Turning off camera"));
    digitalWrite(PowerMode, HIGH);   // open power mosfet ("finger on button")
    delay(3000);                       // 3 second *button push* 
    digitalWrite(PowerMode, LOW);    // close power mosfet ("finger off button")

    delay(1000); // 1 second spacer delay. Adjust as needed
}




