/*
  AboutTimeServer

  This is a system to initialize and calibrate the ageing registers of a DS3231 RTC module using GPS as a reference.
  It also can be used as a time reference source via USB in conjunction with the PC application AboutTimeClient
  which is at https://github.com/RocketManRC/AboutTimeClient.

  This uses my u-blox-m8 library which is optimized for timing applications:

  https://github.com/RocketManRC/u-blox-m8 (automatically downloaded and installed by PlatformIO).

  This runs on the Teensy LC (and most likely Teensy 3.x or 4.x). The U/I is entirely
  via the USB Serial port.

  It requires a u-blox M8 GPS with PPS (optional) with the following connections:

  GPS tx  -> pin 0
  GPS rx  -> pin 1
  GPS PPS -> pin 2

  If you don't have a PPS connection then set the USEPPS define to 0. This will cause
  about +- 5ms of error in the initialzation of the DS3231 and and about the same amount 
  of jitter in the reference data coming over the USB serial port.

  The GPS is powered by 5V on pin Vin but the I/O is 3.3V

  Also a DS3231 module:

  DS3231 SQW -> pin 20
  DS3231 SCL -> pin 19
  DS3231 SDA -> pin 18

  The DS3231 is powered by 3.3V

  The commands on the serial terminal are:
    'i' - initialize the RTC. This waits for the GPS to have more than 6 satellites and 
          estimate time error of less than 100ns.
    'o' - makes the RTC 5 seconds later than GPS. This is for testing.
    'a' - read the ageing register (not really needed).
    '+' - add one to the ageing register, max is 127
    '-' - subtrack one from the ageing register. Max offset is 128 (starts at 255).

    The data output to the terminal is as follows:
      'rt'    ->  the RTC time and date at the last SQW interrupt.
      't'     ->  the GPS time at the last PPS interrupt.
      'dt'    ->  the difference in microseconds between the SQW interrupt and the PPS interrupt.
                  This is circular so varies between 0 and 999999.
      'dtt'   ->  the total difference in microseconds between SQW and PPS (+ or -).
      'dcy'   ->  this is from a free running 16bit counter on Teensy LC and is the difference
                  between SQW and PPS. It give a little bit better feedback on the drifting of the 
                  clock. The idea is to get the change in this to be as small as possible.
      'taac'  ->  this is the estimated time accuracy in nanoseconds from the GPS.
      'numsv' ->  the number of satellites used in the GPS solution. More than 6 is good.
*/

#define USEPPS 1        // if this is zero then PPS will be emulated by calling 
                        // the interrupt handler 960ms after the NAV-PVT message is received.
#define SERIALDEBUG 0   // setting this will send debug messages to the console


#include <Arduino.h>
#include "u-blox-m8.h"
#include "DS3232RTC.h"
#include "TeensyTimerTool.h"
#include <WS2812Serial.h>

const int numled = 2;
const int pin = 24;

#define GPSLED 0
#define DSLED 1

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);

/*
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
*/

// Less intense...

#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

int gpsColour = RED;
int dsColour = RED;

void colorWipe(int color, int wait) {
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait);
  }
}

using namespace TeensyTimerTool;

#define RTC_AGING 0x10 // this is the address of the aging register
const uint8_t RTC_1HZ_PIN( 20 );   // RTC provides a 1Hz interrupt signal on this pin
DS3232RTC RTC;
uint32_t isrTime = 0;
uint32_t rtcSetTime = 0;

const byte interruptPin = 2;              // Assign the interrupt pin for PPS

volatile uint32_t ppsCount = 0;					  // increment this for every PPS pulse
volatile uint32_t ppsMicros = micros();
//volatile uint32_t ppsCycles = ARM_DWT_CYCCNT;  // Teensy 3.x
volatile uint16_t ppsCycles = 0;  // Teensy LC
volatile uint32_t dsMicros = micros();
//volatile uint32_t dsCycles = ARM_DWT_CYCCNT;  // Teensy 3.x
volatile uint16_t dsCycles = 0;   // Teensy LC
uint32_t timerMicros = micros();
volatile uint8_t lastHr;
volatile uint8_t lastMn;
volatile uint8_t lastSc;
volatile uint8_t lastDay;
volatile uint8_t lastMonth;
volatile uint16_t lastYear;
volatile time_t isrUTC;         // ISR's copy of current time in UTC

// these are things we might display so keep them global
int satTypes[7];
double snr;
double sec;
int hr;
int mn;
double sc;
int numSV;
double pDOP;
uint32_t flags;
int tacc;
uint8_t gpsDay;
uint8_t gpsMonth;
uint16_t gpsYear;

Timer t1( TCK ); // TeensyTimerTool timer
Timer t2( TCK ); // This timer is to blink the DSLED (to give feedback on initialization procedure)

int touchValue = touchRead( 4 ); // from touch pin
int lastTouchValue = touchValue;
int touchCount = 0;

// TeensyTimerTool callback
void timerCallback()
{
    timerMicros = micros();

    digitalWriteFast( LED_BUILTIN, HIGH );  // switch off LED
    leds.setPixel( GPSLED, gpsColour );
    leds.show();
}
 
// TeensyTimerTool second timer
void timer2Callback()
{
    leds.setPixel( DSLED, 0 ); // Force the DS LED to blink
    leds.show();
}
 
// PPS - Digital Event Interrupt
// Enters on rising edge
//=======================================
void handleInterrupt()
{
  ppsMicros = micros();
  //ppsCycles = ARM_DWT_CYCCNT;  // Teensy 3.x
  ppsCycles = SYST_CVR; // Teensy LC

  // Save the last reported gps date and time (1 second behind time at PPS)
  lastHr = hr;
  lastMn = mn;
  lastSc = sc;
  lastDay = gpsDay;
  lastMonth = gpsMonth;
  lastYear = gpsYear;

#if !USEPPS
    timerCallback(); // need the blinking lights!
#endif 

  ppsCount++; //  this needs to be last
}

void setupPPS()
{
  pinMode(interruptPin, INPUT_PULLUP);                                            // sets pin high

  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING); // attaches pin to interrupt on Rising Edge
}

ublox gps;
navpvt8 nav( gps );
cfgtp5 tp( gps );
navsat ns( gps );
cfggnss gc( gps );

// Send a byte to the receiver
void sendByte(byte b)
{
  Serial2.write( b );
}

// Send the packet specified to the receiver
void sendPacket(byte *packet, byte len)
{
  for (byte i = 0; i < len; i++)
  {
      Serial2.write( packet[i] );
  }
}

// return current time
time_t getUTC()
{
    noInterrupts();
    time_t utc = isrUTC;
    interrupts();
    return utc;
}

// set the current time
void setUTC(time_t utc)
{
    noInterrupts();
    isrUTC = utc;
    interrupts();
}

// 1Hz RTC interrupt handler increments the current time
void incrementTime()
{
    ++isrUTC;
    dsMicros = micros();
    //dsCycles = ARM_DWT_CYCCNT;  // Teensy 3.x
    dsCycles = SYST_CVR;  // Teensy LC
}

// format and print a time_t value
void printTime(time_t t)
{
    char buf[25];
    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strcpy(m, monthShortStr(month(t)));
    sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d",
        hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t));
    Serial.println(buf);
}

void setupRTC()
{
    //Serial.println(F("\n" __FILE__ " " __DATE__ " " __TIME__));

    pinMode(RTC_1HZ_PIN, INPUT_PULLUP);     // enable pullup on interrupt pin (RTC SQW pin is open drain)
    attachInterrupt(digitalPinToInterrupt(RTC_1HZ_PIN), incrementTime, FALLING);

    RTC.squareWave(SQWAVE_1_HZ);            // 1 Hz square wave

    time_t utc = getUTC();                  // synchronize with RTC
    while ( utc == getUTC() );              // wait for increment to the next second
    utc = RTC.get();                        // get the time from the RTC
    setUTC(utc);                            // set our time to the RTC's time
    //Serial.println("Time set from RTC");
}

void setup()
{
  ARM_DEMCR |= ARM_DEMCR_TRCENA;  // start the Teensy cycle counter (does not work on Teensy LC!)
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  
  leds.begin();
  leds.setPixel( 0, gpsColour );
  leds.show();
  leds.setPixel( 1, dsColour );
  leds.show();

  Serial.begin( 115200 );

  Serial2.setRX( 9 );
  Serial2.setTX( 10 );
  Serial2.begin( 9600); 

  delay( 4000 );
  changeBaudrate( 115200 );  // change to 115200

  delay( 100 );
  Serial2.begin( 115200 );

  delay( 100 );
  Serial2.flush();

  delay( 100 );
  disableNmea();

  delay( 100 );
  enableNavPvt();

  delay( 100 );

#if SERIALDEBUG
  Serial.println( "u-blox initialized" );
#endif

#if USEPPS
  setupPPS();
  t1.beginOneShot( timerCallback ); 
#else
  // If we don't have PPS we will emulate it with a oneshot timer of 960 ms
  t1.beginOneShot( handleInterrupt ); 
#endif
  t2.beginOneShot( timer2Callback ); 

  pinMode( LED_BUILTIN,OUTPUT );   

  setupRTC();
}

// If called after PPS this will be the time (corrected by 1 second from previous GPS report)
time_t gpsTimeAtPPS()
{
  time_t t;
  tmElements_t tm;

  noInterrupts();
  tm.Year = CalendarYrToTm( lastYear );
  tm.Month = lastMonth;
  tm.Day = lastDay;
  tm.Hour = lastHr;
  tm.Minute = lastMn;
  tm.Second = lastSc;
  interrupts();

  t = makeTime(tm);

  t += 1;

  return t;
}

time_t gpsTimeNow()
{
  time_t t;
  tmElements_t tm;

  tm.Year = CalendarYrToTm( gpsYear );
  tm.Month = gpsMonth;
  tm.Day = gpsDay;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;

  t = makeTime(tm);

  return t;
}

void loop()
{
  static uint32_t ppsc = 0;
  static bool initRTC = false;
  static bool gotGPStime = false;
  static uint32_t isrc = 0;
  static uint32_t ppsm = ppsMicros;
  static uint32_t dsm = dsMicros;

  noInterrupts();
  ppsc = ppsCount;
  isrc = isrUTC;
  ppsm = ppsMicros;
  dsm = dsMicros;
  interrupts();  
  
  static uint32_t lastPpsCount = ppsc;
  static uint32_t lastIsrUTC = isrc;
  static time_t rt = 0;
  static bool addOffset = false;
  static byte ageing = 0;

  // handle the touch input here
  touchValue = touchRead( 4 );

  if( touchValue > 2000 && lastTouchValue < 1500 )
  {
    lastTouchValue = touchValue;
    touchCount++;

    if( touchCount >=  3 )
    {
      lastPpsCount = ppsc; // We want to wait for the PPS to occur
      initRTC = true;
      rt = 0; // read from RTC again
      
      touchCount = 0;
    }
  }
  else if( touchValue < 1500 and lastTouchValue > 2000 )
    lastTouchValue = touchValue;
  

  // The command handler for the serial port is here
  if( Serial && Serial.available() )
  {
    char c = Serial.read();
    Serial.println( c );

    if( c == 'i' || c == 'o' ) // Should we init the RTC?
    {
      lastPpsCount = ppsc; // We want to wait for the PPS to occur
      initRTC = true;
      rt = 0; // read from RTC again

      if( c == 'o' )
        addOffset = true;
      else 
        addOffset = false;

      Serial.println( "Setting RTC..." );
    }

    if( c == 'a' || c == '+' || c == '-' )
    {
      ageing = RTC.readRTC( RTC_AGING );

      if( c == '+' )
      {
        if( ageing != 127 ) // 127 is the max
          RTC.writeRTC( RTC_AGING, ++ageing );
      }

      if( c == '-' )
      {
        if( ageing != 128 ) // 128 is the min
          RTC.writeRTC( RTC_AGING, --ageing );
      }      
    }
  }

  // If were are going to intialize the RTC we wait for a PPS pulse to occur 
  // (ppsc incremented in the interrupt handler)
  if( initRTC )
  {
    if( ppsc != lastPpsCount )
    {
      lastPpsCount = ppsc;

      // The arbitrary conditions are number of satellites > 6 and estimated 
      // time accurcy less than 100ns. This can be changed here in the if() statment.
      if( gotGPStime && (numSV > 6) && (tacc < 100) )
      {
        isrTime = micros() - ppsm;
        initRTC = false;
        
        time_t t;

        t = gpsTimeAtPPS();

        if( addOffset )
        {
          delay( 5000 );

          RTC.set( t + 1 );   

          addOffset = false;     
        }
        else 
          RTC.set( t );

        rtcSetTime = micros() - ppsm;
        setTime(t);
      }
      else
      {
        Serial.println( "Waiting for valid GPS time..." );
      }      
    }
  } 
  else if( isrc != lastIsrUTC )
  {
    // This is the monitoring code. It runs every time the SQW interrupt handler
    // increments the counter isrc,

    lastIsrUTC = isrc;

    if( rt == 0 )
    {
      rt = RTC.get(); // only going to read the RTC once at the start and if something changes
      ageing = RTC.readRTC( RTC_AGING );  // same with the ageing register
    } 
    else
      rt++;
   
    // NOTE that the time at PPS is one second more than the last reported time.
    time_t t = gpsTimeAtPPS();

    uint32_t dt = dsm - ppsm;
    int32_t dtt = (t - rt) * 1000000 + dt;

    if( gpsColour != GREEN )
    {
      dsColour = 0;
      touchCount = 0; // if GPS gone stop the initialization procedure
    }
    else if( abs(dtt) < 10000 )
      dsColour = GREEN;
    else if( abs(dtt) >= 10000 && abs(dtt)  < 50000 )
      dsColour = BLUE;
    else if( abs(dtt) >= 50000 && abs(dtt) < 200000 )
      dsColour = ORANGE;
    else
      dsColour = RED;

    leds.setPixel( DSLED, dsColour );
    leds.show();

    if( touchCount > 0 )
    {
      t2.trigger( 50'000 );
    }
    

    if( Serial )
    {
      Serial.println();
      Serial.print( "SQW " );
      Serial.println( rt );
      //printTime( rt );  
      //printTime( t );

      Serial.print( "tacc: " );
      Serial.println( nav.gettacc() );
      Serial.print( "numsv: " );
      Serial.println( nav.getnumSV() );

      Serial.print( "dt: " );
      Serial.println( dt );

      Serial.print( "dtt: " );
      Serial.println( dtt );

      Serial.print( "dcy: " );
      Serial.println( dsCycles - ppsCycles );

      Serial.print( "ageing: " );
      Serial.println( ageing );

      uint32_t dttt = timerMicros - ppsm;
      Serial.println( dttt );

      Serial.println( touchValue );
    }
  }

  // Here we handle the GPS

  // The following flags are for polling because it seems that a few
  // requests might be needed to get a response. So poll until we get one!

  static bool waitForCfgtp5 = true; // for config of the time pulse
  static bool waitForCfgGnss = true;// and the GNSS configuration (to disable SBAS)

  while( Serial2.available() )
  {
    uint8_t c = Serial2.read();

    char *r = (char *)gps.parse( c );

    if( strlen( r ) > 0 )
    {
      if( strcmp( r, "navpvt8" ) == 0 )
      {
        if( waitForCfgtp5 )
        {
          pollTimePulseParameters();  // ask for the current time pulse parameters then we will set them
        }
        else if( waitForCfgGnss )
        {
          gc.pollCfggnss();           // same for the gnss configuration
        }

        sec = 3600.0 * nav.gethour() + 60.0 * nav.getminute() + 1.0 * nav.getsecond() + nav.getnano() * 1e-9;
        hr = sec / 3600;
        mn = (sec - hr * 3600) / 60;
        sc = nav.getsecond();
        gpsDay = nav.getday();
        gpsMonth = nav.getmonth();
        gpsYear = nav.getyear();

        numSV = nav.getnumSV();
        pDOP = nav.getpDOP();
        flags = nav.getflags();
        tacc =  nav.gettacc();

        gotGPStime = true;

        if( Serial )
          digitalWriteFast( LED_BUILTIN, LOW ); // LED on
           
        t1.trigger( 960'000 ); // trigger the callback in 960 ms (can emulate PPS if there is no connection)

        leds.setPixel( GPSLED, 0 );
        leds.show();

        if( numSV == 0 )
          gpsColour = RED;
        else if( numSV > 0 && numSV < 7 )
          gpsColour = ORANGE;
        else if( numSV > 7  && tacc >= 100 )
          gpsColour = BLUE;
        else
          gpsColour = GREEN;

#if SERIALDEBUG
        Serial.println( nav.getnumSV() );
        Serial.print( nav.getlat(), 5 );
        Serial.print( " ");
        Serial.print( nav.getlon(), 5 );
        Serial.print( " ");
        Serial.println( nav.getheight(), 2 );
        Serial.print( nav.getpDOP(), 2 );
        Serial.print( " ");
        Serial.print( nav.gethAcc() );
        Serial.print( " ");
        Serial.println( nav.getvAcc() );
        Serial.print( nav.getnano() );
        Serial.print( " ");
        Serial.println( nav.gettacc() );
        Serial.println( nav.getflags(), 16 );
#endif
      }
      else if( strcmp( r, "cfgtp5" ) == 0 )
      {
#if SERIALDEBUG
        Serial.print( tp.getAntCableDelay() );
        Serial.print( " ");
        Serial.println( tp.getRfGroupDelay() );

        Serial.print( tp.getFreqPeriod() );
        Serial.print( " ");
        Serial.println( tp.getFreqPeriodLock() );

        Serial.print( tp.getPulseLenRatio() );
        Serial.print( " ");
        Serial.println( tp.getPulseLenRatioLock() );

        Serial.print( tp.getUserConfigDelay() );
        Serial.print( " ");
        Serial.println( tp.getFlags(), 16 );

        Serial.println( "Configure time pulse parameters" );
#endif
        //Serial.println( "Configure time pulse parameters" );
        // Here we set our time pulse parameters to a one Hz square wave until we have a fix.
        tp.setPulseLenRatio( 500000 );
        tp.configureTimePulse();

        waitForCfgtp5 = false; // only need to do it once
      }
      else if( strcmp( r, "navsat" ) == 0 )
      {
        int numsvs = ns.getnumSvs();
#if SERIALDEBUG
        Serial.print( "Num SVs: ");
        Serial.println( numsvs );
#endif
        for( int i = 0; i < 7; i++ )
          satTypes[i] = 0;

        snr = 0.0; // average snr
        int c = 0;

        for( int i = 0; i < numsvs; i++ )
        {
          int flags = (int)ns.getflags( i );

          if( flags & 8 )
          {
            c++;
            snr += 1.0 * ns.getcno( i );

            int gnssId = (int)ns.getgnssId( i );
            if( gnssId < 7 && gnssId >= 0 )
              satTypes[gnssId]++;

#if SERIALDEBUG
            Serial.print( gnssId );
            Serial.print( " " );
            Serial.print( (int)ns.getsvId( i ) );
            Serial.print( " " );
            Serial.print( (int)ns.getcno( i ) );
            Serial.print( " " );
            Serial.print( flags, 16 );
            Serial.print( "   " );
#endif
          }
        }

        if( c > 0 )
          snr = snr / c;

#if SERIALDEBUG
        Serial.println( snr );
#endif
      }
      else if( strcmp( r, "cfggnss" ) == 0 )
      {
        int numblocks = gc.getnumConfigBlocks();

        for( int i = 0; i < numblocks; i++ )
        {
          int gnssId = (int)gc.getgnssId(i);

          if( gnssId == 1 )
            gc.setCfggnss( 1, false );  // Disable SBAS - recommende by u-blox for timing applications
        }

        waitForCfgGnss = false;
      }
    }
  }
}
