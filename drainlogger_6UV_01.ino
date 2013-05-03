/*
drainlogger.ino is the program operating a capacitive water level sensing
hardware design developed at the Sustainability Institute at the University 
of East London. 

Copyright Toby Borland 2012-2013

This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Arduino drainlogger 0.0.6UV - V-notch/Uno variant
// Sustainability Research Institute, University of East London
// Toby Borland with contribution from Ertion Axha, Gertz, DeJusto
// tobyborland@hotmail.com
// XINO SERIAL 9600 BAUD OVER DUEMILANOVE BOARD OK
// Xino => set board type to Arduino Uno
#define SENSOR_IDENTIFIER "Unit:            0000"

// Hazen-Williams friction head loss empirical formula: f = (10.67*(100/c)^1.85*q^1.85)*D_h^-4.87 (SI units)
// f = friction head, c = Hazen-Williams roughness constant, q = volume flow, D_h = inside hydraulic diameter
// pipe of roughness factor 150 and 5m vertical drop
// 160mm dia pipe => 0.337 m^3/s discharge rate & velocity of 16.7 m/s
// 125mm dia pipe => 0.176 m^3/s discharge rate & 14.3 m/s velocity

#include <avr/wdt.h> // AVR watchdog timer
#include <avr/sleep.h>

#include <I2cMaster.h>
#include <SoftRTClib.h>

#include <SdFat.h>
#include <SdFatUtil.h> // use Serial.print

// Olimex-32U4 Leonardo architecture issues:
// Serial.read requires XP SP3 updates (Jan2012)

// use Software SPI edit SdFatConfig.h at about line 121 of Dec2012 SdFat
// #define LEONARDO_SOFT_SPI 1

#if defined(__AVR_ATmega32U4__)
#if !LEONARDO_SOFT_SPI
#error set LEONARDO_SOFT_SPI nonzero in libraries/SdFat/SdFatConfig.h
#endif  // LEONARDO_SOFT_SPI
// Is a Leonardo use analog pins 4, 5 for software I2C
const uint8_t RTC_SCL_PIN = 23;
const uint8_t RTC_SDA_PIN = 22;
SoftI2cMaster i2c(RTC_SDA_PIN, RTC_SCL_PIN);

#else  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// Not Mega use hardware I2C
// enable pull-ups on SDA/SCL
TwiMaster i2c(true);
#endif  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// RTC pins commandeered on ladyada logshield
// RTC_SCL - A4
// RTC_SDA - A5

// SPI pins commandeered
// SS – D10
// MOSI – D11
// MISO – D12
// SCK – D13

// ATmega48A/PA/88A/PA/168A/PA/328/P [DATASHEET] p.139
// 16 bit hardware timer interrupt T1 - D5
// NOTE pin not initialised

#define FILE_MAX_LOG  336 // max# entries per file  - tune this to SD cluster/block size of 8KB
// default cluster sizes for FAT32 under >Windows98
// 256 MB–8GB	4 KB	4 KB	4 KB
// 8GB–16GB 	8 KB	8 KB	8 KB
// 16GB–32GB 	16 KB	16 KB	16 KB
// tradeoff between data security and SD storage efficiency: assume/dictate 8KB cluster size => < 8KB file size = 340 lines of 23 char including \n.
#define FILE_CHAR_COUNT  8156  // for 8KB 23char/line 336 entry file

#define ECHO_TO_SERIAL 0  // echo data to serial port
#define SERIAL_BAUD 9600 // defined in several places
#define DEBUG 0
#define TIME_DBG 0
#define LED_ON 0 // RX, TX shared with LEDs, disable LED_ON if ECHO_TO_SERIAL
#define WDT_RESET_LIMIT 8000 // 8 second watchdog timer system reset interval (max), adjusted by order of hibernate() or initWDT() call.
#define RESOLUTION 2 // seconds pause between readings
#define SLEEP_ON 1 // also used in temperature calibration delay
#define SD_LOG 1  // 
#define SD_CHECK 0 // free clusters, time consuming
#define RTC_SYNC 0  // query PC time to reset RTC

// SRI Cap Sense Shield 1.01 pinout & Xino v1.5 revised power
const byte redLEDpin = 0; //D3 protoboard value, D2 Olimex original, D0 Xino (to prevent conflict w/ upload programming)
const byte greenLEDpin = 1; //D4 protoboard value, interferes with pulse sense pin on Cap_Sense_Shield 1.01 (shift to D3?)
const byte gateMOSFET = 2; // switch power to peripherals for low power sleep
const byte pulseSensePin = 4; //D5 on protoboard
const byte CD4060ResetPin = 7; // not present on protoboard, interferes w/ LM555 reset, D7 on Cap_Sense_Shield 1.01D7
const byte LM555ResetPin = 8; //D7 on protoboard, D8 on Cap_Sense_Shield 1.01 (OLIMEXINO LED)
const byte batteryTestEnablePin = 9; //D9 (OLIMEXINO LED)
const byte batteryVoltagePin = 14; //A0/D14
const byte TMP36pin = 15; //Analog 0 on protoboard, A1/D15 on Cap_Sense_Shield 1.01
const byte Conduct_BsensePin = 16; //A2/D16
const byte Conduct_AdrivePin = 17; //A3/D17
const byte chipSelect = 10;

// constants for conductivity testing
const float refVoltage = 5.00; // CHANGE THIS FOR 3.3v Arduinos
const float aResn = refVoltage / 1024;
const float res10K = 9850.0;

const byte WINDOW = 25; // rectangular smoothing window width, must be > 2
// eyeball smoothing dl_58LVt_window & graphing.pde => 25 optimum

unsigned int shiftIndex = WINDOW-2;
unsigned long avg_pulse[WINDOW]; // be warned of rollover issues with long delays (e.g. CD4060 divider)
unsigned long pulse, raw_pulse, ref_pulse, max_pulse, min_pulse;
volatile unsigned long WDT_catch; // always less than 8000, but requires millis() => UL
#if DEBUG
  volatile unsigned long ongoing = 0; // WDT interrupt counter, only used for DEBUG
#endif
#if TIME_DBG
  volatile unsigned long timeProfile = 0;//millis();
#endif

int signal_check;
byte i;

unsigned int JITTER = 2;//___________________________FINE_TUNE
// unsuited as an exit condition for temperature calibration?

unsigned int NO_WATER = 60; // if the meter dries out, there is stray capacitance_________CHECK VALUE
byte no_flow = 0; // global variable set if activity and pulse width below threshold -> hibernate
unsigned long min_loop = 0; // wraparound issues if set to byte
unsigned long still_no_flow = 0;

RTC_DS1307 RTC(&i2c); // define Real Time Clock object
// DS1307RTC chip battery fabled to last 5 years (www.ladyada.net)
DateTime now;

// create file object
SdFat sd;
SdFile logfile;
SdFile readfile;
Sd2Card card; // card insertion & space checking services
SdVolume vol;
cid_t cid;
unsigned long SD_serial;

// store error strings in flash to save RAM <SdFatUtil.h>
#define error(s) sd.errorHalt_P(PSTR(s))
//#define errorHalt(s) error_P(PSTR(s))

ISR (WDT_vect) { // watchdog interrupt
  // anything persistent - note volatile variable requirement
  #if LED_ON
    if (digitalRead(greenLEDpin) == HIGH) {
      digitalWrite(greenLEDpin, LOW);
    }else{
      digitalWrite(greenLEDpin, HIGH);
    } // toggle, delay doesn't work
  #endif
  #if DEBUG
    Serial.print(F("DEBUG ISR() ongoing: "));Serial.println(ongoing);
    ++ongoing;
  #endif
}
 
float getPowerVoltage(void){
  // use spare channel of op-amp to measure battery voltage input
  // data used to determine success of remote power supplies

  // switch voltage divider circuit power on 
  digitalWrite(batteryTestEnablePin, HIGH);
  delay(1000); //allow steady state level
  float V = analogRead(batteryVoltagePin);

  V = (V + 7.72)/34.1; 
  digitalWrite(batteryTestEnablePin, LOW);
  #if DEBUG
    Serial.print(F("DEBUG getPowerVoltage() V: ")); Serial.println(V);
  #endif
  return V;
}

float getTemp() {
  int reading = analogRead(TMP36pin);
  float voltage = reading * 5.0;
  voltage /= 1024.0; 
  float tempC = (voltage - 0.5) * 100 ;
  #if DEBUG
    Serial.print(F("DEBUG getTemp() tempC: ")); Serial.println(tempC);
    //Serial.print(F("DEBUG getTemp() reading: ")); Serial.println(reading);
  #endif
  return tempC;
}  // getTemp()

float getSiemens(void){
  // Environmental Monitoring with Arduino, E. Gertz & P. De Justo, O'Reilly, p.35
  // measure electrical conductivity of water.
  //int threshold = 3;
  
  // rainwater test: 160 mS
  // deionised water 78 mS
  // 100K resistor 2500 mS
  // 10.26 mm seperation between contacts.
  
  int analogV = 0;
  int ref_AnalogV = 1000;
  float V = 0.0;
  float R = 0.0;
  double Siemens;

  while((ref_AnalogV - analogV) > 3){
    ref_AnalogV = analogV;
    digitalWrite( Conduct_AdrivePin, HIGH );
    delay(10); // allow ringing to stop
    analogV = analogRead(Conduct_BsensePin);
    digitalWrite( Conduct_AdrivePin, LOW );
  }

  V = (analogV * aResn);
  R = ((5.00 * res10K) / V) - res10K;
  Siemens = 1.0/(R/1000000);

  #if DEBUG
    if (V > 4.9) Serial.println(F("DEBUG conductivity probe short?"));//____________________________recalibration fn.
    Serial.print(F("DEBUG getSiemens() V: ")); Serial.println(V);
    Serial.print(F("DEBUG getSiemens() Siemens: ")); Serial.println(Siemens);
  #endif

  return Siemens;
} //getSiemens()

#if RTC_SYNC
  void serialSynch(void){
    DateTime dt;
    if(Serial.available()){
      time_t t = processSyncMessage(); // DS1307RTC.h value
      if(t >0){
        dt.settime(t);
        RTC.setTime(&dt);   // set the RTC and the system time to the received value
        //setTime(now);          
        #if DEBUG
          Serial.print(F("DEBUG serialSynch() t: ")); Serial.println(t);
        #endif
      }
    }
  }  //serialSynch()
  
  time_t processSyncMessage(void){
    // return the time if a valid sync message is received on the serial port.
    while(Serial.available() >=  TIME_MSG_LEN ){  // time message consists of a header and ten ascii digits
      char c = Serial.read() ; 
      //Serial.print(c);  
      if( c == TIME_HEADER ){       
        time_t pctime = 0;
        for( i = 0; i < TIME_MSG_LEN -1; i++){   
          c = Serial.read();          
          if( c >= '0' && c <= '9'){   
            pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
          }
        }// Posix seconds since 1/1/1970 00:00:00 epoch
        #if DEBUG
          Serial.print(F("DEBUG processSyncMessage() pctime: ")); Serial.println(pctime);
        #endif
        return pctime; 
      }  
    }
    return 0;
  }  //processSyncMessage()
  
  time_t requestSync(void) {
    Serial.write(TIME_REQUEST); 
    #if DEBUG
      Serial.print(F("DEBUG1 requestSync() Serial.peek(): ")); Serial.println(Serial.peek());
    #endif
      delay(3000); // ~2120 mS required to put TIME_REQUEST in Serial buffer @ 57600
    #if DEBUG
      Serial.print(F("DEBUG2 requestSync() Serial.peek(): ")); Serial.println(Serial.peek());
    #endif
    return 0; // the time will be sent later in response to serial mesg
  }  //requestSync()
#endif 

char *formatCSV(float LPS, DateTime dt){
  // Get litres/Sec value and return CSV formatted string with timestamp
  // CVS format: UTC ISO8601 yyyy-mm-ddThh:mm:ss or yyyymmddThhmmss, rate <newline>
  // buffer = year(t) + month(t) + day(t) + "T" + hour(t) + minute(t) + second(t) + ",";
  char *CSV = "00000000T000000,000.00"; // 23 inc null
  char LPS_buffer[] = "NOTSET";

  unsigned int x;
  //uint8_t i; // global
  char A_num;
  
  #if TIME_DBG
    timeProfile = millis() - timeProfile;
    if (timeProfile >= 8000){
      Serial.print(F("TIMING formatCSV() A: ")); Serial.println(timeProfile);
    }
  #endif


  x = dt.year();
  A_num  = char((x / 1000) + 48); CSV[0] = A_num;
  A_num  = char(((x % 1000) / 100) + 48); CSV[1] = A_num;
  A_num  = char(((x % 100) / 10) + 48); CSV[2] = A_num;
  A_num  = char((x % 10) + 48); CSV[3] = A_num;

  x = dt.month();
  A_num  = char((x / 10) + 48); CSV[4] = A_num;
  A_num  = char((x % 10) + 48); CSV[5] = A_num;

  x = dt.day();
  A_num  = char((x / 10) + 48); CSV[6] = A_num;
  A_num  = char((x % 10) + 48); CSV[7] = A_num;

  x = dt.hour();
  A_num  = char((x / 10) + 48); CSV[9] = A_num;
  A_num  = char((x % 10) + 48); CSV[10] = A_num;

  x = dt.minute();
  A_num  = char((x / 10) + 48); CSV[11] = A_num;
  A_num  = char((x % 10) + 48); CSV[12] = A_num;

  x = dt.second();
  A_num  = char((x / 10) + 48); CSV[13] = A_num;
  A_num  = char((x % 10) + 48); CSV[14] = A_num;

  if (LPS > 999.99){

    #if ECHO_TO_SERIAL
      Serial.println(F("Litres per second value out of range"));
    #endif
    LPS_buffer[0] = ' ';
    LPS_buffer[1] = '>';
    LPS_buffer[2] = 'M';
    LPS_buffer[3] = 'A';
    LPS_buffer[4] = 'X';
    LPS_buffer[5] = ' ';

  }
  else{
    // dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf) 
    // avr-libc alternative to non-functioning String(float)

  #if TIME_DBG
    timeProfile = millis() - timeProfile;
    if (timeProfile >= 8000){
      Serial.print(F("TIMING formatCSV() B: ")); Serial.println(timeProfile);
    }
  #endif

    dtostrf(LPS, 6, 2, LPS_buffer);
    if (LPS < 100){
      LPS_buffer[0] = '0';
    }
    if (LPS < 10){
      LPS_buffer[1] = '0';
    }

  }
  for (i = 0; i < 6; i++) {
    CSV[i+16] = LPS_buffer[i];
  }  
  
  #if DEBUG
    Serial.print(F("DEBUG formatCSV() CSV: ")); Serial.println(CSV);
  #endif
  
  #if TIME_DBG
    timeProfile = millis() - timeProfile;
    if (timeProfile >= 8000){
      Serial.print(F("TIMING formatCSV() C: ")); Serial.println(timeProfile);
    }
  #endif


  return CSV;
}  //formatCSV()

void dateTime(uint16_t* date, uint16_t* time) { //__________________possible naming overload
  // date time callback function.
  // See SdFile::dateTimeCallback() for usage.
  // return date, time using FAT_DATE macro to format fields
  DateTime now = RTC.now();
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}  //dateTime()

void initialiseSDcard(void) {
  #if DEBUG  
    Serial.print(F("DEBUG initializing SD card --> "));
  #endif
  
  pinMode(10, OUTPUT); // chipSelect
  wdt_reset(); // possibly lengthy procedure

  #if SD_CHECK
    // check for SD exist
    #if DEBUG
      Serial.print(F("DEBUG card.init(SPI_HALF_SPEED.. \n"));
    #endif

    if (!card.init(SPI_HALF_SPEED, chipSelect)) {error("\ncard.init failed");}
    if (!card.readCID(&cid)){error("readCID failed");}
    SD_serial = cid.psn;
    Serial.print(F("\nSD card serial number: ")); Serial.println(SD_serial);

    // check for SD card space
    //wdt_reset(); // lengthy procedure
    if (!vol.init(&card)) {error("\nvol.init failed");}
    Serial.print(F("SD card clusterCount: ")); Serial.println(vol.clusterCount());
    Serial.print(F("SD card freeClusters: ")); Serial.println(vol.freeClusterCount());
  #endif

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  //if (!sd.init(SPI_HALF_SPEED)) sd.initErrorHalt();
    // initialize the SD card
  if (!sd.begin(chipSelect,SPI_FULL_SPEED)) sd.initErrorHalt(); //______________________L
  //if (!sd.init(SPI_FULL_SPEED)) sd.initErrorHalt();
  
  #if DEBUG  
    Serial.println(F("DEBUG SD card initialized."));
  #endif
}  //initialiseSDcard()

char* logFileName(DateTime dt, unsigned int inc) {
  // inc: file increment on the day
  // buffer = file# + year(t) + month(t) + day(t)
  // format = "AAAymmdd.CSV" where "AAA" = base 26 alphabetical file ordering
  // => file time period > 20sec

  char *fname = "AAA00000.CSV"; // 13 inc null
  char A_num;
  unsigned int x;
  //uint8_t i;  //global

  inc = inc % 17576;

  A_num = char((inc / 676) + 65); fname[0] = A_num;
  A_num = char(((inc % 676) / 26) + 65); fname[1] = A_num;
  A_num = char((inc % 26) + 65); fname[2] = A_num;

  x = dt.year();
  //A_num  = char(((x % 100) / 10) + 48); fname[2] = A_num;
  A_num  = char((x % 10) + 48); fname[3] = A_num;

  x = dt.month();
  A_num  = char((x / 10) + 48); fname[4] = A_num;
  A_num  = char((x % 10) + 48); fname[5] = A_num;

  x = dt.day();
  A_num  = char((x / 10) + 48); fname[6] = A_num;
  A_num  = char((x % 10) + 48); fname[7] = A_num;

  return fname;
}  //logFileName()

char* uniqueLogFileName(void) {
  // create a new unique filename
  int file_inc = 0;
  char* fname; 
  DateTime dt = RTC.now();

  fname = logFileName(dt, file_inc);
  //Serial.print(" fname ->"); Serial.println(fname); 
  while (sd.exists(fname)) {
    ++file_inc;
    if (file_inc > 1756) { //26^3
      error("filename permutations exceeded");
    }
    fname = logFileName(dt, file_inc);
  }
  //Serial.print(" fname1 ->"); Serial.println(fname); 
  return fname;
}  //uniqueLogFileName()

SdFile createLogFile(char* fname) {
  // create a new file
  SdFile newfile;
  //char *f_tbc; 
  DateTime dt = RTC.now();

  // create a new file with default timestamps
  //if (!newfile.open(fname, O_CREAT | O_WRITE)) {
  if (!newfile.open(fname, O_CREAT | O_APPEND | O_WRITE)) {error("open failed");}
  // set creation date time
  if (!newfile.timestamp(T_CREATE, dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second())) {
    error("set create time failed");
  }
  return newfile;
}  //createLogFile()

char *formatTBC(float temp, float batt, float cond){
  // Return CSV formatted string for header or calibration tests, 
  // terminate with unique char (:) to allow non-interleaving spreadsheet analysis
  // CVS format: UTC ISO8601 yyyy-mm-ddThh:mm:ss or yyyymmddThhmmss, rate <newline>
  // buffer = year(t) + month(t) + day(t) + "T" + hour(t) + minute(t) + second(t) + ",";
  char *TBC = "+00.00,00.00,000.0"; // 19 inc null
  char TBC_buffer[] = "NOTSET";
  
  // out of range buffer overrun protection - chance of unimplemented hardware
  if (temp > 99.99) {temp = 99.99;}
  if (batt > 99.99) {batt = 99.99;}
  if (cond > 999.9) {cond = 999.9;}
  if (temp < -99.99) {temp = -99.99;}
  if (batt < 0.0) {batt = 0.0;}
  if (cond > 999.9) {cond = 999.9;}

  // dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf) 
  // avr-libc alternative to non-functioning String(float)
  dtostrf(temp, 6, 2, TBC_buffer);
   for (i = 0; i < 6; i++) {
    TBC[i] = TBC_buffer[i];
  }  
  dtostrf(batt, 5, 2, TBC_buffer);
   for (i = 0; i < 5; i++) {
    TBC[i+7] = TBC_buffer[i];
  }  
  dtostrf(cond, 5, 1, TBC_buffer);
   for (i = 0; i < 5; i++) {
    TBC[i+13] = TBC_buffer[i];
  }  
  
  #if DEBUG
    Serial.print(F("DEBUG formatTBC() TBC: ")); Serial.println(TBC);
  #endif

  return TBC;
}  //formatTBC

boolean fileCheck(char* f_name){
  // test file for integrity + fixed size
  int data = 0;
  int charCount = 0;
  
  wdt_reset(); 
  if (!readfile.open(f_name, O_READ)) {sd.errorHalt("opening file for read failed");}
  #if ECHO_TO_SERIAL
    Serial.print(f_name); Serial.print(F(" :read test\n"));
  #endif
  // read from the file until there's nothing else in it:
  while ((data = readfile.read()) > 0) {
    #if ECHO_TO_SERIAL
      //Serial.write(data);//__________________________________________Olimexino debugging
    #endif
    ++charCount;
  }
 
  readfile.close();
  #if DEBUG
    Serial.print(F("DEBUG fileCheck() charCount: ")); Serial.println(charCount); // 8156 for 8KB 23char/line 336 entry file
  #endif
  if (charCount == FILE_CHAR_COUNT){return 1;}else{return 0;}
}  //fileCheck()

float mS2LpS(unsigned long rawpulse) {
  // conversion of milliSeconds to a Litre per second value
  float lS = 0.0;
  float pulse = 0.0;
  // perform all calc in mL, convert to float at fn. return, o/p format to 000.00L, 
  // V_notch_calibration gives piecewise approximation to calibration of experimental volumetric analysis 08DEC2012
  
  // temperature correction from 21Dec13
  pulse = float(rawpulse); - (0.461*getTemp()) - 10;

  // data from "121201_0206.txt" -> mL
  //  !pulse min  pulse max  constant    multiplier  !
  //  !2198.7801  2876.6206  -2103.4159  1.2449243   !
  //  !1500.1803  2198.7801  -799.1484   0.6501883   !
  //  !1049.9741  1500.1803  -347.78093  0.3518328   !
  //  !799.99747  1049.9741  -53.076802  0.0741316   !
  //  !712.16909  799.99747  -35.782427  0.0508238   !
  
  // data from "130126_2234.txt" -> mL
  //!pulse min  pulse max  constant    multiplier  !
  //!1950.3748  2553.5931  -1777.735   1.3277528   !
  //!1650.497   1950.3748  -888.12484  0.8758543   !
  //!1250.1619  1650.497   -193.14331  0.4534790   !
  //!649.92724  1250.1619  -325.8315   0.5628297   !
  //!170.09277  649.92724  -12.200581  0.0667296   !
  
  #if DEBUG
    Serial.print(F("DEBUG mS2LpS() pulse: ")); Serial.println(pulse);
  #endif

  if (pulse > 1950) {
    lS = -1777.7 + (1.3277528*pulse);
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() lps>1950: ")); Serial.println(lS/1000);
    #endif
  }
  else if ((pulse > 1650)&&(pulse <= 1950)) {
    lS = -888.1 + (0.8758543*pulse);
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() 1650<lps<1950: ")); Serial.println(lS/1000);
    #endif
  }
  else if ((pulse > 1250)&&(pulse <= 1650)) {
    lS = -193.1 + (0.4534790*pulse);
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() 1250<lps<1650: ")); Serial.println(lS/1000);
    #endif
  }
  else if ((pulse > 650)&&(pulse <= 1250)) {
    lS = -325.8 + (0.5628297*pulse);
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() 650<lps<1250: ")); Serial.println(lS/1000);
    #endif
  }
  else if ((pulse > 170)&&(pulse <= 650)) {
    lS = -12.2 + (0.0667296*pulse);
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() 170<lps<650: ")); Serial.println(lS/1000);
    #endif
  }
  else {
    lS = 0.0;
    #if DEBUG
      Serial.print(F("DEBUG mS2LpS() lps under minimum threshold 170: ")); Serial.println(F("0.0"));
    #endif   
    return(0.0); // caution div zero
  }
//  #if DEBUG
//    Serial.print(F("DEBUG mS2LpS() lS/1000: ")); Serial.println(lS/1000);
//  #endif

// conductivity & temperature deemed too marginal for correction (JAN2013) - values logged
  return(lS/1000);
}  //mS2LpS()

unsigned long getPulseInterval(void) { // V-notch weir variant
  WDT_catch = millis();
  min_loop = 0;
  signal_check = 0;
  pulse = 0;

  // prime the moving average LP filter
  for (shiftIndex = 0; shiftIndex <= (WINDOW-1); shiftIndex++){
    digitalWrite(LM555ResetPin, LOW);
    delayMicroseconds(50); // 100 nS min for ALD555
    digitalWrite(LM555ResetPin, HIGH);
    //CD4060ResetPin ignored at the moment, non-existent on South Ruislip project

    avg_pulse[shiftIndex] = pulseIn(pulseSensePin, HIGH)/WINDOW; // cheap rounding => !5.0
    pulse += avg_pulse[shiftIndex];
  }
  shiftIndex = WINDOW-2;
  min_pulse = pulse + 1;
  max_pulse = min_pulse - 1;

  // stall if readings don't change, record changing flow, not constant flow.
  // but do not hibernate long on constant flow => flow below critical value -> long hibernate (long peripheral switch on times)
  do{
    ref_pulse = pulse;
    pulse -= avg_pulse[shiftIndex];
    // moving window LP filter
    digitalWrite(LM555ResetPin, LOW);
    delayMicroseconds(50); // 100 nS min for ALD555
    digitalWrite(LM555ResetPin, HIGH);
    //CD4060ResetPin ignored at the moment, non-existent on South Ruislip project

    raw_pulse = pulseIn(pulseSensePin, HIGH);
    if (max_pulse < raw_pulse) {
      max_pulse = raw_pulse;
    }
    if (min_pulse > raw_pulse) {
      min_pulse = raw_pulse;
    }
    avg_pulse[shiftIndex] = raw_pulse/WINDOW;
    pulse += avg_pulse[shiftIndex];
    // replace array shuffling with a moving index
    if (shiftIndex >= (WINDOW - 1)){
      shiftIndex = 0;
    }else{
      shiftIndex += 1;
    }

    min_loop++;

    signal_check += pulse - ref_pulse; // reflects overall trends
    
//    Serial.print(F(" DEBUG getPulseInterval() millis() - WDT_catch:"));Serial.println(millis() - WDT_catch);
//    if ((millis() - WDT_catch) >= 1000){
//      Serial.println(F(" DEBUG getPulseInterval() millis() - WDT_catch >= 7000"));
//      Serial.println(millis() - WDT_catch);
//    }
//    
//    if (millis() < WDT_catch){
//      Serial.println(F(" DEBUG getPulseInterval() millis() < WDT_catch"));
//     // Serial.println(millis() - WDT_catch);
//    }
    
    if ((millis() - WDT_catch) >= (WDT_RESET_LIMIT - 500)){ // 250 mS => WDT_RESET_LIMIT in mS, 
      wdt_reset();
      #if DEBUG
        //Serial.println(F(" DEBUG getPulseInterval() wdt_reset() loop"));
        //Serial.print(F("."));
      #endif
      if (pulse < NO_WATER) {
        no_flow = true;
        return 0;
      }else{
       no_flow = false;
      } // returned zero allows system sleep
    }    

  } while ((abs(signal_check) < ((pulse/(max_pulse - min_pulse)) + JITTER)) || (min_loop < 2)); 
  // pulse is WINDOW point moving average smoothed value
  // min_loop < 2 => int(pulse) - int(ref_pulse) normalise
  
  no_flow = false;
  wdt_reset();
  //Serial.println(F(" DEBUG getPulseInterval() >0 exit"));
  return pulse;
}  //getPulseInterval()

void initWDT(int sec){
  // set interrupt mode and an interval 
  // duplicate selection command sequence due to critical instruction timing
  switch(sec){

    case 1: 
      cli(); // disable interrupts
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = _BV(WDCE) | _BV(WDE);// allow changes, disable reset
      WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2) | _BV(WDP1); // set WDIE, and 1 second delay
      sei(); // enable interrupts
    break;

    case 2: 
      cli(); // disable interrupts
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = _BV(WDCE) | _BV(WDE);// allow changes, disable reset
      WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0); // set WDIE, and 2 second delay
      sei(); // enable interrupts
    break;

    case 4: 
      cli(); // disable interrupts
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = _BV(WDCE) | _BV(WDE);// allow changes, disable reset
      WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3); // set WDIE, and 4 second delay
      sei(); // enable interrupts
    break;

    case 8: 
      cli(); // disable interrupts
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = _BV(WDCE) | _BV(WDE);// allow changes, disable reset
      WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP0); // set WDIE, and 8 second delay
      sei(); // enable interrupts
    break;

    default: 
      cli(); // disable interrupts
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = _BV(WDCE) | _BV(WDE);// allow changes, disable reset
      WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2) | _BV(WDP1); // set WDIE, and 1 second delay
      sei(); // enable interrupts
  }
} //initWDT()

void uPCsleep(int sec){ 
  // unlike hibernate, sec must be WDT multiples, 1, 2, 4, 8
  //ADCSRA = 0; // disable ADC.. SLEEP_MODE_PWR_DOWN covers this..
  initWDT(sec);
  wdt_reset(); 
  
  #if ECHO_TO_SERIAL
    Serial.flush();
    while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
    UCSR0A |= 1 << TXC0;  // mark transmission not complete
    while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
  #endif
  
  // The ATmega328 has five different sleep states.
  // See the ATmega 328 datasheet for more information.
  // SLEEP_MODE_IDLE -the least power savings 
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN -the most power savings
  //set_sleep_mode (SLEEP_MODE_IDLE); 
  set_sleep_mode (SLEEP_MODE_PWR_DOWN); 
  sleep_enable();
  // turn off brown-out enable in software
  MCUCR = _BV(BODS) | _BV(BODSE); MCUCR = _BV(BODS); 
  sleep_cpu (); 
  sleep_disable(); // cancel sleep as a precaution
  
//  #if TIME_DBG
//    timeProfile = millis();
//  #endif

  
} //uPCsleep()

void hibernate(int sec){
  // only seems to work with optiboot bootloader
  // switching MOSFET off for less than 2/3 seconds -> MOSFET not fully switched
  // measured 9.3 mA on hibernation with peripherals switched off
  
//  #if ECHO_TO_SERIAL
//   Serial.end();
//  #endif
  
  #if LED_ON
    //pinMode(greenLEDpin, INPUT);  // save power
    //pinMode(redLEDpin, INPUT);  // save power
    digitalWrite(redLEDpin, HIGH);
  #endif
  digitalWrite(gateMOSFET, LOW); // turn off peripherals
  #if LED_ON
    delay(200); 
    digitalWrite(redLEDpin, LOW);
  #endif
  
  int mod = sec%2;
  if (mod){uPCsleep(1);}

  mod = (sec%4)/2;
  if (mod){uPCsleep(2);}

  mod = (sec%8)/4;
  if (mod){uPCsleep(4);}

  mod = sec/8;
  for (i = 0; i < mod; i++){
    uPCsleep(8);  
    #if LED_ON
      digitalWrite(redLEDpin, HIGH);
      delay(200); 
      digitalWrite(redLEDpin, LOW);
    #endif

  }

  digitalWrite(gateMOSFET, HIGH); // turn on peripherals
  delay(1000);  // wait for MOSFET to fully switch (need a bigger capacitor across totem?)
  #if LED_ON
    digitalWrite(greenLEDpin, HIGH);
    delay(200);
    digitalWrite(greenLEDpin, LOW);
  #endif
  
//  #if ECHO_TO_SERIAL
//   Serial.begin(SERIAL_BAUD); 
//  #endif
  
  // order of sleep lengths allow 8 sec uPC WDT timeout
  
//  #if TIME_DBG
//    timeProfile = millis() - timeProfile;
//    Serial.print(F("TIMING hibernate() exit A: "));
//    Serial.println(timeProfile);
//  #endif

  wdt_reset();
  
//  #if TIME_DBG
//    timeProfile = millis() - timeProfile;
//    Serial.print(F("TIMING hibernate() exit B: "));
//    Serial.println(timeProfile);
//  #endif

  //pinMode(greenLEDpin, OUTPUT);
  //pinMode(redLEDpin, OUTPUT);
} //hibernate()            

void setup(void)
{
  wdt_disable();
  //delay(600); // power down + hold reset to upload to WDT bricked chip

  // just protoboard
  pinMode(greenLEDpin, OUTPUT);
  #if ECHO_TO_SERIAL
    pinMode(redLEDpin, INPUT);
  #else
    #if LED_ON
      pinMode(redLEDpin, OUTPUT);
    #endif
  #endif
  pinMode(pulseSensePin, INPUT);
  pinMode(LM555ResetPin, OUTPUT);
  pinMode(TMP36pin, INPUT);
  
  // Caplogger
  pinMode(chipSelect, OUTPUT); //initialised within initialiseSDcard()
  pinMode(CD4060ResetPin, INPUT); // OUTPUT for CD4060 on board
  pinMode(batteryVoltagePin, INPUT);
  pinMode(batteryTestEnablePin, OUTPUT);
  pinMode(Conduct_BsensePin, INPUT);
  pinMode(Conduct_AdrivePin, OUTPUT);
  pinMode(gateMOSFET, OUTPUT);  
  //pinMode(chipSelect, OUTPUT); initialised within initialiseSDcard()
  digitalWrite(gateMOSFET, HIGH); // turn on peripherals
  delay(4000); // power up delay

  #if ECHO_TO_SERIAL
    while (!Serial) {} // Leonardo/Olimex 32U4
    Serial.begin(SERIAL_BAUD);
    Serial.flush();
    Serial.println(F("  RESET"));    
  #endif


  #if ECHO_TO_SERIAL
    if (!RTC.begin()) Serial.println(F("FAILURE RTC initiate")); // connect to RTC
  #else
    RTC.begin();
  #endif

  #if RTC_SYNC
    // ensure WTC disabled
    requestSync();//NOTIFICATIOM TO TETHER PC PROGRAM THAT RTC TIME IS TO BE CHECKED AND UPDATED IF DIFFERENT
  #endif

  // dateTimeCallback() sets the function that is called when a file is created
  // or when a file's directory entry is modified by sync().
  // The callback can be disabled by the call SdFile::dateTimeCallbackCancel()
  
  // set date time callback function
  #if SD_LOG
    SdFile::dateTimeCallback(dateTime);
    DateTime now = RTC.now(); 
    initialiseSDcard();
  #endif

  #if LED_ON
    for (i = 0; i < 8; i++){ // initialise
      digitalWrite(redLEDpin, HIGH);
      digitalWrite(greenLEDpin, LOW);
      delay(100); 
      digitalWrite(redLEDpin, LOW);
      digitalWrite(greenLEDpin, HIGH);
      delay(100); 
    }
    digitalWrite(redLEDpin, LOW);
    digitalWrite(greenLEDpin, LOW);
  #endif
  
  #if DEBUG
    Serial.print(F("DEBUG Free RAM: ")); 
    Serial.println(FreeRam()); //bytes!
  #endif
  
  //wdt_enable(WDTO_8S);
  initWDT(8);
  wdt_reset();


}  //setup()

void loop(void)
{
  unsigned int log_inc = 0;
  unsigned long mS_interval;
  char *f_name;
  char *f_csv; 
  char *f_tbc; 
  float LpS; // litres per second 

  #if RTC_SYNC
    #if ECHO_TO_SERIAL
      serialSynch();
    #endif
  #endif

  #if SD_LOG
    f_name = uniqueLogFileName();
    logfile = createLogFile(f_name); 
    logfile.println(F("Rev:0.6.0,   S.RUISLIP"));
    logfile.println(F(SENSOR_IDENTIFIER));
    logfile.println(F("temp:C,power:V,cond:mS"));
    f_tbc = formatTBC(getTemp(), getPowerVoltage(), getSiemens());
    logfile.println(f_tbc);
    logfile.println(F("yyyymmddThhmmss,dm^3/s"));
  #endif

  #if ECHO_TO_SERIAL
    Serial.print(F("Logfile created: ")); 
    Serial.println(f_name);
    Serial.println(F("Rev:0.6.0,   S.RUISLIP"));
    Serial.println(F(SENSOR_IDENTIFIER));
    Serial.println(F("yyyymmddThhmmss,dm^3/s"));
    Serial.println(f_tbc);
  #endif

  #if TIME_DBG
    timeProfile = millis();
  #endif

  while (log_inc < FILE_MAX_LOG) {

    #if DEBUG
        Serial.print(F("DEBUG--------------------->log_inc: ")); Serial.println(log_inc);
    #endif

    #if TIME_DBG
        timeProfile = millis() - timeProfile;
        //if (timeProfile >= 8000){
          Serial.print(F("TIMING 1: ")); Serial.println(timeProfile);
        //}
    #endif

    //wdt_reset();

    mS_interval = getPulseInterval();
   

    #if TIME_DBG
        timeProfile = millis() - timeProfile;
        //if (timeProfile >= 8000){
          Serial.print(F("TIMING 2: ")); Serial.println(timeProfile);
        //}
    #endif


    if (no_flow == false) {

      #if TIME_DBG
        timeProfile = millis() - timeProfile;
        //if (timeProfile >= 8000){
          Serial.print(F("TIMING 3- flow: ")); Serial.println(timeProfile);
        //}
      #endif

      LpS = mS2LpS(mS_interval);
  
      #if DEBUG
        Serial.print(F("DEBUG loop() mS_interval: ")); Serial.println(mS_interval);
        Serial.print(F("DEBUG loop() LpS: ")); Serial.println(LpS);
      #endif

      #if TIME_DBG
        timeProfile = millis() - timeProfile;
        //if (timeProfile >= 8000){
          Serial.print(F("TIMING 3A flow: ")); Serial.println(timeProfile);
        //}
      #endif

      f_csv = formatCSV(LpS, RTC.now()); // now() skewed by mS2LpS() & getPulseInterval() execution time (< 0.5 S)
  
      #if SD_LOG
        #if LED_ON
          digitalWrite(redLEDpin, HIGH); // warn writing is taking place
        #endif

      #if TIME_DBG
        timeProfile = millis() - timeProfile;
        if (timeProfile >= 8000){
          Serial.print(F("TIMING 3B flow: ")); Serial.println(timeProfile);
        }
      #endif

        logfile.println(f_csv);

      #if TIME_DBG
        timeProfile = millis() - timeProfile;
        if (timeProfile >= 8000){
          Serial.print(F("TIMING 3C flow: ")); Serial.println(timeProfile);
        }
      #endif

        #if LED_ON
          digitalWrite(redLEDpin, LOW);
        #endif
      #endif

      #if ECHO_TO_SERIAL
          Serial.println(f_csv);
      #endif

      still_no_flow = 0;
    } else {
      #if TIME_DBG
      Serial.print(F("TIMING 3 NO-flow appear "));
        timeProfile = millis() - timeProfile;
        if (timeProfile >= 8000){
          Serial.print(F("TIMING 3 NO-flow: ")); Serial.println(timeProfile);
        }
      #endif

      log_inc--; 
    }  // skip a count
  
    #if TIME_DBG
      timeProfile = millis() - timeProfile;
      if (timeProfile >= 8000){
        Serial.print(F("TIMING 4 main loop, flow -> no-flow: ")); Serial.println(timeProfile);
      }
    #endif


    #if SLEEP_ON 
      if (no_flow) {
        #if DEBUG
          Serial.print(F("DEBUG loop() still_no_flow: ")); Serial.println(still_no_flow);
        #endif
        
        #if LED_ON
          digitalWrite(greenLEDpin, LOW);
          digitalWrite(redLEDpin, HIGH);
        #endif
        //By performing a sync() frequently, you can remove the SD card at any time,
        //or power off or press reset, and never lose more than a few lines of data.
        //uint32_t syncTime = 0; // time of last sync()______________________________
      
        #if SD_LOG
          wdt_reset(); // reset WDT here for sync
          logfile.sync();
        #endif
          
        #if LED_ON
          digitalWrite(redLEDpin, LOW);
        #endif
        
        if (still_no_flow < 16) {//32
          uPCsleep(8); // 4 seconds to ensure MOSFET switching, avoid until longer sleep periods
          // careful to leave WDT to 8 second reset..
          //wdt_reset();
        } else if (still_no_flow > 16) {//32
          hibernate(still_no_flow);
        } else if (still_no_flow > 180) {
          hibernate(180); // limit sleep intervals to 180 seconds (mainly to avoid human error)
        }
        
        //wdt_reset();
        #if TIME_DBG
          timeProfile = millis() - timeProfile;
          if (timeProfile >= 8000){
            Serial.print(F("TIMING 5 (no-flow/sleep): ")); Serial.println(timeProfile);
          }
        #endif

        still_no_flow += 3; 
      }
    #endif
    
    log_inc++; // increment to next line of data in CSV file

    #if TIME_DBG
      timeProfile = millis() - timeProfile;
      if (timeProfile >= 8000){
        Serial.print(F("TIMING 6(log_inc++) A: ")); Serial.println(timeProfile);
      }
    #endif
    // minimum time resolution, sleep & WDT sec reset
    uPCsleep(RESOLUTION);
    initWDT(8);
    wdt_reset();

    #if TIME_DBG
      timeProfile = millis() - timeProfile;
      if (timeProfile >= 8000){
        Serial.print(F("TIMING 6(log_inc++) B: ")); Serial.println(timeProfile);
      }
    #endif
  }

  #if SD_LOG
    logfile.close();
  
    #if ECHO_TO_SERIAL
      Serial.print(F("Logfile closed: ")); Serial.println(f_name); 
    #endif
    
    if (!fileCheck(f_name)){
      #if ECHO_TO_SERIAL
        Serial.print(F("FAILURE file length check")); 
      #endif
      while(1){} // force system reset 
    }
  #endif
  
//  #if DEBUG
//    Serial.print(F("DEBUG loop() -sleep between readings")); 
//  #endif
  
  #if TIME_DBG
      timeProfile = millis() - timeProfile;
      if (timeProfile >= 8000){
        Serial.print(F("TIMING 5(new_file) A: ")); Serial.println(timeProfile);
      }
  #endif
  wdt_reset();
}


//__TODO: create cache 512 byte buffer and flush as file append -> low risk of data loss on card removal
// current strategy to flush on no_flow timeout
//__TODO: flashing errorcodes? only for offline debugging.. prefer WDT system reset


