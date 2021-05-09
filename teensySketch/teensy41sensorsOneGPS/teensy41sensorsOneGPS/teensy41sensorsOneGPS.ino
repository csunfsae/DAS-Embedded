/************************************************************************************************************************
 * This code is for the Teensy 4.1 in the MataMotive system. It reads the data coming from
 * the 2 GPS units connected to the teensy pins and sends that data over the CANBUS network. It also
 * reads the data coming from the analong sensors on the vehicle and sends that over the
 * CANBUS network too.
 * 
 * Connect the GPS Power pin to 5V (stable power is better)
 * Connect the GPS Ground pin to ground
 * Connect the GPS TX (transmit) pin to Digital serial pin on teensy 4.1
 * Connect the GPS RX (receive) pin to Digital serial pin on teensy 4.1
 * 
 * // FlexCAN_T4 Library github link https://github.com/tonton81/FlexCAN_T4
 * // Adafruit GPS github link https://github.com/adafruit/Adafruit_GPS
 ************************************************************************************************************************/

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * The problem with this is that it prints out chars that alternate from GPS1 and GPS2
 *  Example: ,$4G8P,G1S6V0,,33,03,,1152,,4143,,12043,,03618,,2461,,2186,,22803,,34120,,1204,,2065,,21287,,04415*,7253,
 *  
 *  Cannot send CAN frames via canbus header on teensy. GPS Teensy will stop displaying GPS data via serial console
 *  when 'CAN2' is used
 *  
 * 
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

#include <FlexCAN_T4.h>    // FlexCan library for Teensy 4.0 and 4.1
#include <Adafruit_GPS.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the arduino Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
#define GPSSerial2 Serial2 // Hardware serial ports on Teensy 4.1

static CAN_message_t canMsg;            // Structure of a CANBUS message that is sent over Teensy CANBUS port
const int ledPin =  LED_BUILTIN;        // The pin number for LED
int ledState = LOW;                     // ledState used to set the LED

unsigned long previousMillis = 0;       // Will store last time the Teensy LED was updated
uint32_t gpsTimer = millis();           // Used for printing GPS data to serial console once per unit of time
const long interval = 1000;             // Interval at which to blink LED on Teensy (milliseconds)

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> canbus; /* CAN2 is Teensy4.1 pins 0 & 1.
                                                     CAN3 pins support regular CAN2.0 and CANFD modes */

Adafruit_GPS GPS(&GPSSerial2);      // Connect to the GPS units on separate hardware serial ports



// ----------------------------------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(9600);      // Initiate serial ports with baud rate of 9600
  Serial.println("Adafruit GPS basic test!");
  pinMode(ledPin, OUTPUT); // Set the digital pin as output
  canbus.begin();          
  canbus.setBaudRate(500000); // This value has to match the baud rate on the Quasar/Jetson TX2 board
  //canbus.setRX(ALT);
  //canbus.setTX(ALT);
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800

  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the GPS update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}



// ------------------------------------------------------------------------------------------------------
void loop(void) {

  unsigned long currentMillis = millis(); // Used for blinking the led on Teensy 4.1
  
  // Blink the LED
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time you blinked the LED

    // If the LED is off turn it on and vice-versa
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    digitalWrite(ledPin, ledState); // Set the LED with the ledState of the variable
  }

  // Print out raw GPS data to audino serial console for each GPS separately
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  // If a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    Serial.println("NMEA has been received on GPS1!");
    // A tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (!GPS.parse(GPS.lastNMEA()))   // This also sets the newNMEAreceived() flag to false
      return;  // We can fail to parse a sentence in which case we should just wait for another
  }

  // Approximately every 2 seconds or so, print out the current GPS stats
  if (millis() - gpsTimer > 2000) {
    gpsTimer = millis(); // reset the timer
    int gpsNum = 1;
    printGPSStats(GPS, gpsNum);
  }

  // 2 CANBUS frames will be sent to include all data.
  // Frame 1 includes hour, minute, seconds, fix, speed, angle
  // Frame 2 includes Latitude and Longitude
  // Populate CANBUS struct with GPS1 data that we want to send
  fillCanbusFrameOne(GPS);
  canbus.read(canMsg); // CANBUS pin will read the canMsg struct just populated with data  
  //canSniff(); // Print out the data contained in canMsg
  canbus.write(canMsg); // CANBUS pin will send CANBUS packet

  
  /*fillCanbusFrameTwo(GPS);
  canbus.read(canMsg); // CANBUS pin will read the canMsg struct just populated with data  
  //canSniff(); // Print out the data contained in canMsg
  canbus.write(canMsg); // CANBUS pin will send CANBUS packet*/

  // Populate CANBUS sruct with GPS2 data that we want to send
  /* Here */
}



void fillCanbusFrameOne(Adafruit_GPS GPS) {
  // Split gpsAngle binary number into 2 separate bytes
  uint16_t gpsAngle = GPS.angle; // 'angle' is mentioned as 'course' in Adafruit lib header files
  uint8_t angleLeftByte = (uint8_t)gpsAngle; 
  uint8_t angleRightByte = (uint8_t)(gpsAngle >> 8);
  
  canMsg.flags.extended = 0;
  canMsg.flags.remote = 0;
  canMsg.id = 0x34; // id was chosen randomly
  canMsg.buf[0] = GPS.hour;
  canMsg.buf[1] = GPS.minute;
  canMsg.buf[2] = GPS.seconds;
  canMsg.buf[3] = GPS.fix;
  canMsg.buf[4] = GPS.speed; 
  canMsg.buf[5] = angleLeftByte; // This is little endian format
  canMsg.buf[6] = angleRightByte;
  //canMsg.buf[7] = Room for one more byte here
}



void fillCanbusFrameTwo(Adafruit_GPS GPS) {
  canMsg.flags.extended = 0;
  canMsg.flags.remote = 0;
  canMsg.id = 0x35; // id was chosen randomly
  /*canMsg.buf[0] = GPS.hour;
  canMsg.buf[1] = GPS.minute;
  canMsg.buf[2] = GPS.seconds;
  canMsg.buf[3] = GPS.fix;
  canMsg.buf[4] = GPS.latitude;
  canMsg.buf[5] = GPS.longitude;
  canMsg.buf[6] = GPS.speed; 
  canMsg.buf[7] = GPS.*/
}



void printGPSStats (Adafruit_GPS GPS, int gpsNum) {
    Serial.print("\nStats for GPS ");
    Serial.print(gpsNum);
    Serial.print(":");
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}



void canSniff() {
    Serial.print("MB "); Serial.print(canMsg.mb);
    Serial.print("  OVERRUN: "); Serial.print(canMsg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(canMsg.len);
    Serial.print(" EXT: "); Serial.print(canMsg.flags.extended);
    Serial.print(" TS: "); Serial.print(canMsg.timestamp);
    Serial.print(" ID: "); Serial.print(canMsg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < canMsg.len; i++ ) {
      Serial.print(canMsg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
}
