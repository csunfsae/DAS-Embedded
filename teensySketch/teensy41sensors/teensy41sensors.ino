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

#include <FlexCAN_T4.h>    // FlexCan library for Teensy 4.0 and 4.1
#include <Adafruit_GPS.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the arduino Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
#define GPSSerial1 Serial1 // Hardware serial ports on Teensy 4.1
#define GPSSerial2 Serial2

static CAN_message_t canMsg;            // Structure of a CANBUS message that is sent over Teensy CANBUS port
const int ledPin =  LED_BUILTIN;        // The pin number for LED
int ledState = LOW;                     // ledState used to set the LED

unsigned long previousMillis = 0;       // Will store last time the Teensy LED was updated
uint32_t gpsTimer = millis();           // Used for printing GPS data to serial console once per unit of time
const long interval = 1000;             // Interval at which to blink LED on Teensy (milliseconds)

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canbus; /* CAN2 is Teensy4.1 pins 0 & 1.
                                                     CAN3 pins support regular CAN2.0 and CANFD modes */

Adafruit_GPS GPS(&GPSSerial1);      // Connect to the GPS units on separate hardware serial ports
Adafruit_GPS GPS2(&GPSSerial2);



// ----------------------------------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(9600);      // Initiate serial ports with baud rate of 9600
  Serial.println("Adafruit GPS basic test!");
  pinMode(ledPin, OUTPUT); // Set the digital pin as output
  canbus.begin();          
  canbus.setBaudRate(500000); // This value has to match the baud rate on the Quasar/Jetson TX2 board
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS2.begin(9600);

  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS2.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the GPS update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS2.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  GPS2.sendCommand(PGCMD_ANTENNA);
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
  // The problem with this is that it prints out chars that alternate from GPS1 and GPS2
  // Example: ,$4G8P,G1S6V0,,33,03,,1152,,4143,,12043,,03618,,2461,,2186,,22803,,34120,,1204,,2065,,21287,,04415*,7253,
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);
  c = GPS2.read();
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
  if (GPS2.newNMEAreceived()) {
    Serial.println("NMEA has been received on GPS2!");
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (!GPS2.parse(GPS2.lastNMEA()))   // This also sets the newNMEAreceived() flag to false
      return;  // We can fail to parse a sentence in which case we should just wait for another
  }

  // Approximately every 2 seconds or so, print out the current GPS stats
  if (millis() - gpsTimer > 2000) {
    gpsTimer = millis(); // reset the timer
    int gpsNum = 1;
    printGPSStats(GPS, gpsNum);
    gpsNum = 2;
    printGPSStats(GPS2, gpsNum);
  }

  // Populate CANBUS struct with GPS1 data that we want to send
  //populateCanbusStruct(GPS);
  canMsg.flags.extended = 0; // = random(0,2);
  canMsg.flags.remote = 0;
  canMsg.len = 8;
  canMsg.id = 0x34;
  canMsg.buf[0] = 0;
  canMsg.buf[1] = 1;
  canMsg.buf[2] = 2;
  canMsg.buf[3] = 3;
  canMsg.buf[4] = 4;
  canMsg.buf[5] = 5;
  canMsg.buf[6] = 6; 
  canMsg.buf[7] = 7; // Angle is mentioned as course in Adafruit lib header files

  //canbus.read(canMsg); // CANBUS pin will read the canMst struct just populated with data  
  //canSniff(); // Print out the data contained in canMsg
  canbus.write(canMsg); // CANBUS pin will send CANBUS packet

  // Populate CANBUS sruct with GPS2 data that we want to send
  //populateCanbusStruct(GPS);
  //canbus.read(canMsg); // CANBUS pin will read the canMst struct just populated with data  
  //canSniff(); // Print out the data contained in canMsg
  //canbus.write(canMsg); // CANBUS pin will send CANBUS packet

  // This is the output of the can frame displayed in the serial monitor
  // MB 0  OVERRUN: 0  LEN: 8 EXT: 0 TS: 0 ID: 34 Buffer: 4 6 28 58 47 0 0 3E
}



void populateCanbusStruct(Adafruit_GPS GPS) {
  canMsg.flags.extended = 0; // = random(0,2);
  canMsg.flags.remote = 0;
  canMsg.len = 8;
  canMsg.id = 0x34;
  canMsg.buf[0] = GPS.hour;
  canMsg.buf[1] = GPS.minute;
  canMsg.buf[2] = GPS.seconds;
  canMsg.buf[3] = GPS.latitude;
  canMsg.buf[4] = GPS.longitude;
  canMsg.buf[5] = GPS.speed;
  canMsg.buf[6] = GPS.magvariation; 
  canMsg.buf[7] = GPS.angle; // Angle is mentioned as course in Adafruit lib header files
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
