/*************************************************************************************************************************************
 * This code is for the Teensy 4.1 in the MataMotive system. It reads the data coming from
 * the 2 GPS units connected to the teensy pins and sends that data over the CANBUS network. It also
 * reads the data coming from the analong sensors on the vehicle and sends that over the
 * CANBUS network too.
 * 
 * Connect the GPS Power pin to 5V (stable power is better)
 * Connect the GPS Ground pin to ground
 * Connect the GPS TX (transmit) pin to Digital serial RX pin on teensy 4.1
 * Connect the GPS RX (receive) pin to Digital serial TX pin on teensy 4.1
 * 
 * FlexCAN_T4 Library github link https://github.com/tonton81/FlexCAN_T4
 * Adafruit GPS github link https://github.com/adafruit/Adafruit_GPS
 ************************************************************************************************************************************/

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * The problem with this is that it prints out chars that alternate from GPS1 and GPS2
 * When 2 GPS units are sending data to the teensy at once and then I try to print out the data
 * received from the GPS units separately, the GPS data from both units gets printed simultaneously
 * and are displayed with the data mixed together.
 * Example: ,$4G8P,G1S6V0,,33,03,,1152,,4143,,12043,,03618,,2461,,2186,,22803,,34120,,1204,,2065,,21287,,04415*,7253,
 *  
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

// TEENSY SENDS GPS DATA TO SERVER. LAT & LON ARE NOT CORRECT. 

#include <FlexCAN_T4.h>               // FlexCan library for Teensy 4.0 and 4.1 ONLY
#include <Adafruit_GPS.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the arduino Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
#define GPSSerial2 Serial2            // Hardware serial ports on Teensy 4.1
#define GPSSerial1 Serial1

const int ledPin =  LED_BUILTIN;      // The pin number for LED
int ledState = LOW;                   // ledState used to set the LED
const long interval = 1000;           // Interval at which to blink LED on Teensy (milliseconds)
unsigned long previousMillis = 0;     // Will store last time the Teensy LED was updated

static CAN_message_t canMsg;          // Structure of a CANBUS message that is sent over Teensy CANBUS port
uint32_t gpsTimer = millis();         // Used for printing & sending GPS data once per unit of time

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canbus; /* CAN2 is Teensy4.1 pins 0 & 1.
                                                     CAN3 pins support regular CAN2.0 and CANFD modes */

Adafruit_GPS GPS(&GPSSerial2);        // Connect to the GPS units on separate hardware serial ports
//Adafruit_GPS GPS2(&GPSSerial1);



// -------------------------------------------------------------------------------------------------------------------
void setup(void)
{
  Serial.println("Adafruit GPS basic test!");
  Serial.begin(9600);         // Allows arduino (not Teensy) to communicate with the arduino serial monitor
  pinMode(ledPin, OUTPUT);    // Set the digital pin as output
  canbus.begin();
  canbus.setBaudRate(500000); // This value has to match the baud rate on the Quasar/Jetson TX2 board
  GPS.begin(9600);            // 9600 NMEA is the default baud rate for Adafruit MTK GPS's - some use 4800

  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the GPS update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  /* For the parsing code to work nicely and have time to sort thru the data, and
                           print it out we don't suggest using anything higher than 1 Hz */

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}



// -------------------------------------------------------------------------------------------------------------------
void loop(void)
{
  // Blink the LED
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;     // Save the last time you blinked the LED

    // If the LED is off turn it on and vice-versa
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    digitalWrite(ledPin, ledState);     // Set the LED with the ledState of the variable
  }

  // Print out raw GPS data to audino serial console for each GPS separately
  char c = GPS.read();
  if ((c) && (GPSECHO)) {
    Serial.write(c);
  }

  // A tricky thing here is if we print the NMEA sentence, or data
  // we end up not listening and catching other sentences!
  // So be very wary if using OUTPUT_ALLDATA and trytng to print out data
  //
  // If a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
  Serial.println("NMEA has been received on GPS1!");
  if (!GPS.parse(GPS.lastNMEA())) // This also sets the newNMEAreceived() flag to false
    return;           // We can fail to parse a sentence in which case we should just wait for another
  }

  // Print the info received from GPS and then send CANBUS packets with a max # of times per second
  if (millis() - gpsTimer > 2000) {
    gpsTimer = millis();          // Reset the timer
    Serial.print("GPS Timer (ms): ");
    Serial.print(gpsTimer);
    int gpsNum = 1;
    printGPSStats(GPS, gpsNum);

    // 2 CANBUS frames will be sent to include all data
    // Frame 1 includes hour, minute, seconds, fix, speed, angle
    // Frame 2 includes Latitude and Longitude
    fillCanbusFrameOne(GPS, gpsNum, gpsTimer);  // Fill CANBUS struct with 1st part of GPS1 data that we want to send
    canbus.read(canMsg);          // CANBUS pin will read the canMsg struct just populated with data  
    Serial.println("____FRAME ONE___ "); canSniff();                 // Print out CAN frame
    canbus.write(canMsg);         // Send packet over CANBUS


    fillCanbusFrameTwo(GPS, gpsNum, gpsTimer);  // Fill CANBUS struct with 2nd part of GPS1 data that we want to send
    canbus.read(canMsg);        // CANBUS pin will read the canMsg struct just populated with data  
    Serial.println("____FRAME TWO___ "); canSniff();               // Print out CAN frame
    canbus.write(canMsg);       // Send packet over CANBUS

    /*
    Maybe here I can fill another CANBUS frame with Day, Month, and Year and send that too.
    Otherwise I will have to have the "gps_lap_timer.cpp" ROS node attach its own Day, Month, Year
    and that may cause problems with having the wrong date & time.
    I may have to pause interrupts on the Teensy to stop the time/day/month/year from being updated while
    it is in the middle of writting that information to a canbus packet. Or maybe the funciton that
    gives me date and time already does that for me.
    */
  }
  
  /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Print out raw GPS data to audino serial console for each GPS separately
  c = GPS2.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  // A tricky thing here is if we print the NMEA sentence, or data
  // we end up not listening and catching other sentences!
  // So be very wary if using OUTPUT_ALLDATA and trytng to print out data
  //
  // If a sentence is received, we can check the checksum, parse it...
  if (GPS2.newNMEAreceived()) {
    Serial.println("NMEA has been received on GPS2!");
    if (!GPS2.parse(GPS2.lastNMEA())) // This also sets the newNMEAreceived() flag to false
      return;             // We can fail to parse a sentence in which case we should just wait for another
  }
  
  // Print the info received from GPS and then send CANBUS packets with a max # of times per second
  if (millis() - gpsTimer > 2000) {
    gpsNum = 2;
    printGPSStats(GPS2, gpsNum);
    gpsTimer = millis();    // Reset the timer
    // Fill CANBUS sruct with GPS2 data that we want to send
    // Frame 1 includes hour, minute, seconds, fix, speed, angle
    // Frame 2 includes Latitude and Longitude
    fillCanbusFrameOne(GPS2, gpsNum); // Fill CANBUS struct with 1st part of GPS2 data that we want to send
    canbus.read(canMsg);          // CANBUS pin will read the canMsg struct just populated with data  
    //canSniff();               // Print out CAN frame
    canbus.write(canMsg);         // CANBUS pin will send CANBUS packet

    fillCanbusFrameTwo(GPS2, gpsNum); // Fill CANBUS struct with 2nd part of GPS2 data that we want to send
    canbus.read(canMsg);        // CANBUS pin will read the canMsg struct just populated with data  
    //canSniff();           // Print out CAN frame
    canbus.write(canMsg);       // CANBUS pin will send CANBUS packet
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
}



void fillCanbusFrameOne(Adafruit_GPS GPS, int gpsNum, uint32_t gpsTimer) {
  canMsg.flags.extended = 0;
  canMsg.flags.remote = 0;
  
  if (gpsNum == 1)
    canMsg.id = 0x35;        // The gps_lap_timer.cpp code is looking for these hex values
   else
    canMsg.id = 0x37;
  
  canMsg.buf[0] = GPS.hour;     // The CAN_message_t struct also contains a uint16_t timespamp field that may be able
  canMsg.buf[1] = GPS.minute;   // to be used in place of GPS.hour, GPS.minute, and GPS.second
  canMsg.buf[2] = GPS.seconds;
  canMsg.buf[3] = GPS.fix;
  canMsg.buf[4] = GPS.speed;
  
  uint16_t angle = (uint16_t)(GPS.angle);
  canMsg.buf[5] = (uint8_t)(angle >> 8); // This is big endian format
  canMsg.buf[6] = (uint8_t)angle;
  //canMsg.buf[7] =         Room for one more byte here
}



void fillCanbusFrameTwo(Adafruit_GPS GPS, int gpsNum, uint32_t gpsTimer) {
  canMsg.flags.extended = 0;
  canMsg.flags.remote = 0;
  canMsg.timestamp = gpsTimer;
  
  if (gpsNum == 1)
    canMsg.id = 0x36;
   else
    canMsg.id = 0x38;
  
  int32_t latitude = (int32_t)(GPS.latitude * 10000);
  if (GPS.lat == 'S')     // If in the Southern hemisphere then represent that as a negative coordinate
    latitude = latitude * -1;
  int32_t longitude = (int32_t)(GPS.longitude * 10000);
  if (GPS.lon == 'E')     // If in the Eastern hemisphere then represent that as a negative coordinate
    longitude = longitude * -1;

  canMsg.buf[0] = (int8_t)(latitude >> 24); // Latitude & Longitude are stored in big endian format
  canMsg.buf[1] = (int8_t)(latitude >> 16); // Lat & Lon is in Degrees, minutes, seconds format. NOT decimal format
  canMsg.buf[2] = (int8_t)(latitude >> 8); 
  canMsg.buf[3] = (int8_t)latitude;
  canMsg.buf[4] = (int8_t)(longitude >> 24);
  canMsg.buf[5] = (int8_t)(longitude >> 16);
  canMsg.buf[6] = (int8_t)(longitude >> 8);
  canMsg.buf[7] = (int8_t)longitude;
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



// Print out the data contained in canMsg to the arduino serial consol
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
  }
  Serial.println();
}

// TODO
/* If the GPS loses power after getting a fix but the rest of the DAS (TX2, Teensy, HUD) are still powered on,
 * then this code will keep sending CANBUS frames filled with the last data it recieved. Deal with this possible
 * situation by sending out CANBUS frames that are filled with 0's in the data fields. */
  
// Teensy should only send out CANBUS frames when new data from the GPS is received.

/* Teensy needs to read GPS data from one unit, send it, then read GPS data from the other (2nd)
 unit and then send that to. Then repeat that process in that order. */

 // GPS.milliseconds is always zero. Fix that

 // Send GPS.milliseconds in the CANBUS packet. Then make sure gps_lap_timer.cpp reads the packet correctly.
