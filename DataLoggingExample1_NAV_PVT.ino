
#include <SPI.h>
#include <SD.h>

// display libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
#include <EEPROM.h>

unsigned long fileCounter;
uint32_t age = 47;

SFE_UBLOX_GNSS myGNSS;

#define RXD2 16
#define TXD2 17

File myFile;

#define sdChipSelect 5
#define packetLength 100  // NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)
uint8_t *myBuffer;        // Use myBuffer to hold the data while we write it to SD card
char date1[22];
char gpxFilename[25];

// everything for the display
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32

//SoftWire wire(13, 12);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_available = false;

int waitingDots = 0;
uint8_t numSats = 0;

void errorState() {
  display.clearDisplay();
  delay(1000);
  display.fillRect(10, 10, 50, 30, WHITE);
  display.display();
  delay(10000);
  ESP.restart();
}


void createGPXFileIfNotExists(fs::FS &fs, const char *path) {
  Serial.printf("Writing file: %s\n", path);

  if (!SD.exists(path)) {
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }

    file.print(F(
      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
      "<gpx version=\"1.1\" creator=\"Batuev\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
      "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
      "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
      "\t<trk>\r\n<trkseg>\r\n"));  //heading of gpx file
    file.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));

    file.close();

    return;
  }
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  Serial.println();

  Serial.print(F("Time: "));           // Print the time
  uint8_t hms = ubxDataStruct->hour;   // Print the hours
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct->min;            // Print the minutes
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct->sec;            // Print the seconds
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F("."));
  unsigned long millisecs = ubxDataStruct->iTOW % 1000;  // Print the milliseconds
  if (millisecs < 100) Serial.print(F("0"));             // Print the trailing zeros correctly
  if (millisecs < 10) Serial.print(F("0"));
  Serial.print(millisecs);

  long latitude = ubxDataStruct->lat;  // Print the latitude
  Serial.print(F(" Lat: "));

  long modolatitude = latitude % 10000000;
  Serial.print((latitude - modolatitude) / 10000000);
  Serial.print(F("."));
  Serial.print(modolatitude);

  long longitude = ubxDataStruct->lon;  // Print the longitude
  Serial.print(F(" Long: "));
  long modolulongitude = longitude % 10000000;
  Serial.print((longitude - modolulongitude) / 10000000);
  Serial.print(F("."));
  Serial.print(modolulongitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = ubxDataStruct->hMSL;  // Print the height above mean sea level
  Serial.print(F(" Height above MSL: "));
  Serial.print(altitude);
  Serial.println(F(" (mm)"));
}

void updateDisplay(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (display_available == false) { return; }
  waitingDots++;
  if (waitingDots > 3) waitingDots = 0;
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (ubxDataStruct->fixType == 0) {
    display.print("Waiting for fix");
    for (int i = 0; i <= waitingDots; i++) {
      display.print(F("."));
    }
    display.println();
    display.print(F("NumSats:"));
    display.print(numSats);
    display.println();
  }

  if (ubxDataStruct->fixType > 0) {
    display.print("Logging Data.");
    for (int i = 0; i <= waitingDots; i++) {
      display.print(F("."));
    }
    display.println();
    display.print("Fix Type:");
    if (ubxDataStruct->fixType == 1) { display.println("DR"); }
    if (ubxDataStruct->fixType == 2) { display.println("2D"); }
    if (ubxDataStruct->fixType == 3) { display.println("3D"); }

    display.print("pDOP:");
    display.println(ubxDataStruct->pDOP);

    display.print("Sats used for lock:");
    display.println(ubxDataStruct->numSV);
  }


  display.display();
}


void writeGPX(UBX_NAV_PVT_data_t *ubxDataStruct) {
  printPVTdata(ubxDataStruct);
  updateDisplay(ubxDataStruct);

  Serial.println();

  if (ubxDataStruct->fixType == 0) {
    Serial.println("Waiting for fix...");
    return;
  }

  sprintf(gpxFilename, "/%4d-%02d-%02d-%08d.gpx", ubxDataStruct->year, ubxDataStruct->month, ubxDataStruct->day, fileCounter);
  Serial.println(gpxFilename);



  createGPXFileIfNotExists(SD, gpxFilename);

  long latitude = ubxDataStruct->lat;
  long longitude = ubxDataStruct->lon;

  sprintf(date1, "%4d-%02d-%02dT%02d:%02d:%02dZ", ubxDataStruct->year, ubxDataStruct->month, ubxDataStruct->day, ubxDataStruct->hour, ubxDataStruct->min, ubxDataStruct->sec);

  float falt = ((float)ubxDataStruct->hMSL) / 1000;
  float pdop = ((float)ubxDataStruct->pDOP) / 100;

  File dataFile = SD.open(gpxFilename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Could not open gpx file.");

    return;
  }

  unsigned long filesize = dataFile.size();
  if(filesize == 0){
    SD.remove(gpxFilename);
    ESP.restart();
  }
  // back up the file pointer to just before the closing tags
  filesize -= 27;
  dataFile.seek(filesize);
  dataFile.print(F("<trkpt lat=\""));

  long modolatitude = latitude % 10000000;
  dataFile.print((latitude - modolatitude) / 10000000);
  dataFile.print(F("."));
  dataFile.print(modolatitude);

  dataFile.print(F("\" lon=\""));

  long modolulongitude = longitude % 10000000;
  dataFile.print((longitude - modolulongitude) / 10000000);
  dataFile.print(F("."));
  dataFile.print(modolulongitude);

  dataFile.println(F("\">"));
  dataFile.print(F("<time>"));
  dataFile.print(date1);
  dataFile.println(F("</time>"));
  dataFile.print(F("<ele>"));
  dataFile.print(falt, 1);
  dataFile.print(F("</ele>\r\n<pdop>"));
  //dataFile.print(F("<hdop>"));
  dataFile.print(pdop, 3);
  dataFile.println(F("</pdop>\r\n</trkpt>"));
  dataFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
  dataFile.close();
}

void writeFile(UBX_NAV_PVT_data_t *ubxDataStruct) {
  Serial.println();

  Serial.print(F("Time: "));           // Print the time
  uint8_t hms = ubxDataStruct->hour;   // Print the hours
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct->min;            // Print the minutes
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct->sec;            // Print the seconds
  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F("."));
  unsigned long millisecs = ubxDataStruct->iTOW % 1000;  // Print the milliseconds
  if (millisecs < 100) Serial.print(F("0"));             // Print the trailing zeros correctly
  if (millisecs < 10) Serial.print(F("0"));
  Serial.print(millisecs);

  long latitude = ubxDataStruct->lat;  // Print the latitude
  Serial.print(F(" Lat: "));
  Serial.print(latitude);

  long longitude = ubxDataStruct->lon;  // Print the longitude
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = ubxDataStruct->hMSL;  // Print the height above mean sea level
  Serial.print(F(" Height above MSL: "));
  Serial.print(altitude);
  Serial.println(F(" (mm)"));
}



void setupDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }

  display_available = true;
}

void newNAVSAT(UBX_NAV_SAT_data_t *ubxDataStruct) {
  Serial.println();

  Serial.print(F("New NAV SAT data received. It contains data for "));
  Serial.print(ubxDataStruct->header.numSvs);
  numSats = ubxDataStruct->header.numSvs;
  if (ubxDataStruct->header.numSvs == 1)
    Serial.println(F(" SV."));
  else
    Serial.println(F(" SVs."));

  // Just for giggles, print the signal strength for each SV as a barchart
  for (uint16_t block = 0; block < ubxDataStruct->header.numSvs; block++)  // For each SV
  {
    switch (ubxDataStruct->blocks[block].gnssId)  // Print the GNSS ID
    {
      case 0:
        Serial.print(F("GPS     "));
        break;
      case 1:
        Serial.print(F("SBAS    "));
        break;
      case 2:
        Serial.print(F("Galileo "));
        break;
      case 3:
        Serial.print(F("BeiDou  "));
        break;
      case 4:
        Serial.print(F("IMES    "));
        break;
      case 5:
        Serial.print(F("QZSS    "));
        break;
      case 6:
        Serial.print(F("GLONASS "));
        break;
      default:
        Serial.print(F("UNKNOWN "));
        break;
    }

    Serial.print(ubxDataStruct->blocks[block].svId);  // Print the SV ID

    if (ubxDataStruct->blocks[block].svId < 10) Serial.print(F("   "));
    else if (ubxDataStruct->blocks[block].svId < 100) Serial.print(F("  "));
    else Serial.print(F(" "));

    // Print the signal strength as a bar chart
    for (uint8_t cno = 0; cno < ubxDataStruct->blocks[block].cno; cno++)
      Serial.print(F("="));

    Serial.println();
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");


  if (!EEPROM.begin(0x100)) {
    Serial.println("Failed to initialize Filecounter");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }



  fileCounter = EEPROM.readULong(0);
  fileCounter = fileCounter + 1;
  Serial.print("Filecounter is ");
  Serial.println(fileCounter);
  EEPROM.writeULong(0, fileCounter);
  EEPROM.commit();




  setupDisplay();

  // See if the card is present and can be initialized:
  if (!SD.begin()) {
    Serial.println("Card failed, or not present. Freezing...");
    errorState();
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }


  do {
    Serial.println("GNSS: trying 38400 baud");
    Serial2.begin(38400, SERIAL_8N1, RXD2, TXD2);
    if (myGNSS.begin(Serial2) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    if (myGNSS.begin(Serial2) == true) {
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      myGNSS.setSerialRate(38400);
      delay(100);
    } else {
      myGNSS.factoryReset();
      delay(2000);  //Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX);  //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX);    //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration();           //Save the current settings to flash and BBR

  while (Serial.available())  // Make sure the Serial buffer is empty
  {
    Serial.read();
  }


  while (Serial.available())  // Empty the Serial buffer
  {
    Serial.read();
  }



  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful GNSS debug messages on Serial

  // NAV PVT messages are 100 bytes long.
  // In this example, the data will arrive no faster than one message per second.
  // So, setting the file buffer size to 301 bytes should be more than adequate.
  // I.e. room for three messages plus an empty tail byte.
  myGNSS.setFileBufferSize(301);  // setFileBufferSize must be called _before_ .begin


  if (myGNSS.begin(Serial2) == false)  //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at Serial2. Please check wiring. Freezing..."));
    while (1)
      ;
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // (This will also disable any "auto" messages that were enabled and saved by other examples and reduce the load on the I2C bus)
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setI2COutput(COM_TYPE_UBX);                  //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(0.5);  //Produce one navigation solution per second

  // myGNSS.setAutoPVTcallbackPtr(&printPVTdata);  // Enable automatic NAV PVT messages with callback to printPVTdata
  myGNSS.setAutoPVTcallbackPtr(&writeGPX);

  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT);  // Enable automatic NAV SAT messages with callback to newNAVSAT

  myGNSS.logNAVPVT();  // Enable NAV PVT data logging

  myBuffer = new uint8_t[packetLength];  // Create our own buffer to hold the data while we write it to SD card

  Serial.println(F("Press any key to stop logging."));
}



void loop() {
  myGNSS.checkUblox();      // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks();  // Check if any callbacks are waiting to be processed.

  if (myGNSS.fileBufferAvailable() >= packetLength)  // Check to see if a new packetLength-byte NAV PVT message has been stored
  {
    myGNSS.extractFileBufferData(myBuffer, packetLength);  // Extract exactly packetLength bytes from the UBX file buffer and put them into myBuffer

    myFile.write(myBuffer, packetLength);  // Write exactly packetLength bytes from myBuffer to the ubxDataFile on the SD card

    //printBuffer(myBuffer); // Uncomment this line to print the data as Hexadecimal bytes
  }

  if (Serial.available())  // Check if the user wants to stop logging
  {
    myFile.close();  // Close the data file
    Serial.println(F("\r\nLogging stopped. Freezing..."));
    while (1)
      ;  // Do nothing more
  }

  Serial.print(".");
  delay(50);
}

// Print the buffer contents as Hexadecimal bytes
// You should see:
// SYNC CHAR 1: 0xB5
// SYNC CHAR 2: 0x62
// CLASS: 0x01 for NAV
// ID: 0x07 for PVT
// LENGTH: 2-bytes Little Endian (0x5C00 = 92 bytes for NAV PVT)
// PAYLOAD: LENGTH bytes
// CHECKSUM_A
// CHECKSUM_B
// Please see the u-blox protocol specification for more details
void printBuffer(uint8_t *ptr) {
  for (int i = 0; i < packetLength; i++) {
    if (ptr[i] < 16) Serial.print("0");  // Print a leading zero if required
    Serial.print(ptr[i], HEX);           // Print the byte as Hexadecimal
    Serial.print(" ");
  }
  Serial.println();
}
