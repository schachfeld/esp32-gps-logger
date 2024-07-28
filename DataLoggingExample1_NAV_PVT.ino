
#include <SPI.h>
#include <SD.h>

// display libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// bluetooth
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#define RXD2 16
#define TXD2 17

File myFile;

#define sdChipSelect 5
#define packetLength 100  // NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)
uint8_t *myBuffer;        // Use myBuffer to hold the data while we write it to SD card
char date1[22];
char gpxFilename[16];

// everything for the display
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32

// Bluetooth
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;

BLECharacteristic hourCharacteristic("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor hourDescriptor(BLEUUID((uint8_t)0x2902));

BLECharacteristic minCharacteristic("a23db600-7691-4fe0-8854-82614492bd4d", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor minDescriptor(BLEUUID((uint8_t)0x2902));

BLECharacteristic secCharacteristic("0ea1b645-837d-4249-98bb-5b0d3cfa0003", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor secDescriptor(BLEUUID((uint8_t)0x2902));

BLECharacteristic hourCharacteristic("702ca3f1-8343-470d-810f-4d49e8ac541c", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor hourDescriptor(BLEUUID((uint16_t)0x2902));



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_available = false;

int waitingDots = 0;

void errorState() {
  while (1) {
    display.clearDisplay();
    delay(1000);
    display.fillRect(10, 10, 50, 30, WHITE);
    display.display();
    delay(1000);
  }
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


// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdataBLE(UBX_NAV_PVT_data_t *ubxDataStruct) {
  Serial.println();


  pCharacteristic->setValue("Hello World says Neil");


  pCharacteristic->setValue(F("Time: "));  // Print the time
  uint8_t hms = ubxDataStruct->hour;       // Print the hours
  if (hms < 10) Serial.print(F("0"));      // Print a leading zero if required
  pCharacteristic->setValue((uint8_t *)&hms);
  pCharacteristic->setValue(F(":"));
  hms = ubxDataStruct->min;  // Print the minutes
  char text;
  sprintf(text, "%x", hms);

  if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
  pCharacteristic->setValue(text);
  pCharacteristic->setValue(F(":"));
  hms = ubxDataStruct->sec;                         // Print the seconds
  if (hms < 10) pCharacteristic->setValue(F("0"));  // Print a leading zero if required
  pCharacteristic->setValue((uint8_t *)&hms);
  pCharacteristic->setValue(F("."));
  unsigned long millisecs = ubxDataStruct->iTOW % 1000;    // Print the milliseconds
  if (millisecs < 100) pCharacteristic->setValue(F("0"));  // Print the trailing zeros correctly
  if (millisecs < 10) pCharacteristic->setValue(F("0"));
  pCharacteristic->setValue(&millisecs);

  long latitude = ubxDataStruct->lat;  // Print the latitude
  pCharacteristic->setValue(F(" Lat: "));

  long modolatitude = latitude % 10000000;
  pCharacteristic->setValue((latitude - modolatitude) / 10000000);
  pCharacteristic->setValue(F("."));
  pCharacteristic->setValue(&modolatitude);

  long longitude = ubxDataStruct->lon;  // Print the longitude
  pCharacteristic->setValue(F(" Long: "));
  long modolulongitude = longitude % 10000000;
  pCharacteristic->setValue((longitude - modolulongitude) / 10000000);
  pCharacteristic->setValue(F("."));
  pCharacteristic->setValue(&modolulongitude);
  pCharacteristic->setValue(F(" (degrees * 10^-7)"));

  long altitude = ubxDataStruct->hMSL;  // Print the height above mean sea level
  pCharacteristic->setValue(F(" Height above MSL: "));
  pCharacteristic->setValue(&altitude);
  pCharacteristic->setValue(F(" (mm)"));
  pCharacteristic->setValue("\n");

  pCharacteristic->notify();
}

void updateDisplay(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (display_available == false) { return; }
  waitingDots++;
  if (waitingDots > 3) waitingDots = 0;
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);

  if (ubxDataStruct->fixType == 0) {
    display.print("Waiting for fix");
    for (int i = 0; i <= waitingDots; i++) {
      display.print(F("."));
    }
    display.println();
    display.print(F("NumSats:"));
    display.print(ubxDataStruct->numSV);
    display.println();
  }

  if (ubxDataStruct->fixType > 0) {
    display.println("Aquired fix. Writing Data");
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

  sprintf(gpxFilename, "/%4d-%02d-%02d.gpx", ubxDataStruct->year, ubxDataStruct->month, ubxDataStruct->day);
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

void setupBluetooth() {
  BLEDevice::init("****+++blevir");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic =
    pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pService->addCharacteristic(&hourCharactaristic);

  pCharacteristic->setValue("Hello! Welcome to the gps logger.");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");

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

  myGNSS.setNavigationFrequency(1);  //Produce one navigation solution per second

  // myGNSS.setAutoPVTcallbackPtr(&printPVTdata);  // Enable automatic NAV PVT messages with callback to printPVTdata
  myGNSS.setAutoPVTcallbackPtr(&writeGPX);

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
