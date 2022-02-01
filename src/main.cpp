#include <Arduino.h>
#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library
#include <BME280I2C.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
#include <OLEDDisplayUi.h>
#include <TimeLib.h>


CCS811 *ccs811;
BME280I2C *bme;

BLEServer *bleServer;
BLEService *bleService;
BLEAdvertising *bleAdvertising;

BLECharacteristic *bleChTemperature;
BLECharacteristic *bleChHumidity;
BLECharacteristic *bleChPressure;
BLECharacteristic *bleChEco2;
BLECharacteristic *bleChTVoc;

float valueTemperature = 10.1;
float valueHumidity = 10.1;
float valuePressure = 10.1;
float valueEvoc = 10.1;
float valueEco2 = 10.1;

unsigned long lastAirUpdate = 0;

#define DEVINFO_SERVICE_UUID        uint16_t (0x180a)
#define ENV_SERVICE_UUID        uint16_t (0x181a)

#define CHAR_TEMPERATURE uint16_t(0x2a6e)
#define CHAR_HUMIDITY uint16_t(0x2a6f)
#define CHAR_PRESSURE uint16_t(0x2a6d)

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SH1106Wire display(0x3c, SDA, SCL);
OLEDDisplayUi ui ( &display );

void stdFrame (OLEDDisplay *display, OLEDDisplayUiState* state);
void frameTemp(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void frameHum(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void framePres(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void frameCO2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void frameTVOC(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);

FrameCallback frames[] = { frameTemp, frameHum, framePres, frameCO2, frameTVOC };

// how many frames are there?
int frameCount = 5;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { stdFrame };
int overlaysCount = 1;

int screenW = 128;
int screenH = 64;
int centerX = screenW / 2;
int centerY = ((screenH - 16) / 2) - 20; // top yellow part is 16 px height

int sY = ((screenH - 16) / 2) + 9;
int tY = ((screenH - 16) / 2) + 20;

void setupIO()
{
    Wire.begin(21, 22);

}

void setupUi()
{
    // The ESP is capable of rendering 60fps in 80Mhz mode
    // but that won't give you much time for anything else
    // run it in 160Mhz mode or just set it to 30 fps
    ui.setTargetFPS(10);

    // Add frames
    ui.setFrames(frames, frameCount);

    // Add overlays
    ui.setOverlays(overlays, overlaysCount);

    ui.disableAllIndicators();

    // Initialising the UI will init the display too.
    ui.init();

    display.flipScreenVertically();
}

void setupAirQ()
{
    ccs811 = new CCS811(23, CCS811_SLAVEADDR_1); // nWAKE on 23
    
    // Enable CCS811
    ccs811->set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
    bool ok= ccs811->begin();
    if( !ok ) 
        Serial.println("setup: CCS811 begin FAILED");

    // Print CCS811 versions
    Serial.print("setup: hardware    version: "); Serial.println(ccs811->hardware_version(),HEX);
    Serial.print("setup: bootloader  version: "); Serial.println(ccs811->bootloader_version(),HEX);
    Serial.print("setup: application version: "); Serial.println(ccs811->application_version(),HEX);

    // Start measuring
    ok= ccs811->start(CCS811_MODE_1SEC);
    if( !ok ) Serial.println("setup: CCS811 start FAILED");
}

void setupTemp()
{
    BME280I2C::Settings settings;
    settings.bme280Addr = BME280I2C::I2CAddr_0x77;
    bme = new BME280I2C(settings);

    while(!bme->begin())
    {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch(bme->chipModel())
    {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor! Success.");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }
}

void setupSerial()
{
    // Enable serial
    Serial.begin(115200);
    Serial.println("");
    Serial.println("setup: Starting CCS811 basic demo");
    Serial.print("setup: ccs811 lib  version: "); Serial.println(CCS811_VERSION);
}

void setupBLE()
{
    BLEDevice::init("Ambient");
    bleServer = BLEDevice::createServer();
    bleService = bleServer->createService(BLEUUID{ENV_SERVICE_UUID});
    bleChTemperature = bleService->createCharacteristic(
            CHAR_TEMPERATURE,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE
    );
    bleChHumidity = bleService->createCharacteristic(
            CHAR_HUMIDITY,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE
    );
    bleChPressure = bleService->createCharacteristic(
            CHAR_PRESSURE,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE
    );

    bleChTemperature->setValue(valueTemperature);
    bleChHumidity->setValue(valueHumidity);
    bleChPressure->setValue(valuePressure);
    bleService->start();

    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(BLEUUID{ENV_SERVICE_UUID});
    bleAdvertising->setScanResponse(true);
    bleAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    bleAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void setup()
{
    setupSerial();
    setupIO();
    setupUi();
    setupAirQ();
    setupTemp();
    setupBLE();
}

void readAirQ()
{
    // Read
    uint16_t eco2, etvoc, errstat, raw;
    ccs811->read(&eco2,&etvoc,&errstat,&raw);

    // Print measurement results based on status
    if( errstat==CCS811_ERRSTAT_OK ) {
        valueEco2 = eco2;
        valueEvoc = etvoc;

        Serial.print("CCS811: ");
        Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
        Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
        //Serial.print("raw6=");  Serial.print(raw/1024); Serial.print(" uA  ");
        //Serial.print("raw10="); Serial.print(raw%1024); Serial.print(" ADC  ");
        //Serial.print("R="); Serial.print((1650*1000L/1023)*(raw%1024)/(raw/1024)); Serial.print(" ohm");
        Serial.println();
    } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
        Serial.println("CCS811: waiting for (new) data");
    } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) {
        Serial.println("CCS811: I2C error");
    } else {
        Serial.print("CCS811: errstat="); Serial.print(errstat,HEX);
        Serial.print("="); Serial.println( ccs811->errstat_str(errstat) );
    }
}

void readTemp()
{
    float temp(NAN), hum(NAN), pres(NAN);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    bme->read(pres, temp, hum, tempUnit, presUnit);

    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
    Serial.print("\t\tHumidity: ");
    Serial.print(hum);
    Serial.print("% RH");
    Serial.print("\t\tPressure: ");
    Serial.print(pres);
    Serial.println("Pa");

    valueTemperature = temp;
    bleChTemperature->indicate();
    valueHumidity = hum;
    bleChHumidity->indicate();
    valuePressure = pres;
    bleChPressure->indicate();
}

void loop()
{
    unsigned long now = millis();
    if (now - lastAirUpdate > 5000) {
        lastAirUpdate = now;
        readAirQ();
        readTemp();
    }

    int remainingTimeBudget = ui.update();

    if (remainingTimeBudget > 0) {
        // You can do some work here
        // Don't do stuff if you are below your
        // time budget.
        delay(remainingTimeBudget);
    }
}

// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
    if (digits < 10) {
        String i = '0' + String(digits);
        return i;
    }
    else {
        return String(digits);
    }
}

void stdFrame(OLEDDisplay *display, OLEDDisplayUiState* state)
{
    String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);

    display->drawString(centerX , centerY, timenow );
}

void frameTemp(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    String tmp = String(floor(valueTemperature * 10 + 0.5) / 10, 1) + "°C";

    display->setFont(ArialMT_Plain_10);
    display->drawString(centerX+x, sY+y, "Temp");

    display->setFont(ArialMT_Plain_16);
    display->drawString(centerX+x, tY+y, tmp);
}
void frameHum(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    String hum = String((int)floor(valueHumidity)) + "%";

    display->setFont(ArialMT_Plain_10);
    display->drawString(centerX+x, sY+y, "Rel Hum");

    display->setFont(ArialMT_Plain_16);
    display->drawString(centerX+x, tY+y, hum);
}
void framePres(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    String pre = String((int)floor(valuePressure/100)) + "hPa";

    display->setFont(ArialMT_Plain_10);
    display->drawString(centerX+x, sY+y, "Pres");

    display->setFont(ArialMT_Plain_16);
    display->drawString(centerX+x, tY+y, pre);
}

void frameCO2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    String pre = String((int)floor(valueEco2)) + "ppm";

    display->setFont(ArialMT_Plain_10);
    display->drawString(centerX+x, sY+y, "CO2");

    display->setFont(ArialMT_Plain_16);
    display->drawString(centerX+x, tY+y, pre);
}
void frameTVOC(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    String pre = String((int) floor(valueEvoc)) + "ppm";

    display->setFont(ArialMT_Plain_10);
    display->drawString(centerX+x, sY+y, "TVOC");

    display->setFont(ArialMT_Plain_16);
    display->drawString(centerX+x, tY+y, pre);
}
