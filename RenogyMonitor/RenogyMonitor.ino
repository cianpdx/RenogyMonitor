/**
  A BLE reader for Renogy Batteries

  Runs on Heltec type ESP + OLED as simple bettery monitor

  I may add back in wifi/mqtt updates but not needed in my application. refer to distancerunner.
  I only have one battery so removed the indexing

  based on excellent work from:
  https://github.com/chadj/renogy-smart-battery
  https://github.com/distancerunner/Renogy-Battery-ESP32-Bluetooth-MQTT-Bridge

  Heltec WifiKit V3
  NimBLE v2
*/





#include <NimBLEDevice.h>

#include "Arduino.h"
#include "heltec.h"

#define RENOGYHEADERSIZE 3 // drop first 3 bytes of response

// The remote service, we wish to connect to.
static BLEUUID serviceWriteUUID("0000ffd0-0000-1000-8000-00805f9b34fb"); // WRITE
static BLEUUID serviceReadUUID("0000fff0-0000-1000-8000-00805f9b34fb"); // READ

static BLEUUID WRITE_UUID("0000ffd1-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

//battery declarations

String callData = "getLevels";
String responseData = "";
String RENOGYpower = "";
String RENOGYcurrent = "";
String RENOGYvoltage = "";
String RENOGYcurrentDebug = "0";
String RENOGYvoltageDebug = "0";
String RENOGYchargeLevel = "";
String RENOGYcapacity = "";
String RENOGYtemperature = "";
String RENOGYtimer = "00:00";
String wifiSSIDValue = "noSSID";
String actualTimeStamp = "00:00:00";


uint16_t timerCounterStart = 0;
uint16_t timerCounterActual = 0;
boolean timerIsRunning = false;

//GUI declarations
int current_frame = 0;
int desired_frame = 0;
long last_button_time;
int button_interval = 250;
int read_interval = 500;
int read_timeout = 1000;
int last_read_time = 0;
int screen_interval = 30000;
int num_frames = 3;
int comm_timeout = 500;
int last_comm_time = 0;
long counter = 0;

float output_voltage;
float output_volts[6]; //cell 1-4,total,max difference
float output_temps[5]; //cell 1-4,pcb
float output_amps;
int output_SOC;

byte commands[3][8] = {
  {0x30, 0x03, 0x13, 0xB2, 0x00, 0x06, 0x65, 0x4A}, // Levels
  {0x30, 0x03, 0x13, 0x88, 0x00, 0x11, 0x05, 0x49}, // Cell volts
  {0x30, 0x03, 0x13, 0x99, 0x00, 0x05, 0x55, 0x43}, // Temperatures
};
static boolean tryReconnect = false;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static boolean sent = false;
static uint32_t timerTickerForWhatchDog = millis();
static uint32_t timerTickerForEggTimer = millis();
static uint32_t timerTicker2 = millis();

BLERemoteService* pRemoteWriteService;
BLERemoteService* pRemoteReadService;
BLERemoteCharacteristic* pRemoteWriteCharacteristic;
BLERemoteCharacteristic* pRemoteNotifyCharacteristic;
// BLEAdvertisedDevice* myDevice;

BLEClient* pClient;
// BLEScan* pBLEScan;

#define DEVICEAMOUNT 1

// Address of my BT battery devices
static const char* deviceAddresses[DEVICEAMOUNT] = {"60:98:66:d9:7c:58"};
static float current[DEVICEAMOUNT] = {0};
static double voltage = 0.0;
static int16_t power[DEVICEAMOUNT] = {0};
uint8_t deviceAddressesNumber = 0;



static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {



  uint32_t tempvalueI;

  if (responseData == "getLevels") {
    // we get Current as signed 2 bytes
    int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE + 0] << 8) | pData[RENOGYHEADERSIZE + 1];
    output_amps = valueSigned * 0.01;

    // we get voltage as uint 2 bytes
    tempvalueI = ((int16_t)pData[RENOGYHEADERSIZE + 2] << 8) | pData[RENOGYHEADERSIZE + 3];
    output_voltage = tempvalueI * 0.1;

    // we get lavel as uint 4 bytes
    tempvalueI = ((uint8_t)pData[RENOGYHEADERSIZE + 4] << 24) | ((uint8_t)pData[RENOGYHEADERSIZE + 5] << 16) | ((uint8_t)pData[RENOGYHEADERSIZE + 6] << 8) | (uint8_t)pData[RENOGYHEADERSIZE + 7];
    output_SOC = tempvalueI * 0.001;
    callData = "getLevels";
  }
  sent = false;

}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      //Serial.println("onDisconnect");
    }
};


bool connectToServer() {
  BLEDevice::init("client");
  //Serial.print("Forming a connection to ");
  //   Serial.println(myDevice->getAddress().toString().c_str());
  //Serial.println(deviceAddresses[deviceAddressesNumber]);

  BLEDevice::setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  //pClient = NimBLEDevice::createClient(NimBLEAddress(deviceAddresses[deviceAddressesNumber]));
  pClient = NimBLEDevice::createClient(NimBLEAddress("60:98:66:d9:7c:58"));
  //Serial.println(" - Created client");
  // delay(700);
  pClient->setClientCallbacks(new MyClientCallback());
  // delay(700);
  // Connect to the remove BLE Server.
  pClient->connect();  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  //Serial.println(" - Connected to server");
  //delay(700);  //this one most recent delay
  // pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  pRemoteWriteService = pClient->getService(serviceWriteUUID);
  // if (true) {
  if (pRemoteWriteService == nullptr) {
    //Serial.print("Failed to find our service UUID: ");
    //Serial.println(serviceWriteUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  //Serial.println(" - Found our pRemoteWriteService");
  pRemoteReadService = pClient->getService(serviceReadUUID);
  if (pRemoteReadService == nullptr) {
    //Serial.print("Failed to find our service UUID: ");
    //Serial.println(serviceReadUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  //Serial.println(" - Found our pRemoteReadService");
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteWriteCharacteristic = pRemoteWriteService->getCharacteristic(WRITE_UUID);
  if (pRemoteWriteCharacteristic == nullptr) {
    //Serial.print(F("Failed to find our characteristic UUID: "));
    //Serial.println(WRITE_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  //Serial.println(F(" - Found our Write characteristic"));
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteNotifyCharacteristic = pRemoteReadService->getCharacteristic(NOTIFY_UUID);
  if (pRemoteNotifyCharacteristic == nullptr) {
    //Serial.print(F("Failed to find our characteristic UUID: "));
    //Serial.println(NOTIFY_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  //Serial.println(F(" - Found our characteristic for notifications"));

  if (pRemoteNotifyCharacteristic->canNotify()) {
    //Serial.println("Subscribe to characteristic...");
    pRemoteNotifyCharacteristic->registerForNotify(notifyCallback);
  }

  // BLEDevice::getScan()->clearResults();
  connected = true;
  //flexiblePollingSpeed = 6000; // next call for data in 2s
  return true;
}




void setup() {
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  //drawFrame0();
  pinMode(0, INPUT_PULLUP);
  Serial.begin(115200);
  //Serial.println("Starting Arduino BLE Client application...");
  connectToServer();
  callData = "getLevels";
  Heltec.display->flipScreenVertically();
} // End of setup.




// This is the Arduino main loop function.
void loop() {
  //default to SOC screen after 30 seconds
  if (millis() - last_button_time > screen_interval) {
    desired_frame = 0;
  }


  read_button();
  draw_ui();

  if (millis() - last_read_time > read_interval && !sent) {
    last_read_time = millis();
    Serial.println(counter);
    sent = 1;
    counter++;
    get_battery_data();

  }


  if (millis() - last_read_time > read_timeout && sent) {

    sent = 0;
    counter = 0;
    NimBLEDevice::deleteClient(pClient);
    connectToServer();
    callData = "getLevels";
  }


  // End of loop
}

void get_battery_data() {
  if (callData == "getLevels") {
    last_comm_time = millis();
    responseData = "getLevels";
    callData = "";
    //Serial.println("Request Level and Voltage Information: ");
    pRemoteWriteCharacteristic->writeValue(commands[0], sizeof(commands[0]));

  }
}

void read_button() {
  if (!digitalRead(0) && (millis() - last_button_time > button_interval) ) {
    last_button_time = millis();
    desired_frame++;
    if (desired_frame >= num_frames) {
      desired_frame = 0;
    }
  }
}

void draw_ui() {

  switch (desired_frame) {
    case 0:
      drawFrame0();
      break;
    case 1:
      drawFrame1();
      break;
    case 2:
      drawFrame2();
      break;
    case 3:
      drawFrame3();
      break;
  }
}


void drawFrame0() {
  //int battery_soc = 87;
  Heltec.display->clear();
  //Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  Heltec.display->drawString(64, 0, String(output_SOC) + "%");
  Heltec.display->drawProgressBar(15, 40, 98, 10, output_SOC);
  Heltec.display->display();
}

void drawFrame1() {
  //float battery_volt = 13.34235;
  Heltec.display->clear();
  char str[6];
  dtostrf(output_voltage, 5, 1, str);
  //Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  //Heltec.display->drawString(64, 0, String(float(int(output_voltage * 10)) / 10));
  Heltec.display->drawString(64, 0, String(str));
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(64, 35, "Volts" );
  //Heltec.display->drawProgressBar(15, 40, 98, 10, battery_volt);
  Heltec.display->display();
}
void drawFrame2() {
  //float battery_amp = -33.65423;
  char str[6];
  dtostrf(output_amps, 5, 1, str);

  Heltec.display->clear();
  //Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  Heltec.display->drawString(64, 0, String(str));
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(64, 35, "Amps" );
  //Heltec.display->drawProgressBar(15, 40, 98, 10, battery_amp);
  Heltec.display->display();
}

void drawFrame3() {
  //float battery_amp = 3.654;
  Heltec.display->clear();
  //Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  Heltec.display->drawString(15, 0, "Cell 1");
  Heltec.display->drawString(15, 10, "Cell 2");
  Heltec.display->drawString(15, 20, "Cell 3");
  Heltec.display->drawString(15, 30, "Cell 4");
  Heltec.display->drawString(15, 40, "PCB");

  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);


  Heltec.display->drawString(50, 0, String(float(int(output_temps[0] * 100)) / 100));
  Heltec.display->drawString(50, 10, String(float(int(output_temps[1] * 100)) / 100));
  Heltec.display->drawString(50, 20, String(float(int(output_temps[2] * 100)) / 100));
  Heltec.display->drawString(50, 30, String(float(int(output_temps[3] * 100)) / 100));
  Heltec.display->drawString(50, 40, String(float(int(output_temps[4] * 100)) / 100));

  Heltec.display->drawString(100, 0, String(float(int(output_volts[0] * 100)) / 100));
  Heltec.display->drawString(100, 10, String(float(int(output_volts[1] * 100)) / 100));
  Heltec.display->drawString(100, 20, String(float(int(output_volts[2] * 100)) / 100));
  Heltec.display->drawString(100, 30, String(float(int(output_volts[3] * 100)) / 100));
  Heltec.display->drawString(100, 40, String(int(output_volts[5])));



  Heltec.display->display();
}
