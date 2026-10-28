/*
   BLE test for the ESP32-S3 board.
   Advertises as "ESP32-UWB" using the Nordic UART Service (NUS) so any
   BLE terminal app on a phone (nRF Connect, Bluefruit Connect, LightBlue)
   can connect, receive text notifications, and send text back.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//BLE Globals
#define BLE_DEVICE_NAME "ESP32-UWB"
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // phone -> ESP32
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // ESP32 -> phone

#define SEND_INTERVAL_MS 1000

BLEServer *bleServer = NULL;
BLECharacteristic *txCharacteristic = NULL;
volatile boolean deviceConnected = false;
boolean wasConnected = false;
unsigned long lastSendTime = 0;
unsigned int msgCount = 0;
char rxBuffer[20];
uint8_t rxRaw[20];
size_t rxLen = 0;
volatile boolean rxPending = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *server) {
    deviceConnected = false;
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    auto rxValue = characteristic->getValue();
    if (rxValue.length() > 0) {
      //Hand off to loop() so the BLE callback stays short
      rxLen = min((size_t)rxValue.length(), sizeof(rxRaw));
      memcpy(rxRaw, rxValue.c_str(), rxLen);
      strlcpy(rxBuffer, rxValue.c_str(), sizeof(rxBuffer));
      rxPending = true;
    }
  }
};

//Send a line of text to the phone. Keep it under 20 bytes unless the
//phone negotiates a larger MTU, otherwise it gets truncated.
void bleSend(const char *msg) {
  if (!deviceConnected) {
    return;
  }
  txCharacteristic->setValue((uint8_t *)msg, strlen(msg));
  txCharacteristic->notify();
}

void setup() {
  Serial.begin(115200);
  //Don't block forever if no USB serial monitor is open
  while (!Serial && millis() < 3000)
  {
    delay(10);
  }
  Serial.println(F("-----------------------------"));
  Serial.println(F("-->       BLE Test        <--"));
  Serial.println(F("-----------------------------"));

  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *service = bleServer->createService(NUS_SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as " BLE_DEVICE_NAME ", waiting for phone...");
}

void loop() {
  //Print what the phone wrote and echo it back so it shows up on the phone too
  if (rxPending) {
    rxPending = false;
    Serial.print("From phone: ");
    Serial.print(rxBuffer);
    //Raw bytes, to tell text from hex writes
    Serial.printf("  [%u bytes:", rxLen);
    for (size_t i = 0; i < rxLen; i++) {
      Serial.printf(" %02X", rxRaw[i]);
    }
    Serial.println("]");
    char echo[20];
    snprintf(echo, sizeof(echo), "got:%s\n", rxBuffer);
    bleSend(echo);
  }

  //Periodically send a test message while a phone is connected
  if (deviceConnected && millis() - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = millis();
    msgCount++;
    char buf[20];
    snprintf(buf, sizeof(buf), "msg %u t=%lus\n", msgCount, millis() / 1000);
    bleSend(buf);
    Serial.print("Sent: ");
    Serial.print(buf);
  }

  //Connection state changes
  if (deviceConnected && !wasConnected) {
    wasConnected = true;
    Serial.println("Phone connected");
  }
  if (!deviceConnected && wasConnected) {
    wasConnected = false;
    Serial.println("Phone disconnected, advertising again");
    delay(500); //give the BLE stack time to clean up
    BLEDevice::startAdvertising();
  }
}
