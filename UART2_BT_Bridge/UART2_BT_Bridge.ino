#include <BLEDevice.h>         // BLE Bluetooth
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>

// UART configuration (change as needed)
#define RX1_PIN 16   // RX pin for UART2
#define TX1_PIN 17   // TX pin for UART2

// BLE device name for iOS
const char* bleName = "ESP32_BT_BLE"; 

// LED pin
const int ledPin = 2;  // Adjust if the LED is on a different pin

// BLE UUIDs for the Nordic UART Service (NUS)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Watchdog timeout in seconds
#define WDT_TIMEOUT 60

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool bleDeviceConnected = false;
bool oldBleDeviceConnected = false;

unsigned long lastUartDataTime = 0;
const unsigned long uartTimeout = 5000; // 5 seconds timeout for UART
unsigned long uartBaudrate = 460800;

void blinkLED(uint32_t millis)
{
    digitalWrite(ledPin, HIGH);
    delay(millis);
    digitalWrite(ledPin, LOW);
}

// Callback class for BLE
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      bleDeviceConnected = true;
      Serial.println("BLE device connected.");
    };

    void onDisconnect(BLEServer* pServer) {
      bleDeviceConnected = false;
      Serial.println("BLE device disconnected.");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial1.write((uint8_t*)value.c_str(), value.length());
            blinkLED(1);
        }
    }
};

// Function for the UART task
void uartTask(void *pvParameters) {
  while (1) {
    while (Serial1.available()) {
      size_t len = Serial1.available();
      uint8_t buffer[len];
      Serial1.readBytes(buffer, len);
      
      if (bleDeviceConnected) {
        pTxCharacteristic->setValue(buffer, len);
        pTxCharacteristic->notify(); // Send data to connected iOS device
        blinkLED(1);
      } else {
        Serial.println("No BLE device connected. Data not sent.");
        blinkLED(1000);
      }
      lastUartDataTime = millis();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial1.begin(uartBaudrate, SERIAL_8N1, RX1_PIN, TX1_PIN);

  // BLE initialization for iOS devices
  BLEDevice::init(bleName); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
  Serial.println("BLE started and advertising sent.");

  // Start UART task
  xTaskCreatePinnedToCore(uartTask, "UART Task", 8192, NULL, 1, NULL, 1);
}

void loop() {
  if (bleDeviceConnected && !oldBleDeviceConnected) {
    oldBleDeviceConnected = bleDeviceConnected;
  }

  if (!bleDeviceConnected && oldBleDeviceConnected) {
    oldBleDeviceConnected = bleDeviceConnected;
    pServer->startAdvertising();
    Serial.println("Advertising restarted.");
  }

  esp_task_wdt_reset();  // Watchdog reset
  delay(10);
}
