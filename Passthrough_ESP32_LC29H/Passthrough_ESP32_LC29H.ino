#define RX_PIN 16
#define TX_PIN 17

HardwareSerial rtkSerial(2);

// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  Serial.begin(460800);
  rtkSerial.begin(460800, SERIAL_8N1, RX_PIN, TX_PIN); // RX, TX

  // Create task to handle Serial -> rtkSerial on Core 0
  xTaskCreatePinnedToCore(
    SerialToRTK,      // Function to implement the task
    "SerialToRTK",    // Name of the task
    2048,             // Stack size
    NULL,             // Task input parameter
    1,                // Priority
    &Task1,           // Task handle
    0);               // Core 0

  // Create task to handle rtkSerial -> Serial on Core 1
  xTaskCreatePinnedToCore(
    RTKToSerial,
    "RTKToSerial",
    2048,
    NULL,
    1,
    &Task2,
    1);               // Core 1
}

void loop() {
  // Nothing to do here; tasks handle everything
}

void SerialToRTK(void * parameter) {
  for (;;) {
    while (Serial.available()) {
      rtkSerial.write(Serial.read());
    }
  }
  vTaskDelay(1); // Short delay to allow context switching
}

void RTKToSerial(void * parameter) {
  for (;;) {
    while (rtkSerial.available()) {
      Serial.write(rtkSerial.read());
    }
  }
  vTaskDelay(1); // Short delay to allow context switching
}
