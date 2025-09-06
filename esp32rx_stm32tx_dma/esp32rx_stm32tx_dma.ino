#define RXD2 16   // STM32 TX -> ESP32 RX
#define TXD2 17   // STM32 RX -> ESP32 TX (optional)

void setup() {
  Serial.begin(115200);                               // Serial Monitor
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);      // UART2 for STM32
  Serial.println("ESP32 UART Filtering Ready...");
}

void loop() {
  // Check if STM32 sent something
  while (Serial2.available()) {
    char c = Serial2.read();

    // Filter out garbage (non-printable bootloader characters)
    if (c >= 32 && c <= 126) {
      Serial.print(c);   // Valid printable ASCII → display it
    }
    // Optionally, handle newline/carriage return
    else if (c == '\r' || c == '\n') {
      Serial.print(c);
    }
    // Otherwise: ignore garbage bytes silently
  }
}
