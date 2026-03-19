#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ===== Wi-Fi configuration =====
const char* ssid = "JAXA_Hybrid_OS";
const char* password = "mext_flight_ctrl";

WiFiUDP udp;
const int udpPort = 14550; // Standard MAVLink port for telemetry
IPAddress broadcastIP(192, 168, 4, 255); // Send to all connected devices
IPAddress telemetryTargetIP(0, 0, 0, 0);
uint16_t telemetryTargetPort = udpPort;

// ===== STM32 UART Configuration =====
HardwareSerial STM32Serial(1); // Use UART1 for communication with STM32
const int RX_PIN = 7;
const int TX_PIN = 6;
const int BAUD_RATE = 115200;

// ===== RTOS / PERFORMANCE SETTINGS =====
static constexpr size_t UART_FRAME_MAX = 256;
static constexpr TickType_t UART_FLUSH_TICKS = pdMS_TO_TICKS(15);
static constexpr TickType_t IDLE_DELAY_TICKS = pdMS_TO_TICKS(1);

SemaphoreHandle_t udpMutex = nullptr;

// Sends a frame of data over UDP to the current telemetry target (or broadcast if no target is set).
static void sendUdpFrame(const uint8_t* data, size_t len) {
  if (len == 0 || data == nullptr) {
    return;
  }

  if (xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    const bool haveClient = (telemetryTargetIP != IPAddress(0, 0, 0, 0));
    const IPAddress targetIP = haveClient ? telemetryTargetIP : broadcastIP;
    const uint16_t targetPort = haveClient ? telemetryTargetPort : static_cast<uint16_t>(udpPort);

    udp.beginPacket(targetIP, targetPort);
    udp.write(data, len);
    udp.endPacket();
    xSemaphoreGive(udpMutex);
  }
}

void uartToUdpTask(void* /*pvParameters*/) {
  uint8_t frame[UART_FRAME_MAX];
  size_t frameLen = 0;
  TickType_t lastByteTick = xTaskGetTickCount();

  for (;;) {
    bool hadData = false;
    while (STM32Serial.available() > 0) {
      int b = STM32Serial.read();
      if (b < 0) {
        break;
      }

      hadData = true;
      frame[frameLen++] = static_cast<uint8_t>(b);
      lastByteTick = xTaskGetTickCount();

      const bool frameFull = (frameLen >= UART_FRAME_MAX);
      const bool lineEnded = (frame[frameLen - 1] == '\n');
      if (frameFull || lineEnded) {
        sendUdpFrame(frame, frameLen);
        frameLen = 0;
      }
    }

    const TickType_t now = xTaskGetTickCount();
    if (frameLen > 0 && (now - lastByteTick) >= UART_FLUSH_TICKS) {
      sendUdpFrame(frame, frameLen);
      frameLen = 0;
    }

    if (!hadData) {
      vTaskDelay(IDLE_DELAY_TICKS);
    } else {
      taskYIELD();
    }
  }
}

void udpToUartTask(void* /*pvParameters*/) {
  uint8_t buffer[UART_FRAME_MAX];

  for (;;) {
    int len = 0;

    if (xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      int packetSize = udp.parsePacket();
      if (packetSize > 0) {
        telemetryTargetIP = udp.remoteIP();
        telemetryTargetPort = udpPort;
        len = udp.read(buffer, UART_FRAME_MAX);
      }
      xSemaphoreGive(udpMutex);
    }

    if (len > 0) {
      STM32Serial.write(buffer, (size_t)len);
      STM32Serial.flush();
      taskYIELD();
    } else {
      vTaskDelay(IDLE_DELAY_TICKS);
    }
  }
}

void setup() {
  // Set CPU frequency to 80 MHz for better performance while keeping power consumption reasonable.
  setCpuFrequencyMhz(80);

  // Initialize USB of the ESP32 for debugging (optional).
  Serial.begin(115200);

  // Initialize communication with the STM32.
  STM32Serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  STM32Serial.setTimeout(0);

  // Create the drone's WiFi network.
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("WiFi Network Created. Waiting for Altair (My laptop)...");

  // Initialize the UDP socket.
  udp.begin(udpPort);

  udpMutex = xSemaphoreCreateMutex();
  if (udpMutex == nullptr) {
    Serial.println("Error: no se pudo crear udpMutex");
    while (true) {
      delay(1000);
    }
  }

  // Task to read from UART and send to UDP.
  xTaskCreate(
    uartToUdpTask,
    "uart_to_udp",
    3072,
    nullptr,
    2,
    nullptr
  );

  // Task to read from UDP and send to UART.
  xTaskCreate(
    udpToUartTask,
    "udp_to_uart",
    3072,
    nullptr,
    2,
    nullptr
  );
}

void loop() {
  // Nothing to do here since all work is done in tasks. Just delay to prevent watchdog resets.
  vTaskDelay(pdMS_TO_TICKS(1000));
}