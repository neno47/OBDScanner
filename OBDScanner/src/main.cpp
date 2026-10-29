#include <Arduino.h>
#include "driver/twai.h"

// ---------------------------------------------------------------------------
// OBDScanner: OBD-II über CAN (ISO 15765-4, 500 kbit/s, 11-Bit-IDs)
// ESP32 + SN65HVD230 (D -> CAN_TX_PIN, R -> CAN_RX_PIN, RS -> GND)
// ---------------------------------------------------------------------------

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22

// 1 = nur mithören (es wird nichts gesendet), alle Frames werden ausgegeben
// 0 = OBD-Anfragen (Mode 01) senden und Antworten dekodieren
#define LISTEN_ONLY 1

static const uint32_t OBD_REQUEST_ID  = 0x7DF;  // Broadcast-Anfrage
static const uint32_t OBD_RESPONSE_ID = 0x7E8;  // Motorsteuergerät

struct PidInfo {
  uint8_t id;
  const char *name;
  const char *unit;
};

static const PidInfo PIDS[] = {
    {0x0C, "Drehzahl", "U/min"},
    {0x0D, "Geschwindigkeit", "km/h"},
    {0x05, "Kuehlmitteltemp.", "C"},
    {0x0F, "Ansauglufttemp.", "C"},
    {0x04, "Motorlast", "%"},
    {0x0B, "Ladedruck (MAP abs.)", "kPa"},
    {0x10, "Luftmasse (MAF)", "g/s"},
};

// ---------------------------------------------------------------------------
// TWAI-Treiber starten
// ---------------------------------------------------------------------------
static bool canBegin() {
#if LISTEN_ONLY
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#else
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_filter_config_t f;
  f.acceptance_code = (uint32_t)OBD_RESPONSE_ID << 21;
  f.acceptance_mask = ~((uint32_t)0x7FF << 21);
  f.single_filter = true;
#endif
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("FEHLER: twai_driver_install");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("FEHLER: twai_start");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Bus-Off automatisch beheben
// ---------------------------------------------------------------------------
static void canCheckBus() {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;
  if (s.state == TWAI_STATE_BUS_OFF) {
    Serial.println("Bus-Off, starte Recovery");
    twai_initiate_recovery();
  } else if (s.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

// ---------------------------------------------------------------------------
// OBD-Anfrage senden / Antwort abwarten
// ---------------------------------------------------------------------------
static bool sendRequest(uint8_t pid) {
  twai_message_t m = {};
  m.identifier = OBD_REQUEST_ID;
  m.data_length_code = 8;
  m.data[0] = 0x02;  // Anzahl der folgenden Nutzbytes
  m.data[1] = 0x01;  // Mode 01: aktuelle Daten
  m.data[2] = pid;
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

// Wartet bis zu timeoutMs auf die Antwort zur angefragten PID.
static bool waitResponse(uint8_t pid, twai_message_t &out, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    if (rx.identifier == OBD_RESPONSE_ID && rx.data[1] == 0x41 && rx.data[2] == pid) {
      out = rx;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Umrechnung der Rohwerte (Formeln nach SAE J1979)
// ---------------------------------------------------------------------------
static bool decodePid(uint8_t pid, const uint8_t *d, float &v) {
  const float A = d[3];
  const float B = d[4];
  switch (pid) {
    case 0x04: v = A * 100.0f / 255.0f; return true;
    case 0x05: v = A - 40.0f; return true;
    case 0x0B: v = A; return true;
    case 0x0C: v = (256.0f * A + B) / 4.0f; return true;
    case 0x0D: v = A; return true;
    case 0x0F: v = A - 40.0f; return true;
    case 0x10: v = (256.0f * A + B) / 100.0f; return true;
    default: return false;
  }
}

// ---------------------------------------------------------------------------
// Unterstützte PIDs abfragen (PID 0x00, 0x20, 0x40, ...)
// ---------------------------------------------------------------------------
static void printSupportedPids(uint8_t base) {
  if (!sendRequest(base)) {
    Serial.printf("Senden von PID 0x%02X fehlgeschlagen\n", base);
    return;
  }
  twai_message_t rx;
  if (!waitResponse(base, rx, 300)) {
    Serial.printf("Keine Antwort auf PID 0x%02X\n", base);
    return;
  }
  uint32_t mask = ((uint32_t)rx.data[3] << 24) | ((uint32_t)rx.data[4] << 16) |
                  ((uint32_t)rx.data[5] << 8) | rx.data[6];
  Serial.printf("Unterstuetzt ab 0x%02X:", base + 1);
  for (int i = 0; i < 32; i++) {
    if (mask & (1UL << (31 - i))) Serial.printf(" %02X", base + 1 + i);
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nOBDScanner startet");

  if (!canBegin()) {
    while (true) delay(1000);
  }

#if LISTEN_ONLY
  Serial.println("Modus: nur mithoeren (LISTEN_ONLY)");
#else
  Serial.println("Modus: OBD-Abfragen");
  printSupportedPids(0x00);
  printSupportedPids(0x20);
  printSupportedPids(0x40);
#endif
}

void loop() {
  canCheckBus();

#if LISTEN_ONLY
  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("ID 0x%03X [%d]:", (unsigned)rx.identifier, rx.data_length_code);
    for (int i = 0; i < rx.data_length_code; i++) Serial.printf(" %02X", rx.data[i]);
    Serial.println();
  }
#else
  for (const PidInfo &p : PIDS) {
    if (!sendRequest(p.id)) {
      Serial.printf("%-22s Senden fehlgeschlagen\n", p.name);
      continue;
    }
    twai_message_t rx;
    float v;
    if (waitResponse(p.id, rx, 200) && decodePid(p.id, rx.data, v)) {
      Serial.printf("%-22s %8.1f %s\n", p.name, v, p.unit);
    } else {
      Serial.printf("%-22s keine Antwort\n", p.name);
    }
    delay(80);  // kleine Pause zwischen den Anfragen
  }
  Serial.println("---");
  delay(500);
#endif
}