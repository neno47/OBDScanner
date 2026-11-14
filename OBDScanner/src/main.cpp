#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------------------------
// OBDScanner: VW-Diagnose über CAN mit TP 2.0 (KWP2000), 500 kbit/s
// ESP32 + SN65HVD230 (D -> CAN_TX_PIN, R -> CAN_RX_PIN, RS -> GND)
//
// Ablauf: Kanalaufbau -> Kanalparameter -> StartDiagnosticSession (10 89)
//         -> ECU-Kennung lesen (1A 9B) -> Messwertbloecke lesen (21 xx)
//
// Der Webserver laeuft in einer eigenen Task (Kern 0), die CAN-Abfrage in loop()
// (Kern 1). Die Werte werden ueber eine Struktur mit Mutex ausgetauscht.
//
// Es werden ausschliesslich lesende Dienste verwendet.
// ---------------------------------------------------------------------------

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22

// 1 = jede Antwort seriell ausgeben. Das bremst stark (115200 Baud ~ 10 ms pro Zeile)!
#define PRINT_BLOCKS 0

static const uint8_t ECU_ADDR = 0x01;  // logische Adresse: Motorsteuergeraet

// Messwertbloecke, die zyklisch gelesen werden
static const uint8_t BLOCKS[] = {1, 2, 3, 4};

static uint32_t txId = 0;  // ID, auf der der Tester sendet (Steuergeraet hoert zu)
static uint32_t rxId = 0;  // ID, auf der das Steuergeraet sendet
static uint8_t txSeq = 0;
static volatile bool connected = false;
static uint32_t nextConnectAt = 0;  // fruehester Zeitpunkt fuer den naechsten Verbindungsversuch

// ---------------------------------------------------------------------------
// Gemeinsame Werte (CAN-Abfrage schreibt, Webserver liest)
// ---------------------------------------------------------------------------
struct Values {
  float rpm = 0;
  float coolant = 0;
};
static Values values;
static SemaphoreHandle_t valuesMutex;

static void setValues(float rpm, float coolant) {
  xSemaphoreTake(valuesMutex, portMAX_DELAY);
  values.rpm = rpm;
  values.coolant = coolant;
  xSemaphoreGive(valuesMutex);
}

static Values getValues() {
  xSemaphoreTake(valuesMutex, portMAX_DELAY);
  Values v = values;
  xSemaphoreGive(valuesMutex);
  return v;
}

// ---------------------------------------------------------------------------
// Webserver
// ---------------------------------------------------------------------------
static WebServer server(80);

const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OBDScanner</title>
</head><body>
<h2>Drehzahl: <span id="r">-</span> U/min</h2>
<h2>Kühlmittel: <span id="c">-</span> °C</h2>
<p id="s"></p>
<script>
const POLL_MS = 0;  // Pause zwischen zwei Abfragen (0 = sofort nach der Antwort)
async function update() {
  try {
    const res = await fetch("/values", { cache: "no-store" });
    const d = await res.json();
    document.getElementById("r").textContent = d.rpm;
    document.getElementById("c").textContent = d.coolant;
    document.getElementById("s").textContent =
      d.connected ? "" : "Keine Verbindung zum Steuergerät";
  } catch (e) {
    document.getElementById("s").textContent = "ESP32 nicht erreichbar";
  }
  setTimeout(update, POLL_MS);
}
update();
</script>
</body></html>
)rawliteral";

static void setupWeb() {
  WiFi.softAP("OBDScanner", "12345678");  // eigenes WLAN
  server.on("/values", []() {
    Values v = getValues();
    String j = "{\"rpm\":" + String(v.rpm, 0) +
               ",\"coolant\":" + String(v.coolant, 1) +
               ",\"connected\":" + String(connected ? "true" : "false") + "}";
    server.send(200, "application/json", j);
  });
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.begin();
}

static void webTask(void *) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));  // gibt anderen Tasks Zeit
  }
}

// ---------------------------------------------------------------------------
// CAN-Grundfunktionen
// ---------------------------------------------------------------------------
static bool canBegin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

static bool canSend(uint32_t id, const uint8_t *d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = len;
  memcpy(m.data, d, len);
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

// Wartet auf einen Frame mit der angegebenen ID, andere Frames werden verworfen.
static bool canRecv(uint32_t id, twai_message_t &m, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (m.identifier == id) return true;
  }
  return false;
}

static void printHex(const uint8_t *d, int n) {
  for (int i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
  Serial.println();
}

// ---------------------------------------------------------------------------
// TP 2.0: Kanal auf- und abbauen
// ---------------------------------------------------------------------------
static bool sendChannelParams(uint8_t opcode) {
  // A0 = Parameteranfrage (Tester), A1 = Antwort
  // Blockgroesse 15, T1 = 100 ms, T3 = 5 ms
  const uint8_t par[6] = {opcode, 0x0F, 0x8A, 0xFF, 0x32, 0xFF};
  return canSend(txId, par, 6);
}

static bool tp20Connect() {
  twai_clear_receive_queue();

  const uint8_t req[7] = {ECU_ADDR, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01};
  if (!canSend(0x200, req, 7)) return false;

  twai_message_t r;
  if (!canRecv(0x200 + ECU_ADDR, r, 1000) || r.data[1] != 0xD0) {
    Serial.println("Kanalaufbau fehlgeschlagen");
    return false;
  }
  rxId = r.data[2] | ((r.data[3] & 0x0F) << 8);
  txId = r.data[4] | ((r.data[5] & 0x0F) << 8);
  Serial.printf("Kanal: Tester sendet auf 0x%03X, Steuergeraet auf 0x%03X\n",
                (unsigned)txId, (unsigned)rxId);
  twai_clear_receive_queue();  // wiederholte Setup-Antworten verwerfen

  sendChannelParams(0xA0);
  if (!canRecv(rxId, r, 1000) || r.data[0] != 0xA1) {
    Serial.println("Kanalparameter nicht bestaetigt");
    return false;
  }
  txSeq = 0;
  return true;
}

static void tp20Disconnect() {
  if (!connected) return;
  const uint8_t a8 = 0xA8;
  canSend(txId, &a8, 1);
  twai_message_t m;
  canRecv(rxId, m, 300);
  connected = false;
}

// ---------------------------------------------------------------------------
// TP 2.0: KWP2000-Nachrichten senden und empfangen
// ---------------------------------------------------------------------------

// Sendet eine Nachricht von bis zu 5 Nutzbytes als einzelnes Paket.
static bool tp20Send(const uint8_t *p, uint8_t len) {
  if (len > 5) return false;
  uint8_t f[8];
  f[0] = 0x10 | (txSeq & 0x0F);  // letztes Paket, Quittung erwartet
  f[1] = 0x00;
  f[2] = len;
  memcpy(f + 3, p, len);
  txSeq = (txSeq + 1) & 0x0F;
  return canSend(txId, f, 3 + len);
}

// Empfaengt eine komplette Antwort. Rueckgabe: Laenge, -1 = Timeout, -2 = Kanal beendet
static int tp20Receive(uint8_t *buf, size_t bufSize, uint32_t timeoutMs) {
  size_t got = 0;
  bool started = false;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    twai_message_t m;
    if (!canRecv(rxId, m, 50)) continue;

    if (m.data[0] == 0xA3) {  // Kanaltest vom Steuergeraet
      sendChannelParams(0xA1);
      continue;
    }
    if (m.data[0] == 0xA8) return -2;  // Steuergeraet beendet den Kanal

    uint8_t op = m.data[0] >> 4;
    uint8_t seq = m.data[0] & 0x0F;
    if (op > 3) continue;  // Quittungen (0xB) und Sonstiges ignorieren

    int off = 1;
    if (!started) {
      off = 3;  // erstes Paket: 2 Bytes Laenge ueberspringen
      started = true;
      got = 0;
    }
    for (int i = off; i < m.data_length_code && got < bufSize; i++) buf[got++] = m.data[i];

    if (op == 0 || op == 1) {  // Quittung gewuenscht
      uint8_t ack = 0xB0 | ((seq + 1) & 0x0F);
      canSend(txId, &ack, 1);
    }
    if (op == 1 || op == 3) {  // letztes Paket
      if (got >= 3 && buf[0] == 0x7F && buf[2] == 0x78) {
        // "Antwort folgt": weiter warten
        started = false;
        got = 0;
        start = millis();
        continue;
      }
      return (int)got;
    }
  }
  return -1;
}

static int kwp(const uint8_t *req, uint8_t len, uint8_t *resp, size_t size, uint32_t timeoutMs = 1500) {
  if (!tp20Send(req, len)) return -3;
  return tp20Receive(resp, size, timeoutMs);
}

// ---------------------------------------------------------------------------
// Messwerte
// ---------------------------------------------------------------------------

// Einheiten-Byte (Typ) -> Umrechnung. Nur wenige Typen, aus dem Gedaechtnis
// uebernommen: bitte gegen bekannte Werte pruefen (z. B. Leerlaufdrehzahl).
static bool decodeMeasure(uint8_t t, uint8_t a, uint8_t b, float &v, const char *&unit) {
  switch (t) {
    case 0x01: v = 0.2f * a * b;          unit = "U/min"; return true;
    case 0x05: v = a * (b - 100) * 0.1f;  unit = "C";     return true;
    case 0x06: v = 0.001f * a * b;        unit = "V";     return true;
    case 0x07: v = 0.01f * a * b;         unit = "km/h";  return true;
    case 0x12: v = 0.04f * a * b;         unit = "mbar";  return true;
    default: return false;
  }
}

static float curRpm = 0;
static float curCoolant = 0;  // Typ 0x05 = Temperatur (vermutlich Kuehlmittel)

static void printBlock(uint8_t group, const uint8_t *r, int n) {
  (void)group;
#if PRINT_BLOCKS
  Serial.printf("Block %03u:", group);
#endif
  for (int i = 2; i + 2 < n; i += 3) {
    float v;
    const char *unit;
    if (decodeMeasure(r[i], r[i + 1], r[i + 2], v, unit)) {
      if (r[i] == 0x01) curRpm = v;
      if (r[i] == 0x05) curCoolant = v;
#if PRINT_BLOCKS
      Serial.printf("  [%02X %3u %3u] = %.1f %s", r[i], r[i + 1], r[i + 2], v, unit);
    } else {
      Serial.printf("  [%02X %3u %3u]", r[i], r[i + 1], r[i + 2]);
#endif
    }
  }
#if PRINT_BLOCKS
  Serial.println();
#endif
  setValues(curRpm, curCoolant);  // fuer den Webserver bereitstellen
}

// ---------------------------------------------------------------------------
// Sitzung aufbauen
// ---------------------------------------------------------------------------
static bool openSession() {
  if (!tp20Connect()) return false;
  connected = true;

  uint8_t resp[64];
  const uint8_t start[2] = {0x10, 0x89};  // StartDiagnosticSession
  int n = kwp(start, 2, resp, sizeof(resp));
  Serial.print("StartDiagnosticSession: ");
  if (n > 0) printHex(resp, n); else Serial.printf("Fehler %d\n", n);
  if (n < 2 || resp[0] != 0x50) {
    tp20Disconnect();
    return false;
  }

  const uint8_t ident[2] = {0x1A, 0x9B};  // ECU-Kennung lesen
  n = kwp(ident, 2, resp, sizeof(resp));
  Serial.print("ECU-Kennung: ");
  if (n > 0) {
    printHex(resp, n);
    Serial.print("  Text: ");
    for (int i = 2; i < n; i++) Serial.print((resp[i] >= 32 && resp[i] < 127) ? (char)resp[i] : '.');
    Serial.println();
  } else {
    Serial.printf("Fehler %d\n", n);
  }
  return true;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nOBDScanner startet (TP 2.0)");

  valuesMutex = xSemaphoreCreateMutex();

  if (!canBegin()) {
    while (true) delay(1000);
  }

  setupWeb();
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
  canCheckBus();

  // Ohne Verbindung wird nichts gesendet, nur alle 3 s ein neuer Versuch
  if (!connected) {
    if ((int32_t)(millis() - nextConnectAt) >= 0) {
      if (!openSession()) {
        Serial.println("Verbindung fehlgeschlagen, neuer Versuch in 3 s");
        nextConnectAt = millis() + 3000;
      }
    } else {
      delay(10);
    }
    return;
  }

  static uint32_t reads = 0;
  static uint32_t lastStat = 0;

  for (uint8_t g : BLOCKS) {
    uint8_t req[2] = {0x21, g};  // readDataByLocalIdentifier (Messwertblock)
    uint8_t resp[64];
    int n = kwp(req, 2, resp, sizeof(resp));
    if (n >= 2 && resp[0] == 0x61) {
      printBlock(g, resp, n);
      reads++;
    } else if (n >= 3 && resp[0] == 0x7F) {
      Serial.printf("Block %03u: Steuergeraet lehnt ab (Dienst %02X, Code %02X)\n", g, resp[1], resp[2]);
    } else {
      Serial.printf("Block %03u: Fehler %d, Verbindung wird neu aufgebaut\n", g, n);
      tp20Disconnect();
      connected = false;
      nextConnectAt = millis() + 1000;
      return;
    }
  }

  // Einmal pro Sekunde die tatsaechliche Messrate ausgeben
  uint32_t now = millis();
  if (now - lastStat >= 1000) {
    Serial.printf("Messrate: %lu Bloecke/s\n", (unsigned long)(reads * 1000UL / (now - lastStat)));
    reads = 0;
    lastStat = now;
  }
}