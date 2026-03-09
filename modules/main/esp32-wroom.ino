#include <Arduino.h>

/*
  uPesy ESP32-WROOM
  - Serial  : debug USB
  - Serial2 : RX 3 bytes depuis XIAO   RX=GPIO21  TX=GPIO22 (optionnel)
  - Serial1 : AT vers LoRa-E5          RX=GPIO16  TX=GPIO17

  Nouveau :
  - GPIO27 -> SHDN du Pololu S7V8F5
  - HIGH = Pololu ON  -> XIAO alimenté
  - LOW  = Pololu OFF -> XIAO coupé
*/

// =====================
// Pololu SHDN
// =====================
static const int POLOLU_SHDN_PIN = 27;
static const uint8_t POLOLU_ON  = HIGH;
static const uint8_t POLOLU_OFF = LOW;

// =====================
// Temps de cycle
// =====================
// Valeurs prudentes pour test
static const uint32_t MEASURE_PERIOD_MS    = 30000; // un cycle toutes les 30 s
static const uint32_t XIAO_BOOT_MS         = 10000; // temps pour que le XIAO boote
static const uint32_t XIAO_READ_WINDOW_MS  = 8000;  // temps max pour attendre les 3 octets
static const uint32_t AFTER_SEND_OFF_MS    = 500;   // petite marge avant extinction

static uint32_t last_cycle_ms = 0;

// =====================
// XIAO -> uPesy (Serial2)
// =====================
static const uint32_t LINK_BAUD = 115200;
static const int XIAO_RX_PIN = 21;  // uPesy RX2 <- XIAO TX
static const int XIAO_TX_PIN = 22;  // uPesy TX2 -> XIAO RX (optionnel)

static uint8_t rx_buf[3];
static uint8_t rx_idx = 0;
static uint32_t last_byte_ms = 0;
static const uint32_t RX_TIMEOUT_MS = 50;

static const bool DROP_ALL_ZERO_PACKET = true;

static bool is_plausible_packet(uint8_t type, uint8_t count, uint8_t conf) {
  if (type < 1 || type > 10) return false;
  if (count > 200) return false;
  if (conf > 100) return false;
  return true;
}

// =====================
// LoRa-E5 (Serial1)
// =====================
static const int LORAE5_RX_PIN = 16;
static const int LORAE5_TX_PIN = 17;
static const uint32_t LORAE5_BAUD = 9600;

// TTN OTAA
static const char* DEV_EUI = "70B3D57ED0073206";
static const char* APP_EUI = "0000000000000000";
static const char* APP_KEY = "FE7BB9E5F461932685E0C59807BC8852";

static bool g_joined = false;
static uint32_t last_join_try_ms = 0;
static const uint32_t REJOIN_PERIOD_MS = 30000;

// =====================
// Helpers
// =====================
static String readAll(Stream& s, uint32_t waitMs = 30) {
  String out;
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    while (s.available()) {
      out += (char)s.read();
      t0 = millis();
    }
    delay(1);
  }
  return out;
}

static String loraCmd(const String& cmd, uint32_t waitMs = 600) {
  Serial1.print(cmd);
  Serial1.print("\r\n");
  String r = readAll(Serial1, waitMs);

  if (r.length()) {
    Serial.printf("[E5] %s -> %s\n", cmd.c_str(), r.c_str());
  } else {
    Serial.printf("[E5] %s -> (no response)\n", cmd.c_str());
  }
  return r;
}

static String toHex3(uint8_t a, uint8_t b, uint8_t c) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X", a, b, c);
  return String(buf);
}

static void reset_rx(const char* why) {
  rx_idx = 0;
  if (why) {
    Serial.print("[uPesy] RX reset: ");
    Serial.println(why);
  }
}

static void pololuOn() {
  digitalWrite(POLOLU_SHDN_PIN, POLOLU_ON);
  Serial.println("[PWR] Pololu ON -> XIAO alimente");
}

static void pololuOff() {
  digitalWrite(POLOLU_SHDN_PIN, POLOLU_OFF);
  Serial.println("[PWR] Pololu OFF -> XIAO coupe");
}

// =====================
// JOIN LoRaWAN
// =====================
static bool loraJoin() {
  Serial.println("[E5] Tentative de JOIN OTAA...");

  loraCmd("AT", 800);
  loraCmd("AT+MODE=LWOTAA", 800);
  loraCmd("AT+DR=EU868", 800);
  loraCmd("AT+DR=5", 800);
  loraCmd("AT+ADR=ON", 800);

  loraCmd(String("AT+ID=DevEui,") + DEV_EUI, 800);
  loraCmd(String("AT+ID=AppEui,") + APP_EUI, 800);
  loraCmd(String("AT+KEY=APPKEY,") + APP_KEY, 1000);

  loraCmd("AT+CLASS=A", 800);
  loraCmd("AT+PORT=2", 800);

  loraCmd("AT+JOIN", 500);

  uint32_t t0 = millis();
  String buf;
  bool fail = false;
  bool success = false;

  while (millis() - t0 < 20000) {
    buf += readAll(Serial1, 300);

    if (buf.length()) {
      Serial.printf("[E5 RESP] %s\n", buf.c_str());

      if (buf.indexOf("Join failed") >= 0 || buf.indexOf("failed") >= 0) {
        fail = true;
      }

      if (buf.indexOf("Joined successfully") >= 0 ||
          buf.indexOf("Network joined") >= 0 ||
          buf.indexOf("accepted") >= 0) {
        success = true;
      }

      if (buf.indexOf("+JOIN: Done") >= 0 || buf.indexOf("Done") >= 0) {
        break;
      }

      if (buf.length() > 1200) {
        buf.remove(0, 600);
      }
    }
  }

  if (fail || !success) {
    Serial.println("[E5] Join KO");
    g_joined = false;
    return false;
  }

  Serial.println("[E5] Join OK");
  g_joined = true;
  return true;
}

// =====================
// Uplink LoRa
// =====================
static bool loraSend3Bytes(uint8_t type, uint8_t count, uint8_t conf) {
  String hex = toHex3(type, count, conf);
  String cmd = "AT+MSGHEX=\"" + hex + "\"";

  String resp = loraCmd(cmd, 4000);

  bool ok = (resp.indexOf("Done") >= 0 || resp.indexOf("Tx") >= 0 || resp.indexOf("OK") >= 0) &&
            (resp.indexOf("ERROR") < 0) &&
            (resp.indexOf("Please join network first") < 0);

  if (!ok && resp.indexOf("Please join network first") >= 0) {
    g_joined = false;
  }

  Serial.printf("[E5] uplink %s\n", ok ? "OK" : "FAIL");
  return ok;
}

// =====================
// Attente paquet XIAO
// =====================
static bool waitPacketFromXIAO(uint8_t &type, uint8_t &count, uint8_t &conf, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  reset_rx("new cycle");

  while (millis() - t0 < timeoutMs) {
    if (rx_idx > 0 && (millis() - last_byte_ms) > RX_TIMEOUT_MS) {
      reset_rx("timeout");
    }

    while (Serial2.available() > 0) {
      int b = Serial2.read();
      if (b < 0) break;

      last_byte_ms = millis();
      rx_buf[rx_idx++] = (uint8_t)b;

      if (rx_idx == 3) {
        type  = rx_buf[0];
        count = rx_buf[1];
        conf  = rx_buf[2];

        if (DROP_ALL_ZERO_PACKET && type == 0 && count == 0 && conf == 0) {
          Serial.println("[uPesy] Drop packet 00 00 00 (noise?)");
          rx_idx = 0;
          continue;
        }

        if (!is_plausible_packet(type, count, conf)) {
          Serial.printf("[uPesy] Drop implausible: %02X %02X %02X\n", type, count, conf);
          rx_idx = 0;
          continue;
        }

        Serial.printf("[uPesy] RX => type=%u count=%u conf=%u\n", type, count, conf);
        Serial.printf("[uPesy] HEX => %02X %02X %02X\n", type, count, conf);

        rx_idx = 0;
        return true;
      }
    }

    delay(2);
  }

  return false;
}

// =====================
// Un cycle complet
// =====================
static void doMeasurementCycle() {
  uint8_t type = 0;
  uint8_t count = 0;
  uint8_t conf = 0;

  Serial.println();
  Serial.println("======================================");
  Serial.println("[CYCLE] Debut cycle mesure");

  // 1) Allumer le XIAO via Pololu
  pololuOn();

  // 2) Attendre son boot
  Serial.printf("[CYCLE] Attente boot XIAO: %lu ms\n", (unsigned long)XIAO_BOOT_MS);
  delay(XIAO_BOOT_MS);

  // 3) Attendre le paquet venant du XIAO
  Serial.printf("[CYCLE] Attente paquet XIAO pendant %lu ms\n", (unsigned long)XIAO_READ_WINDOW_MS);
  bool gotPacket = waitPacketFromXIAO(type, count, conf, XIAO_READ_WINDOW_MS);

  if (!gotPacket) {
    Serial.println("[CYCLE] Aucun paquet valide recu du XIAO");
  } else {
    if (!g_joined) {
      Serial.println("[uPesy] Skip uplink: not joined");
    } else {
      loraSend3Bytes(type, count, conf);
    }
  }

  delay(AFTER_SEND_OFF_MS);

  // 4) Couper le XIAO
  pololuOff();

  Serial.println("[CYCLE] Fin cycle mesure");
  Serial.println("======================================");
}

// =====================
// Arduino
// =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("[uPesy] Start (XIAO->Serial2 GPIO21/22, LoRaE5->Serial1 GPIO16/17, Pololu SHDN GPIO27)");

  pinMode(POLOLU_SHDN_PIN, OUTPUT);

  // Au démarrage, on coupe le Pololu
  pololuOff();

  // UART XIAO
  Serial2.begin(LINK_BAUD, SERIAL_8N1, XIAO_RX_PIN, XIAO_TX_PIN);
  Serial.printf("[uPesy] Serial2 XIAO  RX=%d TX=%d @%lu\n",
                XIAO_RX_PIN, XIAO_TX_PIN, (unsigned long)LINK_BAUD);

  // UART LoRa-E5
  Serial1.begin(LORAE5_BAUD, SERIAL_8N1, LORAE5_RX_PIN, LORAE5_TX_PIN);
  Serial.printf("[uPesy] Serial1 LoRaE5 RX=%d TX=%d @%lu\n",
                LORAE5_RX_PIN, LORAE5_TX_PIN, (unsigned long)LORAE5_BAUD);

  delay(500);
  reset_rx("init");

  // JOIN au boot
  for (int i = 1; i <= 3 && !g_joined; i++) {
    Serial.printf("[uPesy] Join attempt %d/3\n", i);
    loraJoin();
    delay(1500);
  }

  last_join_try_ms = millis();

  if (!g_joined) {
    Serial.println("[WARN] Pas joint a TTN (pour l'instant). Je retenterai periodiquement.");
  }

  // Premier cycle immédiat
  last_cycle_ms = millis() - MEASURE_PERIOD_MS;
}

void loop() {
  // Rejoin si besoin
  if (!g_joined && (millis() - last_join_try_ms) > REJOIN_PERIOD_MS) {
    last_join_try_ms = millis();
    Serial.println("[uPesy] Re-join attempt...");
    loraJoin();
  }

  // Lancer un cycle périodiquement
  if ((millis() - last_cycle_ms) >= MEASURE_PERIOD_MS) {
    last_cycle_ms = millis();
    doMeasurementCycle();
  }

  delay(10);
}