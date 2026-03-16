#include <Arduino.h>

/*
  uPesy ESP32-WROOM
  - Serial  : debug USB
  - Serial2 : RX 3 bytes depuis XIAO   RX=GPIO21  TX=GPIO22 (optionnel)
  - Serial1 : AT vers LoRa-E5          RX=GPIO16  TX=GPIO17
*/

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
  if (type < 1 || type > 10) return false; // adapte si besoin
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
static const char* DEV_EUI = "70B3D57ED0073206";   // attention: ton code "qui marche" utilise ça
static const char* APP_EUI = "0000000000000000";
static const char* APP_KEY = "FE7BB9E5F461932685E0C59807BC8852";

static bool g_joined = false;
static uint32_t last_join_try_ms = 0;
static const uint32_t REJOIN_PERIOD_MS = 30000;

// ===== Helpers type "code qui marche" =====

static String readAll(Stream& s, uint32_t waitMs = 30) {
  String out;
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    while (s.available()) {
      out += (char)s.read();
      t0 = millis(); // reset timeout dès qu'on lit
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

// JOIN comme ton code qui marche, mais corrigé (Done != success)
static bool loraJoin() {
  Serial.println("[E5] Tentative de JOIN OTAA...");

  loraCmd("AT", 800);
  loraCmd("AT+MODE=LWOTAA", 800);
  loraCmd("AT+DR=EU868", 800);

  // Ton code qui marche fait DR=5 + ADR
  loraCmd("AT+DR=5", 800);
  loraCmd("AT+ADR=ON", 800);

  // Même syntaxe que ton code qui marche (SANS guillemets)
  loraCmd(String("AT+ID=DevEui,") + DEV_EUI, 800);
  loraCmd(String("AT+ID=AppEui,") + APP_EUI, 800);
  loraCmd(String("AT+KEY=APPKEY,") + APP_KEY, 1000);

  loraCmd("AT+CLASS=A", 800);
  loraCmd("AT+PORT=2", 800);

  // Lance JOIN et écoute longtemps
  loraCmd("AT+JOIN", 500);

  uint32_t t0 = millis();
  String buf;

  bool fail = false;
  bool success = false;

  while (millis() - t0 < 20000) { // 20 s
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

      // fin de procédure
      if (buf.indexOf("+JOIN: Done") >= 0 || buf.indexOf("Done") >= 0) {
        break;
      }

      // évite que buf grossisse trop
      if (buf.length() > 1200) buf.remove(0, 600);
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

static bool loraSend3Bytes(uint8_t type, uint8_t count, uint8_t conf) {
  String hex = toHex3(type, count, conf);
  String cmd = "AT+MSGHEX=\"" + hex + "\"";

  String resp = loraCmd(cmd, 4000);

  // "Done" sans "ERROR"
  bool ok = (resp.indexOf("Done") >= 0 || resp.indexOf("Tx") >= 0 || resp.indexOf("OK") >= 0) &&
            (resp.indexOf("ERROR") < 0) &&
            (resp.indexOf("Please join network first") < 0);

  Serial.printf("[E5] uplink %s\n", ok ? "OK" : "FAIL");
  return ok;
}

// ===================== RX helper =====================
static void reset_rx(const char* why) {
  rx_idx = 0;
  if (why) {
    Serial.print("[uPesy] RX reset: ");
    Serial.println(why);
  }
}

// ===================== Arduino =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("[uPesy] Start (XIAO->Serial2 GPIO21/22, LoRaE5->Serial1 GPIO16/17)");

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

  // 3 tentatives join au boot
  for (int i = 1; i <= 3 && !g_joined; i++) {
    Serial.printf("[uPesy] Join attempt %d/3\n", i);
    loraJoin();
    delay(1500);
  }
  last_join_try_ms = millis();

  if (!g_joined) {
    Serial.println("[WARN] Pas joint à TTN (pour l'instant). Je retenterai périodiquement.");
  }
}

void loop() {
  // rejoin périodique (sans bloquer la réception XIAO)
  if (!g_joined && (millis() - last_join_try_ms) > REJOIN_PERIOD_MS) {
    last_join_try_ms = millis();
    Serial.println("[uPesy] Re-join attempt...");
    loraJoin();
  }

  // resync si paquet incomplet
  if (rx_idx > 0 && (millis() - last_byte_ms) > RX_TIMEOUT_MS) {
    reset_rx("timeout");
  }

  while (Serial2.available() > 0) {
    int b = Serial2.read();
    if (b < 0) break;

    last_byte_ms = millis();
    rx_buf[rx_idx++] = (uint8_t)b;

    if (rx_idx == 3) {
      uint8_t type  = rx_buf[0];
      uint8_t count = rx_buf[1];
      uint8_t conf  = rx_buf[2];

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

      if (!g_joined) {
        Serial.println("[uPesy] Skip uplink: not joined");
      } else {
        loraSend3Bytes(type, count, conf);
      }

      rx_idx = 0;
      delay(50);
    }
  }
}