// =============================================================
// ADAU1701-TCPi-ESP32  v1.0
// WiFi bridge between SigmaStudio and ADAU1701 DSP
//
// Features:
//   - Full SigmaStudio TCPi protocol over WiFi (port 8086)
//   - Hardware safeload: glitch-free real-time parameter updates
//   - Adapted for ESP8266 platform
//   - Chunked I2C writes: prevents silent data corruption
//   - EEPROM selfboot: save program from web interface
//   - Web portal: WiFi + GPIO pin configuration
//   - AP mode for first-time setup
//   - Write protect pin support on "green" boards
//
// https://github.com/rarranzb/ADAU1701-TCPi-ESP32
// https://github.com/roxt-underground/ADAU1701-TCPi-ESP
// License: MIT
//   - Captures all sl=0 writes during Download
//   - http://IP/ -> button "Save to EEPROM"
//   - Writes captured data to EEPROM in ADAU1701 selfboot format
//   - Replaces "Actions -> Write Latest Compilation to E2Prom"
//     which does not work over TCP/IP in SigmaStudio
//
// https://github.com/rarranzb/ADAU1701-TCPi-ESP32
// License: MIT
// =============================================================
#if defined(ESP32)
#include <WiFi.h>
#include <WebServer.h>

#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#define WebServer ESP8266WebServer

#endif

#include <Wire.h>
#include <Preferences.h>
#include <map>
#include <unordered_set>
#include "preset.hpp"

// ── Factory defaults ──────────────────────────────────────────
#define FACTORY_SSID      ""
#define FACTORY_PASSWORD  ""

// ── AP mode ───────────────────────────────────────────────────
#define AP_SSID           "ADAU1701-ESP"
#define AP_PASSWORD       "adau1701"
#define AP_WAIT_TIMEOUT   (1 * 60 * 1000)

// ── Default pins ──────────────────────────────────────────────
#if defined(ESP32)
#define DEFAULT_SCL       17
#define DEFAULT_SDA       16
#define DEFAULT_RESET     21
#define DEFAULT_SELFBOOT  19
#define DEFAULT_LED        2
#define BOOT_BUTTON_PIN    0

#elif defined(ARDUINO_ARCH_ESP8266)

// Wemos d1 mini pinout ()
#define DEFAULT_SCL       5  // D1
#define DEFAULT_SDA       4  // D2
#define DEFAULT_RESET     0  // D3
#define DEFAULT_SELFBOOT  12  // D6
#define DEFAULT_LED       LED_BUILTIN  // D4 
// #define BOOT_BUTTON_PIN   12 // D6 
#define WRITE_PROTECT_PIN 13  // D7
#endif



// ── DSP ───────────────────────────────────────────────────────
#define DSP_I2C_ADDR      0x34
#define EEPROM_I2C_ADDR   0x50
#define TCP_PORT          8086

// ── ADAU1701 memory map ───────────────────────────────────────
#define PARAM_RAM_END     0x03FF
#define PROG_RAM_START    0x0400
#define PROG_RAM_END      0x07FF
#define CTRL_REG_START    0x0800

// ── Safeload registers ────────────────────────────────────────
#define SAFELOAD_DATA_0   0x0810
#define SAFELOAD_ADDR_0   0x0815
#define IST_BIT           0x20

// ── TCPi protocol ─────────────────────────────────────────────
#define CTRL_WRITE        0x09
#define CTRL_READ_REQ     0x0A
#define CTRL_READ_RESP    0x0B

#if defined(ESP32)
#define BUFFER_SIZE       (1024 * 16)

#elif defined(ARDUINO_ARCH_ESP8266)
#define BUFFER_SIZE       (1024 * 4)

#else
#define BUFFER_SIZE       (1024 * 4)
#endif


// ── EEPROM selfboot capture ───────────────────────────────────
// Selfboot format per record: [len(2)] [addr(2)] [data(len)]
// End marker: 0x00 0x00
// 24LC256 = 32KB, page size = 64 bytes
#define EEPROM_MAX_SIZE   (32 * 1024)
#define EEPROM_PAGE_SIZE  64
#define CAPTURE_MAX_SIZE  (28 * 1024)   // leave margin

uint8_t  captureBuffer[CAPTURE_MAX_SIZE];
int      captureLen    = 0;
bool     captureReady  = false;   // true after DSPRUN=1 seen

// ── Globals ───────────────────────────────────────────────────
Preferences  prefs;
WiFiServer*  tcpServer = nullptr;
WiFiClient   client;
WebServer    httpServer(80);

bool     apMode          = false;
unsigned long  apStartedAt   = 0;
bool     dspRunning      = false;
bool     webUIRunning    = false;
uint8_t  lastCoreCtrl[2] = {0x00, 0x1C};
uint8_t  rxBuffer[BUFFER_SIZE];
int      rxLen           = 0;

String   savedSSID, savedPassword;
int      pinSCL, pinSDA, pinRESET, pinSELFBOOT, pinLED;

#ifdef WRITE_PROTECT_PIN
#define write_protect_high digitalWrite(WRITE_PROTECT_PIN, HIGH)
#define write_protect_low  digitalWrite(WRITE_PROTECT_PIN, LOW)
#else 
#define write_protect_high __asm__ __volatile__ ("nop\n\t")
#define write_protect_low  __asm__ __volatile__ ("nop\n\t")
#endif

// Presets
#define PRESET_MAX_SIZE   16
#define PRESET_ROW_LENTGH 16

#define PRESET_1 "preset1"
#define PRESET_2 "preset2"
#define PRESET_3 "preset3"
#define PRESET_4 "preset4"
#define DEFAULT_PRESET PRESET_1
#define ALL_PRESETS {PRESET_1, PRESET_2, PRESET_3, PRESET_4}

struct presetItem {
  uint16_t address;
  uint8_t value[PRESET_ROW_LENTGH];
};

typedef std::map<uint16_t, uint8_t[PRESET_ROW_LENTGH]> presetMap;
  

// ── Prototypes ────────────────────────────────────────────────
void loadConfig();
void saveWiFi(const String& ssid, const String& pass);
void savePins(int scl, int sda, int rst, int sb, int led);
bool connectWiFi();
void startAP();
void stopAP();
void setupHTTP();
void unloadHTTP();
void initHardware();
void blinkLED(int n);
void scanI2C();
void resetDSP();
void captureWrite(uint16_t address, uint8_t* data, uint16_t dataLen);
bool writeEEPROM();
bool eepromWritePage(uint16_t memAddr, uint8_t* data, int len);
void resetEEPROMCapture();
int  processBuffer(uint8_t* buf, int len);
void handleWrite(uint8_t chipAddr, uint16_t address, uint8_t* data, uint16_t dataLen, uint8_t safeload);
void handleRead(uint8_t chipAddr, uint16_t address, uint16_t nBytes);
void safeloadChunk(uint16_t address, uint8_t* data, int words);
void directWrite(uint8_t i2cAddr, uint16_t regAddr, uint8_t* data, uint16_t dataLen);
    // ── Preset area ───────────────────────────────────────────
presetMap getPreset(String presetName);
void writePreset(String presetName, presetMap * presetData);
bool validatePresetName(String presetNmae);
  

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] ADAU1701-TCPi-ESP32 v1.0");
  delay(500);

  loadConfig();
  initHardware();
  scanI2C();

  if (!connectWiFi()) {
    Serial.println("[WiFi] Failed -> AP mode");
    startAP();
  }

  setupHTTP();
  if (!apMode) {
    tcpServer = new WiFiServer(TCP_PORT);
    tcpServer->begin();
  }

  blinkLED(apMode ? 10 : 2);
  if (apMode)
    Serial.printf("[AP] SSID:'%s' pass:'%s' -> http://192.168.4.1\n", AP_SSID, AP_PASSWORD);
  else
    Serial.printf("[BOOT] Ready  TCP:%d  HTTP:80\n", TCP_PORT);
}

// =============================================================
// LOOP
// =============================================================
void loop() {
  httpServer.handleClient();
  if (apMode) {
    if (WiFi.softAPgetStationNum() > 0) {
      Serial.println("[AP] Found connected clients. Waiting Wi-Fi configuration...");
      delay(1000);
      return;
    }
    if ((millis() - apStartedAt) > AP_WAIT_TIMEOUT) {
      unloadHTTP();
      stopAP();
    }
    return;
  }

  #if defined(ESP32)
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      Serial.println("[BTN] DSP reset");
      blinkLED(3);
      resetDSP();
      while (digitalRead(BOOT_BUTTON_PIN) == LOW) delay(10);
    }
  }
  #endif

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost -> reconnecting...");
    if (connectWiFi()) {
      setupHTTP();
      tcpServer->begin();
    }
    else {
      Serial.println("[WiFi] Fail reconect. Go to sleep next 1 minute.");
      delay(1000*60);
      return;
    }
  }

  if (!client || !client.connected()) {
    digitalWrite(pinLED, LOW);
    client = tcpServer->available();
    if (client) {
      Serial.printf("[TCP] Connection from %s\n", client.remoteIP().toString().c_str());
      digitalWrite(pinLED, HIGH);
      blinkLED(2);
      // Reset capture on new connection (new Download incoming)
      captureLen   = 0;
      captureReady = false;
      rxLen        = 0;
      dspRunning   = false;
    }
  }

  if (client && client.connected() && (client.available() > 0)) {
    int n = client.readBytes(rxBuffer + rxLen, BUFFER_SIZE - rxLen);
    rxLen += n;
    int consumed = processBuffer(rxBuffer, rxLen);
    if (consumed > 0 && consumed < rxLen)
      memmove(rxBuffer, rxBuffer + consumed, rxLen - consumed);
    rxLen = (consumed <= rxLen) ? rxLen - consumed : 0;
  }
}

// =============================================================
// NVS
// =============================================================
void loadConfig() {
  prefs.begin("tcpi", true);
  savedSSID    = prefs.getString("ssid",      FACTORY_SSID);
  savedPassword= prefs.getString("password",  FACTORY_PASSWORD);
  pinSCL       = prefs.getInt("pin_scl",      DEFAULT_SCL);
  pinSDA       = prefs.getInt("pin_sda",      DEFAULT_SDA);
  pinRESET     = prefs.getInt("pin_reset",    DEFAULT_RESET);
  pinSELFBOOT  = prefs.getInt("pin_selfboot", DEFAULT_SELFBOOT);
  pinLED       = prefs.getInt("pin_led",      DEFAULT_LED);
  prefs.end();
  Serial.printf("[NVS] SSID:%s  SCL:%d SDA:%d RST:%d SB:%d LED:%d\n",
    savedSSID.c_str(), pinSCL, pinSDA, pinRESET, pinSELFBOOT, pinLED);
}

void saveWiFi(const String& ssid, const String& pass) {
  prefs.begin("tcpi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("password", pass);
  prefs.end();
}

void savePins(int scl, int sda, int rst, int sb, int led) {
  prefs.begin("tcpi", false);
  prefs.putInt("pin_scl",      scl);
  prefs.putInt("pin_sda",      sda);
  prefs.putInt("pin_reset",    rst);
  prefs.putInt("pin_selfboot", sb);
  prefs.putInt("pin_led",      led);
  prefs.end();
}

// =============================================================
// HARDWARE
// =============================================================
void initHardware() {
  pinMode(pinLED,          OUTPUT); digitalWrite(pinLED,      LOW);
  pinMode(pinSELFBOOT,     OUTPUT); digitalWrite(pinSELFBOOT, LOW);
  pinMode(pinRESET,        OUTPUT); digitalWrite(pinRESET,  HIGH);
  #ifdef BOOT_BUTTON_PIN
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  #endif
  #ifdef WRITE_PROTECT_PIN
  pinMode(WRITE_PROTECT_PIN,       OUTPUT);
  write_protect_high;
  #endif
  #if defined(ESP32)
  Wire.setBufferSize(2048);
  Wire.begin(pinSDA, pinSCL, 400000);
  Wire.setTimeOut(50);
  #else // (ESP8266)
  Wire.begin(pinSDA, pinSCL, 400000);
  Wire.setTimeout(50);
  #endif
  Serial.printf("[I2C] SCL=%d SDA=%d 400kHz\n", pinSCL, pinSDA);
}

// =============================================================
// WIFI
// =============================================================
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
  Serial.printf("[WiFi] Connecting to '%s'", savedSSID.c_str());
  for (int t = 0; t < 20 && WiFi.status() != WL_CONNECTED; t++) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    apMode = false; return true;
  }
  Serial.println("\n[WiFi] Failed");
  return false;
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apMode = true;
  apStartedAt = millis();
}

void stopAP() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  apMode = false;
  apStartedAt = 0;
}

// =============================================================
// EEPROM CAPTURE
// During sl=0 (Download), capture every DSP write in selfboot
// format: [len(2BE)] [addr(2BE)] [data...]
// =============================================================
void captureWrite(uint16_t address, uint8_t* data, uint16_t dataLen) {
  // ADAU1701 selfboot I2C master has limited buffer.
  // Split large writes into chunks of max 32 words to be safe.
  // Word sizes: Param RAM = 4 bytes, Prog RAM = 5 bytes, Ctrl = 2 bytes
  int wordSize = 4;
  if (address >= PROG_RAM_START && address <= PROG_RAM_END) wordSize = 5;
  else if (address >= CTRL_REG_START) wordSize = 2;

  int maxWords  = 32;
  int chunkSize = maxWords * wordSize;
  int offset    = 0;

  while (offset < dataLen) {
    int      bytes   = min(dataLen - offset, chunkSize);
    uint16_t addr    = address + (offset / wordSize);
    uint8_t* chunk   = data + offset;

    int needed = 6 + bytes;  // 1 opcode + 2 count + 1 chipAddr + 2 regAddr + bytes
    if (captureLen + needed + 2 > CAPTURE_MAX_SIZE) {
      Serial.println("[CAP] Buffer full, skipping");
      return;
    }

    // ADAU1701 selfboot EEPROM format (datasheet Table 19):
    // 0x01               = Write message type
    // [len_hi][len_lo]   = chipAddr(1) + regAddr(2) + dataBytes
    // 0x00               = chip address (always 0x00 in EEPROM messages)
    // [addr_hi][addr_lo] = register address
    // [data...]
    uint16_t count = bytes + 3;  // chipAddr(1) + regAddr(2) + data
    captureBuffer[captureLen++] = 0x01;                  // Write opcode
    captureBuffer[captureLen++] = (count >> 8) & 0xFF;
    captureBuffer[captureLen++] = count & 0xFF;
    captureBuffer[captureLen++] = 0x00;                  // chip address
    captureBuffer[captureLen++] = (addr >> 8) & 0xFF;
    captureBuffer[captureLen++] = addr & 0xFF;
    memcpy(captureBuffer + captureLen, chunk, bytes);
    captureLen += bytes;

    offset += bytes;
  }
}

// =============================================================
// EEPROM WRITE - selfboot format
// 24LC256: page size 64 bytes, write cycle ~5ms
// =============================================================
bool writeEEPROM() {
  if (captureLen == 0) {
    Serial.println("[EEPROM] Nothing captured");
    return false;
  }

  // Append end marker 0x0000
  captureBuffer[captureLen++] = 0x00;
  captureBuffer[captureLen++] = 0x00;

  write_protect_low; delay(100);
  digitalWrite(pinRESET,  LOW); delay(10);

  Serial.printf("[EEPROM] Writing %d bytes to 0x%02X...\n", captureLen, EEPROM_I2C_ADDR);

  int offset = 0;
  while (offset < captureLen) {
    // Calculate bytes remaining in current page
    int pageOffset = (offset) % EEPROM_PAGE_SIZE;
    int bytes = min(captureLen - offset, EEPROM_PAGE_SIZE - pageOffset);

    if (!eepromWritePage(offset, captureBuffer + offset, bytes)) {
      Serial.printf("[EEPROM] Write failed at offset %d\n", offset);

      write_protect_high; delay(10);
      digitalWrite(pinRESET,  HIGH); delay(10);
      return false;
    }
    offset += bytes;

    // Progress log every 1KB
    if (offset % 1024 == 0)
      Serial.printf("[EEPROM] %d / %d bytes\n", offset, captureLen);
  }

  Serial.println("[EEPROM] Done!");
  write_protect_high; delay(100);
  digitalWrite(pinRESET,  HIGH); delay(100);
  return true;
}

bool eepromWritePage(uint16_t memAddr, uint8_t* data, int len) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((memAddr >> 8) & 0xFF);
  Wire.write(memAddr & 0xFF);
  Wire.write(data, len);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) {
    Serial.printf("[EEPROM] I2C err=%d at 0x%04X\n", err, memAddr);
    return false;
  }
  // Wait for write cycle to complete (24LC256 max 5ms)
  delay(6);
  return true;
}

// =============================================================
// WEB INTERFACE
// =============================================================
String hexString(uint16_t);

String htmlHead(const String& title) {
  return R"(<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>)" + title + R"(</title>
<style>
  *{box-sizing:border-box}
  body{font-family:sans-serif;max-width:480px;margin:30px auto;padding:0 16px;color:#222}
  h2{margin-bottom:4px}
  h3{margin:24px 0 8px;border-bottom:1px solid #ddd;padding-bottom:4px}
  nav{margin-bottom:20px}
  nav a{margin-right:14px;color:#0078d4;text-decoration:none;font-size:14px}
  nav a:hover{text-decoration:underline}
  .card{background:#f7f7f7;border:1px solid #e0e0e0;border-radius:6px;
        padding:14px;margin-bottom:16px;font-size:14px}
  .card b{display:inline-block;width:110px;color:#555}
  label{display:block;font-size:13px;color:#555;margin-top:10px}
  input[type=text],input[type=password],input[type=number]{
    width:100%;padding:8px 10px;margin-top:4px;border:1px solid #ccc;
    border-radius:4px;font-size:15px}
  .row{display:flex;gap:12px}.row>div{flex:1}
  .btn{display:block;width:100%;padding:10px;margin-top:12px;border:none;
    border-radius:4px;font-size:15px;cursor:pointer;color:#fff}
  .btn-blue{background:#0078d4}.btn-blue:hover{background:#005fa3}
  .btn-green{background:#28a745}.btn-green:hover{background:#1e7e34}
  .btn-red{background:#dc3545}.btn-red:hover{background:#a71d2a}
  .btn-gray{background:#6c757d}.btn-gray:hover{background:#545b62}
  .ok{background:#d4edda;border:1px solid #c3e6cb;border-radius:4px;
    padding:10px;color:#155724;margin-top:12px}
  .warn{background:#fff3cd;border:1px solid #ffc107;border-radius:4px;
    padding:10px;color:#856404;margin-top:12px}
  .err{background:#f8d7da;border:1px solid #f5c6cb;border-radius:4px;
    padding:10px;color:#721c24;margin-top:12px}
  small{color:#888;font-size:12px}
</style></head><body>
<h2>ADAU1701-TCPi-ESP32</h2>
<nav><a href='/'>Status</a><a href='/config'>Configuration</a><a href='/presets'>Presets</a></nav>
)";
}

String messagePage(String title, String msgKlass, String message, String pageDelay) {
  httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  httpServer.send(200, "text/html", "");
  httpServer.sendContent(htmlHead(title));
  httpServer.sendContent("<div class='"+ msgKlass +"'>"+ message +"</div>"
                         "<script>setTimeout(()=>location.href='/', "+ pageDelay + " )</script>"
                         "</body></html>");
  httpServer.sendContent("");
  return "";
}

String readPresetToHtml(String presetName) {
  String body = "", presetRow;
  presetMap preset = getPreset(presetName);

  body += "<div id='presetInputs'>\n";
  Serial.printf("[Preset] Render %d keys\n", preset.size());
  uint8_t i = 0, j, row_len;
  for (const auto& element : preset) {
    presetRow = "";
    row_len = element.second[0];
    for (j=1; j < row_len+1; j++) presetRow += hexString(element.second[j]) + (j == row_len ? "" : "," );

    body += "<div class='row'>\n"
            "<input type='text' name='address" + String(i) + "' value='" + hexString(element.first) + "'/>\n"
            "<input type='text' name='value" + String(i) +   "' value='" + presetRow + "'/>\n</div>\n";
    i++;
  }
  body += "</div>\n"
          "<input id='presetLen' value='" + String(i)+ "' type='number' name='len' hidden='true' />\n"
          "<script>function addPresetRow() {"
          "        const $ = id => document.getElementById(id); let len = +$('presetLen').value;"
          "        $('presetInputs').insertAdjacentHTML('beforeend',`<div class='row'><input type='text' name='address${len}'><input type='text' name='value${len}'></div>`);"
          "        $('presetLen').value = ++len;}</script>"
          "<input class='btn btn-green' onClick='addPresetRow()' type=button value='+'/>\n";
  return body;
}

void setupHTTP() {
  httpServer.close();

  // ── Status page ────────────────────────────────────────────
  httpServer.on("/", []() {
    String ip   = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String body = "";
    httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    httpServer.send(200, "text/html", "");
    httpServer.sendContent(htmlHead("ADAU1701-TCPi-ESP32"));

    body += "<div class='card'>";
    body += "<div><b>IP:</b>" + ip + "</div>";
    if (!apMode) {
      body += "<div><b>Network:</b>" + savedSSID + "</div>";
      body += "<div><b>DSP:</b>" + String(dspRunning ? "&#x25CF; Running" : "&#x25CB; Stopped") + "</div>";
      body += "<div><b>TCP port:</b>" + String(TCP_PORT) + "</div>";

      // EEPROM capture status
      if (captureReady) {
        body += "<div><b>Capture:</b>&#x2705; Ready (" + String(captureLen) + " bytes)</div>";
      } else if (captureLen > 0) {
        body += "<div><b>Capture:</b>&#x23F3; Downloading... (" + String(captureLen) + " bytes)</div>";
      } else {
        body += "<div><b>Capture:</b>&#x25CB; Waiting for Download</div>";
      }
    }
    body += "</div>";
    httpServer.sendContent(body);
    body = "";

    if (apMode) {
      body += "<div class='warn'>Connect to <b>" + String(AP_SSID) +
              "</b> and open <b>http://192.168.4.1/config</b> to set up WiFi.</div>";
    }

    // ── DSP Reset ──────────────────────────────────────────
    body += "<form action='/reset_dsp' method='POST'>"
            "<button class='btn btn-gray'>Reset DSP</button></form>";

    if (captureReady) {
    // ── Capture Reset ──────────────────────────────────────
      body += "<form action='/reset_capture' method='POST'>"
              "<button class='btn btn-gray'>Reset captured program</button></form>";
    // ── Save to EEPROM ─────────────────────────────────────
      body += "<form action='/save_eeprom' method='POST'>"
              "<button class='btn btn-green'>&#x1F4BE; Save to EEPROM (selfboot)</button></form>"
              "<small style='display:block;margin-top:6px'>"
              "Writes the current program to EEPROM. "
              "Enable SELFBOOT on your DSP board to boot autonomously.</small>";
    } else {
      body += "<div class='warn' style='margin-top:12px'>"
              "&#x26A0; Do a <b>Link Compile Download</b> in SigmaStudio first, "
              "then the Save to EEPROM button will appear.</div>";
    }

    body += "</body></html>";
    httpServer.sendContent(body);
    httpServer.sendContent("");
  });

  // ── Save to EEPROM ─────────────────────────────────────────
  httpServer.on("/save_eeprom", HTTP_POST, []() {
    if (!captureReady) {
      httpServer.send(400, "text/html",
        htmlHead("Error") +
        "<div class='err'>No capture available. Do a Download first.</div>"
        "<script>setTimeout(()=>location.href='/',2000)</script></body></html>");
      return;
    }

    // Send immediate response, then write (writing takes a few seconds)
    httpServer.send(200, "text/html",
      htmlHead("Saving...") +
      "<div class='ok'>&#x23F3; Writing to EEPROM, please wait (~5 seconds)...</div>"
      "<script>setTimeout(()=>location.href='/eeprom_result',8000)</script></body></html>");

    // Write EEPROM
    bool ok = writeEEPROM();
    prefs.begin("tcpi", false);
    prefs.putBool("eeprom_ok", ok);
    prefs.end();
  });

  // ── EEPROM result ───────────────────────────────────────────
  httpServer.on("/eeprom_result", []() {
    prefs.begin("tcpi", true);
    bool ok = prefs.getBool("eeprom_ok", false);
    prefs.end();
    String body = htmlHead("EEPROM Result");
    if (ok) {
      body += "<div class='ok'>&#x2705; EEPROM written successfully!<br><br>"
              "<b>Next steps:</b><br>"
              "1. Enable SELFBOOT on your DSP board (jumper or pin HIGH)<br>"
              "2. Power cycle the board<br>"
              "3. The ADAU1701 will boot autonomously from EEPROM</div>";
    } else {
      body += "<div class='err'>&#x274C; EEPROM write failed.<br>"
              "Check I2C wiring and that your board has an EEPROM at 0x50.</div>";
    }
    body += "<a href='/'>Back</a></body></html>";
    httpServer.send(200, "text/html", body);
  });

  // ── Config page ─────────────────────────────────────────────
  httpServer.on("/config", []() {
    String body = "";
    String networks = "";
    if (apMode) {
      int n = WiFi.scanNetworks();
      if (n > 0){
    networks += "<div>Near networks:</div>"
            "<ul>";
        for (int _i = 0; _i < n; _i++) {
    networks += "<li><button onclick=\"document.getElementById('ssidInput').value = '" + String(WiFi.SSID(_i)) + "'\">" + String(WiFi.SSID(_i)) + "</button></li>";
        }
    networks += "</ul>";
      }
    }

    httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    httpServer.send(200, "text/html", "");
    httpServer.sendContent(htmlHead("Configuration"));
    body += "<h3>WiFi</h3>"
            "<form action='/save_wifi' method='POST'>";
    httpServer.sendContent(body);
    body = networks;
    body += "<label>SSID</label>"
            "<input type='text' name='ssid' id='ssidInput' value='" + savedSSID + "' required>"
            "<label>Password</label>"
            "<input type='password' name='pass' placeholder='leave empty to keep current'>"
            "<button class='btn btn-blue'>Save WiFi &amp; reboot</button></form>\n\n";
    httpServer.sendContent(body);
    body =  "<h3>GPIO Pins</h3>"
            "<small>Do not use GPIO 6-11 (reserved for flash).</small>"
            "<form action='/save_pins' method='POST'>"
            "<div class='row'>"
            "<div><label>SCL</label><input type='number' name='scl' value='" + String(pinSCL) + "' min='0' max='39'></div>"
            "<div><label>SDA</label><input type='number' name='sda' value='" + String(pinSDA) + "' min='0' max='39'></div>"
            "</div><div class='row'>";
    httpServer.sendContent(body);
    body =  "<div><label>RESET</label><input type='number' name='rst' value='" + String(pinRESET) + "' min='0' max='39'></div>"
            "<div><label>SELFBOOT</label><input type='number' name='sb' value='" + String(pinSELFBOOT) + "' min='0' max='39'></div>"
            "</div>"
            "<label>LED</label><input type='number' name='led' value='" + String(pinLED) + "' min='0' max='39'>"
            "<button class='btn btn-blue'>Save pins &amp; reboot</button></form>\n\n";
    httpServer.sendContent(body);
    body =  "<h3>Factory reset</h3>"
            "<form action='/factory_reset' method='POST'>"
            "<button class='btn btn-red'>Clear all settings</button></form>";
    body += "</body></html>";
    httpServer.sendContent(body);
    httpServer.sendContent("");
  });

  httpServer.on("/presets", []() {
      String body = "";

      String presetName = httpServer.arg("preset");
      if (presetName.isEmpty() || !validatePresetName(presetName)) presetName = String(DEFAULT_PRESET);
      httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
      httpServer.send(200, "text/html", "");
      httpServer.sendContent(htmlHead("Presets"));
      body += "<h3>Preset edit</h3>"
              "<h4> Current preset: " + presetName + "</h4>\n<div>\n";
              for (String pr : ALL_PRESETS)
                body += "<span><a href='/presets?preset="+pr+"'>"+ pr +"</a></span>\n";
      body += "<form action='/preset_apply' method='POST'>"
              "<input type='text' id='name' value='" + presetName + "' name='name' hidden='true' />\n";
      httpServer.sendContent(body);
      httpServer.sendContent(readPresetToHtml(presetName));
      httpServer.sendContent("<button class='btn btn-blue'>Apply preset</button></form>\n\n");
      httpServer.sendContent("");
  });

  httpServer.on("/save_wifi", HTTP_POST, []() {
    if (!httpServer.hasArg("ssid") || httpServer.arg("ssid").isEmpty()) {
      httpServer.send(400, "text/plain", "Missing SSID"); return;
    }
    String newSSID = httpServer.arg("ssid");
    String newPass = httpServer.arg("pass");
    if (newPass.isEmpty()) newPass = savedPassword;
    saveWiFi(newSSID, newPass);

    // Try to connect while keeping AP alive so browser stays connected
    // WIFI_AP_STA: AP stays up during connection attempt
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(newSSID.c_str(), newPass.c_str());
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 20) delay(500);

    String body = htmlHead("WiFi Saved");
    if (WiFi.status() == WL_CONNECTED) {
      String ip = WiFi.localIP().toString();
      body += "<div class='ok'>&#x2705; Connected!<br><br>"
              "<b>Network:</b> " + newSSID + "<br>"
              "<b>IP address:</b> <a href='http://" + ip + "'>" + ip + "</a><br><br>"
              "Save this address to access the device on your network.<br>"
              "Rebooting in 5 seconds...</div>"
              "<script>setTimeout(()=>location.href='http://" + ip + "/',5000)</script>";
    } else {
      body += "<div class='warn'>&#x26A0; Could not connect to <b>" + newSSID + "</b>.<br>"
              "Check the password and try again. Rebooting in AP mode...</div>"
              "<script>setTimeout(()=>location.href='/',4000)</script>";
    }
    body += "</body></html>";
    httpServer.send(200, "text/html", body);
    delay(1500); ESP.restart();
  });

  httpServer.on("/save_pins", HTTP_POST, []() {
    int scl=httpServer.arg("scl").toInt(), sda=httpServer.arg("sda").toInt();
    int rst=httpServer.arg("rst").toInt(), sb=httpServer.arg("sb").toInt();
    int led=httpServer.arg("led").toInt();
    bool ok = scl>0 && sda>0 && rst>=0 && sb>=0 && led>=0;
    ok = ok && scl!=sda && scl!=rst && scl!=sb && scl!=led;
    ok = ok && sda!=rst && sda!=sb  && sda!=led;
    ok = ok && rst!=sb  && rst!=led && sb!=led;
    for (int p:{scl,sda,rst,sb,led}) if(p>=6 && p<=11) ok=false;
    if (!ok) {
      httpServer.send(400, "text/html",
        htmlHead("Error")+"<div class='err'>Invalid or duplicate pins. "
        "GPIO 6-11 reserved. <a href='/config'>Back</a></div></body></html>");
      return;
    }
    savePins(scl,sda,rst,sb,led);
    messagePage("Saved", "ok", "&#x2705; Saved. Rebooting...", "3000");
    delay(1000); ESP.restart();
  });

  httpServer.on("/reset_dsp", HTTP_POST, []() {
    resetDSP();
    messagePage("DSP Reset", "ok", "&#x2705; DSP reset OK.", "2000");
  });

  httpServer.on("/factory_reset", HTTP_POST, []() {
    prefs.begin("tcpi", false); prefs.clear(); prefs.end();
    httpServer.send(200, "text/html",
      htmlHead("Reset")+"<div class='ok'>Cleared. Rebooting in AP mode...</div></body></html>");
    delay(1000); ESP.restart();
  });

  httpServer.on("/status", []() {
    String ip = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    httpServer.send(200, "application/json",
      "{\"dspRunning\":"  + String(dspRunning?"true":"false") +
      ",\"apMode\":"      + String(apMode?"true":"false") +
      ",\"captureReady\":"+ String(captureReady?"true":"false") +
      ",\"captureLen\":"  + String(captureLen) +
      ",\"ip\":\""        + ip + "\"}");
  });

  httpServer.on("/reset_capture", HTTP_POST, []() {
    messagePage("DSP Reset", "ok", "Captured program reseted...", "3000");
    resetEEPROMCapture();
  });
  httpServer.on("/preset_apply", HTTP_POST, []() {
    uint8_t presetlen, cursor = 0;
    uint16_t address, value;
    String presetName;
    presetMap preset;

    if (!httpServer.hasArg("len") || httpServer.arg("len").isEmpty()) {
      httpServer.send(400, "text/plain", "Missing len"); return;
    }
    presetlen = httpServer.arg("len").toInt();

    if (!httpServer.hasArg("name") || httpServer.arg("name").isEmpty()) {
      httpServer.send(400, "text/plain", "Missing name"); return;
    }
    presetName = httpServer.arg("name");
    if (!validatePresetName(presetName)) {
      httpServer.send(400, "text/plain", "Invalid prese name: " + presetName); return;
    }


    if (presetlen == 0) {httpServer.send(400, "text/plain", "Zero len"); return;}

    for (cursor = 0; cursor < presetlen; cursor++) {
        if (!httpServer.hasArg("address" + String(cursor)) ||  httpServer.arg("address" + String(cursor)).isEmpty()) {
          continue;
        }
        if (!httpServer.hasArg("value" + String(cursor)) || httpServer.arg("value" + String(cursor)).isEmpty()) {
          httpServer.send(400, "text/plain", "Missing value on row " + String(cursor)); return;
        }
        address =  strtol(httpServer.arg("address" + String(cursor)).c_str(), NULL, 16);
        if (!serializeBytes(httpServer.arg("value" + String(cursor)), preset[address], PRESET_ROW_LENTGH)) {
          messagePage("Preset apply", "err", hexString(address) +  ": Invalid value - " + httpServer.arg("value" + String(cursor)), "5000");
          return;
        }
    }
    messagePage("Preset apply", "ok", "Preset applyed...", "1000");
    writePreset(presetName, preset);
  });
  httpServer.begin();
  webUIRunning = true;
  Serial.printf("[Web] Server started\n");
}

void unloadHTTP() {
  httpServer.close();
  httpServer.stop();
  webUIRunning = false;
  Serial.printf("[Web] Server stoped\n");
}

String hexString(uint16_t value) {
  static char _str[16];
  sprintf(_str, "0x%x", value);
  return String(_str);
}

// =============================================================
// TCPi PROTOCOL PARSER
// =============================================================
int processBuffer(uint8_t* buf, int len) {
  int pos = 0;
  while (pos < len) {
    uint8_t ctrl = buf[pos];
    if (ctrl == CTRL_WRITE) {
      if (pos + 10 > len) break;
      uint8_t  safeload = buf[pos + 1];
      uint16_t totalLen = (buf[pos + 3] << 8) | buf[pos + 4];
      uint8_t  chipAddr = buf[pos + 5];
      uint16_t dataLen  = (buf[pos + 6] << 8) | buf[pos + 7];
      uint16_t address  = (buf[pos + 8] << 8) | buf[pos + 9];
      if (pos + totalLen > len) { Serial.printf("[TCP] Frag %d/%d\n", len-pos, totalLen); break; }
      uint8_t* payload  = buf + pos + 10;
      Serial.printf("[W] 0x%04X len=%d sl=%d |", address, dataLen, safeload);
      for (int d = 0; d < min((int)dataLen, 8); d++) Serial.printf(" %02X", payload[d]);
      Serial.println();
      handleWrite(chipAddr, address, payload, dataLen, safeload);
      pos += totalLen;
    } else if (ctrl == CTRL_READ_REQ) {
      if (pos + 8 > len) break;
      uint16_t totalLen = (buf[pos+1]<<8)|buf[pos+2];
      uint8_t  chipAddr = buf[pos+3];
      uint16_t dataLen  = (buf[pos+4]<<8)|buf[pos+5];
      uint16_t address  = (buf[pos+6]<<8)|buf[pos+7];
      if (pos + totalLen > len) break;
      handleRead(chipAddr, address, dataLen);
      pos += totalLen;
    } else { pos++; }
  }
  return pos;
}

// =============================================================
// WRITE HANDLER
// =============================================================
void handleWrite(uint8_t chipAddr, uint16_t address, uint8_t* data, uint16_t dataLen, uint8_t safeload) {
  bool isDSP      = (chipAddr == 0x01 || chipAddr == DSP_I2C_ADDR);
  bool isParamRAM = (address <= PARAM_RAM_END);

  uint8_t i2cAddr = (chipAddr == 0x01) ? DSP_I2C_ADDR :
                    (chipAddr == 0x02) ? EEPROM_I2C_ADDR : chipAddr;

  // Capture sl=0 DSP writes BEFORE updating captureReady
  // This ensures the final 0x081C DSPRUN=1 write is included in the capture
  if (isDSP && safeload == 0 && !captureReady) {
    captureWrite(address, data, dataLen);
  }

  // Track DSP state and detect end of Download
  if (isDSP && address == 0x081C && dataLen >= 2) {
    lastCoreCtrl[0] = data[dataLen - 2];
    lastCoreCtrl[1] = data[dataLen - 1];
    bool wasRunning = dspRunning;
    dspRunning = (lastCoreCtrl[1] & 0x04) != 0;
    if (!wasRunning && dspRunning) {
      // DSPRUN just went high = Download complete
      captureReady = true;
      Serial.printf("[DSP] Running! Capture ready: %d bytes\n", captureLen);
    }
  }

  if (isDSP && isParamRAM && dspRunning && safeload == 1) {
    // Safeload: split into 5-word atomic IST chunks
    int totalWords = dataLen / 4, offset = 0, nChunks = 0;
    while (offset < totalWords) {
      int words = min(totalWords - offset, 5);
      safeloadChunk(address + offset, data + offset * 4, words);
      offset += words; nChunks++;
    }
    Serial.printf("[SAFELOAD] 0x%04X %d words %d ISTs\n", address, totalWords, nChunks);
  } else {
    directWrite(i2cAddr, address, data, dataLen);
  }
}

// =============================================================
// SAFELOAD
// =============================================================
void safeloadChunk(uint16_t address, uint8_t* data, int words) {
  for (int i = 0; i < words; i++) {
    uint16_t reg = SAFELOAD_DATA_0 + i;
    uint8_t* w   = data + (i * 4);
    Wire.beginTransmission(DSP_I2C_ADDR);
    Wire.write((reg>>8)&0xFF); Wire.write(reg&0xFF);
    Wire.write(0x00);
    Wire.write(w[0]); Wire.write(w[1]); Wire.write(w[2]); Wire.write(w[3]);
    Wire.endTransmission(true);
  }
  for (int i = 0; i < words; i++) {
    uint16_t reg  = SAFELOAD_ADDR_0 + i;
    uint16_t dest = address + i;
    Wire.beginTransmission(DSP_I2C_ADDR);
    Wire.write((reg>>8)&0xFF); Wire.write(reg&0xFF);
    Wire.write((dest>>8)&0xFF); Wire.write(dest&0xFF);
    Wire.endTransmission(true);
  }
  Wire.beginTransmission(DSP_I2C_ADDR);
  Wire.write(0x08); Wire.write(0x1C);
  Wire.write(lastCoreCtrl[0]);
  Wire.write(lastCoreCtrl[1] | IST_BIT);
  Wire.endTransmission(true);
}

// =============================================================
// CHUNKED I2C WRITE
// =============================================================
void directWrite(uint8_t i2cAddr, uint16_t regAddr, uint8_t* data, uint16_t dataLen) {
  int wordSize = 4;
  if (regAddr >= PROG_RAM_START && regAddr <= PROG_RAM_END) wordSize = 5;
  else if (regAddr >= CTRL_REG_START) wordSize = 2;
  int chunkBytes = 30 * wordSize, offset = 0;
  while (offset < dataLen) {
    int      bytes = min(dataLen - offset, chunkBytes);
    uint16_t addr  = regAddr + (offset / wordSize);
    Wire.beginTransmission(i2cAddr);
    Wire.write((addr>>8)&0xFF); Wire.write(addr&0xFF);
    Wire.write(data + offset, bytes);
    uint8_t err = Wire.endTransmission(true);
    if (err != 0) {
      Serial.printf("[I2C] ERR 0x%04X len=%d err=%d\n", addr, bytes, err);
      #if defined(ESP32)
      Wire.end(); delay(5);
      Wire.setBufferSize(2048);
      Wire.begin(pinSDA, pinSCL, 400000);
      Wire.setTimeOut(50);
      #else // (ESP8266)
      delay(10);
      Wire.begin(pinSDA, pinSCL, 400000);
      Wire.setTimeout(50);
      #endif
    }
    offset += bytes;
  }
}

// =============================================================
// I2C READ
// =============================================================
void handleRead(uint8_t chipAddr, uint16_t address, uint16_t nBytes) {
  uint8_t i2cAddr = (chipAddr==0x01) ? DSP_I2C_ADDR :
                    (chipAddr==0x02) ? EEPROM_I2C_ADDR : chipAddr;
  Serial.printf("[R] 0x%04X\t0x04X\t%d\n", i2cAddr, address, nBytes);
  Wire.beginTransmission(i2cAddr);
  Wire.write((address>>8)&0xFF); Wire.write(address&0xFF);
  Wire.endTransmission(false);
  Wire.requestFrom((int)i2cAddr, (int)nBytes, true);
  uint8_t rd[256]={0}; int idx=0;
  while (Wire.available() && idx<(int)nBytes && idx<256) rd[idx++]=Wire.read();
  uint8_t resp[270]; int sz=9+idx;
  resp[0]=CTRL_READ_RESP;
  resp[1]=(sz>>8)&0xFF; resp[2]=sz&0xFF;
  resp[3]=chipAddr;
  resp[4]=(idx>>8)&0xFF; resp[5]=idx&0xFF;
  resp[6]=(address>>8)&0xFF; resp[7]=address&0xFF;
  resp[8]=0x01;
  memcpy(resp+9,rd,idx);
  if (client && client.connected()) client.write(resp, 9+idx);
}

// =============================================================
void resetDSP() {
  dspRunning = false;
  digitalWrite(pinRESET, LOW); delay(10);
  digitalWrite(pinRESET, HIGH); delay(50);
  Serial.println("[DSP] Reset OK");
}

void scanI2C() {
  Serial.println("[I2C] Scanning:");
  for (uint8_t a=1; a<127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission()==0) Serial.printf("  Found 0x%02X\n", a);
  }
}

void blinkLED(int n) {
  bool wasOn = (digitalRead(pinLED)==HIGH);
  for (int i=0; i<n; i++) {
    digitalWrite(pinLED,HIGH); delay(100);
    digitalWrite(pinLED,LOW);  delay(100);
  }
  if (wasOn || (!apMode && client && client.connected())) digitalWrite(pinLED,HIGH);
}

void resetEEPROMCapture() {
  captureReady  = false;   // true after DSPRUN=1 seen
  for (int _i; _i < (captureLen +1); _i++) captureBuffer[_i] = 0;
  captureLen    = 0;
  Serial.println("[DSP] Capture reset!");
}

void copyPresetRow(uint8_t* in, uint8_t* out) {
  uint8_t i, len;
  out[0] = len = in[0];
  for (i=1; i < len+1 && i < PRESET_ROW_LENTGH; i++) out[i] = in[i];
}

presetMap getPreset(String presetName) {
  presetMap preset;
  presetItem simpledPreset[PRESET_MAX_SIZE];
  uint8_t _i = 0, preset_size;
  prefs.begin(presetName.c_str(), true);
    preset_size = prefs.getInt("size", 0);
    if (preset_size > PRESET_MAX_SIZE) preset_size = PRESET_MAX_SIZE;
    if (preset_size > 0) {
      if (prefs.getBytes("data", simpledPreset, sizeof(presetItem) * preset_size) > 0){
        for (_i = 0; _i < preset_size; _i++) {
          if (simpledPreset[_i].address == 0) break;
          copyPresetRow(simpledPreset[_i].value, preset[simpledPreset[_i].address]);
        }
      }
    }
  prefs.end();
  Serial.printf("[Preset] Reading %s preset from memory. %d keys. Preset size %d\n", presetName.c_str(), _i, preset.size());
  return preset;
}

void writePreset(String presetName, presetMap & presetData) {
  presetItem simpledPreset[PRESET_MAX_SIZE];
  uint8_t _i = 0;
  const char *pn = presetName.c_str();

  for (auto &element: presetData) {
    simpledPreset[_i].address = element.first;
    copyPresetRow(element.second, simpledPreset[_i].value);
    _i++;
    if (_i > PRESET_MAX_SIZE) break;
  }
  if (_i < (PRESET_MAX_SIZE - 1)) {
    simpledPreset[_i].address = 0;
    simpledPreset[_i].value[0] = 0;
  }
  prefs.begin(pn, false);
    prefs.putInt("size", _i);
    prefs.putBytes("data", &simpledPreset, sizeof(presetItem) * _i);
  prefs.end();
  Serial.printf("[Preset] Saving preset '%s' in memory. %d keys\n", pn, _i);
}

bool validatePresetName(String presetName) {
  for (String variant: ALL_PRESETS) if (variant == presetName) return true;
  return false;
}
