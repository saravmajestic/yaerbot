#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include "soc/soc.h"           // for brownout detector register
#include "soc/rtc_cntl_reg.h"  // RTC_CNTL_BROWN_OUT_REG
#include <ESPmDNS.h>           // advertise farmcam.local so the UNO Q finds us by name
#include <WebServer.h>         // diagnostics server (/status, /log) on port 82

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// WiFi networks — TWO only: the robot's own hotspot and home WiFi.
// Fill in the real SSID/password before flashing (keep placeholders in git).
// FarmOS-AP is open (no password) → pass "" as the key.
// ===========================
WiFiMulti wifiMulti;

static void wifiAddNetworks() {
  // WiFiMulti joins the STRONGEST configured network (RSSI-based, not list order),
  // so in the field the robot's own hotspot wins simply by being closest.
  wifiMulti.addAP("FarmOS-AP", "");            // robot's own hotspot (open)
  wifiMulti.addAP("HOME_SSID", "HOME_PW");     // home WiFi (fill before flashing)
}

void startCameraServer();
void setupLedFlash();

// ── Network diagnostics ─────────────────────────────────────────────────────
// Makes the camera debuggable over WiFi (no serial bridge). /status (JSON) and
// /log (recent boot log) on port 82. If WiFi can't be joined, a SoftAP fallback
// ("FarmCam-Diag") comes up so these are ALWAYS reachable at 192.168.4.1:82.
WebServer diagServer(82);
String    diagBuf;                          // recent log lines (trimmed ring)

// ── Log timestamps ──────────────────────────────────────────────────────────
// The ESP32-CAM has no RTC or battery, so wall-clock time only exists after an
// NTP sync — and on the field hotspot (FarmOS-AP) there is no internet at all.
// So: real date-time when we have it, uptime otherwise. Uptime still orders
// events and measures gaps, which is what matters for a brownout/reboot hunt.
#define TZ_OFFSET_SEC  (5 * 3600 + 1800)    // IST = UTC+05:30 (farm is Salem, TN)
static bool timeSynced = false;

// WiFi keep-alive counters (declared here: handleStatus() reports them)
static unsigned long s_wifi_check_ms = 0;
static uint32_t      s_reconnects    = 0;

static String logStamp() {
  if (timeSynced) {
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char buf[20];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
    return String(buf);
  }
  uint32_t ms = millis();
  char buf[20];
  snprintf(buf, sizeof(buf), "+%lu.%03lus",
           (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
  return String(buf);
}

void diagLog(const String &s) {
  String line = "[" + logStamp() + "] " + s;
  Serial.println(line);
  diagBuf += line; diagBuf += "\n";
  if (diagBuf.length() > 4000) {            // bigger ring: stamps make lines longer
    diagBuf.remove(0, diagBuf.length() - 4000);
    int nl = diagBuf.indexOf('\n');         // drop the partial line the cut left
    if (nl >= 0) diagBuf.remove(0, nl + 1);
  }
}

// Kick off an NTP sync; non-blocking beyond a short bounded wait so a network
// without internet (the robot hotspot) can never stall boot.
static void syncTime() {
  configTime(TZ_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  unsigned long t0 = millis();
  while (time(nullptr) < 1700000000 && millis() - t0 < 3000) delay(100);
  timeSynced = (time(nullptr) >= 1700000000);   // sane epoch => clock is real
  diagLog(timeSynced ? "NTP synced — logs now wall-clock (IST)"
                     : "NTP unavailable — logs stay uptime-relative");
}

static const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";   // <-- the one we kept chasing
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_EXT:       return "EXT";
    default:                return "OTHER";
  }
}

static void handleStatus() {
  bool ap = (WiFi.status() != WL_CONNECTED);
  String j = "{";
  j += "\"ssid\":\"" + (ap ? String("FarmCam-Diag") : WiFi.SSID()) + "\",";
  j += "\"mode\":\"" + String(ap ? "softap" : "sta") + "\",";
  j += "\"rssi\":" + String(ap ? 0 : WiFi.RSSI()) + ",";
  j += "\"ip\":\"" + (ap ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
  j += "\"uptime_s\":" + String(millis() / 1000) + ",";
  j += "\"time\":\"" + logStamp() + "\",";          // wall-clock if NTP synced, else uptime
  j += "\"time_synced\":" + String(timeSynced ? "true" : "false") + ",";
  j += "\"reconnects\":" + String(s_reconnects) + ",";   // WiFi flapping?
  j += "\"heap_free\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"heap_min\":" + String(esp_get_minimum_free_heap_size()) + ",";
  j += "\"reset_reason\":\"" + String(resetReasonStr()) + "\",";
  j += "\"psram\":" + String(psramFound() ? "true" : "false") + ",";
  j += "\"camera\":\"" + String(esp_camera_sensor_get() ? "ok" : "fail") + "\"";
  j += "}";
  diagServer.sendHeader("Access-Control-Allow-Origin", "*");
  diagServer.send(200, "application/json", j);
}

static void handleLog() {
  diagServer.sendHeader("Access-Control-Allow-Origin", "*");
  diagServer.send(200, "text/plain", diagBuf.length() ? diagBuf : String("(no log yet)\n"));
}

static void startDiagServer() {
  diagServer.on("/status", handleStatus);
  diagServer.on("/log", handleLog);
  diagServer.begin();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout detector (ESP32-CAM runs on a marginal 5V rail)
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;  // 20MHz — full frame rate (power is now a solid buck + cap;
                                   // drop to 10000000 only if the camera-init spike browns out)
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  diagLog(String("boot: reset_reason=") + resetReasonStr());
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  wifiAddNetworks();

  diagLog("WiFi connecting (FarmOS-AP / home WiFi, strongest wins)...");
  unsigned long wifiStart = millis();
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart > 25000) {            // 25s -> SoftAP fallback
      diagLog("WiFi FAILED after 25s -> SoftAP 'FarmCam-Diag' @ 192.168.4.1 (open)");
      WiFi.mode(WIFI_AP);
      WiFi.softAP("FarmCam-Diag");
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    diagLog("WiFi connected: " + WiFi.SSID() + "  IP=" + WiFi.localIP().toString()
            + "  RSSI=" + String(WiFi.RSSI()));
    syncTime();                                    // wall-clock stamps if there's internet
    startMdns();                                   // reachable as farmcam.local
  }

  startCameraServer();
  startDiagServer();
  diagLog("servers up: cam :80 + :81/stream, diag :82/status /log");
}

// ── Keep WiFi alive ─────────────────────────────────────────────────────────
// wifiMulti.run() used to be called ONCE in setup() and never again, so any
// momentary dropout (AP reboot, interference, the robot driving out of range)
// left the cam offline until someone power-cycled it — observed in the field
// 2026-08-11. Re-check here and rejoin. mDNS must be restarted after a rejoin
// because the IP can change.
static void startMdns() {
  MDNS.end();
  if (MDNS.begin("farmcam")) {
    MDNS.addService("http", "tcp", 80);
    diagLog("mDNS: farmcam.local");
  }
}

static void wifiTend() {
  const bool softap = (WiFi.getMode() == WIFI_AP);
  // STA: check often. SoftAP fallback: retry STA occasionally, so a cam that
  // booted while the AP was down still rejoins on its own instead of sitting
  // on FarmCam-Diag forever.
  const unsigned long every = softap ? 60000UL : 5000UL;
  if (millis() - s_wifi_check_ms < every) return;
  s_wifi_check_ms = millis();
  if (!softap && WiFi.status() == WL_CONNECTED) return;

  diagLog(softap ? "SoftAP: retrying home WiFi..." : "WiFi lost — rejoining...");
  if (softap) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
  }
  if (wifiMulti.run() == WL_CONNECTED) {
    s_reconnects++;
    diagLog("WiFi reconnected: " + WiFi.SSID() + "  IP=" + WiFi.localIP().toString()
            + "  RSSI=" + String(WiFi.RSSI()));
    startMdns();
    if (!timeSynced) syncTime();          // first sync may have missed its window
  } else if (softap) {
    WiFi.mode(WIFI_AP);                   // still no luck — keep the escape hatch up
    WiFi.softAP("FarmCam-Diag");
  }
}

void loop() {
  // Camera server runs in its own task; here we just service the diag server.
  diagServer.handleClient();
  wifiTend();
  delay(2);
}
