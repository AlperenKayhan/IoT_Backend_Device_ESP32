#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <Preferences.h>

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// === Audio & I2S configuration ===
#define I2S_PORT       I2S_NUM_0
#define SAMPLE_RATE    44100      // in Hz
#define TONE_FREQ      1000       // tone frequency in Hz
#define DURATION_MS    200        // how long to play the tone (ms)
#define DMA_BUF_LEN    1024       // number of samples per DMA buffer

// === Configuration ===
#define PING_INTERVAL  5000       // ms between ping events
#define LED_GPIO       12
#define LED_BRIGHT     100
#define LED_COUNT      1


#define DURATION_MS      10000   // total alarm length
#define BEEP_MS            200   // how long each beep lasts
#define PAUSE_MS           500   // silence between beeps
#define BEEP_AMPLITUDE   10000   // lower than 32767 for quieter output


// ─── HC‑SR04 ────────────────────────────────────────────────
#define ULTRASONIC_TRIG_PIN   32     // wired to TRIG
#define ULTRASONIC_ECHO_PIN   33     // wired to ECHO
#define DISTANCE_THRESHOLD_CM 30     // cm to trigger a warning
#define WARNING_THRESHOLD     3      // # of consecutive warnings before reboot

int warningCount = 0;               // global counter

// === Wi‑Fi Credentials ===
const char* WIFI_SSID     = "ALP";
const char* WIFI_PASSWORD = "123456789";

// === Server ===
static const char* SERVER_HOST = "dev-sample.sample.com.tr";

// === Globals ===
Preferences       prefs;
Adafruit_NeoPixel strip(LED_COUNT, LED_GPIO, NEO_GRB + NEO_KHZ800);
SocketIOclient    socketIO;

String g_sessionId = "";

int g_StrugleCount = 0;
int g_SafeMod_Count = 0;
unsigned long lastPingTime = 0;
bool socketConnected = false;
std::string message_register = "";
double lambda = 5.0;
int ErrorSimulationSentinelVal = 0;
int StrugleCount = 0;
int Reboot_COUNT = 0;

int System_ShutDOWN = 0;

// ----------------------------------------------------------------------------
//  Build the /dv/DvOp query path, using stored session_id if any
std::string buildDvOpPath()
{
  String prev = prefs.getString("session_id", "");
  std::string sParam(prev.c_str());
  std::string path = "/devices/deviceopen"; // -> Sample names
  path += "pts=" + std::to_string(millis());
  path += "&S[S]=" + sParam;
  path += "&S[ptof]=180&S[country]=225&S[lang]=tr";
  path += "&S[serial_no]=251306200097";
  path += "&S[serial_no_hw]="  + std::string("724564882000"); 
  path += "&d_short_code=sample_devic";
  path += "&d_firmware=sample_device";
  path += "&d_mac_id="macAddres"";
  path += "&d_local_ip=" + std::string(WiFi.localIP().toString().c_str());
  path += "&d_oper=Prod&d_mdl_id=9000200&d_sites_id=9000200";
  return path;
}

// ----------------------------------------------------------------------------
//  Create a Socket.IO [ eventName, eventData ] message
std::string create_socket_message(const std::string& eventName, const std::string& message)
{
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();
  arr.add(eventName.c_str());
  arr.add(message.c_str());
  std::string out;
  serializeJson(doc, out);
  return out;
}
// ----------------------------------------------------------------------------
//  Initialize the I²S peripheral for 16‑bit stereo at SAMPLE_RATE
void initI2S() {
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = DMA_BUF_LEN,
    .use_apll             = false,
    .tx_desc_auto_clear   = true
  };
  // Pin mapping — adjust these to your wiring if needed
  i2s_pin_config_t pins = {
    .bck_io_num   = 26,
    .ws_io_num    = 27,
    .data_out_num = 22,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

// ----------------------------------------------------------------------------
//  Play a sine‐tone error alert over I²S
void playErrorTone() {
  Serial.println("ERROR TONE PLAYED!!!");

  // Build a single DMA buffer of one square‑wave cycle at reduced amplitude
  static bool inited = false;
  static int16_t buffer[DMA_BUF_LEN * 2];
  if (!inited) {
    const int samplesPerPeriod = SAMPLE_RATE / TONE_FREQ;
    for (int i = 0; i < DMA_BUF_LEN; i += samplesPerPeriod) {
      for (int j = 0; j < samplesPerPeriod && (i + j) < DMA_BUF_LEN; ++j) {
        // square wave: half +amplitude, half –amplitude
        int16_t s = (j < samplesPerPeriod/2) ?  BEEP_AMPLITUDE : -BEEP_AMPLITUDE;
        buffer[2*(i+j)  ] = s;
        buffer[2*(i+j)+1] = s;
      }
    }
    inited = true;
  }

  size_t bytesWritten;
  unsigned long startTime = millis();
  while (millis() - startTime < DURATION_MS) {
    // ——— play one burst ———
    unsigned long beepEnd = millis() + BEEP_MS;
    while (millis() < beepEnd) {
      i2s_write(I2S_PORT,
                buffer,
                sizeof(buffer),
                &bytesWritten,
                pdMS_TO_TICKS(10));
      socketIO.loop();  // keep the socket alive
    }

    // ——— pause between beeps ———
    unsigned long silentEnd = millis() + PAUSE_MS;
    while (millis() < silentEnd) {
      socketIO.loop();
      delay(10);
    }
  }
}
// ─── readDistanceCM() ────────────────────────────────────────
float readDistanceMeter() {
  // send 10 µs pulse
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // measure echo (timeout 30 ms)
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0f;            // no echo
  return (duration * 0.034f) / 2.0f;          // cm
}
// ----------------------------------------------------------------------------
//  Poisson random for error simulation
int poissonRandom(double lambda) {
  double L = exp(-lambda), p = 1.0;
  int k = 0;
  do {
    k++;
    p *= ((double)rand() / RAND_MAX);
  } while (p > L);
  return k - 1;
}

// ----------------------------------------------------------------------------
//  Handle incoming Socket.IO frames
void socketIOEvent(socketIOmessageType_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case sIOtype_DISCONNECT:
      socketConnected = false;
      Serial.println("[IOc] Disconnected!");
      break;

    case sIOtype_CONNECT:
      socketConnected = true;
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      socketIO.send(sIOtype_CONNECT, "/");
      socketIO.sendEVENT(message_register.c_str());
      break;

    case sIOtype_EVENT: {
      String raw = String((char*)payload).substring(0, len);
      DynamicJsonDocument outer(2048);
      if (deserializeJson(outer, raw)) return;

      const char* frameType = outer[0];
      JsonVariant data = outer[1];

      // —— server “pong&ErrorCheck” —— 
      if (strcmp(frameType, "pong") == 0) 
      {
        float dist = readDistanceMeter();
        //float Xn = 0;
        int Anomaly_Count = 0;
        if (ErrorSimulationSentinelVal) 
        {
          StrugleCount++; //Thiss value should be saved inside preferance
          prefs.putUInt("Strugle_Count", StrugleCount);
          Serial.println("WARNING: System UNSTABLE");

          double t = 7.0 * (dist*10) + 3.0;      // dist is your double input
          double Xn = std::fmod(t, 4.0);    // Xn ∈ (–4, +4)

          if (Xn < 0.0) 
              Xn += 4.0;                    // now Xn ∈ [0, 4)
          Serial.printf("Xn: %f", Xn);
          if (Xn > 0.0 && Xn <= 1.5)
          {
            Serial.println("\tCONDITION 1: Propeller Anomaly detected");
            Anomaly_Count++;
            if(Anomaly_Count>20)
            {
              Serial.println("\t\tSYSTEM FAILURE: Auto-Rebooting\n");
              delay(100);
              ESP.restart();
            }
          }

          else if (Xn > 1.5 && Xn <= 2.1)
          {
            Serial.println("\tCONDITION 2: Propeller Issue detected");
            Serial.println("\t\tManuel Reboot Adviced");
            Anomaly_Count++;
            if(Anomaly_Count>15)
            {
              Serial.println("\t\tSYSTEM FAILURE: Auto-Rebooting");
              delay(100);
              ESP.restart();
            }
          }

          else if (Xn > 2.10 && Xn <= 3.10)
          {
            Serial.println("\tCONDITION 3: Auto Reboot Initiliaze");
            Serial.println("\t\tSYSTEM FAILURE: Auto-Rebooting");
            delay(100);
            playErrorTone();
            ESP.restart();

          }

          else
          {
            Serial.println("\tCONDITION 4: Physical Issue Detected !!");
            Serial.println("\t\tEMERGANT FAILURE: System Shutdown");
            playErrorTone();
            loop();
          }
        }

        break;
      }

      // —— application “m” messages —— 
      if (strcmp(frameType, "m") == 0)
      {
        const char* tStr = data["t"];
        DynamicJsonDocument inner(1024);
        if (deserializeJson(inner, tStr)) return;
        const char* cmd = inner["f"];

        if (strcmp(cmd, "send_msg_log") == 0){
          Serial.print("MESSAGE: ");
          Serial.println(inner["msg"] | "");
        }

        else if (strcmp(cmd, "get_d_parameters") == 0) {
          Serial.println("—> Parameters:");
          g_StrugleCount = prefs.getUInt("Strugle_Count", 0);
          Serial.println("    Session ID: " + g_sessionId);
          Serial.printf("    WARNINGS Before REBOOT: %d", + g_StrugleCount);
          Serial.println(" (Level of System Strugle)");
        }

        else if (strcmp(cmd, "reboot") == 0) {
          Serial.println("Reboot command received. Restarting…");
          delay(100);
          ESP.restart();
        }

        else if (strcmp(cmd, "changed_parameters") == 0) {
          Serial.println("Fan Sensors initialize");
          ErrorSimulationSentinelVal = 1;
        }

        else {
          Serial.printf("Unhandled function: %s\n", cmd);
          System_ShutDOWN = 0;
        }

      }
      break;
    }

    case sIOtype_ACK:
      Serial.printf("[IOc] ACK len=%u\n", len);
      break;

    case sIOtype_ERROR:
      Serial.printf("[IOc] ERROR len=%u\n", len);
      break;

    default:
      break;
  }
}


// ----------------------------------------------------------------------------
//  Fetch (or re‑use) session via HTTPS, store it, then start Socket.IO
void sendDeviceOpenRequest() {
  // build & show the URL
  std::string path = buildDvOpPath();
  Serial.println(" Full URL:");
  Serial.println((String("https://") + SERVER_HOST + path.c_str()).c_str());

  // HTTPS GET
  WiFiClientSecure client; client.setInsecure();
  if (!client.connect(SERVER_HOST, 443)) {
    Serial.println(" X Failed to connect");
    strip.setBrightness(LED_BRIGHT); strip.show();
    strip.setPixelColor(0, strip.Color(255,0,0)); strip.show();
    return;
  }
  client.printf("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", path.c_str(), SERVER_HOST);

  // skip headers
  while (client.connected())
  {
    if (client.readStringUntil('\n') == "\r") break;
  }
  // read body
  String payload;
  while (client.available()) payload += client.readString();
  client.stop();

  // parse response
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, payload) || doc["status"] != "succes")
  {
    Serial.println(" X parse error or bad status");
    strip.setBrightness(LED_BRIGHT); strip.show();
    strip.setPixelColor(0, strip.Color(255,0,0)); strip.show();
    return;
  }

  // extract & save session
  g_sessionId = doc["data"]["S"].as<String>();
  prefs.putString("session_id", g_sessionId);
  Serial.println("# New session: " + g_sessionId);

  // prepare Socket.IO
  String cookie = "cookie: S=" + g_sessionId;
  socketIO.setExtraHeaders(cookie.c_str());
  socketIO.beginSSL(SERVER_HOST, 443, "/socket/EIO=4"); //-> sample
  socketIO.onEvent(socketIOEvent);

  // send registration
  std::string dg = "{\"n\":\\".........\"r\":\"develop\"}"; -> Sample
  message_register = create_socket_message("r", dg);
  socketIO.sendEVENT(message_register.c_str());

  // green LED
  strip.setBrightness(LED_BRIGHT); strip.show();
  strip.setPixelColor(0, strip.Color(0,255,0)); strip.show();
}

// --------------------------------SETUP---------------------------------------
void setup() 
{
  Serial.begin(115200);
  strip.begin(); strip.setBrightness(LED_BRIGHT);
  strip.setPixelColor(0, strip.Color(255,255,0)); strip.show();

  // init NVS
  prefs.begin("kodx", false);
  initI2S();
  // connect Wi‑Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int retry = 0;

  // HC‑SR04 pin setup
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  while (WiFi.status() != WL_CONNECTED && retry++ < 20) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n WiFi failed");
    strip.setPixelColor(0, strip.Color(255,0,0)); strip.show();
    return;
  }
  Serial.println("\n WiFi: " + WiFi.localIP().toString());

  // first‑run vs reuse
  if (!prefs.isKey("session_id")) {
    Serial.println("️ First run: obtaining session");
    sendDeviceOpenRequest();
  }
  else {
    g_sessionId = prefs.getString("session_id");
    Serial.println(" Reusing session: " + g_sessionId);
    g_sessionId = prefs.getString("session_id");

    // show URL even on reuse
    std::string path = buildDvOpPath();
    Serial.println(" Full URL:");
    Serial.println((String("https://") + SERVER_HOST + path.c_str()).c_str());

    // start Socket.IO
    String cookie = "cookie: S=" + g_sessionId;
    socketIO.setExtraHeaders(cookie.c_str());
    socketIO.beginSSL(SERVER_HOST, 443, "/socket/EIO=4"); //-> sample
    socketIO.onEvent(socketIOEvent);

    // send registration
    std::string dg = "{\"n\":...................\"develop\\"}; //-> Sample
    message_register = create_socket_message("r", dg);
    socketIO.sendEVENT(message_register.c_str());
  }
}

// --------------------------------LOOP---------------------------------------
void loop() 
{
  if(System_ShutDOWN)
  {exit(0);}
  socketIO.loop();
  if (socketConnected && millis() - lastPingTime >= PING_INTERVAL) {
    std::string pingMsg = create_socket_message("ping", "{}");
    socketIO.sendEVENT(pingMsg.c_str());
    lastPingTime = millis();
  }
}
