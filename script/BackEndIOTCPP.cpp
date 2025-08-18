#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <Preferences.h>

#include <driver/i2s.h> //I2S library; included for us to use alarm sound
#include <freertos/FreeRTOS.h>//SSL/TLS
#include <freertos/task.h>
#include <math.h>//For poisen calculation, we need to include it

/*######## Audio & I2S configuration - START ########*/
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 44100 // Amount of Heartz(Hz)
#define TONE_FREQ 1000 // Our Tone's frequency in Hz
#define DURATION_MS 200 // milisecond time for our tone to be played.
#define DMA_BUF_LEN 1024 // amount of samples that per that included DMA buffer
/*######## Audio & I2S configuration - END ##########*/

/*######## LED Configration  Configuration - START ########*/
/*We did not use this part after, we decide to not use to our system.
Due to later found it unnecessary*/
#define PING_INTERVAL  5000 // Ping interval is distance between ping events.
#define LED_GPIO       12
#define LED_BRIGHT     100
#define LED_COUNT      1
/*######## LED Configration  Configuration - END ########*/

/*######## ALARM Tuning Setting - START ########*/
/*NOTE THAT: when the alaram duration is over it is still contunio to run.
Because, Alarm sound is in the loop that make the soun run on a bit longer*/
#define DURATION_MS      10000 // total alarm duration
#define BEEP_MS            200   // how long each beep lasts
#define PAUSE_MS           500   // silence between beeps
#define BEEP_AMPLITUDE   10000   // lower than 32767 for quieter output
/*######## ALARM Tuning Setting - END ########*/

/*########  HC‑SR04 - START ########*/
/* HC-SR04 sensor is a sensor that allowed us to read distance between sensor the fan blade.
Sensor read the distance at 5 second time intervall*/
#define ULTRASONIC_TRIG_PIN   32 // it is  a pin locaation for TRIG(Trigger)
#define ULTRASONIC_ECHO_PIN   33 // it is  a pin locaation for  ECHO
#define DISTANCE_THRESHOLD_CM 30 // if the threshold did not excited amount of cm to trigger a warning
#define WARNING_THRESHOLD     3 // warnings before reboot
/*########  HC‑SR04 - END ########*/

/*########  Global Variables - START ########*/
/* Tese are the global variables that we utilize in our code. Which their existance are
multifunction due to that nature. We declare them on a global to also reach them easly */
int warningCount = 0; //Warninig Counter
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
String g_sessionId = "";

/*########  Global Variables - END ##########*/

/*########  WiFi Declarations - START ########*/
/* Here we declare our character pointers to record our used SSID & Passsword. */
const char* WIFI_SSID     = "ALP";
const char* WIFI_PASSWORD = "123456789";
/*#########  WiFi Declarations - END #########*/

/*######## SERVER HOST - START ########*/
/* Declaration of server host. That we use in this case, I am using a different name to not 
show the true name. Due to sportsmanship towards the company that I interned.*/ 
static const char* SERVER_HOST = "dev-sample.sample.com.tr";
/*######## SERVER HOST - END ########*/

/*######## Global Elements - START ########*/
/* This is where we declared the names of the used 
variables that will later implenmented on our code*/
Preferences prefs; /* we add the preferences to record our seassion_id on device */
Adafruit_NeoPixel strip(LED_COUNT, LED_GPIO, NEO_GRB + NEO_KHZ800);/*LED description that exists previous imğlementatipon*/
SocketIOclient socketIO; /*Socket io implementation that we need to use to make Web Socket connection*/
/*######### Global Elements - END #########*/



/* ###################### URL build - START ######################*/
std::string buildDvOpPath()
{/* to send an open request to the database, we need to first build our own URL with
various parameters. Some names may be inconsistent due to not sharing company methods in detail.*/
  String prev = prefs.getString("session_id", "");/* to get our seassion id we firstly called session_id from "prefs" */
  std::string sParam(prev.c_str());
  std::string path = "/devices/dveOpen"; // -> Sample names
  path += "pts=" + std::to_string(millis());
  path += "&S[S]=" + sParam; //Inserting Seassion id that readed aon prefs
  path += "&S[ptof]=180&S[country]=225&S[lang]=tr";// essantial descirebed
  path += "&S[serial_no]=251306200097";//device id, whic we need to record our device's oın ERP system
  path += "&S[serial_no_hw]="  + std::string("724564882000"); 
  path += "&d_short_code=sample_devic";//Device name -> important for ERP to recognize
  path += "&d_firmware=sample_device";//Device name -> important for ERP to recognize
  path += "&d_mac_id="macAddres""; //Unique Device MAC Address. 
  path += "&d_local_ip=" + std::string(WiFi.localIP().toString().c_str()); //Unige WiFi IP
  path += "&d_oper=Prod&d_mdl_id=9000200&d_sites_id=9000200";
  return path;
}
/* ###################### URL build - END ######################*/

/* ###################### Socket.IO Creation - START ######################*/
std::string create_socket_message(const std::string& eventName, const std::string& message)
{// To create the socket message in sending as out to process as message later.
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();
  arr.add(eventName.c_str());
  arr.add(message.c_str());
  std::string out;
  serializeJson(doc, out);
  return out;
}
/* ###################### Socket.IO Creation - END ######################*/

/* ###################### initI2S - START ######################*/
/*initI2S is to used to generate analog electiracal pulses to help us 
to create binary alarm sound. Which, we used "MAX98357A" and then send the generated
sound to speakers. */
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
/* ###################### initI2S - END ######################*/


/* ###################### playErrorTone - START ######################*/
//  Play a sine‐tone error alert over I²S
void playErrorTone() 
{/*In this function, we played the sine-time error alert over I2S. Thus, we build
  single DMA buffer of one square-wave cycle at reduced amplitude*/
  Serial.println("ERROR TONE PLAYED!!!");// Indicator that "Error tone is playing"
  static bool inited = false;
  static int16_t buffer[DMA_BUF_LEN * 2];
  if (!inited) {
    const int samplesPerPeriod = SAMPLE_RATE / TONE_FREQ;
    for (int i = 0; i < DMA_BUF_LEN; i += samplesPerPeriod) 
    {
      for (int j = 0; j < samplesPerPeriod && (i + j) < DMA_BUF_LEN; ++j) 
      {// square wave: half  positive amplitude and  half  negative one.
        int16_t s = (j < samplesPerPeriod/2) ?  BEEP_AMPLITUDE : -BEEP_AMPLITUDE;
        buffer[2*(i+j)  ] = s;
        buffer[2*(i+j)+1] = s;
      }
    }
    inited = true;
  }

  size_t bytesWritten;
  unsigned long startTime = millis();
  while (millis() - startTime < DURATION_MS)
  {
    unsigned long beepEnd = millis() + BEEP_MS;
    while (millis() < beepEnd) 
    {
      i2s_write(I2S_PORT, buffer, sizeof(buffer), &bytesWritten, pdMS_TO_TICKS(10));
      socketIO.loop();  //Helper to keep socket allive
    }

    // It makes a pause between beeps!!
    unsigned long silentEnd = millis() + PAUSE_MS;
    while (millis() < silentEnd) 
    {
      socketIO.loop();
      delay(10);
    }
  }
}
/* ###################### playErrorTone - END ######################*/



/* ###################### ReadDistance - Start ######################*/
float readDistanceMeter()
{/* Commanly used distance reader*/
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0f;
  return (duration * 0.034f) / 2.0f;
}
/* ###################### ReadDistance - END ######################*/
// ----------------------------------------------------------------------------
//  Poisson random for error simulation
/* ###################### Poisson Random - START ######################*/
int poissonRandom(double lambda)
{/* we use this function to generate our random values to between in the possible interval
  as well as allowed us to generate distinctive values. Thus, it might be possible to 
  have libraries that allowed us to generate 64 format random values to make them even more 
  related &distinctive. Yet, adding those libraries may increase the weight of our ESP32 code. 
  Which we don't want because even this version is very heavy.*/

  /* # In future Qt/C++ implementation, there is a  high possibility that I'll be using 
  "QRandomGenerator64" to generate my random values. */
  double L = exp(-lambda), p = 1.0;
  int k = 0;
  do
  {
    k++;
    p *= ((double)rand() / RAND_MAX);
  } while (p > L);
  return k - 1;
}

// ----------------------------------------------------------------------------
//  Handle incoming Socket.IO frames
void socketIOEvent(socketIOmessageType_t type, uint8_t* payload, size_t len) {
  switch (type){
    case sIOtype_DISCONNECT://if our socket connection is disconnected either from a network or the ERP itself
      socketConnected = false;
      Serial.println("[IOc] Disconnected!");
      break;

    case sIOtype_CONNECT:// if connection successful.
      socketConnected = true;
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      socketIO.send(sIOtype_CONNECT, "/");
      socketIO.sendEVENT(message_register.c_str());
      break;

    case sIOtype_EVENT: 
      {/*After a succesful connection, we are arriving at "SIOtype_Envent". Thus, we process the 
      raw data that we got from payload to process. Then, we spearete our raw data in to two parts
      frameType and data itsel.*/ 
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

