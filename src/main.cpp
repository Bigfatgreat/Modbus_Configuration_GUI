#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <vector>

#include <map>
#include <set>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
static SemaphoreHandle_t rs485Mutex;

//---- Pin definitions ----
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 2

//---- Timing intervals ----
constexpr TickType_t MQTT_CHECK_DELAY   = pdMS_TO_TICKS(1000);
constexpr TickType_t RS485_CHECK_DELAY   = pdMS_TO_TICKS(5000);
constexpr TickType_t SENSOR_READ_DELAY   = pdMS_TO_TICKS(10);
constexpr TickType_t SERIAL_CHECK_DELAY  = pdMS_TO_TICKS(50);


std::map<uint8_t, bool> slaveOnlineStatus;  // true = online, false = offline
std::map<uint8_t, uint8_t> slaveFailCount;  // count recent failures
const uint8_t MAX_FAILS = 3;                 // max failures before marking offline


//---- Globals ----
Preferences      preferences;
WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);
ModbusMaster     node;

//---- Data Structures ----
struct SlaveConfig {
  uint8_t  addr;
  uint8_t  func;
  uint16_t startReg;
  uint16_t count;
  String   topic;
  bool     online;
};


struct BusConfig {
  uint32_t baud;
  uint8_t  dataBits;
  char     parity;   // 'N','E','O'
  uint8_t  stopBits;
  bool     rtsCts;
};

struct AppConfig {
  BusConfig                bus;
  String                   wifiSSID;
  String                   wifiPsw;
  String                   mqttServer;
  uint16_t                 mqttPort;
  std::vector<SlaveConfig> slaves;
} config;

//---- Prototypes ----
void loadConfig();
void saveConfig();
void applyBusConfig();
void setupWiFi();
void checkMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void processSerial();
void handleCommand(const String &cmd);
void sendBusCfg();
void sendSlaveList();
void sendStatus();
void checkRS485();
void sendSensorData();
void handleMqttConfigUpdate(const String &msg);
void publishConfig();

// FreeRTOS task functions
void TaskSerial(void *pvParameters);
void TaskMQTT(void *pvParameters);
void TaskRS485Health(void *pvParameters);
void TaskSensorRead(void *pvParameters);

TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t healthTaskHandle = nullptr;
TaskHandle_t mqttTaskHandle   = nullptr;

//---- Setup ----
void setup() {
  Serial.begin(115200);
  
    rs485Mutex = xSemaphoreCreateMutex();
  if (!rs485Mutex) {
    Serial.println("Failed to create RS485 mutex");
    while(true) delay(1000);
  }

  preferences.begin("appConfig", false);
  loadConfig();
  applyBusConfig();
  checkRS485();
  WiFi.mode(WIFI_STA);
  setupWiFi();

  mqttClient.setServer(config.mqttServer.c_str(), config.mqttPort);
  mqttClient.setCallback(mqttCallback);
  checkMQTT();

  

  // Create FreeRTOS tasks
  xTaskCreate(TaskSerial,    "SerialTask",    4096, nullptr, 3, nullptr);
  xTaskCreate(TaskMQTT,      "MQTTTask",      8192, nullptr, 2, &mqttTaskHandle);
  //xTaskCreate(TaskRS485Health,"HealthTask",    4096, nullptr, 1, nullptr);
  xTaskCreate(TaskSensorRead,"SensorTask",    8192, nullptr, 1, &sensorTaskHandle);
  
}

//---- Loop (not used under FreeRTOS) ----
void loop() {
  vTaskDelete(nullptr);
}

//---- Task Definitions ----

void TaskSerial(void *pvParameters) {
  for (;;) {
    if (Serial.available()) {
      processSerial();
    }
    vTaskDelay(SERIAL_CHECK_DELAY);
  }
}

void TaskMQTT(void *pvParameters) {
  for (;;) {
    // Ensure WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[MQTTTask] Reconnecting WiFi...");
      setupWiFi();
    }
    // Ensure MQTT
    if (!mqttClient.connected()) {
      Serial.println("[MQTTTask] Reconnecting MQTT...");
      checkMQTT();
    }
    mqttClient.loop();
    vTaskDelay(MQTT_CHECK_DELAY);
  }
}

void TaskRS485Health(void *pvParameters) {
  for (;;) {
    //checkRS485();
    //vTaskDelay(RS485_CHECK_DELAY);
  }
}

void TaskSensorRead(void *pvParameters) {
  unsigned long lastCheckTime = 0;
  const unsigned long CHECK_INTERVAL = 30 * 1000;  // 30 seconds

  for (;;) {
    sendSensorData();

    // Check if it's time to run checkRS485
    if (millis() - lastCheckTime >= CHECK_INTERVAL) {
      checkRS485();
      lastCheckTime = millis();
    }

    vTaskDelay(SENSOR_READ_DELAY);  // your normal polling delay
  }
}


void setupWiFi() {
  if (config.wifiSSID.isEmpty()) return;
  WiFi.begin(config.wifiSSID.c_str(), config.wifiPsw.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  Serial.printf("WIFI:%s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Failed");
}

void checkMQTT() {
  if (config.mqttServer.isEmpty()) return;
  if (mqttClient.connect("ESP32Client")) {
    mqttClient.subscribe("config/update");
    mqttClient.subscribe("config/get");
    Serial.println("MQTT:Connected");
  } else {
    Serial.println("MQTT:Failed");
  }
}

void processSerial() {
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() > 0) handleCommand(line);
}

void handleCommand(const String &cmd) {
  if (cmd == "GET_BUS_CFG") sendBusCfg();
  else if (cmd.startsWith("SET_BUS_CFG:")) {
    String p = cmd.substring(cmd.indexOf(':') + 1);
    std::vector<String> parts;
    int idx;
    while ((idx = p.indexOf(';')) != -1) {
      parts.push_back(p.substring(0, idx));
      p = p.substring(idx + 1);
    }
    parts.push_back(p);
    config.bus.baud     = parts[0].toInt();
    config.bus.dataBits = parts[1].toInt();
    config.bus.parity   = parts[2][0];
    config.bus.stopBits = parts[3].toInt();
    config.bus.rtsCts   = (parts[4] == "RTS/CTS");
    applyBusConfig();
    saveConfig();
    Serial.println("ACK:SET_BUS_CFG");
  }
  else if (cmd == "GET_SLAVE_LIST") sendSlaveList();
  else if (cmd.startsWith("ADD_SLAVE:")) {
    uint8_t a = cmd.substring(cmd.indexOf(':') + 1).toInt();
    config.slaves.push_back({a, 3, 0, 1, "", false});
    saveConfig();
    Serial.printf("ACK:ADD_SLAVE:%u\n", a);
  }
  else if (cmd.startsWith("REMOVE_SLAVE:")) {
    uint8_t a = cmd.substring(cmd.indexOf(':') + 1).toInt();
    config.slaves.erase(std::remove_if(config.slaves.begin(), config.slaves.end(),
      [a](auto &s){return s.addr == a;}), config.slaves.end());
    saveConfig();
    Serial.printf("ACK:REMOVE_SLAVE:%u\n", a);
  }
  else if (cmd.startsWith("SET_SLAVE_CFG:")) {
    String p = cmd.substring(cmd.indexOf(':') + 1);
    std::vector<String> parts;
    int idx;
    while ((idx = p.indexOf(';')) != -1) {
      parts.push_back(p.substring(0, idx));
      p = p.substring(idx + 1);
    }
    parts.push_back(p);
    uint8_t a = parts[0].toInt();
    for (auto &s : config.slaves) {
      if (s.addr == a) {
        s.func     = parts[1].toInt();
        s.startReg = parts[2].toInt();
        s.count    = parts[3].toInt();
        break;
      }
    }
    saveConfig();
    Serial.printf("ACK:SET_SLAVE_CFG:%u\n", a);
  }
  else if (cmd.startsWith("SET_SLAVE_TOPIC:")) {
    String p = cmd.substring(cmd.indexOf(':') + 1);
    int sep = p.indexOf(';');
    uint8_t a = p.substring(0, sep).toInt();
    String tp = p.substring(sep + 1);
    for (auto &s : config.slaves) {
      if (s.addr == a) { s.topic = tp; break; }
    }
    saveConfig();
    Serial.printf("ACK:SET_SLAVE_TOPIC:%u\n", a);
  }
  else if (cmd == "CHECK_RS485") {
    checkRS485();
    Serial.println("ACK:CHECK_RS485");
  }
  else if (cmd == "GET_STATUS") sendStatus();
  else if (cmd.startsWith("SET_WIFI:")) {
    String p = cmd.substring(cmd.indexOf(':') + 1);
    int idx = p.indexOf(',');
    config.wifiSSID = p.substring(0, idx);
    config.wifiPsw  = p.substring(idx + 1);
    preferences.putString("wifiSSID", config.wifiSSID);
    preferences.putString("wifiPsw", config.wifiPsw);
    setupWiFi();  // Reconnect with new credentials
    Serial.println("ACK:SET_WIFI");
  }
  else if (cmd.startsWith("SET_MQTT:")) {
    // Extract the MQTT server and port from the command
    String p = cmd.substring(cmd.indexOf(':') + 1);
    int idx = p.indexOf(';');
    if (idx > 0) {
      config.mqttServer = p.substring(0, idx);  // MQTT Server
      config.mqttPort   = p.substring(idx + 1).toInt(); // MQTT Port
      saveConfig();  // Save the updated config to preferences
      Serial.println("ACK:SET_MQTT");
    }
  }
  else if (cmd == "PREF_RESET") {
  preferences.clear();
  config.slaves.clear();
  config.bus = {};  // reset bus config too if needed
  // reset other config members as well
  Serial.println("ACK:PREF_RESET");
}
  else if (cmd.startsWith("SET_SLAVE_LIST:")) {
  String payload = cmd.substring(cmd.indexOf(':') + 1);
  config.slaves.clear();

  int start = 0;
  while (start < (int)payload.length()) {
    int sep = payload.indexOf('|', start);
    String item = sep == -1 ? payload.substring(start) : payload.substring(start, sep);

    std::vector<String> parts;
    int idx;
    while ((idx = item.indexOf(';')) != -1) {
      parts.push_back(item.substring(0, idx));
      item = item.substring(idx + 1);
    }
    parts.push_back(item);

    if (parts.size() >= 5) {
      SlaveConfig s;
      s.addr     = parts[0].toInt();
      s.func     = parts[1].toInt();
      s.startReg = parts[2].toInt();
      s.count    = parts[3].toInt();
      s.topic    = parts[4];
      s.online   = false;
      config.slaves.push_back(s);
    }

    if (sep == -1) break;
    start = sep + 1;
  }

  saveConfig();
  Serial.println("ACK:SET_SLAVE_LIST");
}


}

void sendBusCfg() {
  Serial.printf("CFG:BUS:%lu,%u,%c,%u,%s\n",
    config.bus.baud,
    config.bus.dataBits,
    config.bus.parity,
    config.bus.stopBits,
    config.bus.rtsCts ? "RTS/CTS" : "None");
}

void sendSlaveList() {
  for (auto &s : config.slaves) {
    Serial.printf("CFG:SLAVE:%u,%u,%u,%u,%s\n",
      s.addr, s.func, s.startReg, s.count, s.topic.c_str());
  }
  Serial.println("END_SLAVE_LIST");
}

void sendStatus() {
  Serial.printf("WIFI:%s\n", WiFi.status()==WL_CONNECTED?"Connected":"Disconnected");
  Serial.printf("MQTT:%s\n", mqttClient.connected()?"Connected":"Disconnected");
  sendBusCfg();
}

//---- MQTT Handling ----
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];
  String t = topic;
  if (t == "config/update")      handleMqttConfigUpdate(msg);
  else if (t == "config/get")    publishConfig();
}

void handleMqttConfigUpdate(const String &msg) {
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, msg)) {
    mqttClient.publish("config/update/ack", "error");
    return;
  }
  auto jb = doc["bus"].as<JsonObject>();
  config.bus.baud     = jb["baud"].as<uint32_t>();
  config.bus.dataBits = jb["dataBits"].as<uint8_t>();
  const char* p = jb["parity"].as<const char*>();
  config.bus.parity   = p && *p ? p[0] : 'N';
  config.bus.stopBits = jb["stopBits"].as<uint8_t>();
  config.bus.rtsCts   = jb["rtsCts"].as<bool>();

  auto jw = doc["wifi"].as<JsonObject>();
  config.wifiSSID     = jw["ssid"].as<String>();
  config.wifiPsw      = jw["psw"].as<String>();

  auto jm = doc["mqtt"].as<JsonObject>();
  config.mqttServer   = jm["server"].as<String>();
  config.mqttPort     = jm["port"].as<uint16_t>();

  config.slaves.clear();
  for (auto js : doc["slaves"].as<JsonArray>()) {
    SlaveConfig s;
    s.addr     = js["addr"].as<uint8_t>();
    s.func     = js["func"].as<uint8_t>();
    s.startReg = js["startReg"].as<uint16_t>();
    s.count    = js["count"].as<uint16_t>();
    s.topic    = js["topic"].as<String>();
    s.online   = false;
    config.slaves.push_back(s);
  }

  saveConfig();
  applyBusConfig();
  mqttClient.publish("config/update/ack", "success");
}

void publishConfig() {
  DynamicJsonDocument doc(4096);
  auto b = doc.createNestedObject("bus");
  b["baud"]     = config.bus.baud;
  b["dataBits"] = config.bus.dataBits;
  b["parity"]   = String(config.bus.parity);
  b["stopBits"] = config.bus.stopBits;
  b["rtsCts"]   = config.bus.rtsCts;

  auto w = doc.createNestedObject("wifi");
  w["ssid"] = config.wifiSSID; w["psw"] = config.wifiPsw;

  auto m = doc.createNestedObject("mqtt");
  m["server"] = config.mqttServer; m["port"] = config.mqttPort;

  auto arr = doc.createNestedArray("slaves");
  for (auto &s: config.slaves) {
    auto o = arr.createNestedObject();
    o["addr"]     = s.addr;
    o["func"]     = s.func;
    o["startReg"] = s.startReg;
    o["count"]    = s.count;
    o["topic"]    = s.topic;
  }

  String out;
  serializeJson(doc, out);
  mqttClient.publish("config/response", out.c_str());
}

//---- Persistence ----
void loadConfig() {
  String js = preferences.getString("configJson", "");
  if (!js.isEmpty()) {
    DynamicJsonDocument doc(4096);
    if (!deserializeJson(doc, js)) {
      auto jb = doc["bus"].as<JsonObject>();
      config.bus.baud     = jb["baud"];
      config.bus.dataBits = jb["dataBits"];
      const char* p = jb["parity"].as<const char*>();
      config.bus.parity   = p && *p? p[0] : 'N';
      config.bus.stopBits = jb["stopBits"];
      config.bus.rtsCts   = jb["rtsCts"];

      config.wifiSSID   = doc["wifi"]["ssid"].as<String>();
      config.wifiPsw    = doc["wifi"]["psw"].as<String>();
      config.mqttServer = doc["mqtt"]["server"].as<String>();
      config.mqttPort   = doc["mqtt"]["port"];

      config.slaves.clear();
      for (auto js: doc["slaves"].as<JsonArray>()) {
        SlaveConfig s;
        s.addr     = js["addr"];
        s.func     = js["func"];
        s.startReg = js["startReg"];
        s.count    = js["count"];
        s.topic    = js["topic"].as<String>();
        s.online   = false;
        config.slaves.push_back(s);
      }
      return;
    }
  }
  // No saved config -> use your hard‐coded defaults:
  config.bus       = {9600, 8, 'N', 1, false};
  config.wifiSSID  = "";
  config.wifiPsw   = "";
  config.mqttServer= "";
  config.mqttPort  = 1883;

  // **Seed your one vibration slave here**:
  config.slaves.clear();
  config.slaves.push_back({
    1,       // addr
    3,       // func (read holding regs)
    0x34,    // startReg
    3,       // count
    "sensors/acc",
    false    // online
  });
  config.slaves.push_back({
    1,       // addr
    3,       // func (read holding regs)
    0x01,    // startReg
    3,       // count
    "sensors/vel",
    false    // online
  });
}

void saveConfig() {
  DynamicJsonDocument doc(4096);
  auto b = doc.createNestedObject("bus");
  b["baud"]     = config.bus.baud;
  b["dataBits"] = config.bus.dataBits;
  b["parity"]   = String(config.bus.parity);
  b["stopBits"] = config.bus.stopBits;
  b["rtsCts"]   = config.bus.rtsCts;

  auto w = doc.createNestedObject("wifi");
  w["ssid"] = config.wifiSSID; w["psw"]  = config.wifiPsw;

  auto m = doc.createNestedObject("mqtt");
  m["server"] = config.mqttServer; m["port"] = config.mqttPort;

  auto arr = doc.createNestedArray("slaves");
  for (auto &s: config.slaves) {
    auto o = arr.createNestedObject();
    o["addr"]     = s.addr;
    o["func"]     = s.func;
    o["startReg"] = s.startReg;
    o["count"]    = s.count;
    o["topic"]    = s.topic;
  }

  String out;
  serializeJson(doc, out);
  preferences.putString("configJson", out);
}

//---- RS-485 Bus Setup ----
void applyBusConfig() {
  uint32_t flags = SERIAL_8N1;
  if (config.bus.dataBits == 7) {
    if (config.bus.parity == 'E') flags = (config.bus.stopBits==2? SERIAL_7E2: SERIAL_7E1);
    else if (config.bus.parity == 'O') flags = (config.bus.stopBits==2? SERIAL_7O2: SERIAL_7O1);
    else                           flags = (config.bus.stopBits==2? SERIAL_7N2: SERIAL_7N1);
  } else {
    if (config.bus.parity == 'E') flags = (config.bus.stopBits==2? SERIAL_8E2: SERIAL_8E1);
    else if (config.bus.parity == 'O') flags = (config.bus.stopBits==2? SERIAL_8O2: SERIAL_8O1);
    else                           flags = (config.bus.stopBits==2? SERIAL_8N2: SERIAL_8N1);
  }
  Serial2.begin(config.bus.baud, flags, RS485_RX_PIN, RS485_TX_PIN);
}

//---- Health Check ----
void checkRS485() {
  std::set<uint8_t> seen;
  std::vector<uint8_t> uniqueAddrs;

  // 1. Collect unique slave addresses
  for (auto &s : config.slaves) {
    if (seen.find(s.addr) == seen.end()) {
      uniqueAddrs.push_back(s.addr);
      seen.insert(s.addr);
    }
  }

  // 2. Loop through each unique slave address once
  for (auto addr : uniqueAddrs) {
    node.begin(addr, Serial2);
    xSemaphoreTake(rs485Mutex, portMAX_DELAY);
    uint8_t res = node.readHoldingRegisters(0x0000, 1);  // Use minimal ping
    xSemaphoreGive(rs485Mutex);

    if (res == node.ku8MBSuccess) {
      slaveOnlineStatus[addr] = true;
      slaveFailCount[addr] = 0;
      Serial.printf("STATUS:SLAVE:%u,ONLINE\n", addr);
    } else {
      slaveFailCount[addr]++;
      if (slaveFailCount[addr] >= MAX_FAILS) {
        slaveOnlineStatus[addr] = false;
        Serial.printf("STATUS:SLAVE:%u,OFFLINE\n", addr);
      } else {
        Serial.printf("STATUS:SLAVE:%u,UNKNOWN\n", addr);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // prevent bus flooding
  }
}



//---- Sensor Read & Publish ----

//#include <algorithm>   // for std::find_if

void sendSensorData() {
  if (!mqttClient.connected()) return;

  for (auto &s : config.slaves) {
    // Skip offline slaves
    if (slaveOnlineStatus.find(s.addr) != slaveOnlineStatus.end() && !slaveOnlineStatus[s.addr]) {
      continue;
    }

    xSemaphoreTake(rs485Mutex, portMAX_DELAY);
    node.begin(s.addr, Serial2);
    digitalWrite(RS485_DE_RE_PIN, HIGH);

    uint8_t res = node.ku8MBIllegalFunction;

    switch (s.func) {
      case 1: // Read Coils
        res = node.readCoils(s.startReg, s.count);
        break;
      case 2: // Read Discrete Inputs
        res = node.readDiscreteInputs(s.startReg, s.count);
        break;
      case 3: // Read Holding Registers
        res = node.readHoldingRegisters(s.startReg, s.count);
        break;
      case 4: // Read Input Registers
        res = node.readInputRegisters(s.startReg, s.count);
        break;
      default:
        Serial.printf("Unsupported func %u for slave %u\n", s.func, s.addr);
        break;
    }

    digitalWrite(RS485_DE_RE_PIN, LOW);
    xSemaphoreGive(rs485Mutex);

    if (s.topic.length() == 0) {
      Serial.printf("Skipping slave %u: empty topic\n", s.addr);
      continue;
    }

    if (res == node.ku8MBSuccess) {
      // Reset fail count and mark online
      slaveFailCount[s.addr] = 0;
      slaveOnlineStatus[s.addr] = true;

            String funcStr;
      switch (s.func) {
        case 1: funcStr = "coils"; break;
        case 2: funcStr = "discrete_inputs"; break;
        case 3: funcStr = "holding_registers"; break;
        case 4: funcStr = "input_registers"; break;
        default: funcStr = "unknown"; break;
      }

      StaticJsonDocument<256> doc;
      doc["type"] = funcStr;   // Add the type field

      for (uint16_t i = 0; i < s.count; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "r%u", i);
                // Coils and discrete inputs are bits packed in registers, getResponseBuffer returns uint16_t
                // Bit extraction
        if (s.func == 1 || s.func == 2) {
          uint16_t coilReg = node.getResponseBuffer(i / 16);
          uint8_t bitPos = i % 16;
          bool coilVal = (coilReg >> bitPos) & 0x01;
          doc[key] = coilVal;
        } else {
          doc[key] = node.getResponseBuffer(i);
        }
      }

      String payload;
      serializeJson(doc, payload);
      mqttClient.publish(s.topic.c_str(), payload.c_str());

      // Optional debug:
      // Serial.printf("%u→ %s : %s\n", s.addr, s.topic.c_str(), payload.c_str());
    } else {
      Serial.printf("Read failed on addr %u func %u @0x%X count %u\n", s.addr, s.func, s.startReg, s.count);

      slaveFailCount[s.addr]++;
      if (slaveFailCount[s.addr] >= MAX_FAILS) {
        slaveOnlineStatus[s.addr] = false;
        Serial.printf("STATUS:SLAVE:%u,OFFLINE\n", s.addr);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
