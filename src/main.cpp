// ESP32-S3 Environmental Station — firmware for the live website (version 3)
// Framework: Arduino on PlatformIO. FreeRTOS is built in.
//
// FreeRTOS features used, and the real job each one does:
//   Tasks + cores       SensorTask, CommandTask, ButtonTask on core 1; MqttTask (and Wi-Fi) on core 0
//   Queues              dataQueue  SensorTask  -> MqttTask     (readings)
//                       cmdQueue   MqttTask    -> CommandTask  (commands from the website)
//                       outQueue   CommandTask / ButtonTask -> MqttTask (replies + button events)
//   Mutex               serialMutex: all tasks print to the same Serial port; the mutex stops lines mixing
//   Binary semaphore    buttonSem: the BOOT button interrupt (ISR) gives it, ButtonTask takes it
//   Interrupt (ISR)     buttonISR runs the instant GPIO 0 goes LOW
//   Task notifications  ButtonTask -> SensorTask ("read now"), sysTimer -> MqttTask ("send report")
//   Event group         netEvents: WIFI_OK and MQTT_OK bits; SensorTask waits for WIFI_OK before starting
//   Software timer      sysTimer: every 5 s takes a snapshot of all tasks + CPU load, notifies MqttTask
//   Idle hooks          count idle ticks per core to estimate CPU load (approximate)
//
// Only MqttTask touches the MQTT client, so the client itself needs no mutex.
// Every reading, reply and button press carries a "trace": [function, task, start µs, µs taken].
//
// Topics (PREFIX must match the website):
//   PREFIX/telemetry, /system, /ack, /button, /event, /status   ESP32 -> computer
//   PREFIX/cmd                                                    computer -> ESP32

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_freertos_hooks.h"
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>

// ------------------------------------------------------------------ settings
const char* WIFI_SSID = "WIFI_NAME";
const char* WIFI_PASS = "PASSWORD";
const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* PREFIX = "iitp/priyanshi/envstation";

const int BUTTON_PIN = 0;              // BOOT button on ESP32-S3 DevKit boards
const uint32_t SYSTEM_EVERY_MS = 5000;
const int DATA_QUEUE_LEN = 10, CMD_QUEUE_LEN = 5, OUT_QUEUE_LEN = 6;
const EventBits_t WIFI_OK = (1 << 0), MQTT_OK = (1 << 1);
const uint32_t NOTIFY_SYS = 1;
const int32_t RUNNING = -1, NOT_TIMED = -2;

volatile uint32_t readEveryMs = 2000;  // the website can change this

// ------------------------------------------------------------------ trace
const int MAX_STEPS = 18;
struct Step { char fn[32]; char task; uint32_t t; int32_t d; };
struct Trace {
  uint8_t n;
  Step s[MAX_STEPS];
  void add(const char* fn, char task, uint32_t t, int32_t d) {
    if (n >= MAX_STEPS) return;
    Step& x = s[n++];
    strlcpy(x.fn, fn, sizeof(x.fn));
    x.task = task; x.t = t; x.d = d;
  }
  void toJson(JsonArray arr) const {
    for (int i = 0; i < n; i++) {
      JsonArray a = arr.add<JsonArray>();
      a.add(s[i].fn); a.add(String(s[i].task)); a.add(s[i].t); a.add(s[i].d);
    }
  }
};

struct Reading { uint32_t seq; bool byButton; float v[5]; Trace tr; };
struct Cmd { char json[256]; uint32_t rxAt; };
struct Out { char topic[12]; char json[700]; };

// ------------------------------------------------------------------ FreeRTOS objects
QueueHandle_t dataQueue, cmdQueue, outQueue;
SemaphoreHandle_t serialMutex, buttonSem;
EventGroupHandle_t netEvents;
TimerHandle_t sysTimer;
TaskHandle_t sensorTaskH = nullptr, mqttTaskH = nullptr, commandTaskH = nullptr, buttonTaskH = nullptr;

// counters shown on the website
volatile uint32_t droppedReadings = 0, droppedCommands = 0;
volatile uint32_t mutexTakes = 0, mutexWaits = 0, mutexMaxWaitUs = 0;
volatile uint32_t isrCount = 0, isrAtUs = 0, semTaken = 0, buttonPresses = 0, lastLatencyUs = 0;
volatile uint32_t notifySensor = 0, notifyMqtt = 0, timerFires = 0;
volatile uint32_t idleTicks[2] = {0, 0};

WiFiClient net;
PubSubClient mqtt(net);
String topic(const char* name) { return String(PREFIX) + "/" + name; }

// ------------------------------------------------------------------ mutex-protected printing
void logLine(const char* text, Trace* tr, char task) {
  uint32_t t0 = micros();
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  uint32_t w = micros() - t0;
  mutexTakes++;
  if (w > 100) mutexWaits++;                  // had to wait for another task
  if (w > mutexMaxWaitUs) mutexMaxWaitUs = w;
  if (tr) tr->add("xSemaphoreTake(serialMutex)", task, t0, w);
  uint32_t t1 = micros();
  Serial.println(text);
  uint32_t t2 = micros();
  if (tr) tr->add("Serial.println", task, t1, t2 - t1);
  xSemaphoreGive(serialMutex);
  if (tr) tr->add("xSemaphoreGive(serialMutex)", task, t2, micros() - t2);
}

// ------------------------------------------------------------------ sensors
// Wiring (see the guide):
//   BMP280  VCC->3V3  GND->GND  SDA->GPIO 8  SCL->GPIO 9      temperature + pressure (I2C)
//   DHT     VCC->3V3  GND->GND  DATA->GPIO 5                    humidity (+ backup temperature)
//   LDR     3V3 -> LDR -> GPIO 6 -> 10k resistor -> GND         light level 0-100 %
// Only SensorTask uses the I2C bus (Wire), so it needs no mutex.
const int I2C_SDA = 8, I2C_SCL = 9, DHT_PIN = 5, LDR_PIN = 6;
#define DHT_TYPE DHT22          // change to DHT11 if your sensor is the blue DHT11
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bmp(&Wire);
bool bmpOk = false;

float readTemperature() { return bmpOk ? bmp.readTemperature() : dht.readTemperature(); }
float readHumidity()    { return dht.readHumidity(); }                       // NAN if the DHT is not answering
float readLight()       { return analogRead(LDR_PIN) * 100.0f / 4095.0f; }  // more light = higher %
float readPressure()    { return bmpOk ? bmp.readPressure() / 100.0f : NAN; } // Pa -> hPa

struct SensorDef { const char* key; const char* fn; float (*read)(); uint8_t decimals; };
SensorDef SENSORS[] = {
  {"t", "readTemperature", readTemperature, 1}, {"h", "readHumidity", readHumidity, 0},
  {"light", "readLight", readLight, 0}, {"p", "readPressure", readPressure, 1},
};
const int NUM_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, r / 4, g / 4, b / 4);
#elif defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, (r | g | b) ? HIGH : LOW);
#endif
}

// ------------------------------------------------------------------ interrupt + ButtonTask (core 1)
void IRAM_ATTR buttonISR() {
  isrAtUs = micros();
  isrCount++;
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(buttonSem, &woken);   // the ISR only gives the semaphore, nothing slow
  if (woken) portYIELD_FROM_ISR();
}

void buttonTask(void*) {
  for (;;) {
    xSemaphoreTake(buttonSem, portMAX_DELAY);               // "Blocked" until the ISR gives it
    uint32_t tWake = micros(), tIsr = isrAtUs;
    semTaken++;
    Trace tr{};
    tr.add("buttonISR", 'I', tIsr, NOT_TIMED);
    tr.add("xSemaphoreGiveFromISR", 'I', tIsr, NOT_TIMED);
    lastLatencyUs = tWake - tIsr;
    tr.add("xSemaphoreTake(buttonSem)", 'K', tWake, lastLatencyUs);   // time from interrupt to task waking
    uint32_t td = micros();
    vTaskDelay(pdMS_TO_TICKS(50));                           // let the button bounce settle
    tr.add("vTaskDelay(debounce)", 'K', td, micros() - td);
    while (xSemaphoreTake(buttonSem, 0) == pdTRUE) {}        // throw away bounce interrupts
    if (digitalRead(BUTTON_PIN) != LOW) continue;            // too short: only noise
    buttonPresses++;
    char line[80];
    snprintf(line, sizeof line, "ButtonTask: BOOT pressed (#%lu), asking SensorTask for a reading", (unsigned long)buttonPresses);
    logLine(line, &tr, 'K');
    uint32_t tn = micros();
    xTaskNotifyGive(sensorTaskH);
    notifySensor++;
    tr.add("xTaskNotifyGive(SensorTask)", 'K', tn, micros() - tn);
    tr.add("xQueueSend(outQueue)", 'K', micros(), NOT_TIMED);
    Out o{};
    strlcpy(o.topic, "button", sizeof o.topic);
    JsonDocument d;
    d["n"] = buttonPresses;
    d["lat"] = lastLatencyUs;
    tr.toJson(d["tr"].to<JsonArray>());
    serializeJson(d, o.json, sizeof o.json);
    xQueueSend(outQueue, &o, pdMS_TO_TICKS(100));
  }
}

// ------------------------------------------------------------------ SensorTask (core 1)
void sensorTask(void*) {
  logLine("SensorTask: waiting for WIFI_OK in the netEvents event group", nullptr, 'S');
  xEventGroupWaitBits(netEvents, WIFI_OK, pdFALSE, pdTRUE, portMAX_DELAY);
  logLine("SensorTask: Wi-Fi is up, starting readings", nullptr, 'S');
  uint32_t seq = 0;
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    TickType_t wait = (int32_t)(next - now) > 0 ? next - now : 0;
    uint32_t ts = micros();
    bool byButton = ulTaskNotifyTake(pdTRUE, wait) > 0;     // sleeps; a notification wakes it early
    uint32_t tw = micros();
    if (!byButton) {
      next += pdMS_TO_TICKS(readEveryMs);
      if ((int32_t)(xTaskGetTickCount() - next) > 0) next = xTaskGetTickCount();
    }
    Reading r{};
    r.seq = ++seq;
    r.byButton = byButton;
    r.tr.add("ulTaskNotifyTake", 'S', ts, tw - ts);
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint32_t t0 = micros();
      r.v[i] = SENSORS[i].read();
      r.tr.add(SENSORS[i].fn, 'S', t0, micros() - t0);
    }
    char line[64];
    snprintf(line, sizeof line, "SensorTask: reading #%lu ready (%s)", (unsigned long)r.seq, byButton ? "button" : "timer");
    logLine(line, &r.tr, 'S');
    r.tr.add("xQueueSend(dataQueue)", 'S', micros(), NOT_TIMED);   // the copy is already made, so not timed
    if (xQueueSend(dataQueue, &r, pdMS_TO_TICKS(100)) != pdTRUE) droppedReadings++;
  }
}

// ------------------------------------------------------------------ CommandTask (core 1)
void commandTask(void*) {
  Cmd c;
  for (;;) {
    if (xQueueReceive(cmdQueue, &c, portMAX_DELAY) != pdTRUE) continue;
    Trace tr{};
    tr.add("mqttCallback", 'M', c.rxAt, NOT_TIMED);
    tr.add("xQueueReceive(cmdQueue)", 'C', micros(), NOT_TIMED);
    JsonDocument in;
    DeserializationError err = deserializeJson(in, c.json);
    JsonDocument out;
    out["id"] = in["id"] | 0;
    const char* cmd = in["cmd"] | "";
    out["cmd"] = cmd;
    bool ok = true, restart = false;
    String reply;
    uint32_t t0 = micros();
    if (err) {
      ok = false; reply = "Could not read the JSON";
      tr.add("deserializeJson", 'C', t0, micros() - t0);
    } else if (!strcmp(cmd, "led")) {
      const char* col = in["color"] | "#000000";
      uint32_t rgb = strtoul(col[0] == '#' ? col + 1 : col, nullptr, 16);
      setLedColor(rgb >> 16, (rgb >> 8) & 0xFF, rgb & 0xFF);
      out["led"] = col;
      reply = String("LED set to ") + col;
      tr.add("setLed", 'C', t0, micros() - t0);
    } else if (!strcmp(cmd, "interval")) {
      uint32_t ms = in["ms"] | 2000;
      if (ms < 500) ms = 500;
      if (ms > 60000) ms = 60000;
      readEveryMs = ms;
      reply = "SensorTask now reads every " + String(ms) + " ms";
      tr.add("setInterval", 'C', t0, micros() - t0);
    } else if (!strcmp(cmd, "msg")) {
      String line = String(">>> Message from the website: ") + (const char*)(in["text"] | "");
      tr.add("showMessage", 'C', t0, NOT_TIMED);
      logLine(line.c_str(), &tr, 'C');                       // uses serialMutex
      reply = "Printed on the Serial Monitor";
    } else if (!strcmp(cmd, "ping")) {
      reply = "pong";
      tr.add("ping", 'C', t0, micros() - t0);
    } else if (!strcmp(cmd, "restart")) {
      reply = "Restarting in 1 second";
      restart = true;
      tr.add("ESP.restart", 'C', t0, NOT_TIMED);
    } else {
      ok = false; reply = String("Unknown command: ") + cmd;
      tr.add("handleCommand", 'C', t0, micros() - t0);
    }
    tr.add("xQueueSend(outQueue)", 'C', micros(), NOT_TIMED);
    out["ok"] = ok;
    out["reply"] = reply;
    tr.toJson(out["tr"].to<JsonArray>());
    Out o{};
    strlcpy(o.topic, "ack", sizeof o.topic);
    serializeJson(out, o.json, sizeof o.json);
    xQueueSend(outQueue, &o, pdMS_TO_TICKS(200));
    if (restart) { vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart(); }
  }
}

// ------------------------------------------------------------------ software timer + CPU load
bool idleHook0() { idleTicks[0]++; return true; }   // true = called at most once per tick
bool idleHook1() { idleTicks[1]++; return true; }

TaskHandle_t* const TASKS[4] = {&sensorTaskH, &mqttTaskH, &commandTaskH, &buttonTaskH};
const uint8_t TASK_CORE[4] = {1, 0, 1, 1};
struct Snapshot { char st[4]; uint32_t stack[4]; uint8_t prio[4]; uint8_t cpu[2]; char holder[16]; } snap;

char stateChar(TaskHandle_t h) {
  switch (eTaskGetState(h)) {
    case eRunning: return 'R'; case eReady: return 'Y'; case eBlocked: return 'B';
    case eSuspended: return 'S'; default: return 'D';
  }
}

// Runs inside the FreeRTOS timer task, so the snapshot is not biased towards MqttTask
void sysTimerCb(TimerHandle_t) {
  static uint32_t lastIdle[2] = {0, 0};
  static TickType_t lastTick = 0;
  timerFires++;
  TickType_t now = xTaskGetTickCount();
  uint32_t dt = now - lastTick;
  for (int c = 0; c < 2; c++) {
    uint32_t di = idleTicks[c] - lastIdle[c];
    lastIdle[c] = idleTicks[c];
    uint32_t idlePct = dt ? di * 100 / dt : 100;
    if (idlePct > 100) idlePct = 100;
    snap.cpu[c] = 100 - idlePct;
  }
  lastTick = now;
  for (int i = 0; i < 4; i++) {
    snap.st[i] = stateChar(*TASKS[i]);
    snap.stack[i] = uxTaskGetStackHighWaterMark(*TASKS[i]);
    snap.prio[i] = uxTaskPriorityGet(*TASKS[i]);
  }
  TaskHandle_t h = xSemaphoreGetMutexHolder(serialMutex);
  strlcpy(snap.holder, h ? pcTaskGetName(h) : "", sizeof snap.holder);
  xTaskNotify(mqttTaskH, NOTIFY_SYS, eSetBits);              // "please send the system report"
  notifyMqtt++;
}

// ------------------------------------------------------------------ MqttTask helpers (MqttTask only)
void publishEvent(const char* e, const char* msg, const char* lvl) {
  JsonDocument d;
  d["e"] = e; d["msg"] = msg; d["lvl"] = lvl;
  char buf[200];
  serializeJson(d, buf);
  mqtt.publish(topic("event").c_str(), buf);
}

void mqttCallback(char* t, byte* payload, unsigned int len) {   // runs inside mqtt.loop()
  Cmd c{};
  memcpy(c.json, payload, min(len, (unsigned int)sizeof(c.json) - 1));
  c.rxAt = micros();
  if (xQueueSend(cmdQueue, &c, 0) != pdTRUE) droppedCommands++;
}

void ensureConnected() {
  static bool everConnected = false;
  bool wifiCameBack = false;
  if (WiFi.status() != WL_CONNECTED) {
    xEventGroupClearBits(netEvents, WIFI_OK | MQTT_OK);
    logLine("MqttTask: waiting for Wi-Fi...", nullptr, 'M');
    while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(250));
    char line[64];
    snprintf(line, sizeof line, "MqttTask: Wi-Fi OK, IP %s", WiFi.localIP().toString().c_str());
    logLine(line, nullptr, 'M');
    xEventGroupSetBits(netEvents, WIFI_OK);                  // wakes SensorTask the first time
    wifiCameBack = everConnected;
  }
  if (!mqtt.connected()) {
    xEventGroupClearBits(netEvents, MQTT_OK);
    String id = "esp32s3-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    while (!mqtt.connect(id.c_str(), nullptr, nullptr, topic("status").c_str(), 1, true, "offline")) {
      logLine("MqttTask: MQTT connect failed, retrying in 2 s", nullptr, 'M');
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
    mqtt.subscribe(topic("cmd").c_str());
    mqtt.publish(topic("status").c_str(), "online", true);
    xEventGroupSetBits(netEvents, MQTT_OK);
    publishEvent("mqtt_connected", "ESP32 connected to the MQTT broker", "");
    logLine("MqttTask: MQTT connected, listening for commands", nullptr, 'M');
    everConnected = true;
  }
  if (wifiCameBack) publishEvent("wifi_reconnect", "Wi-Fi connection came back", "warn");
}

void publishSystem() {
  JsonDocument d;
  d["heap"] = ESP.getFreeHeap();
  d["minheap"] = ESP.getMinFreeHeap();
  d["uptime"] = millis() / 1000;
  d["rssi"] = WiFi.RSSI();
  d["interval"] = readEveryMs;
  d["q"] = uxQueueMessagesWaiting(dataQueue);  d["qmax"] = DATA_QUEUE_LEN;
  d["cq"] = uxQueueMessagesWaiting(cmdQueue);  d["cqmax"] = CMD_QUEUE_LEN;
  d["oq"] = uxQueueMessagesWaiting(outQueue);  d["oqmax"] = OUT_QUEUE_LEN;
  d["dropped"] = droppedReadings;
  d["cdropped"] = droppedCommands;
  JsonArray cpu = d["cpu"].to<JsonArray>();
  cpu.add(snap.cpu[0]); cpu.add(snap.cpu[1]);
  JsonObject mx = d["mutex"].to<JsonObject>();
  mx["holder"] = snap.holder; mx["takes"] = mutexTakes; mx["waits"] = mutexWaits; mx["maxwait"] = mutexMaxWaitUs;
  JsonObject sm = d["sem"].to<JsonObject>();
  sm["isr"] = isrCount; sm["taken"] = semTaken; sm["presses"] = buttonPresses; sm["lat"] = lastLatencyUs;
  EventBits_t bits = xEventGroupGetBits(netEvents);
  JsonObject ev = d["ev"].to<JsonObject>();
  ev["wifi"] = (bits & WIFI_OK) ? 1 : 0; ev["mqtt"] = (bits & MQTT_OK) ? 1 : 0;
  JsonObject tm = d["tmr"].to<JsonObject>();
  tm["period"] = SYSTEM_EVERY_MS; tm["fires"] = timerFires; tm["active"] = xTimerIsTimerActive(sysTimer) ? 1 : 0;
  JsonObject nt = d["ntf"].to<JsonObject>();
  nt["sensor"] = notifySensor; nt["mqtt"] = notifyMqtt;
  JsonArray tasks = d["tasks"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    JsonObject o = tasks.add<JsonObject>();
    o["n"] = pcTaskGetName(*TASKS[i]);
    o["s"] = String(snap.st[i]);
    o["c"] = TASK_CORE[i];
    o["pr"] = snap.prio[i];
    o["st"] = snap.stack[i];
  }
  char buf[1200];
  serializeJson(d, buf, sizeof buf);
  mqtt.publish(topic("system").c_str(), buf);
}

void publishReading(Reading& r) {
  r.tr.add("xQueueReceive(dataQueue)", 'M', micros(), NOT_TIMED);
  char line[64];
  snprintf(line, sizeof line, "MqttTask: publishing reading #%lu", (unsigned long)r.seq);
  logLine(line, &r.tr, 'M');
  uint32_t tb = micros();
  JsonDocument d;
  d["seq"] = r.seq;
  d["why"] = r.byButton ? "button" : "timer";
  for (int i = 0; i < NUM_SENSORS; i++)
    if (!isnan(r.v[i])) d[SENSORS[i].key] = serialized(String(r.v[i], (unsigned int)SENSORS[i].decimals));
  r.tr.add("buildJson", 'M', tb, micros() - tb);
  r.tr.add("mqtt.publish", 'M', micros(), RUNNING);          // this message is being sent right now
  r.tr.toJson(d["tr"].to<JsonArray>());
  char buf[1500];
  serializeJson(d, buf, sizeof buf);
  mqtt.publish(topic("telemetry").c_str(), buf);
}

// ------------------------------------------------------------------ MqttTask (core 0)
void mqttTask(void*) {
  for (;;) {
    ensureConnected();
    mqtt.loop();                                             // receives commands -> mqttCallback
    uint32_t bits = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &bits, 0) == pdTRUE && (bits & NOTIFY_SYS)) publishSystem();
    Out o;
    while (xQueueReceive(outQueue, &o, 0) == pdTRUE) mqtt.publish(topic(o.topic).c_str(), o.json);
    Reading r;
    if (xQueueReceive(dataQueue, &r, pdMS_TO_TICKS(50)) == pdTRUE) publishReading(r);
  }
}

// ------------------------------------------------------------------ setup
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#if !defined(RGB_BUILTIN) && defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  setLedColor(0, 0, 0);

  // sensors (before the tasks start, so no other task touches them yet)
  Wire.begin(I2C_SDA, I2C_SCL);
  bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
  Serial.println(bmpOk ? "BMP280 found" : "BMP280 NOT found: check SDA=8, SCL=9, 3V3, GND (BMP180? tell me)");
  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);   // full 0-3.3 V range

  serialMutex = xSemaphoreCreateMutex();
  buttonSem = xSemaphoreCreateBinary();
  netEvents = xEventGroupCreate();
  dataQueue = xQueueCreate(DATA_QUEUE_LEN, sizeof(Reading));
  cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(Cmd));
  outQueue = xQueueCreate(OUT_QUEUE_LEN, sizeof(Out));
  esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
  esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);   // MqttTask waits for it and sets WIFI_OK
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(2400);

  xTaskCreatePinnedToCore(mqttTask,    "MqttTask",    10240, nullptr, 1, &mqttTaskH,    0);
  xTaskCreatePinnedToCore(sensorTask,  "SensorTask",  5120,  nullptr, 2, &sensorTaskH,  1);
  xTaskCreatePinnedToCore(commandTask, "CommandTask", 8192,  nullptr, 1, &commandTaskH, 1);
  xTaskCreatePinnedToCore(buttonTask,  "ButtonTask",  6144,  nullptr, 3, &buttonTaskH,  1);

  sysTimer = xTimerCreate("sysTimer", pdMS_TO_TICKS(SYSTEM_EVERY_MS), pdTRUE, nullptr, sysTimerCb);
  xTimerStart(sysTimer, 0);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  Serial.println("setup done: 4 tasks, 3 queues, 1 mutex, 1 semaphore, 1 event group, 1 timer");
}

void loop() {
  vTaskDelete(nullptr);
}