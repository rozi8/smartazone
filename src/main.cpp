#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <TinyGPSPlus.h>
#include "lora_manager.h"
#include "mqtt_manager.h"
#include "data.h"

#ifdef DEVICE_MODE_CLIENT
#elif defined(DEVICE_MODE_BASE)
#include <AsyncHTTPRequest_Generic.h>   
#include "ArduinoJson.h"
#endif

#define LED_PIN GPIO_NUM_37

#ifdef DEVICE_MODE_CLIENT
#include "ble_manager.h"
// BLE target Aolon
const int DEVICE_ID = 2;
const char targetAddress[] PROGMEM = "78:02:B7:35:54:B4";
#define SOS_PIN GPIO_NUM_42 
#define AOLON_SERVICE_UUID "000055ff-0000-1000-8000-00805f9b34fb"
#define AOLON_WRITE_UUID   "000033f1-0000-1000-8000-00805f9b34fb"
#define AOLON_NOTIFY_UUID  "000033f2-0000-1000-8000-00805f9b34fb"

#define GPS_BAUD 9600
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;


static const uint32_t BLE_RECONNECT_MS = 5000;
static const uint32_t TRIGGER_INTERVAL_MS = 300000;          // 5 menit
static const uint32_t INTERVAL_BETWEEN_SPO2_STRESS = 120000; // 2 menit
static const uint32_t GPS_INTERVAL_MS = 60000;               // 1 menit
#define BUTTON_LONG_TIME 2000
#define BUTTON_DEBUNCE_TIME 50
bool is_pressed = false;
bool streesTriggerPending = true;
GPSData gpsData;

struct Timers
{
    uint32_t bleReconnect{0};
    uint32_t sendTick{0};
    uint32_t status{0};
    uint32_t triggerTick{0};
    uint32_t interval_spo_stress{0};
    uint32_t gps_tick{0};
    uint32_t debounce_tick{0};
    uint32_t hold_tick{UINT32_MAX};
} timers;

// BLE & Sensor instance
BLEManager ble(targetAddress, 6);

void IRAM_ATTR handle_button_callback(){
    uint32_t now = millis();
    if ((now - timers.debounce_tick) < BUTTON_DEBUNCE_TIME){
        // is_pressed = false;
        return;
    }
    timers.debounce_tick = now;
    is_pressed = !digitalRead(SOS_PIN);
}

// GPS Task
void gps_task(void *pvParameters)
{
    GPSData *data = (GPSData *)pvParameters;
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPIO_NUM_43, GPIO_NUM_44); // RX, TX
    while (true)
    {
        uint32_t now = millis();
        // while (GPSSerial.available())
        // {
        //     char c = GPSSerial.read();
        //     gps.encode(c);
        // }
        // if (gps.location.isUpdated())
        // {
        //     if (!gps.location.isValid())
        //     {
        //         Serial.println("[GPS] Location invalid");
        //         continue;
        //     }
        //     if (now - timers.gps_tick < GPS_INTERVAL_MS)
        //         continue;
        //     timers.gps_tick = now;
        //     data->lattitude = gps.location.lat();
        //     data->longitude = gps.location.lng();
        //     data->isNew = true;
        //     Serial.printf("[GPS] New location: %.6f, %.6f\n", data->lattitude, data->longitude);
        // }
        if (now - timers.gps_tick < GPS_INTERVAL_MS)
            continue;
        timers.gps_tick = now;
        data->lattitude = -7.334967968864027;
        data->longitude = 112.78784320020455;
        data->isNew = true;
        Serial.printf("[GPS] New location: %.6f, %.6f\n", data->lattitude, data->longitude);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
#endif
std::string TopictoString(Topic topic)
{
    switch (topic)
    {
    case Topic::HEART_RATE:
        return "heart_rate";
    case Topic::SPO2:
        return "spo2";
    case Topic::STRESS:
        return "stress";
    case Topic::GPS:
        return "GPS";
    case Topic::SOS:
        return "SOS";
    default:
        return "unknown";
    }
}

#ifdef DEVICE_MODE_BASE
// MQTT setup
const char *WIFI_SSID = "MAMINO_XL_4G";
const char *WIFI_PASS = "kopihitam";
const char *MQTT_SERVER = "43.156.68.148";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_USER = "mqtt";
const char *MQTT_PASS = "mqttpass";
const char *MQTT_TOPIC = "device/health";
const char *API_URL = "http://smartazone.com/api/update-log";
const char *SOS_API_URL = "http://10.29.46.255:8000/api/sos-trigger";
MqttManager mqtt(WIFI_SSID, WIFI_PASS, MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS);
AsyncHTTPRequest request;

// void testInternet() {
//     WiFiClient client;
//     if (client.connect("8.8.8.8", 53)) {
//         Serial.println("[TEST] Internet OK");
//     } else {
//         Serial.println("[TEST] Internet FAILED");
//     }
// }


// Sinkronisasi waktu (NTP)
time_t bootEpoch = 0;
unsigned long bootMillis = 0;
bool ntpSynced = false;

void setupTime()
{
    Serial.print("[Time] Connecting WiFi for NTP...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long start = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\n[Time] WiFi connected");
        configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
        delay(2000);

        time_t now;
        if (time(&now))
        {
            bootEpoch = now;
            bootMillis = millis();
            ntpSynced = true;
            Serial.printf("[Time] NTP sync success: %lu\n", (unsigned long)now);
        }
        else
        {
            Serial.println("[Time] NTP sync failed");
        }
    }
    else
    {
        Serial.println("\n[Time] WiFi not connected, fallback to millis()");
    }
}

void PostDeviceData(const DeviceData &data){
    static bool requestOpenResult = false;
    StaticJsonDocument<256> doc;
    doc["device_id"] = data.device_id; // data.device_id
    if (data.topic == Topic::GPS || data.topic == Topic::SOS)
    {
        doc["lattitude"] = data.sensor.location.lattitude;
        doc["longitude"] = data.sensor.location.longitude;
    }
    else if (data.topic == Topic::HEART_RATE )
    {
        doc["heart_rate"] = data.sensor.value;
    }
    else if (data.topic == Topic::SPO2 )
    {
        doc["spo2"] = data.sensor.value;
    }
    else if (data.topic == Topic::STRESS )
    {
        doc["stress_level"] = data.sensor.value;
    }
    String json;
    const char *URL; 
    if (data.topic == Topic::SOS )
       URL =  SOS_API_URL;
    else
        URL =  API_URL;
    Serial.println("[HTTP] WiFi status: " + String(WiFi.status()));
    Serial.println("[HTTP] Target URL: " + String(URL));
    serializeJson(doc, json);
    Serial.println("[HTTP] Preparing to post to " + String(URL));
    if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone){
        requestOpenResult =  request.open("POST", URL);
        request.setReqHeader("Content-Type", "application/json");
        if (!requestOpenResult){
            Serial.println("[HTTP] Failed to open request");
            return;
        }else{
            request.send(json);
            Serial.println("[HTTP] Posting data: " + json);
        }
    }else{
        Serial.println("[HTTP] Request busy, skipping...");
    }
}

void requestCallback(void *optParm, AsyncHTTPRequest* request, int readyState)
{
    if (readyState == readyStateDone)
    {
        int status = request->responseHTTPcode();

        Serial.println("========== HTTP DEBUG ==========");
        Serial.printf("Status Code: %d\n", status);
        Serial.printf("ReadyState: %d\n", readyState);
        Serial.printf("Content-Length: %d\n", request->responseLength());

        String response = request->responseText();

        Serial.println("Body:");
        Serial.println(response);
        Serial.println("================================");
    }
}

struct Timers
{
    uint32_t status{0};
} timers;

time_t getCurrentTime()
{
    if (ntpSynced)
        return bootEpoch + ((millis() - bootMillis) / 1000);
    return millis() / 1000;
}
#endif

// LoRa pin mapping
static const uint8_t LORA_NSS = 7;
static const uint8_t LORA_SCK = 5;
static const uint8_t LORA_MOSI = 6;
static const uint8_t LORA_MISO = 3;
static const uint8_t LORA_DIO1 = 33;
static const uint8_t LORA_BUSY = 34;
static const uint8_t LORA_RST = 8;

LoRaHandler lora(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LORA_SCK, LORA_MISO, LORA_MOSI);
static const uint32_t STATUS_INTERVAL_MS = 60000;
#ifdef DEVICE_MODE_BASE

#endif


// Setup
void setup()
{
    Serial.begin(115200);
    delay(300);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

#ifdef DEVICE_MODE_CLIENT
    Serial.println(F("[Main] Mode: CLIENT"));
    ble.begin("EoRa-S3");
    if (!lora.begin(923.0))
    {
        Serial.println(F("[Main] LoRa init failed"));
        while (true)
            delay(1000);
    }
    Serial.println(F("[Main] LoRa ready"));
    // for auto start trigger
    timers.triggerTick = -300000;
    // Start GPS task
    xTaskCreatePinnedToCore(
        gps_task,         /* Task function. */
        "GPS Task",       /* name of task. */
        4096,             /* Stack size of task */
        (void *)&gpsData, /* parameter of the task */
        1,                /* priority of the task */
        NULL,             /* Task handle to keep track of created task */
        1);               /* pin task to core 1 */
    pinMode(GPIO_NUM_47,OUTPUT);
    digitalWrite(GPIO_NUM_47,LOW);
    pinMode(SOS_PIN,INPUT_PULLUP);
    attachInterrupt(SOS_PIN,handle_button_callback,CHANGE);

#elif defined(DEVICE_MODE_BASE)
    setupTime();
    Serial.println(F("[Main] Mode: BASE"));
    //lora.begin(923.0);
   
    if (!lora.begin(923.0)) {
    Serial.println(F("[Main] LoRa init failed"));
    while (true) delay(1000);
}
    request.setDebug(false);
    request.onReadyStateChange(requestCallback);
    mqtt.begin();
#endif
}


// Loop utama
void loop()
{
    uint32_t now = millis();

    // Status periodik
    if (now - timers.status >= STATUS_INTERVAL_MS)
    {
        timers.status = now;
        Serial.printf("[Status] Uptime:%lus | Heap:%u bytes\n", now / 1000, ESP.getFreeHeap());
    }

#ifdef DEVICE_MODE_CLIENT
    BLEData HR, SpO2, Stress;
    DeviceData new_data;
    if (is_pressed &&( timers.hold_tick == UINT32_MAX))
        timers.hold_tick = now;
    else if(!is_pressed)
        timers.hold_tick = UINT32_MAX;
    if (is_pressed && now - timers.hold_tick >= BUTTON_LONG_TIME){
        timers.hold_tick = UINT32_MAX;
        is_pressed = false;
        DeviceData data ={0};
        data.device_id = DEVICE_ID;
        data.sensor.location.lattitude = gpsData.lattitude;
        data.sensor.location.longitude = gpsData.longitude;
        data.topic= Topic::SOS;
        new_data = data;
        lora.transmit((uint8_t*)&new_data, sizeof(DeviceData));
        Serial.printf("send sos trigger data\n");
    }
    // Reconnect BLE jika terputus
    if (now - timers.bleReconnect >= BLE_RECONNECT_MS)
    {
        timers.bleReconnect = now;
        if (!ble.isConnected())
        {
            Serial.println("[BLE] Reconnecting...");
            if (ble.tryReconnect())
            {
                Serial.println("[BLE] Notify re-enabled after reconnect");
            }
        }
    }

    // Trigger SPO2 dan STRESS setiap 10 detik
    if (ble.isConnected() && (now - timers.triggerTick >= TRIGGER_INTERVAL_MS))
    {
        timers.triggerTick = now;
        Serial.println("[BLE] Triggering SPO2 sensors...");
        ble.triggerSpO2();
        delay(500);
        ble.triggerSpO2();
        timers.interval_spo_stress = now + INTERVAL_BETWEEN_SPO2_STRESS;
        streesTriggerPending = false;
    }
    if (ble.isConnected() && !streesTriggerPending && (now >= timers.interval_spo_stress))
    {
        Serial.println("[BLE] Triggering Stress sensor");
        ble.triggerStress();
        delay(500);
        ble.triggerStress();
        streesTriggerPending = true;
    }

    HR = ble.getLastHR();
    SpO2 = ble.getLastSpO2();
    Stress = ble.getLastStress();
    if (HR.isNew)
    {
        Serial.printf("send hr data: %d \n", HR.data);
        new_data = ble.BLEDataToSensorData(DEVICE_ID, Topic::HEART_RATE, HR);
        lora.transmit((uint8_t*)&new_data, sizeof(DeviceData));
    }
    if (SpO2.isNew)
    {
        Serial.printf("send spo2 data: %d \n", SpO2.data);
        new_data = ble.BLEDataToSensorData(DEVICE_ID, Topic::SPO2, SpO2);
        lora.transmit((uint8_t*)&new_data, sizeof(DeviceData));
    }
    if (Stress.isNew)
    {
        Serial.printf("send Stress data: %d \n", Stress.data);
        new_data = ble.BLEDataToSensorData(DEVICE_ID, Topic::STRESS, Stress);
        lora.transmit((uint8_t*)&new_data, sizeof(DeviceData));
    }
    if (gpsData.isNew)
    {
        gpsData.isNew = false;
        Serial.printf("send GPS data: (%.6f, %.6f) \n", gpsData.lattitude, gpsData.longitude);
        new_data = DeviceData();
        new_data.device_id = DEVICE_ID;
        new_data.topic = Topic::GPS;
        new_data.sensor.location.lattitude = gpsData.lattitude;
        new_data.sensor.location.longitude = gpsData.longitude;
        lora.transmit((uint8_t*)&new_data, sizeof(DeviceData));
    }
#elif defined(DEVICE_MODE_BASE)
    // Mode RX → terima dan forward ke MQTT
    mqtt.loop();
    String msg;
    DeviceData device_data;
    struct tm timeinfo;
    char timeStringBuff[64];
    String mqtt_payload;
    std::string full_topic;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    if (lora.receive())
    {
        receivedPacket packet = lora.getNewPacket();
        if (!packet.isNew)
            return;
        device_data = packet.device_data;
        if (device_data.topic != Topic::GPS && device_data.topic != Topic::SOS)
        {
            std::string topic_str = TopictoString(device_data.topic);
            Serial.printf("[LORA] get data from device: %d on Topic : %s and value: %d\n at %s\n", device_data.device_id, topic_str.c_str(), device_data.sensor.value, timeStringBuff);
            mqtt_payload = String(device_data.sensor.value);
            full_topic = std::to_string(device_data.device_id) + "/" + TopictoString(device_data.topic);
        }
        else
        {
            Serial.printf("[LORA] get data from device: %d on Topic : %s and location: (%.6f, %.6f) at %s\n", device_data.device_id, TopictoString(device_data.topic).c_str(), device_data.sensor.location.lattitude, device_data.sensor.location.longitude, timeStringBuff);
            mqtt_payload = String("{\"lattitude\":") + String(device_data.sensor.location.lattitude, 6) + String(", \"longitude\":") + String(device_data.sensor.location.longitude, 6) + String("}");
            full_topic = std::to_string(device_data.device_id) + "/" + TopictoString(device_data.topic);
        }
        //PostDeviceData(device_data);
        if (mqtt.isConnected())
{
    mqtt.publish((char *)full_topic.c_str(), mqtt_payload);
}
    }
#endif
    delay(50);
}
