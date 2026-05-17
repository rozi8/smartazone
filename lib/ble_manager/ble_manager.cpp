#include "ble_manager.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
BLEManager *BLEManager::instance = nullptr;

// UUID service & characteristic Aolon Curve
static const BLEUUID GENERIC_SERVICE("000055ff-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_WRITE("000033f1-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_NOTIFY("000033f2-0000-1000-8000-00805f9b34fb");
static const BLEUUID HR_SERVICE("000056ff-0000-1000-8000-00805f9b34fb");
static const BLEUUID HR_NOTIFY_CHAR("000034f2-0000-1000-8000-00805f9b34fb");

BLEManager::BLEManager(const char *targetAddress, uint32_t scanTime)
    : targetAddress(targetAddress),
      scanTime(scanTime),
      deviceConnected(false),
      lastReconnectAttempt(0),
      pClient(nullptr),
      clientCb(this)
{
    instance = this;
}


// Callback koneksi
void BLEManager::MyClientCallback::onConnect(BLEClient *)
{
    parent_->deviceConnected = true;
    parent_->lastReconnectAttempt = 0;
    Serial.println("[BLE] Connected");
}

void BLEManager::MyClientCallback::onDisconnect(BLEClient *)
{
    // remove service and characteristic
    parent_->deviceConnected = false;
    parent_->pGenericService = nullptr;
    parent_->pHRRemoteService = nullptr;
    parent_->pGenericWriteCharacteristic = nullptr;
    parent_->pGenericNotifyCharacteristic = nullptr;
    parent_->pHRNotifyCharacteristic = nullptr;
    Serial.println("[BLE] Disconnected");
    delay(300); 
}


// Scan dan koneksi
BLEAddress BLEManager::scanTarget()
{
    BLEScan *scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    Serial.println("[BLE] Scanning for target...");
    BLEScanResults results = scan->start(scanTime, false);
    Serial.printf("[BLE] Found %d devices\n", results.getCount());

    strlwr((char *)targetAddress);

    for (int i = 0; i < results.getCount(); ++i)
    {
        BLEAdvertisedDevice dev = results.getDevice(i);
        char *addr = (char *) dev.getAddress().toString().c_str();
        strlwr(addr);
        if (strcmp(targetAddress, addr) == 0)
        {
            Serial.printf("[BLE] Found target %s\n", addr);
            return dev.getAddress();
        }
    }
    Serial.println("[BLE] Device not found");
    return BLEAddress("");
}

void BLEManager::begin(const char *deviceName)
{
    // BLEDevice::deinit(true);
    // delay(200);
    BLEDevice::init(deviceName);
    delay(200);
    connect();
}

bool BLEManager::connect()
{
    BLEAddress addr = scanTarget();
    if (!addr.toString().length())
    {
        Serial.println("[BLE] Device not found during scan");
        return false;
    }

    if (!pClient)
        pClient = BLEDevice::createClient();

    if (pClient->isConnected())
    {
        Serial.println("[BLE] Device already connected");
        return true;
    }

    pClient->setClientCallbacks(&clientCb);

    Serial.printf("[BLE] Connecting to %s...\n", addr.toString().c_str());

    if (!pClient->connect(addr))
    {
        Serial.println("[BLE] Connect failed");
        return false;
    }

    deviceConnected = true;
    Serial.println("[BLE] Connected to server");

    delay(500);

    std::map<std::string, BLERemoteService *> *services = pClient->getServices();

    if (services && !services->empty())
    {
        Serial.println("[BLE] Services discovered:");
        for (auto &s : *services)
        {
            Serial.println(s.first.c_str());
        }
    }


    setupServicesAndCharacteristics();

    if (services)
    {
        Serial.println("[BLE] Safe enabling notify...");

        for (auto &s : *services)
        {
            auto chars = s.second->getCharacteristics();

            for (auto &c : *chars)
            {
                if (c.second->canNotify())
                {
                    std::string uuid = c.first;

                    if (
                        uuid == "000033f2-0000-1000-8000-00805f9b34fb" ||
                        uuid == "000034f2-0000-1000-8000-00805f9b34fb"
                    )
                    { 
                        Serial.printf("[SAFE NOTIFY] %s\n", uuid.c_str());

                        enableNotify(s.second, c.second, notifyThunk);

                        delay(200);
                    }
                }
            }
        }
    }

    return true;
}


// Reconnect handler
bool BLEManager::tryReconnect()
{
    unsigned long now = millis();
    if (!deviceConnected && (now - lastReconnectAttempt >= reconnectInterval))
    {
        lastReconnectAttempt = now;
        return connect();
    }
    return false;
}

bool BLEManager::setupServicesAndCharacteristics()
{
    if (!isConnected())
    {
        Serial.println("[BLE] Not connected");
        return false;
    }

    pGenericService = pClient->getService(GENERIC_SERVICE);
    if (!pGenericService)
    {
        Serial.println("[BLE] Generic service not found");
        return false;
    }

    pGenericWriteCharacteristic = pGenericService->getCharacteristic(CHAR_WRITE);
    pGenericNotifyCharacteristic = pGenericService->getCharacteristic(CHAR_NOTIFY);

    if (!pGenericWriteCharacteristic)
    {
        Serial.println("[BLE] Write char not found");
    }

    if (!pGenericNotifyCharacteristic)
    {
        Serial.println("[BLE] Notify char not found");
    }

    Serial.println("[BLE] Service setup done");
    return true;
}

bool BLEManager::checkServicesAndCharacteristics()
{
    if (!pHRRemoteService || !pGenericService ||
        !pHRNotifyCharacteristic || !pGenericWriteCharacteristic || !pGenericNotifyCharacteristic)
    {
        Serial.println("[BLE] Services or characteristics not properly set up");
        return false;
    }
    Serial.println("[BLE] Services and characteristics are properly set up");
    return true;
}


// Enable notify function
bool BLEManager::enableNotify(
    BLERemoteService *service,
    BLERemoteCharacteristic *characteristic,
    void (*callback)(BLERemoteCharacteristic *, uint8_t *, size_t, bool))
{
    if (!service || !characteristic)
    {
        Serial.println("[BLE] Notify failed: null pointer");
        return false;
    }

    BLERemoteDescriptor *desc = characteristic->getDescriptor(BLEUUID((uint16_t)0x2902));

    if (!desc)
    {
        Serial.println("[BLE] CCCD not found");
        return false;
    }

    uint8_t notifyOn[] = {0x01, 0x00};

    desc->writeValue(notifyOn, sizeof(notifyOn), true);

    characteristic->registerForNotify(callback);

    Serial.println("[BLE] Notify enabled");

    return true;
}

// Perintah trigger sensor
bool BLEManager::triggerSpO2()
{
    static const uint8_t cmd[] = {0xFE, 0xEA, 0x20, 0x06, 0x6B, 0x00};
    delay(150);
    if (pGenericWriteCharacteristic == nullptr)
    {
        Serial.println("[BLE] Generic Write Characterristic Uninitialized");
        return false;
    }
    pGenericWriteCharacteristic->writeValue((uint8_t *)cmd, sizeof(cmd));
    Serial.println("[BLE] Trigger SPO2 sent");
    return true;
}

bool BLEManager::triggerStress()
{
    static const uint8_t cmd[] = {0xFE, 0xEA, 0x20, 0x08, 0xB9, 0x01, 0x00, 0x00};
    delay(150);
    if (pGenericWriteCharacteristic == nullptr)
    {
        Serial.println("[BLE] Generic Write Characterristic Uninitialized");
        return false;
    }
    pGenericWriteCharacteristic->writeValue((uint8_t *)cmd, sizeof(cmd));
    Serial.println("[BLE] Trigger STRESS sent");
    return true;
}

// Callback untuk data BLE masuk
void BLEManager::notifyThunk(
    BLERemoteCharacteristic *ch,
    uint8_t *data,
    size_t len,
    bool)
{
    if (!data || len == 0) return;

    
    // PRINT RAW
    Serial.print("\n[BLE RAW] len=");
    Serial.print(len);
    Serial.print(" data: ");
    for (int i = 0; i < len; i++)
    {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    
    // F7 = STREAM SPO2
    if (data[0] == 0x34)
{
    if (len >= 4)
    {
        uint8_t value = data[3];

        Serial.printf("[DECODE] SPO2 (0x34): %d\n", value);

        instance->SpO2.data = value;
        instance->SpO2.isNew = true;
    }

    return;
}


    // E5 = RESPONSE  HR
    if (data[0] == 0xE5)
    {
        if (len >= 4)
        {
            uint8_t value = data[3];

            Serial.printf("[DECODE] HR RESPONSE: %d\n", value);

            instance->HR.data = value;
            instance->HR.isNew = true;
        }

        return;
    }

    Serial.println("[BLE] Unknown packet");
}

void BLEManager::HRNotifyCallback(BLERemoteCharacteristic *ch, uint8_t *data, size_t len, bool)
{
    // return if not valid
    if (len == 0 || !ch)
        return;
    if (data[1] == 0xFF)
    {
        return;
    }
    uint32_t now = millis();
    instance->HR.data = data[1];
    instance->HR.isNew = true;
}

BLEData BLEManager::getLastSpO2()
{
    BLEData copy = SpO2;
    SpO2.isNew = false;
    return copy;
}
BLEData BLEManager::getLastStress()
{
    BLEData copy = Stress;
    Stress.isNew = false;
    return copy;
}
BLEData BLEManager::getLastHR()
{
    BLEData copy = HR;
    HR.isNew = false;
    return copy;
}

DeviceData BLEManager::BLEDataToSensorData(uint8_t device_id, Topic topic, BLEData data)
{
    DeviceData dev_data={};
    dev_data.device_id = device_id;
    dev_data.topic = topic;
    dev_data.sensor.value = data.data;
    return dev_data;
}