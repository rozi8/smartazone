#include "ble_manager.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
BLEManager *BLEManager::instance = nullptr;

// UUID service & characteristic Aolon Curve
static const BLEUUID GENERIC_SERVICE("0000fee0-0000-1000-8000-00805f9b34fb");
static const BLEUUID CHAR_WRITE("00000013-0000-3512-2118-0009af100700");
static const BLEUUID CHAR_NOTIFY("00000013-0000-3512-2118-0009af100700");
static const BLEUUID HR_SERVICE("0000180d-0000-1000-8000-00805f9b34fb");
static const BLEUUID HR_NOTIFY_CHAR("00002a37-0000-1000-8000-00805f9b34fb");

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

// ===========================================
// Callback koneksi
// ===========================================
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
    delay(300); // beri waktu cleanup stack BLE internal
}

// ===========================================
// Scan dan koneksi
// ===========================================
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
        Serial.println("[BLE] Device has already connected");
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
    Serial.printf("[BLE] Connected to server\n");

// Tampilkan daftar service untuk debug
#ifdef DEBUG
    std::map<std::string, BLERemoteService *> *services = pClient->getServices();
    if (services && !services->empty())
    {
        Serial.println("[BLE] Services discovered: ");
        for (auto &s : *services)
            Serial.println(s.first.c_str());
    }
    else
    {
        Serial.println("[BLE] No services discovered (may still work)");
    }
#endif

    delay(500);
    if (!setupServicesAndCharacteristics())
        return false;
    enableNotify(pGenericService, pGenericNotifyCharacteristic, &BLEManager::notifyThunk);
    enableNotify(pHRRemoteService, pHRNotifyCharacteristic, &BLEManager::HRNotifyCallback);
    return true;
}

// ===========================================
// Reconnect handler
// ===========================================
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
        Serial.println("[BLE] Device not connected, cannot setup services and characteristics");
        return false;
    }

    pHRRemoteService = pClient->getService(HR_SERVICE);
    if (!pHRRemoteService)
    {
        Serial.println("[BLE] Heart Rate service not found");
        return false;
    }
    pGenericService = pClient->getService(GENERIC_SERVICE);
    if (!pGenericService)
    {
        Serial.println("[BLE] Generic service not found");
        return false;
    }

    pHRNotifyCharacteristic = pHRRemoteService->getCharacteristic(HR_NOTIFY_CHAR);
    if (!pHRNotifyCharacteristic)
    {
        Serial.println("[BLE] HR Notify characteristic not found");
        return false;
    }

    pGenericWriteCharacteristic = pGenericService->getCharacteristic(CHAR_WRITE);
    if (!pGenericWriteCharacteristic)
    {
        Serial.println("[BLE] Generic Write characteristic not found");
        return false;
    }

    pGenericNotifyCharacteristic = pGenericService->getCharacteristic(CHAR_NOTIFY);
    if (!pGenericNotifyCharacteristic)
    {
        Serial.println("[BLE] Generic Notify characteristic not found");
        return false;
    }

    Serial.println("[BLE] Services and characteristics setup complete");
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

// ===========================================
// Enable notify function
// ===========================================
bool BLEManager::enableNotify(BLERemoteService *service, BLERemoteCharacteristic *characteristic, void (*callback)(BLERemoteCharacteristic *, uint8_t *, size_t, bool))
{
    if (!service)
    {
        Serial.println("[BLE] Service Uninitialized");
        return false;
    }
    if (!characteristic)
    {
        Serial.println("[BLE] Characteristic Uninitialized");
        return false;
    }

    BLERemoteDescriptor *desc = characteristic->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (!desc)
    {
        Serial.println("[BLE] Notify descriptor not found");
        return false;
    }
    uint8_t notifyOn[] = {0x01, 0x00};
    desc->writeValue(notifyOn, sizeof(notifyOn), true);
    Serial.println("[BLE] Notify enabled");
    characteristic->registerForNotify(callback);
    return true;
}

// ===========================================
// Perintah trigger sensor
// ===========================================
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

// ===========================================
// Callback untuk data BLE masuk
// ===========================================
void BLEManager::notifyThunk(BLERemoteCharacteristic *ch, uint8_t *data, size_t len, bool)
{
    if (len == 0 || !ch)
        return;

    char stress_prefix[] = {0xFE, 0xEA, 0x20, 0x08, 0xB9, 0x11, 0x00};
    char spo2_prefix[] = {0xFE, 0xEA, 0x20, 0x06, 0x6B};

    // check prefix if stress
    if (len == 8 && memcmp(data, stress_prefix, sizeof(stress_prefix)) == 0)
    {
        if (data[7] == 0xFF)
            return;

        instance->Stress.data = data[7];
        instance->Stress.isNew = true;
        return;
    }
    // check prefix if SpO2
    if (len == 6 && memcmp(data, spo2_prefix, sizeof(spo2_prefix)) == 0)
    {
        if (data[5] == 0xFF)
            return;
        instance->SpO2.data = data[5];
        instance->SpO2.isNew = true;
        return;
    }

    Serial.printf("[BLE] Unknown Data,len %d, data :%X \n", len, data);
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