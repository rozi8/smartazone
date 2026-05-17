#include "lora_manager.h"

LoRaHandler::LoRaHandler(int nss, int dio1, int rst, int busy, int sck, int miso, int mosi)
    : _nss(nss), _dio1(dio1), _rst(rst), _busy(busy),
      _sck(sck), _miso(miso), _mosi(mosi),
      spi(FSPI),
      module(_nss, _dio1, _rst, _busy, spi),
      radio(&module) {}

bool LoRaHandler::begin(float frequency)
{
    Serial.println(F("[LoRa] Init SX1262..."));

    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
    delay(10);
    digitalWrite(_rst, HIGH);
    delay(10);

    spi.begin(_sck, _miso, _mosi, _nss);

    int state = radio.begin(frequency, 62.5, 12, 5, 0x34, 22, 8, 0.0f, false);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("[LoRa] init failed: "));
        Serial.println(state);
        return false;
    }

    Serial.print(F("[LoRa] OK freq "));
    Serial.print(frequency);
    Serial.println(F(" MHz"));
    return true;
}

void LoRaHandler::sendMessage(const String &message)
{
    String msg = message; // RadioLib transmit butuh non-const
    int state = radio.transmit(msg);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("[LoRa] Pesan terkirim"));
    }
    else
    {
        Serial.print(F("[LoRa] TX error: "));
        Serial.println(state);
    }
}

bool LoRaHandler::receiveMessage(String &message, int &rssi, float &snr)
{
    String tmp;
    int state = radio.receive(tmp);
    if (state == RADIOLIB_ERR_NONE)
    {
        message = tmp;
        rssi = radio.getRSSI();
        snr = radio.getSNR();
        return true;
    }
    return false;
}

void LoRaHandler::transmit(const uint8_t* data, size_t len)
{
    int state = radio.transmit((uint8_t*)data, len);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("[LoRa] Data transmitted"));
    }
    else
    {
        Serial.print(F("[LoRa] TX error: "));
        Serial.println(state);
    }
}
#ifdef DEVICE_MODE_BASE
bool LoRaHandler::receive()
{
    uint8_t data[RADIOLIB_SX126X_MAX_PACKET_LENGTH];
    size_t len = RADIOLIB_SX126X_MAX_PACKET_LENGTH;
    int state = radio.receive((uint8_t*)data, sizeof(DeviceData));
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("[LoRa] Data received"));
        memcpy(&packet_data.device_data, data, sizeof(DeviceData));
        packet_data.isNew = true;
        packet_data.rssi = radio.getRSSI();
        packet_data.snr = radio.getSNR();
        Serial.printf("[LoRa] RSSI: %d, SNR: %.1f\n", packet_data.rssi, packet_data.snr);
        return true;
    }
    else if (state != RADIOLIB_ERR_RX_TIMEOUT)
    {
        Serial.print(F("[LoRa] RX error: "));
        Serial.println(state);
        return false;
    }
    return false;
}
#endif