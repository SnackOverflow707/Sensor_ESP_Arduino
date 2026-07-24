#include "UART.h"
#include <string.h>
namespace UART
{

static constexpr int UART_NUMBER = 1;
static constexpr int UART_TX_PIN = 4;
static constexpr int UART_RX_PIN = 5;

static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint8_t FRAME_SYNC = 0xAA;

static HardwareSerial uart(UART_NUMBER);

void begin()
{
    uart.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );
}


static void sendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        checksum ^= payload[i];
    }

    uart.write(FRAME_SYNC);
    uart.write(type);
    uart.write(len);
    uart.write(payload, len);
    uart.write(checksum);
}

void sendIRData(
    uint16_t mag1,
    uint16_t mag2,
    uint8_t mask
)
{
    uint8_t payload[5];

    payload[0] = mag1 & 0xFF;
    payload[1] = (mag1 >> 8) & 0xFF;

    payload[2] = mag2 & 0xFF;
    payload[3] = (mag2 >> 8) & 0xFF;

    payload[4] = mask;

    sendFrame(FRAME_TYPE_IR, payload, sizeof(payload));
}

void sendMetalData(
    uint8_t sensorId,
    float freqHz
)
{
    uint8_t payload[1 + sizeof(float)];

    payload[0] = sensorId;
    memcpy(payload + 1, &freqHz, sizeof(float));   // native little-endian layout, same convention SENSOR_ESP uses

    sendFrame(FRAME_TYPE_METAL, payload, sizeof(payload));
}

} // namespace UART