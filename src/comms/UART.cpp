
#include "UART.h"
namespace UART
{

static constexpr int UART_NUMBER = 1;
static constexpr int UART_TX_PIN = 4;
static constexpr int UART_RX_PIN = 5;

static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint8_t FRAME_SYNC = 0xAA;

static constexpr uint8_t PAYLOAD_LENGTH = 5;

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

void sendIRData(
    uint16_t mag1,
    uint16_t mag2,
    uint8_t mask
)
{
    uint8_t payload[PAYLOAD_LENGTH];

    payload[0] = mag1 & 0xFF;
    payload[1] = (mag1 >> 8) & 0xFF;

    payload[2] = mag2 & 0xFF;
    payload[3] = (mag2 >> 8) & 0xFF;

    payload[4] = mask;

    uint8_t checksum = 0;

    for (int i = 0; i < PAYLOAD_LENGTH; i++)
    {
        checksum ^= payload[i];
    }

    uart.write(FRAME_SYNC);
    uart.write(PAYLOAD_LENGTH);
    uart.write(payload, PAYLOAD_LENGTH);
    uart.write(checksum);
}

} // namespace UART