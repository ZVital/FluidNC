#include "ST7567_SPI.h"

#include <Arduino.h>
#include "driver/spi_master.h"

#include "Config.h"
#include "Machine/MachineConfig.h"

ST7567_SPI::ST7567_SPI(int cs, int dc, int rst, OLEDDISPLAY_GEOMETRY g) :
    _cs_pin(cs), _dc_pin(dc), _rst_pin(rst), _spi_handle(nullptr) {
    setGeometry(g);
    BufferOffset = getBufferOffset();  // Must init BEFORE allocateBuffer() reads it
}

ST7567_SPI::~ST7567_SPI() {
    if (_spi_handle) {
        spi_bus_remove_device(_spi_handle);
        _spi_handle = nullptr;
    }
}

bool ST7567_SPI::connect() {
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    pinMode(_dc_pin, OUTPUT);
    digitalWrite(_dc_pin, HIGH);
    if (_rst_pin >= 0) {
        pinMode(_rst_pin, OUTPUT);
        digitalWrite(_rst_pin, HIGH);
    }

    // Add display as a device on the shared HW SPI bus
    // The bus must already be initialized by SPIBus::init() (from spi: section in config)
    if (!config->_spi || !config->_spi->defined()) {
        log_error("ST7567: SPI bus not configured (need spi: section in config)");
        return false;
    }

    spi_device_interface_config_t devcfg = {
        .mode       = 0,          // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz
        .spics_io_num = -1,       // Manual CS control
        .queue_size = 1,
    };
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi_handle);
    if (ret != ESP_OK) {
        log_error("ST7567: spi_bus_add_device failed: " << int(ret));
        _spi_handle = nullptr;
        return false;
    }
    log_info("ST7567: added SPI device on shared bus (4 MHz)");  // Mini 12864
    return true;
}

void ST7567_SPI::hardwareReset() {
    if (_rst_pin < 0) return;
    digitalWrite(_rst_pin, LOW);
    delay(10);
    digitalWrite(_rst_pin, HIGH);
    delay(10);
}

bool ST7567_SPI::spiWriteByte(uint8_t data) {
    if (!_spi_handle) return false;
    spi_transaction_t trans = {};
    trans.length    = 8;
    trans.tx_buffer = &data;
    return spi_device_transmit(_spi_handle, &trans) == ESP_OK;
}

bool ST7567_SPI::spiWriteBulk(const uint8_t* data, size_t len) {
    if (!_spi_handle || len == 0) return false;
    spi_transaction_t trans = {};
    trans.length    = len * 8;
    trans.tx_buffer = data;
    return spi_device_transmit(_spi_handle, &trans) == ESP_OK;
}

void ST7567_SPI::writeCommand(uint8_t cmd) {
    digitalWrite(_dc_pin, LOW);
    digitalWrite(_cs_pin, LOW);
    spiWriteByte(cmd);
    digitalWrite(_cs_pin, HIGH);
}

void ST7567_SPI::writeData(uint8_t data) {
    digitalWrite(_dc_pin, HIGH);
    digitalWrite(_cs_pin, LOW);
    spiWriteByte(data);
    digitalWrite(_cs_pin, HIGH);
}

void ST7567_SPI::writeCmdSequence(const uint8_t* cmds, int count) {
    digitalWrite(_dc_pin, LOW);
    digitalWrite(_cs_pin, LOW);
    for (int i = 0; i < count; i++) {
        spiWriteByte(cmds[i]);
    }
    digitalWrite(_cs_pin, HIGH);
}

void ST7567_SPI::uc1701Init() {
    delay(200);

    writeCommand(0xE2);
    delay(10);

    // UC1701 init sequence from Marlin (u8g_dev_uc1701_mini12864_HAL.cpp)
    // The controller is UC1701, not ST7567 — the 0xF8 booster command is UC1701-specific
    static const uint8_t init_seq[] = {
        0x40,
        0xA0,              // SEG normal (column 0 = SEG0)
        0xC8,              // COM reverse (COM63=top)
        0xA6,
        0xA2,
        0x2F,
        0xF8, 0x00,        // Booster ratio ×4 (UC1701-specific)
        0x24,              // V5 resistor ratio (4)
        0x81, 0x2F,        // Contrast 0x2F (47)
        0xAC, 0x00,        // Indicator disable (UC1701-specific)
        0xAF,
    };
    writeCmdSequence(init_seq, sizeof(init_seq));
    delay(50);
}

void ST7567_SPI::setContrast(uint8_t value) {
    writeCommand(0x81);
    writeCommand(value);
}

void ST7567_SPI::display() {
    // 8 pages × 128 columns, each byte = 8 vertical pixels (bit0=top in buffer).
    // With 0xC8 (COM63→COM0 scan) and MSB-first SPI, no bit reversal needed.
    // Combine the 3 address-set commands into one bulk write per page to halve
    // SPI transaction count (less bus-lock time, better SD-card sharing).
    uint8_t cmd[3];
    for (int page = 0; page < 8; page++) {
        cmd[0] = 0xB0 | page;
        cmd[1] = 0x10;
        cmd[2] = 0x00;

        digitalWrite(_dc_pin, LOW);
        digitalWrite(_cs_pin, LOW);
        spiWriteBulk(cmd, 3);
        digitalWrite(_dc_pin, HIGH);
        spiWriteBulk(&buffer[page * 128], 128);
        digitalWrite(_cs_pin, HIGH);
    }
}
