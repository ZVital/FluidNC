#pragma once

#include <OLEDDisplay.h>
#include <cstdint>
#include "driver/spi_master.h"

class ST7567_SPI : public OLEDDisplay {
private:
    int                 _cs_pin;
    int                 _dc_pin;
    int                 _rst_pin;
    spi_device_handle_t _spi_handle;

    bool spiWriteByte(uint8_t data);
    bool spiWriteBulk(const uint8_t* data, size_t len);
    void writeCommand(uint8_t cmd);
    void writeData(uint8_t data);
    void writeCmdSequence(const uint8_t* cmds, int count);

public:
    ST7567_SPI(int cs, int dc, int rst, OLEDDISPLAY_GEOMETRY g);
    ~ST7567_SPI();

    // OLEDDisplay interface
    bool connect() override;
    void display() override;
    int  getBufferOffset() override { return 0; }

    // Called externally (since OLEDDisplay::init() is not virtual and sends SSD1306 commands)
    void hardwareReset();
    void uc1701Init();
};
