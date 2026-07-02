#pragma once

#include "Config.h"

#include "Configuration/Configurable.h"

#include "Channel.h"
#include "Module.h"
#include "SSD1306_I2C.h"
#include "ST7567_SPI.h"
#include "Menu.h"

typedef const uint8_t* font_t;

class OLED : public Channel, public ConfigurableModule {
public:
    OLED(const char* name) : Channel(name), ConfigurableModule(name) {}
    struct Layout {
        uint8_t                    _x;
        uint8_t                    _y;
        uint8_t                    _width_required;
        font_t                     _font;
        OLEDDISPLAY_TEXT_ALIGNMENT _align;
    };
    static Layout bannerLayout128;
    static Layout bannerLayout64;
    static Layout stateLayout;
    static Layout tickerLayout;
    static Layout filenameLayout;
    static Layout percentLayout128;
    static Layout percentLayout64;
    static Layout limitLabelLayout;
    static Layout posLabelLayout;
    static Layout radioAddrLayout;

private:
    std::string _report;

    std::string _radio_info;
    std::string _radio_addr;

    std::string _state;
    std::string _filename;

    float _percent;
    float _feed_rate     = 0;
    float _spindle_speed = 0;
    char  _ip_buf[16]    = "";
    float _axes[3] = { 0, 0, 0 };
    bool  _limits[3] = { false, false, false };
    bool  _probe  = false;
    bool  _isMpos = false;
    std::string _ticker;

    int32_t _radio_delay        = 0;
    int32_t _report_interval_ms = 500;

    uint8_t _i2c_num = 0;

    // SPI mode pins (ST7567 / Mini 12864 V3 on MKS TinyBee V1.0)
    int _spi_cs   = 21;
    int _spi_dc   = 4;
    int _spi_sck  = 18;
    int _spi_mosi = 23;
    int _spi_rst  = 0;
    int _psb_pin  = 15;
    int _en1_pin  = 14;
    int _en2_pin  = 12;
    int _enc_pin  = 13;
    int _neo_pin  = 16;
    std::string _neo_color = "FF500F";  // default rust
    Pin _buz_pin;

    uint8_t _enc_selected_axis = 0;  // 0=X, 1=Y, 2=Z
    uint8_t _contrast = 43;
  int32_t _jog_step_mm     = 1;
  uint32_t _jog_last_ms    = 0;
  uint32_t _menu_last_render = 0;
  volatile bool _needs_render = false;

    void parse_report();
    void parse_status_report();
    void parse_gcode_report();
    void parse_STA();
    void parse_IP();
    void parse_AP();
    void parse_BT();
    void parse_WebUI();

    void parse_axes(std::string s, float* axes);
    void parse_numbers(std::string s, float* nums, uint8_t maxnums);

    void show_limits(bool probe, const bool* limits);
    void show_state();
    void show_file();
    void show_dro(const float* axes, bool isMpos, bool* limits);
    void show_radio_info();
    void draw_checkbox(int16_t x, int16_t y, int16_t width, int16_t height, bool checked);
    void renderDRO();

    void wrapped_draw_string(int16_t y, const std::string& s, font_t font);

    void show(Layout& layout, const std::string& msg) { show(layout, msg.c_str()); }
    void show(Layout& layout, const char* msg);

    uint8_t font_width(font_t font);
    uint8_t font_height(font_t font);
    size_t  char_width(char s, font_t font);

    OLEDDISPLAY_GEOMETRY _geometry = GEOMETRY_64_48;

    bool _error = false;

public:
    OLED(const OLED&)            = delete;
    OLED(OLED&&)                 = delete;
    OLED& operator=(const OLED&) = delete;
    OLED& operator=(OLED&&)      = delete;

    virtual ~OLED() = default;

    void init() override;

    OLEDDisplay* _oled;

    // Configurable

    uint8_t _address = 0x3c;
    int32_t _width   = 64;
    int32_t _height  = 48;
    bool    _flip    = true;
    bool    _mirror  = false;

    // Channel method overrides
    size_t write(uint8_t data) override;

    int read(void) override { return -1; }
    int peek(void) override { return -1; }

    Error pollLine(char* line) override;
    void  flushRx() override {}

    bool   lineComplete(char*, char) override { return false; }
    size_t timedReadBytes(char* buffer, size_t length, TickType_t timeout) override { return 0; }

    // Configuration handlers:
    void validate() override {}

    void afterParse() override;

    void group(Configuration::HandlerBase& handler) override {
        handler.item("report_interval_ms", _report_interval_ms, 100, 5000);
        handler.item("radio_delay_ms", _radio_delay);
        handler.item("jog_step_mm", _jog_step_mm, 1, 100);
        handler.item("contrast", _contrast, 30, 50);
        handler.item("neo_color", _neo_color);
        handler.item("flip", _flip);
        handler.item("mirror", _mirror);
    }
};
