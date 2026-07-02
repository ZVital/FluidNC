#include "OLED.h"
#include "string_util.h"

#include "Machine/MachineConfig.h"
#include "Job.h"
#include "InputFile.h"
#include "Report.h"

#include <WiFi.h>
#include <driver/rmt.h>

void OLED::show(Layout& layout, const char* msg) {
    if (_width < layout._width_required) {
        return;
    }
    _oled->setTextAlignment(layout._align);
    _oled->setFont(layout._font);
    _oled->drawString(layout._x, layout._y, msg);
}

OLED::Layout OLED::bannerLayout128  = { 0, 0, 0, ArialMT_Plain_24, TEXT_ALIGN_CENTER };
OLED::Layout OLED::bannerLayout64   = { 0, 0, 0, ArialMT_Plain_16, TEXT_ALIGN_CENTER };
OLED::Layout OLED::stateLayout      = { 0, 0, 0, ArialMT_Plain_10, TEXT_ALIGN_LEFT };
OLED::Layout OLED::tickerLayout     = { 63, 0, 128, ArialMT_Plain_10, TEXT_ALIGN_CENTER };
OLED::Layout OLED::filenameLayout   = { 63, 13, 128, ArialMT_Plain_10, TEXT_ALIGN_CENTER };
OLED::Layout OLED::percentLayout128 = { 128, 0, 128, ArialMT_Plain_16, TEXT_ALIGN_RIGHT };
OLED::Layout OLED::percentLayout64  = { 64, 0, 64, ArialMT_Plain_16, TEXT_ALIGN_RIGHT };
OLED::Layout OLED::limitLabelLayout = { 80, 14, 128, ArialMT_Plain_10, TEXT_ALIGN_LEFT };
OLED::Layout OLED::posLabelLayout   = { 60, 14, 128, ArialMT_Plain_10, TEXT_ALIGN_RIGHT };
OLED::Layout OLED::radioAddrLayout  = { 50, 0, 128, ArialMT_Plain_10, TEXT_ALIGN_LEFT };

void OLED::afterParse() {
    bool use_spi = _spi_cs >= 0;
    if (use_spi) {
        // SPI mode (ST7567 / Mini 12864) — no I2C config needed
        // Mini 12864 V3 is always 128x64
        _width  = 128;
        _height = 64;
        _geometry = GEOMETRY_128_64;
        return;
    }
    if (!config->_i2c[_i2c_num]) {
        log_error("i2c" << _i2c_num << " section must be defined for OLED");
        _error = true;
        return;
    }
    switch (_width) {
        case 128:
            switch (_height) {
                case 64:
                    _geometry = GEOMETRY_128_64;
                    break;
                case 32:
                    _geometry = GEOMETRY_128_32;
                    break;
                default:
                    log_error("For OLED width 128, height must be 32 or 64");
                    _error = true;
                    break;
            }
            break;
        case 64:
            switch (_height) {
                case 48:
                    _geometry = GEOMETRY_64_48;
                    break;
                case 32:
                    _geometry = GEOMETRY_64_32;
                    break;
                default:
                    log_error("For OLED width 64, height must be 32 or 48");
                    _error = true;
                    break;
            }
            break;
        default:
            log_error("OLED width must be 64 or 128");
            _error = true;
    }
}

void OLED::init() {
    if (_error) {
        return;
    }
    if (_spi_cs >= 0) {
        // SPI mode — ST7567 (Mini 12864)
        // OLEDDisplay::init() is NOT virtual — calling it through OLEDDisplay* would invoke
        // the base class init which sends SSD1306 commands. Instead we allocate the buffer
        // (public method) and initialize the display directly.

        // Force PSB (mode select) LOW if a pin is configured
        // On Mini 12864 V3, EXP1-7 = PSB. If PSB is HIGH, display is in 6800 parallel mode.
        // We drive PSB LOW for SPI mode, then user must press RESET on the display
        // while PSB is LOW so the mode is latched at hardware reset.
        if (_psb_pin >= 0) {
            pinMode(_psb_pin, OUTPUT);
            digitalWrite(_psb_pin, LOW);
            log_info("OLED PSB pin " << _psb_pin << " set LOW for SPI mode");
        }

        log_info("OLED SPI mode UC1701 cs:" << _spi_cs << " dc:" << _spi_dc << " rst:" << _spi_rst
                 << " (HW SPI on shared bus)");
        auto* st7567 = new ST7567_SPI(_spi_cs, _spi_dc, _spi_rst, _geometry);
        _oled = st7567;
        if (!_oled->allocateBuffer()) {
            log_error("ST7567 buffer allocation failed");
            _error = true;
            return;
        }

        // First init attempt
        st7567->hardwareReset();
        st7567->uc1701Init();
        _oled->clear();
        _oled->display();

        // Retry a couple times in case display needed more settling time
        for (int retry = 0; retry < 3; retry++) {
            delay(100);
            st7567->uc1701Init();
            _oled->clear();
            _oled->display();
        }
    } else {
        // I2C mode — SSD1306
        log_info("OLED I2C address: " << to_hex(_address) << " width: " << _width << " height: " << _height);
        _oled = new SSD1306_I2C(_address, _geometry, config->_i2c[_i2c_num], 400000);
        _oled->init();
    }

    if (_flip) {
        _oled->flipScreenVertically();
    }
    if (_mirror) {
        _oled->mirrorScreen();
    }
    _oled->setTextAlignment(TEXT_ALIGN_LEFT);

    _oled->clear();

    show((_width == 128) ? bannerLayout128 : bannerLayout64, "FluidNC");

    _oled->display();

    allChannels.registration(this);
    setReportInterval(_report_interval_ms);

    // Encoder pins (Mini 12864) — delegate to Menu debounced reader
    if (_en1_pin >= 0) {
        initEncoder(_en1_pin, _en2_pin, _enc_pin);
        log_info("Encoder enabled on pins en1:" << _en1_pin << " en2:" << _en2_pin << " btn:" << _enc_pin);
    }
    if (_buz_pin.defined()) {
        _buz_pin.setAttr(Pin::Attr::Output);
        _buz_pin.synchronousWrite(false);
        log_info("Buzzer enabled on " << _buz_pin.name());
    }
    if (_neo_pin >= 0) {
        // WS2812 via RMT — rust color for 3 LEDs
        // 80MHz / 8 = 10MHz → 100ns per tick
        rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)_neo_pin, RMT_CHANNEL_0);
        cfg.clk_div = 8;
        rmt_config(&cfg);
        rmt_driver_install(RMT_CHANNEL_0, 0, 0);

        // WS2812: 0=4H+9L, 1=8H+5L (100ns ticks)
        // RGB: r=255=0xFF, g=80=0x50, b=15=0x0F
        uint8_t data[9] = { 0xFF, 0x50, 0x0F,  0xFF, 0x50, 0x0F,  0xFF, 0x50, 0x0F };
        rmt_item32_t bits[72];
        int idx = 0;
        for (int i = 0; i < 9; i++) {
            for (int b = 7; b >= 0; b--) {
                bits[idx++] = (data[i] & (1 << b))
                    ? rmt_item32_t{ { 8, 1, 5, 0 } }
                    : rmt_item32_t{ { 4, 1, 9, 0 } };
            }
        }
        rmt_write_items(RMT_CHANNEL_0, bits, 72, true);
        delayMicroseconds(300);
        rmt_driver_uninstall(RMT_CHANNEL_0);
        pinMode(_neo_pin, OUTPUT);
        digitalWrite(_neo_pin, LOW);
        log_info("RGB backlight set on pin " << _neo_pin);
    }

    // Set up info screen callback (field 0=IP, 1=WiFi, 2=Version)
    static OLED* self = this;
    infoText = [](int field) -> const char* {
        switch (field) {
            case 0:
                if (self->_radio_addr.empty()) {
                    IPAddress ip = WiFi.localIP();
                    if (ip) {
                        snprintf(self->_ip_buf, sizeof(self->_ip_buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                        return self->_ip_buf;
                    }
                }
                return self->_radio_addr.c_str();
            case 1: return self->_radio_info.c_str();
            case 2: return grbl_version;
            default: return nullptr;
        }
    };

    // Set up edit apply callback
    editApplyCB = [](int value) {
        if (editingItem == 0 && self->_spi_cs >= 0) {
            self->_contrast = (uint8_t)value;
            contrastValue = value;
            auto* st7567 = static_cast<ST7567_SPI*>(self->_oled);
            st7567->setContrast((uint8_t)value);
        }
    };

    // Apply configured contrast to display and sync with menu
    contrastValue = _contrast;
    if (_spi_cs >= 0) {
        auto* st7567 = static_cast<ST7567_SPI*>(_oled);
        st7567->setContrast(_contrast);
    }
}

Error OLED::pollLine(char* line) {
    autoReport();

    // If an SD file was selected from the menu, start it
    if (sdFilePending) {
        sdFilePending = false;
        Job::save();
        try {
            InputFile* theFile = new InputFile(SD, sdSelectedFile);
            Job::nest(theFile, this);
        } catch (...) {
            Job::restore();
        }
    }

    // If probe gcode is pending (from Probe Z wizard), inject it
    if (probeGcodePending) {
        probeGcodePending = false;
        strncpy(line, probeGcode, Channel::maxLine - 1);
        line[Channel::maxLine - 1] = '\0';
        return Error::Ok;
    }

    if (_en1_pin >= 0) {
        pollEncoder();
        BtnState btn = readButtonState();

        if (btn == BtnState::LONG_PRESS) {
            if (_state == "Alarm") {
                // Reset alarm on long-press
                strncpy(line, "$X\n", Channel::maxLine - 1);
                line[Channel::maxLine - 1] = '\0';
                return Error::Ok;
            }
            menuActive = !menuActive;
            if (menuActive) {
                goScreen(menu_main);
                encoderLine    = 0;
                encoderTopLine = 0;
                _oled->clear();
                drawMenu(_oled);
                _oled->display();
            } else {
                goScreen(nullptr);
                resetButtonState();
                _needs_render = true;
            }
            return Error::NoData;
        }

        if (editActive && menuActive) {
            if (btn == BtnState::SHORT_CLICK) {
                menuEditStop();
                _oled->clear();
                drawMenu(_oled);
                _oled->display();
                return Error::NoData;
            }
            if (btn == BtnState::LONG_PRESS) {
                editActive = false;
                _oled->clear();
                drawMenu(_oled);
                _oled->display();
                return Error::NoData;
            }
            int encDelta = readEncoderDelta();
            if (encDelta != 0) {
                int step = (encDelta > 0) ? editStep : -editStep;
                editValue += step;
                if (editValue < editMin) editValue = editMin;
                if (editValue > editMax) editValue = editMax;
                if (editApplyCB) editApplyCB(editValue);
                _oled->clear();
                drawMenu(_oled);
                _oled->display();
            }
            return Error::NoData;
        }

        if (jogActive && menuActive) {
            if (btn == BtnState::SHORT_CLICK) {
                menuJogStop();
                resetEncoder();
                return Error::NoData;
            }
            if (btn == BtnState::LONG_PRESS) {
                jogActive = false;
                popScreen();
                encoderLine = 0;
                encoderTopLine = 0;
                return Error::NoData;
            }
            int encDelta = readEncoderDelta();
            if (encDelta >= 2 || encDelta <= -2) {
                uint32_t now = millis();
                if (now - _jog_last_ms > 150) {
                    _jog_last_ms = now;
                    int32_t steps = encDelta > 0 ? jogStepMm : -jogStepMm;
                    snprintf(line, Channel::maxLine, "$J=G91 G21 F500 %c%d\n",
                             "XYZ"[jogAxis], steps);
                    return Error::Ok;
                }
            }
            return Error::NoData;
        }

        if (menuActive) {
            if (screen_items > 0) {
                if (btn == BtnState::SHORT_CLICK) {
                    const char* gcode = menuSelect();
                    if (gcode && gcode[0] != '\0') {
                        bool isHome = (gcode[0] == '$' && (gcode[1] == 'H' || gcode[1] == 'X'));
                        if (isHome) {
                            menuActive = false;
                            goScreen(nullptr);
                            resetButtonState();
                            _needs_render = true;
                        }
                        strncpy(line, gcode, Channel::maxLine - 1);
                        line[Channel::maxLine - 1] = '\0';
                        return Error::Ok;
                    }
                    if (!menuActive) {
                        // Exit item selected — menu closed, render DRO next pollLine
                        _needs_render = true;
                    } else if (screen_items > 0) {
                        _oled->clear();
                        drawMenu(_oled);
                        _oled->display();
                    }
                }
                int encDelta = readEncoderDelta();
                if (encDelta != 0) {
                    uint32_t now = millis();
                    if (now - _menu_last_render > 200) {
                        _menu_last_render = now;
                        int step = (encDelta > 0) ? 1 : -1;
                        encoderLine += step;
                        if (encoderLine < 0)            encoderLine = screen_items - 1;
                        if (encoderLine >= screen_items) encoderLine = 0;
                        scroll_screen();
                        _oled->clear();
                        drawMenu(_oled);
                        _oled->display();
                    }
                }
            }
            return Error::NoData;
        }

        if (btn == BtnState::SHORT_CLICK) {
            if (_state != "Run") {
                _enc_selected_axis = (_enc_selected_axis + 1) % 3;
                _needs_render = true;
            }
        }
        // Peek at encoder position without consuming — accumulate across calls
        int encDelta = peekEncoderPos();
        if (encDelta >= 2 || encDelta <= -2) {
            resetEncoder();  // consume only when threshold met
            uint32_t now = millis();
            if (now - _jog_last_ms > 150) {
                _jog_last_ms = now;
                int steps = encDelta > 0 ? _jog_step_mm : -_jog_step_mm;
                snprintf(line, Channel::maxLine, "$J=G91 G21 F500 %c%d\n",
                         "XYZA"[_enc_selected_axis], steps);
                return Error::Ok;
            }
        }
    }

    renderDRO();
    return Error::NoData;
}

void OLED::show_state() {
    show(stateLayout, _state);
}

void OLED::show_limits(bool probe, const bool* limits) {
    if (_width != 128) {
        return;
    }
    if (_filename.length() != 0) {
        return;
    }
    if (_state == "Alarm") {
        return;
    }
    for (axis_t axis = X_AXIS; axis < 3; axis++) {
        draw_checkbox(80, 15 + (axis * 10), 7, 7, limits[axis]);
    }
}
void OLED::show_file() {
    Percent pct = Percent(_percent);
    if (_filename.length() == 0) {
        return;
    }
    if (_state != "Run" && pct == 100) {
        // This handles the case where the system returns to idle
        // but shows one last SD report
        return;
    }
    if (_width == 128) {
        show(percentLayout128, std::to_string(pct) + '%');

        _ticker += "-";
        if (_ticker.length() >= 12) {
            _ticker = "-";
        }
        show(tickerLayout, _ticker);

        wrapped_draw_string(14, _filename, ArialMT_Plain_16);

        _oled->drawProgressBar(0, 45, 120, 10, pct);
    } else {
        show(percentLayout64, std::to_string(pct) + '%');
    }
}
void OLED::show_dro(const float* axes, bool isMpos, bool* limits) {
    if (_state == "Alarm") {
        return;
    }
    if (_width == 128 && _filename.length()) {
        return;
    }

    auto n_axis = Axes::_numberAxis;
    char buf[24];

    // Axis rows
    _oled->setFont(ArialMT_Plain_10);
    for (axis_t axis = X_AXIS; axis < n_axis && axis < 3; axis++) {
        uint8_t y = 12 + (axis * 10);

        if (axis == _enc_selected_axis && _width == 128) {
            _oled->fillRect(0, y + 2, 54, 9);
            _oled->setColor(BLACK);
        }

        snprintf(buf, sizeof(buf), "%c:", Machine::Axes::axisName(axis)[0]);
        _oled->setTextAlignment(TEXT_ALIGN_LEFT);
        _oled->drawString(0, y, buf);

        _oled->setTextAlignment(TEXT_ALIGN_RIGHT);
        snprintf(buf, sizeof(buf), "%.3f", axes[axis]);
        _oled->drawString(55, y, buf);

        if (axis == _enc_selected_axis && _width == 128) {
            _oled->setColor(WHITE);
        }
    }

    // Bottom line: feed rate + spindle speed
    _oled->setFont(ArialMT_Plain_10);
    _oled->setTextAlignment(TEXT_ALIGN_LEFT);
    snprintf(buf, sizeof(buf), "F:%.0f S:%.0f", _feed_rate, _spindle_speed);
    _oled->drawString(0, 54, buf);
}

void OLED::show_radio_info() {
    if (_filename.length()) {
        return;
    }
    if (_width == 128) {
        if (_state == "Alarm") {
            wrapped_draw_string(18, _radio_info, ArialMT_Plain_10);
            wrapped_draw_string(30, _radio_addr, ArialMT_Plain_10);
        } else if (_state != "Run") {
            show(radioAddrLayout, _radio_addr);
        }
    } else {
        if (_state == "Alarm") {
            wrapped_draw_string(10, _radio_info, ArialMT_Plain_10);
            wrapped_draw_string(28, _radio_addr, ArialMT_Plain_10);
        }
    }
}

void OLED::parse_numbers(std::string s, float* nums, uint8_t maxnums) {
    size_t pos     = 0;
    size_t nextpos = -1;
    size_t i       = 0;
    do {
        if (i >= maxnums) {
            return;
        }
        nextpos  = s.find_first_of(",", pos);
        auto num = s.substr(pos, nextpos - pos);
        string_util::from_float(num, nums[i++]);
        pos = nextpos + 1;
    } while (nextpos != std::string::npos);
}

void OLED::parse_axes(std::string s, float* axes) {
    size_t pos     = 0;
    size_t nextpos = -1;
    axis_t axis    = X_AXIS;
    do {
        nextpos  = s.find_first_of(",", pos);
        auto num = s.substr(pos, nextpos - pos);
        if (axis < MAX_N_AXIS) {
            string_util::from_float(num, axes[axis++]);
        }
        pos = nextpos + 1;
    } while (nextpos != std::string::npos);
}

void OLED::parse_status_report() {
    if (_report.back() == '>') {
        _report.pop_back();
    }
    // Now the string is a sequence of field|field|field
    size_t pos     = 0;
    auto   nextpos = _report.find_first_of("|", pos);
    _state         = _report.substr(pos + 1, nextpos - pos - 1);

    bool probe              = false;
    bool limits[MAX_N_AXIS] = { false };

    float axes[MAX_N_AXIS];
    bool  isMpos = false;
    _filename    = "";
    uint32_t linenum;

    // ... handle it
    while (nextpos != std::string::npos) {
        pos        = nextpos + 1;
        nextpos    = _report.find_first_of("|", pos);
        auto field = _report.substr(pos, nextpos - pos);
        // MPos:, WPos:, Bf:, Ln:, FS:, Pn:, WCO:, Ov:, A:, SD: (ISRs:, Heap:)
        auto colon = field.find_first_of(":");
        auto tag   = field.substr(0, colon);
        auto value = field.substr(colon + 1);
        if (tag == "MPos") {
            // x,y,z,...
            parse_axes(value, axes);
            isMpos = true;
            continue;
        }
        if (tag == "WPos") {
            // x,y,z...
            parse_axes(value, axes);
            isMpos = false;
            continue;
        }
        if (tag == "Bf") {
            // buf_avail,rx_avail
            continue;
        }
        if (tag == "Ln") {
            // n
            string_util::from_decimal(value, linenum);
            continue;
        }
        if (tag == "FS") {
            // feedrate,spindle_speed
            float fs[2];
            parse_numbers(value, fs, 2);  // feed in [0], spindle in [1]
            _feed_rate     = fs[0];
            _spindle_speed = fs[1];
            continue;
        }
        if (tag == "Pn") {
            // PXxYy etc
            for (char const& c : value) {
                switch (c) {
                    case 'P':
                        probe = true;
                        break;
                    case 'X':
                        limits[X_AXIS] = true;
                        break;
                    case 'Y':
                        limits[Y_AXIS] = true;
                        break;
                    case 'Z':
                        limits[Z_AXIS] = true;
                        break;
                    case 'A':
                        limits[A_AXIS] = true;
                        break;
                    case 'B':
                        limits[B_AXIS] = true;
                        break;
                    case 'C':
                        limits[C_AXIS] = true;
                        break;
                }
                continue;
            }
        }
        if (tag == "WCO") {
            // x,y,z,...
            // We do not use the WCO values because the DROs show whichever
            // position is in the status report
            // float wcos[MAX_N_AXIS];
            // auto  wcos = parse_axes(value, wcos);
            continue;
        }
        if (tag == "Ov") {
            // feed_ovr,rapid_ovr,spindle_ovr
            float frs[3];
            parse_numbers(value, frs, 3);  // feed in [0], rapid in [1], spindle in [2]
            continue;
        }
        if (tag == "A") {
            // SCFM
            /* Unused.
            uint8_t spindle = 0;
            bool    flood   = false;
            bool    mist    = false;
            for (char const& c : value) {
                switch (c) {
                    case 'S':
                        spindle = 1;
                        break;
                    case 'C':
                        spindle = 2;
                        break;
                    case 'F':
                        flood = true;
                        break;
                    case 'M':
                        mist = true;
                        break;
                }
            }
            */
            continue;
        }
        if (tag == "SD") {
            auto commaPos = value.find_first_of(",");
            string_util::from_float(value.substr(0, commaPos), _percent);
            _filename = value.substr(commaPos + 1);
            continue;
        }
    }
    if (menuActive) {
        // Menu display is rendered from pollLine(), not here
    } else {
        _axes[0] = axes[0]; _axes[1] = axes[1]; _axes[2] = axes[2];
        _limits[0] = limits[0]; _limits[1] = limits[1]; _limits[2] = limits[2];
        _probe  = probe;
        _isMpos = isMpos;
        _needs_render = true;
    }

    // Alarm buzzer - beep continuously while in alarm state
    if (_buz_pin.defined() && _state == "Alarm") {
        static uint32_t lastBeep = 0;
        static bool     beepOn   = false;
        uint32_t now = millis();
        if (now - lastBeep > 500) {
            lastBeep = now;
            beepOn   = !beepOn;
            _buz_pin.synchronousWrite(beepOn);
        }
    } else if (_buz_pin.defined()) {
        _buz_pin.synchronousWrite(false);
    }
}

void OLED::renderDRO() {
    if (_needs_render && !menuActive) {
        _needs_render = false;
        _oled->clear();
        show_state();
        show_file();
        show_limits(_probe, _limits);
        show_dro(_axes, _isMpos, _limits);
        show_radio_info();
        _oled->display();
    }
}

void OLED::parse_gcode_report() {
    size_t pos     = 0;
    size_t nextpos = _report.find_first_of(":", pos);
    auto   name    = _report.substr(pos, nextpos - pos);
    if (name != "[GC") {
        return;
    }
    pos = nextpos + 1;
    do {
        nextpos  = _report.find_first_of(" ", pos);
        auto tag = _report.substr(pos, nextpos - pos);
        // G80 G0 G1 G2 G3  G38.2 G38.3 G38.4 G38.5
        // G54 .. G59
        // G17 G18 G19
        // G20 G21
        // G90 G91
        // G94 G93
        // M0 M1 M2 M30
        // M3 M4 M5
        // M7 M8 M9
        // M56
        // Tn
        // Fn
        // Sn
        //        if (tag == "G0") {
        //            continue;
        //        }
        pos = nextpos + 1;
    } while (nextpos != std::string::npos);
}

// [MSG:INFO: Connecting to STA:SSID foo]
void OLED::parse_STA() {
    size_t start = strlen("[MSG:INFO: Connecting to STA SSID:");
    _radio_info  = _report.substr(start, _report.size() - start - 1);

    _oled->clear();
    wrapped_draw_string(0, _radio_info, ArialMT_Plain_10);
    _oled->display();
}

// [MSG:INFO: Connected - IP is 192.168.68.134]
void OLED::parse_IP() {
    size_t start = _report.rfind(" ") + 1;
    _radio_addr  = _report.substr(start, _report.size() - start - 1);

    _oled->clear();
    auto fh = font_height(ArialMT_Plain_10);
    wrapped_draw_string(0, _radio_info, ArialMT_Plain_10);
    wrapped_draw_string(fh * 2, _radio_addr, ArialMT_Plain_10);
    _oled->display();
    dwell_ms(_radio_delay, DwellMode::SysSuspend);
}

// [MSG:INFO: AP SSID foo IP 192.168.68.134 mask foo channel foo]
void OLED::parse_AP() {
    size_t start    = strlen("[MSG:INFO: AP SSID ");
    size_t ssid_end = _report.rfind(" IP ");
    size_t ip_end   = _report.rfind(" mask ");
    size_t ip_start = ssid_end + strlen(" IP ");

    _radio_info = "AP: ";
    _radio_info += _report.substr(start, ssid_end - start);
    _radio_addr = _report.substr(ip_start, ip_end - ip_start);

    _oled->clear();
    auto fh = font_height(ArialMT_Plain_10);
    wrapped_draw_string(0, _radio_info, ArialMT_Plain_10);
    wrapped_draw_string(fh * 2, _radio_addr, ArialMT_Plain_10);
    _oled->display();
    dwell_ms(_radio_delay, DwellMode::SysSuspend);
}

void OLED::parse_BT() {
    size_t      start  = strlen("[MSG:INFO: BT Started with ");
    std::string btname = _report.substr(start, _report.size() - start - 1);
    _radio_info        = "BT: ";
    _radio_info += btname.c_str();

    _oled->clear();
    wrapped_draw_string(0, _radio_info, ArialMT_Plain_10);
    _oled->display();
    dwell_ms(_radio_delay, DwellMode::SysSuspend);
}

void OLED::parse_WebUI() {
    size_t      start  = strlen("[MSG:INFO: WebUI: Request from ");
    std::string ipaddr = _report.substr(start, _report.size() - start - 1);

    _oled->clear();
    auto fh = font_height(ArialMT_Plain_10);
    wrapped_draw_string(0, "WebUI from", ArialMT_Plain_10);
    wrapped_draw_string(fh * 2, ipaddr, ArialMT_Plain_10);
    _oled->display();
}

void OLED::parse_report() {
    if (_report.length() == 0) {
        return;
    }
    if (_report.rfind("<", 0) == 0) {
        parse_status_report();
        return;
    }
    if (_report.rfind("[GC:", 0) == 0) {
        parse_gcode_report();
        return;
    }
    if (_report.rfind("[MSG:INFO: Connecting to STA SSID:", 0) == 0) {
        parse_STA();
        return;
    }
    if (_report.rfind("[MSG:INFO: Connected", 0) == 0) {
        parse_IP();
        return;
    }
    if (_report.rfind("[MSG:INFO: AP SSID ", 0) == 0) {
        parse_AP();
        return;
    }
    if (_report.rfind("[MSG:INFO: BT Started with ", 0) == 0) {
        parse_BT();
        return;
    }
    if (_report.rfind("[MSG:INFO: WebUI: Request from ", 0) == 0) {
        parse_WebUI();
        return;
    }
}

// This is how the OLED driver receives channel data
size_t OLED::write(uint8_t data) {
    char c = data;
    if (c == '\r') {
        return 1;
    }
    if (c == '\n') {
        parse_report();
        _report = "";
        return 1;
    }
    _report += c;
    return 1;
}

uint8_t OLED::font_width(font_t font) {
    return ((uint8_t*)font)[0];
}
uint8_t OLED::font_height(font_t font) {
    return ((uint8_t*)font)[1];
}
struct glyph_t {
    uint8_t msb;
    uint8_t lsb;
    uint8_t size;
    uint8_t width;
};
struct xfont_t {
    uint8_t width;
    uint8_t height;
    uint8_t first;
    uint8_t nchars;
    glyph_t glyphs[];
};
size_t OLED::char_width(char c, font_t font) {
    xfont_t* xf    = (xfont_t*)font;
    int16_t  index = c - xf->first;
    return (index < 0) ? 0 : xf->glyphs[index].width;
}

void OLED::wrapped_draw_string(int16_t y, const std::string& s, font_t font) {
    _oled->setFont(font);
    _oled->setTextAlignment(TEXT_ALIGN_LEFT);

    size_t slen   = s.length();
    size_t swidth = 0;
    size_t i;
    for (i = 0; i < slen && swidth < _width; i++) {
        swidth += char_width(s[i], font);
        if (swidth > _width) {
            break;
        }
    }
    if (swidth < _width) {
        _oled->drawString(0, y, s.c_str());
    } else {
        _oled->drawString(0, y, s.substr(0, i).c_str());
        _oled->drawString(0, y + font_height(font) - 1, s.substr(i, slen).c_str());
    }
}

void OLED::draw_checkbox(int16_t x, int16_t y, int16_t width, int16_t height, bool checked) {
    if (checked) {
        _oled->fillRect(x, y, width, height);  // If log.0
    } else {
        _oled->drawRect(x, y, width, height);  // If log.1
    }
}

ConfigurableModuleFactory::InstanceBuilder<OLED> oled_module __attribute__((init_priority(104))) ("oled");
