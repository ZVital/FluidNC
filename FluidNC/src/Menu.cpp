#include <Arduino.h>
#include "Menu.h"
#include <OLEDDisplay.h>
#include "Channel.h"
#include "FluidPath.h"
#include <dirent.h>
#include "InputFile.h"
#include "Job.h"

// ---- Global state ----
int8_t encoderLine       = 0;
int8_t encoderTopLine    = 0;
int8_t screen_items      = 0;
bool   menuActive        = false;
bool   alarmBlocked      = false;
bool   jogActive         = false;
int32_t jogStepMm        = 1;
int     jogAxis          = 0;    // 0=X, 1=Y, 2=Z
bool   editActive        = false;
int    editValue         = 0;
int    editMin           = 0;
int    editMax           = 255;
int    editStep          = 10;
const MenuItem* currentMenuItems = nullptr;

// ---- Pending G-code for pollLine() injection ----
static char _pendingGcode[Channel::maxLine];

// Edit mode apply callback — set by menu screens before entering edit mode
void (*editApplyCB)(int) = nullptr;

// ---- SD file browser state ----
char sdSelectedFile[64]    = { 0 };
volatile bool sdFilePending = false;

static constexpr int MAX_SD_FILES = 20;
static char sdFileNames[MAX_SD_FILES][64];
static int  sdFileCount = 0;

static void loadSdFiles() {
    sdFileCount = 0;  // SD card menu: use WebUI for file listing
}

static void sdSelectAction() {
    if (encoderLine < 0 || encoderLine >= sdFileCount) return;
    const char* name = sdFileNames[encoderLine];
    if (name[0] == '\0') return;
    char c = name[0];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))) return;
    memcpy(sdSelectedFile, name, 64);
    sdFilePending = true;
    popScreen();
}

// ---- Probe Z wizard ----
ProbeStep  probeStep          = ProbeStep::IDLE;
char       probeGcode[64]     = { 0 };
volatile bool probeGcodePending = false;

static void probeAdvance() {
    switch (probeStep) {
        case ProbeStep::PLATE:
            probeStep = ProbeStep::PROBING;
            snprintf(probeGcode, sizeof(probeGcode), "G38.2 Z-60 F60\n");
            probeGcodePending = true;
            break;
        case ProbeStep::PROBING:
            probeStep = ProbeStep::SUCCESS;
            break;
        case ProbeStep::SUCCESS:
            probeStep = ProbeStep::REMOVE;
            break;
        case ProbeStep::REMOVE:
            probeStep = ProbeStep::LIFTING;
            snprintf(probeGcode, sizeof(probeGcode), "G91 G0 Z5\nG90\nG92 Z0\n");
            probeGcodePending = true;
            break;
        case ProbeStep::DONE:
            probeStep = ProbeStep::IDLE;
            popScreen();
            break;
        default:
            break;
    }
}

void menuJogStop() {
    jogActive = false;
}

int contrastValue = 43;
static int _speedValue    = 100;
int editingItem   = -1;

void menuEditStop() {
    if (editActive) {
        if (editApplyCB) {
            editApplyCB(editValue);
        }
        switch (editingItem) {
            case 0: contrastValue = editValue; break;
            case 1: _speedValue = editValue; break;
        }
        editingItem = -1;
    }
    editActive = false;
}

const char* menuSelect() {
    _pendingGcode[0] = '\0';

    if (!currentMenuItems || encoderLine < 0 || encoderLine >= screen_items) {
        return nullptr;
    }

    const MenuItem& item = currentMenuItems[encoderLine];

    if (item.isSubmenu && item.action) {
        if (item.action == popScreen) {
            popScreen();
        } else {
            pushScreen(reinterpret_cast<screenFunc_t>(item.action));
        }
        encoderLine    = 0;
        encoderTopLine = 0;
        return nullptr;
    }

    if (item.gcode) {
        strncpy(_pendingGcode, item.gcode, sizeof(_pendingGcode) - 1);
        return _pendingGcode;
    }

    if (item.action) {
        item.action();
    }

    return nullptr;
}

// ---- Screen stack ----
static screenFunc_t _screenStack[MAX_SCREEN_DEPTH];
static int          _screenDepth = 0;

void pushScreen(screenFunc_t screen) {
    if (_screenDepth < MAX_SCREEN_DEPTH) {
        _screenStack[_screenDepth++] = screen;
    }
}

void popScreen() {
    if (_screenDepth > 0) {
        _screenDepth--;
    }
}

void goScreen(screenFunc_t screen) {
    _screenDepth = 0;
    if (screen) {
        pushScreen(screen);
    }
}

screenFunc_t currentScreen() {
    if (_screenDepth > 0) {
        return _screenStack[_screenDepth - 1];
    }
    return nullptr;
}

// ---- Encoder interface ----
static const int8_t _encTable[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

static int      _en1Pin    = -1;
static int      _en2Pin    = -1;
static int      _encPin    = -1;
static uint8_t  _encState  = 0;
static int32_t  _encPos    = 0;
static uint32_t _encLastUs = 0;
static uint32_t _btnLockUntil  = 0;
static const uint32_t BTN_ENCODER_MUTE_MS = 150;

void initEncoder(int en1Pin, int en2Pin, int encPin) {
    _en1Pin    = en1Pin;
    _en2Pin    = en2Pin;
    _encPin    = encPin;
    _encState  = 0;
    _encPos    = 0;
    _encLastUs = 0;

    if (_en1Pin >= 0) {
        pinMode(_en1Pin, INPUT_PULLUP);
        pinMode(_en2Pin, INPUT_PULLUP);
        pinMode(_encPin, INPUT_PULLUP);
        _encState = (digitalRead(_en1Pin) << 1) | digitalRead(_en2Pin);
        _encLastUs = micros();
    }
}

void pollEncoder() {
    if (_en1Pin < 0) return;

    int en1      = digitalRead(_en1Pin);
    int en2      = digitalRead(_en2Pin);
    int newState = (en1 << 1) | en2;

    uint32_t now = micros();
    if (newState != _encState) {
        if (now - _encLastUs >= ENCODER_DEBOUNCE_US) {
            int8_t dir = _encTable[(_encState << 2) | newState];
            if (dir != 0) {
                _encPos -= dir;  // inverted: CW = +, CCW = -
            }
            _encState  = newState;
            _encLastUs = now;
        }
    } else {
        _encLastUs = now;
    }
}

int readEncoderDelta() {
    if (millis() < _btnLockUntil) {
        _encPos = 0;
        return 0;
    }
    int delta = _encPos;
    _encPos = 0;
    return delta;
}

void resetEncoder() { _encPos = 0; }

int peekEncoderPos() { return _encPos; }

int encoderPin1()   { return _en1Pin; }
int encoderPin2()   { return _en2Pin; }
int encoderBtnPin() { return _encPin; }

// ---- Button state machine ----
static const uint32_t BTN_DEBOUNCE_MS = 5;

static uint32_t _btnDebounceMs = 0;
static bool     _btnDebouncing = false;
static bool     _btnWasDown    = false;
static bool     _btnConsumed   = false;
static uint32_t _btnPressMs    = 0;

BtnState readButtonState() {
    if (_encPin < 0) return BtnState::IDLE;

    bool btnDown = !digitalRead(_encPin);
    uint32_t now = millis();

    if (btnDown && !_btnWasDown && !_btnDebouncing) {
        _btnDebounceMs = now;
        _btnDebouncing = true;
        _btnLockUntil  = now + BTN_ENCODER_MUTE_MS;
        return BtnState::IDLE;
    }

    if (_btnDebouncing) {
        if (btnDown && (now - _btnDebounceMs >= BTN_DEBOUNCE_MS)) {
            _btnWasDown    = true;
            _btnDebouncing = false;
            _btnPressMs    = now;
            _btnConsumed   = false;
            return BtnState::PRESSED;
        }
        if (!btnDown) {
            _btnDebouncing = false;
            return BtnState::IDLE;
        }
        return BtnState::IDLE;
    }

    if (btnDown && _btnWasDown && !_btnConsumed) {
        if (now - _btnPressMs >= LONG_PRESS_MS) {
            _btnConsumed = true;
            return BtnState::LONG_PRESS;
        }
    }

    if (!btnDown && _btnWasDown) {
        _btnWasDown = false;
        _btnLockUntil = millis() + BTN_ENCODER_MUTE_MS;
        if (!_btnConsumed) {
            return BtnState::SHORT_CLICK;
        }
    }

    return BtnState::IDLE;
}

void resetButtonState() {
    _btnWasDown    = false;
    _btnConsumed   = true;  // prevent spurious SHORT_CLICK on release
    _btnPressMs    = 0;
    _btnDebouncing = false;
}

// ---- Actions ----
void action_home() {
    // Home All is handled via gcode injection from menuSelect()
}

void action_unlock() {
    // $X unlock is handled via gcode injection from menuSelect()
}

static void action_exitMenu() {
    menuActive = false;
    goScreen(nullptr);
    resetButtonState();
}

bool injectGcode(const char* gcode) {
    if (!gcode || gcode[0] == '\0') return false;
    strncpy(_pendingGcode, gcode, sizeof(_pendingGcode) - 1);
    return true;
}

// ---- Draw helpers ----
static void drawMenuItem(OLEDDisplay* display, int row, const char* label, bool selected, const char* value = nullptr) {
    int y = 0;
    if (row > 0) {
        y = 10 + (row - 1) * 10;
    }

    if (selected && row > 0) {
        display->fillRect(0, y, 128, 10);
        display->setColor(BLACK);
    }

    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(2, row == 0 ? 0 : y, label);

    if (value && row > 0) {
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(126, y, value);
    }

    if (selected && row > 0) {
        display->setColor(WHITE);
    }
}

// Draw a standard scrollable item list
static void drawItemList(OLEDDisplay* display, const char* title) {
    // Draw title
    drawMenuItem(display, 0, title, false);

    // Draw visible items
    for (int row = 1; row < VISIBLE_LINES; row++) {
        int idx = encoderTopLine + (row - 1);
        if (idx >= screen_items) break;
        if (currentMenuItems && currentMenuItems[idx].label) {
            drawMenuItem(display, row, currentMenuItems[idx].label, idx == encoderLine);
        }
    }
}

// ---- Info screen data ----
// These are populated from OLED's _radio_info, _radio_addr via a pointer
const char* (*infoText)(int field) = nullptr;  // field 0=IP, 1=WiFi SSID, 2=Version

// ---- Menu screen functions ----
static const MenuItem menu_main_items[] = {
    { "Home All",  nullptr,                    "$H\n",                             false },
    { "Jog",       (void(*)())menu_jog,        nullptr,                            true  },
    { "Probe Z",   (void(*)())menu_probe,      nullptr,                            true  },
    { "SD Card",   (void(*)())menu_sd,         nullptr,                            true  },
    { "Spindle",   (void(*)())menu_spindle,    nullptr,                            true  },
    { "Settings",  (void(*)())menu_settings,   nullptr,                            true  },
    { "Info",      (void(*)())menu_info,       nullptr,                            true  },
    { "Exit",      (void(*)())action_exitMenu, nullptr,                            false },
};

void menu_main(OLEDDisplay* display) {
    screen_items = sizeof(menu_main_items) / sizeof(menu_main_items[0]);
    currentMenuItems = menu_main_items;
    scroll_screen();
    drawItemList(display, "Main Menu");
}

// ---- Jog submenu helpers ----
static void jogSelectAxis0() { jogAxis = 0; jogActive = true; }
static void jogSelectAxis1() { jogAxis = 1; jogActive = true; }
static void jogSelectAxis2() { jogAxis = 2; jogActive = true; }

static void jogCycleStep() {
    static const int32_t steps[] = { 1, 10, 100 };
    static int idx = 0;
    idx = (idx + 1) % 3;
    jogStepMm = steps[idx];
}

void menu_jog(OLEDDisplay* display) {
    static char stepBuf[16];
    snprintf(stepBuf, sizeof(stepBuf), "Step: %dmm", (int)jogStepMm);

    static const MenuItem items[] = {
        { "< Back",    (void(*)())popScreen,       nullptr, true  },
        { "X Axis",    (void(*)())jogSelectAxis0,   nullptr, false },
        { "Y Axis",    (void(*)())jogSelectAxis1,   nullptr, false },
        { "Z Axis",    (void(*)())jogSelectAxis2,   nullptr, false },
        { stepBuf,     (void(*)())jogCycleStep,     nullptr, false },
    };
    screen_items = sizeof(items) / sizeof(items[0]);
    currentMenuItems = items;
    scroll_screen();
    drawItemList(display, "Jog");
}

void menu_probe(OLEDDisplay* display) {
    if (probeStep == ProbeStep::IDLE) {
        probeStep = ProbeStep::PLATE;
    }

    static char msg[64];
    static MenuItem items[2];

    items[0] = { "< Back", (void(*)())popScreen, nullptr, true };
    items[0].label = "< Back";

    switch (probeStep) {
        case ProbeStep::PLATE:
            snprintf(msg, sizeof(msg), "Place plate, click start");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::PROBING:
            snprintf(msg, sizeof(msg), "Click when probing done");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::SUCCESS:
            snprintf(msg, sizeof(msg), "Probed! Click next");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::REMOVE:
            snprintf(msg, sizeof(msg), "Remove plate, click");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::LIFTING:
            snprintf(msg, sizeof(msg), "Lifting Z...");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::DONE:
            snprintf(msg, sizeof(msg), "Z=0 set. Click done");
            items[1] = { msg, (void(*)())probeAdvance, nullptr, false };
            break;
        case ProbeStep::FAILED:
            snprintf(msg, sizeof(msg), "Probe failed");
            items[1] = { msg, nullptr, nullptr, false };
            break;
        default:
            break;
    }

    screen_items = 2;
    currentMenuItems = &items[0];
    scroll_screen();
    drawItemList(display, "Probe Z");
}

void menu_sd(OLEDDisplay* display) {
    loadSdFiles();

    if (sdFileCount < 0) {
        static const MenuItem items[] = {
            { "< Back", (void(*)())popScreen, nullptr, true },
        };
        screen_items = 2;
        currentMenuItems = items;
        scroll_screen();
        drawItemList(display, "SD Card");
        display->setFont(ArialMT_Plain_10);
        display->drawString(2, 20, "SD not mounted");
        return;
    }

    // Build menu items: Back + up to MAX_SD_FILES file entries
    struct SdMenuItem {
        MenuItem item;
        char     label[64];
    };
    // We use a static pool so the items persist across display refreshes
    static SdMenuItem sdMenuBuf[MAX_SD_FILES + 1];
    static int        cachedCount = -1;

    if (sdFileCount != cachedCount) {
        cachedCount = sdFileCount;
        // Back button
        sdMenuBuf[0].item = { nullptr, (void(*)())popScreen, nullptr, true };
        snprintf(sdMenuBuf[0].label, sizeof(sdMenuBuf[0].label), "< Back");
        sdMenuBuf[0].item.label = sdMenuBuf[0].label;

        for (int i = 0; i < sdFileCount && i < MAX_SD_FILES; i++) {
            snprintf(sdMenuBuf[i + 1].label, sizeof(sdMenuBuf[i + 1].label), "%s", sdFileNames[i]);
            sdMenuBuf[i + 1].item.label     = sdMenuBuf[i + 1].label;
            sdMenuBuf[i + 1].item.action    = sdSelectAction;
            sdMenuBuf[i + 1].item.gcode     = nullptr;
            sdMenuBuf[i + 1].item.isSubmenu = false;
        }
    }

    screen_items = sdFileCount + 1;  // +1 for Back
    currentMenuItems = &sdMenuBuf[0].item;
    scroll_screen();
    drawItemList(display, "SD Card");
    display->setFont(ArialMT_Plain_10);
    display->drawString(2, 20, "Use WebUI for files");
}

// ---- Spindle/Laser control ----
static int spindleSpeed = 50;  // 0-100%

static void cycleSpindleSpeed() {
    spindleSpeed += 10;
    if (spindleSpeed > 100) spindleSpeed = 0;
}

void menu_spindle(OLEDDisplay* display) {
    static char speedBuf[24];
    static char startGcode[24];
    snprintf(speedBuf, sizeof(speedBuf), "Speed: %d%%", spindleSpeed);
    snprintf(startGcode, sizeof(startGcode), "M3 S%d\n", spindleSpeed * 10);

    static const MenuItem items[] = {
        { "< Back",       (void(*)())popScreen,            nullptr,    true  },
        { "Start Laser",  nullptr,                         startGcode, false },
        { "Stop Laser",   nullptr,                         "M5\n",     false },
        { speedBuf,       (void(*)())cycleSpindleSpeed,     nullptr,    false },
    };
    screen_items = sizeof(items) / sizeof(items[0]);
    currentMenuItems = items;
    scroll_screen();
    drawItemList(display, "Laser");
}

// ---- Settings actions ----
static void settingsStartContrast() {
    editMin   = 30;
    editMax   = 50;
    editStep  = 1;
    editValue = contrastValue;
    editingItem = 0;
    editActive = true;
}

static void settingsStartSpeed() {
    editMin   = 1;
    editMax   = 200;
    editStep  = 5;
    editValue = _speedValue;
    editingItem = 1;
    editActive = true;
}

static void settingsCycleJogStep() {
    static const int32_t steps[] = { 1, 10, 100 };
    static int idx = 0;
    idx = (idx + 1) % 3;
    jogStepMm = steps[idx];
}

void menu_settings(OLEDDisplay* display) {
    if (editActive) {
        const char* title = (editingItem == 0) ? "Contrast" :
                            (editingItem == 1) ? "Speed" : "Settings";
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(64, 10, title);
        char buf[32];
        snprintf(buf, sizeof(buf), "Value: %d", editValue);
        display->drawString(64, 24, buf);
        return;
    }

    static char jogStepBuf[16];
    snprintf(jogStepBuf, sizeof(jogStepBuf), "Jog Step: %dmm", (int)jogStepMm);

    static char contrastBuf[16];
    snprintf(contrastBuf, sizeof(contrastBuf), "Contrast: %d", contrastValue);

    static char speedBuf[16];
    snprintf(speedBuf, sizeof(speedBuf), "Speed: %d%%", _speedValue);

    static const MenuItem items[] = {
        { "< Back",       (void(*)())popScreen,             nullptr, true  },
        { jogStepBuf,     (void(*)())settingsCycleJogStep,   nullptr, false },
        { contrastBuf,    (void(*)())settingsStartContrast,   nullptr, false },
        { speedBuf,       (void(*)())settingsStartSpeed,      nullptr, false },
    };
    screen_items = sizeof(items) / sizeof(items[0]);
    currentMenuItems = items;
    scroll_screen();
    drawItemList(display, "Settings");
}

void menu_info(OLEDDisplay* display) {
    static const MenuItem items[] = {
        { "< Back",    (void(*)())popScreen,    nullptr, true },
    };
    screen_items = sizeof(items) / sizeof(items[0]);
    currentMenuItems = items;
    scroll_screen();

    drawItemList(display, "Info");

    if (infoText) {
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        for (int i = 0; i < 3; i++) {
            const char* txt = infoText(i);
            if (txt && txt[0] != '\0') {
                display->drawString(2, 30 + i * 10, txt);
            }
        }
    }
}

// ---- Scroll rendering ----
constexpr int ITEMS_PER_PAGE = VISIBLE_LINES - 1;  // rows 1..5, row 0 = title

void scroll_screen() {
    if (screen_items <= ITEMS_PER_PAGE) {
        encoderTopLine = 0;
        return;
    }
    if (encoderLine < encoderTopLine) {
        encoderTopLine = encoderLine;
    }
    if (encoderLine >= encoderTopLine + ITEMS_PER_PAGE) {
        encoderTopLine = encoderLine - ITEMS_PER_PAGE + 1;
    }
    if (encoderTopLine > screen_items - ITEMS_PER_PAGE) {
        encoderTopLine = screen_items - ITEMS_PER_PAGE;
    }
    if (encoderTopLine < 0) encoderTopLine = 0;
}

void drawMenu(OLEDDisplay* display) {
    screenFunc_t screen = currentScreen();
    if (!screen) return;

    screen(display);

    // Show scroll indicators
    if (screen_items > ITEMS_PER_PAGE) {
        display->setFont(ArialMT_Plain_10);
        if (encoderTopLine > 0) {
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawString(126, 0, "^");
        }
        if (encoderTopLine + VISIBLE_LINES < screen_items) {
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawString(126, 56, "v");
        }
    }
}
