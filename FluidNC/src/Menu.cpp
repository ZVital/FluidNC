#include <Arduino.h>
#include "Menu.h"
#include <OLEDDisplay.h>
#include "Channel.h"
#include "Protocol.h"
#include "MotionControl.h"

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

// ---- Pending g-code queue for pollLine() injection ----
// pollLine() may inject exactly one command per call, so multi-command
// sequences are queued '\n'-separated and drained one line at a time.
static char _gcodeQueue[192];
static size_t _gcodeQueueLen = 0;

void gcodeQueuePush(const char* cmd) {
    if (!cmd || cmd[0] == '\0') return;
    size_t len = strlen(cmd);
    if (_gcodeQueueLen + len + 1 >= sizeof(_gcodeQueue)) {
        return;  // queue full - drop rather than corrupt
    }
    memcpy(_gcodeQueue + _gcodeQueueLen, cmd, len);
    _gcodeQueueLen += len;
    _gcodeQueue[_gcodeQueueLen++] = '\n';
}

bool gcodeQueuePop(char* out, size_t cap) {
    if (_gcodeQueueLen == 0) return false;
    const char* nl      = (const char*)memchr(_gcodeQueue, '\n', _gcodeQueueLen);
    size_t      linelen = nl ? (size_t)(nl - _gcodeQueue) : _gcodeQueueLen;
    if (linelen >= cap) linelen = cap - 1;
    memcpy(out, _gcodeQueue, linelen);
    out[linelen] = '\0';
    size_t consumed = (nl ? 1 : 0) + (nl ? (size_t)(nl - _gcodeQueue) : _gcodeQueueLen);
    memmove(_gcodeQueue, _gcodeQueue + consumed, _gcodeQueueLen - consumed);
    _gcodeQueueLen -= consumed;
    return true;
}

void cancelQueuedJogs() {
    // Same path as the 0x85 real-time byte: flushes the planner jog chain and
    // kills any parsed-but-not-yet-planned jog.
    protocol_send_event(&motionCancelEvent);
    mc_cancel_jog();
}

// ---- Probe Z wizard ----
ProbeStep  probeStep = ProbeStep::IDLE;

static void probeStart() {
    probeStep = ProbeStep::PROBING;
    gcodeQueuePush("G38.2 Z-60 F60");
}

static void probeFinishProbing() {
    if (machineIdle()) {
        // Tool is still at the contact point here - zero BEFORE lifting,
        // otherwise the datum ends up one lift-height above the surface.
        gcodeQueuePush("G92 Z0");
        probeStep = ProbeStep::REMOVE;
        return;
    }
    // Not Idle after probing means an alarm (missed contact / fault)
    probeStep = ProbeStep::FAILED;
}

static void probeLift() {
    gcodeQueuePush("G91 G0 Z5");
    gcodeQueuePush("G90");
    probeStep = ProbeStep::DONE;
}

static void probeRetry() {
    probeStep = ProbeStep::PLATE;
}

void menuJogStop() {
    jogActive = false;
    // Already-planned $J blocks would keep executing otherwise
    cancelQueuedJogs();
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
    if (jogActive) {
        cancelQueuedJogs();
    }
    jogActive   = false;
    editActive  = false;
    editingItem = -1;
    menuActive  = false;
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
static void jogSelectAxis0() {
    if (!machineIdle()) return;
    jogAxis = 0;
    jogActive = true;
}
static void jogSelectAxis1() {
    if (!machineIdle()) return;
    jogAxis = 1;
    jogActive = true;
}
static void jogSelectAxis2() {
    if (!machineIdle()) return;
    jogAxis = 2;
    jogActive = true;
}

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
    void (*stepAction)() = nullptr;

    items[0] = { "< Back", (void(*)())popScreen, nullptr, true };

    switch (probeStep) {
        case ProbeStep::PLATE:
            snprintf(msg, sizeof(msg), "Place plate, click start");
            stepAction = probeStart;
            break;
        case ProbeStep::PROBING:
            snprintf(msg, sizeof(msg), "Probing... click when stopped");
            stepAction = probeFinishProbing;
            break;
        case ProbeStep::REMOVE:
            snprintf(msg, sizeof(msg), "Z=0 set. Remove plate, click");
            stepAction = probeLift;
            break;
        case ProbeStep::DONE:
            snprintf(msg, sizeof(msg), "Click to finish");
            stepAction = [] {
                probeStep = ProbeStep::IDLE;
                popScreen();
            };
            break;
        case ProbeStep::FAILED:
            snprintf(msg, sizeof(msg), "Probe failed. Click to retry");
            stepAction = probeRetry;
            break;
        default:
            break;
    }

    items[1] = { msg, (void(*)())stepAction, nullptr, false };

    screen_items = 2;
    currentMenuItems = &items[0];
    scroll_screen();
    drawItemList(display, "Probe Z");
}

void menu_sd(OLEDDisplay* display) {
    // File browsing lives in the WebUI; the panel only offers a way back.
    static const MenuItem items[] = {
        { "< Back", (void(*)())popScreen, nullptr, true },
    };
    screen_items = sizeof(items) / sizeof(items[0]);
    currentMenuItems = items;
    encoderLine      = 0;
    encoderTopLine   = 0;
    drawItemList(display, "SD Card");
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64, 30, "Use WebUI for files");
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
        if (encoderTopLine + ITEMS_PER_PAGE < screen_items) {
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawString(126, 56, "v");
        }
    }
}
