#pragma once

#include <cstdint>

// ---- Menu types (Marlin-style) ----
class OLEDDisplay;  // forward decl
typedef void (*screenFunc_t)(OLEDDisplay*);

// ---- Constants ----
constexpr int MAX_SCREEN_DEPTH    = 8;    // Maximum nested screen depth
constexpr int LONG_PRESS_MS       = 800;  // ms hold for long-press
constexpr int ENCODER_DEBOUNCE_US = 1000; // encoder debounce window in µs
constexpr int VISIBLE_LINES       = 6;    // rows on 128×64 (1 title + 5 items)

// ---- Button state machine ----
enum class BtnState : uint8_t {
    IDLE,
    PRESSED,
    SHORT_CLICK,
    LONG_PRESS
};

// ---- Menu item descriptor ----
struct MenuItem {
    const char* label;    // Display label (PROGMEM string)
    void (*action)();     // Callback when selected
    const char* gcode;    // G-code to inject (or nullptr)
    bool        isSubmenu; // true = push submenu screen
};

// ---- Global menu state (extern, defined in Menu.cpp) ----
extern int8_t        encoderLine;      // Currently selected item index
extern int8_t        encoderTopLine;   // First visible line index
extern int8_t        screen_items;     // Total items in current screen
extern bool          menuActive;       // DRO ↔ Menu mode flag
extern bool          jogActive;        // Jog submenu active — encoder produces $J commands
extern int32_t       g_jogStepMM[3];   // Live X/Y/Z jog steps in mm (seeded from oled config)
extern int           jogAxis;          // Selected jog axis: 0=X, 1=Y, 2=Z
extern const MenuItem* currentMenuItems; // Items for current screen
extern volatile uint32_t g_sdFailMsgUntilMs; // SD-start fail message deadline (ms, millis() base)

// ---- Edit mode (contrast, speed override) ----
extern bool   editActive;       // Encoder adjusts a value instead of navigating
extern int    editValue;        // Current editing value
extern int    editMin;
extern int    editMax;
extern int    editStep;
extern int editingItem;      // -1=none, 0=contrast, 1=speed
extern int contrastValue;    // current contrast (synced with OLED config)

// ---- Menu item dispatch ----
const char* menuSelect();       // Returns gcode to inject (or nullptr) for current encoderLine
void menuJogStop();             // Deactivate jog mode, cancel queued jogs, return to jog submenu
void menuEditStop();            // Deactivate edit mode, apply value

// Called from pollLine() on every status report — dispatches on state
// transitions (probe wizard, SD watchdog, homing beep)
void menuNotifyState(const char* state);

// Cross-module beep bridge — routes to OLED::menuBeep (defined in OLED.cpp)
void panelBeep(uint16_t ms);

// True while the machine state reported to the OLED is Idle (safe to start a jog)
bool machineIdle();

// Edit mode apply callback — set by menu screens before entering edit mode
extern void (*editApplyCB)(int);

// ---- Probe Z wizard ----
enum class ProbeStep : uint8_t { IDLE, PLATE, PROBING, REMOVE, DONE, FAILED };
extern ProbeStep probeStep;

// ---- Pending g-code queue ----
// Queue holds '\n'-separated commands; pollLine() injects ONE command per call.
void gcodeQueuePush(const char* cmd);
bool gcodeQueuePop(char* out, size_t cap);
void cancelQueuedJogs();  // Send motion-cancel (flushes planner jog chain) + drop in-flight jog

// ---- Screen stack ----
void pushScreen(screenFunc_t screen);
void popScreen();
void goScreen(screenFunc_t screen);
screenFunc_t currentScreen();

// ---- Encoder / button interface ----
void     initEncoder(int en1Pin, int en2Pin, int encPin);
void     pollEncoder();  // Call from pollLine() — reads pins, applies debounce
int      readEncoderDelta();
BtnState readButtonState();
void     resetButtonState();
void     resetEncoder();
int      encoderPin1();
int      encoderPin2();
int      encoderBtnPin();
int      peekEncoderPos();   // Returns current encoder pos WITHOUT resetting

// ---- Menu drawing ----
void drawMenu(class OLEDDisplay* display);
void scroll_screen();

// ---- Menu screen declarations ----
void menu_main(class OLEDDisplay* display);
void menu_jog(class OLEDDisplay* display);
void menu_probe(class OLEDDisplay* display);
void menu_sd(class OLEDDisplay* display);
void menu_spindle(class OLEDDisplay* display);
void menu_settings(class OLEDDisplay* display);
void menu_info(class OLEDDisplay* display);

// ---- G-code injection helper ----
bool injectGcode(const char* gcode);

// ---- Info screen text ----
extern const char* (*infoText)(int field);  // field 0=IP, 1=WiFi SSID, 2=Version

// ---- Actions ----
void action_home();
void action_unlock();
