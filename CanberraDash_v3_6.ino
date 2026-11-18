/*
 * Canberra Dash - clock / date / calendar / weather / moon / Claude voice
 * Version: 3.6
 *
 * Changelog:
 *   1.0  First version (LVGL display layer, blank screen on the FNK0104S)
 *   2.0  Screen driver + presentation taken from Aircraft_Tracker_v2_0_3 (plain TFT_eSPI)
 *   2.1  Date text enlarged to fill its zone; 12 / 3 / 6 / 9 numerals on the watch face;
 *        version number shown on the start-up screen
 *   2.2  Date smaller and centred between clock and boxes; all location data (lat/lon, timezones,
 *        place name) moved to secrets.h; digital time one size larger
 *   2.3  Voice: shows microphone level on failure, serial mic diagnostics, no endless wait if the
 *        microphone delivers no data
 *   2.4  Date drawn 40% larger using smooth scaling (DATE_SCALE_PCT)
 *   2.5  Voice rewritten for the real hardware: ES8311 codec + on-board mic (pins from the schematic)
 *   2.6  ES8311 start-up sequence completed (analog bias registers), mic read from whichever
 *        I2S slot carries audio, lower speech threshold, left/right levels shown on failure
 *   2.7  Microphone gain settings (MIC_PGA_GAIN, MIC_ADC_SCALE) and ES8311 register dump on Serial
 *   2.8  Voice: speech threshold now relative to room noise, more mic gain, recording is levelled
 *        before speech-to-text; weather retries every 20 s and logs the error text
 *   2.9  Date 10% larger (DATE_SCALE_PCT 140 -> 154)
 *   2.10 API errors now show the server message; voice status wraps to 2 lines; weather failure
 *        prints TLS / DNS / heap details
 *   3.0  Settings page (gear icon): WiFi scan + on-screen password keyboard, time zone, city.
 *        Time is drawn with smooth-scaled 7-segment digits. Settings are saved in flash.
 *   3.1  Attempted fix for the PickKind compile error (enum moved, but still below the first function)
 *   3.2  PickKind enum now above the first function in the file, so the IDE prototypes can see it
 *   3.3  Speaker fixed: amplifier enable pin is active LOW (as in Freenove Sketch_07_1_Music);
 *        short beep when the voice box starts listening (also proves the speaker works)
 *   3.4  WiFi scan fixed: stops reconnect attempts first (they made scans fail), retries once,
 *        shows hidden networks, and reports the scan result code; no auto-reconnect while in Settings
 *   3.5  Speaker volume slider on the Settings page (drag or tap; saved; plays a beep when released)
 *   3.6  Screen brightness slider on the Settings page (backlight PWM on GPIO45, 10-100%, saved)
 *
 * Screen driver: plain TFT_eSPI + FT6336U (ST77922 sprite path for the 3.5" board).
 * No LVGL and no display.cpp/display.h are used - just like the tracker sketch.
 *
 * Needs: TFT_eSPI, FT6336U (or the ST77922 libs for that board), ArduinoJson v7+
 *
 * Home screen (landscape):
 *   top 35%   Canberra time: analog face + digital h:mm AM/PM (auto daylight saving)
 *   next 15%  date  dd MMMMMMM yyyy
 *   last 50%  4 boxes: [Calendar] [Weather] [Moon] [Ask Claude]
 * Tap a box for the detail screen; HOME button (top left) comes back.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <time.h>
#include <math.h>
#include "secrets.h"

#define FW_VERSION "3.6"

// Types used in function parameters must be declared before the first function in the file,
// because the Arduino IDE inserts its auto-generated prototypes just above that first function.
enum PickKind { PK_TZ, PK_CITY, PK_WIFI };

// ====================== USER SETTINGS ======================
// WiFi, time zone and city are chosen on the Settings page (gear icon on the home screen).
#define DATE_SCALE_PCT       154    // date size as % of the basic font-4 size (154 = 54% larger); shrinks if too wide
#define WEEK_STARTS_MONDAY   1      // 0 = Sunday first
#define BOOT_TEST            1      // flash red/green/blue at power-up to prove the panel works
#define ENABLE_VOICE         1
#define CLAUDE_MODEL         "claude-haiku-5-5"   // fast for voice; "claude-sonnet-5-5" is smarter
#define BL_PIN             45   // LCD_BL on the FNK0104S schematic (GPIO45 -> MOSFET -> backlight)
#define SPK_VOLUME_DEFAULT 60   // 0..100, volume on first start (change it on the Settings page)

// Voice hardware: the FNK0104S has an ES8311 audio codec with an on-board microphone.
// Pins are taken from the board schematic. Connect a speaker to the SP+ / SP- header (JP3).
#define ES_I2C_SDA      16   // codec control bus, shared with the touch chip
#define ES_I2C_SCL      15
#define ES8311_ADDR     0x18
#define ES_MCLK          4
#define ES_BCLK          5
#define ES_WS            7
#define ES_DIN           6   // microphone data, codec -> ESP32  (schematic net "I2S_DO")
#define ES_DOUT          8   // speaker data, ESP32 -> codec     (schematic net "I2S_DI")
#define AUDIO_EN_PIN     1   // speaker amplifier shutdown / enable
#define AUDIO_EN_ACTIVE  LOW    // LOW = amplifier on (Freenove Sketch_07_1_Music drives this pin LOW)
// Microphone gain. The on-board mic (LMA2718B381-OA7) is analog, -38 dBV at 94 dB SPL, 60 dB SNR.
#define MIC_PGA_GAIN     10     // codec analog gain, 0..10 = 0..30 dB in 3 dB steps
#define MIC_ADC_SCALE     6     // codec digital gain, 0..7 = 0..42 dB in 6 dB steps
// ===========================================================

// ---------------- Screen driver (same as the tracker example) ----------------
#ifdef FNK0104N_3P5_320x480_ST77922
  #include "ST77922.h"
  #include "ST77922_Touch.h"

  TFT_eSPI tft_qspi = TFT_eSPI();
  TFT_eSprite tft = TFT_eSprite(&tft_qspi);
  ST77922 tft_st77922 = ST77922();
  ST77922_TOUCH touch_st77922;

  void flushScreen() {
    tft_st77922.Fill_Colors(0, 0, tft_st77922.Get_Width(), tft_st77922.Get_Height(), (uint16_t *)tft.getPointer());
  }
#else
  #include "FT6336U.h"

  TFT_eSPI tft = TFT_eSPI();

  #define I2C_SDA 16
  #define I2C_SCL 15
  #define RST_N_PIN 18
  #define INT_N_PIN 17

  FT6336U ft6336u(I2C_SDA, I2C_SCL, RST_N_PIN, INT_N_PIN);
  FT6336U_TouchPointType tp;

  void flushScreen() {}   // direct-draw displays need no flush
#endif

#ifdef FNK0104N_3P5_320x480_ST77922
  const uint8_t BarH = 30;
  #define NATIVE_W 320
#elif defined FNK0104AB_2P8_240x320_ILI9341
  const uint8_t BarH = 21;
  #define NATIVE_W 240
#elif defined FNK0104S_4P0_320x480_ST7796
  const uint8_t BarH = 30;
  #define NATIVE_W 320
#else
  const uint8_t BarH = 30;
  #define NATIVE_W 320
#endif

// ---------------- Colours (tracker palette) ----------------
const uint16_t C_RING  = 0x2945;                          // dim grey-blue, as the radar rings
const uint16_t C_MOONB = 0x2124;                          // dark side of the moon
const uint16_t C_MOONL = 0xEF5B;                          // lit side of the moon

// ---------------- Settings data (time zones and cities you can pick) ----------------
struct TzInfo   { const char* name; const char* posix; const char* api; };
struct CityInfo { const char* name; float lat, lon; uint8_t tz; };

const TzInfo TZS[] = {
  {"Canberra / Sydney / Melbourne",  "AEST-10AEDT,M10.1.0,M4.1.0/3", "Australia/Sydney"},     // 0
  {"Brisbane (Queensland)",          "AEST-10",                      "Australia/Brisbane"},   // 1
  {"Adelaide (South Australia)",     "ACST-9:30ACDT,M10.1.0,M4.1.0/3","Australia/Adelaide"},  // 2
  {"Darwin (Northern Territory)",    "ACST-9:30",                    "Australia/Darwin"},     // 3
  {"Perth (Western Australia)",      "AWST-8",                       "Australia/Perth"},      // 4
  {"New Zealand",                    "NZST-12NZDT,M9.5.0,M4.1.0/3",  "Pacific/Auckland"},     // 5
  {"Japan",                          "JST-9",                        "Asia/Tokyo"},           // 6
  {"Singapore",                      "SGT-8",                        "Asia/Singapore"},       // 7
  {"Hong Kong",                      "HKT-8",                        "Asia/Hong_Kong"},       // 8
  {"China",                          "CST-8",                        "Asia/Shanghai"},        // 9
  {"India",                          "IST-5:30",                     "Asia/Kolkata"},         // 10
  {"Dubai / Gulf",                   "GST-4",                        "Asia/Dubai"},           // 11
  {"Moscow",                         "MSK-3",                        "Europe/Moscow"},        // 12
  {"United Kingdom",                 "GMT0BST,M3.5.0/1,M10.5.0",     "Europe/London"},        // 13
  {"Central Europe",                 "CET-1CEST,M3.5.0,M10.5.0/3",   "Europe/Paris"},         // 14
  {"Eastern Europe / Greece",        "EET-2EEST,M3.5.0/3,M10.5.0/4", "Europe/Athens"},        // 15
  {"South Africa",                   "SAST-2",                       "Africa/Johannesburg"},  // 16
  {"US Eastern",                     "EST5EDT,M3.2.0,M11.1.0",       "America/New_York"},     // 17
  {"US Central",                     "CST6CDT,M3.2.0,M11.1.0",       "America/Chicago"},      // 18
  {"US Mountain",                    "MST7MDT,M3.2.0,M11.1.0",       "America/Denver"},       // 19
  {"US Pacific",                     "PST8PDT,M3.2.0,M11.1.0",       "America/Los_Angeles"},  // 20
  {"Hawaii",                         "HST10",                        "Pacific/Honolulu"},     // 21
  {"Brazil (Sao Paulo)",             "BRT3",                         "America/Sao_Paulo"},    // 22
};
const int TZ_N = sizeof(TZS) / sizeof(TZS[0]);

const CityInfo CITIES[] = {
  {"Canberra",     -35.2809f,  149.1300f,  0}, {"Sydney",       -33.8688f,  151.2093f,  0},
  {"Melbourne",    -37.8136f,  144.9631f,  0}, {"Hobart",       -42.8821f,  147.3272f,  0},
  {"Brisbane",     -27.4698f,  153.0251f,  1}, {"Adelaide",     -34.9285f,  138.6007f,  2},
  {"Darwin",       -12.4634f,  130.8456f,  3}, {"Perth",        -31.9505f,  115.8605f,  4},
  {"Auckland",     -36.8485f,  174.7633f,  5}, {"Wellington",   -41.2866f,  174.7756f,  5},
  {"Tokyo",         35.6762f,  139.6503f,  6}, {"Singapore",      1.3521f,  103.8198f,  7},
  {"Hong Kong",     22.3193f,  114.1694f,  8}, {"Shanghai",      31.2304f,  121.4737f,  9},
  {"Mumbai",        19.0760f,   72.8777f, 10}, {"Dubai",         25.2048f,   55.2708f, 11},
  {"Moscow",        55.7558f,   37.6173f, 12}, {"London",        51.5074f,   -0.1278f, 13},
  {"Paris",         48.8566f,    2.3522f, 14}, {"Berlin",        52.5200f,   13.4050f, 14},
  {"Rome",          41.9028f,   12.4964f, 14}, {"Athens",        37.9838f,   23.7275f, 15},
  {"Johannesburg", -26.2041f,   28.0473f, 16}, {"New York",      40.7128f,  -74.0060f, 17},
  {"Toronto",       43.6532f,  -79.3832f, 17}, {"Chicago",       41.8781f,  -87.6298f, 18},
  {"Denver",        39.7392f, -104.9903f, 19}, {"Los Angeles",   34.0522f, -118.2437f, 20},
  {"Honolulu",      21.3069f, -157.8583f, 21}, {"Sao Paulo",    -23.5505f,  -46.6333f, 22},
};
const int CITY_N = sizeof(CITIES) / sizeof(CITIES[0]);

Preferences prefs;
String cfgSsid, cfgPass;
int cfgTz = 0, cfgCity = 0;          // defaults: Canberra / Sydney time zone
int spkVolume = SPK_VOLUME_DEFAULT;  // speaker volume 0..100
int briPct = 100;                    // screen brightness 10..100
bool sliderDragging = false;
int sliderActive = 0;                // 0 = volume, 1 = brightness

static const char* MONTHS[12] = {"January","February","March","April","May","June","July","August","September","October","November","December"};
static const char* MON3[12]   = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
static const char* DAY3[7]    = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
static const char* PHASE_FULL[8]  = {"New Moon","Waxing Crescent","First Quarter","Waxing Gibbous","Full Moon","Waning Gibbous","Last Quarter","Waning Crescent"};
static const char* PHASE_SHORT[8] = {"New","Wax Cres","1st Qtr","Wax Gib","Full","Wan Gib","Last Qtr","Wan Cres"};

// ---------------- State ----------------
enum Page { PAGE_HOME, PAGE_CAL, PAGE_WX, PAGE_MOON, PAGE_VOICE, PAGE_SETTINGS, PAGE_PICK, PAGE_KB };
Page page = PAGE_HOME;

int W, H, topH, dateH, botY;
int boxX[4], boxW[4], boxY, boxH;
int calPage = 0, calPages = 1;
int lastMinute = -1, lastDayKey = -1;
int lastWifiLevel = -2;
uint32_t lastTouchMs = 0;
bool wasTouching = false;

struct Wx {
  bool ok = false;
  int  temp = 0, code = 0;
  int  hi[7], lo[7], dcode[7], pop[7];
} wx;
uint32_t lastWxTry = 0;

// ---------------- Touch (same mapping as the tracker example) ----------------
bool readTouch(uint16_t &tx, uint16_t &ty) {
#ifdef FNK0104N_3P5_320x480_ST77922
  if (touch_st77922.Get_Touch()) {
    tx = touch_st77922.touch.x[0];
    ty = touch_st77922.touch.y[0];
    return true;
  }
  return false;
#else
  tp = ft6336u.scan();
  if (tp.touch_count != 0) {
    tx = tp.tp[0].y;
    ty = NATIVE_W - tp.tp[0].x;
    return true;
  }
  return false;
#endif
}

// ---------------- Date / moon helpers ----------------
bool getNow(tm &t) {
  time_t n = time(nullptr);
  if (n < 1700000000) return false;   // not synced yet
  localtime_r(&n, &t);
  return true;
}

// local noon of (today + off)
bool dayTm(int off, tm &out, time_t *epoch = nullptr) {
  time_t n = time(nullptr);
  if (n < 1700000000) return false;
  localtime_r(&n, &out);
  out.tm_mday += off; out.tm_hour = 12; out.tm_min = 0; out.tm_sec = 0; out.tm_isdst = -1;
  time_t t = mktime(&out);
  localtime_r(&t, &out);
  if (epoch) *epoch = t;
  return true;
}

int dowOf(int y, int m, int d) {          // m = 1..12, 0 = Sunday
  static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y--;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}
int dim(int y, int m) {
  static const int d[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
  return d[m - 1];
}

double moonPhase(time_t t) {              // 0 = new, 0.5 = full
  double jd = t / 86400.0 + 2440587.5;
  double p = fmod((jd - 2451550.1) / 29.530588853, 1.0);
  if (p < 0) p += 1.0;
  return p;
}
int phaseIndex(double p) { return ((int)floor(p * 8.0 + 0.5)) % 8; }
int illumPct(double p) { return (int)lround((1.0 - cos(2.0 * PI * p)) / 2.0 * 100.0); }

// ---------------- Drawing helpers (tracker look) ----------------
void drawButton(int x, int w, const char* label) {          // identical to the tracker's button
  tft.fillRect(x, 0, w, BarH, TFT_DARKGREY);
  tft.drawRect(x, 0, w, BarH, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, x + w / 2, BarH / 2, 2);
  tft.setTextDatum(TL_DATUM);
}

int wifiLevel() {
  if (WiFi.status() != WL_CONNECTED) return -1;
  int r = WiFi.RSSI();
  if (r > -55) return 4;
  if (r > -65) return 3;
  if (r > -75) return 2;
  return 1;
}

void drawWifiIcon() {                      // signal bars, top-right corner
  int lvl = wifiLevel();
  lastWifiLevel = lvl;
  int x0 = W - 26;
  int yb = BarH / 2 + 7;
  tft.fillRect(x0 - 2, 0, 24, BarH, TFT_BLACK);
  for (int i = 0; i < 4; i++) {
    int h = 4 + i * 3;
    uint16_t col = (lvl > i) ? TFT_GREEN : TFT_DARKGREY;
    tft.fillRect(x0 + i * 5, yb - h, 4, h, col);
  }
  if (lvl < 0) {
    tft.drawLine(x0 - 1, yb - 14, x0 + 18, yb, TFT_RED);
    tft.drawLine(x0,     yb - 14, x0 + 19, yb, TFT_RED);
  }
}

// bar for detail pages: HOME [NEXT] title ... wifi
void drawBar(const char* title, bool nextBtn, const char* left = "HOME") {
  tft.fillRect(0, 0, W, H, TFT_BLACK);
  drawButton(0, 60, left);
  if (nextBtn) drawButton(62, 60, "NEXT");
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(title, 130, (BarH - 16) / 2, 2);
  drawWifiIcon();
  tft.drawFastHLine(0, BarH, W, TFT_DARKGREY);
}

void thickLine(int x0, int y0, int x1, int y1, int w, uint16_t col) {
  for (int i = -(w / 2); i <= w / 2; i++) {
    tft.drawLine(x0 + i, y0, x1 + i, y1, col);
    tft.drawLine(x0, y0 + i, x1, y1 + i, col);
  }
}

int tempWidth(int v, uint8_t font) {
  char b[8]; snprintf(b, sizeof(b), "%d", v);
  return tft.textWidth(b, font) + 8;
}
int txtTemp(int x, int y, int v, uint8_t font, uint16_t col) {   // number + degree ring, TL datum
  char b[8]; snprintf(b, sizeof(b), "%d", v);
  tft.setTextColor(col);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(b, x, y, font);
  int w = tft.textWidth(b, font);
  tft.drawCircle(x + w + 3, y + 3, 2, col);
  return w + 8;
}

// ---------------- Weather icons ----------------
void drawSun(int cx, int cy, int s) {
  tft.fillCircle(cx, cy, s * 38 / 100, TFT_YELLOW);
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4;
    tft.drawLine(cx + cosf(a) * s * 55 / 100, cy + sinf(a) * s * 55 / 100,
                 cx + cosf(a) * s * 85 / 100, cy + sinf(a) * s * 85 / 100, TFT_YELLOW);
  }
}
void drawCloud(int cx, int cy, int s, uint16_t c) {
  tft.fillCircle(cx - s * 45 / 100, cy + s * 10 / 100, s * 38 / 100, c);
  tft.fillCircle(cx,                cy - s * 10 / 100, s * 48 / 100, c);
  tft.fillCircle(cx + s * 50 / 100, cy + s * 12 / 100, s * 34 / 100, c);
  tft.fillRect(cx - s * 45 / 100, cy + s * 10 / 100, s * 95 / 100, s * 36 / 100, c);
}
void drawWxIcon(int code, int cx, int cy, int s) {
  const uint16_t light = TFT_LIGHTGREY, dark = TFT_DARKGREY;
  if (code == 0) { drawSun(cx, cy, s); return; }
  if (code <= 2) { drawSun(cx - s / 3, cy - s / 3, s * 70 / 100); drawCloud(cx + s / 6, cy + s / 6, s * 80 / 100, light); return; }
  if (code == 3) { drawCloud(cx, cy, s, dark); return; }
  if (code == 45 || code == 48) {
    drawCloud(cx, cy - s / 5, s * 85 / 100, light);
    for (int i = 0; i < 3; i++) tft.drawFastHLine(cx - s / 2, cy + s * 40 / 100 + i * (s / 5), s, light);
    return;
  }
  drawCloud(cx, cy - s / 5, s * 85 / 100, dark);
  int by = cy + s * 38 / 100;
  if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    for (int i = -1; i <= 1; i++) tft.drawLine(cx + i * s * 30 / 100, by, cx + i * s * 30 / 100 - s / 8, by + s * 45 / 100, TFT_CYAN);
  } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
    for (int i = -1; i <= 1; i++) tft.fillCircle(cx + i * s * 30 / 100, by + s / 5, 2, TFT_WHITE);
  } else if (code >= 95) {
    tft.drawLine(cx + s / 8, by - s / 10, cx - s / 8, by + s / 5, TFT_YELLOW);
    tft.drawLine(cx - s / 8, by + s / 5, cx + s / 6, by + s / 5, TFT_YELLOW);
    tft.drawLine(cx + s / 6, by + s / 5, cx - s / 8, by + s * 50 / 100, TFT_YELLOW);
  }
}

// ---------------- Moon ----------------
void drawMoon(int cx, int cy, int r, double p) {
  tft.fillCircle(cx, cy, r, C_MOONB);
  double k = cos(2.0 * PI * p);
  bool waxing = p < 0.5;
  bool south = CITIES[cfgCity].lat < 0;        // mirror the moon for the southern hemisphere
  for (int dy = -r; dy <= r; dy++) {
    double w = sqrt((double)r * r - (double)dy * dy);
    double xa, xb;
    if (waxing) { xa = w * k; xb = w; } else { xa = -w; xb = -w * k; }
    if (south) { double a = -xb, b = -xa; xa = a; xb = b; }
    int x0 = cx + (int)lround(xa), x1 = cx + (int)lround(xb);
    if (x1 > x0) tft.drawFastHLine(x0, cy + dy, x1 - x0 + 1, C_MOONL);
  }
}

// ---------------- Month grid ----------------
void drawMonthGrid(int x, int y, int w, int h, int year, int month, int today, const char* title) {
  int unit = h / 8, cw = w / 7;
  int font = (cw >= 17 && unit >= 17) ? 2 : 1;
  int hf = (unit >= 17) ? 2 : 1;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(title, x + w / 2, y + unit / 2, hf);
#if WEEK_STARTS_MONDAY
  static const char* WK = "MTWTFSS";
#else
  static const char* WK = "SMTWTFS";
#endif
  tft.setTextColor(TFT_DARKGREY);
  for (int c = 0; c < 7; c++) {
    char b[2] = {WK[c], 0};
    tft.drawString(b, x + c * cw + cw / 2, y + unit + unit / 2, font);
  }
  int first = dowOf(year, month, 1);
#if WEEK_STARTS_MONDAY
  first = (first + 6) % 7;
#endif
  int n = dim(year, month);
  for (int d = 1; d <= n; d++) {
    int idx = first + d - 1;
    int cx = x + (idx % 7) * cw + cw / 2;
    int cy = y + (2 + idx / 7) * unit + unit / 2;
    if (d == today) {
      tft.fillRect(cx - cw / 2 + 1, cy - unit / 2 + 1, cw - 2, unit - 2, TFT_YELLOW);
      tft.setTextColor(TFT_BLACK);
    } else {
      tft.setTextColor(TFT_WHITE);
    }
    char b[4]; snprintf(b, sizeof(b), "%d", d);
    tft.drawString(b, cx, cy, font);
  }
  tft.setTextDatum(TL_DATUM);
}

// =====================================================================
//  Home page
// =====================================================================
// Renders `s` in a built-in font into an off-screen sprite, then draws it scaled by `scale` with
// smoothing, so large text is not blocky. x is the left edge (or the centre if `centered`), yc the
// vertical centre. Returns the drawn width.
int drawScaledText(const char *s, uint8_t font, float scale, int x, int yc, bool centered, uint16_t col) {
  tft.setTextSize(1);
  int sw = tft.textWidth(s, font) + 2, sh = tft.fontHeight(font);
  int dw = (int)(sw * scale), dh = (int)(sh * scale);
  int dx0 = centered ? x - dw / 2 : x, dy0 = yc - dh / 2;
  TFT_eSprite spr = TFT_eSprite(&tft);
  spr.setColorDepth(16);
  uint8_t *src = nullptr;
  if (spr.createSprite(sw, sh)) src = (uint8_t *)malloc(sw * sh);
  if (!src) {                                       // out of memory: plain unscaled text
    spr.deleteSprite();
    tft.setTextColor(col);
    tft.setTextDatum(centered ? MC_DATUM : ML_DATUM);
    tft.drawString(s, x, yc, font);
    tft.setTextDatum(TL_DATUM);
    return sw;
  }
  spr.fillSprite(TFT_BLACK);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setTextDatum(TL_DATUM);
  spr.drawString(s, 1, 0, font);
  for (int y = 0; y < sh; y++)
    for (int xx = 0; xx < sw; xx++)
      src[y * sw + xx] = (uint8_t)((((spr.readPixel(xx, y) >> 5) & 0x3F) * 255) / 63);   // brightness
  spr.deleteSprite();
  int cr = ((col >> 11) & 31) * 255 / 31, cg = ((col >> 5) & 63) * 255 / 63, cb = (col & 31) * 255 / 31;
  for (int y = 0; y < dh; y++) {                    // bilinear scale-up
    float fy = (y + 0.5f) / scale - 0.5f;
    int y0 = (int)floorf(fy); float wy = fy - y0; int y1 = y0 + 1;
    if (y0 < 0) y0 = 0; if (y1 < 0) y1 = 0; if (y0 > sh - 1) y0 = sh - 1; if (y1 > sh - 1) y1 = sh - 1;
    for (int xx = 0; xx < dw; xx++) {
      float fx = (xx + 0.5f) / scale - 0.5f;
      int x0 = (int)floorf(fx); float wx = fx - x0; int x1 = x0 + 1;
      if (x0 < 0) x0 = 0; if (x1 < 0) x1 = 0; if (x0 > sw - 1) x0 = sw - 1; if (x1 > sw - 1) x1 = sw - 1;
      float v = src[y0 * sw + x0] * (1 - wx) * (1 - wy) + src[y0 * sw + x1] * wx * (1 - wy)
              + src[y1 * sw + x0] * (1 - wx) * wy       + src[y1 * sw + x1] * wx * wy;
      int i = (int)v; if (i > 255) i = 255;
      if (i > 12) tft.drawPixel(dx0 + xx, dy0 + y, tft.color565(cr * i / 255, cg * i / 255, cb * i / 255));
    }
  }
  free(src);
  return dw;
}

// true if the TFT_eSPI build includes `font` (draws `sample` into a scratch sprite and counts lit pixels)
bool fontAvailable(uint8_t font, const char *sample, int minLit) {
  TFT_eSprite spr = TFT_eSprite(&tft);
  spr.setColorDepth(16);
  if (!spr.createSprite(60, 70)) return false;
  spr.fillSprite(TFT_BLACK);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setTextDatum(TL_DATUM);
  spr.drawString(sample, 0, 0, font);
  int lit = 0;
  for (int y = 0; y < 70; y++) for (int x = 0; x < 60; x++) if (spr.readPixel(x, y) != 0) lit++;
  spr.deleteSprite();
  return lit >= minLit;
}

void drawGear(int cx, int cy) {                     // settings icon
  const uint16_t c = TFT_LIGHTGREY;
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4;
    thickLine(cx + cosf(a) * 6, cy + sinf(a) * 6, cx + cosf(a) * 11, cy + sinf(a) * 11, 3, c);
  }
  tft.fillCircle(cx, cy, 8, c);
  tft.fillCircle(cx, cy, 3, TFT_BLACK);
}

void drawClockZone(const tm* t) {
  tft.fillRect(0, 0, W, topH, TFT_BLACK);
  int r = (topH - 8) / 2, cx = r + 6, cy = topH / 2;
  tft.drawCircle(cx, cy, r, TFT_DARKGREY);
  tft.drawCircle(cx, cy, r - 1, TFT_DARKGREY);
  for (int i = 0; i < 12; i++) {
    float a = i * PI / 6, s = sinf(a), c = cosf(a);
    int len = (i % 3 == 0) ? r / 5 : r / 10;
    int x0 = cx + (r - 4) * s, y0 = cy - (r - 4) * c;
    int x1 = cx + (r - 4 - len) * s, y1 = cy - (r - 4 - len) * c;
    if (i % 3 == 0) thickLine(x0, y0, x1, y1, 2, TFT_WHITE);
    else tft.drawLine(x0, y0, x1, y1, TFT_LIGHTGREY);
  }
  {   // numerals 12, 3, 6, 9 just inside the major ticks
    uint8_t nf = (r >= 48) ? 2 : 1;
    int fh = tft.fontHeight(nf);
    int inner = r - 4 - r / 5 - 3;                 // radius of the inner end of the major ticks
    const char* num[4] = {"12", "3", "6", "9"};
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    for (int k = 0; k < 4; k++) {
      int tw = tft.textWidth(num[k], nf);
      int dx = 0, dy = 0;
      if (k == 0) dy = -(inner - fh / 2);            // 12
      if (k == 1) dx =  (inner - tw / 2);            // 3
      if (k == 2) dy =  (inner - fh / 2);            // 6
      if (k == 3) dx = -(inner - tw / 2);            // 9
      tft.drawString(num[k], cx + dx, cy + dy, nf);
    }
    tft.setTextDatum(TL_DATUM);
  }
  if (t) {
    float ma = t->tm_min * PI / 30.0f;
    float ha = ((t->tm_hour % 12) + t->tm_min / 60.0f) * PI / 6.0f;
    thickLine(cx, cy, cx + r * 50 / 100 * sinf(ha), cy - r * 50 / 100 * cosf(ha), 4, TFT_WHITE);
    thickLine(cx, cy, cx + r * 78 / 100 * sinf(ma), cy - r * 78 / 100 * cosf(ma), 2, TFT_YELLOW);
    tft.fillCircle(cx, cy, 3, TFT_WHITE);
  }

  // digital time: large smooth-scaled 7-segment digits (falls back to font 4 if font 7 is not built in)
  static uint8_t timeFont = 0;
  if (!timeFont) timeFont = (fontAvailable(7, "8", 40) && fontAvailable(7, ":", 6)) ? 7 : 4;
  char tb[8], ap[3];
  if (t) {
    int h12 = t->tm_hour % 12; if (h12 == 0) h12 = 12;
    snprintf(tb, sizeof(tb), "%d:%02d", h12, t->tm_min);
    strcpy(ap, t->tm_hour < 12 ? "AM" : "PM");
  } else {
    strcpy(tb, "--:--"); ap[0] = 0;
  }
  int x0 = 2 * r + 20, w = W - x0 - 30;
  tft.setTextSize(1);
  int sw1 = tft.textWidth(tb, timeFont) + 2, fh1 = tft.fontHeight(timeFont);
  const float apScale = 1.15f;
  int apW = ap[0] ? (int)((tft.textWidth(ap, 4) + 2) * apScale) + 8 : 0;
  float scale = (float)(w - apW) / sw1;
  float hs = (float)(topH - 12) / fh1;
  if (hs < scale) scale = hs;
  if (scale < 1.0f) scale = 1.0f;
  int tw = (int)(sw1 * scale), th = (int)(fh1 * scale);
  int sx = x0 + (w - tw - apW) / 2;
  drawScaledText(tb, timeFont, scale, sx, cy, false, TFT_WHITE);
  if (ap[0]) {
    int ah = (int)(tft.fontHeight(4) * apScale);
    drawScaledText(ap, 4, apScale, sx + tw + 8, cy + th / 2 - ah / 2, false, TFT_YELLOW);
  }
  tft.setTextDatum(TL_DATUM);
  drawWifiIcon();
}

void drawDateZone(const tm* t) {
  // The date sits centred between the bottom of the clock (face / time) and the top of the boxes;
  // the settings gear sits at the right-hand end of the same band.
  int yTop = topH - 3, yBot = boxY - 1;
  tft.fillRect(0, yTop, W, yBot - yTop + 1, TFT_BLACK);
  int cy = ((topH - 4) + boxY) / 2;
  drawGear(W - 24, cy);
  if (!t) return;
  char b[32];
  snprintf(b, sizeof(b), "%02d %s %04d", t->tm_mday, MONTHS[t->tm_mon], t->tm_year + 1900);
  tft.setTextSize(1);
  int sw = tft.textWidth(b, 4) + 2;
  float scale = DATE_SCALE_PCT / 100.0f;
  if (sw * scale > W - 80) scale = (float)(W - 80) / sw;   // keep clear of the gear
  drawScaledText(b, 4, scale, W / 2, cy, true, TFT_YELLOW);
}

void drawBoxFrame(int i, const char* label) {
  tft.fillRect(boxX[i], boxY, boxW[i], boxH, TFT_BLACK);
  tft.drawRect(boxX[i], boxY, boxW[i], boxH, TFT_DARKGREY);
  if (label) {
    tft.setTextColor(TFT_CYAN);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(label, boxX[i] + boxW[i] / 2, boxY + 4, 2);
    tft.setTextDatum(TL_DATUM);
  }
}

void drawBoxCal(const tm* t) {
  drawBoxFrame(0, nullptr);
  if (!t) return;
  char title[16];
  snprintf(title, sizeof(title), "%s %d", MON3[t->tm_mon], t->tm_year + 1900);
  drawMonthGrid(boxX[0] + 3, boxY + 3, boxW[0] - 6, boxH - 6, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, title);
}

void drawBoxWeather() {
  drawBoxFrame(1, "WEATHER");
  int cx = boxX[1] + boxW[1] / 2;
  tft.setTextDatum(MC_DATUM);
  if (!wx.ok) {
    tft.setTextColor(TFT_DARKGREY);
    tft.drawString("loading...", cx, boxY + boxH / 2, 2);
    tft.setTextDatum(TL_DATUM);
    return;
  }
  drawWxIcon(wx.code, cx, boxY + boxH * 38 / 100, boxW[1] * 28 / 100);
  int tw = tempWidth(wx.temp, 4);
  txtTemp(cx - tw / 2, boxY + boxH * 58 / 100, wx.temp, 4, TFT_WHITE);
  char hb[8], lb[8], sb[2] = {'/', 0};
  snprintf(hb, sizeof(hb), "%d", wx.hi[0]);
  snprintf(lb, sizeof(lb), "%d", wx.lo[0]);
  int w1 = tft.textWidth(hb, 2), w2 = tft.textWidth(sb, 2), w3 = tft.textWidth(lb, 2);
  int sx = cx - (w1 + w2 + w3) / 2, yy = boxY + boxH * 88 / 100;
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_ORANGE);    tft.drawString(hb, sx, yy, 2);
  tft.setTextColor(TFT_DARKGREY);  tft.drawString(sb, sx + w1, yy, 2);
  tft.setTextColor(TFT_CYAN);      tft.drawString(lb, sx + w1 + w2, yy, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawBoxMoon() {
  drawBoxFrame(2, "MOON");
  int cx = boxX[2] + boxW[2] / 2;
  time_t e; tm d;
  if (!dayTm(0, d, &e)) return;
  double p = moonPhase(e);
  int r = min(boxW[2] / 2 - 10, boxH * 24 / 100);
  drawMoon(cx, boxY + boxH * 42 / 100, r, p);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(PHASE_SHORT[phaseIndex(p)], cx, boxY + boxH * 74 / 100, 2);
  char b[8]; snprintf(b, sizeof(b), "%d%%", illumPct(p));
  tft.setTextColor(TFT_DARKGREY);
  tft.drawString(b, cx, boxY + boxH * 90 / 100, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawBoxVoice() {
  drawBoxFrame(3, "CLAUDE");
  int cx = boxX[3] + boxW[3] / 2, y = boxY + boxH * 28 / 100;
  tft.drawRoundRect(cx - 12, y + 8, 24, 28, 12, TFT_CYAN);       // U shape...
  tft.fillRect(cx - 14, y + 4, 28, 16, TFT_BLACK);               // ...cover its top half
  tft.fillRoundRect(cx - 7, y, 14, 28, 7, TFT_WHITE);            // capsule
  tft.drawFastVLine(cx, y + 36, 8, TFT_CYAN);
  tft.drawFastHLine(cx - 8, y + 44, 17, TFT_CYAN);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Tap to talk", cx, boxY + boxH * 90 / 100, 1);
  tft.setTextDatum(TL_DATUM);
}

void drawBottom() {
  tm t; bool ok = getNow(t);
  drawBoxCal(ok ? &t : nullptr);
  drawBoxWeather();
  drawBoxMoon();
  drawBoxVoice();
}

void drawHome() {
  page = PAGE_HOME;
  tm t; bool ok = getNow(t);
  tft.fillRect(0, 0, W, H, TFT_BLACK);
  drawClockZone(ok ? &t : nullptr);
  drawDateZone(ok ? &t : nullptr);
  drawBottom();
  flushScreen();
}

// =====================================================================
//  Detail pages
// =====================================================================
void drawCalPage() {
  page = PAGE_CAL;
  int cols = (W >= 480) ? 4 : 3;
  int per  = (W >= 480) ? 12 : 6;
  calPages = 12 / per;
  char title[28];
  if (calPages > 1) snprintf(title, sizeof(title), "Next 12 Months  %d/%d", calPage + 1, calPages);
  else              snprintf(title, sizeof(title), "Next 12 Months");
  drawBar(title, calPages > 1);
  tm n;
  if (!getNow(n)) { flushScreen(); return; }
  int rows = per / cols, cw = W / cols, ch = (H - BarH - 1) / rows;
  for (int k = 0; k < per; k++) {
    int mi = calPage * per + k;
    int mm = (n.tm_mon + mi) % 12;
    int yy = n.tm_year + 1900 + (n.tm_mon + mi) / 12;
    int cx = (k % cols) * cw, cy = BarH + 1 + (k / cols) * ch;
    tft.drawRect(cx + 1, cy + 1, cw - 2, ch - 2, TFT_DARKGREY);
    char t2[24];
    snprintf(t2, sizeof(t2), "%s %d", MONTHS[mm], yy);
    drawMonthGrid(cx + 5, cy + 4, cw - 10, ch - 8, yy, mm + 1, mi == 0 ? n.tm_mday : 0, t2);
  }
  flushScreen();
}

void rowLabel(int i, int y, int rowH, uint8_t f) {
  tm d; char b[16];
  if (!dayTm(i, d)) return;
  if (i == 0) strcpy(b, "Today"); else snprintf(b, sizeof(b), "%s %d", DAY3[d.tm_wday], d.tm_mday);
  tft.setTextColor(i == 0 ? TFT_YELLOW : TFT_WHITE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(b, 6, y + rowH / 2, f);
  tft.setTextDatum(TL_DATUM);
}

void drawWxPage() {
  page = PAGE_WX;
  drawBar("7-Day Weather", false);
  if (!wx.ok) {
    tft.setTextColor(TFT_DARKGREY); tft.setTextDatum(MC_DATUM);
    tft.drawString("No weather data yet", W / 2, H / 2, 4);
    tft.setTextDatum(TL_DATUM);
    flushScreen(); return;
  }
  int rowH = (H - BarH - 2) / 7;
  uint8_t rf = (rowH >= 36) ? 4 : 2;
  int fh = tft.fontHeight(rf);
  for (int i = 0; i < 7; i++) {
    int y = BarH + 2 + i * rowH;
    rowLabel(i, y, rowH, rf);
    drawWxIcon(wx.dcode[i], W * 30 / 100, y + rowH / 2, rowH * 40 / 100);
    txtTemp(W * 44 / 100, y + (rowH - fh) / 2, wx.hi[i], rf, TFT_ORANGE);
    txtTemp(W * 61 / 100, y + (rowH - fh) / 2, wx.lo[i], rf, TFT_CYAN);
    char b[8]; snprintf(b, sizeof(b), "%d%%", wx.pop[i]);
    tft.setTextColor(TFT_CYAN); tft.setTextDatum(MR_DATUM);
    tft.drawString(b, W - 8, y + rowH / 2, rf);
    tft.setTextDatum(TL_DATUM);
    tft.drawFastHLine(0, y + rowH - 1, W, 0x2104);
  }
  flushScreen();
}

void drawMoonPage() {
  page = PAGE_MOON;
  drawBar("Moon Phases", false);
  int rowH = (H - BarH - 2) / 7;
  uint8_t rf = (rowH >= 36) ? 4 : 2;
  int r = rowH * 40 / 100;
  for (int i = 0; i < 7; i++) {
    int y = BarH + 2 + i * rowH;
    rowLabel(i, y, rowH, rf);
    tm d; time_t e;
    if (!dayTm(i, d, &e)) continue;
    double p = moonPhase(e);
    int mx = W * 30 / 100;
    drawMoon(mx, y + rowH / 2, r, p);
    tft.setTextColor(TFT_WHITE); tft.setTextDatum(ML_DATUM);
    tft.drawString(PHASE_FULL[phaseIndex(p)], mx + r + 12, y + rowH / 2, rf);
    char b[8]; snprintf(b, sizeof(b), "%d%%", illumPct(p));
    tft.setTextColor(TFT_DARKGREY); tft.setTextDatum(MR_DATUM);
    tft.drawString(b, W - 8, y + rowH / 2, rf);
    tft.setTextDatum(TL_DATUM);
    tft.drawFastHLine(0, y + rowH - 1, W, 0x2104);
  }
  flushScreen();
}

// =====================================================================
//  Weather (Open-Meteo, no API key)
// =====================================================================
bool fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure c; c.setInsecure();
  HTTPClient http; http.setTimeout(15000);
  String tzName = TZS[cfgTz].api; tzName.replace("/", "%2F");
  String url = String("https://api.open-meteo.com/v1/forecast?latitude=") + String(CITIES[cfgCity].lat, 4) +
    "&longitude=" + String(CITIES[cfgCity].lon, 4) +
    "&current=temperature_2m,weather_code"
    "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max"
    "&timezone=" + tzName + "&forecast_days=7";
  if (!http.begin(c, url)) return false;
  int code = http.GET();
  if (code != 200) { Serial.printf("weather HTTP %d (%s)\n", code, HTTPClient::errorToString(code).c_str());
    if (code < 0) {
      char e[100] = ""; c.lastError(e, sizeof(e));
      IPAddress ip; int r = WiFi.hostByName("api.open-meteo.com", ip);
      Serial.printf("  TLS: %s | DNS %d -> %s | free heap %u\n", e, r, ip.toString().c_str(), (unsigned)ESP.getFreeHeap());
    } http.end(); return false; }
  String body = http.getString();
  http.end();
  JsonDocument d;
  if (deserializeJson(d, body)) { Serial.println("weather JSON error"); return false; }
  wx.temp = (int)lroundf(d["current"]["temperature_2m"] | 0.0f);
  wx.code = d["current"]["weather_code"] | 0;
  JsonObject dl = d["daily"];
  for (int i = 0; i < 7; i++) {
    wx.hi[i]    = (int)lroundf(dl["temperature_2m_max"][i] | 0.0f);
    wx.lo[i]    = (int)lroundf(dl["temperature_2m_min"][i] | 0.0f);
    wx.dcode[i] = dl["weather_code"][i] | 0;
    wx.pop[i]   = dl["precipitation_probability_max"][i] | 0;
  }
  wx.ok = true;
  return true;
}

// =====================================================================
//  Voice: mic -> OpenAI speech-to-text -> Claude -> OpenAI text-to-speech -> speaker
// =====================================================================
String vStatus = "Tap the button to talk", vUser = "", vReply = "";
int voiceBtnY = 0;

String asciiFy(const String &s) {          // display fonts are ASCII only
  String o; int n = s.length();
  for (int i = 0; i < n;) {
    uint8_t c = s[i];
    if (c < 0x80) { o += (char)c; i++; continue; }
    int len = (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : 2;
    if (c == 0xE2 && i + 2 < n && (uint8_t)s[i + 1] == 0x80) {
      uint8_t b = s[i + 2];
      if (b == 0x98 || b == 0x99) o += '\'';
      else if (b == 0x9C || b == 0x9D) o += '"';
      else if (b == 0x93 || b == 0x94) o += '-';
      else if (b == 0xA6) o += "...";
    }
    i += len;
  }
  return o;
}

void drawWrapped(const String &s, int x, int y, int w, int maxLines, uint8_t font, uint16_t col) {
  tft.setTextColor(col); tft.setTextDatum(TL_DATUM);
  int lh = tft.fontHeight(font) + 2, line = 0, i = 0, n = s.length();
  String cur;
  while (i < n && line < maxLines) {
    int j = s.indexOf(' ', i); if (j < 0) j = n;
    String word = s.substring(i, j);
    String trial = cur.length() ? cur + " " + word : word;
    if (tft.textWidth(trial, font) <= w) cur = trial;
    else {
      if (cur.length()) { tft.drawString(cur, x, y + line * lh, font); line++; }
      cur = word;
    }
    i = j + 1;
  }
  if (line < maxLines && cur.length()) tft.drawString(cur, x, y + line * lh, font);
}

void drawVoicePage() {
  page = PAGE_VOICE;
  drawBar("Ask Claude", false);
  int lh = tft.fontHeight(2) + 2;
  voiceBtnY = H - 44;
  int y = BarH + 6;
  drawWrapped(vStatus, 6, y, W - 12, 2, 2, TFT_GREEN);
  y += 2 * lh + 2;
  if (vUser.length()) drawWrapped("You: " + vUser, 6, y, W - 12, 2, 2, TFT_LIGHTGREY);
  y += 2 * lh + 6;
  int lines = (voiceBtnY - y - 4) / lh;
  if (vReply.length() && lines > 0) drawWrapped(vReply, 6, y, W - 12, lines, 2, TFT_WHITE);
  tft.fillRect(6, voiceBtnY, W - 12, 38, TFT_DARKGREY);
  tft.drawRect(6, voiceBtnY, W - 12, 38, TFT_WHITE);
  tft.setTextColor(TFT_WHITE); tft.setTextDatum(MC_DATUM);
  tft.drawString("TAP TO TALK", W / 2, voiceBtnY + 19, 4);
  tft.setTextDatum(TL_DATUM);
  flushScreen();
}

#if ENABLE_VOICE
#include "driver/i2s.h"

bool voiceHwReady = false;
const int MIC_RATE = 16000;            // codec runs at 16 kHz both ways (MCLK = 256 x 16 kHz)
int16_t *recBuf = nullptr;
size_t recLen = 0;
const size_t REC_MAX = MIC_RATE * 8;     // 8 seconds max

struct Msg { bool user; String text; };  // short rolling conversation memory
Msg hist[8];
int histN = 0;
void pushHist(bool user, const String &t) {
  if (histN >= 8) { for (int i = 0; i < histN - 2; i++) hist[i] = hist[i + 2]; histN -= 2; }
  hist[histN].user = user; hist[histN].text = t; histN++;
}

bool es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
uint8_t es8311Read(uint8_t reg) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ES8311_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ES8311 set-up for 16 kHz, 16-bit I2S, slave mode, analog mic on MIC1P/MIC1N, speaker via OUTP/OUTN.
// (Register values follow Espressif's reference driver. MCLK must already be running.)
bool es8311Init() {
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) return false;           // codec does not answer
  bool ok = true;
  ok &= es8311Write(0x00, 0x1F); delay(20);                // reset
  ok &= es8311Write(0x00, 0x00);
  ok &= es8311Write(0x44, 0x08);
  ok &= es8311Write(0x01, 0x30);
  ok &= es8311Write(0x02, 0x00);
  ok &= es8311Write(0x03, 0x10);
  ok &= es8311Write(0x16, 0x24);
  ok &= es8311Write(0x04, 0x10);
  ok &= es8311Write(0x05, 0x00);
  ok &= es8311Write(0x0B, 0x00);                           // analog bias / power registers
  ok &= es8311Write(0x0C, 0x00);
  ok &= es8311Write(0x10, 0x1F);
  ok &= es8311Write(0x11, 0x7F);
  ok &= es8311Write(0x00, 0x80);                           // power on, slave mode
  ok &= es8311Write(0x01, 0x3F);                           // all clocks on, MCLK from the pin
  ok &= es8311Write(0x02, 0x00);                           // dividers for MCLK 4.096 MHz / 16 kHz
  ok &= es8311Write(0x03, 0x10);
  ok &= es8311Write(0x04, 0x20);
  ok &= es8311Write(0x05, 0x00);
  ok &= es8311Write(0x06, 0x03);
  ok &= es8311Write(0x07, 0x00);
  ok &= es8311Write(0x08, 0xFF);
  ok &= es8311Write(0x09, 0x0C);                           // I2S, 16 bit (data in)
  ok &= es8311Write(0x0A, 0x0C);                           // I2S, 16 bit (data out)
  ok &= es8311Write(0x0D, 0x01);                           // power up analog circuitry
  ok &= es8311Write(0x0E, 0x02);                           // enable PGA and ADC modulator
  ok &= es8311Write(0x12, 0x00);                           // power up DAC
  ok &= es8311Write(0x13, 0x10);                           // enable output drive
  ok &= es8311Write(0x14, 0x10 | (MIC_PGA_GAIN & 0x0F));   // analog mic (MIC1P/N) + PGA gain
  ok &= es8311Write(0x16, MIC_ADC_SCALE & 7);             // ADC digital gain scale
  ok &= es8311Write(0x17, 0xBF);                           // ADC volume
  ok &= es8311Write(0x1B, 0x0A);
  ok &= es8311Write(0x1C, 0x6A);                           // ADC equaliser bypass, DC offset cancel
  ok &= es8311Write(0x31, 0x00);                           // DAC unmute
  ok &= es8311Write(0x32, 0xBF);                           // DAC volume
  ok &= es8311Write(0x37, 0x08);                           // bypass DAC equaliser
  ok &= es8311Write(0x44, 0x08);
  Serial.printf("ES8311 chip id %02X %02X (expect 83 11)\n", es8311Read(0xFD), es8311Read(0xFE));
  Serial.printf("ES8311 regs: 00=%02X 01=%02X 0D=%02X 0E=%02X 12=%02X 13=%02X 14=%02X 16=%02X 17=%02X 1C=%02X\n",
                es8311Read(0x00), es8311Read(0x01), es8311Read(0x0D), es8311Read(0x0E), es8311Read(0x12),
                es8311Read(0x13), es8311Read(0x14), es8311Read(0x16), es8311Read(0x17), es8311Read(0x1C));
  return ok;
}

String voiceHwErr;
bool voiceHwInit() {
  static bool i2sUp = false;
  if (voiceHwReady) return true;
  voiceHwErr = "";
  if (!psramFound()) { voiceHwErr = "Enable PSRAM (OPI) for voice"; return false; }
  pinMode(AUDIO_EN_PIN, OUTPUT);
  digitalWrite(AUDIO_EN_PIN, AUDIO_EN_ACTIVE ? LOW : HIGH);   // speaker amp off until we speak

  if (!i2sUp) {                                            // I2S master: gives the codec MCLK / BCLK / WS
    i2s_config_t c; memset(&c, 0, sizeof(c));
    c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    c.sample_rate = MIC_RATE;
    c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    c.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    c.dma_buf_count = 8; c.dma_buf_len = 256;
    c.tx_desc_auto_clear = true;
    if (i2s_driver_install(I2S_NUM_0, &c, 0, NULL) != ESP_OK) { voiceHwErr = "I2S driver failed"; return false; }
    i2s_pin_config_t p; memset(&p, 0, sizeof(p));
    p.mck_io_num = ES_MCLK; p.bck_io_num = ES_BCLK; p.ws_io_num = ES_WS;
    p.data_out_num = ES_DOUT; p.data_in_num = ES_DIN;
    if (i2s_set_pin(I2S_NUM_0, &p) != ESP_OK) { voiceHwErr = "I2S pins failed"; return false; }
    i2sUp = true;
  }
  Wire.begin(ES_I2C_SDA, ES_I2C_SCL);
  if (!es8311Init()) { voiceHwErr = "Audio chip (ES8311) not answering"; return false; }
  voiceHwReady = true;
  return true;
}

// short tone through the speaker: a "start talking" cue, and a quick check that the speaker works
void beep(int freq = 880, int ms = 160) {
  int frames = MIC_RATE * ms / 1000;
  static int16_t buf[512];
  digitalWrite(AUDIO_EN_PIN, AUDIO_EN_ACTIVE ? HIGH : LOW);   // amp on
  delay(20);
  int done = 0;
  while (done < frames) {
    int n = min(256, frames - done);
    for (int i = 0; i < n; i++) {
      int16_t v = (int16_t)(sinf(2.0f * PI * freq * (done + i) / MIC_RATE) * 12000.0f * spkVolume / 100.0f);
      buf[2 * i] = v; buf[2 * i + 1] = v;
    }
    size_t bw;
    i2s_write(I2S_NUM_0, buf, n * 4, &bw, portMAX_DELAY);
    done += n;
  }
  delay(60);
  i2s_zero_dma_buffer(I2S_NUM_0);
  digitalWrite(AUDIO_EN_PIN, AUDIO_EN_ACTIVE ? LOW : HIGH);   // amp off
}

float micNoise = 0, micThr = 0, micPeak = 0, micPeakL = 0, micPeakR = 0;     // last recording stats, for diagnostics

// record until ~1 s of silence after speech (or 8 s). Returns true if speech captured.
bool recordUtterance() {
  if (!recBuf) recBuf = (int16_t *)ps_malloc(REC_MAX * 2);
  if (!recBuf) return false;
  recLen = 0;
  static int16_t raw[1024];                       // 512 stereo frames; the mic is on the left slot
  uint32_t t0 = millis(), lastVoice = 0;
  bool spoke = false;
  float noise = 0, thr = 60;
  micNoise = 0; micThr = 0; micPeak = 0; micPeakL = 0; micPeakR = 0;
  bool firstRead = true;
  while (recLen + 512 <= REC_MAX) {
    size_t br = 0;
    i2s_read(I2S_NUM_0, raw, sizeof(raw), &br, 200 / portTICK_PERIOD_MS);
    int n = br / 4;
    uint32_t now = millis();
    if (n <= 0) { if (now - t0 > 3000) break; continue; }    // codec delivering nothing: give up
    if (firstRead) { firstRead = false; Serial.printf("raw: %d %d %d %d %d %d\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]); }
    long accL = 0, accR = 0;
    for (int i = 0; i < n; i++) { accL += abs(raw[2 * i]); accR += abs(raw[2 * i + 1]); }
    int ch = (accL >= accR) ? 0 : 1;                          // use whichever slot carries the audio
    int16_t *dst = recBuf + recLen;
    for (int i = 0; i < n; i++) dst[i] = raw[2 * i + ch];
    float e = (float)(ch ? accR : accL) / n;
    if ((float)accL / n > micPeakL) micPeakL = (float)accL / n;
    if ((float)accR / n > micPeakR) micPeakR = (float)accR / n;
    if (now - t0 < 150) continue;                             // throw away stale audio
    if (now - t0 < 450) { if (e > noise) noise = e; thr = max(noise * 3.0f, 40.0f); micNoise = noise; micThr = thr; continue; }  // calibrate
    recLen += n;
    if (e > micPeak) micPeak = e;
    if (e > thr) { spoke = true; lastVoice = now; }
    if (spoke && now - lastVoice > 1000) break;
    if (!spoke && now - t0 > 5000) break;
  }
  Serial.printf("mic: noise %.0f  threshold %.0f  peak %.0f (L %.0f R %.0f)  samples %u\n", micNoise, micThr, micPeak, micPeakL, micPeakR, (unsigned)recLen);
  return spoke && recLen > (size_t)(MIC_RATE / 4);
}

void wavHeader(uint8_t *h, uint32_t dataBytes, uint32_t rate) {
  uint32_t fileLen = dataBytes + 36, byteRate = rate * 2;
  uint16_t one = 1, bits = 16, align = 2; uint32_t fmtLen = 16;
  memcpy(h, "RIFF", 4); memcpy(h + 4, &fileLen, 4); memcpy(h + 8, "WAVEfmt ", 8);
  memcpy(h + 16, &fmtLen, 4); memcpy(h + 20, &one, 2); memcpy(h + 22, &one, 2);
  memcpy(h + 24, &rate, 4); memcpy(h + 28, &byteRate, 4); memcpy(h + 32, &align, 2);
  memcpy(h + 34, &bits, 2); memcpy(h + 36, "data", 4); memcpy(h + 40, &dataBytes, 4);
}

String apiErrorText(const String &body) {      // pull the "error.message" text out of an API error reply
  JsonDocument d;
  if (deserializeJson(d, body)) return "";
  const char *m = d["error"]["message"] | "";
  String o(m);
  if (o.length() > 90) o = o.substring(0, 90);
  return o;
}

void normalizeRecording() {                     // bring quiet recordings up to a good level for speech-to-text
  int32_t pk = 1;
  for (size_t i = 0; i < recLen; i++) { int32_t a = abs((int32_t)recBuf[i]); if (a > pk) pk = a; }
  float g = 20000.0f / pk;
  if (g > 40.0f) g = 40.0f;
  if (g <= 1.2f) return;
  for (size_t i = 0; i < recLen; i++) {
    int32_t v = (int32_t)(recBuf[i] * g);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    recBuf[i] = (int16_t)v;
  }
  Serial.printf("mic: sample peak %d, boosted x%.1f\n", (int)pk, g);
}

String sttTranscribe(String &err) {
  normalizeRecording();
  const char *B = "----cdsh7Xk2";
  String head = String("--") + B + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-1\r\n"
              + "--" + B + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\nen\r\n"
              + "--" + B + "\r\nContent-Disposition: form-data; name=\"response_format\"\r\n\r\ntext\r\n"
              + "--" + B + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"a.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
  String tail = String("\r\n--") + B + "--\r\n";
  uint32_t dataBytes = recLen * 2;
  size_t total = head.length() + 44 + dataBytes + tail.length();
  uint8_t *body = (uint8_t *)ps_malloc(total);
  if (!body) { err = "Out of memory"; return ""; }
  size_t o = 0;
  memcpy(body, head.c_str(), head.length()); o += head.length();
  wavHeader(body + o, dataBytes, MIC_RATE); o += 44;
  memcpy(body + o, recBuf, dataBytes); o += dataBytes;
  memcpy(body + o, tail.c_str(), tail.length());

  WiFiClientSecure c; c.setInsecure();
  HTTPClient http; http.setTimeout(30000);
  http.begin(c, "https://api.openai.com/v1/audio/transcriptions");
  http.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
  http.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
  int code = http.POST(body, total);
  free(body);
  String out;
  if (code == 200) { out = http.getString(); out.trim(); }
  else {
    String m = apiErrorText(http.getString());
    err = "Speech-to-text error " + String(code);
    if (m.length()) err += ": " + m;
    Serial.println(err);
  }
  http.end();
  return out;
}

String askClaude(const String &q, String &err) {
  pushHist(true, q);
  JsonDocument req;
  req["model"] = CLAUDE_MODEL;
  req["max_tokens"] = 300;
  String sys = String("You are Claude, talking through a small desk display and speaker in ") + CITIES[cfgCity].name +
               ". Reply in one to three short plain sentences. Use plain ASCII text only: no markdown, lists, emoji or special symbols.";
  req["system"] = sys;
  JsonArray m = req["messages"].to<JsonArray>();
  for (int i = 0; i < histN; i++) {
    JsonObject o = m.add<JsonObject>();
    o["role"] = hist[i].user ? "user" : "assistant";
    o["content"] = hist[i].text;
  }
  String body; serializeJson(req, body);
  WiFiClientSecure c; c.setInsecure();
  HTTPClient http; http.setTimeout(30000);
  http.begin(c, "https://api.anthropic.com/v1/messages");
  http.addHeader("x-api-key", ANTHROPIC_API_KEY);
  http.addHeader("anthropic-version", "2023-06-01");
  http.addHeader("content-type", "application/json");
  int code = http.POST(body);
  String resp = http.getString();
  http.end();
  JsonDocument d;
  DeserializationError de = deserializeJson(d, resp);
  if (code != 200 || de) {
    const char *msg = d["error"]["message"] | "";
    err = String("Claude error ") + code + (msg[0] ? String(": ") + msg : String(""));
    histN--;                                  // drop the unanswered question
    return "";
  }
  const char *tx = d["content"][0]["text"] | "";
  String out(tx);
  out.trim();
  pushHist(false, out);
  return out;
}

class PcmSink : public Stream {                // streamed 24 kHz mono PCM -> 16 kHz stereo for the codec
  bool haveLo = false; uint8_t lo = 0;
  int ph = 0; int16_t hold = 0;
  int16_t out[512]; int n = 0;
  void emit(int16_t v) { out[n++] = v; out[n++] = v; if (n >= 512) flushOut(); }
  void feed(int16_t s) {                       // 3 input samples -> 2 output samples
    s = (int16_t)(((int32_t)s * spkVolume) / 100);
    if (ph == 0) { emit(s); ph = 1; }
    else if (ph == 1) { hold = s; ph = 2; }
    else { emit((int16_t)(((int32_t)hold + s) / 2)); ph = 0; }
  }
public:
  void flushOut() { if (n) { size_t bw; i2s_write(I2S_NUM_0, out, n * 2, &bw, portMAX_DELAY); n = 0; } }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t *buf, size_t len) override {
    for (size_t i = 0; i < len; i++) {
      if (haveLo) { feed((int16_t)((uint16_t)lo | ((uint16_t)buf[i] << 8))); haveLo = false; }
      else { lo = buf[i]; haveLo = true; }
    }
    return len;
  }
};

bool speak(const String &text, String &err) {
  JsonDocument req;
  req["model"] = "tts-1";
  req["input"] = text;
  req["voice"] = "alloy";
  req["response_format"] = "pcm";            // raw 24 kHz, 16-bit, mono
  String body; serializeJson(req, body);
  WiFiClientSecure c; c.setInsecure();
  HTTPClient http; http.setTimeout(30000);
  http.begin(c, "https://api.openai.com/v1/audio/speech");
  http.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code != 200) {
    String m = apiErrorText(http.getString());
    err = "Text-to-speech error " + String(code);
    if (m.length()) err += ": " + m;
    Serial.println(err);
    http.end(); return false;
  }
  digitalWrite(AUDIO_EN_PIN, AUDIO_EN_ACTIVE ? HIGH : LOW);   // speaker amp on
  Serial.printf("speaking: amp enable pin set to %d\n", AUDIO_EN_ACTIVE ? HIGH : LOW);
  delay(20);
  PcmSink sink;
  http.writeToStream(&sink);
  sink.flushOut();
  http.end();
  delay(150);
  i2s_zero_dma_buffer(I2S_NUM_0);
  digitalWrite(AUDIO_EN_PIN, AUDIO_EN_ACTIVE ? LOW : HIGH);   // amp off again
  return true;
}
#endif  // ENABLE_VOICE

void voiceTurn() {
#if ENABLE_VOICE
  vUser = ""; vReply = "";
  if (WiFi.status() != WL_CONNECTED) { vStatus = "No WiFi"; drawVoicePage(); return; }
  vStatus = "Listening..."; drawVoicePage();
  if (!voiceHwInit()) { vStatus = voiceHwErr.length() ? voiceHwErr : String("Audio init failed"); drawVoicePage(); return; }
  beep();                                                    // "start talking" cue
  if (!recordUtterance()) { vStatus = (micThr == 0) ? String("No data from microphone - check wiring/pins")
                                  : String("No speech heard (L ") + (int)micPeakL + " R " + (int)micPeakR + ", needs " + (int)micThr + ")";
    drawVoicePage(); return; }
  String err;
  vStatus = "Transcribing..."; drawVoicePage();
  String q = sttTranscribe(err);
  if (!q.length()) { vStatus = err.length() ? err : "Couldn't understand - tap to retry"; drawVoicePage(); return; }
  vUser = asciiFy(q);
  vStatus = "Thinking..."; drawVoicePage();
  String a = askClaude(q, err);
  if (!a.length()) { vStatus = err.length() ? err : "No reply"; drawVoicePage(); return; }
  vReply = asciiFy(a);
  vStatus = "Speaking..."; drawVoicePage();
  if (!speak(a, err)) vStatus = err; else vStatus = "Tap to talk again";
  drawVoicePage();
#else
  vStatus = "Voice disabled"; drawVoicePage();
#endif
}

// =====================================================================
//  Settings: WiFi (scan + password), time zone, city
// =====================================================================
void loadConfig() {
  prefs.begin("dash", false);
  cfgSsid = prefs.getString("ssid", WIFI_SSID);      // first boot: use the secrets.h values
  cfgPass = prefs.getString("pass", WIFI_PASS);
  cfgTz   = prefs.getInt("tz", 0);
  cfgCity = prefs.getInt("city", 0);
  spkVolume = prefs.getInt("vol", SPK_VOLUME_DEFAULT);
  briPct = prefs.getInt("bri", 100);
  prefs.end();
  if (spkVolume < 0 || spkVolume > 100) spkVolume = SPK_VOLUME_DEFAULT;
  if (briPct < 10 || briPct > 100) briPct = 100;
  if (cfgTz < 0 || cfgTz >= TZ_N) cfgTz = 0;
  if (cfgCity < 0 || cfgCity >= CITY_N) cfgCity = 0;
}
void saveWifi() {
  prefs.begin("dash", false);
  prefs.putString("ssid", cfgSsid);
  prefs.putString("pass", cfgPass);
  prefs.end();
}
void saveBrightness() {
  prefs.begin("dash", false);
  prefs.putInt("bri", briPct);
  prefs.end();
}

// backlight brightness: PWM on the backlight pin (squared so the slider feels even to the eye)
void applyBrightness() {
  static bool attached = false;
  int duty = 255 * briPct * briPct / 10000;
  if (duty < 3) duty = 3;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!attached) { ledcAttach(BL_PIN, 12000, 8); attached = true; }
  ledcWrite(BL_PIN, duty);
#else
  if (!attached) { ledcSetup(7, 12000, 8); ledcAttachPin(BL_PIN, 7); attached = true; }
  ledcWrite(7, duty);
#endif
}

void saveVolume() {
  prefs.begin("dash", false);
  prefs.putInt("vol", spkVolume);
  prefs.end();
}
void saveLocation() {
  prefs.begin("dash", false);
  prefs.putInt("tz", cfgTz);
  prefs.putInt("city", cfgCity);
  prefs.end();
}
void applyTz() {
  setenv("TZ", TZS[cfgTz].posix, 1);
  tzset();
  lastMinute = -1; lastDayKey = -1;
  wx.ok = false; lastWxTry = 0;                      // refetch weather for the new place / zone
}

void drawBtn(int x, int y, int w, int h, const char *label, uint16_t fill = TFT_DARKGREY) {
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, x + w / 2, y + h / 2, 2);
  tft.setTextDatum(TL_DATUM);
}

void showMessage(const char *l1, const char *l2 = nullptr) {
  tft.fillRect(0, 0, W, H, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  uint8_t f = (tft.textWidth(l1, 4) <= W - 10) ? 4 : 2;
  tft.drawString(l1, W / 2, H / 2 - (l2 ? 16 : 0), f);
  if (l2) {
    tft.setTextColor(TFT_YELLOW);
    tft.drawString(l2, W / 2, H / 2 + 20, 2);
  }
  tft.setTextDatum(TL_DATUM);
  flushScreen();
}

// ---------- WiFi scan ----------
struct WifiNet { char ssid[33]; int rssi; bool enc; };
WifiNet nets[12];
int netN = 0;
int lastScanCode = 0;                                // what WiFi.scanNetworks() returned (negative = failed)

void scanNetworks() {
  netN = 0;
  bool wasConnected = (WiFi.status() == WL_CONNECTED);
  WiFi.mode(WIFI_STA);
  if (!wasConnected) { WiFi.disconnect(); delay(200); }   // stop any reconnect attempts: they make scans fail
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) { delay(500); n = WiFi.scanNetworks(false, true); }   // retry once
  lastScanCode = n;
  Serial.printf("WiFi scan returned %d\n", n);
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (!s.length() || s.length() > 32) continue;
    int rssi = WiFi.RSSI(i);
    bool enc = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    int k = -1;
    for (int j = 0; j < netN; j++) if (s == nets[j].ssid) { k = j; break; }
    if (k >= 0) { if (rssi > nets[k].rssi) { nets[k].rssi = rssi; nets[k].enc = enc; } continue; }
    if (netN < 12) k = netN++;
    else {
      int w = 0;
      for (int j = 1; j < netN; j++) if (nets[j].rssi < nets[w].rssi) w = j;
      if (rssi <= nets[w].rssi) continue;
      k = w;
    }
    strncpy(nets[k].ssid, s.c_str(), 32); nets[k].ssid[32] = 0;
    nets[k].rssi = rssi; nets[k].enc = enc;
  }
  WiFi.scanDelete();
  for (int a = 1; a < netN; a++) {                   // strongest first
    WifiNet t = nets[a]; int b = a - 1;
    while (b >= 0 && nets[b].rssi < t.rssi) { nets[b + 1] = nets[b]; b--; }
    nets[b + 1] = t;
  }
}

// ---------- list picker (time zones, cities, WiFi networks) ----------
PickKind pickKind = PK_TZ;
int pickPage = 0;

int pickCount() { return pickKind == PK_TZ ? TZ_N : pickKind == PK_CITY ? CITY_N : netN; }
int pickRowH()  { return H >= 300 ? 38 : 30; }
int pickRows()  { return (H - BarH - 44) / pickRowH(); }
int pickPages() { int c = pickCount(), r = pickRows(); return c <= 0 ? 1 : (c + r - 1) / r; }

void drawPicker() {
  page = PAGE_PICK;
  const char *title = pickKind == PK_TZ ? "Time zone" : pickKind == PK_CITY ? "City" : "WiFi networks";
  drawBar(title, false, "BACK");
  int rowH = pickRowH(), rows = pickRows(), cnt = pickCount();
  int sel = pickKind == PK_TZ ? cfgTz : pickKind == PK_CITY ? cfgCity : -1;
  for (int k = 0; k < rows; k++) {
    int i = pickPage * rows + k;
    if (i >= cnt) break;
    int y = BarH + 4 + k * rowH;
    bool isSel = (i == sel) || (pickKind == PK_WIFI && WiFi.status() == WL_CONNECTED && cfgSsid == nets[i].ssid);
    tft.fillRect(4, y, W - 8, rowH - 3, isSel ? 0x0320 : TFT_BLACK);
    tft.drawRect(4, y, W - 8, rowH - 3, TFT_DARKGREY);
    const char *lb = pickKind == PK_TZ ? TZS[i].name : pickKind == PK_CITY ? CITIES[i].name : nets[i].ssid;
    uint8_t f = (rowH >= 38 && tft.textWidth(lb, 4) <= W - 100) ? 4 : 2;
    tft.setTextColor(isSel ? TFT_YELLOW : TFT_WHITE);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(lb, 12, y + (rowH - 3) / 2, f);
    tft.setTextDatum(TL_DATUM);
    if (pickKind == PK_WIFI) {
      int lvl = nets[i].rssi > -55 ? 4 : nets[i].rssi > -65 ? 3 : nets[i].rssi > -75 ? 2 : 1;
      int yb = y + (rowH - 3) / 2 + 8;
      for (int b = 0; b < 4; b++) {
        int h = 4 + b * 3;
        tft.fillRect(W - 44 + b * 6, yb - h, 4, h, lvl > b ? TFT_GREEN : TFT_DARKGREY);
      }
      if (nets[i].enc) {                             // padlock
        int yc = y + (rowH - 3) / 2;
        tft.fillRect(W - 74, yc - 1, 10, 8, TFT_YELLOW);
        tft.drawRect(W - 72, yc - 7, 6, 7, TFT_YELLOW);
      }
    }
  }
  if (cnt == 0) {
    tft.setTextColor(TFT_DARKGREY); tft.setTextDatum(MC_DATUM);
    tft.drawString("No networks found", W / 2, H / 2 - 12, 4);
    if (pickKind == PK_WIFI) {
      char sc[40]; snprintf(sc, sizeof(sc), "Scan returned %d (tap RESCAN)", lastScanCode);
      tft.setTextColor(TFT_LIGHTGREY); tft.drawString(sc, W / 2, H / 2 + 18, 2);
    }
    tft.setTextDatum(TL_DATUM);
  }
  int by = H - 38;
  drawBtn(6, by, 90, 34, "PREV");
  drawBtn(100, by, 90, 34, "NEXT");
  if (pickKind == PK_WIFI) drawBtn(W - 96, by, 90, 34, "RESCAN");
  char pg[16]; snprintf(pg, sizeof(pg), "%d/%d", pickPage + 1, pickPages());
  tft.setTextColor(TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM);
  tft.drawString(pg, (190 + (pickKind == PK_WIFI ? W - 96 : W)) / 2, by + 17, 2);
  tft.setTextDatum(TL_DATUM);
  flushScreen();
}

void openPicker(PickKind k) {
  pickKind = k;
  int sel = (k == PK_TZ) ? cfgTz : (k == PK_CITY) ? cfgCity : 0;
  pickPage = sel / pickRows();
  drawPicker();
}

void openWifiList() {
  showMessage("Scanning for WiFi...");
  scanNetworks();
  pickKind = PK_WIFI;
  pickPage = 0;
  drawPicker();
}

// ---------- on-screen keyboard ----------
String kbText, kbTarget;
int kbMode = 0;                                      // 0 abc, 1 ABC, 2 symbols 1, 3 symbols 2

const char *kbRow(int mode, int row) {               // \001 = shift, \002 = delete
  static const char *L[3] = {"qwertyuiop", "asdfghjkl", "\001zxcvbnm\002"};
  static const char *U[3] = {"QWERTYUIOP", "ASDFGHJKL", "\001ZXCVBNM\002"};
  static const char *A[3] = {"1234567890", "@#$%&*-_+=", "\001!?.,:;'\002"};
  static const char *B[3] = {"1234567890", "~`^|\\/<>[]", "\001{}()\"=\002"};
  return mode == 0 ? L[row] : mode == 1 ? U[row] : mode == 2 ? A[row] : B[row];
}

bool kbKeyRect(int row, int idx, int &x, int &y, int &w, int &h, char &code) {
  int unit = W / 10, y0 = BarH + 44, keyH = (H - y0 - 4) / 4;
  y = y0 + row * keyH; h = keyH;
  if (row < 3) {
    const char *k = kbRow(kbMode, row);
    int n = strlen(k);
    if (idx >= n) return false;
    x = (W - n * unit) / 2 + idx * unit; w = unit; code = k[idx];
    return true;
  }
  if (idx == 0)      { x = 0;        w = 2 * unit;     code = 3; }     // 123 / ABC
  else if (idx == 1) { x = 2 * unit; w = 6 * unit;     code = ' '; }   // space
  else if (idx == 2) { x = 8 * unit; w = W - 8 * unit; code = 4; }     // OK
  else return false;
  return true;
}

void kbDrawKey(int row, int idx) {
  int x, y, w, h; char c;
  if (!kbKeyRect(row, idx, x, y, w, h, c)) return;
  bool special = (c >= 1 && c <= 4);
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, c == 4 ? 0x03E0 : special ? 0x2945 : 0x4208);
  tft.drawRect(x + 1, y + 1, w - 2, h - 2, TFT_DARKGREY);
  char lb[8];
  switch (c) {
    case 1: strcpy(lb, kbMode < 2 ? "Aa" : (kbMode == 2 ? "1/2" : "2/2")); break;
    case 2: strcpy(lb, "DEL"); break;
    case 3: strcpy(lb, kbMode < 2 ? "123" : "ABC"); break;
    case 4: strcpy(lb, "OK"); break;
    case ' ': strcpy(lb, "space"); break;
    default: lb[0] = c; lb[1] = 0;
  }
  uint8_t f = (!special && c != ' ' && h >= 40 && w >= 40) ? 4 : 2;
  tft.setTextColor(TFT_WHITE); tft.setTextDatum(MC_DATUM);
  tft.drawString(lb, x + w / 2, y + h / 2, f);
  tft.setTextDatum(TL_DATUM);
}

void kbDrawKeys() {
  int y0 = BarH + 44;
  tft.fillRect(0, y0, W, H - y0, TFT_BLACK);
  for (int r = 0; r < 4; r++) for (int i = 0; i < 10; i++) kbDrawKey(r, i);
}

void kbDrawField() {
  int fy = BarH + 4;
  tft.fillRect(4, fy, W - 8, 34, TFT_BLACK);
  tft.drawRect(4, fy, W - 8, 34, TFT_WHITE);
  uint8_t f = (W >= 400) ? 4 : 2;
  String s = kbText + "_";
  while (s.length() > 1 && tft.textWidth(s, f) > W - 24) s.remove(0, 1);   // show the tail if too long
  tft.setTextColor(TFT_YELLOW); tft.setTextDatum(ML_DATUM);
  tft.drawString(s, 10, fy + 17, f);
  tft.setTextDatum(TL_DATUM);
}

void drawKeyboard() {
  page = PAGE_KB;
  char title[40];
  int mc = (W - 170) / 9 - 10; if (mc < 4) mc = 4;
  snprintf(title, sizeof(title), "Password: %.*s", mc, kbTarget.c_str());
  drawBar(title, false, "BACK");
  kbDrawField();
  kbDrawKeys();
  flushScreen();
}

void connectTo(const String &ssid, const String &pass) {
  showMessage("Connecting...", ssid.c_str());
  WiFi.disconnect(); delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    cfgSsid = ssid; cfgPass = pass; saveWifi();
    showMessage("Connected", WiFi.localIP().toString().c_str());
    delay(1500);
    lastWxTry = 0;                                   // fetch weather straight away
    if (time(nullptr) < 1700000000) configTzTime(TZS[cfgTz].posix, "pool.ntp.org", "time.google.com");
    extern void drawSettings();
    drawSettings();
  } else {
    WiFi.disconnect(); delay(100);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());    // go back to the previous network
    showMessage("Could not connect", "Check the password and try again");
    delay(2000);
    if (pass.length()) drawKeyboard(); else drawPicker();
  }
}

void kbPress(char c) {
  if (c == 1) {                                      // shift / symbol page
    kbMode = (kbMode == 0) ? 1 : (kbMode == 1) ? 0 : (kbMode == 2) ? 3 : 2;
    kbDrawKeys(); flushScreen();
  } else if (c == 2) {                               // delete
    if (kbText.length()) kbText.remove(kbText.length() - 1);
    kbDrawField(); flushScreen();
  } else if (c == 3) {                               // abc <-> 123
    kbMode = (kbMode < 2) ? 2 : 0;
    kbDrawKeys(); flushScreen();
  } else if (c == 4) {                               // OK
    if (kbText.length()) connectTo(kbTarget, kbText);
  } else {
    if (kbText.length() < 63) kbText += c;
    if (kbMode == 1) { kbMode = 0; kbDrawKeys(); }   // one capital, then back to lower case
    kbDrawField(); flushScreen();
  }
}

void kbTap(uint16_t tx, uint16_t ty) {
  if (ty < BarH + 4) { if (tx < 60) drawPicker(); return; }
  for (int r = 0; r < 4; r++)
    for (int i = 0; i < 10; i++) {
      int x, y, w, h; char c;
      if (kbKeyRect(r, i, x, y, w, h, c) && (int)tx >= x && (int)tx < x + w && (int)ty >= y && (int)ty < y + h) {
        kbPress(c);
        return;
      }
    }
}

void wifiSelected(int i) {
  kbTarget = nets[i].ssid;
  if (!nets[i].enc) { connectTo(kbTarget, ""); return; }
  kbText = ""; kbMode = 0;
  drawKeyboard();
}

void pickTap(uint16_t tx, uint16_t ty) {
  if (ty < BarH + 4) { if (tx < 60) { extern void drawSettings(); drawSettings(); } return; }
  int by = H - 38;
  if (ty >= by) {
    int pages = pickPages();
    if (tx < 98)       { pickPage = (pickPage + pages - 1) % pages; drawPicker(); }
    else if (tx < 194) { pickPage = (pickPage + 1) % pages;         drawPicker(); }
    else if (pickKind == PK_WIFI && (int)tx >= W - 100) openWifiList();
    return;
  }
  int rowH = pickRowH(), rows = pickRows();
  int k = ((int)ty - (BarH + 4)) / rowH;
  if (k < 0 || k >= rows) return;
  int i = pickPage * rows + k;
  if (i >= pickCount()) return;
  extern void drawSettings();
  if (pickKind == PK_TZ)        { cfgTz = i; saveLocation(); applyTz(); drawSettings(); }
  else if (pickKind == PK_CITY) { cfgCity = i; cfgTz = CITIES[i].tz; saveLocation(); applyTz(); drawSettings(); }
  else wifiSelected(i);
}

// ---------- settings main page ----------
int sliderLineH() { return H >= 300 ? 30 : 26; }
int slidersH()    { return 2 * sliderLineH() + 12; }
int settingRowH() { return (H - BarH - 40 - slidersH()) / 3; }
int slidersY()    { return BarH + 8 + 3 * (settingRowH() + 8); }
int sliderX0()    { return 112; }
int sliderX1()    { return W - 66; }
int sliderValue(int i) { return i == 0 ? spkVolume : briPct; }

void drawSettingRow(int r, const char *label, const char *value, const char *sub) {
  int rowH = settingRowH();
  bool compact = rowH < 56;
  int y = BarH + 8 + r * (rowH + 8);
  tft.fillRect(6, y, W - 12, rowH, 0x10A2);
  tft.drawRect(6, y, W - 12, rowH, TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN);  tft.drawString(label, 14, y + (compact ? 2 : 6), 2);
  uint8_t vf = (!compact && tft.textWidth(value, 4) <= W - 70) ? 4 : 2;
  tft.setTextColor(TFT_WHITE); tft.drawString(value, 14, y + (compact ? 19 : 26), vf);
  if (sub && sub[0] && tft.textWidth(label, 2) + tft.textWidth(sub, 2) + 40 < W - 60) {   // detail, right-aligned on the label line
    tft.setTextColor(TFT_LIGHTGREY); tft.setTextDatum(TR_DATUM);
    tft.drawString(sub, W - 40, y + (compact ? 2 : 6), 2);
    tft.setTextDatum(TL_DATUM);
  }
  tft.setTextColor(TFT_YELLOW); tft.setTextDatum(MR_DATUM);
  tft.drawString(">", W - 14, y + rowH / 2, 4);
  tft.setTextDatum(TL_DATUM);
}

void drawSliderLine(int i) {                         // i: 0 = volume, 1 = brightness
  int lh = sliderLineH();
  int y = slidersY() + 6 + i * lh, cy = y + lh / 2;
  tft.fillRect(8, y, W - 16, lh, 0x10A2);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_CYAN); tft.drawString(i == 0 ? "Volume" : "Brightness", 14, cy, 2);
  int v = sliderValue(i);
  int x0 = sliderX0(), x1 = sliderX1(), r = lh / 2 - 4;
  int kx = x0 + (x1 - x0) * v / 100;
  tft.fillRect(x0, cy - 3, x1 - x0, 7, TFT_DARKGREY);
  if (kx > x0) tft.fillRect(x0, cy - 3, kx - x0, 7, TFT_YELLOW);
  tft.fillCircle(kx, cy, r, TFT_WHITE);
  tft.drawCircle(kx, cy, r, TFT_DARKGREY);
  char pc[8]; snprintf(pc, sizeof(pc), "%d%%", v);
  tft.setTextColor(TFT_WHITE); tft.setTextDatum(MR_DATUM);
  tft.drawString(pc, W - 14, cy, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawSliders() {
  tft.fillRect(6, slidersY(), W - 12, slidersH(), 0x10A2);
  tft.drawRect(6, slidersY(), W - 12, slidersH(), TFT_WHITE);
  drawSliderLine(0);
  drawSliderLine(1);
}

void sliderSetFromX(int tx) {                        // sets the slider that was touched (sliderActive)
  lastTouchMs = millis();
  int x0 = sliderX0(), x1 = sliderX1();
  int v = (tx - x0) * 100 / (x1 - x0);
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  if (sliderActive == 1 && v < 10) v = 10;           // never let the screen go fully dark
  int &ref = (sliderActive == 0) ? spkVolume : briPct;
  if (v == ref) return;
  ref = v;
  if (sliderActive == 1) applyBrightness();          // brightness changes live while dragging
  drawSliderLine(sliderActive);
  flushScreen();
}

void sliderReleased() {                              // save; volume also plays a beep at the new level
  if (sliderActive == 1) { saveBrightness(); return; }
  saveVolume();
#if ENABLE_VOICE
  if (spkVolume > 0 && voiceHwInit()) beep(880, 120);
#endif
}

void drawSettings() {
  page = PAGE_SETTINGS;
  drawBar("Settings", false, "HOME");
  char sub[48], sub2[40];
  if (WiFi.status() == WL_CONNECTED) snprintf(sub, sizeof(sub), "Connected  %s", WiFi.localIP().toString().c_str());
  else strcpy(sub, "Not connected");
  drawSettingRow(0, "WiFi network", cfgSsid.length() ? cfgSsid.c_str() : "(none)", sub);
  drawSettingRow(1, "Time zone", TZS[cfgTz].name, TZS[cfgTz].api);
  snprintf(sub2, sizeof(sub2), "%.2f, %.2f", CITIES[cfgCity].lat, CITIES[cfgCity].lon);
  drawSettingRow(2, "City (weather and moon)", CITIES[cfgCity].name, sub2);
  drawSliders();
  flushScreen();
}

void settingsTap(uint16_t tx, uint16_t ty) {
  if (ty < BarH + 6) { if (tx < 60) drawHome(); return; }
  int sy = slidersY();
  if ((int)ty >= sy && (int)ty < sy + slidersH()) {                // volume / brightness sliders: tap, then drag
    int i = ((int)ty - (sy + 6)) / sliderLineH();
    sliderActive = (i < 0) ? 0 : (i > 1 ? 1 : i);
    sliderDragging = true;
    sliderSetFromX(tx);
    return;
  }
  int rowH = settingRowH();
  for (int r = 0; r < 3; r++) {
    int y = BarH + 8 + r * (rowH + 8);
    if ((int)ty >= y && (int)ty < y + rowH) {
      if (r == 0) openWifiList();
      else openPicker(r == 1 ? PK_TZ : PK_CITY);
      return;
    }
  }
}

// =====================================================================
//  App plumbing
// =====================================================================
void splash(const char* msg) {
  tft.fillRect(0, 0, W, H, TFT_BLACK);
  tft.setTextColor(TFT_WHITE); tft.setTextDatum(MC_DATUM);
  tft.drawString(msg, W / 2, H / 2, 4);
  tft.setTextColor(TFT_DARKGREY);
  tft.drawString("Canberra Dash v" FW_VERSION, W / 2, H - 20, 2);
  tft.setTextDatum(TL_DATUM);
  flushScreen();
}

void computeGeometry() {
  W = tft.width(); H = tft.height();
  topH = H * 35 / 100;
  dateH = H * 15 / 100;
  botY = topH + dateH;
  // calendar box is wider so two-digit days fit; weights 14:10:10:10
  const int wt[4] = {14, 10, 10, 10};
  int x = 0;
  for (int i = 0; i < 4; i++) {
    int slot = W * wt[i] / 44;
    boxX[i] = x + 3; boxW[i] = slot - 6;
    x += slot;
  }
  boxY = botY + 3; boxH = H - botY - 6;
}

void redraw() {
  switch (page) {
    case PAGE_CAL:   drawCalPage();   break;
    case PAGE_WX:    drawWxPage();    break;
    case PAGE_MOON:  drawMoonPage();  break;
    case PAGE_VOICE: drawVoicePage(); break;
    case PAGE_SETTINGS: drawSettings(); break;
    case PAGE_PICK:  drawPicker();   break;
    case PAGE_KB:    drawKeyboard(); break;
    default:         drawHome();      break;
  }
}

void handleTap(uint16_t tx, uint16_t ty) {
  lastTouchMs = millis();
  switch (page) {
    case PAGE_SETTINGS: settingsTap(tx, ty); return;
    case PAGE_PICK:     pickTap(tx, ty);     return;
    case PAGE_KB:       kbTap(tx, ty);       return;
    default: break;
  }
  if (page == PAGE_HOME) {
    if ((int)tx >= W - 56 && (int)ty >= topH - 6 && (int)ty < botY + 4) { drawSettings(); return; }   // gear
    if (ty < botY) return;
    int i = -1;
    for (int k = 0; k < 4; k++) if (tx >= boxX[k] - 3 && tx < boxX[k] + boxW[k] + 3) i = k;
    if (i == 0) { calPage = 0; drawCalPage(); }
    else if (i == 1) drawWxPage();
    else if (i == 2) drawMoonPage();
    else if (i == 3) { vStatus = "Tap the button to talk"; vUser = ""; vReply = ""; drawVoicePage(); }
    return;
  }
  if (ty < BarH + 10) {                                  // bar buttons, as in the tracker
    if (tx < 60) { drawHome(); return; }                 // HOME
    if (tx < 122 && page == PAGE_CAL && calPages > 1) {  // NEXT
      calPage = (calPage + 1) % calPages;
      drawCalPage();
    }
    return;
  }
  if (page == PAGE_VOICE && ty >= voiceBtnY) { voiceTurn(); lastTouchMs = millis(); }
}

void tickClock() {
  tm t;
  bool ok = getNow(t);
  int minKey = ok ? t.tm_hour * 60 + t.tm_min : -2;
  int dayKey = ok ? (t.tm_year * 400 + t.tm_yday) : -2;
  bool minChanged = (minKey != lastMinute), dayChanged = (dayKey != lastDayKey);
  lastMinute = minKey; lastDayKey = dayKey;
  if (page != PAGE_HOME || !(minChanged || dayChanged)) return;
  drawClockZone(ok ? &t : nullptr);
  drawDateZone(ok ? &t : nullptr);
  if (dayChanged) drawBottom();
  flushScreen();
}

void tickWeather(uint32_t now) {
  uint32_t interval = wx.ok ? 30UL * 60UL * 1000UL : 20UL * 1000UL;
  if (lastWxTry != 0 && now - lastWxTry < interval) return;
  lastWxTry = now ? now : 1;
  if (!fetchWeather()) return;
  if (page == PAGE_HOME) { drawBoxWeather(); flushScreen(); }
  else if (page == PAGE_WX) drawWxPage();
}

void tickWifi(uint32_t now) {
  static uint32_t last = 0;
  if (page >= PAGE_SETTINGS) return;                     // do not interrupt scans / password entry
  if (WiFi.status() == WL_CONNECTED || now - last < 20000) return;
  last = now;
  WiFi.disconnect();
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("Canberra Dash v" FW_VERSION);

#ifdef FNK0104N_3P5_320x480_ST77922
  tft_st77922.Init();
  tft_st77922.Set_Rotation(1);
  tft.createSprite(tft_st77922.Get_Width(), tft_st77922.Get_Height());
  tft.setSwapBytes(true);
  tft.fillSprite(TFT_BLACK);
  touch_st77922.init();
  touch_st77922.Set_Rotation(1);
#else
  ft6336u.begin();
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
#endif

  W = tft.width();
  H = tft.height();
  Serial.printf("Display %d x %d\n", W, H);

#if BOOT_TEST
  tft.fillRect(0, 0, W, H, TFT_RED);   flushScreen(); delay(250);
  tft.fillRect(0, 0, W, H, TFT_GREEN); flushScreen(); delay(250);
  tft.fillRect(0, 0, W, H, TFT_BLUE);  flushScreen(); delay(250);
  tft.fillRect(0, 0, W, H, TFT_BLACK); flushScreen();
#endif

  loadConfig();
  applyBrightness();                                 // saved screen brightness
  computeGeometry();
  splash("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "No WiFi");

  configTzTime(TZS[cfgTz].posix, "pool.ntp.org", "time.google.com");
  splash("Setting clock...");
  t0 = millis();
  while (time(nullptr) < 1700000000 && millis() - t0 < 15000) delay(250);

  for (int i = 0; i < 3 && !fetchWeather(); i++) delay(2000);   // first try can fail right after WiFi connects
  lastWxTry = millis() ? millis() : 1;
  lastTouchMs = millis();
  drawHome();
}

void loop() {
  uint16_t tx, ty;
  bool touching = readTouch(tx, ty);
  if (touching && !wasTouching) handleTap(tx, ty);   // act on touch-down only
  else if (touching && sliderDragging) sliderSetFromX(tx);                       // dragging the volume slider
  if (!touching && wasTouching && sliderDragging) { sliderDragging = false; sliderReleased(); }
  wasTouching = touching;

  uint32_t now = millis();
  static uint32_t lastTick = 0;
  if (now - lastTick >= 500) { lastTick = now; tickClock(); }
  tickWeather(now);
  tickWifi(now);

  // WiFi icon only when the connection state / strength changes
  if (wifiLevel() != lastWifiLevel) {
    drawWifiIcon();
    flushScreen();
  }

  // return to the home page after 60 s without a touch
  uint32_t idleMs = (page >= PAGE_SETTINGS) ? 180000UL : 60000UL;
  if (page != PAGE_HOME && now - lastTouchMs > idleMs) drawHome();

  delay(20);
}
