#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include "Boardoza_FT800.h"
#include "DFRobotDFPlayerMini.h"
#include "VNH5019A.h"
#include "T3A33BRG.h"
#include <SparkFun_VL53L5CX_Library.h>

const bool KULLAN_MOTOR = true;
const bool KULLAN_LED   = true;
const bool KULLAN_LIDAR = true;
const bool KULLAN_MP3   = true;
const bool KULLAN_RTC   = true;
const bool KULLAN_WIFI  = true;

// Set to 0 if the installed library lacks setRangingMode/setIntegrationTime.
#define LIDAR_UZUN_MENZIL 1

// ---------------------------------------------------------------- Audio

constexpr uint8_t MP3_RX_PIN = 44;
constexpr uint8_t MP3_TX_PIN = 43;

const uint8_t MP3_SES_MAKS   = 30;   // DFPlayer hardware maximum
uint8_t       mp3SesSeviyesi = 30;
bool          sesAcik        = true;

// 1: files live in /mp3/000X.mp3 (addressed by name)
// 0: root directory, addressed by FAT write order
#define MP3_KLASOR_MODU 1

// Display labels only; index must match the file number on the card.
const uint8_t MUZIK_ADET = 9;
const char* MUZIK_ISIM[MUZIK_ADET] = {
  "JET ENGINE",
  "STAR WARS",
  "TURKISH MARCH",
  "GTA",
  "GAME OF THRONES",
  "AVENGERS",
  "TOKYO DRIFT",
  "FORMULA 1",
  "PIRATES CARIBBEAN"
};

uint8_t aktifParca = 1;   // selected track
uint8_t calanParca = 0;   // last track handed to the DFPlayer

HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini mp3Player;
bool    mp3Hazir   = false;
bool    sesCaliyor = false;
uint8_t sesAnlikVolume = 0;
bool    mp3IlkCalma = true;
unsigned long sesSonIslem = 0;

const uint8_t SES_FADE_IN_ADIM  = 3;   // volume steps per 50 ms
const uint8_t SES_FADE_OUT_ADIM = 4;

inline void mp3ParcaCal(uint8_t n) {
#if MP3_KLASOR_MODU
  mp3Player.playMp3Folder(n);
#else
  mp3Player.play(n);
#endif
}

// ----------------------------------------------------------- Motor limits

const int MOTOR_NOMINAL_RPM = 130;   // no-load RPM at 6V, full duty

const int MOTOR_TAVAN_YUZDE = 60;    // duty ceiling as a percentage of full
const int MOTOR_PWM_TAM = 400;       // VNH5019A full duty
const int MOTOR_MIN_PWM = 130;       // dead-zone threshold; raise if the shaft stalls
const int MOTOR_MAX_PWM =
      MOTOR_MIN_PWM + ((MOTOR_PWM_TAM - MOTOR_MIN_PWM) * MOTOR_TAVAN_YUZDE) / 100;

// Displayed RPM scales linearly with throttle, capped by MOTOR_TAVAN_YUZDE.
const int MAKS_RPM = (MOTOR_NOMINAL_RPM * MOTOR_TAVAN_YUZDE) / 100;

const int MOTOR_TAVAN_DUTY = (MOTOR_MAX_PWM * 100) / MOTOR_PWM_TAM;

// ---------------------------------------------------------- Proximity trigger

const bool LIDAR_TETIK_KULLAN = true;

const int LIDAR_MIN_AYAR = 200;    // mm
const int LIDAR_MAK_AYAR = 3500;   // mm, practical sensor limit
const int LIDAR_ADIM     = 100;    // mm per button press

int  lidarTetikMesafe = 1500;
int  lidarBirakMesafe = 1700;      // release threshold (hysteresis)
const int  LIDAR_TETIK_HIZ = 100;  // throttle percentage on trigger
const unsigned long LIDAR_BIRAK_GECIKME = 1500;   // ms
const uint8_t LIDAR_ONAY_SAYISI = 2;              // consecutive hits before firing

bool     lidarTetikAktif = false;
uint8_t  lidarOnaySayaci = 0;
unsigned long lidarSonGormeZamani = 0;
bool     otomatikMod = true;

// ---------------------------------------------------------------- Pin map

constexpr uint8_t FT800_INT  = 47;
constexpr uint8_t FT800_PD   = 48;
constexpr uint8_t FT800_CS   = 7;
constexpr uint8_t FT800_MISO = 16;
constexpr uint8_t FT800_MOSI = 15;
constexpr uint8_t FT800_SCK  = 17;

Boardoza_FT800 ft800(FT800_INT, FT800_PD, FT800_CS);

constexpr uint8_t I2C_SDA_PIN = 1;
constexpr uint8_t I2C_SCL_PIN = 2;
#define TOUCH_TD_STATUS (0x02)

constexpr uint8_t MOTOR_INA_PIN   = 36;
constexpr uint8_t MOTOR_INB_PIN   = 37;
constexpr uint8_t MOTOR_PWM_PIN   = 35;
constexpr uint8_t MOTOR_DIAGA_PIN = 38;
constexpr uint8_t MOTOR_DIAGB_PIN = 38;
constexpr uint8_t MOTOR_CS_PIN    = 6;

VNH5019A motor(MOTOR_INA_PIN, MOTOR_INB_PIN, MOTOR_PWM_PIN,
               MOTOR_DIAGA_PIN, MOTOR_DIAGB_PIN, MOTOR_CS_PIN);

// ---------------------------------------------------------------- Motor state

float motorHiziAnlik = 0;
int   motorHiziHedef = 0;
int   motorRPM       = 0;
int   pwmAnlik       = 0;
bool  acilDurdurma   = false;   // STOP button: fast ramp down
unsigned long sonSpoolZamani = 0;

const float SPOOL_UP_HIZ    = 12.0;   // %/s, automatic mode
const float SPOOL_DOWN_HIZ  = 6.0;
const float MANUEL_UP_HIZ   = 90.0;   // %/s, manual mode
const float MANUEL_DOWN_HIZ = 90.0;

const int BASLANGIC_HIZ = 45;

// Start-up assist: a decaying PWM boost that clears the gearbox stiction
// without introducing a step the slew limiter would have to chase.
const int KALKIS_EK_PWM = 90;
const int KALKIS_SURESI = 260;   // ms
const unsigned long KALKIS_MIN_DURUS = 200;
const int PWM_SLEW = 26;         // max PWM change per 20 ms tick
const float DURMA_ESIGI = 0.40;

bool kalkisAktif = false;
unsigned long kalkisBaslangic = 0;
bool motorDurdu = true;
unsigned long motorDurmaZamani = 0;

// ------------------------------------------------------------------ LEDs

constexpr uint8_t LED_DATA_PIN  = 41;
constexpr uint8_t LED_CLOCK_PIN = 42;
constexpr uint8_t LED_ADET      = 16;

T3A33BRG ledSerit(LED_DATA_PIN, LED_CLOCK_PIN);

enum LedMod : uint8_t {
  LED_RAINBOW = 0,   // ignores the selected color
  LED_SOLID,
  LED_FADE,
  LED_PING,
  LED_CHASE,
  LED_MOD_ADET
};
const char* LED_MOD_ISIM[LED_MOD_ADET] = { "RAINBOW", "SOLID", "FADE", "PING", "CHASE" };

int ledParlakligiYuzde = 75;
uint8_t ledModu = LED_RAINBOW;

uint8_t ledHue = 170;   // 0..255, position on the color wheel
uint8_t ledSat = 255;   // 0..255, 0 is white

uint8_t  ledHueOffset = 0;
uint32_t ledAnimStep  = 0;
unsigned long ledSonAnimZamani = 0;
float ledFadeAcisi = 0;
float ledPingPoz   = 0;
int   ledPingYon   = 1;

// ----------------------------------------------------------------- LIDAR

SparkFun_VL53L5CX lidarImager;
VL53L5CX_ResultsData lidarVeri;
bool lidarHazir = false;
int  lidarEnYakinMesafe = 0;

const bool LIDAR_SERI_YAZ = false;

// ---------------------------------------------------------------- Palette

#define ZEMIN      0x070B10   // screen background
#define PANEL_BG   0x111A24   // panel body
#define PANEL_ACIK 0x1C2836   // title band, filled blocks
#define CYAN       0x35B7E8   // primary accent
#define NEON_GREEN 0x35D07F   // active, confirmed
#define ORANGE_RED 0xFF5A2B   // warning, stop
#define SARI       0xFFB020   // caution
#define TEXT_DIM   0x8AA0B4   // secondary text
#define LINE_DIM   0x3D5266   // panel borders
#define ARC_DIM    0x24323F   // inactive arcs and lamps
#define WHITE      0xE9F0F6   // primary text

// ------------------------------------------------------------- Page layout

enum Sayfa : uint8_t {
  SAYFA_MOTOR = 0,
  SAYFA_ISIK,
  SAYFA_SENSOR,
  SAYFA_MUZIK,
  SAYFA_SAAT,
  SAYFA_AYAR,
  SAYFA_ADET
};

enum Katman : uint8_t {
  KATMAN_YOK = 0,
  KATMAN_WIFI,
  KATMAN_KLAVYE
};

uint8_t aktifSayfa  = SAYFA_MOTOR;
uint8_t aktifKatman = KATMAN_YOK;

const char* SAYFA_ISIM[SAYFA_ADET] = { "ENGINE", "LIGHTS", "SENSOR", "MUSIC", "CLOCK", "SETTINGS" };
const char* SEKME_ISIM[SAYFA_ADET] = { "ENGINE", "LIGHTS", "SENSOR", "MUSIC", "CLOCK", "SETUP"    };

const int EKRAN_W  = 480;
const int EKRAN_H  = 272;
const int UST_H    = 28;
const int ICERIK_Y = 34;
const int ICERIK_ALT = 204;
const int SEKME_Y  = 230;
const int SEKME_H  = 36;
const int SEKME_W  = EKRAN_W / SAYFA_ADET;

// Left panel (gauge, wheel, dial) and right column, shared by all pages.
const int SOL_X = 8,   SOL_W = 180;
const int SAG_X = 196, SAG_W = 276;

const int X_SOL   = 16;

unsigned long sonDokunusZamani = 0;
const unsigned long beklemeSuresi = 30000;   // idle timeout before the clock screen
bool saatModu = false;

bool oncekiDokunma = false;
int  dokunmaBasX = 0, dokunmaBasY = 0;
unsigned long dokunmaBasZamani = 0;
bool kontrolYakaladi = false;   // a control consumed the gesture

const int JEST_MIN_DX = 70;
const int JEST_MAKS_DY = 60;
const unsigned long JEST_MAKS_SURE = 800;

int sonTouchX = 0, sonTouchY = 0;

// Uncomment if the library does not declare these:
// uint8_t first_x_touch = 0;
// uint8_t first_y_touch = 0;

// ------------------------------------------------------ RTC, WiFi and NTP

#define PCF8563_ADRES 0x51
#define PCF8563_REG_KONTROL1 0x00
#define PCF8563_REG_SANIYE   0x02

bool rtcVar = false;
bool rtcGuvenilir = false;
struct tm suankiZaman;
bool zamanGecerli = false;
bool ntpEsitlendi = false;

const char* ZAMAN_DILIMI = "<+03>-3";   // Turkey, no DST
const char* NTP_SUNUCU_1 = "pool.ntp.org";
const char* NTP_SUNUCU_2 = "time.google.com";

const char* GUN_ISIM[7] = { "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY",
                            "THURSDAY", "FRIDAY", "SATURDAY" };

Preferences ayarDepo;

String wifiSSID = "";
String wifiPASS = "";

enum WifiDurum : uint8_t { WD_KAPALI = 0, WD_BAGLANIYOR, WD_BAGLI, WD_HATA };
uint8_t wifiDurum = WD_KAPALI;
unsigned long wifiDenemeZamani = 0;
const unsigned long WIFI_YENIDEN_DENEME = 20000;

int  taramaSonuc   = -2;
bool taramaSuruyor = false;
int  taramaKaydirma = 0;
const int TARAMA_SATIR_ADET = 4;

char klavyeMetin[64];
uint8_t klavyeUzunluk = 0;
uint8_t klavyeKatman = 0;
String  klavyeHedefSSID = "";

// Settings are written once, a few seconds after the last change,
// so dragging a slider does not hammer the flash.
bool ayarDegisti = false;
unsigned long ayarDegisimZamani = 0;
const unsigned long AYAR_KAYIT_GECIKME = 4000;

// ------------------------------------------------------------ Prototypes

void mp3DetayYazdir(uint8_t type, int value);
void motorSesiGuncelle();
void motorHedefBelirle(int yeniHedef);
void ayarlariYukle();
void ayarlariKaydet();
void ayarIsaretle();
bool rtcBaslat();
bool rtcSaatiOku(struct tm &zaman);
bool rtcSaatiYaz(const struct tm &zaman);
void wifiBaslat();
void ntpEsitle();
void wifiGorevGuncelle();
void wifiTaramaBaslat();
void klavyeAc(const String &ssid);
void ekranCiz();
void ledRenkTekeri(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b);
void hueSatAyir(uint8_t hue, uint8_t sat, uint8_t &r, uint8_t &g, uint8_t &b);
uint32_t hueSatRenk(uint8_t hue, uint8_t sat);

// ------------------------------------------------------------ Text helpers

int fontGenislik(int font) {
  switch (font) {
    case 26: return 8;
    case 27: return 9;
    case 28: return 11;
    case 29: return 13;
    case 30: return 17;
    case 31: return 22;
    default: return 10;
  }
}

void yaz(int x, int y, int font, uint32_t renk, const char* s) {
  ft800.drawText(x, y, font, renk, (char*)s);
}

void yazOrta(int cx, int y, int font, uint32_t renk, const char* s) {
  int w = strlen(s) * fontGenislik(font);
  int x = cx - w / 2;
  if (x < 2) x = 2;
  ft800.drawText(x, y, font, renk, (char*)s);
}

void yazSag(int sagX, int y, int font, uint32_t renk, const char* s) {
  int w = strlen(s) * fontGenislik(font);
  int x = sagX - w;
  if (x < 2) x = 2;
  ft800.drawText(x, y, font, renk, (char*)s);
}

// ----------------------------------------------------------- Drawing core

// drawLine() treats its width argument as a radius: the rendered stroke is
// twice as wide and its rounded caps overhang both ends. Every primitive
// goes through ciz(), which takes pixels and halves them here. This divisor
// is the single calibration point for stroke weight.
const int KALINLIK_BOLEN = 2;

inline void ciz(int x1, int y1, int x2, int y2, uint32_t renk, int kalinlikPx) {
  int k = kalinlikPx / KALINLIK_BOLEN;
  if (k < 1) k = 1;
  ft800.drawLine(x1, y1, x2, y2, renk, k);
}

void cizgiYatay(int x1, int x2, int y, uint32_t renk) {
  ciz(x1, y, x2, y, renk, 1);
}

void cizgiDikey(int x, int y1, int y2, uint32_t renk) {
  ciz(x, y1, x, y2, renk, 1);
}

void cerceve(int x, int y, int w, int h, uint32_t renk, int kal = 2) {
  ciz(x,     y,     x + w, y,     renk, kal);
  ciz(x,     y + h, x + w, y + h, renk, kal);
  ciz(x,     y,     x,     y + h, renk, kal);
  ciz(x + w, y,     x + w, y + h, renk, kal);
}

// Filled rectangle drawn as a single thick line to keep the display list
// short. The ends are shortened by half the stroke width so the rounded
// caps stay inside the requested box.
void dolu(int x, int y, int w, int h, uint32_t renk) {
  if (w <= 0 || h <= 0) return;
  if (w >= h) {
    int yari = h / 2;
    int x1 = x + yari, x2 = x + w - yari;
    if (x2 < x1) { x1 = x2 = x + w / 2; }
    ciz(x1, y + h / 2, x2, y + h / 2, renk, h);
  } else {
    int yari = w / 2;
    int y1 = y + yari, y2 = y + h - yari;
    if (y2 < y1) { y1 = y2 = y + h / 2; }
    ciz(x + w / 2, y1, x + w / 2, y2, renk, w);
  }
}

void buton(int x, int y, int w, int h, uint32_t renk, const char* etiket, bool secili = false) {
  if (secili) dolu(x + 2, y + 2, w - 4, h - 4, PANEL_BG);
  cerceve(x, y, w, h, renk, 2);
  yazOrta(x + w / 2, y + (h - 16) / 2, 26, renk, etiket);
}

// Bordered panel with an optional filled title tab in the top-left corner.
void panelCiz(int x, int y, int w, int h, const char* baslik,
              uint32_t cerRenk = LINE_DIM, bool govdeDolgu = false) {
  if (govdeDolgu) dolu(x + 2, y + 2, w - 4, h - 4, PANEL_BG);
  cerceve(x, y, w, h, cerRenk, 2);
  if (baslik && baslik[0]) {
    int bw = (int)strlen(baslik) * 8 + 14;
    dolu(x + 2, y + 2, bw, 16, PANEL_ACIK);
    yaz(x + 9, y + 2, 26, TEXT_DIM, baslik);
  }
}

void blokButon(int x, int y, int w, int h, const char* etiket,
               uint32_t dolgu, uint32_t yaziRenk, uint32_t cerRenk) {
  dolu(x + 2, y + 2, w - 4, h - 4, dolgu);
  cerceve(x, y, w, h, cerRenk, 2);
  yazOrta(x + w / 2, y + (h - 16) / 2, 26, yaziRenk, etiket);
}

void lamba(int x, int y, bool yanik, uint32_t renk, const char* etiket) {
  dolu(x, y, 12, 12, yanik ? renk : ARC_DIM);
  if (etiket && etiket[0]) yaz(x + 18, y - 2, 26, yanik ? renk : ARC_DIM, etiket);
}

bool icinde(int tx, int ty, int x, int y, int w, int h) {
  return (tx >= x && tx <= x + w && ty >= y && ty <= y + h);
}

void barCiz(int x, int y, int w, int h, int pct, uint32_t renk) {
  pct = constrain(pct, 0, 100);
  dolu(x + 2, y + 2, w - 4, h - 4, PANEL_BG);
  int doluGenislik = ((w - 4) * pct) / 100;
  if (doluGenislik > 3) dolu(x + 2, y + 2, doluGenislik, h - 4, renk);
  cerceve(x, y, w, h, LINE_DIM, 2);
}

// Bar plus handle; hedefPct < 0 hides the commanded-value marker.
void surguCiz(int x, int y, int w, int h, int pct, uint32_t renk, int hedefPct) {
  barCiz(x, y, w, h, pct, renk);
  int hx = x + ((w - 4) * constrain(pct, 0, 100)) / 100 + 2;
  hx = constrain(hx, x + 7, x + w - 7);
  dolu(hx - 6, y - 4, 12, h + 8, PANEL_ACIK);
  cerceve(hx - 6, y - 4, 12, h + 8, WHITE, 2);
  if (hedefPct >= 0 && hedefPct != pct) {
    int tx = x + ((w - 4) * constrain(hedefPct, 0, 100)) / 100 + 2;
    tx = constrain(tx, x + 1, x + w - 1);
    ciz(tx, y - 7, tx, y + h + 7, SARI, 2);
  }
}

// ------------------------------------------------------------ Arcs, gauges

// Angles: 0 points up, positive is clockwise. The FT800 has no circle
// command, so arcs are approximated with short line segments.
void yayCiz(int cx, int cy, int r, float basDer, float bitDer,
            uint32_t renk, int kalinlik, float adimDer = 9.0f) {
  if (bitDer <= basDer + 0.5f) return;
  int n = (int)((bitDer - basDer) / adimDer) + 1;
  float adim = (bitDer - basDer) / n;

  float a = basDer * DEG_TO_RAD;
  int px = cx + (int)(r * sinf(a));
  int py = cy - (int)(r * cosf(a));
  for (int i = 1; i <= n; i++) {
    float b = (basDer + adim * i) * DEG_TO_RAD;
    int qx = cx + (int)(r * sinf(b));
    int qy = cy - (int)(r * cosf(b));
    ciz(px, py, qx, qy, renk, kalinlik);
    px = qx; py = qy;
  }
}

void daireCiz(int cx, int cy, int r, uint32_t renk, int kalinlik) {
  yayCiz(cx, cy, r, -180.0f, 180.0f, renk, kalinlik, 15.0f);
}

void doluDaire(int cx, int cy, int r, uint32_t renk) {
  // Small radii: a zero-length stroke with rounded caps is already a disc.
  if (r <= 5) { ciz(cx, cy, cx, cy, renk, 2 * r); return; }
  const int ADIM = 6;
  for (int dy = -r + ADIM / 2; dy <= r - ADIM / 2; dy += ADIM) {
    int hw = (int)sqrtf((float)(r * r - dy * dy));
    int x1 = cx - hw + ADIM / 2, x2 = cx + hw - ADIM / 2;
    if (x2 < x1) { x1 = x2 = cx; }
    ciz(x1, cy + dy, x2, cy + dy, renk, ADIM + 1);
  }
}

const float GAUGE_BAS = -135.0f;   // 270 degree sweep
const float GAUGE_ARA = 270.0f;

void gaugeCiz(int cx, int cy, int r, int pct, uint32_t renk,
              const char* buyukDeger, int buyukFont, const char* altEtiket,
              const char* solEtiket, const char* sagEtiket) {
  pct = constrain(pct, 0, 100);

  // 18 degree segments: deviation stays below half a pixel while halving
  // the number of display list commands.
  const float YAY_ADIM = 18.0f;
  yayCiz(cx, cy, r, GAUGE_BAS, GAUGE_BAS + GAUGE_ARA, ARC_DIM, 9, YAY_ADIM);
  if (pct > 0)
    yayCiz(cx, cy, r, GAUGE_BAS, GAUGE_BAS + (GAUGE_ARA * pct) / 100.0f, renk, 9, YAY_ADIM);

  // Green / amber / red operating band, as on an aircraft instrument.
  const int rb = r + 14;
  yayCiz(cx, cy, rb, GAUGE_BAS,                     GAUGE_BAS + GAUGE_ARA * 0.75f, NEON_GREEN, 5, YAY_ADIM);
  yayCiz(cx, cy, rb, GAUGE_BAS + GAUGE_ARA * 0.75f, GAUGE_BAS + GAUGE_ARA * 0.90f, SARI,       5, YAY_ADIM);
  yayCiz(cx, cy, rb, GAUGE_BAS + GAUGE_ARA * 0.90f, GAUGE_BAS + GAUGE_ARA,         ORANGE_RED, 5, YAY_ADIM);

  for (int i = 0; i <= 4; i++) {
    float a = (GAUGE_BAS + (GAUGE_ARA / 4.0f) * i) * DEG_TO_RAD;
    ciz(cx + (int)((r + 8) * sinf(a)),  cy - (int)((r + 8) * cosf(a)),
                   cx + (int)((r + 21) * sinf(a)), cy - (int)((r + 21) * cosf(a)),
                   TEXT_DIM, 2);
  }

  // The needle starts at an inner radius so it never covers the readout.
  float ai = (GAUGE_BAS + (GAUGE_ARA * pct) / 100.0f) * DEG_TO_RAD;
  ciz(cx + (int)((r - 30) * sinf(ai)), cy - (int)((r - 30) * cosf(ai)),
                 cx + (int)((r + 3)  * sinf(ai)), cy - (int)((r + 3)  * cosf(ai)),
                 WHITE, 3);
  doluDaire(cx, cy, 4, TEXT_DIM);

  yazOrta(cx, cy - 38, buyukFont, renk, buyukDeger);
  yazOrta(cx, cy + 18, 26, TEXT_DIM, altEtiket);

  if (solEtiket) yaz(cx - r - 20, cy + r - 2, 26, ARC_DIM, solEtiket);
  if (sagEtiket) yazSag(cx + r + 24, cy + r - 2, 26, ARC_DIM, sagEtiket);
}

// --------------------------------------------------------- Analog clock face

const char* SAAT_RAKAM[12] = { "12","1","2","3","4","5","6","7","8","9","10","11" };

void ibreCiz(int cx, int cy, float aciDer, int uzunluk, int kuyruk,
             uint32_t renk, int kalinlik) {
  float a = aciDer * DEG_TO_RAD;
  float sn = sinf(a), cs = cosf(a);
  ciz(cx - (int)(kuyruk * sn), cy + (int)(kuyruk * cs),
                 cx + (int)(uzunluk * sn), cy - (int)(uzunluk * cs),
                 renk, kalinlik);
}

void analogSaatCiz(int cx, int cy, int r) {
  daireCiz(cx, cy, r,      TEXT_DIM, 3);
  daireCiz(cx, cy, r - 13, LINE_DIM, 1);

  for (int i = 0; i < 12; i++) {
    float a = (i * 30.0f) * DEG_TO_RAD;
    float sn = sinf(a), cs = cosf(a);
    ciz(cx + (int)((r - 13) * sn), cy - (int)((r - 13) * cs),
                   cx + (int)((r - 4)  * sn), cy - (int)((r - 4)  * cs),
                   WHITE, (i % 3 == 0) ? 3 : 2);
    int rx = cx + (int)((r - 30) * sn);
    int ry = cy - (int)((r - 30) * cs);
    yazOrta(rx, ry - 8, 26, WHITE, SAAT_RAKAM[i]);
  }

  int sa = zamanGecerli ? suankiZaman.tm_hour : 0;
  int dk = zamanGecerli ? suankiZaman.tm_min  : 0;
  int sn = zamanGecerli ? suankiZaman.tm_sec  : 0;

  float aciSaat = (sa % 12) * 30.0f + dk * 0.5f;
  float aciDk   = dk * 6.0f + sn * 0.1f;
  float aciSn   = sn * 6.0f;

  ibreCiz(cx, cy, aciSaat, (int)(r * 0.52f), (int)(r * 0.16f), WHITE, 5);
  ibreCiz(cx, cy, aciDk,   (int)(r * 0.76f), (int)(r * 0.18f), WHITE, 3);
  if (zamanGecerli)
    ibreCiz(cx, cy, aciSn, (int)(r * 0.84f), (int)(r * 0.22f), ORANGE_RED, 2);

  doluDaire(cx, cy, 5, TEXT_DIM);
}

// ------------------------------------------------------------- Color picker

void renkCarkiCiz(int cx, int cy, int disR, int kalinlik, uint8_t hue, uint8_t sat) {
  const int DILIM = 32;   // 11 degree wedges
  int rOrta = disR - kalinlik / 2;
  for (int i = 0; i < DILIM; i++) {
    float r0 = ((360.0f * i) / DILIM) * DEG_TO_RAD;
    float r1 = ((360.0f * (i + 1)) / DILIM) * DEG_TO_RAD;
    uint8_t h = (uint8_t)((256L * i) / DILIM);
    ciz(cx + (int)(rOrta * sinf(r0)), cy - (int)(rOrta * cosf(r0)),
                   cx + (int)(rOrta * sinf(r1)), cy - (int)(rOrta * cosf(r1)),
                   hueSatRenk(h, 255), kalinlik);
  }

  float sa = ((360.0f * hue) / 256.0f) * DEG_TO_RAD;
  ciz(cx + (int)((disR - kalinlik - 3) * sinf(sa)),
                 cy - (int)((disR - kalinlik - 3) * cosf(sa)),
                 cx + (int)((disR + 6) * sinf(sa)),
                 cy - (int)((disR + 6) * cosf(sa)),
                 WHITE, 3);

  int icR = disR - kalinlik - 8;   // preview disc in the middle
  doluDaire(cx, cy, icR, hueSatRenk(hue, sat));
}

// Saturation ramp, white through to the pure hue.
void doygunlukBariCiz(int x, int y, int w, int h, uint8_t hue, uint8_t sat) {
  const int N = 8;
  for (int i = 0; i < N; i++) {
    uint8_t sv = (uint8_t)((255L * i) / (N - 1));
    int sx = x + (w * i) / N;
    int ex = x + (w * (i + 1)) / N;
    if (i == 0)     sx += h / 2;   // keep the rounded caps inside the bar
    if (i == N - 1) ex -= h / 2;
    ciz(sx, y + h / 2, ex, y + h / 2, hueSatRenk(hue, sv), h);
  }
  cerceve(x, y, w, h, LINE_DIM);
  int px = x + (w * (int)sat) / 255;
  px = constrain(px, x + 1, x + w - 1);
  ciz(px, y - 5, px, y + h + 5, WHITE, 2);
}

// ------------------------------------------------------------- LED strip

void ledCerceveyiKilitle() {
  digitalWrite(LED_DATA_PIN, HIGH);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(LED_CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LED_CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
}

void ledRenkTekeri(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (pos < 85)       { r = pos * 3;       g = 255 - pos * 3; b = 0; }
  else if (pos < 170) { pos -= 85;  r = 255 - pos * 3; g = 0; b = pos * 3; }
  else                { pos -= 170; r = 0; g = pos * 3; b = 255 - pos * 3; }
}

// Lower saturation blends the hue towards white.
void hueSatAyir(uint8_t hue, uint8_t sat, uint8_t &r, uint8_t &g, uint8_t &b) {
  ledRenkTekeri(hue, r, g, b);
  r = (uint8_t)(255 - ((255 - r) * (int)sat) / 255);
  g = (uint8_t)(255 - ((255 - g) * (int)sat) / 255);
  b = (uint8_t)(255 - ((255 - b) * (int)sat) / 255);
}

uint32_t hueSatRenk(uint8_t hue, uint8_t sat) {
  uint8_t r, g, b;
  hueSatAyir(hue, sat, r, g, b);
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void ledSeridiGuncelleVeCiz() {
  if (!KULLAN_LED) return;

  if (millis() - ledSonAnimZamani > 3) {
    ledHueOffset += 2;
    ledAnimStep++;
    ledFadeAcisi += 0.02;
    if (ledFadeAcisi > PI * 2) ledFadeAcisi = 0;

    ledPingPoz += 0.10f * ledPingYon;
    if (ledPingPoz > (float)(LED_ADET - 1)) { ledPingPoz = LED_ADET - 1; ledPingYon = -1; }
    if (ledPingPoz < 0)                     { ledPingPoz = 0;            ledPingYon =  1; }

    ledSonAnimZamani = millis();
  }

  uint8_t aktifParlaklik = map(ledParlakligiYuzde, 0, 100, 0, 31);

  uint8_t pr, pg, pb;
  hueSatAyir(ledHue, ledSat, pr, pg, pb);

  float fadeK = (sinf(ledFadeAcisi) + 1.0f) / 2.0f;
  fadeK = 0.10f + 0.90f * fadeK;   // never fully dark

  ledSerit.send32Bit(0, 0, 0, 0, 0);
  for (uint8_t i = 0; i < LED_ADET; i++) {
    uint8_t r = 0, g = 0, b = 0;
    switch (ledModu) {

      case LED_RAINBOW:
        ledRenkTekeri((i * 256 / LED_ADET + ledHueOffset) & 0xFF, r, g, b);
        break;

      case LED_SOLID:
        r = pr; g = pg; b = pb;
        break;

      case LED_FADE:
        r = (uint8_t)(pr * fadeK);
        g = (uint8_t)(pg * fadeK);
        b = (uint8_t)(pb * fadeK);
        break;

      case LED_PING: {
        float d = fabsf((float)i - ledPingPoz);
        float k = 1.0f - (d / 2.5f);   // two to three LED tail
        if (k < 0) k = 0;
        r = (uint8_t)(pr * k);
        g = (uint8_t)(pg * k);
        b = (uint8_t)(pb * k);
        break;
      }

      case LED_CHASE:
        if ((i + ledAnimStep / 30) % 3 == 0) { r = pr; g = pg; b = pb; }
        break;

      default:
        break;
    }
    ledSerit.send32Bit(0x07, aktifParlaklik, b, g, r);
  }
  ledCerceveyiKilitle();
}

// ------------------------------------------------------------ LIDAR ranging

void lidarVeriyiOku() {
  if (!lidarHazir) return;

  static unsigned long sonLidarOkuma = 0;
  if (millis() - sonLidarOkuma < 100) return;
  sonLidarOkuma = millis();

  if (!lidarImager.isDataReady()) return;
  if (!lidarImager.getRangingData(&lidarVeri)) return;

  // Keep the nearest zone with a valid target status (5, 6 or 9).
  int enYakin = 9999;
  for (int i = 0; i < 64; i++) {
    uint8_t durum = lidarVeri.target_status[i];
    if (durum == 5 || durum == 6 || durum == 9) {
      int m = lidarVeri.distance_mm[i];
      if (m > 0 && m < enYakin) enYakin = m;
    }
  }
  lidarEnYakinMesafe = (enYakin == 9999) ? 0 : enYakin;

  if (!LIDAR_SERI_YAZ) return;
  static unsigned long lidarSonYazZamani = 0;
  if (millis() - lidarSonYazZamani < 500) return;
  lidarSonYazZamani = millis();
  Serial.print(F("EN YAKIN: "));
  Serial.print(lidarEnYakinMesafe);
  Serial.println(F(" mm"));
}

void lidarTetikGuncelle() {
  if (!LIDAR_TETIK_KULLAN || !lidarHazir || !otomatikMod) return;

  bool gecerli = (lidarEnYakinMesafe > 0);

  if (gecerli && lidarEnYakinMesafe < lidarTetikMesafe) {
    lidarSonGormeZamani = millis();
    if (!lidarTetikAktif) {
      if (lidarOnaySayaci < 255) lidarOnaySayaci++;
      if (lidarOnaySayaci >= LIDAR_ONAY_SAYISI) {
        lidarTetikAktif = true;
        acilDurdurma = false;
        motorHedefBelirle(LIDAR_TETIK_HIZ);
        Serial.print(F("LIDAR tetik: "));
        Serial.print(lidarEnYakinMesafe);
        Serial.println(F(" mm"));
      }
    }
  } else {
    lidarOnaySayaci = 0;
    if (lidarTetikAktif) {
      bool uzaklasti = (!gecerli) || (lidarEnYakinMesafe > lidarBirakMesafe);
      if (uzaklasti && (millis() - lidarSonGormeZamani > LIDAR_BIRAK_GECIKME)) {
        lidarTetikAktif = false;
        motorHedefBelirle(0);
        Serial.println(F("LIDAR tetik birakildi"));
      }
    }
  }
}

// ----------------------------------------------------------- Motor control

void motorHedefBelirle(int yeniHedef) {
  if (yeniHedef < 0)   yeniHedef = 0;
  if (yeniHedef > 100) yeniHedef = 100;

  if (yeniHedef > 0) acilDurdurma = false;

  // Start-up assist only applies after the shaft has actually been stopped.
  if (yeniHedef > 0 && motorDurdu && !kalkisAktif &&
      (millis() - motorDurmaZamani) > KALKIS_MIN_DURUS) {
    kalkisAktif = true;
    kalkisBaslangic = millis();
  }
  motorHiziHedef = yeniHedef;
}

void motorAcilDurdur() {
  lidarTetikAktif = false;
  lidarOnaySayaci = 0;
  motorHiziHedef  = 0;
  kalkisAktif     = false;
  acilDurdurma    = true;
}

void spoolGuncelle() {
  unsigned long simdi = millis();
  if (simdi - sonSpoolZamani < 20) return;
  float dt = (simdi - sonSpoolZamani) / 1000.0f;
  if (dt > 0.2f) dt = 0.2f;
  sonSpoolZamani = simdi;

  float upHiz   = otomatikMod ? SPOOL_UP_HIZ : MANUEL_UP_HIZ;
  float downHiz = (acilDurdurma || !otomatikMod) ? MANUEL_DOWN_HIZ : SPOOL_DOWN_HIZ;

  if (motorHiziAnlik < motorHiziHedef) {
    motorHiziAnlik += upHiz * dt;
    if (motorHiziAnlik > motorHiziHedef) motorHiziAnlik = motorHiziHedef;
  } else if (motorHiziAnlik > motorHiziHedef) {
    motorHiziAnlik -= downHiz * dt;
    if (motorHiziAnlik < motorHiziHedef) motorHiziAnlik = motorHiziHedef;
  }

  if (motorHiziAnlik < 0.2f) {
    if (!motorDurdu) { motorDurdu = true; motorDurmaZamani = simdi; }
    acilDurdurma = false;
  } else {
    motorDurdu = false;
  }

  // Map throttle onto the usable PWM range above the dead zone.
  int pwmHedef;
  if (motorHiziAnlik < DURMA_ESIGI) {
    pwmHedef = 0;
  } else {
    pwmHedef = MOTOR_MIN_PWM +
               (int)((motorHiziAnlik / 100.0f) * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
    if (pwmHedef > MOTOR_MAX_PWM) pwmHedef = MOTOR_MAX_PWM;
  }

  if (kalkisAktif) {
    unsigned long gecen = simdi - kalkisBaslangic;
    if (gecen >= (unsigned long)KALKIS_SURESI || pwmHedef == 0) {
      kalkisAktif = false;
    } else {
      float kalan = 1.0f - ((float)gecen / (float)KALKIS_SURESI);
      pwmHedef += (int)(KALKIS_EK_PWM * kalan);
      if (pwmHedef > MOTOR_MAX_PWM) pwmHedef = MOTOR_MAX_PWM;
    }
  }

  int fark = pwmHedef - pwmAnlik;
  if      (fark >  PWM_SLEW) pwmAnlik += PWM_SLEW;
  else if (fark < -PWM_SLEW) pwmAnlik -= PWM_SLEW;
  else                       pwmAnlik  = pwmHedef;

  if (pwmHedef == 0 && pwmAnlik < (MOTOR_MIN_PWM - 40)) pwmAnlik = 0;
  if (pwmAnlik < 0) pwmAnlik = 0;

  motor.drive(pwmAnlik);

  motorRPM = (int)((motorHiziAnlik / 100.0f) * MAKS_RPM + 0.5f);
}

// Audio follows the motor: it fades in on spool-up and out on shutdown.
// The MUSIC page only selects which track plays; changing it mid-playback
// switches immediately.
void motorSesiGuncelle() {
  if (!mp3Hazir) return;
  if (millis() - sesSonIslem < 50) return;
  sesSonIslem = millis();

  bool motorCalisiyor = (pwmAnlik > 0) || (motorHiziHedef > 0);
  bool sesIstenen = motorCalisiyor && sesAcik;

  if (sesIstenen) {
    if (!sesCaliyor) {
      sesAnlikVolume = 0;
      mp3Player.volume(0);
      if (!mp3IlkCalma && calanParca == aktifParca) {
        mp3Player.start();   // resume the paused track
      } else {
        mp3ParcaCal(aktifParca);
        calanParca  = aktifParca;
        mp3IlkCalma = false;
      }
      sesCaliyor = true;
    } else if (calanParca != aktifParca) {
      mp3ParcaCal(aktifParca);
      calanParca = aktifParca;
    }

    if (sesAnlikVolume < mp3SesSeviyesi) {
      int v = sesAnlikVolume + SES_FADE_IN_ADIM;
      if (v > mp3SesSeviyesi) v = mp3SesSeviyesi;
      sesAnlikVolume = (uint8_t)v;
      mp3Player.volume(sesAnlikVolume);
    } else if (sesAnlikVolume > mp3SesSeviyesi) {
      sesAnlikVolume = mp3SesSeviyesi;   // user lowered the level mid-fade
      mp3Player.volume(sesAnlikVolume);
    }
  } else {
    if (sesCaliyor) {
      if (sesAnlikVolume > 0) {
        sesAnlikVolume = (sesAnlikVolume > SES_FADE_OUT_ADIM)
                       ? (uint8_t)(sesAnlikVolume - SES_FADE_OUT_ADIM) : 0;
        mp3Player.volume(sesAnlikVolume);
      } else {
        mp3Player.pause();
        sesCaliyor = false;
      }
    }
  }
}

// ---------------------------------------------------------- RTC (PCF8563)

static uint8_t bcd2dec(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

bool rtcRegOku(uint8_t reg, uint8_t* veri, uint8_t adet) {
  Wire.beginTransmission(PCF8563_ADRES);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t okunan = Wire.requestFrom((uint8_t)PCF8563_ADRES, (uint8_t)adet);
  if (okunan != adet) return false;
  for (uint8_t i = 0; i < adet; i++) veri[i] = Wire.read();
  return true;
}

bool rtcRegYaz(uint8_t reg, const uint8_t* veri, uint8_t adet) {
  Wire.beginTransmission(PCF8563_ADRES);
  Wire.write(reg);
  for (uint8_t i = 0; i < adet; i++) Wire.write(veri[i]);
  return (Wire.endTransmission() == 0);
}

bool rtcBaslat() {
  Wire.beginTransmission(PCF8563_ADRES);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("RTC (PCF8563) bulunamadi."));
    return false;
  }
  uint8_t kontrol[2] = { 0x00, 0x00 };
  rtcRegYaz(PCF8563_REG_KONTROL1, kontrol, 2);
  Serial.println(F("RTC (PCF8563) baglandi."));
  return true;
}

bool rtcSaatiOku(struct tm &zaman) {
  uint8_t d[7];
  if (!rtcRegOku(PCF8563_REG_SANIYE, d, 7)) return false;

  rtcGuvenilir = ((d[0] & 0x80) == 0);   // VL bit: clear means the time held

  zaman.tm_sec  = bcd2dec(d[0] & 0x7F);
  zaman.tm_min  = bcd2dec(d[1] & 0x7F);
  zaman.tm_hour = bcd2dec(d[2] & 0x3F);
  zaman.tm_mday = bcd2dec(d[3] & 0x3F);
  zaman.tm_wday = d[4] & 0x07;
  zaman.tm_mon  = bcd2dec(d[5] & 0x1F) - 1;
  zaman.tm_year = 100 + bcd2dec(d[6]);   // 2000..2099
  zaman.tm_isdst = 0;

  if (zaman.tm_mon < 0 || zaman.tm_mon > 11) return false;
  return rtcGuvenilir;
}

bool rtcSaatiYaz(const struct tm &zaman) {
  uint8_t d[7];
  d[0] = dec2bcd(zaman.tm_sec)  & 0x7F;   // clears VL
  d[1] = dec2bcd(zaman.tm_min)  & 0x7F;
  d[2] = dec2bcd(zaman.tm_hour) & 0x3F;
  d[3] = dec2bcd(zaman.tm_mday) & 0x3F;
  d[4] = zaman.tm_wday & 0x07;
  d[5] = dec2bcd(zaman.tm_mon + 1) & 0x1F;
  int yil = zaman.tm_year - 100;
  if (yil < 0)  yil = 0;
  if (yil > 99) yil = 99;
  d[6] = dec2bcd((uint8_t)yil);
  bool ok = rtcRegYaz(PCF8563_REG_SANIYE, d, 7);
  if (ok) rtcGuvenilir = true;
  return ok;
}

void sistemSaatiniAyarla(const struct tm &zaman) {
  struct tm kopya = zaman;
  time_t t = mktime(&kopya);
  if (t <= 0) return;
  struct timeval tv;
  tv.tv_sec  = t;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

void rtcGuncelle() {
  static unsigned long sonOkuma = 0;
  if (millis() - sonOkuma < 500) return;
  sonOkuma = millis();

  time_t t = time(nullptr);
  localtime_r(&t, &suankiZaman);
  zamanGecerli = (t > 1700000000);

  // Refresh the RTC from NTP-backed system time every ten minutes.
  static unsigned long sonRtcYazma = 0;
  if (rtcVar && ntpEsitlendi && zamanGecerli &&
      (sonRtcYazma == 0 || millis() - sonRtcYazma > 600000UL)) {
    sonRtcYazma = millis();
    rtcSaatiYaz(suankiZaman);
  }
}

void saatMetni(char* buf, size_t n, bool saniyeli) {
  if (!zamanGecerli) { snprintf(buf, n, saniyeli ? "--:--:--" : "--:--"); return; }
  if (saniyeli)
    snprintf(buf, n, "%02d:%02d:%02d", suankiZaman.tm_hour, suankiZaman.tm_min, suankiZaman.tm_sec);
  else
    snprintf(buf, n, "%02d:%02d", suankiZaman.tm_hour, suankiZaman.tm_min);
}

void tarihMetni(char* buf, size_t n) {
  if (!zamanGecerli) { snprintf(buf, n, "--.--.----"); return; }
  snprintf(buf, n, "%02d.%02d.%04d",
           suankiZaman.tm_mday, suankiZaman.tm_mon + 1, suankiZaman.tm_year + 1900);
}

// ------------------------------------------------------------------- WiFi

void wifiBaslat() {
  if (!KULLAN_WIFI) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if (wifiSSID.length() == 0) { wifiDurum = WD_KAPALI; return; }

  Serial.print(F("WiFi baglaniyor: "));
  Serial.println(wifiSSID);
  WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());
  wifiDurum = WD_BAGLANIYOR;
  wifiDenemeZamani = millis();
}

void ntpEsitle() {
  if (WiFi.status() != WL_CONNECTED) return;
  configTzTime(ZAMAN_DILIMI, NTP_SUNUCU_1, NTP_SUNUCU_2);
  Serial.println(F("NTP istegi gonderildi."));
}

void wifiGorevGuncelle() {
  if (!KULLAN_WIFI) return;

  static unsigned long sonKontrol = 0;
  if (millis() - sonKontrol < 400) return;
  sonKontrol = millis();

  wl_status_t st = WiFi.status();

  if (st == WL_CONNECTED) {
    if (wifiDurum != WD_BAGLI) {
      wifiDurum = WD_BAGLI;
      Serial.print(F("WiFi bagli. IP: "));
      Serial.println(WiFi.localIP());
      ntpEsitle();
    }
    if (!ntpEsitlendi) {
      time_t t = time(nullptr);
      if (t > 1700000000) {
        ntpEsitlendi = true;
        struct tm z;
        localtime_r(&t, &z);
        if (rtcVar) rtcSaatiYaz(z);
        Serial.println(F("Saat NTP ile esitlendi."));
      }
    }
  } else {
    if (wifiDurum == WD_BAGLANIYOR && millis() - wifiDenemeZamani > 15000) {
      wifiDurum = WD_HATA;
      Serial.println(F("WiFi baglanamadi."));
    }
    if (wifiDurum == WD_HATA && wifiSSID.length() > 0 &&
        millis() - wifiDenemeZamani > WIFI_YENIDEN_DENEME) {
      WiFi.disconnect();
      WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());
      wifiDurum = WD_BAGLANIYOR;
      wifiDenemeZamani = millis();
    }
  }

  if (taramaSuruyor) {
    int r = WiFi.scanComplete();
    if (r >= 0)       { taramaSonuc = r; taramaSuruyor = false; }
    else if (r == -2) { taramaSuruyor = false; taramaSonuc = -2; }
  }
}

void wifiTaramaBaslat() {
  if (!KULLAN_WIFI) return;
  WiFi.scanDelete();
  taramaSonuc = -1;
  taramaKaydirma = 0;
  taramaSuruyor = true;
  WiFi.scanNetworks(true, false);
}

void wifiUnut() {
  wifiSSID = "";
  wifiPASS = "";
  WiFi.disconnect(true);
  wifiDurum = WD_KAPALI;
  ntpEsitlendi = false;
  ayarIsaretle();
}

const char* wifiDurumMetni() {
  switch (wifiDurum) {
    case WD_BAGLI:      return "CONNECTED";
    case WD_BAGLANIYOR: return "CONNECTING...";
    case WD_HATA:       return "FAILED";
    default:            return "OFF";
  }
}

// -------------------------------------------------------- Settings storage

void ayarlariYukle() {
  ayarDepo.begin("jetmotor", true);
  wifiSSID           = ayarDepo.getString("ssid", "");
  wifiPASS           = ayarDepo.getString("pass", "");
  mp3SesSeviyesi     = ayarDepo.getUChar ("ses",  30);
  sesAcik            = ayarDepo.getUChar ("sesa",  1) != 0;
  aktifParca         = ayarDepo.getUChar ("parca", 1);
  ledParlakligiYuzde = ayarDepo.getUChar ("ledp", 75);
  ledModu            = ayarDepo.getUChar ("ledm", LED_RAINBOW);
  ledHue             = ayarDepo.getUChar ("ledh", 170);
  ledSat             = ayarDepo.getUChar ("leds", 255);
  lidarTetikMesafe   = ayarDepo.getUShort("ltet", 1500);
  lidarBirakMesafe   = ayarDepo.getUShort("lbir", 1700);
  ayarDepo.end();

  // Clamp stored values; the playlist or limits may have changed since.
  if (mp3SesSeviyesi > MP3_SES_MAKS) mp3SesSeviyesi = MP3_SES_MAKS;
  if (aktifParca < 1 || aktifParca > MUZIK_ADET) aktifParca = 1;
  if (ledModu >= LED_MOD_ADET)       ledModu = LED_RAINBOW;
  ledParlakligiYuzde = constrain(ledParlakligiYuzde, 0, 100);
  lidarTetikMesafe   = constrain(lidarTetikMesafe, LIDAR_MIN_AYAR, LIDAR_MAK_AYAR);
  lidarBirakMesafe   = constrain(lidarBirakMesafe, lidarTetikMesafe + 100, LIDAR_MAK_AYAR + 500);
}

void ayarlariKaydet() {
  ayarDepo.begin("jetmotor", false);
  ayarDepo.putString("ssid", wifiSSID);
  ayarDepo.putString("pass", wifiPASS);
  ayarDepo.putUChar ("ses",  mp3SesSeviyesi);
  ayarDepo.putUChar ("sesa", sesAcik ? 1 : 0);
  ayarDepo.putUChar ("parca", aktifParca);
  ayarDepo.putUChar ("ledp", (uint8_t)ledParlakligiYuzde);
  ayarDepo.putUChar ("ledm", ledModu);
  ayarDepo.putUChar ("ledh", ledHue);
  ayarDepo.putUChar ("leds", ledSat);
  ayarDepo.putUShort("ltet", (uint16_t)lidarTetikMesafe);
  ayarDepo.putUShort("lbir", (uint16_t)lidarBirakMesafe);
  ayarDepo.end();
  Serial.println(F("Ayarlar kaydedildi."));
}

void ayarIsaretle() {
  ayarDegisti = true;
  ayarDegisimZamani = millis();
}

void ayarKaydetGerekiyorsa() {
  if (!ayarDegisti) return;
  if (millis() - ayarDegisimZamani < AYAR_KAYIT_GECIKME) return;
  ayarDegisti = false;
  ayarlariKaydet();
}

// --------------------------------------------------------- DFPlayer events

void mp3DetayYazdir(uint8_t type, int value) {
  switch (type) {
    case TimeOut:              Serial.println(F("MP3: Zaman asimi!")); break;
    case WrongStack:           Serial.println(F("MP3: Hatali paket.")); break;
    case DFPlayerCardInserted: Serial.println(F("MP3: SD Kart takildi.")); break;
    case DFPlayerCardRemoved:  Serial.println(F("MP3: SD Kart cikarildi.")); break;
    case DFPlayerPlayFinished:
      // Loop the selected track, but only while the engine is running.
      if (mp3Hazir && sesCaliyor) { mp3ParcaCal(aktifParca); calanParca = aktifParca; }
      break;
    case DFPlayerError:
      Serial.print(F("MP3: Hata: "));
      switch (value) {
        case Busy:             Serial.println(F("Kart mesgul")); break;
        case Sleeping:         Serial.println(F("Uyku modunda")); break;
        case SerialWrongStack: Serial.println(F("Seri haberlesme hatasi")); break;
        case CheckSumNotMatch: Serial.println(F("Checksum eslesmedi")); break;
        case FileIndexOut:     Serial.println(F("Dosya indeksi disinda")); break;
        case FileMismatch:     Serial.println(F("Dosya bulunamadi")); break;
        default: break;
      }
      break;
    default: break;
  }
}

// --------------------------------------------------------------- Keyboard

// Control codes inside the layout table: \x01 switches layer, \x02 deletes.
const int KLV_SUTUN   = 10;
const int KLV_SUTUN_W = EKRAN_W / KLV_SUTUN;
const int KLV_Y0      = 124;
const int KLV_SATIR_H = 36;
const int KLV_SATIR4_Y = KLV_Y0 + 3 * KLV_SATIR_H;

const char* KLV_DUZEN[4][3] = {
  { "qwertyuiop", "asdfghjkl-", "\x01zxcvbnm.\x02" },
  { "QWERTYUIOP", "ASDFGHJKL_", "\x01ZXCVBNM,\x02" },
  { "1234567890", "!@#$%^&*()", "\x01-_=+;:'\"\x02" },
  { "[]{}<>/\\|~", "`?,.\"';:!@", "\x01#$%^&*=+\x02" }
};

struct KlvTus { int x, w; const char* etiket; uint8_t kod; };
const KlvTus KLV_SATIR4[6] = {
  {   0,  96, "123",   1 },
  {  96,  48, ",",     2 },
  { 144, 192, "SPACE", 3 },
  { 336,  48, ".",     4 },
  { 384,  48, "OK",    5 },
  { 432,  48, "X",     6 }
};

void klavyeAc(const String &ssid) {
  klavyeHedefSSID = ssid;
  klavyeMetin[0] = 0;
  klavyeUzunluk = 0;
  klavyeKatman = 0;
  aktifKatman = KATMAN_KLAVYE;
}

void klavyeKarakterEkle(char c) {
  if (klavyeUzunluk >= sizeof(klavyeMetin) - 1) return;
  klavyeMetin[klavyeUzunluk++] = c;
  klavyeMetin[klavyeUzunluk] = 0;
}

void klavyeSil() {
  if (klavyeUzunluk == 0) return;
  klavyeMetin[--klavyeUzunluk] = 0;
}

void klavyeOnayla() {
  wifiSSID = klavyeHedefSSID;
  wifiPASS = String(klavyeMetin);
  ayarlariKaydet();
  ayarDegisti = false;
  ntpEsitlendi = false;
  WiFi.disconnect();
  wifiBaslat();
  aktifKatman = KATMAN_WIFI;
}

void klavyeCiz() {
  ft800.clearScreencolorBackground(ZEMIN, CLR_COL | CLR_STN | CLR_TAG);

  char buf[80];
  snprintf(buf, sizeof(buf), "PASSWORD  -  %s", klavyeHedefSSID.c_str());
  yaz(12, 8, 26, TEXT_DIM, buf);

  cerceve(10, 30, 460, 34, CYAN);
  yaz(20, 39, 27, WHITE, klavyeMetin);
  int imlecX = 20 + klavyeUzunluk * fontGenislik(27);
  if (imlecX < 462 && (millis() / 500) % 2 == 0)
    cizgiDikey(imlecX, 36, 58, CYAN);

  snprintf(buf, sizeof(buf), "%d CHARACTERS", klavyeUzunluk);
  yaz(12, 72, 26, TEXT_DIM, buf);
  yazSag(468, 72, 26, TEXT_DIM,
         (klavyeKatman < 2) ? "123 = NUMBERS/SYMBOLS" : "ABC = LETTERS");
  yazSag(468, 96, 26, LINE_DIM,
         (klavyeKatman < 2) ? "SHFT = UPPER/LOWER" : ">> = MORE SYMBOLS");

  for (int r = 0; r <= 3; r++)
    cizgiYatay(0, EKRAN_W, KLV_Y0 + r * KLV_SATIR_H, LINE_DIM);
  for (int c = 0; c <= KLV_SUTUN; c++)
    cizgiDikey(c * KLV_SUTUN_W, KLV_Y0, KLV_Y0 + 3 * KLV_SATIR_H, LINE_DIM);

  char tek[2] = { 0, 0 };
  for (int r = 0; r < 3; r++) {
    const char* satir = KLV_DUZEN[klavyeKatman][r];
    for (int c = 0; c < KLV_SUTUN; c++) {
      char ch = satir[c];
      if (ch == 0) break;
      int cx = c * KLV_SUTUN_W + KLV_SUTUN_W / 2;
      int cy = KLV_Y0 + r * KLV_SATIR_H + 10;
      if (ch == '\x01')      yazOrta(cx, cy, 26, SARI, (klavyeKatman < 2) ? "SHFT" : ">>");
      else if (ch == '\x02') yazOrta(cx, cy, 26, ORANGE_RED, "DEL");
      else { tek[0] = ch; yazOrta(cx, cy, 27, WHITE, tek); }
    }
  }

  for (int i = 0; i < 6; i++) {
    const KlvTus &t = KLV_SATIR4[i];
    uint32_t renk = CYAN;
    const char* et = t.etiket;
    if (t.kod == 5) renk = NEON_GREEN;
    if (t.kod == 6) renk = ORANGE_RED;
    if (t.kod == 1) et = (klavyeKatman < 2) ? "123" : "ABC";
    cerceve(t.x, KLV_SATIR4_Y, t.w, KLV_SATIR_H, LINE_DIM);
    yazOrta(t.x + t.w / 2, KLV_SATIR4_Y + 10, 26, renk, et);
  }
}

void klavyeDokunma(int tx, int ty) {
  if (ty >= KLV_Y0 && ty < KLV_Y0 + 3 * KLV_SATIR_H) {
    int r = (ty - KLV_Y0) / KLV_SATIR_H;
    int c = tx / KLV_SUTUN_W;
    if (c < 0) c = 0;
    if (c >= KLV_SUTUN) c = KLV_SUTUN - 1;
    char ch = KLV_DUZEN[klavyeKatman][r][c];
    if (ch == 0) return;
    if (ch == '\x01')
      klavyeKatman = (klavyeKatman < 2) ? (klavyeKatman ^ 1) : (klavyeKatman == 2 ? 3 : 2);
    else if (ch == '\x02') klavyeSil();
    else                   klavyeKarakterEkle(ch);
    return;
  }

  if (ty >= KLV_SATIR4_Y && ty < KLV_SATIR4_Y + KLV_SATIR_H) {
    for (int i = 0; i < 6; i++) {
      const KlvTus &t = KLV_SATIR4[i];
      if (tx >= t.x && tx < t.x + t.w) {
        switch (t.kod) {
          case 1: klavyeKatman = (klavyeKatman < 2) ? 2 : 0; break;
          case 2: klavyeKarakterEkle(','); break;
          case 3: klavyeKarakterEkle(' '); break;
          case 4: klavyeKarakterEkle('.'); break;
          case 5: klavyeOnayla(); break;
          case 6: aktifKatman = KATMAN_WIFI; break;
        }
        return;
      }
    }
  }
}

// -------------------------------------------------------- Shared chrome

void ustBarCiz() {
  dolu(0, 0, EKRAN_W, UST_H, PANEL_ACIK);
  yaz(10, 4, 27, WHITE, SAYFA_ISIM[aktifSayfa]);

  char buf[24];
  tarihMetni(buf, sizeof(buf));
  yazOrta(EKRAN_W / 2 + 20, 4, 27, zamanGecerli ? TEXT_DIM : ARC_DIM, buf);

  saatMetni(buf, sizeof(buf), false);
  yazSag(472, 4, 27, zamanGecerli ? WHITE : ARC_DIM, buf);

  cizgiYatay(0, EKRAN_W, UST_H, LINE_DIM);
}

void sekmeCubuguCiz() {
  for (int i = 0; i < SAYFA_ADET; i++) {
    int x = i * SEKME_W + 3;
    int w = SEKME_W - 6;
    if (i == aktifSayfa) {
      dolu(x + 2, SEKME_Y + 2, w - 4, SEKME_H - 4, CYAN);
      cerceve(x, SEKME_Y, w, SEKME_H, CYAN, 2);
      yazOrta(x + w / 2, SEKME_Y + 10, 26, ZEMIN, SEKME_ISIM[i]);
    } else {
      cerceve(x, SEKME_Y, w, SEKME_H, ARC_DIM, 2);
      yazOrta(x + w / 2, SEKME_Y + 10, 26, TEXT_DIM, SEKME_ISIM[i]);
    }
  }
}

const char* motorDurumMetni(uint32_t &renk) {
  if ((int)motorHiziAnlik < motorHiziHedef)      { renk = ORANGE_RED; return "SPOOLING UP"; }
  if ((int)motorHiziAnlik > motorHiziHedef)      { renk = ORANGE_RED; return acilDurdurma ? "SHUTTING DOWN" : "SPOOLING DOWN"; }
  if (motorHiziHedef == 0) { renk = TEXT_DIM;   return otomatikMod ? "ARMED - WAITING" : "STOPPED"; }
  renk = NEON_GREEN;                             return "STEADY";
}

uint32_t motorRenk() {
  if ((int)motorHiziAnlik >= 90) return ORANGE_RED;
  if (motorHiziAnlik > 0.5f)     return CYAN;
  return TEXT_DIM;
}

// ------------------------------------------------------------ ENGINE page

struct Alan { int x, y, w, h; };

const int   GAUGE_CX = 98, GAUGE_CY = 124, GAUGE_R = 54;
const Alan  THR_ALAN = { 206,  64, 256, 24 };
const Alan  MOD_BTN  = { 196, 158, 132, 46 };
const Alan  STP_BTN  = { 340, 158, 132, 46 };

void sayfaMotorCiz() {
  char buf[48], rpmBuf[16], pctBuf[16], maxBuf[16];
  uint32_t durumRenk;
  const char* durum = motorDurumMetni(durumRenk);
  int pct = (int)motorHiziAnlik;

  snprintf(rpmBuf, sizeof(rpmBuf), "%d", motorRPM);
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", pct);
  snprintf(maxBuf, sizeof(maxBuf), "%d", MAKS_RPM);

  panelCiz(SOL_X, ICERIK_Y, SOL_W, ICERIK_ALT - ICERIK_Y, "SHAFT SPEED");
  gaugeCiz(GAUGE_CX, GAUGE_CY, GAUGE_R, pct, motorRenk(),
           rpmBuf, 31, "RPM", "0", maxBuf);

  panelCiz(SAG_X, ICERIK_Y, SAG_W, 66, "THROTTLE");
  yazSag(SAG_X + SAG_W - 8, ICERIK_Y + 3, 26, motorRenk(), pctBuf);
  surguCiz(THR_ALAN.x, THR_ALAN.y, THR_ALAN.w, THR_ALAN.h, pct,
           motorRenk(), motorHiziHedef);

  panelCiz(SAG_X, 106, SAG_W, 46, nullptr);
  yaz(SAG_X + 10, 112, 26, TEXT_DIM, "STATUS");
  yazSag(SAG_X + SAG_W - 10, 112, 26, durumRenk, durum);
  yaz(SAG_X + 10, 130, 26, TEXT_DIM, "PROXIMITY");
  if (lidarHazir && lidarEnYakinMesafe > 0)
    snprintf(buf, sizeof(buf), "%d mm", lidarEnYakinMesafe);
  else
    snprintf(buf, sizeof(buf), "---");
  yazSag(SAG_X + SAG_W - 10, 130, 26, lidarTetikAktif ? ORANGE_RED : WHITE, buf);

  blokButon(MOD_BTN.x, MOD_BTN.y, MOD_BTN.w, MOD_BTN.h,
            otomatikMod ? "AUTO" : "MANUAL",
            otomatikMod ? NEON_GREEN : PANEL_ACIK,
            otomatikMod ? ZEMIN : SARI,
            otomatikMod ? NEON_GREEN : SARI);
  blokButon(STP_BTN.x, STP_BTN.y, STP_BTN.w, STP_BTN.h, "STOP",
            (pwmAnlik > 0) ? ORANGE_RED : PANEL_BG,
            (pwmAnlik > 0) ? ZEMIN : ARC_DIM,
            (pwmAnlik > 0) ? ORANGE_RED : ARC_DIM);
}

void sayfaMotorDokunma(int tx, int ty, bool yeni) {
  // Touching the throttle hands control to the operator.
  if (icinde(dokunmaBasX, dokunmaBasY,
             THR_ALAN.x - 14, THR_ALAN.y - 18, THR_ALAN.w + 28, THR_ALAN.h + 36)) {
    kontrolYakaladi = true;
    otomatikMod = false;
    lidarTetikAktif = false;
    lidarOnaySayaci = 0;
    int x = constrain(tx, THR_ALAN.x, THR_ALAN.x + THR_ALAN.w);
    motorHedefBelirle(map(x, THR_ALAN.x, THR_ALAN.x + THR_ALAN.w, 0, 100));
    return;
  }
  if (!yeni) return;

  if (icinde(tx, ty, MOD_BTN.x, MOD_BTN.y, MOD_BTN.w, MOD_BTN.h)) {
    kontrolYakaladi = true;
    otomatikMod = !otomatikMod;
    lidarTetikAktif = false;
    lidarOnaySayaci = 0;
    motorAcilDurdur();
    return;
  }
  if (icinde(tx, ty, STP_BTN.x, STP_BTN.y, STP_BTN.w, STP_BTN.h)) {
    kontrolYakaladi = true;
    motorAcilDurdur();
    return;
  }
}

// ------------------------------------------------------------ LIGHTS page

const int CARK_CX = 98, CARK_CY = 122, CARK_R = 54, CARK_KAL = 14;

const int ISK_PAR_Y = 60, ISK_SAT_Y = 124, ISK_BAR_H = 22;
const int ISK_BAR_X = 206, ISK_BAR_W = 256;
const int ISK_MOD_Y = 162, ISK_MOD_H = 40, ISK_MOD_W = 51, ISK_MOD_GAP = 4;

const char* LED_MOD_KISA[LED_MOD_ADET] = { "RNBW", "SOLID", "FADE", "PING", "CHASE" };

void sayfaIsikCiz() {
  char buf[56];

  panelCiz(SOL_X, ICERIK_Y, SOL_W, ICERIK_ALT - ICERIK_Y, "COLOR");
  renkCarkiCiz(CARK_CX, CARK_CY, CARK_R, CARK_KAL, ledHue, ledSat);
  snprintf(buf, sizeof(buf), "#%06lX", (unsigned long)hueSatRenk(ledHue, ledSat));
  yazOrta(CARK_CX, 184, 26, TEXT_DIM, buf);

  panelCiz(SAG_X, ICERIK_Y, SAG_W, 58, "BRIGHTNESS");
  snprintf(buf, sizeof(buf), "%d%%", ledParlakligiYuzde);
  yazSag(SAG_X + SAG_W - 8, ICERIK_Y + 3, 26, NEON_GREEN, buf);
  surguCiz(ISK_BAR_X, ISK_PAR_Y, ISK_BAR_W, ISK_BAR_H, ledParlakligiYuzde, NEON_GREEN, -1);

  panelCiz(SAG_X, 98, SAG_W, 58, "SATURATION");
  snprintf(buf, sizeof(buf), "%d%%", (ledSat * 100) / 255);
  yazSag(SAG_X + SAG_W - 8, 101, 26, WHITE, buf);
  doygunlukBariCiz(ISK_BAR_X, ISK_SAT_Y, ISK_BAR_W, ISK_BAR_H, ledHue, ledSat);

  for (int i = 0; i < LED_MOD_ADET; i++) {
    int x = SAG_X + i * (ISK_MOD_W + ISK_MOD_GAP);
    bool a = (i == ledModu);
    blokButon(x, ISK_MOD_Y, ISK_MOD_W, ISK_MOD_H, LED_MOD_KISA[i],
              a ? NEON_GREEN : PANEL_BG, a ? ZEMIN : TEXT_DIM,
              a ? NEON_GREEN : ARC_DIM);
  }
}

void sayfaIsikDokunma(int tx, int ty, bool yeni) {
  if (icinde(dokunmaBasX, dokunmaBasY,
             ISK_BAR_X - 12, ISK_PAR_Y - 16, ISK_BAR_W + 24, ISK_BAR_H + 30)) {
    kontrolYakaladi = true;
    int x = constrain(tx, ISK_BAR_X, ISK_BAR_X + ISK_BAR_W);
    int v = map(x, ISK_BAR_X, ISK_BAR_X + ISK_BAR_W, 0, 100);
    if (v != ledParlakligiYuzde) { ledParlakligiYuzde = v; ayarIsaretle(); }
    return;
  }
  if (icinde(dokunmaBasX, dokunmaBasY,
             ISK_BAR_X - 12, ISK_SAT_Y - 14, ISK_BAR_W + 24, ISK_BAR_H + 28)) {
    kontrolYakaladi = true;
    int x = constrain(tx, ISK_BAR_X, ISK_BAR_X + ISK_BAR_W);
    uint8_t v = (uint8_t)map(x, ISK_BAR_X, ISK_BAR_X + ISK_BAR_W, 0, 255);
    if (v != ledSat) { ledSat = v; ayarIsaretle(); }
    return;
  }

  // Hit test the color ring: inside the outer radius, outside the preview.
  int dx = tx - CARK_CX, dy = ty - CARK_CY;
  int mes2 = dx * dx + dy * dy;
  int icSinir = CARK_R - CARK_KAL - 10;
  if (mes2 <= (CARK_R + 10) * (CARK_R + 10) && mes2 >= icSinir * icSinir) {
    kontrolYakaladi = true;
    float aci = atan2f((float)dx, (float)(-dy)) / DEG_TO_RAD;
    if (aci < 0) aci += 360.0f;
    uint8_t h = (uint8_t)((aci * 256.0f) / 360.0f);
    if (h != ledHue) {
      ledHue = h;
      if (ledModu == LED_RAINBOW) ledModu = LED_SOLID;   // rainbow ignores hue
      ayarIsaretle();
    }
    return;
  }
  if (!yeni) return;

  for (int i = 0; i < LED_MOD_ADET; i++) {
    int x = SAG_X + i * (ISK_MOD_W + ISK_MOD_GAP);
    if (icinde(tx, ty, x, ISK_MOD_Y, ISK_MOD_W, ISK_MOD_H)) {
      kontrolYakaladi = true;
      ledModu = (uint8_t)i;
      ayarIsaretle();
      return;
    }
  }
}

// ------------------------------------------------------------ SENSOR page

const int SNS_SKALA_X = 196, SNS_SKALA_Y = 128, SNS_SKALA_W = 276, SNS_SKALA_H = 24;
const int SNS_EKS_X = 368, SNS_ART_X = 420, SNS_BTN_Y = 52, SNS_BTN_W = 44, SNS_BTN_H = 38;
const int SNS_OTO_X = 196, SNS_OTO_Y = 166, SNS_OTO_W = 276, SNS_OTO_H = 38;

void sayfaSensorCiz() {
  char buf[56];

  panelCiz(SOL_X, ICERIK_Y, SOL_W, ICERIK_ALT - ICERIK_Y, "PROXIMITY");

  if (!lidarHazir) {
    yazOrta(SOL_X + SOL_W / 2, 110, 26, ORANGE_RED, "LIDAR NOT FOUND");
    return;
  }

  bool yakin = (lidarEnYakinMesafe > 0 && lidarEnYakinMesafe < lidarTetikMesafe);

  if (lidarEnYakinMesafe > 0) snprintf(buf, sizeof(buf), "%d", lidarEnYakinMesafe);
  else                        snprintf(buf, sizeof(buf), "---");
  yazOrta(SOL_X + SOL_W / 2, 68, 31, yakin ? ORANGE_RED : CYAN, buf);
  yazOrta(SOL_X + SOL_W / 2, 116, 26, TEXT_DIM, "mm");

  lamba(SOL_X + 16, 142, lidarTetikAktif, ORANGE_RED, "TRIGGER");
  lamba(SOL_X + 16, 166, otomatikMod,     NEON_GREEN, "ARMED");

  snprintf(buf, sizeof(buf), "REL %d", lidarBirakMesafe);
  yazSag(SOL_X + SOL_W - 10, 168, 26, ARC_DIM, buf);

  panelCiz(SAG_X, ICERIK_Y, SAG_W, 66, "TRIGGER AT");
  snprintf(buf, sizeof(buf), "%d mm", lidarTetikMesafe);
  yaz(SAG_X + 12, 58, 29, WHITE, buf);
  blokButon(SNS_EKS_X, SNS_BTN_Y, SNS_BTN_W, SNS_BTN_H, "-", PANEL_ACIK, CYAN, CYAN);
  blokButon(SNS_ART_X, SNS_BTN_Y, SNS_BTN_W, SNS_BTN_H, "+", PANEL_ACIK, CYAN, CYAN);

  // The scale spans twice the trigger distance, so the marker sits mid-bar.
  int skalaMaks = lidarTetikMesafe * 2;
  int pct = 0;
  if (lidarEnYakinMesafe > 0)
    pct = constrain((lidarEnYakinMesafe * 100) / skalaMaks, 0, 100);

  snprintf(buf, sizeof(buf), "RANGE  0 - %d mm", skalaMaks);
  yaz(SNS_SKALA_X, SNS_SKALA_Y - 18, 26, TEXT_DIM, buf);
  snprintf(buf, sizeof(buf), "MAX %d", LIDAR_MAK_AYAR);
  yazSag(SNS_SKALA_X + SNS_SKALA_W, SNS_SKALA_Y - 18, 26, ARC_DIM, buf);

  barCiz(SNS_SKALA_X, SNS_SKALA_Y, SNS_SKALA_W, SNS_SKALA_H, pct,
         yakin ? ORANGE_RED : CYAN);
  int mx = SNS_SKALA_X + (SNS_SKALA_W * lidarTetikMesafe) / skalaMaks;
  mx = constrain(mx, SNS_SKALA_X + 1, SNS_SKALA_X + SNS_SKALA_W - 1);
  ciz(mx, SNS_SKALA_Y - 6, mx, SNS_SKALA_Y + SNS_SKALA_H + 6, SARI, 2);

  blokButon(SNS_OTO_X, SNS_OTO_Y, SNS_OTO_W, SNS_OTO_H,
            otomatikMod ? "AUTO START: ON" : "AUTO START: OFF",
            otomatikMod ? NEON_GREEN : PANEL_BG,
            otomatikMod ? ZEMIN : TEXT_DIM,
            otomatikMod ? NEON_GREEN : ARC_DIM);
}

void lidarEsikDegistir(int delta) {
  lidarTetikMesafe = constrain(lidarTetikMesafe + delta, LIDAR_MIN_AYAR, LIDAR_MAK_AYAR);
  lidarBirakMesafe = lidarTetikMesafe + 200;
  ayarIsaretle();
}

void sayfaSensorDokunma(int tx, int ty, bool yeni) {
  if (!yeni) return;
  if (icinde(tx, ty, SNS_EKS_X, SNS_BTN_Y, SNS_BTN_W, SNS_BTN_H)) {
    kontrolYakaladi = true; lidarEsikDegistir(-LIDAR_ADIM); return;
  }
  if (icinde(tx, ty, SNS_ART_X, SNS_BTN_Y, SNS_BTN_W, SNS_BTN_H)) {
    kontrolYakaladi = true; lidarEsikDegistir(+LIDAR_ADIM); return;
  }
  if (icinde(tx, ty, SNS_OTO_X, SNS_OTO_Y, SNS_OTO_W, SNS_OTO_H)) {
    kontrolYakaladi = true;
    otomatikMod = !otomatikMod;
    motorAcilDurdur();
    return;
  }
}

// ------------------------------------------------------------- MUSIC page

const int MZK_LST_X = 202, MZK_LST_W = 216;
const int MZK_LST_Y = 40,  MZK_SATIR_H = 38;
const int MZK_SATIR_ADET = 4;
const int MZK_BTN_X = 424, MZK_BTN_W = 42, MZK_BTN_H = 70;
const int MZK_YUK_Y = 40,  MZK_ASA_Y = 118;

int muzikKaydirma = 0;

void muzikParcaSec(uint8_t p) {
  if (p < 1) p = 1;
  if (p > MUZIK_ADET) p = MUZIK_ADET;
  if (p == aktifParca) return;
  aktifParca = p;
  sesSonIslem = 0;   // skip the audio rate limit so the switch is immediate
  ayarIsaretle();
}

void sayfaMuzikCiz() {
  char buf[64];

  panelCiz(SOL_X, ICERIK_Y, SOL_W, ICERIK_ALT - ICERIK_Y, "NOW PLAYING");

  if (!mp3Hazir) {
    yazOrta(SOL_X + SOL_W / 2, 110, 26, ORANGE_RED, "NO MP3 MODULE");
  } else {
    snprintf(buf, sizeof(buf), "%02d", aktifParca);
    yazOrta(SOL_X + SOL_W / 2, 58, 31, sesCaliyor ? NEON_GREEN : TEXT_DIM, buf);
    yazOrta(SOL_X + SOL_W / 2, 96, 26, WHITE, MUZIK_ISIM[aktifParca - 1]);

    lamba(SOL_X + 12, 120, sesCaliyor, NEON_GREEN, sesCaliyor ? "PLAYING" : "IDLE");
    if (sesAcik) {
      snprintf(buf, sizeof(buf), "VOL %d", mp3SesSeviyesi);
      yazSag(SOL_X + SOL_W - 10, 118, 26, TEXT_DIM, buf);
    } else {
      yazSag(SOL_X + SOL_W - 10, 118, 26, ORANGE_RED, "MUTED");
    }

    panelCiz(SOL_X + 8, 148, SOL_W - 16, 46, nullptr);
    yazOrta(SOL_X + SOL_W / 2, 156, 26, ARC_DIM, "PLAYS WHILE THE");
    yazOrta(SOL_X + SOL_W / 2, 174, 26, ARC_DIM, "ENGINE IS RUNNING");
  }

  panelCiz(SAG_X, ICERIK_Y, SAG_W, ICERIK_ALT - ICERIK_Y, nullptr);

  int gosterilecek = min(MZK_SATIR_ADET, (int)MUZIK_ADET - muzikKaydirma);
  for (int i = 0; i < gosterilecek; i++) {
    int idx = muzikKaydirma + i;
    int y = MZK_LST_Y + i * MZK_SATIR_H;
    bool secili = ((idx + 1) == aktifParca);

    dolu(MZK_LST_X + 2, y + 2, MZK_LST_W - 4, MZK_SATIR_H - 8,
         secili ? PANEL_ACIK : PANEL_BG);
    cerceve(MZK_LST_X, y, MZK_LST_W, MZK_SATIR_H - 4,
            secili ? NEON_GREEN : ARC_DIM, 2);

    snprintf(buf, sizeof(buf), "%02d  %s", idx + 1, MUZIK_ISIM[idx]);
    yaz(MZK_LST_X + 10, y + 8, 26, secili ? NEON_GREEN : WHITE, buf);
    if (secili && sesCaliyor)
      yazSag(MZK_LST_X + MZK_LST_W - 10, y + 8, 26, NEON_GREEN, ">");
  }

  bool yukVar = (muzikKaydirma > 0);
  bool asaVar = (muzikKaydirma + MZK_SATIR_ADET < (int)MUZIK_ADET);
  blokButon(MZK_BTN_X, MZK_YUK_Y, MZK_BTN_W, MZK_BTN_H, "^",
            PANEL_ACIK, yukVar ? CYAN : ARC_DIM, yukVar ? CYAN : ARC_DIM);
  blokButon(MZK_BTN_X, MZK_ASA_Y, MZK_BTN_W, MZK_BTN_H, "v",
            PANEL_ACIK, asaVar ? CYAN : ARC_DIM, asaVar ? CYAN : ARC_DIM);
}

void sayfaMuzikDokunma(int tx, int ty, bool yeni) {
  if (!yeni) return;

  if (icinde(tx, ty, MZK_BTN_X, MZK_YUK_Y, MZK_BTN_W, MZK_BTN_H)) {
    kontrolYakaladi = true;
    muzikKaydirma = max(0, muzikKaydirma - MZK_SATIR_ADET);
    return;
  }
  if (icinde(tx, ty, MZK_BTN_X, MZK_ASA_Y, MZK_BTN_W, MZK_BTN_H)) {
    kontrolYakaladi = true;
    if (muzikKaydirma + MZK_SATIR_ADET < (int)MUZIK_ADET)
      muzikKaydirma += MZK_SATIR_ADET;
    return;
  }

  int gosterilecek = min(MZK_SATIR_ADET, (int)MUZIK_ADET - muzikKaydirma);
  for (int i = 0; i < gosterilecek; i++) {
    int y = MZK_LST_Y + i * MZK_SATIR_H;
    if (icinde(tx, ty, MZK_LST_X, y, MZK_LST_W, MZK_SATIR_H - 4)) {
      kontrolYakaladi = true;
      muzikParcaSec((uint8_t)(muzikKaydirma + i + 1));
      return;
    }
  }
}

// ------------------------------------------------------------- CLOCK page

const int SAAT_CX = 98, SAAT_CY = 119, SAAT_R = 76;

// Shared by the CLOCK page and the idle screen; sayfaModu hides the hint.
void saatIcerikCiz(bool sayfaModu) {
  char buf[56];

  panelCiz(SOL_X, ICERIK_Y, SOL_W, ICERIK_ALT - ICERIK_Y, nullptr);
  analogSaatCiz(SAAT_CX, SAAT_CY, SAAT_R);

  dolu(SAG_X + 2, ICERIK_Y + 2, SAG_W - 4, 52, PANEL_ACIK);
  cerceve(SAG_X, ICERIK_Y, SAG_W, 56, LINE_DIM, 2);
  saatMetni(buf, sizeof(buf), true);
  yazOrta(SAG_X + SAG_W / 2, ICERIK_Y + 14, 31, zamanGecerli ? CYAN : ARC_DIM, buf);

  panelCiz(SAG_X, 98, 134, 40, nullptr);
  tarihMetni(buf, sizeof(buf));
  yazOrta(SAG_X + 67, 110, 26, WHITE, buf);

  panelCiz(SAG_X + 142, 98, 134, 40, nullptr);
  if (zamanGecerli && suankiZaman.tm_wday >= 0 && suankiZaman.tm_wday < 7)
    yazOrta(SAG_X + 209, 110, 26, TEXT_DIM, GUN_ISIM[suankiZaman.tm_wday]);
  else
    yazOrta(SAG_X + 209, 110, 26, ARC_DIM, "---");

  panelCiz(SAG_X, 146, SAG_W, 58, nullptr);
  unsigned long up = millis() / 1000;
  snprintf(buf, sizeof(buf), "UPTIME  %02lu:%02lu:%02lu",
           (up / 3600) % 100, (up / 60) % 60, up % 60);
  yaz(SAG_X + 12, 154, 26, TEXT_DIM, buf);

  if (pwmAnlik > 0) {
    snprintf(buf, sizeof(buf), "ENGINE  %d RPM", motorRPM);
    yaz(SAG_X + 12, 176, 26, ORANGE_RED, buf);
  } else {
    yaz(SAG_X + 12, 176, 26, ARC_DIM, "ENGINE  IDLE");
  }

  if (!sayfaModu)
    yazOrta(SAG_X + SAG_W / 2, 222, 26, ARC_DIM, "TOUCH TO CONTINUE");
}

void sayfaSaatCiz()      { saatIcerikCiz(true); }
void ekranKoruyucuCiz()  { saatIcerikCiz(false); }

// ------------------------------------------------------------- SETUP page

const int AYR_BAR_X = 18,  AYR_BAR_Y = 62, AYR_BAR_W = 412, AYR_BAR_H = 24;
const int AYR_BTN_Y = 104, AYR_BTN_H = 44;
const int AYR_SES_BTN_X = 8,   AYR_SES_BTN_W = 138;
const int AYR_WIFI_BTN_X = 154, AYR_WIFI_BTN_W = 138;
const int AYR_NTP_BTN_X = 300,  AYR_NTP_BTN_W = 140;

void sayfaAyarCiz() {
  char buf[72];

  panelCiz(SOL_X, ICERIK_Y, 432, 58, "SPEAKER VOLUME");
  if (!mp3Hazir) snprintf(buf, sizeof(buf), "NO MP3 MODULE");
  else snprintf(buf, sizeof(buf), "%d / %d%s", mp3SesSeviyesi, MP3_SES_MAKS,
                (mp3SesSeviyesi >= MP3_SES_MAKS) ? "  MAX" : "");
  yazSag(432, ICERIK_Y + 3, 26, mp3Hazir ? NEON_GREEN : ORANGE_RED, buf);
  surguCiz(AYR_BAR_X, AYR_BAR_Y, AYR_BAR_W, AYR_BAR_H,
           (mp3SesSeviyesi * 100) / MP3_SES_MAKS, SARI, -1);

  blokButon(AYR_SES_BTN_X, AYR_BTN_Y, AYR_SES_BTN_W, AYR_BTN_H,
            sesAcik ? "SOUND ON" : "SOUND OFF",
            sesAcik ? NEON_GREEN : PANEL_BG,
            sesAcik ? ZEMIN : ORANGE_RED,
            sesAcik ? NEON_GREEN : ORANGE_RED);
  blokButon(AYR_WIFI_BTN_X, AYR_BTN_Y, AYR_WIFI_BTN_W, AYR_BTN_H,
            "WiFi SETUP", PANEL_ACIK, CYAN, CYAN);
  blokButon(AYR_NTP_BTN_X, AYR_BTN_Y, AYR_NTP_BTN_W, AYR_BTN_H, "SYNC CLOCK",
            (wifiDurum == WD_BAGLI) ? PANEL_ACIK : PANEL_BG,
            (wifiDurum == WD_BAGLI) ? NEON_GREEN : ARC_DIM,
            (wifiDurum == WD_BAGLI) ? NEON_GREEN : ARC_DIM);
}

void sayfaAyarDokunma(int tx, int ty, bool yeni) {
  if (icinde(dokunmaBasX, dokunmaBasY,
             AYR_BAR_X - 12, AYR_BAR_Y - 16, AYR_BAR_W + 24, AYR_BAR_H + 30)) {
    kontrolYakaladi = true;
    int x = constrain(tx, AYR_BAR_X, AYR_BAR_X + AYR_BAR_W);
    uint8_t v = (uint8_t)map(x, AYR_BAR_X, AYR_BAR_X + AYR_BAR_W, 0, MP3_SES_MAKS);
    if (v != mp3SesSeviyesi) { mp3SesSeviyesi = v; ayarIsaretle(); }
    return;
  }
  if (!yeni) return;

  if (icinde(tx, ty, AYR_SES_BTN_X, AYR_BTN_Y, AYR_SES_BTN_W, AYR_BTN_H)) {
    kontrolYakaladi = true; sesAcik = !sesAcik; ayarIsaretle(); return;
  }
  if (icinde(tx, ty, AYR_WIFI_BTN_X, AYR_BTN_Y, AYR_WIFI_BTN_W, AYR_BTN_H)) {
    kontrolYakaladi = true; aktifKatman = KATMAN_WIFI; wifiTaramaBaslat(); return;
  }
  if (icinde(tx, ty, AYR_NTP_BTN_X, AYR_BTN_Y, AYR_NTP_BTN_W, AYR_BTN_H)) {
    kontrolYakaladi = true;
    if (wifiDurum == WD_BAGLI) { ntpEsitlendi = false; ntpEsitle(); }
    return;
  }
}

// ------------------------------------------------------------- WiFi layer

const int WF_TARA_X = 20,  WF_TARA_Y = 40, WF_TARA_W = 100, WF_BTN_H = 28;
const int WF_UNUT_X = 128, WF_UNUT_W = 100;
const int WF_GERI_X = 372, WF_GERI_W = 88;
const int WF_LISTE_Y = 80, WF_SATIR_H = 32;
const int WF_YUK_X = 436, WF_ASA_X = 436;

void katmanWifiCiz() {
  ft800.clearScreencolorBackground(ZEMIN, CLR_COL | CLR_STN | CLR_TAG);
  char buf[96];

  yaz(14, 8, 27, CYAN, "WiFi SETUP");
  yazSag(466, 10, 26, TEXT_DIM, "REQUIRED FOR THE CLOCK");
  cizgiYatay(0, EKRAN_W, 32, LINE_DIM);

  buton(WF_TARA_X, WF_TARA_Y, WF_TARA_W, WF_BTN_H, CYAN,
        taramaSuruyor ? "SCANNING" : "SCAN", true);
  buton(WF_UNUT_X, WF_TARA_Y, WF_UNUT_W, WF_BTN_H,
        wifiSSID.length() ? ORANGE_RED : LINE_DIM, "FORGET", true);
  buton(WF_GERI_X, WF_TARA_Y, WF_GERI_W, WF_BTN_H, NEON_GREEN, "BACK", true);

  if (taramaSuruyor || taramaSonuc == -1) {
    yaz(20, WF_LISTE_Y + 20, 27, TEXT_DIM, "SCANNING NETWORKS...");
  } else if (taramaSonuc <= 0) {
    yaz(20, WF_LISTE_Y + 12, 27, TEXT_DIM, "NO NETWORKS LISTED");
    yaz(20, WF_LISTE_Y + 42, 26, LINE_DIM, "1) PRESS SCAN");
    yaz(20, WF_LISTE_Y + 62, 26, LINE_DIM, "2) PICK YOUR NETWORK FROM THE LIST");
    yaz(20, WF_LISTE_Y + 82, 26, LINE_DIM, "3) TYPE THE PASSWORD AND PRESS OK");
  } else {
    int gosterilecek = min(TARAMA_SATIR_ADET, taramaSonuc - taramaKaydirma);
    for (int i = 0; i < gosterilecek; i++) {
      int idx = taramaKaydirma + i;
      int y = WF_LISTE_Y + i * WF_SATIR_H;
      String s = WiFi.SSID(idx);
      if (s.length() == 0) s = "(hidden)";
      int rssi = WiFi.RSSI(idx);
      bool sifreli = (WiFi.encryptionType(idx) != WIFI_AUTH_OPEN);
      bool bagli = (s == wifiSSID);

      cerceve(16, y, 412, WF_SATIR_H - 4, bagli ? NEON_GREEN : LINE_DIM);
      snprintf(buf, sizeof(buf), "%s%s", sifreli ? "* " : "  ", s.c_str());
      yaz(26, y + 6, 26, bagli ? NEON_GREEN : WHITE, buf);

      const char* guc = (rssi > -60) ? "||||" : (rssi > -70) ? "|||"
                      : (rssi > -80) ? "||"   : "|";
      yazSag(420, y + 6, 26, (rssi > -70) ? NEON_GREEN : SARI, guc);
    }
    if (taramaKaydirma > 0)
      buton(WF_YUK_X, WF_LISTE_Y, 36, 40, CYAN, "^", true);
    if (taramaKaydirma + TARAMA_SATIR_ADET < taramaSonuc)
      buton(WF_ASA_X, WF_LISTE_Y + 48, 36, 40, CYAN, "v", true);
  }

  cizgiYatay(0, EKRAN_W, 216, LINE_DIM);
  if (wifiDurum == WD_BAGLI)
    snprintf(buf, sizeof(buf), "CONNECTED: %s   IP: %s",
             wifiSSID.c_str(), WiFi.localIP().toString().c_str());
  else if (wifiSSID.length())
    snprintf(buf, sizeof(buf), "%s: %s", wifiSSID.c_str(), wifiDurumMetni());
  else
    snprintf(buf, sizeof(buf), "NO NETWORK SELECTED YET");
  yaz(16, 224, 26, (wifiDurum == WD_BAGLI) ? NEON_GREEN : TEXT_DIM, buf);

  yaz(16, 246, 26, LINE_DIM, "* = secured network   /   clock is set automatically over NTP");
}

void katmanWifiDokunma(int tx, int ty, bool yeni) {
  if (!yeni) return;

  if (icinde(tx, ty, WF_TARA_X, WF_TARA_Y, WF_TARA_W, WF_BTN_H)) { wifiTaramaBaslat(); return; }
  if (icinde(tx, ty, WF_UNUT_X, WF_TARA_Y, WF_UNUT_W, WF_BTN_H)) { wifiUnut(); return; }
  if (icinde(tx, ty, WF_GERI_X, WF_TARA_Y, WF_GERI_W, WF_BTN_H)) { aktifKatman = KATMAN_YOK; return; }

  if (taramaSonuc > 0) {
    if (taramaKaydirma > 0 && icinde(tx, ty, WF_YUK_X, WF_LISTE_Y, 36, 40)) {
      taramaKaydirma = max(0, taramaKaydirma - TARAMA_SATIR_ADET); return;
    }
    if (taramaKaydirma + TARAMA_SATIR_ADET < taramaSonuc &&
        icinde(tx, ty, WF_ASA_X, WF_LISTE_Y + 48, 36, 40)) {
      taramaKaydirma += TARAMA_SATIR_ADET; return;
    }
    int gosterilecek = min(TARAMA_SATIR_ADET, taramaSonuc - taramaKaydirma);
    for (int i = 0; i < gosterilecek; i++) {
      int y = WF_LISTE_Y + i * WF_SATIR_H;
      if (icinde(tx, ty, 16, y, 412, WF_SATIR_H - 4)) {
        int idx = taramaKaydirma + i;
        String s = WiFi.SSID(idx);
        if (s.length() == 0) return;
        if (WiFi.encryptionType(idx) == WIFI_AUTH_OPEN) {
          wifiSSID = s; wifiPASS = "";
          ayarlariKaydet();
          ntpEsitlendi = false;
          WiFi.disconnect();
          wifiBaslat();
        } else {
          klavyeAc(s);
        }
        return;
      }
    }
  }
}

// ----------------------------------------------------------- Frame render

void ekranCiz() {
  ft800.startFrame();

  if (aktifKatman == KATMAN_KLAVYE) {
    klavyeCiz();
  } else if (aktifKatman == KATMAN_WIFI) {
    katmanWifiCiz();
  } else {
    ft800.clearScreencolorBackground(ZEMIN, CLR_COL | CLR_STN | CLR_TAG);
    if (saatModu) {
      ekranKoruyucuCiz();
    } else {
      ustBarCiz();
      switch (aktifSayfa) {
        case SAYFA_MOTOR:  sayfaMotorCiz();  break;
        case SAYFA_ISIK:   sayfaIsikCiz();   break;
        case SAYFA_SENSOR: sayfaSensorCiz(); break;
        case SAYFA_MUZIK:  sayfaMuzikCiz();  break;
        case SAYFA_SAAT:   sayfaSaatCiz();   break;
        case SAYFA_AYAR:   sayfaAyarCiz();   break;
      }
      sekmeCubuguCiz();
    }
  }

  ft800.endFrame();
  ft800.waitForCommandCompletion();
}

void sayfaDegistir(int yon) {
  int y = (int)aktifSayfa + yon;
  if (y < 0) y = SAYFA_ADET - 1;
  if (y >= SAYFA_ADET) y = 0;
  aktifSayfa = (uint8_t)y;
}

// ------------------------------------------------------------------ Setup

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("=== Jet Motor Kontrol Modulu v8 ==="));

  ayarlariYukle();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  ft800.begin(FT800_SCK, FT800_MISO, FT800_MOSI, FT800_CS);
  Serial.println(F("Ekran baslatildi."));

  setenv("TZ", ZAMAN_DILIMI, 1);
  tzset();
  if (KULLAN_RTC) {
    rtcVar = rtcBaslat();
    if (rtcVar) {
      struct tm z;
      memset(&z, 0, sizeof(z));
      if (rtcSaatiOku(z)) {
        sistemSaatiniAyarla(z);
        Serial.println(F("Saat RTC'den alindi."));
      } else {
        Serial.println(F("RTC saati gecersiz - NTP bekleniyor."));
      }
    }
  }

  if (KULLAN_MOTOR) {
    motor.begin();
    motor.drive(0);
    motorHiziAnlik = 0;
    pwmAnlik = 0;
    motorDurdu = true;
    motorDurmaZamani = millis();
    sonSpoolZamani = millis();
    if (LIDAR_TETIK_KULLAN) {
      motorHiziHedef = 0;
      Serial.println(F("Motor hazir - LIDAR tetigi bekleniyor."));
    } else {
      motorHedefBelirle(BASLANGIC_HIZ);
      Serial.println(F("Motor baslatildi."));
    }
    Serial.print(F("Motor: 6V 25mm reduktorlu, nominal "));
    Serial.print(MOTOR_NOMINAL_RPM);
    Serial.println(F(" RPM"));
    Serial.print(F("PWM tavani "));
    Serial.print(MOTOR_MAX_PWM);
    Serial.print(F("/"));
    Serial.print(MOTOR_PWM_TAM);
    Serial.print(F("  = %"));
    Serial.print(MOTOR_TAVAN_DUTY);
    Serial.print(F(" duty  ->  ekranda 0 - "));
    Serial.print(MAKS_RPM);
    Serial.println(F(" RPM"));
  } else {
    Serial.println(F("Motor kapali."));
  }

  if (KULLAN_LED) {
    ledSerit.begin();
    Serial.println(F("LED seridi baslatildi."));
  }

  if (KULLAN_LIDAR) {
    if (lidarImager.begin()) {
      lidarImager.setResolution(8 * 8);
#if LIDAR_UZUN_MENZIL
      // Autonomous mode with a long integration time and low frequency
      // keeps readings usable out to about 3.5 m.
      lidarImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS);
      lidarImager.setIntegrationTime(20);
      lidarImager.setSharpenerPercent(20);
      lidarImager.setRangingFrequency(5);
#else
      lidarImager.setRangingFrequency(10);
#endif
      lidarImager.startRanging();
      lidarHazir = true;
      Serial.print(F("LIDAR baglandi. Tetik "));
      Serial.print(lidarTetikMesafe);
      Serial.print(F(" mm  (maks ayar "));
      Serial.print(LIDAR_MAK_AYAR);
      Serial.println(F(" mm)"));
    } else {
      Serial.println(F("LIDAR bulunamadi."));
    }
  } else {
    Serial.println(F("LIDAR kapali."));
  }

  if (KULLAN_MP3) {
    Serial.println(F("MP3 baslatiliyor..."));
    mp3Serial.begin(9600, SERIAL_8N1, MP3_RX_PIN, MP3_TX_PIN);
    delay(1000);
    for (int i = 0; i < 3 && !mp3Hazir; i++) {
      if (mp3Player.begin(mp3Serial, true, false)) { mp3Hazir = true; break; }
      delay(500);
    }
    if (mp3Hazir) {
      Serial.print(F("DFPlayer baglandi. Ses: "));
      Serial.println(mp3SesSeviyesi);
      mp3Player.volume(0);   // stay silent until the motor spools up
      sesAnlikVolume = 0;
      sesCaliyor = false;
      mp3IlkCalma = true;
      calanParca = 0;
      Serial.print(F("Calma listesi: "));
      Serial.print(MUZIK_ADET);
      Serial.print(F(" parca, secili: "));
      Serial.println(aktifParca);
    } else {
      Serial.println(F("DFPlayer bulunamadi."));
    }
  } else {
    Serial.println(F("MP3 kapali."));
  }

  if (KULLAN_WIFI) {
    wifiBaslat();
    if (wifiSSID.length() == 0)
      Serial.println(F("WiFi agi kayitli degil. SETTINGS > WiFi SETUP."));
  }

  sonDokunusZamani = millis();
  Serial.println(F("Sistem hazir."));
}

// ------------------------------------------------------------------- Loop

void loop() {
  lidarVeriyiOku();
  lidarTetikGuncelle();
  spoolGuncelle();
  motorSesiGuncelle();

  if (mp3Hazir && mp3Player.available())
    mp3DetayYazdir(mp3Player.readType(), mp3Player.read());

  ledSeridiGuncelleVeCiz();
  wifiGorevGuncelle();
  rtcGuncelle();

  uint8_t touchCount = ft800.read_i2c_register(TOUCH_TD_STATUS);
  bool dokunmaVar = false;
  uint16_t touchX = 0, touchY = 0;

  if (touchCount > 0) {
    touchX = ft800.touchXPosition(first_x_touch);
    touchY = ft800.touchYPosition(first_y_touch);
    dokunmaVar = (touchX < EKRAN_W && touchY < EKRAN_H);
  }

  bool yeniDokunma = dokunmaVar && !oncekiDokunma;

  if (dokunmaVar) {
    sonDokunusZamani = millis();
    sonTouchX = touchX;
    sonTouchY = touchY;

    if (yeniDokunma) {
      dokunmaBasX = touchX;
      dokunmaBasY = touchY;
      dokunmaBasZamani = millis();
      kontrolYakaladi = false;
    }

    if (saatModu) {
      if (yeniDokunma) { saatModu = false; kontrolYakaladi = true; }
    }
    else if (aktifKatman == KATMAN_KLAVYE) {
      kontrolYakaladi = true;
      if (yeniDokunma) klavyeDokunma(touchX, touchY);
    }
    else if (aktifKatman == KATMAN_WIFI) {
      kontrolYakaladi = true;
      katmanWifiDokunma(touchX, touchY, yeniDokunma);
    }
    else if (touchY >= SEKME_Y) {
      kontrolYakaladi = true;
      if (yeniDokunma) {
        int i = touchX / SEKME_W;
        if (i >= 0 && i < SAYFA_ADET) aktifSayfa = (uint8_t)i;
      }
    }
    else if (touchY > UST_H) {
      switch (aktifSayfa) {
        case SAYFA_MOTOR:  sayfaMotorDokunma (touchX, touchY, yeniDokunma); break;
        case SAYFA_ISIK:   sayfaIsikDokunma  (touchX, touchY, yeniDokunma); break;
        case SAYFA_SENSOR: sayfaSensorDokunma(touchX, touchY, yeniDokunma); break;
        case SAYFA_MUZIK:  sayfaMuzikDokunma (touchX, touchY, yeniDokunma); break;
        case SAYFA_SAAT:   break;
        case SAYFA_AYAR:   sayfaAyarDokunma  (touchX, touchY, yeniDokunma); break;
      }
    }
  }

  // Swipe is evaluated on release, and only if no control took the gesture.
  if (!dokunmaVar && oncekiDokunma) {
    if (!kontrolYakaladi && aktifKatman == KATMAN_YOK && !saatModu) {
      int dx = sonTouchX - dokunmaBasX;
      int dy = sonTouchY - dokunmaBasY;
      unsigned long sure = millis() - dokunmaBasZamani;
      if (abs(dy) < JEST_MAKS_DY && sure < JEST_MAKS_SURE) {
        if (dx <= -JEST_MIN_DX)     sayfaDegistir(+1);
        else if (dx >= JEST_MIN_DX) sayfaDegistir(-1);
      }
    }
  }
  oncekiDokunma = dokunmaVar;

  if (aktifKatman == KATMAN_YOK && millis() - sonDokunusZamani > beklemeSuresi)
    saatModu = true;

  ekranCiz();
  ayarKaydetGerekiyorsa();
}
