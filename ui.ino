#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include "ui.h"
#include <Adafruit_XCA9554.h>
#include "HWCDC.h"

// --- TAMBAHAN SENSOR FLOW & PID ---
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include "PLIC_Utils.h"

// --- TAMBAHAN SENSOR IMU (QMI8658) ---
#include "SensorQMI8658.hpp"

// --- TAMBAHAN SD CARD ---
#include <FS.h>
#include <SD_MMC.h>
#include <RTClib.h>

#include "BleManager.h"
BleManager ble;

// --- AUDIO CONFIG (Persis dari 15_ES8311.ino) ---
#include "ESP_I2S.h"
#include "es8311.h"

// Konfigurasi Suara
#define EXAMPLE_SAMPLE_RATE 48000
#define EXAMPLE_VOICE_VOLUME 90
#define EXAMPLE_MIC_GAIN (es8311_mic_gain_t)(3)
#define I2C_NUM 0  // Menggunakan Port I2C 0 (Sama dengan Wire)

// Konfigurasi Pin (Sama Persis)
#define I2S_MCK_IO 16
#define I2S_BCK_IO 9
#define I2S_DI_IO  10
#define I2S_WS_IO  45
#define I2S_DO_IO  8
#define PA_PIN     46 // Pin Amplifier

I2SClass i2s;
uint8_t *g_audioBuffer = NULL;
size_t g_audioSize = 0;
volatile bool g_playAlarm = false;

File transferFile;
bool isTransferring = false; // Flag status transfer file

bool btn3Hooked = false;
RTC_DS3231 rtc; // Membuat objek RTC
char timeBuf[16]; // Buffer untuk string jam
volatile int32_t g_touchX = 0;
volatile int32_t g_touchY = 0;
volatile bool g_isTouched = false;
volatile int g_curYear = 2025;
volatile int g_curMonth = 1;
volatile int g_curDay = 1;
volatile int g_curHour = 0;
volatile int g_curMinute = 0;
volatile int g_curSecond = 0;
volatile bool g_isEditingTime = false;
// --- VARIABEL GLOBAL JEMBATAN (BRIDGE) ---
// Core 0 tulis kesini, Core 1 baca dari sini
static DateTime lastHwTime; 
static bool firstRun = true;
bool g_isCountingDown = false;
volatile bool g_reqUpdateRollers = false;
volatile float g_flowLPM = 0.0f;
volatile int g_batPercent = 0;
volatile bool g_imuReady = false;
volatile float g_accX = 0, g_accY = 0, g_accZ = 0;
// Untuk String/Text, kita pakai array char biar aman (bukan String object)
char g_rtcTimeBuf[10] = "00:00";
unsigned long autoStartTime = 0;

volatile float g_statMin = 9999.0f; // Set angka sangat besar agar mudah tertimpa
volatile float g_statMax = -9999.0f; // Set angka sangat kecil

volatile bool g_reqSetTime = false; // Flag permintaan set waktu
volatile int g_setH = 0;            // Penampung Jam Baru
volatile int g_setM = 0;            // Penampung Menit Baru
volatile int g_setS = 0;            // Penampung Detik Baru
volatile bool g_hardwareAlarm = false;
unsigned long lastInteractionTime = 0;
volatile bool g_sdReady = false;
char g_logFileName[64] = "";

HWCDC USBSerial;
Adafruit_XCA9554 expander;
TaskHandle_t TaskSensorHandle;
// ================= SETUP SENSOR FLOW =================
Adafruit_ADS1115 ads;
PLIC_Utils plic;
Preferences pref;

const float VOUT_MIN = 0.5f;
const float VOUT_MAX = 2.5f;
const float FLOW_MAX_LPM = 1.812f;
const int AVG_N = 32;
float vbuf[AVG_N];
uint8_t vidx = 0;
uint8_t vcount = 0;

bool useCalibration = true;
const char* NVS_NS = "flowcal";
const char* NVS_KEYT = "tbl";
float g_latestFlowVolt = 0.0f;
unsigned long lastFlowReadTime = 0;

// ================= SETUP MOTOR & PID =================
const int FAN_PWM_PIN = 17;
int minPWM = 1;

bool isAutoMode = false;
float currentPwm = 0.0f;
float targetFlow = 0.0f;

// Tuning PID
float Kp = 50.0;
float Ki = 20.0;
float integralErr = 0.0f;

// ================= SETUP IMU (QMI8658) =================
SensorQMI8658 qmi;
IMUdata acc;
unsigned long lastImuReadTime = 0;

// ================= LVGL SETUP =================
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
static const uint16_t screenWidth = 368;
static const uint16_t screenHeight = 448;
static lv_disp_draw_buf_t draw_buf;
lv_color_t *buf1 = NULL;
lv_color_t *buf2 = NULL;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_SH8601 *gfx = new Arduino_SH8601(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

#if LV_USE_LOG != 0
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/* Read the touchpad (VERSI RINGAN - TANPA I2C) */
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  // Kita hanya baca status dari variabel global yang diupdate oleh Core 0
  if (g_isTouched) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = g_touchX;
    data->point.y = g_touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// --- FUNGSI KALIBRASI FLOW ---
void calLoadNVS() {
  pref.begin(NVS_NS, true);
  String buf = pref.getString(NVS_KEYT, "");
  plic.remove_table();
  int start = 0;
  while (start < buf.length()) {
    int nl = buf.indexOf('\n', start);
    String line = buf.substring(start, (nl < 0) ? buf.length() : nl);
    line.trim();
    if (line.length()) {
      int c = line.indexOf(',');
      if (c > 0) {
        plic.add_data(line.substring(0, c).toFloat(), line.substring(c + 1).toFloat(), true);
      }
    }
    if (nl < 0) break; else start = nl + 1;
  }
  pref.end();
}

void calSaveNVS() {
  pref.begin(NVS_NS, false);
  String buf;
  size_t n = plic.get_row_count();
  for (size_t i = 0; i < n; ++i) {
    buf += String(plic.get_x(i), 6);
    buf += ',';
    buf += String(plic.get_y(i), 6); buf += '\n';
  }
  pref.putString(NVS_KEYT, buf);
  pref.end();
  USBSerial.println("KALIBRASI: Tabel tersimpan!");
}

void calClearNVS() {
  pref.begin(NVS_NS, false);
  pref.remove(NVS_KEYT);
  pref.end();
  plic.remove_table();
  USBSerial.println("KALIBRASI: Reset!");
}

float getFlowLPM(float voltage) {
  float lpm;
  float defaultLPM = 0.0f;
  if (voltage > VOUT_MIN) {
     defaultLPM = ((voltage - VOUT_MIN) / (VOUT_MAX - VOUT_MIN)) * FLOW_MAX_LPM;
     if (defaultLPM > FLOW_MAX_LPM) defaultLPM = FLOW_MAX_LPM;
  }
  if (useCalibration) {
    float y = plic.interpolate(voltage);
    lpm = isnan(y) ? defaultLPM : y;
  } else {
    lpm = defaultLPM;
  }
  return lpm;
}

// Fungsi Start System (Dipanggil Tombol atau Bluetooth)
void startSystem() {
    if (isAutoMode) return; // Jika sudah jalan, abaikan

    if (TaskSensorHandle != NULL) {
        vTaskSuspend(TaskSensorHandle); 
    }

    DateTime now = DateTime(g_curYear, g_curMonth, g_curDay, g_curHour, g_curMinute, g_curSecond);
    USBSerial.printf("DEBUG RTC: Bln=%d, Tgl=%d\n", now.month(), now.day());
    snprintf(g_logFileName, sizeof(g_logFileName), "/%04d%02d%02d_%02d%02d%02d.csv", 
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
             
    // Buat File & Tulis Header
    if (g_sdReady) {
        File file = SD_MMC.open(g_logFileName, FILE_WRITE);
        if (file) {
            file.println("Date,Time,Flow_LPM,SetPoint,Motor_Percent,Acc_X,Acc_Y,Acc_Z");
            file.close();
            USBSerial.printf("LOG STARTED: %s\n", g_logFileName);
        } else {
            USBSerial.println("ERR: Gagal membuat file log!");
        }
    }

    // Baca Roller untuk Target Flow
    if (ui_Roller1 != NULL) { // Pastikan UI sudah load
        int digit1 = lv_roller_get_selected(ui_Roller1);
        int digit2 = lv_roller_get_selected(ui_Roller2); 
        int digit3 = lv_roller_get_selected(ui_Roller3); 
        targetFlow = (float)digit1 + ((float)digit2 * 0.1f) + ((float)digit3 * 0.01f);
    }

    integralErr = 0.0f;
    isAutoMode = true;
    autoStartTime = millis();
    
    // Update UI
    if (ui_Spinner1 != NULL) lv_obj_clear_flag(ui_Spinner1, LV_OBJ_FLAG_HIDDEN);
    if (ui_Button1 != NULL) lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    if (ui_Label3 != NULL) {
        lv_label_set_text(ui_Label3, "Stop");
        lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if (TaskSensorHandle != NULL) {
        vTaskResume(TaskSensorHandle);
    }

    USBSerial.printf("SYSTEM STARTED: Target %.2f LPM\n", targetFlow);
}

// Fungsi Stop System
void stopSystem() {
    if (!isAutoMode) return;

    isAutoMode = false;
    integralErr = 0.0f;
    
    // --- TAMBAHKAN BAGIAN INI (FIX AUDIO) ---
    g_hardwareAlarm = false; // Matikan pemicu alarm dari sensor
    g_playAlarm = false;     // Matikan loop pemutaran audio segera
    
    // Matikan juga amplifier fisik jika perlu (opsional, tapi disarankan)
    digitalWrite(PA_PIN, LOW); 
    // ----------------------------------------

    // Update UI
    if (ui_Spinner1 != NULL) lv_obj_add_flag(ui_Spinner1, LV_OBJ_FLAG_HIDDEN);
    if (ui_Button1 != NULL) lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xCDFF41), LV_PART_MAIN | LV_STATE_DEFAULT);
    if (ui_Label3 != NULL) {
        lv_label_set_text(ui_Label3, "Start");
        lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    
    // Jika panel warning (ui_Panel3) sedang berkedip, sembunyikan juga di sini
    if (ui_Panel3 != NULL) lv_obj_add_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);

    USBSerial.println("SYSTEM STOPPED");
}

void on_btn_set_date(lv_event_t * e) {
    // 1. Ambil indeks pilihan (Pilihan pertama selalu 0)
    int yearIdx  = lv_dropdown_get_selected(ui_Dropdown8);
    int monthIdx = lv_dropdown_get_selected(ui_Dropdown4);
    int dayIdx   = lv_dropdown_get_selected(ui_Dropdown6);

    // 2. Konversi ke angka kalender
    // Pastikan Tahun mulai dari 2025 (sesuai isi dropdown Anda)
    int finalYear  = 2025 + yearIdx; 
    int finalMonth = monthIdx + 1; // Index 0 (Jan) -> jadi 1
    int finalDay   = dayIdx + 1;   // Index 0 (Tgl 1) -> jadi 1

    // 3. Ambil jam saat ini agar tidak berubah
    DateTime now = rtc.now();

    // 4. Update RTC
    rtc.adjust(DateTime(finalYear, finalMonth, finalDay, now.hour(), now.minute(), now.second()));

    // DEBUG: Cek di Serial Monitor apakah angka sudah sesuai input
    USBSerial.printf("LOG SET: Thn:%d Bln:%d Tgl:%d | Jam:%02d:%02d\n", 
                     finalYear, finalMonth, finalDay, now.hour(), now.minute());
}

void on_btn_auto_start(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    static uint32_t press_start_time = 0;

    // 1. JIKA SISTEM SEDANG JALAN (RECORDING)
    if (isAutoMode) {
        // Hanya trigger STOP jika tombol BARU ditekan (bukan sisa dari countdown)
        if (code == LV_EVENT_CLICKED) {
            stopSystem();
        }
        return;
    }

    // 2. JIKA SISTEM BERHENTI (LOGIKA LONG PRESS 3 DETIK)
    if (code == LV_EVENT_PRESSED) {
        press_start_time = millis();
        g_isCountingDown = true; 
        
        if (ui_Panel3 != NULL) lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);
        if (ui_Label22 != NULL) {
            lv_obj_set_style_text_font(ui_Label22, &ui_font_MAIN, 0); 
            lv_label_set_text(ui_Label22, "3");
        }
    } 
    else if (code == LV_EVENT_PRESSING && g_isCountingDown) {
        uint32_t duration = millis() - press_start_time;
        
        if (duration >= 3000) {
            g_isCountingDown = false; // Matikan flag sebelum start agar tidak tertangkap event release
            if (ui_Panel3 != NULL) lv_obj_add_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);
            
            startSystem(); // Memulai record [cite: 45, 57]
            
            // PAKSA LVGL menganggap input ini sudah selesai agar tidak men-trigger event selanjutnya
            lv_indev_t * indev = lv_indev_get_act();
            if(indev) lv_indev_wait_release(indev); 
        } else {
            int countdown = 3 - (duration / 1000);
            if (ui_Label22 != NULL) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%d", countdown);
                lv_label_set_text(ui_Label22, buf);
            }
        }
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (g_isCountingDown) {
            g_isCountingDown = false; 
            if (ui_Panel3 != NULL) lv_obj_add_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void on_arc_value_changed(lv_event_t * e) {
    isAutoMode = false;
    integralErr = 0.0f; 
    
        lv_obj_add_flag(ui_Spinner1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xCDFF41), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(ui_Label3, "Start");
        lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * arc = lv_event_get_target(e);
    int32_t value = lv_arc_get_value(arc); 
    int pwmValue = 0;
    if (value <= 0) {
        pwmValue = 0;
    } else {
        pwmValue = map(value, 1, 100, minPWM, 255);
    }
    currentPwm = (float)pwmValue;
    ledcWrite(FAN_PWM_PIN, pwmValue); 
}

void processSerialInput() {
  if (USBSerial.available()) {
    String cmd = USBSerial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // --- FITUR BARU: RESET STATISTIK ---
    if (cmd.equalsIgnoreCase("reset")) {
        g_statMin = 9999.0f;
        g_statMax = -9999.0f;
        USBSerial.println(">>> STATISTIK DI-RESET! <<<");
    }
    // -----------------------------------
    else if (cmd.startsWith("cal ")) {
      float realFlow = cmd.substring(4).toFloat();
      uint8_t res = plic.add_data(g_latestFlowVolt, realFlow, true);
      USBSerial.printf("CAL: %.4f V -> %.3f LPM (Res:%d)\n", g_latestFlowVolt, realFlow, res);
      calSaveNVS();
    }
    else if (cmd.equalsIgnoreCase("table")) plic.print_table();
    else if (cmd.equalsIgnoreCase("clear")) calClearNVS();
  }
}

void logToSD() {
    // Cek SD Ready, Auto Mode Aktif, dan Nama File tidak kosong
    if (isTransferring) return;
    if (!g_sdReady || !isAutoMode || strlen(g_logFileName) == 0) return;

    DateTime now = rtc.now();
    float motorPercent = (currentPwm / 255.0f) * 100.0f;

    // Siapkan Data
    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf), "%04d-%02d-%02d,%02d:%02d:%02d,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f\n",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second(),
             g_flowLPM, targetFlow, motorPercent,
             g_accX, g_accY, g_accZ);

    // Tulis ke File (Gunakan variabel global g_logFileName)
    File file = SD_MMC.open(g_logFileName, FILE_APPEND);
    if (file) {
        file.print(logBuf);
        file.close();
    }
}

void TaskSensorCode(void * pvParameters) {
  unsigned long lastTouchReadTime = 0;
  unsigned long lastFlowReadTime = 0;
  unsigned long lastImuReadTime = 0;
  unsigned long lastRtcReadTime = 0;
  unsigned long lastLogTime = 0;

  for(;;) {
      processSerialInput();

      // --- 1. TOUCHSCREEN ---
      if (millis() - lastTouchReadTime > 20) {
          lastTouchReadTime = millis();
          int32_t tx = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
          int32_t ty = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
          
          if (FT3168->IIC_Interrupt_Flag) {
              FT3168->IIC_Interrupt_Flag = false;
              g_touchX = tx; 
              g_touchY = ty;
              g_isTouched = true;
          } else {
              g_isTouched = false;
          }
      }

      // --- 2. LOGIKA SET WAKTU (PERBAIKAN DISINI) ---
      if (g_reqSetTime) {
          g_reqSetTime = false; 

          DateTime now = rtc.now(); 
          // Buat objek waktu baru sesuai input user
          DateTime newTime = DateTime(now.year(), now.month(), now.day(), g_setH, g_setM, g_setS);
          
          // A. Update Hardware RTC
          rtc.adjust(newTime);
          
          // B. Update Buffer Software (PENTING! Agar tidak lompat ke waktu lama)
          lastHwTime = newTime; 
          
          // C. Update String Buffer (Tampilkan Detik juga!)
          snprintf(g_rtcTimeBuf, sizeof(g_rtcTimeBuf), "%02d:%02d:%02d", g_setH, g_setM, g_setS);
          
          // D. Reset Timer
          lastRtcReadTime = millis(); 
          
          // Update variabel global UI agar sinkron
          g_curHour   = g_setH;
          g_curMinute = g_setM;
          g_curSecond = g_setS;

          USBSerial.printf("RTC SET SUCCESS: %02d:%02d:%02d\n", g_setH, g_setM, g_setS);
      }

      // --- 3. FLOW SENSOR & PID ---
      if (millis() - lastFlowReadTime > 20) {
          lastFlowReadTime = millis();
          float dt = 0.1f;
          int16_t adc0 = ads.readADC_SingleEnded(0);
          float voltFlow = ads.computeVolts(adc0);
          
          vbuf[vidx] = voltFlow;
          vidx = (vidx + 1) % AVG_N;
          if (vcount < AVG_N) vcount++;
          float sum = 0.0f; 
          for(uint8_t i=0; i<vcount; i++) sum += vbuf[i];
          float avgVoltFlow = (vcount > 0) ? (sum / vcount) : 0.0f;
          g_latestFlowVolt = avgVoltFlow;

          float flowLPM = getFlowLPM(avgVoltFlow);
          g_flowLPM = flowLPM; // Update Global

          if (flowLPM > 0.1f) {
              if (flowLPM < g_statMin) g_statMin = flowLPM;
              if (flowLPM > g_statMax) g_statMax = flowLPM;
          }

          // ==========================================================
          // 2. KIRIM DATA KE SERIAL PLOTTER (Setiap 100ms)
          // ==========================================================
          // PENGAMAN: Jika baru di-reset (nilai 9999), paksa Min/Max = Flow saat ini
          // tujuannya agar grafik tidak "zoom out" kejauhan.
          float plotMin = (g_statMin > 5000.0f) ? flowLPM : g_statMin;
          float plotMax = (g_statMax < -5000.0f) ? flowLPM : g_statMax;
          float plotGap = plotMax - plotMin;

          // FORMAT SERIAL PLOTTER: "Label:Angka,Label:Angka,..."
          // USBSerial.print("Target:"); 
          // USBSerial.print(isAutoMode ? targetFlow : 0);
          // USBSerial.print(",");
          
          // USBSerial.print("Flow:");   
          // USBSerial.print(flowLPM);
          // USBSerial.print(",");
          
          // USBSerial.print("Min:");    
          // USBSerial.print(plotMin);
          // USBSerial.print(",");
          
          // USBSerial.print("Max:");    
          // USBSerial.print(plotMax);
          // USBSerial.print(",");
          
          // USBSerial.print("Gap:");    
          // USBSerial.println(plotGap); // println di akhir

          // PID Logic
            if (isAutoMode) {
              float error = targetFlow - flowLPM;
              integralErr += (error * dt);
              if (integralErr > 100.0f) integralErr = 100.0f;
              if (integralErr < -100.0f) integralErr = -100.0f;
              float pidOutput = (Kp * error) + (Ki * integralErr);

              if (pidOutput <= 0) currentPwm = 0;
              else {
                  currentPwm = pidOutput;
                  if (currentPwm > 10.0f && currentPwm < minPWM) currentPwm = minPWM;
              }
              if (currentPwm > 255.0f) currentPwm = 255.0f;
              if (currentPwm < 0.0f) currentPwm = 0.0f;

              ledcWrite(FAN_PWM_PIN, (int)currentPwm);

              // ==========================================================
              // FITUR BARU: DETEKSI KEGAGALAN HARDWARE (ALARM)
              // ==========================================================
              static unsigned long failureStartTime = 0;
              
              // Cek Kondisi: PWM Mentok MAX (255) TAPI Flow masih kurang dari Target (selisih > 0.3 LPM)
              if (currentPwm >= 255.0f && flowLPM < (targetFlow - 0.3f)) {
                  
                  // Jika ini baru pertama kali terdeteksi, catat waktunya
                  if (failureStartTime == 0) {
                      failureStartTime = millis();
                  } 
                  // Jika kondisi ini bertahan lebih dari 4000ms (4 detik)
                  else if (millis() - failureStartTime > 4000) {
                      g_hardwareAlarm = true; // AKTIFKAN ALARM
                  }
              } else {
                  // Jika kondisi normal (atau PWM belum mentok), reset timer & alarm
                  failureStartTime = 0;
                  g_hardwareAlarm = false;
              }
              // ==========================================================
          } else {
              // Jika mode manual, pastikan alarm mati
              g_hardwareAlarm = false;
          }
      }

      // --- 4. RTC UPDATE & DISPLAY (Optimized) ---
      // Tentukan interval: 1 detik jika Auto/Recording, 60 detik jika Standby
      unsigned long rtcInterval = isAutoMode ? 1000 : 60000;
      
      // Simpan data waktu terakhir dari hardware (Static agar tersimpan antar-loop)

      // A. BACA HARDWARE RTC (Sesuai Interval)
      if (firstRun || (millis() - lastRtcReadTime > rtcInterval)) {
          lastRtcReadTime = millis();
          lastHwTime = rtc.now(); // Baca I2C yang berat hanya disini
          firstRun = false;
      }

      // B. HITUNG WAKTU TAMPIL (Software Calculation)
      // Waktu Sekarang = Waktu Hardware Terakhir + Selisih Detik berlalu
      unsigned long deltaSec = (millis() - lastRtcReadTime) / 1000;
      DateTime showTime = lastHwTime + TimeSpan(deltaSec);

      static int lastDisplayedSec = -1;
      if (showTime.second() != lastDisplayedSec) {
          lastDisplayedSec = showTime.second();
          snprintf(g_rtcTimeBuf, sizeof(g_rtcTimeBuf), "%02d:%02d:%02d", 
                   showTime.hour(), showTime.minute(), showTime.second());
          
          // --- TAMBAHKAN KODE INI ---
          g_curYear   = showTime.year();
          g_curMonth  = showTime.month();
          g_curDay    = showTime.day();
          g_curHour   = showTime.hour();
          g_curMinute = showTime.minute();
          g_curSecond = showTime.second();
          // --------------------------
      }

      // --- 5. IMU / QMI8658 ---
      if (millis() - lastImuReadTime > 200) {
          lastImuReadTime = millis();
          if (qmi.getDataReady()) {
              if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                  g_accX = acc.x; g_accY = acc.y; g_accZ = acc.z;
                  g_imuReady = true;
              }
          }
      }

      if (millis() - lastLogTime > 1000) {
          lastLogTime = millis();
          
          // Panggil fungsi log (Hanya akan mencatat jika isAutoMode = true)
          logToSD(); 
      }

      vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}
void on_btn_set_time(lv_event_t * e) {
    // 1. Ambil nilai index dari Dropdown
    int h = lv_dropdown_get_selected(ui_Dropdown1);
    int m = lv_dropdown_get_selected(ui_Dropdown2);
    int s = lv_dropdown_get_selected(ui_Dropdown3);

    // 2. Kirim ke Variabel Global
    g_setH = h;
    g_setM = m;
    g_setS = s;

    g_reqSetTime = true;       // Request update RTC
    g_isEditingTime = false;   // <--- TAMBAHKAN INI: Matikan mode edit agar jam jalan lagi

    USBSerial.printf("REQ SET TIME: %02d:%02d:%02d\n", h, m, s);
}

void sendFileList() {
  if (!g_sdReady) {
      ble.sendTelemetry("$ERR:SD Not Ready;");
      return;
  }
  
  String list = "$LST:";
  File root = SD_MMC.open("/");
  File file = root.openNextFile();
  
  while(file){
    if(!file.isDirectory()){
      String fname = String(file.name());
      
      // --- LOGIKA BARU (CASE INSENSITIVE) ---
      String fnameLower = fname;
      fnameLower.toLowerCase(); // Ubah ke huruf kecil semua untuk pengecekan
      
      // Cek apakah berakhiran .csv (file system mac ._ diabaikan)
      if (fnameLower.endsWith(".csv") && !fname.startsWith("._")) {
          
          if (fname.startsWith("/")) fname = fname.substring(1); // Hilangkan slash
          list += fname + ",";
      }
      // --------------------------------------
    }
    file = root.openNextFile();
  }
  list += ";";
  ble.sendTelemetry(list);
}

// 2. Mulai Transfer File ($READ:...)
void startFileTransfer(String filename) {
  if (isTransferring) return; // Jangan mulai kalau sedang jalan

  if (!filename.startsWith("/")) filename = "/" + filename;
  
  // Gunakan SD_MMC sesuai setup Anda
  if (SD_MMC.exists(filename)) {
    transferFile = SD_MMC.open(filename, FILE_READ);
    if (transferFile) {
      isTransferring = true;
      USBSerial.println("Start Sending: " + filename);
    }
  } else {
    USBSerial.println("File Not Found: " + filename);
    ble.sendTelemetry("$ERR:File Not Found;");
  }
}

// 3. Loop Transfer (Dipanggil di void loop)
void handleFileTransfer() {
  if (isTransferring && transferFile) {
    if (transferFile.available()) {
      uint8_t buffer[240]; // Buffer chunk (max MTU biasanya 247-an)
      int bytesRead = transferFile.read(buffer, sizeof(buffer));
      
      ble.sendRawData(buffer, bytesRead);
      
      // Delay kecil agar stack BLE tidak penuh
      delay(10); 
    } else {
      ble.sendTelemetry("$EOF;"); // Sinyal selesai
      transferFile.close();
      isTransferring = false;
      USBSerial.println("Transfer Done.");
    }
  }
}

void onCommandReceived(String cmd) {
  cmd.trim();
  if (cmd.startsWith("$") && cmd.endsWith(";")) {
    String payload = cmd.substring(1, cmd.length() - 1);
    int splitIndex = payload.indexOf(':');
    String header = "";
    String value = "";

    if (splitIndex != -1) {
      header = payload.substring(0, splitIndex);
      value = payload.substring(splitIndex + 1);
    } else {
      header = payload;
    }

    USBSerial.printf("BLE CMD -> Header: %s, Value: %s\n", header.c_str(), value.c_str());

    if (header == "ST") {
       if (value == "1") startSystem();
       else stopSystem();
    }
    else if (header == "SP") {
       targetFlow = value.toFloat();
       g_reqUpdateRollers = true;
    }
    else if (header == "LIST") {
       sendFileList();
    }
    else if (header == "READ") {
       startFileTransfer(value);
    }
    // --- TAMBAHAN LOGIKA HAPUS FILE SPESIFIK ---
    else if (header == "DEL") {
       deleteSpecificFile(value);
    }
    else if (header == "DELALL") {

    if (!g_sdReady) {
        USBSerial.println("ERR: SD Card tidak siap."); 
        return;
    }

    if (isAutoMode) {
        USBSerial.println("ERR: Tidak bisa menghapus saat logging aktif.");
        return;
    }

    File root = SD_MMC.open("/"); 
    if (!root || !root.isDirectory()) return;

    File file = root.openNextFile(); 
    int deletedCount = 0;

    while (file) {
        if (!file.isDirectory()) {
            String fileName = String(file.name()); 
            
            // PERBAIKAN DISINI:
            String fileNameLower = fileName; 
            fileNameLower.toLowerCase(); // Mengubah isi variabel menjadi huruf kecil 

            // Sekarang cek menggunakan variabel yang sudah huruf kecil
            if (fileNameLower.endsWith(".csv") && !fileName.startsWith("._")) { 
                String fullPath = fileName;
                if (!fullPath.startsWith("/")) fullPath = "/" + fullPath; 
                
                file.close(); 
                if (SD_MMC.remove(fullPath)) {
                    deletedCount++;
                    USBSerial.printf("DELETED: %s\n", fullPath.c_str());
                }
                
                // Reset pointer direktori setelah penghapusan agar tidak error saat openNextFile
                root = SD_MMC.open("/"); 
            }
        }
        file = root.openNextFile(); 
    }
    
    USBSerial.printf("Selesai. %d file dihapus.\n", deletedCount);

    }
  }
}

// --- FUNGSI INIT CODEC (Sama Persis dengan Contoh) ---
esp_err_t audio_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
  
  if (!es_handle) {
      USBSerial.println("AUDIO ERR: Gagal membuat handle ES8311");
      return ESP_FAIL;
  }

  // KONFIGURASI CLOCK (INI KUNCINYA!)
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256, // 16000 * 256 = 4096000 Hz
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  // Terapkan Konfigurasi
  es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency);
  es8311_microphone_config(es_handle, false); // Matikan Mic
  es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL);
  es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN);
  
  USBSerial.println("AUDIO: ES8311 Init Sukses!");
  return ESP_OK;
}

// Fungsi Load Audio (Tetap sama)
void loadAudioToRAM(const char* filename) {
    if (!g_sdReady) return;
    File file = SD_MMC.open(filename, FILE_READ);
    if (!file) {
        USBSerial.printf("AUDIO ERR: %s tidak ditemukan!\n", filename);
        return;
    }
    g_audioSize = file.size();
    g_audioBuffer = (uint8_t*) heap_caps_malloc(g_audioSize, MALLOC_CAP_SPIRAM);
    if (g_audioBuffer != NULL) {
        file.read(g_audioBuffer, g_audioSize);
        USBSerial.printf("AUDIO: %s loaded (%d bytes)\n", filename, g_audioSize);
    } else {
        USBSerial.println("AUDIO ERR: PSRAM Penuh!");
    }
    file.close();
}


void TaskAudioCode(void * pvParameters) {
    // Pastikan Amplifier mati di awal
    pinMode(PA_PIN, OUTPUT);
    digitalWrite(PA_PIN, LOW);

    for(;;) {
        // Cek Trigger
        if (g_playAlarm && g_audioBuffer != NULL) {
            
            // Nyalakan Amplifier
            digitalWrite(PA_PIN, HIGH);
            vTaskDelay(50 / portTICK_PERIOD_MS); // Tunggu Amp bangun (pop noise reduction)

            size_t cursor = 0;
            size_t chunkSize = 2048; // Chunk lebih besar biar stabil

            // Loop memutar audio dari RAM
            while (cursor < g_audioSize && g_playAlarm) {
                size_t remaining = g_audioSize - cursor;
                size_t toWrite = (remaining < chunkSize) ? remaining : chunkSize;
                
                // Tulis ke I2S (Suara keluar disini)
                i2s.write(g_audioBuffer + cursor, toWrite);
                
                cursor += toWrite;
            }

            // Jika file habis tapi alarm masih aktif (LOOPING), beri jeda sedikit
            if (g_playAlarm && cursor >= g_audioSize) {
                 vTaskDelay(100 / portTICK_PERIOD_MS); 
            }

        } else {
            // Jika tidak ada alarm, matikan Amp & Tidur
            digitalWrite(PA_PIN, LOW);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}

// ================= SETUP UTAMA =================
void setup() {
  USBSerial.begin(115200);
  Wire.begin(IIC_SDA, IIC_SCL, 400000);

  // 1. SETUP EXPANDER & SD CARD POWER
  // ---------------------------------
  if (!expander.begin(0x20)) { 
    Serial.println("Failed to find XCA9554 chip");
    while (1);
  }
  
  // Konfigurasi pin expander (0,1,2 untuk LCD/Touch, 7 untuk SD Power)
  expander.pinMode(0, OUTPUT);
  expander.pinMode(1, OUTPUT);
  expander.pinMode(2, OUTPUT);
  expander.pinMode(7, OUTPUT); // <--- Pin Power SD Card
  
  expander.digitalWrite(0, LOW);
  expander.digitalWrite(1, LOW);
  expander.digitalWrite(2, LOW);
  
  // Nyalakan Power SD Card
  expander.digitalWrite(7, HIGH); 
  delay(100); // Tunggu power stabil

  expander.digitalWrite(0, HIGH);
  expander.digitalWrite(1, HIGH);
  expander.digitalWrite(2, HIGH);

  // 2. SETUP SENSOR FLOW
  if (!ads.begin()) {
    USBSerial.println("ADS1115 Failed!");
  } else {
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_128SPS);
  }
  calLoadNVS();

  // 3. SETUP IMU (QMI8658)
  if (qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
      USBSerial.println("QMI8658 Ready!");
      // 3 Argumen saja untuk library versi terbaru
      qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
      qmi.enableAccelerometer();
  } else {
      USBSerial.println("QMI8658 Failed!");
  }

  if (!rtc.begin()) {
    USBSerial.println("Couldn't find RTC");
  } else {
    USBSerial.println("RTC Connected");
    
    // Cek jika RTC kehilangan daya, set waktu default (opsional)
    if (rtc.lostPower()) {
      USBSerial.println("RTC lost power, let's set the time!");
      // Set waktu ke waktu kompilasi sketch saat ini
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // 4. SETUP TOUCH & LCD
  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(1000);
  }
  gfx->begin();
  gfx->setBrightness(200);
  
  lv_init();
  
  #if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
  #endif

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

  uint32_t buffer_size = screenWidth * screenHeight; 
  
  buf1 = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  buf2 = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

  // Cek apakah PSRAM berhasil dialokasikan
  if (buf1 == NULL || buf2 == NULL) {
      USBSerial.println("GAGAL Alokasi PSRAM! Pastikan opsi PSRAM 'OPI' aktif di Tools > PSRAM");
      while(1); // Stop disini jika gagal
  }

  // Inisialisasi Double Buffer
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buffer_size);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_90;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  ui_init(); // Load UI dari SquareLine Studio

  // 5. SETUP SD CARD (CHECK & ICON)
  // -------------------------------
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);

  if (SD_MMC.begin("/sdcard", true)) {
      g_sdReady = true; // Tandai SD Card Siap
      USBSerial.println("SD Card Mounted Successfully");
      
      // Update UI Icon
      lv_obj_set_style_text_color(ui_LabelSD, lv_palette_main(LV_PALETTE_GREEN), 0);
      
      uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
      USBSerial.printf("SD Size: %llu MB\n", cardSize);
      
  } else {
      g_sdReady = false;
      USBSerial.println("SD Card Mount Failed / Not Inserted");
      lv_obj_set_style_text_color(ui_LabelSD, lv_palette_main(LV_PALETTE_GREY), 0);
  }

  // 6. SETUP FAN MOTOR
  ledcAttach(FAN_PWM_PIN, 25000, 8);
  ledcWrite(FAN_PWM_PIN, 0);

  ble.init("NAPAS-04-CORE"); // Nama Device
  ble.setCommandCallback(onCommandReceived);

  USBSerial.println("Setup done. BLE Active.");

  xTaskCreatePinnedToCore(
      TaskSensorCode,   /* Fungsi task */
      "TaskSensor",     /* Nama task */
      10000,            /* Stack depth (bytes) */
      NULL,             /* Parameter */
      1,                /* Prioritas (1 = rendah, biar UI lebih smooth) */
      &TaskSensorHandle,/* Handle */
      0);               /* Core 0 (Core 1 dipakai Arduino/LVGL) */

  USBSerial.println("Setup done. Dual Core Active.");

  if (g_sdReady) {
      loadAudioToRAM("/alarm.pcm"); 
  }

  // 3. INIT AUDIO HARDWARE (Urutan Sesuai Contoh Pabrikan)
  pinMode(PA_PIN, OUTPUT);
  digitalWrite(PA_PIN, HIGH); // Nyalakan sebentar saat init
  
  // A. Set Pin I2S
  i2s.setPins(I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, I2S_DI_IO, I2S_MCK_IO);
  
  // B. Start I2S
  if (!i2s.begin(I2S_MODE_STD, EXAMPLE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    USBSerial.println("AUDIO ERR: Failed to initialize I2S bus!");
  }
  
  // C. Init Codec ES8311 (Wajib setelah Wire.begin)
  audio_codec_init();

  digitalWrite(PA_PIN, LOW); // Matikan Amp setelah init selesai (tunggu trigger)

  // 4. BUAT TASK AUDIO
  xTaskCreatePinnedToCore(TaskAudioCode, "TaskAudio", 4096, NULL, 2, NULL, 1);

  USBSerial.println("Setup done. System Ready.");

  if(ui_Arc1 != NULL) lv_obj_add_event_cb(ui_Arc1, on_arc_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
  if(ui_Button1 != NULL) lv_obj_add_event_cb(ui_Button1, on_btn_auto_start, LV_EVENT_ALL, NULL);
  if(ui_Button3 != NULL) lv_obj_add_event_cb(ui_Button3, on_btn_set_time, LV_EVENT_CLICKED, NULL);
  if(ui_Spinner1 != NULL) lv_obj_add_flag(ui_Spinner1, LV_OBJ_FLAG_HIDDEN);
  if(ui_Button14 != NULL) lv_obj_add_event_cb(ui_Button14, on_btn_delete_all, LV_EVENT_CLICKED, NULL);
  if(ui_Button20 != NULL) lv_obj_add_event_cb(ui_Button20, on_btn_set_date, LV_EVENT_CLICKED, NULL);
  lv_scr_load(ui_Screen2);
  lv_timer_handler(); // Biarkan LVGL membangun objek
  delay(2);
  
  lv_scr_load(ui_Screen1);
  lv_timer_handler();
  delay(2);
}

void refreshCsvList() {

    if (!g_sdReady || ui_Dropdown5 == NULL) {
        if (ui_Dropdown5 != NULL) {
            lv_dropdown_set_options(ui_Dropdown5, "SD Card Error");
            lv_dropdown_set_text(ui_Dropdown5, "SD Error");
        }
        return;
    }

    // 2. Buka direktori root SD Card [cite: 208, 209]
    File root = SD_MMC.open("/");
    if (!root || !root.isDirectory()) {
        USBSerial.println("ERR: Gagal buka direktori root");
        return;
    }

    String fileList = ""; 
    int fileCount = 0;
    File file = root.openNextFile(); 

    // 3. Scanning file .csv [cite
    while (file) {
        if (!file.isDirectory()) {
            String fileName = String(file.name());
            String fileNameLower = fileName;
            fileNameLower.toLowerCase();

            // Filter hanya file CSV dan abaikan file sistem [cite: 212, 213]
            if (fileNameLower.endsWith(".csv") && !fileName.startsWith("._")) {
                if (fileList.length() > 0) fileList += "\n";
                if (fileName.startsWith("/")) fileName = fileName.substring(1); 
                
                fileList += fileName;
                fileCount++; 
            }
        }
        file = root.openNextFile(); 
    }
 
    // 4. Update Tampilan Dropdown
    if (fileCount > 0) {
        // Masukkan daftar file ke dalam list internal [cite: 216]
        lv_dropdown_set_options(ui_Dropdown5, fileList.c_str());
        
        // Ganti index ke -1 agar tidak ada item yang otomatis terpilih sebagai label utama
        lv_dropdown_set_selected(ui_Dropdown5, -1); 

        // PAKSA teks utama menampilkan jumlah file
        static char headerBuf[32]; // Gunakan static agar memory tetap tersedia untuk LVGL
        snprintf(headerBuf, sizeof(headerBuf), "Found %d file", fileCount);
        lv_dropdown_set_text(ui_Dropdown5, headerBuf); 
        
        USBSerial.printf("CSV List Updated: %d files\n", fileCount);
    } else {
        lv_dropdown_set_options(ui_Dropdown5, "No CSV Found"); 
        lv_dropdown_set_text(ui_Dropdown5, "0 Files Found");
    }
}

void on_btn_delete_all(lv_event_t * e) {
    // Panggil fungsi hapus
    deleteAllCsv();
}

void deleteSpecificFile(String filename) {
    if (!g_sdReady) {
        ble.sendTelemetry("$ERR:SD Not Ready;");
        return;
    }

    // Tambahkan slash di depan jika belum ada agar path valid
    if (!filename.startsWith("/")) filename = "/" + filename;

    if (SD_MMC.exists(filename)) {
        // Pastikan file tidak sedang dibuka untuk transfer atau logging
        if (SD_MMC.remove(filename)) {
            USBSerial.println("Deleted File: " + filename);
            ble.sendTelemetry("$DEL_OK:" + filename + ";");
            
            // Refresh daftar di UI jika sedang berada di layar daftar file
            refreshCsvList();
        } else {
            USBSerial.println("Failed to delete: " + filename);
            ble.sendTelemetry("$ERR:Delete Failed;");
        }
    } else {
        USBSerial.println("File not found for deletion: " + filename);
        ble.sendTelemetry("$ERR:File Not Found;");
    }
}

void deleteAllCsv() {
    if (!g_sdReady) {
        USBSerial.println("ERR: SD Card tidak siap."); 
        return;
    }

    if (isAutoMode) {
        USBSerial.println("ERR: Tidak bisa menghapus saat logging aktif.");
        return;
    }

    File root = SD_MMC.open("/"); 
    if (!root || !root.isDirectory()) return;

    File file = root.openNextFile(); 
    int deletedCount = 0;

    while (file) {
        if (!file.isDirectory()) {
            String fileName = String(file.name()); 
            
            // PERBAIKAN DISINI:
            String fileNameLower = fileName; 
            fileNameLower.toLowerCase(); // Mengubah isi variabel menjadi huruf kecil 

            // Sekarang cek menggunakan variabel yang sudah huruf kecil
            if (fileNameLower.endsWith(".csv") && !fileName.startsWith("._")) { 
                String fullPath = fileName;
                if (!fullPath.startsWith("/")) fullPath = "/" + fullPath; 
                
                file.close(); 
                if (SD_MMC.remove(fullPath)) {
                    deletedCount++;
                    USBSerial.printf("DELETED: %s\n", fullPath.c_str());
                }
                
                // Reset pointer direktori setelah penghapusan agar tidak error saat openNextFile
                root = SD_MMC.open("/"); 
            }
        }
        file = root.openNextFile(); 
    }
    
    USBSerial.printf("Selesai. %d file dihapus.\n", deletedCount);
    
    // Kembali ke Screen 5 dan refresh list
    if (ui_Screen5 != NULL) {
        _ui_screen_change(&ui_Screen5, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Screen5_screen_init); 
    }
    refreshCsvList(); 
}

void loop() {
  lv_timer_handler(); // Biarkan LVGL menggambar
  
  if (isTransferring) {
    handleFileTransfer();
    ble.handleConnectionLoop();
    return; // Skip sensor update agar pengiriman cepat
  }

  // Status Layar
  static bool isScreenOn = true; 

  // ============================================================
  // FITUR 1: DETEKSI SENTUHAN (RESET TIMER & WAKE UP)
  // ============================================================
  if (g_isTouched) {
      lastInteractionTime = millis();
      
      // Wake Up jika disentuh
      if (!isScreenOn) {
          gfx->setBrightness(200); 
          isScreenOn = true;
          USBSerial.println("SCREEN: WAKE UP (TOUCH)");
      }
  }

  // Ambil layar yang sedang aktif
  lv_obj_t * actScr = lv_scr_act();

  // ============================================================
  // FITUR 2: AUTO SLEEP (DENGAN PENGECUALIAN ALARM)
  // ============================================================
  if (ui_Screen1 != NULL && actScr == ui_Screen1) {
      
      // Syarat Sleep:
      // 1. Layar sedang Nyala
      // 2. Tidak ada sentuhan > 10 Detik
      // 3. DAN TIDAK ADA ALARM ( !g_hardwareAlarm ) <--- INI TAMBAHANNYA
      if (isScreenOn && (millis() - lastInteractionTime > 20000) && !g_hardwareAlarm) {
          gfx->setBrightness(0); // Matikan Backlight
          isScreenOn = false;
          USBSerial.println("SCREEN: SLEEP (10s Timeout)");
      }
  }

  // ============================================================
  // FITUR 3: AUTO RETURN TO SCREEN 1
  // ============================================================
  // Jika layar aktif saat ini bukan Screen 1
  if (actScr != ui_Screen1) {
    if (millis() - lastInteractionTime > 15000) {
        USBSerial.println("AUTO RETURN: Back to Home Screen"); 
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init); 
        lastInteractionTime = millis(); 
    }
  }

  // ============================================================
  // FITUR 4: ALARM HARDWARE (KEDIP PANEL & FORCE WAKE UP)
  // ============================================================
  static unsigned long blinkTimer = 0;
  static bool blinkState = false;

  if (ui_Panel3 != NULL) {
      if (g_hardwareAlarm) {
          
          // --- [PERBAIKAN DISINI: NYALAKAN AUDIO] ---
          g_playAlarm = true; 
          // ------------------------------------------
          lv_obj_set_style_text_font(ui_Label22, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_label_set_text(ui_Label22, "Nyangkut !!");
          // A. FORCE WAKE UP (Jika alarm bunyi saat layar mati)
          if (!isScreenOn) {
              gfx->setBrightness(200);
              isScreenOn = true;
              lastInteractionTime = millis(); // Reset timer sleep juga
              USBSerial.println("ALARM: FORCE WAKE UP!");
          }

          // B. Logika Kedip
          if (millis() - blinkTimer > 500) {
              blinkTimer = millis();
              blinkState = !blinkState; 
              
              if (blinkState) {
                  lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);
              } else {
                  lv_obj_add_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN);
              }
          }
      } else {
        // HANYA sembunyikan jika TIDAK sedang countdown tombol
        if (!g_isCountingDown) { 
            lv_obj_add_flag(ui_Panel3, LV_OBJ_FLAG_HIDDEN); 
        }
    }
  }
  // ============================================================
  // FITUR 5: HOOK TOMBOL SET TIME
  // ============================================================
  if (ui_Screen3 != NULL) {
      if (ui_Button3 != NULL && !btn3Hooked) {
          lv_obj_add_event_cb(ui_Button3, on_btn_set_time, LV_EVENT_CLICKED, NULL);
          btn3Hooked = true; 
      }
  } else {
      btn3Hooked = false;
  }

  static bool screen4Loaded = false; 

  // Bagian pengecekan Screen di dalam loop()
  if (ui_Screen5 != NULL && actScr == ui_Screen5) { 
      if (!screen4Loaded) {
          refreshCsvList(); // Memperbarui Dropdown5 [cite: 239]
          screen4Loaded = true;
      }
  } else {
      screen4Loaded = false; 
  }

  static unsigned long lastBleLog = 0;
  if (millis() - lastBleLog >= 1000) { 
      lastBleLog = millis();

      // 1. HITUNG ACTIVITY DARI SENSOR QMI (GABUNGAN X, Y, Z)
      // Kita salin ke variabel lokal dulu agar aman
      float ax = g_accX;
      float ay = g_accY;
      float az = g_accZ;
      
      // Rumus Magnitude: Akar dari (x*x + y*y + z*z)
      // Nilai normal diam = ~1.0 (Gravitasi). Getaran = >1.0
      float activityVal = sqrt((ax * ax) + (ay * ay) + (az * az));

      if (ble.isConnected()) {
         char buffer[128];
         
         // 2. MASUKKAN 'activityVal' KE PROTOKOL ($...:ACT:%.2f;)
         snprintf(buffer, sizeof(buffer), 
           "$FLW:%.2f:SP:%.2f:BAT:%d:ST:%d:ACT:%.2f;", 
           g_flowLPM, targetFlow, g_batPercent, isAutoMode ? 1 : 0, activityVal
         );

         ble.sendTelemetry(String(buffer));

         if(ui_Label26) lv_obj_set_style_text_color(ui_Label26, lv_color_hex(0x0000FF), 0);

      } else {
         // Update Iklan (Opsional: ACT tidak muat di iklan karena keterbatasan byte, 
         // jadi flow & bat saja yang diupdate)
         ble.updateAdvertising(isAutoMode ? 1 : 0, g_flowLPM, (float)g_batPercent);
         
         if(ui_Label26) lv_obj_set_style_text_color(ui_Label26, lv_color_hex(0x808080), 0);
      }
  }

    ble.handleConnectionLoop();

    if (g_reqUpdateRollers) {
       g_reqUpdateRollers = false; // Matikan flag setelah dikerjakan
       
       // Contoh Target: 2.53
       
       // 1. Ambil Digit Pertama (Roller 1) -> 2
       int d1 = (int)targetFlow; 
       
       // Sisa desimal -> 0.53
       float rem1 = targetFlow - d1;
       
       // 2. Ambil Digit Kedua (Roller 2) -> 0.53 * 10 = 5.3 -> jadi 5
       int d2 = (int)(rem1 * 10);
       
       // Sisa desimal kedua -> 5.3 - 5 = 0.3
       float rem2 = (rem1 * 10) - d2;
       
       // 3. Ambil Digit Ketiga (Roller 3) -> 0.3 * 10 = 3
       // Kita tambah +0.5 agar pembulatan presisi (menghindari error float 2.9999 jadi 2)
       int d3 = (int)(rem2 * 10 + 0.5);

       // 4. Perintahkan Roller Berputar (Dengan Animasi)
       if (ui_Roller1 != NULL) lv_roller_set_selected(ui_Roller1, d1, LV_ANIM_ON);
       if (ui_Roller2 != NULL) lv_roller_set_selected(ui_Roller2, d2, LV_ANIM_ON);
       if (ui_Roller3 != NULL) lv_roller_set_selected(ui_Roller3, d3, LV_ANIM_ON);
       
       USBSerial.printf("UI UPDATE: Rollers set to %d . %d %d\n", d1, d2, d3);
    }


  // ============================================================
  // FITUR 6: UPDATE DATA UI
  // ============================================================
  static unsigned long lastUiUpdate = 0;
  
  // Update UI hanya jika layar nyala
  if (isScreenOn && (millis() - lastUiUpdate > 100)) {
      lastUiUpdate = millis();

      // 1. Update Flow
      if (ui_Label1 != NULL) {
         char strBuf[16];
         snprintf(strBuf, sizeof(strBuf), "%.2f", g_flowLPM); 
         lv_label_set_text(ui_Label1, strBuf);
      }

      // 2. Update Arc
      if (isAutoMode && ui_Arc1 != NULL) {
          int displayVal = 0;
          if (currentPwm >= minPWM) displayVal = map((int)currentPwm, minPWM, 255, 1, 100);
          lv_arc_set_value(ui_Arc1, displayVal);
      }

      // 3. Update Jam RTC
      if (ui_Label10 != NULL) {
          lv_label_set_text(ui_Label10, g_rtcTimeBuf);
      }

      // A. LOGIKA WAKTU (Jam, Menit, Detik)
      // Cek apakah user sedang membuka dropdown
      bool isAnyDropdownOpen = lv_dropdown_is_open(ui_Dropdown1) || 
                               lv_dropdown_is_open(ui_Dropdown2) || 
                               lv_dropdown_is_open(ui_Dropdown3);

      // Jika dropdown sedang dibuka, AKTIFKAN mode edit (KUNCI)
      if (isAnyDropdownOpen) {
          g_isEditingTime = true;
      }

      // Hanya update otomatis jika TIDAK sedang dalam mode edit
      // (Artinya: Sekali disentuh, dia akan diam TERUS sampai tombol SET ditekan)
      if (!g_isEditingTime) {
          if (ui_Dropdown1) lv_dropdown_set_selected(ui_Dropdown1, g_curHour);
          if (ui_Dropdown2) lv_dropdown_set_selected(ui_Dropdown2, g_curMinute);
          if (ui_Dropdown3) lv_dropdown_set_selected(ui_Dropdown3, g_curSecond);
      }

      // 4. Update Stopwatch
      if (ui_Label9 != NULL) {
          if (isAutoMode) {
              unsigned long elapsedSec = (millis() - autoStartTime) / 1000;
              int hh = elapsedSec / 3600;
              int mm = (elapsedSec % 3600) / 60;
              int ss = elapsedSec % 60;
              char timerBuf[16];
              snprintf(timerBuf, sizeof(timerBuf), "%02d:%02d:%02d", hh, mm, ss);
              lv_label_set_text(ui_Label9, timerBuf);
          } else {
              lv_label_set_text(ui_Label9, "00:00:00");
          }
      }

      // 5. Update IMU
      if (g_imuReady && ui_Label6 != NULL) {
          char imuBuf[32];
          snprintf(imuBuf, sizeof(imuBuf), "X:%.1f Y:%.1f Z:%.1f", g_accX, g_accY, g_accZ);
          lv_label_set_text(ui_Label6, imuBuf);
          g_imuReady = false;
      }
  }
  
  delay(5);
}
