#include <Wire.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include "pins_config.h"

// Include Modul Sensor & Display
#include "FusionIndex.h"
#include "GSR.h"
#include "MAX30102.h"
#include "MLX90614.h"
#include "MPU6050.h"
#include "OLED.h"

// =============================================================================
//  MODE DEBUG — Pengujian Sensor Satu Persatu
//
//  Petunjuk Penggunaan:
//  1. Uncomment SATU baris #define di bawah ini untuk menguji sensor tertentu.
//  2. Upload sketch ke board Seeed Studio XIAO ESP32-C3.
//  3. Buka Serial Monitor (115200 baud). Informasi lengkap akan muncul di
//     Serial Monitor DAN Layar OLED secara bersamaan.
//
//  Jika SEMUA baris di bawah di-comment (//) -> Berjalan pada MODE NORMAL
//  (Semua sensor aktif + BLE Advertising/Streaming JSON + OLED).
// =============================================================================
// #define TEST_MAX30102    // Test detak jantung (HR), IBI, HRV & sensor proximity
// #define TEST_MLX90614    // Test suhu inframerah non-kontak objek kulit
// #define TEST_MPU6050     // Test pergerakan akselerometer 3-sumbu (X, Y, Z)
// #define TEST_GSR         // Test konduktansi listrik kulit (ADC resistansi)
// #define TEST_BLE         // Test komunikasi Bluetooth Low Energy tanpa sensor
// =============================================================================

// ======================== IDENTITAS & UUID BLE ========================
#define DEVICE_NAME     "MindTrack_BLE"
#define SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Mengirim JSON via Notify
#define CHAR_CMD_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9" // Menerima Command (misal: ping)

// ======================== VARIABEL GLOBAL & TIMER ========================
FusionIndex fusion;
NimBLECharacteristic *pDataChar = nullptr;
NimBLECharacteristic *pCmdChar  = nullptr;
bool deviceConnected            = false;

unsigned long lastSend = 0;             // Timer transmisi data BLE & Serial (ms)
unsigned long lastOLED = 0;             // Timer refresh layar OLED (ms)
const uint32_t SEND_INTERVAL = 500;     // Kirim payload setiap 500 ms (2 Hz)
const uint32_t OLED_INTERVAL = 500;     // Refresh layar OLED setiap 500 ms

// ======================== CALLBACK PERINTAH BLE (COMMAND) ========================
// Menangani data teks yang dikirimkan oleh aplikasi ponsel ke perangkat
class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    std::string val = pChar->getValue();
    if (val.empty()) return;
    Serial.printf("[BLE-CMD] Perintah diterima: %s\n", val.c_str());

    // Respon ping-pong untuk validasi komunikasi 2 arah
    if (val == "ping") {
      pDataChar->setValue("{\"pong\":1}");
      pDataChar->notify();
      Serial.println("[BLE-CMD] Membalas respon: {\"pong\":1}");
    }
  }
};

// ======================== CALLBACK SERVER BLE (KONEKSI) ========================
// Mendeteksi event koneksi, diskoneksi, dan negosiasi MTU ponsel
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
    deviceConnected = true;
    Serial.printf("[BLE] Terhubung ke perangkat client: %s\n",
                  connInfo.getAddress().toString().c_str());
    // Optimasi parameter koneksi interval: min 7.5ms, max 15ms, latency 0, timeout 4s
    pServer->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 400);
    // Request MTU besar (247 byte) agar JSON 1 paket tidak terfragmentasi
    NimBLEDevice::setMTU(247);
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("[BLE] Koneksi terputus. Mengaktifkan re-advertising...");
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
    Serial.printf("[BLE] Ukuran MTU dinegosiasikan menjadi: %d bytes\n", MTU);
  }
};

// ======================== INISIALISASI BLE ========================
void initBLE() {
  Serial.println("[BLE] Memulai inisialisasi NimBLE Stack...");
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Daya transmisi maksimal (+9dBm)

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  // Karakteristik Data (Mengirimkan data hasil pengolahan sensor)
  pDataChar = pService->createCharacteristic(
      CHAR_DATA_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  // Karakteristik Perintah (Menerima input dari aplikasi)
  pCmdChar = pService->createCharacteristic(
      CHAR_CMD_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pCmdChar->setCallbacks(new CmdCallbacks());

  pService->start();

  // Konfigurasi paket advertising
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setMinInterval(32);  // ~20ms
  pAdv->setMaxInterval(160); // ~100ms
  NimBLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising aktif. Siap ditemukan sebagai 'MindTrack_BLE'.");
}

// ======================== INISIALISASI SEMUA SENSOR ========================
void initSensors() {
  Serial.println("[SYSTEM] Menginisialisasi I2C Bus & Sensor...");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); // 400 kHz Fast-Mode I2C

  initMAX30102(); // Inisialisasi PPG sensor
  initMLX();      // Inisialisasi sensor suhu inframerah
  initMPU();      // Inisialisasi IMU accelerometer
  initGSR();      // Inisialisasi pin ADC GSR
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

#if defined(TEST_MAX30102)
  // ── PENGUJIAN 1: MAX30102 (Detak Jantung & HRV) ──────────────────────────
  Serial.println("\n=======================================================");
  Serial.println("  MODE TEST: SENSOR MAX30102 (PPG / HR / HRV)");
  Serial.println("=======================================================");
  Serial.println("Kabel: VCC->3.3V, GND->GND, SDA->GPIO6, SCL->GPIO7");
  Serial.println("Instruksi: Tempelkan jari di atas sensor optik.");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initOLED();
  initMAX30102();
  oledShowBoot("Test: MAX30102");

#elif defined(TEST_MLX90614)
  // ── PENGUJIAN 2: MLX90614 (Suhu Inframerah Kulit) ─────────────────────────
  Serial.println("\n=======================================================");
  Serial.println("  MODE TEST: SENSOR MLX90614 (Suhu Non-Kontak)");
  Serial.println("=======================================================");
  Serial.println("Kabel: VCC->3.3V, GND->GND, SDA->GPIO6, SCL->GPIO7");
  Serial.println("Instruksi: Arahkan sensor ke permukaan kulit (+- 2-5 cm).");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initOLED();
  initMLX();
  oledShowBoot("Test: MLX90614");

#elif defined(TEST_MPU6050)
  // ── PENGUJIAN 3: MPU6050 (Akselerometer & Gerakan Tubuh) ──────────────────
  Serial.println("\n=======================================================");
  Serial.println("  MODE TEST: SENSOR MPU6050 (Akselerometer IMU)");
  Serial.println("=======================================================");
  Serial.println("Kabel: VCC->3.3V, GND->GND, SDA->GPIO6, SCL->GPIO7");
  Serial.println("Instruksi: Goyangkan / gerakkan board untuk memicu akselerasi.");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initOLED();
  initMPU();
  oledShowBoot("Test: MPU6050");

#elif defined(TEST_GSR)
  // ── PENGUJIAN 4: GSR (Galvanic Skin Response / Hambatan Kulit) ────────────
  Serial.println("\n=======================================================");
  Serial.println("  MODE TEST: SENSOR GSR (Konduktansi Kulit)");
  Serial.println("=======================================================");
  Serial.printf("Kabel: VCC->3.3V, GND->GND, Signal->GPIO%d (A0)\n", PIN_GSR);
  Serial.println("Instruksi: Pasang elektroda pada dua jari tangan.");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initOLED();
  initGSR();
  oledShowBoot("Test: GSR");

#elif defined(TEST_BLE)
  // ── PENGUJIAN 5: BLE (Komunikasi Nirkabel ke Ponsel) ─────────────────────
  Serial.println("\n=======================================================");
  Serial.println("  MODE TEST: BLUETOOTH LOW ENERGY (BLE)");
  Serial.println("=======================================================");
  Serial.println("Instruksi: Buka aplikasi nRF Connect / LightBlue di ponsel.");
  Serial.println("Cari device 'MindTrack_BLE' lalu Subscribe Notification.");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initOLED();
  initBLE();
  oledShowBoot("Test: BLE");

#else
  // ── MODE NORMAL: Seluruh Sistem Berjalan Penuh ───────────────────────────
  Serial.println("\n=======================================================");
  Serial.println("   MINDTRACK v2.1 — MONITORING STRES WEARABLE (NORMAL)");
  Serial.println("=======================================================");
  initSensors();
  initOLED();
  initBLE();
  Serial.println("[SYSTEM] Sistem siap. Menunggu pembacaan sensor...\n");
#endif
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {

// ── EKSEKUSI PENGUJIAN 1: MAX30102 ───────────────────────────────────────────
#if defined(TEST_MAX30102)
  // Sensor MAX30102 wajib dipanggil sesering mungkin agar tidak kehilangan beat
  unsigned long ibi = 0;
  float hr = readHeartRate(ibi);

  if (millis() - lastSend < 500) return;
  lastSend = millis();

  float hrv   = calculateHRV();
  uint32_t ir = getIRValue();
  bool worn   = (ir > 50000);

  // Output Terstruktur di Serial Monitor
  Serial.println("┌─── [DEBUG SENSOR MAX30102] ─────────────────");
  Serial.printf("│ Proximity IR : %lu (Ambang batas: > 50000)\n", (unsigned long)ir);
  Serial.printf("│ Kondisi Jari : %s\n", worn ? "TERTEMPEL DENGAN BAIK [OK]" : "LEPAS / TIDAK MENYENTUH [x]");
  if (worn) {
    Serial.printf("│ Detak Jantung: %.0f BPM\n", hr);
    Serial.printf("│ Interval IBI : %lu ms (Waktu antar detak)\n", ibi);
    Serial.printf("│ HRV (RMSSD)  : %s\n",
                  (hrv < 0) ? "Mengumpulkan minimal 2 beat..." : (String(hrv, 1) + " ms").c_str());
  } else {
    Serial.println("│ Catatan      : Letakkan jari tepat di atas LED sensor");
  }
  Serial.println("└──────────────────────────────────────────────");

  // Output Terstruktur di Layar OLED
  oledShowTestMAX30102(worn, ir, hr, ibi, hrv);

// ── EKSEKUSI PENGUJIAN 2: MLX90614 ───────────────────────────────────────────
#elif defined(TEST_MLX90614)
  if (millis() - lastSend < 500) return;
  lastSend = millis();

  float temp = readTemperature();

  // Output Terstruktur di Serial Monitor
  Serial.println("┌─── [DEBUG SENSOR MLX90614] ─────────────────");
  Serial.printf("│ Suhu Objek   : %.2f °C\n", temp);
  if (temp < 30.0f) {
    Serial.println("│ Status Suhu  : ⚠ Terlalu dingin (Sensor terlalu jauh dari kulit?)");
  } else if (temp > 40.0f) {
    Serial.println("│ Status Suhu  : ⚠ Terlalu tinggi (Cek kondisi lingkungan)");
  } else {
    Serial.println("│ Status Suhu  : ✓ Normal fisiologis kulit manusia (30 - 40 °C)");
  }
  Serial.println("└──────────────────────────────────────────────");

  // Output Terstruktur di Layar OLED
  oledShowTestMLX(temp);

// ── EKSEKUSI PENGUJIAN 3: MPU6050 ────────────────────────────────────────────
#elif defined(TEST_MPU6050)
  if (millis() - lastSend < 200) return;
  lastSend = millis();

  sensors_event_t accel;
  float motion    = readMotion(accel);
  float ax        = accel.acceleration.x;
  float ay        = accel.acceleration.y;
  float az        = accel.acceleration.z;
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // Output Terstruktur di Serial Monitor
  Serial.println("┌─── [DEBUG SENSOR MPU6050] ──────────────────");
  Serial.printf("│ Akselerasi   : X: %+6.2f | Y: %+6.2f | Z: %+6.2f m/s²\n", ax, ay, az);
  Serial.printf("│ Total Vektor : %.2f m/s² (Termasuk gravitasi ~9.81)\n", magnitude);
  Serial.printf("│ Net Motion   : %.2f (Pergerakan tubuh bersih)\n", motion);
  if (motion < 5) {
    Serial.println("│ Status Gerak : Diam / Istirahat [Bobot HR maksimal]");
  } else if (motion < 25) {
    Serial.println("│ Status Gerak : Gerakan Ringan / Berjalan");
  } else {
    Serial.println("│ Status Gerak : Aktif Bergerak / Lari [Filter artefak aktif]");
  }
  Serial.println("└──────────────────────────────────────────────");

  // Output Terstruktur di Layar OLED
  oledShowTestMPU(ax, ay, az, magnitude, motion);

// ── EKSEKUSI PENGUJIAN 4: GSR ────────────────────────────────────────────────
#elif defined(TEST_GSR)
  if (millis() - lastSend < 300) return;
  lastSend = millis();

  int raw = readGSR();
  int pct = (int)constrain(map(raw, 0, 4095, 0, 100), 0, 100);

  // Output Terstruktur di Serial Monitor
  Serial.println("┌─── [DEBUG SENSOR GSR] ──────────────────────");
  Serial.printf("│ Nilai ADC Raw: %d / 4095 (Resolusi 12-Bit)\n", raw);
  Serial.printf("│ Konduktansi  : %d %%\n", pct);
  if (raw == 0) {
    Serial.println("│ Indikasi     : ⚠ Elektroda lepas / tidak menempel ke kulit!");
  } else if (pct < 20) {
    Serial.println("│ Status Kulit : Kering / Rileks santai");
  } else if (pct < 55) {
    Serial.println("│ Status Kulit : Normal basal konduktansi");
  } else {
    Serial.println("│ Status Kulit : Konduktansi tinggi (Berkeringat / Respons simpatik)");
  }
  Serial.println("└──────────────────────────────────────────────");

  // Output Terstruktur di Layar OLED
  oledShowTestGSR(raw, pct);

// ── EKSEKUSI PENGUJIAN 5: BLE ────────────────────────────────────────────────
#elif defined(TEST_BLE)
  if (millis() - lastSend < 1000) return;
  lastSend = millis();

  static int txCounter = 0;
  txCounter++;
  uint32_t uptimeSec = millis() / 1000;

  Serial.println("┌─── [DEBUG KONEKSI BLE] ─────────────────────");
  Serial.printf("│ Status BLE   : %s\n", deviceConnected ? "TERHUBUNG KE PONSEL ✓" : "MENUNGGU KONEKSI...");
  Serial.printf("│ Paket Terkirim: %d tx\n", txCounter);
  Serial.printf("│ Waktu Uptime : %lu detik\n", (unsigned long)uptimeSec);

  if (deviceConnected) {
    char testBuf[64];
    snprintf(testBuf, sizeof(testBuf), "{\"test\":1,\"n\":%d,\"ts\":%lu}", txCounter, (unsigned long)uptimeSec);
    pDataChar->setValue((uint8_t *)testBuf, strlen(testBuf));
    pDataChar->notify();
    Serial.printf("│ Payload JSON : %s\n", testBuf);
  }
  Serial.println("└──────────────────────────────────────────────");

  // Output Terstruktur di Layar OLED
  oledShowTestBLE(deviceConnected, txCounter, uptimeSec);

// ── MODE NORMAL: SENSOR FUSION + BLE NOTIFY + OLED DISPLAY ───────────────────
#else
  // 1. Polling sensor detak jantung secepat mungkin
  unsigned long currentIbi = 0;
  float latestHR = readHeartRate(currentIbi);

  unsigned long now = millis();

  // 2. Perbarui Layar OLED setiap OLED_INTERVAL (500 ms)
  if (now - lastOLED >= OLED_INTERVAL) {
    lastOLED = now;

    uint32_t irVal = getIRValue();
    bool worn      = (irVal > 50000);

    float hrvDisp  = calculateHRV();
    float tempDisp = readTemperature();
    int gsrDisp    = readGSR();
    int gsrPct     = (int)constrain(map(gsrDisp, 0, 4095, 0, 100), 0, 100);

    sensors_event_t accelDisp;
    readMotion(accelDisp);

    int stressDisp = 0;
    if (worn && latestHR > 0) {
      stressDisp = (int)round(fusion.compute(
          latestHR, hrvDisp, (float)gsrDisp, tempDisp,
          accelDisp.acceleration.x, accelDisp.acceleration.y, accelDisp.acceleration.z
      ));
    }

    if (!worn) {
      oledShowUnworn(); // Layar peringatan jika alat dilepas
    } else {
      oledShowData(worn, latestHR, (hrvDisp >= 0 ? hrvDisp : 0),
                   tempDisp, gsrPct, stressDisp, deviceConnected);
    }
  }

  // 3. Eksekusi Pengiriman Data BLE & Cetak Serial setiap SEND_INTERVAL (500 ms)
  if (now - lastSend < SEND_INTERVAL) return;
  lastSend = now;

  uint32_t irValue = getIRValue();
  bool isWorn      = (irValue > 50000);

  float hrv        = calculateHRV();
  float temp       = readTemperature();
  int gsrRaw       = readGSR();
  int gsrPct       = (int)constrain(map(gsrRaw, 0, 4095, 0, 100), 0, 100);

  sensors_event_t accel;
  readMotion(accel);

  // Bitmask status kesehatan sensor (1 = Valid/Ready, 0 = Menunggu/Lepas)
  // bit0: Heart Rate, bit1: HRV RMSSD, bit2: GSR, bit3: Suhu, bit4: Gerak
  uint8_t sensorStatus = 0;
  if (isWorn && latestHR > 0) sensorStatus |= (1 << 0);
  if (hrv >= 0)               sensorStatus |= (1 << 1);
  if (gsrRaw > 0)             sensorStatus |= (1 << 2);
  if (temp > 30.0f)           sensorStatus |= (1 << 3);
  sensorStatus |= (1 << 4); // MPU6050 selalu aktif

  float stress = 0;
  if (isWorn && latestHR > 0) {
    // Hitung Fusion Index dengan penyesuaian bobot gerak & Kalman filter
    stress = fusion.compute(latestHR, hrv, (float)gsrRaw, temp,
                            accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  } else {
    // Jika sensor tidak dipakai, reset nilai metrik ke nol
    latestHR   = 0;
    hrv        = -1;
    currentIbi = 0;
    stress     = 0;
  }

  // Serialisasi data JSON untuk transmisi BLE
  StaticJsonDocument<256> doc;
  doc["worn"] = isWorn ? 1 : 0;
  doc["hr"]   = (int)round(latestHR);
  doc["ibi"]  = (int)currentIbi;
  doc["hrv"]  = (hrv >= 0) ? (int)round(hrv) : -1;
  doc["st"]   = (int)round(stress);
  doc["t"]    = round(temp * 10.0f) / 10.0f;
  doc["g"]    = gsrPct;
  doc["ir"]   = irValue;
  doc["ss"]   = sensorStatus;
  doc["ts"]   = (uint32_t)(millis() / 1000);
  doc["ax"]   = round(accel.acceleration.x * 100.0f) / 100.0f;
  doc["ay"]   = round(accel.acceleration.y * 100.0f) / 100.0f;
  doc["az"]   = round(accel.acceleration.z * 100.0f) / 100.0f;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);

  // Kirimkan ke Ponsel jika terkoneksi
  if (deviceConnected) {
    pDataChar->setValue((uint8_t *)buffer, len);
    pDataChar->notify();
  }

  // Cetak ke Serial Monitor untuk keperluan logging dan monitoring langsung
  Serial.printf("[DATA] %s | BLE:%s | Stres:%d\n",
                buffer, deviceConnected ? "CONNECT" : "DISCONNECT", (int)round(stress));
#endif
}