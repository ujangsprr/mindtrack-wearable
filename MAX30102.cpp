#include "MAX30102.h"

static MAX30105 max30102;
static float lastValidHR = 0;     // HR terakhir yang valid (0 = belum ada data)
static unsigned long lastIBI = 0; // IBI terakhir yang valid
static unsigned long lastBeat = 0;

// IBI & HRV — circular buffer 10 sampel
#define IBI_WINDOW 10
static unsigned long ibiBuffer[IBI_WINDOW];
static int ibiIndex = 0;
static int ibiCount = 0;

void initMAX30102() {
  if (!max30102.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 tidak terdeteksi! Cek kabel I2C.");
    while (1)
      delay(100);
  }
  max30102.setup(60,  // LED Brightness — ~12mA
                 4,   // Sample Average
                 2,   // LED Mode (Red + IR)
                 200, // Sample Rate (Hz)
                 411, // Pulse Width (us) — ADC 18-bit
                 4096 // ADC Range
  );
  max30102.setPulseAmplitudeRed(0x3C);
  max30102.setPulseAmplitudeIR(0x3C);
  max30102.setPulseAmplitudeGreen(0);
  Serial.println("[OK] MAX30102 siap.");
}

float readHeartRate(unsigned long &ibi) {
  long irValue = max30102.getIR();
  ibi = lastIBI; // default: kembalikan IBI terakhir yang valid

  if (irValue < 50000) {
    // Sensor tidak dipakai — reset semua state
    lastValidHR = 0;
    lastIBI = 0;
    ibi = 0;
    ibiCount = 0; // reset buffer HRV supaya sesi baru mulai bersih
    ibiIndex = 0;
    return 0;
  }

  if (checkForBeat(irValue)) {
    unsigned long now = millis();
    unsigned long newIBI = now - lastBeat;
    lastBeat = now;

    // Hanya terima IBI fisiologis (300–2000ms = 30–200 BPM)
    if (newIBI >= 300 && newIBI <= 2000) {
      lastIBI = newIBI;
      ibi = newIBI;

      float hr = 60000.0f / (float)newIBI;
      lastValidHR = hr;

      // Simpan ke buffer IBI untuk RMSSD
      ibiBuffer[ibiIndex] = newIBI;
      ibiIndex = (ibiIndex + 1) % IBI_WINDOW;
      if (ibiCount < IBI_WINDOW)
        ibiCount++;
    }
    // Jika IBI di luar range: abaikan beat ini, pertahankan lastValidHR
  }
  return lastValidHR;
}

// HRV menggunakan RMSSD
// Mengembalikan -1 jika data belum cukup (< 2 sampel IBI)
float calculateHRV() {
  if (ibiCount < 2)
    return -1; // Sinyal "belum ada data HRV"

  float sumSqDiff = 0;
  int pairs = 0;
  for (int i = 1; i < ibiCount; i++) {
    int prev = (ibiIndex - ibiCount + i - 1 + IBI_WINDOW) % IBI_WINDOW;
    int curr = (ibiIndex - ibiCount + i + IBI_WINDOW) % IBI_WINDOW;
    float diff = (float)ibiBuffer[curr] - (float)ibiBuffer[prev];
    sumSqDiff += diff * diff;
    pairs++;
  }
  if (pairs == 0)
    return -1;
  return sqrt(sumSqDiff / pairs);
}

uint32_t getIRValue() { return (uint32_t)max30102.getIR(); }
