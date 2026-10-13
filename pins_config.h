#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// =============================================================
//  MINDTRACK — Konfigurasi Pin Terpusat
//  Board target: Seeed Studio XIAO ESP32-C3
//
//  Ganti nilai di sini jika pindah ke board lain.
//  Tidak perlu mengubah file sensor (.cpp/.h) sama sekali.
// =============================================================

// ── I2C Bus ──────────────────────────────────────────────────
// Digunakan oleh: MAX30102, MLX90614, MPU6050, OLED SSD1306
#define PIN_I2C_SDA 6 // GPIO6 → SDA
#define PIN_I2C_SCL 7 // GPIO7 → SCL

// ── Analog Input ─────────────────────────────────────────────
// Digunakan oleh: GSR (Galvanic Skin Response)
#define PIN_GSR 2 // GPIO2 / A0 — valid analog: GPIO2,3,4,5

// ── OLED SSD1306 ─────────────────────────────────────────────
// Menggunakan I2C bus yang sama (SDA/SCL di atas)
#define OLED_ADDR 0x3C // Alamat I2C default SSD1306 (0x3C atau 0x3D)
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// =============================================================

#endif // PINS_CONFIG_H
