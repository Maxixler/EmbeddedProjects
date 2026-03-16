# ESP32 Gomulu Sistem Projeleri

Bu dizin, **ESP32-DevKitC V4** gelistirme kartı uzerinde **ESP-IDF (Espressif IoT Development Framework)** kullanilarak gelistirilmis kapsamli gomulu sistem projelerini icerir.

## Donanim Bilgileri

| Ozellik | Deger |
|---------|-------|
| **MCU** | ESP32-WROOM-32 (Xtensa LX6 Dual-Core) |
| **Cekirdek Sayisi** | 2 (PRO_CPU + APP_CPU) |
| **Saat Frekansi** | 240 MHz (maks.) |
| **Flash** | 4 MB SPI Flash |
| **SRAM** | 520 KB |
| **WiFi** | 802.11 b/g/n (2.4 GHz) |
| **Bluetooth** | BLE 4.2 + Classic |
| **GPIO** | 34 adet (programlanabilir) |
| **ADC** | 2x 12-bit SAR ADC, 18 kanal |
| **DAC** | 2x 8-bit DAC |
| **UART** | 3 adet |
| **SPI** | 4 adet |
| **I2C** | 2 adet |
| **PWM** | 16 kanal (LEDC) |
| **Timer** | 4x 64-bit genel amacli timer |
| **RTC** | Dusuk guclu RTC, 16 KB RTC SRAM |
| **Calisma Gerilimi** | 3.3V |
| **Sicaklik Araligi** | -40C ~ +85C |

## Proje Listesi

| # | Proje Adi | Konu | Zorluk |
|---|-----------|------|--------|
| 01 | [WiFi-MQTT-SensorHub](01-WiFi-MQTT-SensorHub/) | WiFi baglantisi, MQTT protokolu, sensor veri yayini | Orta-Ileri |
| 02 | [BLE-OTA-Update](02-BLE-OTA-Update/) | BLE GATT Server, OTA firmware guncelleme | Ileri |
| 03 | [FreeRTOS-MultiSensor-Dashboard](03-FreeRTOS-MultiSensor-Dashboard/) | FreeRTOS coklu gorev, kuyruk, semaphore, sensor fuzyonu | Orta-Ileri |
| 04 | [ESP-NOW-Mesh-Network](04-ESP-NOW-Mesh-Network/) | ESP-NOW peer-to-peer ag, mesh topoloji | Ileri |
| 05 | [WebSocket-RealTime-Control](05-WebSocket-RealTime-Control/) | HTTP sunucu, WebSocket, gercek zamanli motor kontrol | Ileri |
| 06 | [DeepSleep-PowerManager](06-DeepSleep-PowerManager/) | Deep sleep modlari, RTC wake-up, guc yonetimi | Orta-Ileri |

## Gelistirme Ortami

| Arac | Aciklama |
|------|----------|
| **Framework** | ESP-IDF v5.x |
| **IDE** | VS Code + ESP-IDF Extension / Terminal |
| **Derleyici** | Xtensa GCC Toolchain |
| **Flash Araci** | esptool.py (ESP-IDF icinde dahili) |
| **Monitor** | idf.py monitor (seri terminal) |
| **Debug** | OpenOCD + JTAG (opsiyonel) |

## Derleme ve Yukleme

```bash
# ESP-IDF ortam degiskenlerini yukle
. $HOME/esp/esp-idf/export.sh

# Proje dizinine gir
cd 01-WiFi-MQTT-SensorHub

# Hedef cipi ayarla
idf.py set-target esp32

# Menuconfig ile yapilandirma (opsiyonel)
idf.py menuconfig

# Derle
idf.py build

# Flash'a yaz ve seri monitor baslat
idf.py -p /dev/ttyUSB0 flash monitor
```

> **Not:** Windows'ta port `/dev/ttyUSB0` yerine `COM3` gibi bir port kullanilir.

## Lisans

Bu projeler egitim amaciyla gelistirilmistir. [MIT Lisansi](../LICENSE) altinda dagitilmaktadir.
