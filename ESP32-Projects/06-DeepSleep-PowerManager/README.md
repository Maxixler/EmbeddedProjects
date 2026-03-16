# 06 - Deep Sleep Power Manager

## Proje Ozeti

Bu proje, ESP32'nin **derin uyku (deep sleep)** modlarini, **RTC wake-up** kaynaklarini ve **guc yonetimi** stratejilerini kullanarak pil ile calisan IoT cihazlar icin dusuk guc tuketimli bir sistem tasarlar. Cihaz periyodik olarak uyanir, sensorleri okur, verileri RTC belleginde biriktirir ve belirli kosullar saglandiginda WiFi uzerinden verileri gonderir. Pil gerilimi izlenerek adaptif uyku suresi ayarlanir ve kritik seviyede belirsiz suresiz uyku moduna gecilebilir.

---

## Teorik Arka Plan

### ESP32 Guc Modlari

ESP32, farkli guc tuketimi seviyelerine sahip bes ana calisma moduna sahiptir:

```
  Guc Tuketimi (logaritmik olcek):

  ~240 mA |  [======] ACTIVE (WiFi TX)
          |
  ~80 mA  |  [====]   ACTIVE (CPU calisir, WiFi kapali)
          |
  ~20 mA  |  [==]     MODEM SLEEP (CPU calisir, WiFi uyur)
          |
  ~0.8 mA |  [=]      LIGHT SLEEP (CPU durur, RAM korunur)
          |
  ~10 uA  |  [.]      DEEP SLEEP (RTC + RTC bellek korunur)
          |
  ~5 uA   |  [.]      HIBERNATION (sadece RTC timer)
          +----------------------------------------------->
```

### Power Mode Karsilastirma Tablosu

| Mod | Akim | CPU | WiFi/BT | RAM | RTC | Wake-up Kaynaklari |
|-----|------|-----|---------|-----|-----|-------------------|
| **Active** | 80-240 mA | Calisir | Calisir | Korunur | Calisir | - |
| **Modem Sleep** | ~20 mA | Calisir | Uyur | Korunur | Calisir | WiFi interval |
| **Light Sleep** | ~0.8 mA | Durur | Uyur | Korunur | Calisir | Timer, GPIO, Touch, UART |
| **Deep Sleep** | ~10 uA | Kapali | Kapali | Kayip | Calisir | Timer, ext0, ext1, Touch, ULP |
| **Hibernation** | ~5 uA | Kapali | Kapali | Kayip | Timer | Sadece RTC Timer |

### ESP32 Power Domain Diyagrami

```
  +================================================================+
  |                      ESP32 Cip                                  |
  |                                                                 |
  |  +---------------------------+   +---------------------------+  |
  |  |    Digital Domain         |   |    RTC Domain             |  |
  |  |    (Deep Sleep'te KAPALI) |   |    (Deep Sleep'te ACIK)   |  |
  |  |                           |   |                           |  |
  |  |  +-------+  +-------+    |   |  +-------+  +-------+    |  |
  |  |  |CPU 0  |  |CPU 1  |    |   |  | RTC   |  | RTC   |    |  |
  |  |  |(PRO)  |  |(APP)  |    |   |  | Ctrl  |  | Timer |    |  |
  |  |  +-------+  +-------+    |   |  +-------+  +-------+    |  |
  |  |                           |   |                           |  |
  |  |  +-------+  +-------+    |   |  +-------+  +-------+    |  |
  |  |  | Main  |  |Periph-|    |   |  | RTC   |  | ULP   |    |  |
  |  |  | SRAM  |  |erals  |    |   |  |8KB MEM|  | Copro |    |  |
  |  |  |520 KB |  |(SPI,  |    |   |  |(SLOW+ |  | cessor|    |  |
  |  |  |       |  | I2C,  |    |   |  | FAST) |  |       |    |  |
  |  |  +-------+  | UART) |    |   |  +-------+  +-------+    |  |
  |  |              +-------+    |   |                           |  |
  |  |  +-------+  +-------+    |   |  +-------+  +-------+    |  |
  |  |  | WiFi  |  | BT/BLE|    |   |  | RTC   |  | Touch |    |  |
  |  |  | Radio |  | Radio |    |   |  | GPIO  |  | Sensor|    |  |
  |  |  +-------+  +-------+    |   |  +-------+  +-------+    |  |
  |  +---------------------------+   +---------------------------+  |
  +================================================================+

  Deep Sleep'te:
  - Sol taraf (Digital Domain): Tamamen KAPALI
  - Sag taraf (RTC Domain): ACIK (tuketim ~10 uA)
```

### Wake-up Kaynaklari

**1. Timer Wake-up (RTC Timer):**

```
  esp_sleep_enable_timer_wakeup(60 * 1000000ULL);  // 60 saniye

  Zaman Akisi:
  [ACTIVE]--sleep-->[DEEP SLEEP ... 60s ...]--wake-->[ACTIVE]--sleep-->
  | 5s   |          |--- 60 saniye ---|              | 5s   |
  ~80mA              ~10 uA                           ~80mA
```

**2. ext0 Wake-up (Tek GPIO):**

```
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);  // GPIO33 LOW'da uyan

  +----+         +----+
  | 3V3|---[10K]-+----+--- GPIO33 (RTC_GPIO8)
  +----+         |
                 + Buton
                 |
  +----+         |
  | GND|---------+
  +----+

  Buton basilinca -> GPIO33 = LOW -> ESP32 uyanir
```

**3. ext1 Wake-up (Coklu GPIO):**

```
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

  Birden fazla GPIO'yu izler:
  - ANY_HIGH: Herhangi biri HIGH oldugunda uyan
  - ALL_LOW:  Hepsi LOW oldugunda uyan (OR baglanti)

  GPIO25 -+
  GPIO26 -+-- ANY_HIGH --> Herhangi biri '1' -> UYAN
  GPIO27 -+
```

**4. Touch Pad Wake-up:**

```
  esp_sleep_enable_touchpad_wakeup();

  Touch Pad kapasitans degisimi:

  Normal:     Dokunma:
  |           |   +---+
  |           |   |   | (parmak kapasitansi)
  +---+       +---+   +---+
  |   |       |   |   |   |
  |PCB|       |PCB|   |   |
  +---+       +---+---+---+

  Kapasite artisi -> Esik degerin altina dusme -> Wake-up
```

### RTC Bellek Haritalama

```
  RTC Memory Map:
  +---------------------------+
  | RTC FAST Memory (8 KB)    |  -> ULP program kodu
  | 0x3FF8_0000 - 0x3FF8_1FFF |     RTC_DATA_ATTR degiskenler
  +---------------------------+
  | RTC SLOW Memory (8 KB)    |  -> ULP veri bellegi
  | 0x5000_0000 - 0x5000_1FFF |     RTC_SLOW_ATTR degiskenler
  +---------------------------+

  RTC_DATA_ATTR uint32_t boot_count = 0;
  -> Deep sleep sonrasi deger korunur!
  -> Normal restart'ta sifirlanir

  Ornek kullanim:
  RTC_DATA_ATTR sensor_buffer_t sensor_history[MAX_SAMPLES];
  RTC_DATA_ATTR uint16_t sample_index = 0;
  RTC_DATA_ATTR uint32_t total_runtime_ms = 0;
```

### Battery Gerilim Bolucu

```
  LiPo Batarya (3.7V nominal, 4.2V full, 3.0V empty)

  Vbat ----+
           |
          [R1 = 100K]
           |
           +---- ADC GPIO36 (max 3.3V)
           |
          [R2 = 100K]
           |
          GND

  Formul: Vadc = Vbat * R2 / (R1 + R2)
         Vadc = Vbat * 100K / (100K + 100K)
         Vadc = Vbat / 2

  Vbat = 4.2V -> Vadc = 2.1V  (ADC okumasi ~2600)
  Vbat = 3.7V -> Vadc = 1.85V (ADC okumasi ~2290)
  Vbat = 3.0V -> Vadc = 1.5V  (ADC okumasi ~1860)

  Pil Yuzdesi (lineer yaklasiim):
  percent = (Vbat - 3.0) / (4.2 - 3.0) * 100
```

**ESP32 ADC Kalibrasyonu:**

| ADC | Attenuation | Olcum Araligi | Cozunurluk |
|-----|-------------|---------------|------------|
| ADC1_CH0 (GPIO36) | 11 dB | 0 - 3.1V | 12-bit (0-4095) |

---

## Pin Baglanti Semasi

```
    ESP32 DevKitC V4               Cevre Birimleri
    +-------------------+
    |                   |
    |  GPIO36 (ADC)     |<--+--[100K]-- Vbat (+)
    |                   |   |
    |                   |  [100K]  (Gerilim bolucu)
    |                   |   |
    |  GND              |---+--------- Batarya (-)
    |                   |
    |  GPIO33 (ext0)    |---[10K pull-up]---+---[Buton]--- GND
    |                   |                   |
    |                   |            (Wake-up butonu)
    |                   |
    |  GPIO32 (Touch)   |---[bakir pad]  (Touch wake-up)
    |                   |
    |  GPIO4  (DHT22)   |---[4.7K]--- DHT22 DATA
    |                   |
    |  GPIO2  (LED)     |---[330R]--- LED (durum)
    |                   |
    |  3V3              |--- Sensor VCC
    |  GND              |--- Sensor GND
    |                   |
    |  USB (UART0)      |--- PC (Monitor)
    +-------------------+

    Batarya Baglantisi:
    +--------+
    | LiPo   |---(+)---[100K]---+--- GPIO36
    | 3.7V   |                  |
    | 2000mAh|             [100K]
    |        |                  |
    |        |---(-)--------+---+--- GND
    +--------+              |
                     ESP32 GND
```

---

## ESP-IDF Yapilandirma Adimlari

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(deep_sleep_power_manager)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "power_manager.c" "rtc_storage.c"
    INCLUDE_DIRS "." "../inc"
)
```

### menuconfig Ayarlari

| Menu Yolu | Ayar | Deger |
|-----------|------|-------|
| Component config > Power Management | Enable PM | Yes |
| Component config > ESP32-specific | RTC clock source | Internal 150kHz |
| Component config > Log output | Default log verbosity | Info |
| Serial flasher config | Flash size | 4 MB |

### Derleme ve Yukleme

```bash
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

---

## Kodun Calisma Mantigi

```
                    ESP32 Acilis (Boot)
                          |
                          v
                  +------------------+
                  | Wake-up sebebini |
                  | belirle          |
                  +--------+---------+
                           |
              +------------+------------+
              |            |            |
         TIMER         GPIO          TOUCH
              |            |            |
              v            v            v
        +---------+  +---------+  +---------+
        | Sensor  |  | Buton   |  | Touch   |
        | oku,    |  | basild,i|  | algilandi|
        | RTC'ye  |  | WiFi ac |  | LED yak |
        | kaydet  |  | veri    |  |         |
        +---------+  | gonder  |  +---------+
              |       +---------+       |
              v            |            v
        +---------+        |      +---------+
        | Buffer  |        v      | Tekrar  |
        | dolu mu?|   +---------+ | uyku    |
        +----+----+   | MQTT    | +---------+
             |        | publish |
        HAYIR|EVET    +---------+
             |             |
             v             v
        +---------+  +---------+
        | Tekrar  |  | Buffer  |
        | uyku    |  | temizle |
        | (60s)   |  +---------+
        +---------+        |
                           v
                     +---------+
                     | Uyku    |
                     | (60s)   |
                     +---------+

  === Pil Kontrolu (Her uyanmada) ===

        +-------------------+
        | Pil gerilimi oku  |
        +---------+---------+
                  |
        +---------+---------+
        |                   |
     >= 20%              < 20%
        |                   |
        v                   v
  +-----------+     +---------------+
  | Normal    |     | < 10%?        |
  | calisma   |     +-------+-------+
  +-----------+             |
                     EVET   |  HAYIR
                       |    |
                       v    v
               +--------+ +---------+
               |Belirsiz | | Uyku    |
               |uyku     | | suresini|
               |(sadece  | | 2x artir|
               |GPIO ile | +---------+
               |uyan)    |
               +--------+
```

---

## RTC Bellek Kullanimi

| Degisken | Tip | Boyut | Aciklama |
|----------|-----|-------|----------|
| `boot_count` | uint32_t | 4B | Toplam boot sayisi |
| `sample_index` | uint16_t | 2B | Mevcut ornek indeksi |
| `sensor_buffer[]` | struct array | ~512B | Son N sensor okumasi |
| `wake_history[]` | struct array | ~128B | Son 8 wake-up kaydesi |
| `total_active_ms` | uint32_t | 4B | Toplam aktif sure |
| `crc32` | uint32_t | 4B | Veri butunlugu kontrolu |
| `last_send_boot` | uint32_t | 4B | Son veri gonderme boot'u |

---

## Test Proseduru

### 1. Seri Monitor ile Test

```bash
idf.py -p COM3 flash monitor
```

**Beklenen Cikti (Ilk boot):**

```
I (300) main: === Deep Sleep Power Manager ===
I (305) main: Boot #1, wake-up cause: POWER_ON
I (310) power: Battery: 3.85V (70%)
I (315) power: Sleep interval: 60s (normal)
I (400) sensor: DHT22: temp=24.3C, humid=51.0%
I (405) rtc: Sample saved to RTC buffer (index=0/8)
I (410) main: Buffer not full (1/8), going to sleep...
I (415) power: Entering deep sleep for 60 seconds
```

**Beklenen Cikti (5. boot - buffer dolu):**

```
I (300) main: Boot #5, wake-up cause: TIMER
I (310) power: Battery: 3.82V (68%)
I (400) sensor: DHT22: temp=24.5C, humid=52.1%
I (405) rtc: Sample saved to RTC buffer (index=4/5)
I (410) main: Buffer full! Connecting WiFi to send data...
I (2500) wifi: Connected, IP: 192.168.1.50
I (3000) mqtt: Published 5 samples to sensor/data
I (3005) rtc: Buffer cleared, index reset to 0
I (3010) main: Data sent, going to sleep...
```

**Beklenen Cikti (GPIO wake-up):**

```
I (300) main: Boot #12, wake-up cause: EXT0 (GPIO33)
I (305) main: Button pressed - sending all data immediately
```

### 2. Multimetre ile Akim Olcumu

```
  Olcum Duzenegi:
  Batarya (+) ---> Multimetre (uA modu) ---> ESP32 VIN

  Beklenen Degerler:
  +-------------------+-------------+
  | Durum             | Akim        |
  +-------------------+-------------+
  | Active (WiFi TX)  | ~150-240 mA |
  | Active (CPU only) | ~50-80 mA   |
  | Deep Sleep        | ~10-15 uA   |
  | Hibernation       | ~5 uA       |
  +-------------------+-------------+
```

### 3. Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| Timer wake-up | 60s sonra uyanir | [ ] |
| GPIO wake-up | Butonla aninda uyanir | [ ] |
| Touch wake-up | Dokunmayla uyanir | [ ] |
| RTC bellek korunma | boot_count her boot'ta artar | [ ] |
| Sensor veri biriktirme | Buffer'a kayit yapilir | [ ] |
| Buffer dolu -> WiFi | 5 okumada WiFi acar, gonderir | [ ] |
| Pil izleme | ADC ile gerilim dogru okunur | [ ] |
| Dusuk pil uyarisi | %20 altinda uyku suresi artar | [ ] |
| Kritik pil | %10 altinda belirsiz uyku | [ ] |
| Deep sleep akim | Multimetre ile ~10 uA olculur | [ ] |
| CRC32 kontrolu | Bozuk veri tespit edilir | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| Deep sleep'e girmiyor | Aktif timer/interrupt | Tum periferleri kapat, WiFi disconnect et |
| Uyku akimi yuksek (>100uA) | GPIO yapilari dagitilmamis | `gpio_deep_sleep_hold_en()` ve izolasyon aktif et |
| RTC bellegi sifirlanmis | Cold boot olmus | `esp_reset_reason()` ile kontrol et, brownout olabilir |
| Timer wake-up calismaz | Timer degeri cok kucuk | Minimum 100us, mikrosaniye cinsinden verin |
| ext0 wake-up yok | RTC GPIO degil | Sadece RTC GPIO destekli pinler (GPIO 0,2,4,12-15,25-27,32-39) |
| ext1 mask yanlis | Bit maskesi hatasi | `(1ULL << GPIO_NUM_XX)` formatini kullan |
| Touch wake-up yok | Esik degeri yanlis | `touch_pad_set_thresh()` ile kalibrasyon yap |
| ADC degeri tutarsiz | Attenuation yanlis | 11dB attenuation ile 0-3.1V arasi kullan |
| Pil yuzdesi yanlis | Lineer hesaplama | LiPo discharge egrisi non-lineer, LUT kullan |
| Brownout reset | Pil gerilimi cok dusuk | brownout detector esigini 2.44V'dan 2.27V'a dusur |
| Boot loop | Uyanma sonrasi hata | Hata sayacini RTC'de tut, N hatada rota degistir |

---

## Pil Omru Hesaplama Ornegi

```
  Senaryo: 2000 mAh LiPo pil, 60s uyku, 5s aktif

  Deep Sleep akim:   10 uA
  Active akim:       100 mA (ortalama, WiFi dahil)

  Bir dongudeki tuketim:
  Q_sleep = 10 uA * 60s = 600 uAs = 0.167 uAh
  Q_active = 100 mA * 5s = 500 mAs = 138.9 uAh
  Q_total = 139.1 uAh / dongu

  Dongu periyodu: 65 saniye
  Dongu/saat: 3600 / 65 = 55.4 dongu
  Saatlik tuketim: 55.4 * 139.1 uAh = 7.7 mAh

  Pil omru: 2000 mAh / 7.7 mAh = ~260 saat = ~10.8 gun

  Karsilastirma (sadece aktif mod):
  2000 mAh / 100 mA = 20 saat = ~0.8 gun

  Deep sleep ile pil omru ~13x artar!
```

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP-IDF Sleep Modes](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html) | Deep sleep API referansi |
| [ESP32 Technical Reference - RTC](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf) | RTC ve power management detaylari |
| [ESP-IDF ULP Coprocessor](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ulp.html) | ULP programlama rehberi |
| [ESP32 Power Consumption](https://www.espressif.com/sites/default/files/9b-esp32-low_power_solutions_en.pdf) | Espressif guc optimizasyon kilavuzu |
| [ESP-IDF ADC API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html) | ADC driver API referansi |
| [LiPo Battery Guide](https://learn.adafruit.com/li-ion-and-lipoly-batteries) | Lityum polimer pil rehberi |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
