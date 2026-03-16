# 03 - FreeRTOS Multi-Sensor Dashboard

## Proje Ozeti

Bu proje, ESP32 uzerinde **FreeRTOS** isletim sisteminin gelismis ozelliklerini kullanarak coklu sensor verilerini toplayan, isleyen ve OLED ekranda goruntuleyen bir **gercek zamanli sensor dashboard** uygulamasidir. Dort ayri FreeRTOS gorevi (task), kuyruklar (queue), mutex, semaphore, event group ve yazilim zamanlayicisi (software timer) kullanilarak gosterilmektedir. Sensorler arasinda BME280 (sicaklik/nem/basinc), MPU6050 (ivmeolcer/jiroskop) ve potansiyometre (ADC) bulunur. Sensor fuzyonu icin hareketli ortalama filtresi ve complementary filtre uygulanmistir.

---

## Teorik Arka Plan

### FreeRTOS Gorev Durum Makinesi

FreeRTOS'ta her gorev (task) dort durumdan birinde bulunur:

```
                    xTaskCreate()
                         |
                         v
    +------------------------------------------+
    |               READY                      |
    |  (Calistirilmaya hazir, scheduler        |
    |   sirasini bekliyor)                     |
    +----------+------------------+------------+
               |                  ^
   Scheduler   |                  | Olay gerceklesti
   sec (en     |                  | (kuyruk, semaphore,
   yuksek      |                  |  timer, vTaskDelay
   oncelik)    v                  |  suresi doldu)
    +----------+------------------+------------+
    |              RUNNING                     |
    |  (CPU uzerinde aktif olarak             |
    |   calisiyor - tek gorev)                |
    +----------+--+---------------+------------+
               |  |               |
  Daha yuksek  |  | Bekleme       | vTaskSuspend()
  oncelikli    |  | (queue,       |
  gorev hazir  |  |  semaphore,   |
               |  |  vTaskDelay)  |
               v  v               v
    +----------+------+   +------+-------------+
    |    READY        |   |    BLOCKED          |
    | (Tekrar siraya  |   | (Bir olayun         |
    |  girer)         |   |  gerceklesmesini    |
    +-----------------+   |  bekliyor)          |
                          +---------+-----------+
                                    |
                          vTaskResume()
                                    |
                                    v
                          +---------+-----------+
                          |    SUSPENDED        |
                          | (Askiya alinmis,    |
                          |  scheduler tarafindan|
                          |  secilmez)          |
                          +---------------------+
```

### FreeRTOS Zamanlayici (Scheduler) Algoritmasi

ESP-IDF'deki FreeRTOS, **oncelikli preemptive scheduling** kullanir:

1. **Preemptive:** Yuksek oncelikli gorev hazir oldugunda, dusuk oncelikli gorev kesintiye ugrar.
2. **Time-slicing:** Ayni oncelikli gorevler arasinda round-robin ile gecis yapilir.
3. **Dual-core:** ESP32'de iki cekirdek (PRO_CPU, APP_CPU) uzerinde bagımsız zamanlanabilir.

| Ozellik | Deger |
|---------|-------|
| Tick rate | 1000 Hz (1 ms) |
| Preemption | Aktif |
| Time-slicing | Aktif |
| Idle task | Her cekirdekte bir tane |
| Tick hook | Destekleniyor |

### Oncelik Terslenmesi (Priority Inversion) ve Mutex

**Problem:** Dusuk oncelikli gorev bir kaynak tutar, yuksek oncelikli gorev bu kaynagi bekler, orta oncelikli gorev dusuk oncelikliden once calisir.

```
  Oncelik
    ^
    |  Yuksek (H)    ----[BLOCKED]----[RUNNING]-->
    |                     ^              ^
    |                     |              | Mutex serbest
    |  Orta (M)      =====[RUNNING]=====         (M, H'yi bloke eder!)
    |                ^
    |  Dusuk (L)  ---[Mutex al]---[BLOCKED]---[Mutex birak]-->
    |
    +-------------------------------------------------> Zaman

  Mutex ile Oncelik Kalitimi (Priority Inheritance):
  L, mutex'i tuttugunda H beklerken, L'nin onceligi
  gecici olarak H seviyesine yukseltilir. Boylece M,
  L'yi bloke edemez.
```

### Queue (Kuyruk) Yapisi

```
  xQueueCreate(length=5, item_size=sizeof(data_t))

  +-----+-----+-----+-----+-----+
  |  0  |  1  |  2  |  3  |  4  |
  +--+--+--+--+-----+-----+-----+
     ^     ^
     |     +-- Head (okuma noktasi)
     +-- Tail (yazma noktasi)

  xQueueSend():  Veriyi kopyalar (derin kopya), tail ilerler
  xQueueReceive(): Veriyi kopyalar, head ilerler
  Kuyruk doluysa: Gorev BLOCKED durumuna gecer (timeout ile)
  Kuyruk bossa:   Okuyan gorev BLOCKED durumuna gecer
```

### Mutex vs Semaphore Karsilastirmasi

| Ozellik | Mutex | Binary Semaphore |
|---------|-------|------------------|
| Amac | Kaynak koruma | Senkronizasyon |
| Sahiplik | Var (alan gorev birakir) | Yok (farkli gorev verebilir) |
| Oncelik kalitimi | Var | Yok |
| Recursion | xSemaphoreCreateRecursiveMutex | Desteklenmiyor |
| ISR'dan kullanim | Hayir | Evet (xSemaphoreGiveFromISR) |
| Tipik kullanim | I2C/SPI bus paylasimi | Gorev senkronizasyonu |

### Complementary Filtre

MPU6050'den alinan ivmeolcer ve jiroskop verilerini birlestirerek kararli aci hesabi:

```
  Formul:
  angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle

  Burada:
    alpha     = 0.96 (tipik deger, jiroskopa guven orani)
    gyro_rate = jiroskop aci hizi [derece/saniye]
    dt        = ornekleme periyodu [saniye]
    accel_angle = atan2(ax, az) * 180 / PI  [derece]

  Neden iki sensor?
  +------------------+----------------------------+
  | Sensor           | Avantaj       | Dezavantaj  |
  +------------------+---------------+-------------+
  | Ivmeolcer        | Uzun vadede   | Titresime   |
  | (accelerometer)  | dogru         | duyarli     |
  +------------------+---------------+-------------+
  | Jiroskop         | Kisa vadede   | Drift       |
  | (gyroscope)      | dogru, puruz- | birikir     |
  |                  | suz           | (zaman icinde)|
  +------------------+---------------+-------------+
```

### Hareketli Ortalama Filtresi

```
  Moving Average (N orneklik pencere):

  y[n] = (1/N) * SUM(x[n-k], k=0..N-1)

  Ornek (N=4):
  Giris: [23.1, 23.5, 22.8, 23.3, 23.7, 22.9, ...]
  Cikis: [ -  ,  -  ,  -  , 23.18, 23.33, 23.18, ...]
                              (23.1+23.5+22.8+23.3)/4

  Ring buffer ile implementasyon:
  +------+------+------+------+
  | x[0] | x[1] | x[2] | x[3] |  <- Pencere
  +------+------+------+------+
     ^
     |-- Yazma indeksi (dairesel)
```

### BME280 Sensor (I2C)

| Parametre | Deger |
|-----------|-------|
| I2C Adres | 0x76 (SDO=GND) veya 0x77 (SDO=VCC) |
| Sicaklik | -40 ~ +85 C  (+-1 C) |
| Nem | 0 ~ 100% RH (+-3%) |
| Basinc | 300 ~ 1100 hPa (+-1 hPa) |
| Olcum Suresi | ~8 ms (forced mode) |

### MPU6050 Sensor (I2C)

| Parametre | Deger |
|-----------|-------|
| I2C Adres | 0x68 (AD0=GND) veya 0x69 (AD0=VCC) |
| Ivmeolcer Araligi | +-2g, +-4g, +-8g, +-16g |
| Jiroskop Araligi | +-250, +-500, +-1000, +-2000 dps |
| ADC Cozunurlugu | 16-bit |
| Ornekleme Hizi | 1 kHz (maks) |

### SSD1306 OLED Ekran (I2C)

| Parametre | Deger |
|-----------|-------|
| I2C Adres | 0x3C |
| Cozunurluk | 128 x 64 piksel |
| Renk | Tek renk (beyaz/mavi) |
| Frame buffer | 128 * 64 / 8 = 1024 byte |
| Yenileme Hizi | ~60 FPS (maks) |

---

## Pin Baglanti Semasi

```
    ESP32 DevKitC V4                    Cevresel Birimler
    +------------------+
    |                  |
    |  GPIO21 (SDA)    |----+-------- SDA ---- BME280 SDA
    |                  |    |-------- SDA ---- MPU6050 SDA
    |                  |    +-------- SDA ---- SSD1306 SDA
    |                  |
    |  GPIO22 (SCL)    |----+-------- SCL ---- BME280 SCL
    |                  |    |-------- SCL ---- MPU6050 SCL
    |                  |    +-------- SCL ---- SSD1306 SCL
    |                  |
    |  GPIO34 (ADC1_6) |----[POT]---- Potansiyometre orta bacak
    |                  |              (uc bacaklar: 3V3 ve GND)
    |                  |
    |  GPIO2  (LED)    |----[330R]---- LED (+) ---- GND
    |                  |
    |  3V3             |----+-------- VCC ---- BME280
    |                  |    |-------- VCC ---- MPU6050
    |                  |    +-------- VCC ---- SSD1306
    |                  |
    |  GND             |----+-------- GND ---- BME280
    |                  |    |-------- GND ---- MPU6050
    |                  |    |-------- GND ---- SSD1306
    |                  |    +-------- GND ---- LED (-)
    |                  |
    |  USB (UART0)     |-------- PC (Monitor/Flash)
    +------------------+

    I2C Adresleri:
    - BME280  : 0x76 (SDO -> GND)
    - MPU6050 : 0x68 (AD0 -> GND)
    - SSD1306 : 0x3C

    Pull-up: I2C SDA/SCL hatlarina 4.7K Ohm (3V3'e)
```

---

## Task Oncelik ve Zamanlama Tablosu

| Gorev | Oncelik | Stack (word) | Periyot | Aciklama |
|-------|---------|-------------|---------|----------|
| sensor_read_task | 5 (En yuksek) | 4096 | 100 ms | Sensor okuma |
| data_process_task | 4 | 4096 | Queue-driven | Filtre uygula |
| alert_task | 3 | 2048 | 200 ms | Esik izleme, LED |
| display_task | 2 (En dusuk) | 4096 | 500 ms | OLED guncelle |

---

## Gorevler Arasi Iletisim Diyagrami

```
  +-------------------+       raw_data_queue       +-------------------+
  |  sensor_read_task |  ========================> |  data_process_task|
  |  (Pri 5, 100ms)   |    (sensor_raw_data_t)     |  (Pri 4)          |
  +---+----------+----+                            +---+----------+----+
      |          |                                     |          |
      |          |                                     |          |
  [I2C Mutex]  [I2C Mutex]                     [Event Group]   [Data Mutex]
      |          |                                     |          |
      v          v                                     v          |
  +--------+ +--------+                        +------+------+   |
  | BME280 | | MPU6050|                        | alert_task  |   |
  +--------+ +--------+                        | (Pri 3)     |   |
                                               | LED toggle  |   |
                                               +-------------+   |
                                                                  |
                                          processed_data_queue    |
                                     +----------------------------+
                                     |    (processed_data_t)
                                     v
                              +------+------+
                              | display_task|
                              | (Pri 2)     |
                              | SSD1306     |
                              +-------------+
                                     |
                                 [I2C Mutex]
                                     |
                                     v
                              +-------------+
                              | SSD1306     |
                              | OLED 128x64 |
                              +-------------+

  Senkronizasyon Primitifleri:
  +-----------------------+-----------------------------------+
  | Primitif              | Amac                              |
  +-----------------------+-----------------------------------+
  | g_raw_data_queue      | Ham sensor -> isleme              |
  | g_processed_data_queue| Islenmis veri -> ekran            |
  | g_i2c_mutex           | I2C bus paylasimi (3 cihaz)       |
  | g_data_mutex          | Paylasilmis veri koruma           |
  | g_sync_semaphore      | Baslangic senkronizasyonu         |
  | g_alert_event_group   | Alarm sinyal bitlerI              |
  | g_watchdog_timer      | Gorev izleme (software timer)     |
  +-----------------------+-----------------------------------+
```

---

## ESP-IDF Yapilandirma Adimlari

### CMakeLists.txt

```cmake
# Ana CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(freertos_multisensor_dashboard)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "app_tasks.c" "sensor_fusion.c" "display_driver.c"
    INCLUDE_DIRS "." "../inc"
)
```

### menuconfig Ayarlari

```bash
idf.py menuconfig
```

| Menu Yolu | Ayar | Deger |
|-----------|------|-------|
| Component config > FreeRTOS | Tick rate (Hz) | 1000 |
| Component config > FreeRTOS | Enable FreeRTOS trace | Aktif |
| Component config > ESP System | Main task stack size | 4096 |
| Component config > Driver | I2C | Aktif |

### Derleme ve Yukleme

```bash
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

---

## Kodun Calisma Mantigi

```
                      app_main()
                          |
                          v
                +-------------------+
                | Sistem bilgisi    |
                | log (chip, heap)  |
                +--------+----------+
                         |
                         v
                +-------------------+
                | I2C Master Init   |
                | (GPIO21/22, 100K) |
                +--------+----------+
                         |
                         v
                +-------------------+
                | FreeRTOS Primitif |
                | Olusturma:        |
                | - 2 Queue         |
                | - 2 Mutex         |
                | - 1 Semaphore     |
                | - 1 Event Group   |
                | - 1 SW Timer      |
                +--------+----------+
                         |
                         v
                +-------------------+
                | Task Olusturma:   |
                | - sensor_read     |
                | - data_process    |
                | - alert           |
                | - display         |
                +--------+----------+
                         |
                         v
                +-------------------+
                | Sync Semaphore    |
                | Give (basla!)     |
                +--------+----------+
                         |
         +===============+===============+
         |               |               |
    +----v----+    +-----v-----+   +-----v-----+
    | sensor  |    | process   |   | display   |
    | read    |    | task      |   | task      |
    | task    |    |           |   |           |
    +----+----+    +-----+-----+   +-----+-----+
         |               |               |
    Sonsuz dongu:   Queue'dan al:   Queue'dan al:
    1. Mutex al     1. Raw veri oku  1. Processed al
    2. BME280 oku   2. Mov.avg uygula 2. SSD1306'ya yaz
    3. MPU6050 oku  3. Comp.filter   3. Frame guncelle
    4. Mutex birak  4. Esik kontrol  4. 500ms bekle
    5. ADC oku      5. Event set
    6. Queue'a gonder 6. Processed
    7. 100ms bekle     queue'a gonder
```

---

## Test Proseduru

### 1. Donanim Kurulumu

BME280, MPU6050 ve SSD1306'yi I2C bus'a baglayin (yukaridaki pin semasina bakin).

### 2. Flash ve Monitor

```bash
idf.py -p COM3 flash monitor
```

### 3. Beklenen Seri Monitor Ciktisi

```
I (325) main: ========================================
I (330) main:  FreeRTOS Multi-Sensor Dashboard
I (335) main:  Version 1.0.0
I (340) main: ========================================
I (345) main: ESP-IDF version : v5.1
I (350) main: Chip model      : ESP32 (rev 1)
I (355) main: CPU cores       : 2
I (360) main: Free heap       : 270000 bytes
I (370) main: I2C master initialised (port=0, SDA=GPIO21, SCL=GPIO22)
I (380) main: All tasks started:
I (385) main: +------------------+-----+-------+--------+
I (390) main: | Task             | Pri | Stack | Period |
I (395) main: +------------------+-----+-------+--------+
I (400) main: | sensor_read      |  5  |  4096 | 100 ms |
I (405) main: | data_process     |  4  |  4096 | (queue)|
I (410) main: | alert            |  3  |  2048 | 200 ms |
I (415) main: | display          |  2  |  4096 | 500 ms |
I (420) main: +------------------+-----+-------+--------+
I (430) main: Sync semaphore released - sensor pipeline started
```

### 4. Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| I2C baglanti | BME280 + MPU6050 + SSD1306 algılanır | [ ] |
| BME280 okuma | Sicaklik, nem, basinc degerleri logda | [ ] |
| MPU6050 okuma | Ivme ve jiroskop degerleri logda | [ ] |
| ADC okuma | Potansiyometre degeri (0-4095) logda | [ ] |
| Hareketli ortalama | Filtrelenmis degerler purussuz | [ ] |
| Complementary filtre | Roll/pitch aci degerleri stabil | [ ] |
| OLED ekran | Sensor degerleri ekranda gorunur | [ ] |
| LED alarm | Esik asildiginda LED yanar | [ ] |
| I2C mutex | Coklu gorev I2C erisiminde hata yok | [ ] |
| Queue iletisim | Veriler gorevler arasi dogru akar | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| I2C cihaz algilanmiyor | Kablo baglantisi | SDA/SCL kablolarini ve pull-up direncleri kontrol et |
| BME280 0x76 yanit yok | Yanlis adres | SDO pininin GND'ye baglandigini dogrula |
| MPU6050 veri donmuyor | Uyku modunda | WHO_AM_I registeri (0x75) okunarak dogrula |
| OLED ekran bos | Init komutu basarisiz | I2C adresinin 0x3C oldugunu dogrula |
| Gorevler baslamiyor | Stack yetersiz | Stack boyutlarini artir (configSTACK_DEPTH_TYPE) |
| Queue tasiyor | Uretici tuketiciden hizli | Queue derinligini artir veya periyodu ayarla |
| Priority inversion | Mutex eksik | I2C erisiminde mutex kullanildigini kontrol et |
| Watchdog tetikleniyor | Gorev bloke olmus | Engellenen gorevi tespit et, timeout degerlerini kontrol et |
| ADC degeri sabit | Potansiyometre baglantisi | GPIO34'e dogru baglandigini ve attenuation ayarini kontrol et |
| Complementary filter kayiyor | Yanlis alpha degeri | alpha=0.96 degerini ve dt hesabini dogrula |
| Heap yetersiz | Cok fazla gorev/kuyruk | esp_get_free_heap_size() ile izle, stack'leri kucult |
| Gorev onceligi sorunu | Yanlis oncelik | Sensor > Process > Alert > Display sirasini koru |

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP-IDF FreeRTOS API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html) | FreeRTOS API referansi |
| [FreeRTOS Resmi Dokumantasyon](https://www.freertos.org/Documentation/RTOS_book.html) | FreeRTOS kitap ve rehber |
| [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf) | BME280 sensor datasheet |
| [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) | MPU6050 sensor datasheet |
| [SSD1306 Datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf) | SSD1306 OLED surucu datasheet |
| [Complementary Filter Tutorial](https://www.pieter-jan.com/node/11) | Complementary filtre aciklamasi |
| [ESP-IDF I2C Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html) | I2C surucu API referansi |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
