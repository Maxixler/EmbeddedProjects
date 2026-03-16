# 02 - BLE GATT Server + OTA Firmware Update

## Proje Ozeti

Bu proje, ESP32 mikrodenetleyicisi uzerinde **Bluetooth Low Energy (BLE) GATT sunucusu** ve **OTA (Over-The-Air) firmware guncelleme** sistemi gelistirmektedir. NimBLE yigini (stack) kullanilarak ozel GATT servisleri tanimlanmis, sensor verisi okuma/bildirim ve kablosuz firmware guncelleme yetenekleri saglanmistir.

Proje iki temel bileseneden olusur:

1. **BLE GATT Sunucusu**: Sensor verilerini BLE uzerinden istemcilere sunan ve OTA guncelleme komutlarini alan ozel GATT servisleri.
2. **OTA Guncelleme Yoneticisi**: BLE uzerinden alinan firmware parcalarini (chunk) ESP32'nin flash bellegindeki OTA bolumune yazan, SHA-256 ile dogrulayan ve yeni firmware'i aktif hale getiren durum makinesi.

Kullanim senaryolari:
- Sahada calisirken firmware guncelleme (USB baglantisi gerektirmeden)
- BLE uzerinden sensor verisi izleme
- Uzaktan cihaz yonetimi ve firmware surum kontrolu

---

## Teorik Arka Plan

### Bluetooth Low Energy (BLE) Protokol Yigini

BLE, dusuk guclu kablosuz iletisim icin tasarlanmis bir Bluetooth protokoludur. Klasik Bluetooth'tan farkli olarak, cok dusuk enerji tuketimi ile kisa veri paketleri gondermeye optimize edilmistir.

**BLE Protokol Katmanlari:**

```
+----------------------------------------------------------+
|                    Uygulama Katmani                      |
|              (GATT Profilleri ve Servisler)               |
+----------------------------------------------------------+
|                    Host Katmani                          |
|  +-------------------+  +-----------------------------+  |
|  | Generic Access     |  | Generic Attribute           |  |
|  | Profile (GAP)      |  | Profile (GATT)              |  |
|  | - Kesfetme         |  | - Servis tanimlama          |  |
|  | - Baglanti yonetimi|  | - Karakteristik okuma/yazma |  |
|  +-------------------+  +-----------------------------+  |
|  +-------------------+  +-----------------------------+  |
|  | Security Manager   |  | Attribute Protocol (ATT)    |  |
|  | Protocol (SMP)     |  | - Request/Response          |  |
|  | - Eslestirme       |  | - Notification/Indication   |  |
|  | - Sifreleme        |  | - MTU degisimi              |  |
|  +-------------------+  +-----------------------------+  |
|  +----------------------------------------------------+  |
|  | Logical Link Control & Adaptation Protocol (L2CAP)  |  |
|  | - Kanal coklamalama (multiplexing)                  |  |
|  | - Parcalama ve birlestime (fragmentation)           |  |
|  +----------------------------------------------------+  |
|  +----------------------------------------------------+  |
|  | Host Controller Interface (HCI)                     |  |
|  | - Host ile Controller arasi komut/veri iletisimi    |  |
|  +----------------------------------------------------+  |
+----------------------------------------------------------+
|                  Controller Katmani                      |
|  +----------------------------------------------------+  |
|  | Link Layer (LL)                                     |  |
|  | - Advertising / Scanning / Connection               |  |
|  | - Paket formatlama (PDU)                            |  |
|  | - Frekans atlamali yayilim (FHSS)                   |  |
|  +----------------------------------------------------+  |
|  +----------------------------------------------------+  |
|  | Physical Layer (PHY)                                |  |
|  | - 2.4 GHz ISM bandi                                |  |
|  | - 1 Mbps (BLE 4.x) / 2 Mbps (BLE 5.0)             |  |
|  | - GFSK modülasyonu                                  |  |
|  +----------------------------------------------------+  |
+----------------------------------------------------------+
```

### GAP (Generic Access Profile)

GAP, BLE cihazlarinin nasil kesfedilecegini, baglanacagini ve guvenlik parametrelerinin nasil yapilandirilacagini tanimlar.

**GAP Rolleri:**

| Rol | Aciklama | Bu Projede |
|-----|----------|------------|
| Broadcaster | Yalnizca advertise yapar, baglanti kabul etmez | - |
| Observer | Yalnizca scan yapar, baglanti baslatmaz | - |
| Peripheral | Advertise yapar, baglanti kabul eder (sunucu) | ESP32 |
| Central | Scan yapar, baglanti baslatir (istemci) | nRF Connect |

**Advertising Sureci:**

```
  ESP32 (Peripheral)                     Mobil Cihaz (Central)
       |                                        |
       |======= ADV_IND (Ch 37) =============>|
       |======= ADV_IND (Ch 38) =============>|  (Scan)
       |======= ADV_IND (Ch 39) =============>|
       |                                        |
       |<------ SCAN_REQ ----------------------|
       |------- SCAN_RSP --------------------->|
       |                                        |
       |<------ CONNECT_IND -------------------|
       |                                        |
       |<====== Baglanti Kuruldu ==============>|
       |                                        |
       |<------ MTU Exchange Request ----------|
       |------- MTU Exchange Response -------->|
       |        (MTU = 512)                     |
       |                                        |
       |<------ Service Discovery -------------|
       |------- Service List ----------------->|
       |                                        |
```

**Advertising Data Formati (maksimum 31 byte):**

```
+------+------+----------+------+------+------+---...---+------+
| Len1 | Type1|  Data1   | Len2 | Type2| Flag | Name    | ...  |
| (1B) | (1B) | (Len1-1B)| (1B) | (1B) | (1B) | (N B)   |      |
+------+------+----------+------+------+------+---...---+------+

Ornek (bu proje):
  02 01 06              -> Flags: General Discoverable + BR/EDR Not Supported
  0F 09 45 53 50 33 32  -> Complete Local Name: "ESP32-OTA-GATT"
        2D 4F 54 41 2D
        47 41 54 54
```

### ATT (Attribute Protocol)

ATT, BLE'nin en temel veri erisim protokoludur. Tum GATT islemleri ATT uzerinde gerceklestirilir.

**ATT PDU (Protocol Data Unit) Yapisi:**

```
+----------+-----------+---...---+
| Opcode   | Parameters|  Value  |
| (1 byte) | (0-N B)   | (0-N B) |
+----------+-----------+---...---+

Opcode Degerleri:
  0x02 = Exchange MTU Request
  0x03 = Exchange MTU Response
  0x08 = Read By Type Request
  0x09 = Read By Type Response
  0x0A = Read Request
  0x0B = Read Response
  0x12 = Write Request
  0x13 = Write Response
  0x1B = Handle Value Notification
  0x1D = Handle Value Indication
  0x52 = Write Command (No Response)
```

**MTU (Maximum Transmission Unit):**

```
  Varsayilan MTU: 23 byte
  Efektif payload: MTU - 3 = 20 byte (ATT header overhead)

  Bu projede istenen MTU: 512 byte
  Efektif payload: 512 - 3 = 509 byte

  OTA throughput karsilastirmasi:
  +----------+----------+------------------+
  | MTU      | Payload  | 100KB Guncelleme |
  +----------+----------+------------------+
  | 23 byte  | 20 byte  | ~5000 paket      |
  | 185 byte | 182 byte | ~549 paket       |
  | 512 byte | 509 byte | ~197 paket       |
  +----------+----------+------------------+

  Daha buyuk MTU = Daha az paket = Daha hizli OTA guncelleme
```

### GATT (Generic Attribute Profile)

GATT, BLE cihazlari arasinda yapilandirilmis veri degisimini tanimlar. Sunucu (server) tarafindaki veriler hiyerarsik bir attribute veritabaninda saklanir.

**GATT Hiyerarsisi:**

```
  GATT Server (ESP32)
  |
  +-- Service: Sensor Data (UUID: 0x00FF)
  |   |
  |   +-- Characteristic: Sensor Data (UUID: 0xFF01)
  |       |-- Properties: Read, Notify
  |       |-- Value: [temp_lo, temp_hi, humid_lo, humid_hi, tick_0..3]
  |       +-- Descriptor: CCCD (UUID: 0x2902)
  |           +-- Value: 0x0001 = Notifications enabled
  |
  +-- Service: OTA Update (UUID: 0x00FE)
      |
      +-- Characteristic: OTA Data (UUID: 0xFE01)
      |   |-- Properties: Write, Write Without Response
      |   +-- Value: [firmware binary chunk data]
      |
      +-- Characteristic: OTA Control (UUID: 0xFE02)
          |-- Properties: Read, Write, Notify
          |-- Value: [state, error, progress]
          +-- Descriptor: CCCD (UUID: 0x2902)
              +-- Value: 0x0001 = Notifications enabled
```

**GATT Islem Akislari:**

```
  Read (Sensor Data okuma):
  Central                          Peripheral (ESP32)
     |-- Read Request (0x0A) -------->|
     |   (Handle: sensor_data)        |
     |<-- Read Response (0x0B) -------|
     |   (Value: sensor bytes)        |

  Write (OTA Data yazma):
  Central                          Peripheral (ESP32)
     |-- Write Command (0x52) ------->|  (Write Without Response)
     |   (Handle: ota_data)           |
     |   (Value: firmware chunk)      |  -> esp_ota_write()

  Notification (Sensor bildirim):
  Central                          Peripheral (ESP32)
     |<-- Notification (0x1B) --------|
     |   (Handle: sensor_data)        |
     |   (Value: new sensor data)     |  (CCCD = 0x0001 olmali)
```

### ESP32 Flash Bellek Organizasyonu ve OTA Partition Tablosu

ESP32, SPI arayuzu uzerinden harici flash bellek kullanir. Firmware, veri ve yapilandirma bilgileri bu flash bellekte farkli bolumlere (partition) ayrilir.

**Flash Bellek Fiziksel Yapisi:**

```
  +-------------------------------------------------------------------+
  |                    ESP32 SPI Flash (4 MB)                         |
  +-------------------------------------------------------------------+
  | Adres       | Boyut    | Icerik                                   |
  +-------------+----------+------------------------------------------+
  | 0x0000_0000 | -        | (Kullanilmaz - boot vector)              |
  | 0x0000_1000 | 28 KB    | Second Stage Bootloader                  |
  | 0x0000_8000 | 4 KB     | Partition Table                          |
  | 0x0000_9000 | 24 KB    | NVS (Non-Volatile Storage)               |
  | 0x0000_F000 | 4 KB     | OTA Data (otadata)                       |
  | 0x0001_0000 | 1.5 MB   | OTA_0 (ota_0) - Uygulama Bolumu 1       |
  | 0x0019_0000 | 1.5 MB   | OTA_1 (ota_1) - Uygulama Bolumu 2       |
  | 0x0031_0000 | ~960 KB  | Bos / Kullanici verisi                   |
  +-------------+----------+------------------------------------------+
```

**Partition Table CSV Formati:**

```csv
# Name,    Type,  SubType,  Offset,    Size,     Flags
nvs,       data,  nvs,      0x9000,    0x6000,
otadata,   data,  ota,      0xf000,    0x1000,
ota_0,     app,   ota_0,    0x10000,   0x180000,
ota_1,     app,   ota_1,    0x190000,  0x180000,
```

**OTA Guncelleme Mekanizmasi:**

```
  Ilk Yukleme (Factory):
  +-------------+     +-------------+
  |   OTA_0     | <-- | Bootloader  |  (otadata: ota_0 aktif)
  | (v1.0.0)    |     |             |
  +-------------+     +-------------+
  |   OTA_1     |
  |   (bos)     |
  +-------------+

  OTA Guncelleme Sirasinda:
  +-------------+     +-------------+
  |   OTA_0     | <-- | Bootloader  |  (otadata: ota_0 hala aktif)
  | (v1.0.0)    |     |             |
  +-------------+     +-------------+
  |   OTA_1     | <== | BLE'den     |  esp_ota_write() ile yaziliyor
  | (v1.1.0)    |     | gelen veri  |
  +-------------+     +-------------+

  Guncelleme Sonrasi (commit + reboot):
  +-------------+     +-------------+
  |   OTA_0     |     |             |
  | (v1.0.0)    |     | Bootloader  |  (otadata: ota_1 aktif)
  +-------------+     +-------------+
  |   OTA_1     | <-- | Yeni yukle  |
  | (v1.1.0)    |     |             |
  +-------------+     +-------------+

  Rollback (hata durumunda):
  +-------------+     +-------------+
  |   OTA_0     | <-- | Bootloader  |  (otadata: ota_0'a geri don)
  | (v1.0.0)    |     |             |
  +-------------+     +-------------+
  |   OTA_1     |     |             |
  | (v1.1.0)    |     | (gecersiz)  |
  +-------------+     +-------------+
```

**OTA Data Partition (otadata) Yapisi:**

```
  otadata bolumu (0x1000 = 4096 byte):
  +----------------------------------------------------------+
  | Entry 0 (32 byte)           | Entry 1 (32 byte)          |
  +----+--------+-------+------+----+--------+-------+------+
  |Seq | Label  | State | Hash |Seq | Label  | State | Hash |
  |(4B)| (16B)  | (4B)  |(32B) |(4B)| (16B)  | (4B)  |(32B) |
  +----+--------+-------+------+----+--------+-------+------+

  Bootloader, en yuksek Seq degerine sahip gecerli entry'yi secer.

  OTA Image State Degerleri:
  +---------------------------+-------+----------------------------+
  | State                     | Deger | Aciklama                   |
  +---------------------------+-------+----------------------------+
  | ESP_OTA_IMG_NEW           | 0x0   | Yeni yuklenmis, dogrulanmamis |
  | ESP_OTA_IMG_PENDING_VERIFY| 0x1   | Dogrulama bekliyor         |
  | ESP_OTA_IMG_VALID         | 0x2   | Gecerli, onaylandi         |
  | ESP_OTA_IMG_INVALID       | 0x3   | Gecersiz, rollback gerekli |
  | ESP_OTA_IMG_ABORTED       | 0x4   | Iptal edildi               |
  +---------------------------+-------+----------------------------+
```

### SHA-256 Hash Dogrulamasi

Firmware butunlugunu dogrulamak icin SHA-256 kriptografik hash fonksiyonu kullanilir. Bu, veri bozulmasini veya eksik transfer durumlarini tespit etmeyi saglar.

**SHA-256 Islem Akisi:**

```
  +----------------+     +----------------+     +----------------+
  | mbedtls_       |     | mbedtls_       |     | mbedtls_       |
  | sha256_init()  |---->| sha256_starts()|---->| sha256_update()|
  | (Context init) |     | (Hash baslat)  |     | (Chunk ekle)   |
  +----------------+     +----------------+     +-------+--------+
                                                         |
                                          Her chunk icin tekrarla
                                                         |
                                                +--------v--------+
                                                | mbedtls_         |
                                                | sha256_finish()  |
                                                | (32-byte digest) |
                                                +------------------+

  Inkremental hesaplama avantaji:
  - Tum firmware'i RAM'de tutmaya gerek yok
  - Her chunk geldiginde hash guncellenir
  - Sonunda 32-byte digest ile beklenen deger karsilastirilir
```

### NimBLE Yigini (Stack) Mimarisi

ESP-IDF, BLE islemleri icin Apache NimBLE acik kaynak BLE yiginini kullanir. NimBLE, dusuk bellek tuketimi ve yuksek performans icin optimize edilmistir.

**NimBLE Baslatma Sirasi:**

```
  nimble_port_init()
       |
       v
  ble_hs_cfg yapilandirma
  (sync_cb, reset_cb, MTU)
       |
       v
  ble_svc_gap_init()      -> Zorunlu GAP servisi
  ble_svc_gatt_init()     -> Zorunlu GATT servisi
       |
       v
  ble_gatts_count_cfg()   -> Attribute sayisini hesapla
  ble_gatts_add_svcs()    -> Ozel servisleri kaydet
       |
       v
  ble_svc_gap_device_name_set()  -> Cihaz adi ayarla
       |
       v
  nimble_port_freertos_init()    -> NimBLE host task baslat
       |
       v
  ble_on_sync() callback         -> Controller hazir
       |
       v
  ble_gap_adv_start()            -> Advertising baslat
```

---

## Pin Baglanti Semasi

Bu proje yalnizca BLE (dahili radyo) kullandigindan harici donanim baglantisi gerektirmez. ESP32'nin dahili BLE radyosu ve anteni kullanilir.

```
    ESP32 DevKitC V4
    +----------------------------------+
    |                                  |
    |  [Dahili BLE Radyo + Anten]      |
    |                                  |
    |  GPIO2  (LED)  |----[220R]----LED---GND   (Opsiyonel: Durum LED)
    |                |                 |
    |  EN    (Reset) |----[Buton]---GND         (Opsiyonel: Reset butonu)
    |                |                 |
    |  GPIO0 (Boot)  |----[Buton]---GND         (Flash modu butonu)
    |                |                 |
    |  USB (UART0)   |---- PC (Monitor/Flash)   |
    |                |                 |
    |  3V3           |---- VCC                  |
    |  GND           |---- GND                  |
    +----------------------------------+

    Notlar:
    - BLE anteni ESP32 modulunun uzerinde dahilidir.
    - Harici anten gerekli degildir (PCB anten yeterlidir).
    - USB baglantisi yalnizca ilk firmware yukleme ve seri izleme icindir.
    - OTA guncelleme tamamen BLE uzerinden yapilir (kablo gerektirmez).
    - Opsiyonel LED, OTA guncelleme durumunu gostermek icin kullanilabilir.
```

---

## ESP-IDF Yapilandirma Adimlari

### 1. Proje Olusturma

```bash
# ESP-IDF ornek projesinden kopyala
cp -r $IDF_PATH/examples/get-started/hello_world ./02-BLE-OTA-Update
cd 02-BLE-OTA-Update
```

### 2. Partition Table Olusturma

OTA guncelleme icin ozel bir partition tablosu gereklidir. `partitions.csv` dosyasi olusturun:

```csv
# Name,    Type,  SubType,  Offset,    Size,     Flags
nvs,       data,  nvs,      0x9000,    0x6000,
otadata,   data,  ota,      0xf000,    0x1000,
ota_0,     app,   ota_0,    0x10000,   0x180000,
ota_1,     app,   ota_1,    0x190000,  0x180000,
```

### 3. CMakeLists.txt Yapilandirmasi

```cmake
# Ana CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ble_ota_update)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "ble_gatt_server.c" "ota_manager.c"
    INCLUDE_DIRS "." "../inc"
    REQUIRES bt nvs_flash esp_ota_ops mbedtls
)
```

### 4. sdkconfig Yapilandirmasi (menuconfig)

```bash
idf.py menuconfig
```

| Menu Yolu | Ayar | Deger |
|-----------|------|-------|
| Partition Table | Partition Table | Custom partition table CSV |
| Partition Table | Custom Partition CSV | partitions.csv |
| Component config > Bluetooth | Bluetooth | Enable |
| Component config > Bluetooth > Host | NimBLE - BLE only | Enable |
| Component config > Bluetooth > NimBLE Options | BLE Role: Peripheral | Enable |
| Component config > Bluetooth > NimBLE Options | Maximum MTU size | 512 |
| Component config > Bluetooth > NimBLE Options | Maximum connections | 1 |
| Component config > Bluetooth > NimBLE Options | GAP device name | ESP32-OTA-GATT |
| Component config > Bluetooth > NimBLE Options | Host task stack size | 4096 |
| Component config > ESP-TLS | | |
| Component config > mbedTLS | SHA-256 | Enable (varsayilan) |
| App update > OTA | Number of OTA partitions | 2 |

### 5. Derleme ve Yukleme

```bash
# Hedef platform ayarla
idf.py set-target esp32

# Derleme
idf.py build

# Flash ve seri monitor (Windows)
idf.py -p COM3 flash monitor

# Flash ve seri monitor (Linux)
idf.py -p /dev/ttyUSB0 flash monitor

# Yalnizca seri monitor
idf.py -p COM3 monitor
```

### 6. OTA Icin Ikinci Firmware Hazirlama

```bash
# Firmware surumunu degistir (main.c'de FIRMWARE_VERSION)
# Degisiklik yaptiktan sonra derle
idf.py build

# Derlenen binary dosya:
# build/ble_ota_update.bin
# Bu dosya BLE uzerinden OTA guncelleme icin kullanilacaktir.
```

---

## Kodun Calisma Mantigi

### Genel Sistem Akisi

```
                        app_main()
                            |
                            v
                  +--------------------+
                  |   NVS Init         |
                  | (nvs_flash_init)   |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  |  Boot Info Log     |
                  | (FW ver, partition)|
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  |  OTA Manager Init  |
                  | (partition query)  |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  | Confirm Image      |
                  | (anti-rollback)    |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  | BLE GATT Server    |
                  |   Init             |
                  | (NimBLE + GATT     |
                  |  + Advertising)    |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  | Register Callbacks |
                  | (OTA + BLE events) |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  | xTaskCreate        |
                  | sensor_sim_task    |
                  +---------+----------+
                            |
              +-------------+-------------+
              |                           |
    +---------v-----------+    +----------v----------+
    | sensor_sim_task     |    | NimBLE Host Task    |
    | (FreeRTOS)          |    | (FreeRTOS)          |
    |                     |    |                     |
    | Sonsuz dongu:       |    | BLE olay isleme:    |
    | 1. Sensor simule et |    | - GAP olaylari      |
    | 2. BLE notify gonder|    | - GATT erisim       |
    | 3. 2 sn bekle       |    | - OTA veri/komut    |
    +---------------------+    +---------------------+
```

### OTA Guncelleme Akisi

```
  Mobil Cihaz                              ESP32
  (nRF Connect)                         (GATT Server)
       |                                      |
       |-- Scan & Connect ------------------>|
       |                                      |
       |-- MTU Exchange (512) -------------->|
       |<-- MTU Exchange Response -----------|
       |                                      |
       |-- Service Discovery --------------->|
       |<-- Service List (0x00FF, 0x00FE) ---|
       |                                      |
       |-- Enable OTA Ctrl Notifications --->|
       |   (CCCD = 0x0001)                    |
       |                                      |
       |== OTA BASLAT ========================|
       |-- Write OTA Control: 0x01 + size -->|  --> ota_manager_start()
       |   [01 xx xx xx xx]                   |      esp_ota_begin()
       |<-- Notification: [01 00 00] --------|      state=RECEIVING
       |                                      |
       |== FIRMWARE TRANSFER ==================|
       |-- Write OTA Data: chunk_1 --------->|  --> ota_manager_write_chunk()
       |-- Write OTA Data: chunk_2 --------->|      esp_ota_write()
       |-- Write OTA Data: chunk_3 --------->|      sha256_update()
       |   ...                                |
       |<-- Notification: [01 00 25] --------|      progress=25%
       |   ...                                |
       |-- Write OTA Data: chunk_N --------->|
       |<-- Notification: [01 00 99] --------|      progress=99%
       |                                      |
       |== DOGRULAMA & COMMIT ================|
       |-- Write OTA Control: 0x03 --------->|  --> ota_manager_finish()
       |   [03 sha256_hash...]                |      sha256_finish()
       |                                      |      esp_ota_end()
       |<-- Notification: [03 00 64] --------|      state=COMPLETE
       |                                      |  --> ota_manager_commit()
       |                                      |      esp_ota_set_boot_partition()
       |                                      |
       |<-- Disconnect (device restarting) ---|  --> esp_restart()
       |                                      |
       |                              [ REBOOT ]
       |                                      |
       |-- Scan & Connect (yeni FW) -------->|
       |-- Write OTA Control: 0x05 --------->|  --> VERSION_REQ
       |<-- Notification: "1.1.0" -----------|      (yeni surum dogrulandi)
```

---

## BLE GATT Servis ve Karakteristik Tablosu

### Servis 1: Sensor Data Service

| Ozellik | Deger |
|---------|-------|
| Servis UUID | 0x00FF |
| Servis Turu | Primary |

| Karakteristik | UUID | Ozellikler | Aciklama |
|---------------|------|------------|----------|
| Sensor Data | 0xFF01 | Read, Notify | Sensor verisini okuma ve bildirim |

**Sensor Data Payload Formati (8 byte):**

| Byte | Icerik | Format | Ornek |
|------|--------|--------|-------|
| 0-1 | Sicaklik (x100) | int16_t, LE | 0xE108 = 2273 -> 22.73 C |
| 2-3 | Nem (x100) | int16_t, LE | 0xC413 = 5060 -> 50.60 % |
| 4-7 | Tick sayaci | uint32_t, LE | Artan sayac |

### Servis 2: OTA Update Service

| Ozellik | Deger |
|---------|-------|
| Servis UUID | 0x00FE |
| Servis Turu | Primary |

| Karakteristik | UUID | Ozellikler | Aciklama |
|---------------|------|------------|----------|
| OTA Data | 0xFE01 | Write, Write No Rsp | Firmware binary chunklari |
| OTA Control | 0xFE02 | Read, Write, Notify | OTA komutlari ve durum |

**OTA Control Komut Formati (Write):**

| CMD | Deger | Payload | Aciklama |
|-----|-------|---------|----------|
| START | 0x01 | 4 byte (total_size, LE) | OTA oturumu baslat |
| STOP | 0x02 | Yok | OTA oturumunu iptal et |
| COMMIT | 0x03 | 32 byte (SHA-256, opsiyonel) | Dogrula ve onayla |
| ROLLBACK | 0x04 | Yok | Onceki firmware'e geri don |
| VERSION_REQ | 0x05 | Yok | Firmware surumu iste |

**OTA Control Durum Formati (Read / Notification):**

| Byte | Icerik | Degerler |
|------|--------|----------|
| 0 | State | 0=IDLE, 1=RECEIVING, 2=VALIDATING, 3=COMPLETE, 4=ERROR |
| 1 | Error | 0=Yok, 1-12=Hata kodlari (bkz. ota_error_t) |
| 2 | Progress | 0-100 (yuzde) |

---

## OTA Durum Makinesi Diyagrami

```
                          +-------------------+
                          |                   |
          +-------------->|       IDLE        |<--------------+
          |               |   (State = 0)     |               |
          |               +--------+----------+               |
          |                        |                          |
          |                start() | (total_size)             |
          |                        v                          |
          |               +-------------------+               |
          |               |                   |               |
          |    +--------->|    RECEIVING       |               |
          |    |          |   (State = 1)      |               |
          |    |          +--------+----------+               |
          |    |                   |                          |
          |    | write_chunk()     | finish()                  |
          |    | (her chunk icin)  v                          |
          |    |          +-------------------+               |
          |    +----------+                   |               |
          |               |   VALIDATING      |               |
     abort()              |   (State = 2)     |          commit()
          |               +---+----------+----+          (reboot)
          |                   |          |                    |
          |            BASARILI|          |BASARISIZ           |
          |                   v          v                    |
          |         +-----------+  +----------+              |
          |         |           |  |          |              |
          +---------+ COMPLETE  |  |  ERROR   |--------------+
                    | (State=3) |  | (State=4)|   abort()
                    +-----+-----+  +----------+
                          |
                     commit()
                          |
                          v
                    [esp_restart()]
                          |
                          v
                  [Yeni firmware baslar]
                          |
                          v
                [ota_manager_confirm_image()]
                          |
                          v
                  [Rollback iptal edildi]


  Hata Kodlari (ota_error_t):
  +------+----------------------------+--------------------------------+
  | Kod  | Isim                       | Aciklama                       |
  +------+----------------------------+--------------------------------+
  |  0   | OTA_ERR_NONE               | Hata yok                       |
  |  1   | OTA_ERR_ALREADY_IN_PROGRESS| Zaten guncelleme devam ediyor  |
  |  2   | OTA_ERR_NOT_IN_PROGRESS    | Aktif oturum yok               |
  |  3   | OTA_ERR_PARTITION_NOT_FOUND| OTA bolumu bulunamadi          |
  |  4   | OTA_ERR_BEGIN_FAILED       | esp_ota_begin() basarisiz      |
  |  5   | OTA_ERR_WRITE_FAILED       | esp_ota_write() basarisiz      |
  |  6   | OTA_ERR_IMAGE_TOO_SMALL    | Firmware cok kucuk             |
  |  7   | OTA_ERR_IMAGE_TOO_LARGE    | Firmware partition'dan buyuk   |
  |  8   | OTA_ERR_VALIDATION_FAILED  | SHA-256 dogrulama basarisiz    |
  |  9   | OTA_ERR_END_FAILED         | esp_ota_end() basarisiz        |
  | 10   | OTA_ERR_SET_BOOT_FAILED    | Boot partition ayarlanamadi    |
  | 11   | OTA_ERR_ROLLBACK_FAILED    | Rollback basarisiz             |
  | 12   | OTA_ERR_INVALID_STATE      | Gecersiz durum gecisi          |
  +------+----------------------------+--------------------------------+
```

---

## Test Proseduru

### Gerekli Araclar

| Arac | Aciklama |
|------|----------|
| ESP32 DevKitC V4 | Hedef gelistirme karti |
| nRF Connect (Android/iOS) | BLE istemci uygulamasi |
| ESP-IDF v5.x | Derleme ve flash araci |
| Python 3.x | OTA binary hazirlama scripti (opsiyonel) |

### Test 1: Temel BLE Baglanti Testi

**Adim 1:** Firmware'i ESP32'ye yukleyin:

```bash
idf.py -p COM3 flash monitor
```

**Adim 2:** Seri monitor ciktisini dogrulayin:

```
I (325) main: ========================================
I (330) main:   BLE GATT Server + OTA Firmware Update
I (335) main:   Firmware version: 1.0.0
I (340) main: ========================================
I (345) main: NVS initialized successfully
I (350) main: --- Boot Information ---
I (355) main: App name:    ble_ota_update
I (360) main: App version: 1.0.0
I (365) main: Running partition: 'ota_0' at 0x00010000 (size: 0x00180000)
I (370) main: Boot partition:    'ota_0' at 0x00010000
I (375) main: Chip: ESP32 rev 1, 2 CPU cores, WiFi/BT/BLE
I (380) main: Free heap: 270000 bytes
I (385) ota_mgr: Running partition: label='ota_0', offset=0x00010000
I (390) ota_mgr: Next update partition: label='ota_1', offset=0x00190000
I (400) ble_gatt: Initializing BLE GATT server...
I (430) ble_gatt: BLE GATT server initialized successfully
I (435) ble_gatt: BLE host synchronized with controller
I (440) ble_gatt: Device address: a4:cf:12:xx:xx:xx (type=0)
I (445) ble_gatt: Advertising started as 'ESP32-OTA-GATT'
I (450) main: System initialization complete
I (455) main: Waiting for BLE connection from client...
```

**Adim 3:** nRF Connect uygulamasini acin ve "ESP32-OTA-GATT" cihazini bulun.

**Adim 4:** Cihaza baglanin. Seri monitorde su ciktiyi gorun:

```
I (5230) ble_gatt: Connected: handle=1, peer_addr=xx:xx:xx:xx:xx:xx
I (5235) main: BLE client connected (handle=1)
I (5240) ble_gatt: MTU updated: conn_handle=1, mtu=512 (payload=509)
```

### Test 2: Sensor Veri Okuma ve Bildirim Testi

**Adim 1:** nRF Connect'te "Unknown Service" (UUID: 0x00FF) servisini genisin.

**Adim 2:** "Unknown Characteristic" (UUID: 0xFF01) karakteristigini bulun.

**Adim 3:** Read butonuna (ok isareti) basin. Sensor verisini gormelisiniz:

```
Value: 0x E108 C413 01000000
         ^     ^     ^
         |     |     +-- Tick: 1
         |     +-- Humidity: 50.60%
         +-- Temperature: 22.73 C
```

**Adim 4:** Notify butonuna (cift ok) basin. Her 2 saniyede sensor verisi bildirimi alin:

```
I (10230) main: Sensor notify: temp=22.73 C, humid=50.60 %, tick=5
I (12230) main: Sensor notify: temp=23.15 C, humid=49.12 %, tick=6
I (14230) main: Sensor notify: temp=23.55 C, humid=47.45 %, tick=7
```

### Test 3: OTA Firmware Guncelleme Testi

**Adim 1:** Yeni firmware hazirlayin (main.c'de surum numarasini degistirin):

```c
#define FIRMWARE_VERSION  "1.1.0"
```

```bash
idf.py build
# build/ble_ota_update.bin dosyasi olusacaktir
```

**Adim 2:** Binary dosyanin boyutunu ogrenin:

```bash
# Linux/macOS
ls -l build/ble_ota_update.bin
# veya
stat --format=%s build/ble_ota_update.bin

# Windows (PowerShell)
(Get-Item build/ble_ota_update.bin).Length
```

**Adim 3:** nRF Connect'te OTA Service (UUID: 0x00FE) servisini genisin.

**Adim 4:** OTA Control (0xFE02) uzerinde Notification'lari etkinlestirin (CCCD = 0x0001).

**Adim 5:** OTA Control'e START komutu yayin:

```
Write Value (hex): 01 XX XX XX XX
                   ^  ^^^^^^^^^
                   |  firmware boyutu (little-endian, 4 byte)
                   START komutu

Ornek (firmware boyutu = 256000 byte = 0x0003E800):
Write Value: 01 00 E8 03 00
```

**Adim 6:** Seri monitorde OTA baslangicini dogrulayin:

```
I (25000) main: OTA START: firmware size = 256000 bytes
I (25005) ota_mgr: Starting OTA update: total_size=256000, target_partition='ota_1'
I (26500) ota_mgr: State: IDLE -> RECEIVING
```

**Adim 7:** OTA Data (0xFE01) karakteristigine firmware parcalarini yazmaya baslayin.

> **Not:** nRF Connect uygulamasinda "Write" secenegini secin ve firmware binary verisini parcalar halinde gonderin. Daha pratik bir yontem icin ozel bir BLE OTA istemci uygulamasi veya Python scripti kullanmaniz onerilir.

**Adim 8:** Transfer tamamlandiginda OTA Control'e COMMIT komutu yayin:

```
Write Value (hex): 03
                   ^
                   COMMIT komutu (SHA-256 hash opsiyonel)
```

**Adim 9:** ESP32 otomatik olarak yeniden baslar ve yeni firmware ile acilir:

```
I (45000) ota_mgr: All firmware data received, validating image...
I (45005) ota_mgr: SHA-256 verification PASSED
I (45100) ota_mgr: Firmware validation passed. Ready to commit.
I (45105) ota_mgr: Committing new firmware: setting boot partition to 'ota_1'
I (45110) ota_mgr: Boot partition updated. Restarting device in 1 second...

  ... REBOOT ...

I (325) main: ========================================
I (330) main:   BLE GATT Server + OTA Firmware Update
I (335) main:   Firmware version: 1.1.0          <-- Yeni surum!
I (340) main: ========================================
I (370) main: Running partition: 'ota_1' at 0x00190000   <-- Yeni partition!
```

### Test 4: Rollback Testi

**Adim 1:** Yeni firmware ile calisan ESP32'ye BLE ile baglanin.

**Adim 2:** OTA Control karakteristigine ROLLBACK komutu yayin:

```
Write Value (hex): 04
                   ^
                   ROLLBACK komutu
```

**Adim 3:** ESP32 onceki firmware ile yeniden baslar:

```
I (500) main: Firmware version: 1.0.0     <-- Eski surum
I (505) main: Running partition: 'ota_0'  <-- Onceki partition
```

### Test 5: Hata Durumu Testi

**Adim 1:** OTA oturumu baslatmadan firmware verisi gonderin:

```
OTA Data'ya Write -> Hata bildirimi alinmali (state=4, error=12)
```

**Adim 2:** Eksik firmware transferi yapip COMMIT gonderin:

```
1. START komutu gonderin (buyuk boyut belirtin)
2. Yalnizca birka chunk gonderin (tamamlanmadan)
3. COMMIT gonderin -> SHA-256 hatasi veya boyut hatasi alinmali
```

**Adim 3:** OTA sirasinda BLE baglantisini kopartamak:

```
nRF Connect'te Disconnect -> ESP32 OTA oturumunu otomatik iptal eder
Seri monitorde: "Aborting OTA session due to BLE disconnect"
```

### Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| BLE advertising | "ESP32-OTA-GATT" nRF Connect'te gorunur | [ ] |
| BLE baglanti | Basarili baglanti, MTU degisimi | [ ] |
| Sensor veri okuma | Read ile sensor verisi doner | [ ] |
| Sensor notification | Her 2 sn'de bildirim alinir | [ ] |
| OTA START | Oturum baslar, RECEIVING durumuna gecer | [ ] |
| OTA veri transfer | Firmware chunklari yazilir, ilerleme bildirimi | [ ] |
| OTA COMMIT | Dogrulama basarili, commit sonrasi reboot | [ ] |
| Yeni firmware boot | Yeni surum numarasi ve partition gorulur | [ ] |
| OTA ROLLBACK | Onceki firmware'e geri donus | [ ] |
| BLE disconnect sirasinda OTA | Oturum otomatik iptal edilir | [ ] |
| OTA STOP komutu | Aktif oturum iptal edilir | [ ] |
| VERSION_REQ | Firmware surumu notification olarak doner | [ ] |
| Coklu baglanti/disconnect | Reklam yeniden baslar, stabil calisma | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| BLE cihazi gorunmuyor | Advertising baslamadi | Seri monitorde "Advertising started" kontrolu, NimBLE init hatalarini incele |
| BLE cihazi gorunmuyor | Bluetooth kapali | Mobil cihazda Bluetooth'un acik oldugunu dogrula |
| Baglanti kurulamiyor | Advertising durduruldu | ESP32'yi resetle, NimBLE konfigurasyonunu kontrol et |
| Baglanti hemen kopuyor | MTU uyumsuzlugu | nRF Connect ayarlarinda MTU degerini kontrol et |
| Sensor verisi bos | Henuz veri uretilmedi | Birka saniye bekle, sensor task'in basladigini kontrol et |
| Notification gelmiyor | CCCD etkin degil | nRF Connect'te notification butonuna basildigini kontrol et |
| OTA START basarisiz | Gecersiz boyut | Firmware boyutunun 32KB-1.5MB araliginda oldugunu dogrula |
| OTA START basarisiz | Partition bulunamadi | Partition tablosunun dogru yapildirildigini kontrol et (ota_0, ota_1) |
| OTA yazma hatasi | Flash bellek sorunu | Flash bellek bozuk olabilir, cihazi yeniden flash'la |
| OTA dogrulama basarisiz | SHA-256 uyumsuzlugu | Binary dosyanin bozulmadan transfer edildigini kontrol et |
| OTA dogrulama basarisiz | Yanlis firmware | Firmware'in ESP32 icin derlendigini dogrula (set-target esp32) |
| OTA COMMIT basarisiz | Image header gecersiz | Derleme ayarlarini kontrol et, bootloader uyumsuzlugu olabilir |
| Rollback calismiyor | Tek OTA bolumu | Partition tablosunda ota_0 ve ota_1 oldugunu dogrula |
| ESP32 boot loop'a giriyor | Bozuk firmware | Bootloader otomatik rollback yapar, seri monitorde izle |
| Dusuk BLE throughput | Kucuk MTU | MTU exchange basarisini kontrol et (512 olmali) |
| Dusuk BLE throughput | Write With Response | Write Without Response (0x52) kullanildigini dogrula |
| Heap yetersiz | Bellek sizintisi | `esp_get_free_heap_size()` ile bellek izle |
| NimBLE host reset | Controller hatasi | ESP32'yi resetle, NVS'yi sil (nvs_flash_erase) |

---

## Performans Olcumleri

### BLE OTA Transfer Hizi

| MTU | Payload/Paket | Write Turu | Teorik Hiz | Pratik Hiz |
|-----|---------------|------------|------------|------------|
| 23 | 20 byte | Write No Rsp | ~26 KB/s | ~10-15 KB/s |
| 185 | 182 byte | Write No Rsp | ~240 KB/s | ~30-50 KB/s |
| 512 | 509 byte | Write No Rsp | ~400 KB/s | ~40-60 KB/s |
| 512 | 509 byte | Write (Rsp) | ~200 KB/s | ~20-30 KB/s |

### OTA Guncelleme Suresi Tahmini

| Firmware Boyutu | MTU=23 (20B) | MTU=512 (509B) |
|-----------------|--------------|----------------|
| 100 KB | ~7-10 sn | ~2-3 sn |
| 500 KB | ~35-50 sn | ~8-13 sn |
| 1 MB | ~70-100 sn | ~17-25 sn |
| 1.5 MB | ~100-150 sn | ~25-38 sn |

> **Not:** Pratik hizlar BLE sinyal kalitesi, baglanti araligi (connection interval), ve flash yazma hizina bagli olarak degisir.

---

## Guvenlik Degerlendirmeleri

| Risk | Seviye | Oneri |
|------|--------|-------|
| Sifrelenmemis BLE baglanti | Yuksek | Production'da BLE pairing + encryption aktif edin |
| Imzasiz firmware | Orta | ESP-IDF Secure Boot v2 kullanin |
| Flash sifreleme yok | Orta | ESP-IDF Flash Encryption etkinlestirin |
| Herkes OTA baslatabiliyor | Yuksek | BLE bonding + yetkilendirme ekleyin |
| Rollback sinirsiz | Dusuk | Anti-rollback counter kullanin |

> **Uyari:** Bu proje egitim amaclidir. Production kullanimi icin BLE guvenlik (pairing, bonding, encryption) ve ESP-IDF Secure Boot mutlaka aktif edilmelidir.

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf) | ESP32 register ve cevre birimi referansi |
| [ESP-IDF BLE API (NimBLE)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/nimble/index.html) | NimBLE BLE API referansi |
| [ESP-IDF OTA API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html) | OTA guncelleme API referansi |
| [ESP-IDF Partition Table](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html) | Partition tablosu yapisi ve yapilandirmasi |
| [Bluetooth Core Specification v5.3](https://www.bluetooth.com/specifications/specs/core-specification-5-3/) | Bluetooth protokol spesifikasyonu |
| [BLE GATT Specification](https://www.bluetooth.com/specifications/gatt/) | GATT servis ve karakteristik tanimlari |
| [Apache NimBLE Dokumantasyonu](https://mynewt.apache.org/latest/network/) | NimBLE yigininin resmi dokumantasyonu |
| [nRF Connect for Mobile](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) | BLE test uygulamasi |
| [mbedTLS SHA-256](https://tls.mbed.org/sha-256-source-code) | SHA-256 implementasyonu |
| [ESP-IDF Secure Boot](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/secure-boot-v2.html) | Guvenli boot ve firmware imzalama |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
