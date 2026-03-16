# 01 - WiFi MQTT Sensor Hub

## Proje Ozeti

Bu proje, ESP32'nin WiFi STA (Station) modunu ve MQTT protokolunu kullanarak bir **IoT sensor hub** uygulamasi gerceklestirir. DHT22 (sicaklik/nem) ve BH1750 (ortam isigi) sensorlerinden okunan veriler JSON formatinda MQTT broker'a periyodik olarak yayinlanir. Uzaktan kontrol topic'i uzerinden okuma araligi degistirme ve cihaz yeniden baslatma komutlari alinabilir. WiFi baglantisi icin exponential back-off ile otomatik yeniden baglanti mekanizmasi uygulanmistir.

---

## Teorik Arka Plan

### WiFi 802.11 ve ESP32 WiFi Mimarisi

ESP32, dahili WiFi radyosu ile 802.11 b/g/n standartlarini destekler (2.4 GHz bandi). WiFi alt sistemi su katmanlardan olusur:

```
+--------------------------------------------------+
|              Uygulama Katmani (app_main)          |
+--------------------------------------------------+
|              ESP-IDF WiFi API                     |
|  esp_wifi_init / esp_wifi_start / esp_wifi_connect|
+--------------------------------------------------+
|              LwIP TCP/IP Stack                    |
|         (DHCP Client, DNS, TCP, UDP)              |
+--------------------------------------------------+
|              WiFi Driver (closed-source)          |
|         (MAC, PHY, kanal yonetimi)                |
+--------------------------------------------------+
|              RF Donanim (2.4 GHz Radio)           |
+--------------------------------------------------+
```

**STA (Station) Modu Baglanti Sureci:**

```
  ESP32 (STA)                        Access Point (AP)
      |                                     |
      |------- Probe Request ------------->|
      |<------ Probe Response -------------|
      |                                     |
      |------- Authentication ------------>|
      |<------ Authentication Response ----|
      |                                     |
      |------- Association Request ------->|
      |<------ Association Response -------|
      |                                     |
      |<======= 4-Way Handshake =========>|
      |         (WPA2-PSK)                  |
      |                                     |
      |------- DHCP Discover ------------->|
      |<------ DHCP Offer -----------------|
      |------- DHCP Request -------------->|
      |<------ DHCP ACK ------------------|
      |                                     |
      |  [IP_EVENT_STA_GOT_IP tetiklenir]   |
```

### MQTT Protokolu

MQTT (Message Queuing Telemetry Transport), IoT uygulamalari icin tasarlanmis hafif bir publish/subscribe mesajlasma protokoludur. TCP/IP uzerinde calisir ve minimum bant genisligi kullanir.

**MQTT Mesaj Akisi:**

```
  +----------+                    +----------+                    +----------+
  | Publisher |                    |  Broker  |                    |Subscriber|
  | (ESP32)  |                    | (Server) |                    | (Client) |
  +----+-----+                    +----+-----+                    +----+-----+
       |                               |                               |
       |---- CONNECT ----------------->|                               |
       |<--- CONNACK ------------------|                               |
       |                               |<---- SUBSCRIBE (sensor/data) -|
       |                               |---- SUBACK ------------------>|
       |                               |                               |
       |---- PUBLISH (sensor/data) --->|                               |
       |<--- PUBACK (QoS 1) ----------|---- PUBLISH (sensor/data) --->|
       |                               |<--- PUBACK (QoS 1) ----------|
       |                               |                               |
```

**QoS Seviyeleri:**

| QoS | Isim | Aciklama | Kullanim |
|-----|------|----------|----------|
| 0 | At most once | Mesaj en fazla bir kez gonderilir, onay yok | Sensor verisi (kayip kabul edilebilir) |
| 1 | At least once | Mesaj en az bir kez teslim edilir (PUBACK ile) | Genel IoT veri yayini |
| 2 | Exactly once | Mesaj tam olarak bir kez teslim edilir (4-step handshake) | Kritik komutlar |

### DHT22 (AM2302) Sensor Protokolu

DHT22, tek hat (one-wire) protokolu kullanan dijital bir sicaklik ve nem sensorudur. Veri transferi asagidaki sekilde gerceklesir:

**Zamanlama Diyagrami:**

```
  MCU Baslangic Sinyali:
  _____          ____________________________________
       |        |
       |________|
       | >=1ms  | 20-40us

  DHT22 Yanit:
  ____________        ________
              |      |        |
              |______|        |______
              | 80us | 80us  |

  Bit '0':                    Bit '1':
  ______        __            ______        ________
        |      |  |                 |      |        |
        |______|  |                 |______|        |
        | 50us |26us              | 50us  | 70us   |
```

**Veri Formati (40 bit = 5 byte):**

```
  +----------+----------+----------+----------+----------+
  | Nem (H)  | Nem (L)  | Sic.(H)  | Sic.(L)  | Checksum |
  | Byte 0   | Byte 1   | Byte 2   | Byte 3   | Byte 4   |
  +----------+----------+----------+----------+----------+

  Nem = (Byte0 << 8 | Byte1) / 10.0        [%RH]
  Sicaklik = (Byte2 << 8 | Byte3) / 10.0   [°C]
  Checksum = (Byte0 + Byte1 + Byte2 + Byte3) & 0xFF
```

| Parametre | Deger |
|-----------|-------|
| Calisma Gerilimi | 3.3V - 5V |
| Sicaklik Araligi | -40C ~ +80C (+-0.5C) |
| Nem Araligi | 0% ~ 100% (+-2%) |
| Okuma Araligi | Min. 2 saniye |
| Cozunurluk | 0.1C / 0.1% |

### BH1750 Isik Sensoru (I2C)

BH1750, I2C arabirimi uzerinden calisir ve ortam isik siddetini dogrudan lux biriminde olcer.

**I2C Iletisim:**

```
  Master (ESP32)                         Slave (BH1750)
      |                                       |
      |-- START ----[0x23 + W]-- ACK -------->|
      |-- [0x10 (Cont. H-Res)] -- ACK ------->|  (Olcum baslat)
      |-- STOP --------------------------------|
      |                                       |
      |  ... ~120 ms olcum suresi ...         |
      |                                       |
      |-- START ----[0x23 + R]-- ACK -------->|
      |<- [Data_H] -- ACK -- [Data_L] -- NACK-|  (2 byte oku)
      |-- STOP --------------------------------|

  Lux = (Data_H << 8 | Data_L) / 1.2
```

| Parametre | Deger |
|-----------|-------|
| I2C Adres | 0x23 (ADDR=LOW) veya 0x5C (ADDR=HIGH) |
| Olcum Araligi | 1 - 65535 lux |
| Cozunurluk | 1 lux (H-Res modu) |
| Olcum Suresi | ~120 ms (H-Res modu) |
| Calisma Akimi | ~120 uA |

---

## Pin Baglanti Semasi

```
    ESP32 DevKitC V4                    Sensorler
    +----------------+
    |                |
    |  GPIO4  (D4)   |-------- DATA --------+---- DHT22 DATA
    |                |                      |
    |                |                  [4.7K Ohm]
    |                |                      |
    |  3V3           |-------- VCC ---------+---- DHT22 VCC
    |  GND           |-------- GND -------------- DHT22 GND
    |                |
    |  GPIO21 (SDA)  |-------- SDA ----[4.7K]---- BH1750 SDA
    |  GPIO22 (SCL)  |-------- SCL ----[4.7K]---- BH1750 SCL
    |  3V3           |-------- VCC --------------- BH1750 VCC
    |  GND           |-------- GND --------------- BH1750 GND
    |                |                             BH1750 ADDR -> GND (0x23)
    |                |
    |  USB (UART0)   |-------- PC (Monitor/Flash)
    +----------------+

    Pull-up direncler:
    - DHT22 DATA hatti: 4.7K Ohm (VCC ile DATA arasi)
    - I2C SDA/SCL: 4.7K Ohm (3V3 ile hat arasi, harici veya dahili)
```

---

## ESP-IDF Yapilandirma Adimlari

### 1. Proje Olusturma

```bash
# ESP-IDF ornek projesinden kopyala
cp -r $IDF_PATH/examples/get-started/hello_world ./01-WiFi-MQTT-SensorHub
cd 01-WiFi-MQTT-SensorHub
```

### 2. CMakeLists.txt Yapilandirmasi

```cmake
# Ana CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wifi_mqtt_sensor_hub)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "wifi_manager.c" "mqtt_client_handler.c" "sensor_reader.c"
    INCLUDE_DIRS "." "../inc"
)
```

### 3. sdkconfig Yapilandirmasi (menuconfig)

```bash
idf.py menuconfig
```

| Menu Yolu | Ayar | Deger |
|-----------|------|-------|
| Component config > ESP-TLS | | |
| Component config > MQTT | MQTT Task Stack Size | 4096 |
| Component config > MQTT | MQTT Buffer Size | 1024 |
| Component config > WiFi | WiFi Task Stack Size | 4096 |
| Component config > FreeRTOS | Tick rate (Hz) | 1000 |

### 4. Derleme ve Yukleme

```bash
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor    # Windows
idf.py -p /dev/ttyUSB0 flash monitor  # Linux
```

---

## Kodun Calisma Mantigi

```
                    app_main()
                        |
                        v
              +-------------------+
              | WiFi Manager Init |
              |  (STA Mode)       |
              +--------+----------+
                       |
                       v
              +-------------------+
              | WiFi Connect      |
              | (SSID/Password)   |
              +--------+----------+
                       |
                       v
              +-------------------+
              | IP Alindi mi?     |----> HAYIR: Exponential back-off
              +--------+----------+       ile yeniden dene
                       | EVET
                       v
              +-------------------+
              | MQTT Client Init  |
              | (Broker URI)      |
              +--------+----------+
                       |
                       v
              +-------------------+
              | MQTT Connect      |
              +--------+----------+
                       |
                       v
              +-------------------+
              | Subscribe:        |
              | sensor/control    |
              +--------+----------+
                       |
                       v
              +-------------------+
              | Sensor Init       |
              | (DHT22 + BH1750)  |
              +--------+----------+
                       |
                       v
              +-------------------+
              | xTaskCreate       |
              | sensor_pub_task   |
              +--------+----------+
                       |
          +============+============+
          |    sensor_pub_task      |  (FreeRTOS Task - Sonsuz Dongu)
          |                        |
          |  +------------------+  |
          |  | sensor_read_all  |  |
          |  | (DHT22 + BH1750) |  |
          |  +--------+---------+  |
          |           |            |
          |           v            |
          |  +------------------+  |
          |  | to_json()        |  |
          |  | (JSON formatlama)|  |
          |  +--------+---------+  |
          |           |            |
          |           v            |
          |  +------------------+  |
          |  | mqtt_publish()   |  |
          |  | (sensor/data)    |  |
          |  +--------+---------+  |
          |           |            |
          |           v            |
          |  +------------------+  |
          |  | vTaskDelay()     |  |
          |  | (Interval bekle) |  |
          |  +------------------+  |
          +========================+
```

---

## MQTT Topic Yapisi

```
sensor/
  ├── data        [PUBLISH]   ESP32 -> Broker -> Subscriber
  │                           {"temperature":25.3,"humidity":48.2,"light":320.5,
  │                            "timestamp":12345,"dht22_valid":true,"bh1750_valid":true}
  │
  ├── status      [PUBLISH]   ESP32 -> Broker (Retain)
  │                           {"status":"online"} / {"status":"restarting"}
  │
  └── control     [SUBSCRIBE] Publisher -> Broker -> ESP32
                              {"interval":10000}  -> Okuma araligini degistir
                              {"restart":true}    -> ESP32'yi yeniden baslat
```

---

## Test Proseduru

### 1. MQTT Broker Kurulumu

```bash
# Mosquitto broker kurulumu (Linux)
sudo apt install mosquitto mosquitto-clients

# Broker baslatma
sudo systemctl start mosquitto
```

### 2. Subscriber Baslatma (PC'de)

```bash
# Sensor verisini dinle
mosquitto_sub -h localhost -t "sensor/data" -v

# Cihaz durumunu dinle
mosquitto_sub -h localhost -t "sensor/status" -v
```

### 3. ESP32'yi Flash'lama ve Izleme

```bash
idf.py -p COM3 flash monitor
```

**Beklenen Seri Monitor Ciktisi:**

```
I (325) main: === WiFi MQTT Sensor Hub ===
I (325) main: Firmware version: 1.0.0
I (335) wifi_mgr: WiFi manager initialized, connecting to 'MyNetwork'...
I (2150) wifi_mgr: Connected! IP: 192.168.1.42
I (2155) main: WiFi connected, IP: 192.168.1.42
I (2160) mqtt_hdlr: MQTT client started, connecting to mqtt://192.168.1.100:1883
I (2350) mqtt_hdlr: MQTT connected to broker
I (2355) sensor: Sensor reader initialized (DHT22: GPIO4, BH1750: I2C addr 0x23)
I (2360) main: System initialized. Publishing every 5000 ms.
I (7365) main: Published: {"temperature":24.5,"humidity":52.3,"light":450.0,...}
```

### 4. Uzaktan Kontrol Testi

```bash
# Okuma araligini 10 saniye yap
mosquitto_pub -h localhost -t "sensor/control" -m '{"interval":10000}'

# ESP32'yi yeniden baslat
mosquitto_pub -h localhost -t "sensor/control" -m '{"restart":true}'
```

### 5. Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| WiFi baglantisi | IP adresi alinir | [ ] |
| MQTT baglantisi | Broker'a basarili baglanti | [ ] |
| DHT22 okuma | Sicaklik ve nem degerleri JSON'da | [ ] |
| BH1750 okuma | Isik degeri JSON'da (lux) | [ ] |
| Periyodik yayin | Her 5 saniyede bir MQTT mesaji | [ ] |
| Interval degistirme | "interval" komutu ile aralik degisir | [ ] |
| WiFi kopma/baglanti | Otomatik yeniden baglanti | [ ] |
| MQTT kopma/baglanti | Otomatik yeniden baglanma + re-subscribe | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| WiFi baglanmiyor | Yanlis SSID/sifre | `main.c`'deki `WIFI_SSID` ve `WIFI_PASSWORD` degerlerini kontrol et |
| WiFi "reason: 201" | AP bulunamadi | ESP32'nin AP menzilinde oldugundan emin ol |
| MQTT baglanmiyor | Broker adresi yanlis | `MQTT_BROKER_URI`'yi kontrol et, broker'in dinledigini dogrula |
| MQTT "transport error" | Firewall engelliyor | 1883 portunun acik oldugunu kontrol et |
| DHT22 timeout | Kablo baglantisi | DATA hattindaki 4.7K pull-up direnci kontrol et |
| DHT22 checksum hatasi | Sinyal bozulmasi | Kablo uzunlugunu kisalt, pull-up degerini kontrol et |
| BH1750 I2C hatasi | Yanlis adres | ADDR pininin GND'ye baglandigini dogrula (0x23) |
| BH1750 0 lux | Sensor uyku modunda | Power On komutu gonderildigini kontrol et |
| JSON parse hatasi | Buffer tasmasi | `JSON_BUFFER_SIZE` degerini artir |
| Heap yetersiz | Bellek sizintisi | `esp_get_free_heap_size()` ile izle |

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf) | ESP32 register-level dokumantasyon |
| [ESP-IDF WiFi API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html) | WiFi driver API referansi |
| [ESP-IDF MQTT API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html) | MQTT client API referansi |
| [DHT22 Datasheet](https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf) | DHT22 sensor datasheet |
| [BH1750 Datasheet](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf) | BH1750 isik sensoru datasheet |
| [MQTT v3.1.1 Specification](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html) | MQTT protokol spesifikasyonu |
| [Mosquitto MQTT Broker](https://mosquitto.org/) | Acik kaynak MQTT broker |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
