# 05 - WebSocket RealTime Control

## Proje Ozeti

Bu proje, ESP32 uzerinde **HTTP web sunucusu** ve **WebSocket** protokolunu kullanarak bir **gercek zamanli motor kontrol sistemi** gerceklestirir. ESP32, WiFi STA modunda aga baglanir ve gomulu bir web arayuzu sunar. Tarayici uzerinden erisilen HTML/JavaScript arayuzu ile DC motor hizi (PWM), servo motor acisi ve sistem parametreleri WebSocket uzerinden milisaniye gecikmesiyle kontrol edilir. Motor durum bilgileri (encoder hizi, PWM duty cycle, servo acisi) gercek zamanli olarak tarayiciya geri gonderilir.

---

## Teorik Arka Plan

### HTTP Protokolu ve Gomulu Web Sunucu

ESP-IDF'in `httpd` bileseni, ESP32 uzerinde hafif bir HTTP/1.1 sunucusu calistirir. Statik HTML/CSS/JS dosyalari bu sunucu uzerinden istemciye sunulur.

```
  Tarayici (Client)                    ESP32 (Server)
       |                                     |
       |--- GET / HTTP/1.1 ----------------->|
       |                                     |  index.html gonder
       |<-- HTTP/1.1 200 OK ----------------|
       |    Content-Type: text/html          |
       |    <html>...</html>                 |
       |                                     |
       |--- GET /style.css ----------------->|
       |<-- HTTP/1.1 200 OK ----------------|
       |                                     |
       |--- Upgrade: WebSocket ------------->|  (WebSocket Handshake)
       |<-- 101 Switching Protocols ---------|
       |                                     |
       |<========= WebSocket ===============>|  (Full-duplex)
```

### WebSocket Protokolu

WebSocket, HTTP uzerinden baslayip full-duplex TCP baglantisina gecis yapan bir protokoldur. Gercek zamanli, dusuk gecikmeli, cift yonlu iletisime olanak tanir.

**WebSocket Handshake:**

```
  Client -> Server (HTTP Upgrade Request):
  GET /ws HTTP/1.1
  Host: 192.168.1.50
  Upgrade: websocket
  Connection: Upgrade
  Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
  Sec-WebSocket-Version: 13

  Server -> Client (101 Switching Protocols):
  HTTP/1.1 101 Switching Protocols
  Upgrade: websocket
  Connection: Upgrade
  Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
```

**WebSocket Frame Formati:**

```
   0                   1                   2                   3
   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  +-+-+-+-+-------+-+-------------+-------------------------------+
  |F|R|R|R| opcode|M| Payload len |    Extended payload length    |
  |I|S|S|S|  (4)  |A|     (7)     |            (16/64)            |
  |N|V|V|V|       |S|             |   (if payload len==126/127)   |
  | |1|2|3|       |K|             |                               |
  +-+-+-+-+-------+-+-------------+-------------------------------+
  |     Extended payload length continued, if payload len == 127  |
  +--------------------------------+-------------------------------+
  |     Masking-key, if MASK set to 1  (4 bytes)                  |
  +--------------------------------+-------------------------------+
  |          Payload Data                                          |
  +---------------------------------------------------------------+

  Opcode degerleri:
  0x0 = Continuation Frame
  0x1 = Text Frame       (JSON komutlar icin)
  0x2 = Binary Frame     (ham veri icin)
  0x8 = Connection Close
  0x9 = Ping
  0xA = Pong
```

**HTTP vs WebSocket Karsilastirmasi:**

| Ozellik | HTTP | WebSocket |
|---------|------|-----------|
| Baglanti | Her istek icin yeni | Kalici (persistent) |
| Yonluluk | Istek-yanit (tek yonlu) | Full-duplex (cift yonlu) |
| Baslik Yuku | Her istekte ~200-800 byte | Baglanti sonrasi 2-14 byte |
| Gecikme | Yuksek (baglanti kurma) | Dusuk (~1ms) |
| Gercek Zamanli | Polling gerekli | Native push destegi |
| Kullanim | Statik icerik, REST API | Canli veri, motor kontrol |

### PWM ve Motor Kontrol

**ESP32 LEDC (LED Control) Periferi:**

ESP32'nin LEDC periferi, 16 kanal (8 high-speed + 8 low-speed) PWM sinyali uretir. Motor kontrol icin kullanilir.

```
  PWM Sinyali:
                    Period (T)
  |<-------------------------------->|
  |                                  |
  |  Duty Cycle     |  OFF           |
  |<-------------->|                 |
  |                 |                |
  +--+     +--------+               +--+     +--------+
  |  |     |        |               |  |     |        |
  |  |     |        |               |  |     |        |
  +--+-----+--------+---------------+--+-----+--------+---
     t_on             t_off

  Duty Cycle = t_on / T * 100%
  Frekans = 1 / T

  Motor hizi ~ Duty Cycle (orantili)
  Duty = 0%   -> Motor duruk
  Duty = 50%  -> Yarim hiz
  Duty = 100% -> Tam hiz
```

**Servo Motor PWM Zamanlama:**

```
  Servo (SG90) kontrol sinyali: 50 Hz (20 ms periyod)

  0 derece (sol):     90 derece (orta):    180 derece (sag):
  +--+                +----+              +------+
  |  |                |    |              |      |
  |  |                |    |              |      |
  +--+-------------+  +----+-----------+  +------+---------+
  500us   19.5ms      1500us  18.5ms      2500us  17.5ms
  |<---- 20ms ---->|  |<---- 20ms ---->|  |<---- 20ms ---->|

  Aci = (pulse_us - 500) / (2500 - 500) * 180
  pulse_us = 500 + (aci / 180) * 2000
```

**H-Bridge Motor Surucu (L298N):**

```
  +-------+     +---L298N---+     +-------+
  | ESP32 |     |           |     | Motor |
  |       |     |  IN1  OUT1|---->|       |
  | GPIO12|---->|  IN2  OUT2|---->|       |
  | GPIO14|---->|           |     +-------+
  |       |     |  ENA      |
  | GPIO13|---->| (PWM hiz) |
  |       |     |           |
  | GND   |---->| GND    VS |<---- 12V
  +-------+     +-----------+

  H-Bridge Dogruluk Tablosu:
  +------+------+------+------------------+
  | ENA  | IN1  | IN2  | Motor Durumu     |
  +------+------+------+------------------+
  |  0   |  X   |  X   | Motor duruk      |
  |  1   |  0   |  0   | Motor frenleme   |
  |  1   |  1   |  0   | Ileri yon        |
  |  1   |  0   |  1   | Geri yon         |
  |  1   |  1   |  1   | Motor frenleme   |
  +------+------+------+------------------+
  ENA: PWM sinyali (hiz kontrolu)
```

### Encoder (Hiz Olcumu)

```
  Rotary Encoder Sinyalleri:

  Ileri Yon:               Geri Yon:
  A: _|-----|_____|-----   A: _|-----|_____|-----
  B: __|-----|_____|---    B: |-----|_____|-----_

  Ileri: A yukselen kenar -> B=LOW
  Geri:  A yukselen kenar -> B=HIGH

  RPM = (pulse_count / PPR) / dt * 60
  PPR = Pulse Per Revolution (encoder cozunurlugu)
```

---

## Pin Baglanti Semasi

```
    ESP32 DevKitC V4              Motor Surucu + Sensorler
    +-------------------+
    |                   |         L298N Motor Driver:
    |  GPIO12 (IN1)     |-------> IN1 (yon bit 1)
    |  GPIO14 (IN2)     |-------> IN2 (yon bit 2)
    |  GPIO13 (ENA/PWM) |-------> ENA (hiz - PWM)
    |                   |
    |  GPIO27 (SERVO)   |-------> SG90 Servo (sinyal)
    |                   |
    |  GPIO34 (ENC_A)   |<------- Encoder A kanali
    |  GPIO35 (ENC_B)   |<------- Encoder B kanali
    |                   |
    |  GPIO2  (LED)     |---[330R]--- Durum LED
    |                   |
    |  3V3              |-------> Servo VCC (harici 5V onerilir)
    |  GND              |-------> Ortak GND
    |                   |
    |  USB (UART0)      |-------> PC (Monitor)
    +-------------------+

    L298N Baglanti:
    +--L298N--+
    | VS: 12V | <-- Harici guc kaynagi
    | GND     | <-- Ortak GND
    | 5V (reg)| --> (kullanilmadan birakilabilir)
    | OUT1    | --> DC Motor (+)
    | OUT2    | --> DC Motor (-)
    +---------+
```

---

## Web Arayuzu

```
  +--------------------------------------------------+
  |  ESP32 Motor Control Dashboard                    |
  +--------------------------------------------------+
  |                                                   |
  |  DC Motor Kontrol:                                |
  |  +--- Hiz ---+  [=====>        ] 65%              |
  |  | [<]  [>]  |  PWM Duty: 65%                     |
  |  +-----------+  Yon: ILERI                         |
  |               RPM: 1250                            |
  |                                                    |
  |  Servo Motor:                                      |
  |  +--- Aci ---+  [===========>  ] 135°             |
  |  | [0] [180] |  Pulse: 2000us                      |
  |  +-----------+                                     |
  |                                                    |
  |  Durum: Bagli | Gecikme: 2ms | Heap: 142KB        |
  +--------------------------------------------------+
```

WebSocket uzerinden gonderilen JSON komutlari:

```json
// Client -> ESP32 (Kontrol komutlari)
{"cmd":"motor_speed", "value":65}
{"cmd":"motor_dir", "value":"forward"}
{"cmd":"servo_angle", "value":135}
{"cmd":"motor_stop"}

// ESP32 -> Client (Durum bildirimi, 100ms aralikla)
{"rpm":1250, "duty":65, "dir":"forward", "servo":135, "heap":142356}
```

---

## ESP-IDF Yapilandirma Adimlari

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(websocket_motor_control)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "web_server.c" "motor_controller.c"
    INCLUDE_DIRS "." "../inc"
    EMBED_FILES "index.html"
)
```

### menuconfig Ayarlari

| Menu Yolu | Ayar | Deger |
|-----------|------|-------|
| Component config > HTTP Server | Max URI Handlers | 8 |
| Component config > HTTP Server | Max Open Sockets | 4 |
| Component config > ESP HTTPS Server | Enable HTTPS | No (HTTP yeterli) |

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
            +-------------+-------------+
            |             |             |
            v             v             v
     +------------+ +------------+ +------------+
     | WiFi Init  | | Motor Init | | Encoder    |
     | (STA mode) | | (LEDC PWM) | | Init (ISR) |
     +------+-----+ +------+-----+ +------+-----+
            |             |             |
            +-------+-----+-------------+
                    |
                    v
            +---------------+
            | WiFi Connect  |
            | IP al         |
            +-------+-------+
                    |
                    v
            +---------------+
            | HTTP Server   |
            | baslar        |
            |  GET /        |
            |  WS /ws       |
            +-------+-------+
                    |
          +---------+---------+
          |                   |
    +-----v------+     +------v------+
    | HTTP GET   |     | WebSocket   |
    | index.html |     | handler     |
    | (statik)   |     | (dinamik)   |
    +------------+     +------+------+
                              |
                    +---------+---------+
                    |                   |
              +-----v------+    +------v------+
              | WS Receive |    | WS Send     |
              | (komutlar) |    | (100ms timer)|
              | motor_speed|    | rpm, duty,  |
              | motor_dir  |    | servo, heap |
              | servo_angle|    |             |
              +------------+    +-------------+
```

---

## Test Proseduru

### 1. Derleme ve Yukleme

```bash
idf.py -p COM3 flash monitor
```

### 2. Web Arayuzune Erisim

```
Seri monitorden IP adresini not edin:
I (2500) main: WiFi connected, IP: 192.168.1.50

Tarayicida acin: http://192.168.1.50/
```

### 3. Beklenen Seri Monitor Ciktisi

```
I (300) main: === WebSocket RealTime Motor Control ===
I (2500) wifi: Connected, IP: 192.168.1.50
I (2510) httpd: HTTP server started on port 80
I (2515) httpd: WebSocket URI /ws registered
I (5200) ws: WebSocket client connected (fd=3)
I (5300) ws: Received: {"cmd":"motor_speed","value":65}
I (5305) motor: PWM duty set to 65%
I (5400) ws: Status sent: {"rpm":1250,"duty":65,"dir":"forward","servo":90}
```

### 4. Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| WiFi baglantisi | IP adresi alinir | [ ] |
| HTTP sunucu | http://IP/ tarayicida acilir | [ ] |
| WebSocket baglanti | Tarayici WS baglantisi kurar | [ ] |
| Motor hiz kontrolu | Slider ile PWM degisir | [ ] |
| Motor yon degistirme | Ileri/geri butonlari calisir | [ ] |
| Servo kontrol | Aci slider'i servo'yu dondurur | [ ] |
| Gercek zamanli geri bildirim | RPM ve duty ekranda gorulur | [ ] |
| Coklu istemci | Birden fazla tarayici baglanabilir | [ ] |
| Baglanti kopma | WS disconnect olayinda motor durur | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| Web sayfasi acilmiyor | WiFi bagli degil | IP adresini seri monitorden kontrol et |
| WebSocket baglanmiyor | Port engellenmis | Firewall/antivirus kontrolu yap |
| Motor donmuyor | Kablo baglantisi | IN1/IN2/ENA pinlerini kontrol et |
| PWM sinyali yok | LEDC yapilandirmasi | Kanal ve timer ayarlarini dogrula |
| Servo titriyor | Guc yetersiz | Servo icin ayri 5V guc kaynagi kullan |
| Servo tepki vermiyor | Yanlis pulse | 500-2500us araligini kontrol et |
| Encoder saymiyor | ISR calismiyor | GPIO interrupt modu kontrol et |
| Yuksek gecikme | WiFi yogunlugu | 5GHz AP veya ESP32'ye yakin konumlan |
| Sayfa bos gorunuyor | HTML embed edilmemis | CMakeLists.txt EMBED_FILES kontrol et |
| Heap yetersiz | Cok fazla WS istemci | Max open sockets'i sinirla |
| JSON parse hatasi | Format uyumsuz | Tarayici konsol loglarini kontrol et |

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP-IDF HTTP Server](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html) | HTTP sunucu API referansi |
| [RFC 6455 - WebSocket Protocol](https://tools.ietf.org/html/rfc6455) | WebSocket protokol spesifikasyonu |
| [ESP-IDF LEDC API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html) | PWM (LEDC) API referansi |
| [L298N Datasheet](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf) | H-Bridge motor surucu |
| [SG90 Servo Datasheet](http://www.ee.ic.ac.uk/pcw/SG90Servo.pdf) | Mikro servo motor |
| [MDN WebSocket API](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket) | JavaScript WebSocket API |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
