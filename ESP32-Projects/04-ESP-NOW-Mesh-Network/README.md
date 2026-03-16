# 04 - ESP-NOW Mesh Network

## Proje Ozeti

Bu proje, ESP32'lerin **ESP-NOW** protokolunu kullanarak birbirleriyle dogrudan (peer-to-peer) iletisim kurmasini ve **mesh ag topolojisi** ile coklu hop uzerinden veri iletimini saglar. WiFi router veya internet baglantisina ihtiyac duymadan, birden fazla ESP32 cihazi arasinda dusuk gecikmeli, guvenilir bir kablosuz ag olusturulur. Proje, AODV benzeri bir routing algoritmasiyla rota kesfinden, TTL ile mesaj iletiminden, duplike tespit mekanizmasindan ve sifreli iletisimden olusur.

---

## Teorik Arka Plan

### ESP-NOW Protokolu

ESP-NOW, Espressif tarafindan gelistirilmis, WiFi Action Frame tabanli bir peer-to-peer iletisim protokoludur.

| Ozellik | Deger |
|---------|-------|
| Maksimum Peer Sayisi | 20 (sifrelenmis: 10, sifresiz: 20) |
| Maksimum Payload | 250 byte |
| MAC Tabanli Adresleme | Evet |
| Baglanti Gerektirme | Hayir (connectionless) |
| Sifreli Iletisim | AES-128 (PMK + LMK) |
| WiFi Baglantisi Gerekli | Hayir (STA modu yeterli) |
| Gecikme | < 1 ms (ayni kanaldaki peerler) |
| Kanal | WiFi kanallarini kulllanir (1-14) |

**ESP-NOW Mesaj Akisi:**

```
  Gonderici (Node A)                   Alici (Node B)
       |                                     |
       |-- esp_now_send(MAC_B, data) ------->|
       |                                     |  esp_now_recv_cb() tetiklenir
       |<-- esp_now_send_cb(status) ---------|  (ACK icin)
       |    MAC_STATUS_SUCCESS               |
       |                                     |
  Not: ESP-NOW, WiFi Action Frame kullanir.
  802.11 MAC katmaninda ACK mekanizmasi vardir.
```

### Mesh Ag Topolojileri

```
  1) STAR (Yildiz):               2) TREE (Agac):

       +---+                           +---+
       | A |                           | A | (Root)
       +---+                           +---+
      / | \ \                         /     \
     /  |  \  \                      /       \
  +---++---++---++---+           +---+       +---+
  | B || C || D || E |           | B |       | C |
  +---++---++---++---+           +---+       +---+
                                 / \           |
                              +---++---+     +---+
                              | D || E |     | F |
                              +---++---+     +---+

  3) MESH (Ag):

  +---+-------+---+
  | A |-------| B |
  +---+\     /+---+
    |   \   /   |
    |    \ /    |
    |     X     |
    |    / \    |
    |   /   \   |
  +---+/     \+---+
  | C |-------| D |
  +---+-------+---+
    \           /
     \         /
      \       /
       +---+
       | E |
       +---+

  Mesh topolojisinde her dugu birden fazla komsusuna baglidir.
  Bir yol kopsa bile alternatif yollar uzerinden iletisim devam eder.
```

### Paket Yapisi

```
  +------+------+------+------+--------+------+---------+
  | Type | Src  | Dst  | Seq  | TTL    | Hop  | Payload |
  | 1B   | 6B   | 6B   | 2B   | 1B     | 1B   | 0-233B  |
  +------+------+------+------+--------+------+---------+

  Type: Mesaj turu
    0x01 = DATA         (Veri mesaji)
    0x02 = BEACON       (Dugu kesfetme)
    0x03 = ROUTE_REQ    (Rota istegi)
    0x04 = ROUTE_REPLY  (Rota yaniti)
    0x05 = ACK          (Onay)
    0x06 = PING         (Baglanti testi)
    0x07 = PONG         (Ping yaniti)

  Src:  Kaynak MAC adresi (6 byte)
  Dst:  Hedef MAC adresi (6 byte, FF:FF:FF:FF:FF:FF = broadcast)
  Seq:  Sira numarasi (duplike tespiti icin)
  TTL:  Time-To-Live (her hop'ta 1 azalir, 0'da dusurulur)
  Hop:  Simdiye kadar gecen hop sayisi
```

### AODV Benzeri Routing

```
  Rota Kesfetme (Route Discovery):

  Node A, Node D'ye veri gondermek istiyor ama rota bilmiyor:

  1. ROUTE_REQ broadcast:
     A --[RREQ: dst=D, seq=1]--> B, C
     B --[RREQ: dst=D, seq=1]--> C, D    (B, A'ya ters rota kaydeder)
     D RREQ'yi alir (hedef kendisi!)

  2. ROUTE_REPLY unicast:
     D --[RREP: dst=A, hop=0]--> B       (D, A'ya rota bilir)
     B --[RREP: dst=A, hop=1]--> A       (B, D'ye rota bilir)

  3. Veri iletimi:
     A --[DATA: dst=D]--> B --[DATA: dst=D]--> D
```

### Routing Tablosu Ornegi

```
  Node B'nin routing tablosu:
  +-------------+-----------+-----------+----------+---------+
  | Hedef MAC   | Sonraki   | Hop       | Son      | Gecerli |
  |             | Hop MAC   | Sayisi    | Guncelle |         |
  +-------------+-----------+-----------+----------+---------+
  | AA:BB:CC:01 | Dogrudan  | 0         | 12345    | Evet    |
  | (Node A)    |           |           |          |         |
  +-------------+-----------+-----------+----------+---------+
  | AA:BB:CC:03 | Dogrudan  | 0         | 12340    | Evet    |
  | (Node C)    |           |           |          |         |
  +-------------+-----------+-----------+----------+---------+
  | AA:BB:CC:04 | AA:BB:CC:03| 1        | 12300    | Evet    |
  | (Node D)    | (Node C)  |           |          |         |
  +-------------+-----------+-----------+----------+---------+
  | AA:BB:CC:05 | AA:BB:CC:01| 2        | 12200    | Evet    |
  | (Node E)    | (Node A)  |           |          |         |
  +-------------+-----------+-----------+----------+---------+
```

---

## Pin Baglanti Semasi

```
    ESP32 #1 (Node A)              ESP32 #2 (Node B)
    +------------------+           +------------------+
    |  GPIO2  (LED)    |---[330R]  |  GPIO2  (LED)    |---[330R]
    |  USB (UART0)     |--- PC     |  USB (UART0)     |--- PC
    |  3V3 / GND       |           |  3V3 / GND       |
    +------------------+           +------------------+

    ESP32 #3 (Node C)
    +------------------+
    |  GPIO2  (LED)    |---[330R]
    |  USB (UART0)     |--- PC
    |  3V3 / GND       |
    +------------------+

    Not: ESP-NOW dahili WiFi radyosunu kullanir.
    Harici anten veya ek donanim gerekmez.
    Her node'a LED baglantisi aktivite gostergesi icindir.

    Tum node'lar AYNI WiFi kanalinda olmalidir (varsayilan: 1).
```

---

## ESP-IDF Yapilandirma Adimlari

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espnow_mesh_network)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "espnow_manager.c" "mesh_protocol.c"
    INCLUDE_DIRS "." "../inc"
)
```

### Derleme (Her Node Icin)

```bash
# Node rolunu menuconfig veya main.c'de ayarlayin
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor    # Node A
idf.py -p COM4 flash monitor    # Node B (farkli port)
idf.py -p COM5 flash monitor    # Node C
```

---

## Kodun Calisma Mantigi

```
                      app_main()
                          |
                          v
                +-------------------+
                | NVS Init          |
                +--------+----------+
                         |
                         v
                +-------------------+
                | WiFi Init         |
                | (STA mode, kanal) |
                +--------+----------+
                         |
                         v
                +-------------------+
                | ESP-NOW Init      |
                | (send/recv cb)    |
                +--------+----------+
                         |
                         v
                +-------------------+
                | Mesh Protocol Init|
                | (routing table)   |
                +--------+----------+
                         |
           +-------------+-------------+
           |                           |
    +------v-------+           +-------v------+
    | beacon_task  |           | message_task |
    | (Periyodik   |           | (Mesaj isleme|
    |  broadcast   |           |  ve yonlendir|
    |  beacon)     |           |  me)         |
    +--------------+           +--------------+
           |                           |
           v                           v
    +-------------------+     +-------------------+
    | Komsulari kesfet  |     | UART'tan komut al |
    | Routing guncelle  |     | Mesh uzerinden    |
    | Node durumu yayinla|    | veri gonder/al    |
    +-------------------+     +-------------------+
```

---

## Test Proseduru

### 1. 3 ESP32 Hazirla

Her ESP32'ye firmware yukleyin. MAC adreslerini seri monitorden not edin.

### 2. Node Kesfetme Testi

```
# Node A seri monitor:
I (1000) mesh: Beacon sent (broadcast)
I (3500) mesh: New node discovered: AA:BB:CC:DD:EE:02 (Node B)
I (4200) mesh: New node discovered: AA:BB:CC:DD:EE:03 (Node C)
I (5000) mesh: Routing table: 2 entries
```

### 3. Dogrudan Mesaj Testi

```
# Node A UART'a yazin: send <Node_B_MAC> "Hello"
I (10000) mesh: Sending DATA to AA:BB:CC:DD:EE:02 (direct)
I (10001) mesh: Send callback: SUCCESS

# Node B seri monitor:
I (10001) mesh: DATA received from AA:BB:CC:DD:EE:01: "Hello"
```

### 4. Multi-Hop Testi

Node A'yi Node C'nin menzili disina cikarin. B aracilik etmelidir.

### 5. Dogrulama Kontrol Listesi

| Test | Beklenen Sonuc | Durum |
|------|----------------|-------|
| Node kesfetme | Beacon ile komsular algilanir | [ ] |
| Dogrudan mesaj | Peer'a veri gonderilir ve alinir | [ ] |
| Multi-hop iletim | Aracilik eden node mesaji iletir | [ ] |
| TTL mekanizmasi | TTL=0 olan mesaj dusurulur | [ ] |
| Duplike tespiti | Ayni seq numarali mesaj alinmaz | [ ] |
| Routing tablosu | Rotalar dogru olusur | [ ] |
| Broadcast mesaj | Tum node'lar alir | [ ] |
| Node disconnect | Rota tablosu guncellenir | [ ] |

---

## Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| ESP-NOW init hatasi | WiFi baslatilmamis | WiFi STA modu baslatildiktan sonra ESP-NOW init yapin |
| Peer eklenemiyor | Maksimum peer | Sifrelenmis peer siniri 10, toplam 20 |
| Mesaj iletilmiyor | Farkli WiFi kanali | Tum node'larin ayni kanalda oldugunu dogrula |
| Send callback: FAIL | Peer menzil disi | Mesafeyi kisalt veya multi-hop kullan |
| Beacon alinmiyor | Broadcast adresi yanlis | FF:FF:FF:FF:FF:FF kullanildigini kontrol et |
| Routing tablosu bos | Beacon periyodu fazla uzun | Beacon araligini 3-5 saniyeye dusur |
| Duplike mesajlar | Sequence counter hatasi | Her node benzersiz seq counter kullanmali |
| Sifreleme hatasi | PMK/LMK uyumsuzlugu | Tum node'larda ayni PMK ayarini dogrula |
| Yuksek gecikme | Cok fazla hop | TTL degerini ve topolojiyi optimize et |
| Bellek tasmasi | Routing tablosu buyuk | Maks route sayisini sinirla, eski rotalari sil |

---

## Kaynaklar

| Kaynak | Aciklama |
|--------|----------|
| [ESP-IDF ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html) | ESP-NOW API referansi |
| [ESP-NOW Kullanim Kilavuzu](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/esp-now.html) | ESP-NOW rehberi |
| [AODV RFC 3561](https://www.rfc-editor.org/rfc/rfc3561) | Ad-hoc On-demand Distance Vector protokolu |
| [IEEE 802.11 Action Frames](https://standards.ieee.org/ieee/802.11/7028/) | WiFi Action Frame spesifikasyonu |
| [ESP32 WiFi API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html) | WiFi API referansi |

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. [MIT Lisansi](../../LICENSE) altinda dagitilmaktadir.
