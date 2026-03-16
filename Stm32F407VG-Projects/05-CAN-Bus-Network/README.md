# STM32F407VG - CAN Bus Haberlesme Projesi

## Proje Genel Bakis

Bu proje, STM32F407VG mikrodenetleyicisi uzerinde CAN (Controller Area Network) bus
haberlesme protokolunun kapsamli bir sekilde uygulanmasini icerir. Proje, otomotiv ve
endustriyel uygulamalarda yaygin olarak kullanilan CAN protokolunun temel prensiplerini,
donanimsal gereksinimlerini ve yazilimsal implementasyonunu detayli olarak ele almaktadir.

Proje uc farkli calisma modunu destekler:
- **Loopback Modu**: Harici donanimya ihtiyac duymadan test yapilabilir
- **TX (Verici) Modu**: Periyodik olarak CAN mesajlari gonderir
- **RX (Alici) Modu**: CAN bus uzerinden gelen mesajlari alir ve isler

---

## Icerik Tablosu

1. [CAN Protokolu Nedir?](#can-protokolu-nedir)
2. [CAN Fiziksel Katmani](#can-fiziksel-katmani)
3. [CAN Cerceve Yapisi](#can-cerceve-yapisi)
4. [Arbitrasyon Mekanizmasi](#arbitrasyon-mekanizmasi)
5. [Hata Yonetimi](#hata-yonetimi)
6. [Bit Zamanlama ve Senkronizasyon](#bit-zamanlama-ve-senkronizasyon)
7. [STM32 bCAN Kaydedicileri](#stm32-bcan-kaydedicileri)
8. [Filtre Banklari](#filtre-banklari)
9. [CAN Transceiver Devreleri](#can-transceiver-devreleri)
10. [Otomotiv Uygulamalari](#otomotiv-uygulamalari)
11. [Pin Diyagrami ve Baglanti Semasi](#pin-diyagrami-ve-baglanti-semasi)
12. [STM32CubeMX Yapilandirmasi](#stm32cubemx-yapilandirmasi)
13. [Test Proseduru](#test-proseduru)
14. [Sorun Giderme](#sorun-giderme)

---

## CAN Protokolu Nedir?

CAN (Controller Area Network), 1986 yilinda Robert Bosch GmbH tarafindan otomotiv
endustrisi icin gelistirilmis, coklu ana (multi-master) yapisinda, mesaj tabanli bir
seri haberlesme protokoludur. ISO 11898 standardi ile tanimlanmistir.

### CAN Protokolunun Temel Ozellikleri

| Ozellik | Aciklama |
|---------|----------|
| **Topoloji** | Lineer bus (iki ucta sonlandirma direnci) |
| **Hiz** | 1 Mbit/s (40m), 125 Kbit/s (500m) |
| **Kablo** | Diferansiyel cift bukumlu (twisted pair) |
| **Dugum Sayisi** | Maksimum 127 dugum (teorik) |
| **Mesaj Onceliklendirme** | Arbitrasyon ile catisma cozumu |
| **Hata Tespiti** | CRC, bit izleme, cerceve kontrolu |
| **Standart** | ISO 11898-1 (veri baglantisi), ISO 11898-2 (fiziksel) |

### CAN Versiyonlari

- **CAN 2.0A (Standard CAN)**: 11-bit tanimlayici, 2048 farkli mesaj ID'si
- **CAN 2.0B (Extended CAN)**: 29-bit tanimlayici, ~536 milyon farkli mesaj ID'si
- **CAN FD (Flexible Data-rate)**: Daha yuksek veri hizi ve 64 byte'a kadar veri alani
- **CAN XL**: Yeni nesil, 2048 byte'a kadar veri destegi

### CAN Protokolunun Avantajlari

1. **Gercek Zamanli Iletisim**: Oncelik tabanli arbitrasyon ile deterministik gecikme
2. **Guclu Hata Tespiti**: Bes farkli hata tespit mekanizmasi
3. **Coklu Ana Yapilar**: Her dugum mesaj gonderebilir
4. **Diferansiyel Sinyal**: Elektromanyetik girisimlere karsi yuksek dayaniklilik
5. **Otomatik Yeniden Iletim**: Basarisiz mesajlar otomatik olarak tekrar gonderilir
6. **Dusuk Maliyet**: Az sayida kablo ile cok sayida dugum baglantisi

---

## CAN Fiziksel Katmani

### Diferansiyel Sinyal Iletimi

CAN bus, diferansiyel sinyal iletimi kullanir. Iki hat kullanilir:
- **CAN_H (CAN High)**: Yuksek sinyal hatti
- **CAN_L (CAN Low)**: Dusuk sinyal hatti

```
Gerilim Seviyeleri (ISO 11898-2 Yuksek Hiz):

        CAN_H    CAN_L    Fark (CAN_H - CAN_L)
Recessive: 2.5V    2.5V        0V (mantiksal 1)
Dominant:  3.5V    1.5V        2V (mantiksal 0)

          Volt
  5.0 |
  4.0 |
  3.5 |----+     +----+     +----     CAN_H
  3.0 |    |     |    |     |
  2.5 |    +-----+    +-----+         Bosta (Recessive)
  2.0 |
  1.5 |    +-----+    +-----+         CAN_L
  1.0 |----+     +----+     +----
  0.5 |
  0.0 +-----------------------------> Zaman
       Dom  Rec  Dom  Rec  Dom
```

### Sonlandirma Direnci (Termination)

CAN bus hattinin her iki ucuna 120 ohm sonlandirma direnci baglanmalidir.
Bu direncler sinyal yansimalarini onler ve bus empedansini dengeler.

```
  Dugum 1     Dugum 2     Dugum 3     Dugum 4
    |           |           |           |
    |    120R   |           |   120R    |
  --+---[===]---+-----+-----+---[===]---+--  CAN_H
  --+---[===]---+-----+-----+---[===]---+--  CAN_L
    |    120R   |     |     |   120R    |
                    Dugum
                   (ortada)
```

**Onemli**: Sonlandirma direnci olmadan CAN bus dogru calismaz. Toplam bus
empedansi yaklasik 60 ohm olmalidir (iki 120 ohm paralel).

### Kablo Ozellikleri

- **Kablo Tipi**: Ekranli veya ekransiz bukumlu cift (twisted pair)
- **Empedans**: 120 ohm (nominal)
- **Maksimum Kablo Uzunlugu**: Baud rate'e baglidir
  - 1 Mbit/s: 40 metre
  - 500 Kbit/s: 100 metre
  - 250 Kbit/s: 250 metre
  - 125 Kbit/s: 500 metre
  - 10 Kbit/s: 5 kilometre

---

## CAN Cerceve Yapisi

CAN protokolunde dort farkli cerceve tipi bulunur:

### 1. Veri Cercevesi (Data Frame)

Veri tasimak icin kullanilan temel cerceve tipidir.

```
Standard CAN 2.0A Veri Cercevesi:
+-----+----+-----+---+----+-----+--------+-----+-----+---+-----+---+
| SOF | ID | RTR | IDE| r0 | DLC |  DATA  | CRC | ACK | | EOF |IFS|
| 1b  |11b | 1b  | 1b| 1b | 4b  | 0-64b  | 16b | 2b  | | 7b  |3b |
+-----+----+-----+---+----+-----+--------+-----+-----+---+-----+---+

Extended CAN 2.0B Veri Cercevesi:
+-----+------+-----+---+------+---+----+-----+--------+-----+-----+---+-----+---+
| SOF |ID_A  | SRR |IDE| ID_B | RTR| r1 | DLC |  DATA  | CRC | ACK | | EOF |IFS|
| 1b  |11b   | 1b  |1b | 18b  | 1b| 1b | 4b  | 0-64b  | 16b | 2b  | | 7b  |3b |
+-----+------+-----+---+------+---+----+-----+--------+-----+-----+---+-----+---+
```

**Alan Aciklamalari:**

| Alan | Bit | Aciklama |
|------|-----|----------|
| **SOF** | 1 | Start of Frame - Dominant bit ile baslar |
| **ID** | 11/29 | Mesaj tanimlayicisi ve onceligi |
| **RTR** | 1 | Remote Transmission Request (0=Veri, 1=Uzak istek) |
| **IDE** | 1 | Identifier Extension (0=Standard, 1=Extended) |
| **DLC** | 4 | Data Length Code - Veri uzunlugu (0-8 byte) |
| **DATA** | 0-64 | Gercek veri alani (0 ile 8 byte arasi) |
| **CRC** | 15+1 | Cyclic Redundancy Check + Delimiter |
| **ACK** | 1+1 | Onay biti + Delimiter |
| **EOF** | 7 | End of Frame - 7 recessive bit |
| **IFS** | 3 | Intermission - Cerceveler arasi bosluk |

### 2. Uzak Cerceve (Remote Frame)

Bir dugumun baska bir dugumden veri talep etmesi icin kullanilir.
Veri cercevesiyle aynidir ancak RTR biti recessive (1) ve veri alani yoktur.

### 3. Hata Cercevesi (Error Frame)

Bir hata tespit edildiginde hata bildirimi icin kullanilir.

```
+------------------+--------------+
| Error Flag (6b)  | Error Delim  |
| 6 dominant veya  | (8 recessive)|
| 6 recessive bit  |              |
+------------------+--------------+
```

### 4. Asiri Yukleme Cercevesi (Overload Frame)

Alici dugumun islem icin ek zamana ihtiyac duymasi halinde kullanilir.

---

## Arbitrasyon Mekanizmasi

CAN protokolunun en onemli ozelliklerinden biri, tahrip etmeyen (non-destructive)
arbitrasyon mekanizmasidir. Birden fazla dugum ayni anda mesaj gondermeye calistiginda,
en yuksek oncelikli mesaj (en dusuk ID) kazanir.

### Calısma Prensibi

```
Bus Hatti:   __                     ___________
               \___________________/
              SOF  Arbitration Field

Dugum A (ID=0x100 = 001 0000 0000):
Gonderir:    0  0  0  1  0  0  0  0  0  0  0
                                              ↑ Kazanir!

Dugum B (ID=0x101 = 001 0000 0001):
Gonderir:    0  0  0  1  0  0  0  0  0  0  1
                                           ↑ Kaybeder, cekilir
Bus Durumu:  0  0  0  1  0  0  0  0  0  0  0  (Dominant kazanir)
```

### Arbitrasyon Kurallari

1. Tum dugumlar SOF ile ayni anda baslar
2. Her dugum ID bitlerini sirayla gonderir
3. Dominant (0) bit, recessive (1) bit uzerinde baskin gelir
4. Bir dugum recessive gonderip dominant okursa, arbitrasyonu kaybeder
5. Kaybeden dugum gondermeyi durdurur ve alici moda gecer
6. Kazanan dugum kesintisiz olarak mesajini gondermeye devam eder
7. Kaybeden dugum, bus bos kaldiginda tekrar deneyebilir

**Oncelik Sirasi**: Dusuk ID = Yuksek oncelik
- ID 0x000: En yuksek oncelik
- ID 0x7FF: En dusuk oncelik (11-bit)

---

## Hata Yonetimi

CAN protokolu, bes farkli hata tespit mekanizmasi kullanir:

### Hata Tespit Mekanizmalari

#### 1. Bit Izleme (Bit Monitoring)
Gonderici, gonderdigi her biti bus uzerinden okuyarak karsilastirir.
Gonderdigi bit ile okudugu bit farkli ise hata tespit edilir.
(Arbitrasyon alani haricinde)

#### 2. Bit Doldurma Kontrolu (Stuff Check)
CAN protokolunde ardisik 5 ayni bitten sonra zit bir bit eklenir (bit stuffing).
Alici tarafta 6 ardisik ayni bit tespit edilirse stuff hatasi olusur.

```
Bit Stuffing Ornegi:
Orijinal:   1 1 1 1 1 1 0 0 0 0 0 0
Stuffed:    1 1 1 1 1 [0] 1 0 0 0 0 0 [1] 0
                      ^                ^
                  Stuff bit         Stuff bit
```

#### 3. CRC Kontrolu (CRC Check)
15-bit CRC polinomu ile hesaplanan kontrol toplami karsilastirilir.
Polinom: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1

#### 4. Cerceve Kontrolu (Form Check)
Cerceve yapisindaki sabit alanlarin (CRC delimiter, ACK delimiter, EOF)
beklenen degerlere sahip olup olmadigi kontrol edilir.

#### 5. Onay Kontrolu (ACK Check)
Gonderici, ACK slotunda en az bir alicinin dominant bit gondermesini bekler.
Hicbir alici ACK gondermezse, onay hatasi olusur.

### Hata Durum Makinesi

CAN dugumlerinin uc hata durumu vardir:

```
                   TEC < 128 ve REC < 128
  +-----------+  ←------------------------  +---------------+
  | Error     |                              | Error         |
  | Active    |  ----------------------→     | Passive       |
  | (Normal)  |  TEC >= 128 veya REC >= 128  | (Kisitli)     |
  +-----------+                              +---------------+
       ↑                                           |
       |         TEC < 128 ve REC < 128            |
       +----←--------------------------------------+
                                                   |
                         TEC >= 256                 |
                   +---------------+  ←------------+
                   | Bus Off       |
                   | (Devre Disi)  |
                   +---------------+
                         |
                   128 x 11 recessive bit
                         |
                         ↓
                   Error Active'e don
```

| Durum | TEC | REC | Davranis |
|-------|-----|-----|----------|
| **Error Active** | < 128 | < 128 | Normal iletisim, aktif hata cercevesi gonderir |
| **Error Passive** | >= 128 | >= 128 | Iletisim devam eder, pasif hata cercevesi gonderir |
| **Bus Off** | >= 256 | - | Bus'tan tamamen ayrilir, iletisim durur |

### Hata Sayaclari

- **TEC (Transmit Error Counter)**: Gonderim hata sayaci
- **REC (Receive Error Counter)**: Alim hata sayaci
- Basarili iletimde sayac azalir, hatada artar
- Bus-off durumundan kurtulmak icin 128 x 11 recessive bit gerekir

---

## Bit Zamanlama ve Senkronizasyon

### Bit Zamanlama Yapisi

Bir CAN biti dort segmentten olusur:

```
  |<-------------- 1 Bit Suresi (tq cinsinden) ------------->|
  |                                                           |
  +--------+-----------+-------------+-----------+
  |  SYNC  |  PROP_SEG |  PHASE_SEG1 | PHASE_SEG2|
  |  (1tq) | (1-8 tq)  |  (1-8 tq)   | (1-8 tq)  |
  +--------+-----------+-------------+-----------+
                                  ^
                           Ornekleme Noktasi
                          (Sample Point)
```

### Segmentler

| Segment | Aciklama | Uzunluk |
|---------|----------|---------|
| **SYNC_SEG** | Senkronizasyon, kenar gecisleri burada beklenir | 1 tq (sabit) |
| **PROP_SEG** | Fiziksel gecikmeleri telafi eder | 1-8 tq |
| **PHASE_SEG1** | Ornekleme noktasindan once, SJW ile uzatilabilir | 1-8 tq |
| **PHASE_SEG2** | Ornekleme noktasindan sonra, SJW ile kisaltilabilir | 1-8 tq |

### STM32 Bit Zamanlama Hesaplamasi

STM32F407VG'de CAN1 ve CAN2, APB1 bus'a baglidir.
APB1 saat frekansi: 42 MHz (168 MHz / 4)

```
Baud Rate = APB1_Clock / (Prescaler * (1 + BS1 + BS2))

Ornek: 500 Kbit/s icin:
  APB1_Clock = 42 MHz
  Prescaler = 6
  BS1 = 11 tq
  BS2 = 2 tq
  Toplam tq = 1 + 11 + 2 = 14 tq
  Baud Rate = 42,000,000 / (6 * 14) = 500,000 bit/s = 500 Kbit/s
  Ornekleme Noktasi = (1 + 11) / 14 = 85.7%

Ornek: 250 Kbit/s icin:
  Prescaler = 12
  BS1 = 11 tq
  BS2 = 2 tq
  Baud Rate = 42,000,000 / (12 * 14) = 250,000 bit/s

Ornek: 125 Kbit/s icin:
  Prescaler = 24
  BS1 = 11 tq
  BS2 = 2 tq
  Baud Rate = 42,000,000 / (24 * 14) = 125,000 bit/s

Ornek: 1 Mbit/s icin:
  Prescaler = 3
  BS1 = 11 tq
  BS2 = 2 tq
  Baud Rate = 42,000,000 / (3 * 14) = 1,000,000 bit/s
```

### Senkronizasyon

- **Hard Synchronization**: SOF bitinde yapilir, tq sayaci sifirlanir
- **Resynchronization**: Recessive -> Dominant gecislerinde yapilir
- **SJW (Synchronization Jump Width)**: 1-4 tq, faz segmentlerinin
  ayarlanabilecegi maksimum miktar

---

## STM32 bCAN Kaydedicileri

STM32F407VG, iki adet bCAN (basic CAN) modulu icerir:
- **CAN1** (Master): Filtre banklarini yonetir, bagimsiz calisabilir
- **CAN2** (Slave): CAN1'in filtre banklarini paylasir, CAN1 saati gerektirir

### Ana Kaydediciler

#### CAN Master Kontrol Kaydedicisi (CAN_MCR)

| Bit | Alan | Aciklama |
|-----|------|----------|
| 0 | INRQ | Initialization Request - Baslangic modu istegi |
| 1 | SLEEP | Sleep Mode Request - Uyku modu istegi |
| 2 | TXFP | TX FIFO Priority - FIFO onceliklendirme |
| 3 | RFLM | Receive FIFO Locked Mode - Alim FIFO kilidi |
| 4 | NART | No Automatic Retransmission - Otomatik yeniden iletim |
| 5 | AWUM | Automatic Wakeup Mode - Otomatik uyanma |
| 6 | ABOM | Automatic Bus-Off Management - Bus-off yonetimi |
| 7 | TTCM | Time Triggered Communication Mode |

#### CAN Bit Zamanlama Kaydedicisi (CAN_BTR)

| Bit | Alan | Aciklama |
|-----|------|----------|
| 0-9 | BRP | Baud Rate Prescaler (0-1023) |
| 16-19 | TS1 | Time Segment 1 (0-15, +1 tq) |
| 20-22 | TS2 | Time Segment 2 (0-7, +1 tq) |
| 24-25 | SJW | Resync Jump Width (0-3, +1 tq) |
| 30 | LBKM | Loopback Mode |
| 31 | SILM | Silent Mode |

#### CAN Durum Kaydedicileri

- **CAN_MSR**: Master Status - Baslangic modu onayi, uyku onayi
- **CAN_TSR**: Transmit Status - Posta kutusu durumu, iletim sonucu
- **CAN_RF0R/RF1R**: Receive FIFO 0/1 - Bekleyen mesaj sayisi, doluluk

#### CAN Posta Kutulari (Mailbox)

STM32 CAN modulu 3 adet TX posta kutusu ve 2 adet RX FIFO (her biri 3 mesaj) icerir:

```
TX Posta Kutulari:
  +------------------+
  | TX Mailbox 0     |  CAN_TI0R, CAN_TDT0R, CAN_TDL0R, CAN_TDH0R
  +------------------+
  | TX Mailbox 1     |  CAN_TI1R, CAN_TDT1R, CAN_TDL1R, CAN_TDH1R
  +------------------+
  | TX Mailbox 2     |  CAN_TI2R, CAN_TDT2R, CAN_TDL2R, CAN_TDH2R
  +------------------+

RX FIFO'lar:
  +------------------+
  | RX FIFO 0        |  3 mesaj derinliginde
  | (Mesaj 0,1,2)    |  CAN_RI0R, CAN_RDT0R, CAN_RDL0R, CAN_RDH0R
  +------------------+
  | RX FIFO 1        |  3 mesaj derinliginde
  | (Mesaj 0,1,2)    |  CAN_RI1R, CAN_RDT1R, CAN_RDL1R, CAN_RDH1R
  +------------------+
```

### Kesme (Interrupt) Yapisi

| Kesme | Aciklama |
|-------|----------|
| CAN1_TX_IRQn | Iletim tamamlandi |
| CAN1_RX0_IRQn | FIFO 0'da mesaj alindi |
| CAN1_RX1_IRQn | FIFO 1'de mesaj alindi |
| CAN1_SCE_IRQn | Durum degisikligi ve hata kesmesi |

---

## Filtre Banklari

STM32F407VG'de toplam 28 filtre banki bulunur (CAN1 ve CAN2 arasinda paylasilir).
Her filtre banki, gelen mesajlarin ID'sine gore filtreleme yapar.

### Filtre Modlari

#### 1. Maske Modu (Identifier Mask Mode)

Belirli bitleri maskleyerek bir aralik tanimlar:

```
Ornek: Sadece 0x100 - 0x10F araligindaki mesajlari kabul et

  Filtre ID:    0x100  = 001 0000 0000
  Filtre Mask:  0x7F0  = 111 1111 0000
                                  ^^^^
                          Bu bitler onemli degil (don't care)

  Kabul edilen: 0x100, 0x101, ..., 0x10F
  Reddedilen:   0x110, 0x200, vb.
```

#### 2. Liste Modu (Identifier List Mode)

Sadece belirli ID'leri kabul eder:

```
Ornek: Sadece 0x100 ve 0x200 mesajlarini kabul et

  Filtre 1 ID: 0x100
  Filtre 2 ID: 0x200

  Kabul edilen: 0x100, 0x200
  Reddedilen:   Diger tum ID'ler
```

### Filtre Olcegi (Scale)

| Olcek | Aciklama |
|-------|----------|
| **16-bit** | Her bank 2 filtre icerir (sadece Standard ID) |
| **32-bit** | Her bank 1 filtre icerir (Standard veya Extended ID) |

### Filtre Konfigurasyonu

```
Filtre Bank Yapilandirmasi:

  Bank 0:  [Mode][Scale][FIFO][Active] -> 32-bit Mask, FIFO 0
  Bank 1:  [Mode][Scale][FIFO][Active] -> 16-bit List, FIFO 0
  ...
  Bank 13: [Mode][Scale][FIFO][Active] -> CAN1 son banki (varsayilan)
  Bank 14: [Mode][Scale][FIFO][Active] -> CAN2 ilk banki (CAN1_FMR.CAN2SB)
  ...
  Bank 27: [Mode][Scale][FIFO][Active] -> CAN2 son banki
```

### Filtre Atama

- **FIFO 0 veya FIFO 1**: Her filtre, eslesen mesajlari belirli bir FIFO'ya yonlendirir
- **Oncelik**: Dusuk numarali filtre banki yuksek onceliklidir
- **Eslesmeme**: Hicbir filtreyle eslesmeyen mesajlar atilir

---

## CAN Transceiver Devreleri

### CAN Transceiver Nedir?

CAN transceiver, mikrodenetleyicinin dijital CAN TX/RX sinyallerini
fiziksel CAN bus sinyallerine (CAN_H/CAN_L) donusturen arayuz entegresidir.

### Yaygin CAN Transceiver Entegreleri

| Entegre | Uretici | Ozellik |
|---------|---------|---------|
| **MCP2551** | Microchip | Klasik, yaygin, 5V |
| **MCP2562** | Microchip | 3.3V/5V uyumlu, CAN FD destekli |
| **SN65HVD230** | Texas Instruments | 3.3V, dusuk guc |
| **SN65HVD232** | Texas Instruments | 3.3V, standby modu |
| **TJA1050** | NXP | Yuksek hiz, otomotiv sinifi |
| **TJA1051** | NXP | Dusuk guc, sessiz mod |

### Tipik Baglanti Semasi

```
  STM32F407VG                MCP2551 / SN65HVD230
  +-----------+              +-------------------+
  |           |              |                   |
  |  PD1 (TX) |------>-------| TXD          CANH |-----> CAN_H
  |           |              |                   |       (Bus)
  |  PD0 (RX) |------<-------| RXD          CANL |-----> CAN_L
  |           |              |                   |       (Bus)
  |     3.3V  |---->---------| VCC     GND       |----> GND
  |           |              |         Rs        |
  +-----------+              +------|---|---------+
                                    |   |
                                   GND  |
                                  (Slope Control)

  Not: Rs pini slope control icindir:
  - GND'ye baglanirsa: Yuksek hiz modu
  - Direnc ile: Slope kontrollu mod (EMI azaltma)
  - VCC'ye baglanirsa: Standby modu (bazi modellerde)
```

### Transceiver Secim Kriterleri

1. **Calisma Gerilimi**: 3.3V veya 5V (STM32 icin 3.3V uyumlu secilmeli)
2. **Veri Hizi**: Standart CAN veya CAN FD destegi
3. **ESD Korumasi**: Otomotiv uygulamalarinda onemli
4. **Standby/Sleep Modu**: Dusuk guc uygulamalari icin
5. **Sicaklik Araligi**: Otomotiv: -40C ile +125C

---

## Otomotiv Uygulamalari

### Arac Ici CAN Aglari

Modern bir otomobilde birden fazla CAN agi bulunur:

```
  +-------------------------------------------+
  |            Otomobil CAN Aglari            |
  +-------------------------------------------+
  |                                           |
  |  Powertrain CAN (500 Kbit/s)              |
  |  +------+  +------+  +------+  +------+  |
  |  | Motor |  |Sanzim|  | ABS  |  | ESP  |  |
  |  | ECU   |  | ECU  |  | ECU  |  | ECU  |  |
  |  +------+  +------+  +------+  +------+  |
  |                                           |
  |  Body CAN (125 Kbit/s)                    |
  |  +------+  +------+  +------+  +------+  |
  |  |Kapi  |  |Klima |  |Aydin.|  |Koltuk|  |
  |  |Modul |  | ECU  |  |Modul |  |Modul |  |
  |  +------+  +------+  +------+  +------+  |
  |                                           |
  |  Infotainment CAN (250 Kbit/s)            |
  |  +------+  +------+  +------+  +------+  |
  |  |Radyo |  |Navi  |  |Gosterge| |Direksi|  |
  |  |      |  |      |  |Paneli |  |Kumanda|  |
  |  +------+  +------+  +------+  +------+  |
  |                                           |
  |  Gateway ECU (Aglar arasi kopru)          |
  +-------------------------------------------+
```

### OBD-II (On-Board Diagnostics)

OBD-II, arac arizalarinin teshisi icin standartlastirilmis bir arayuzdur.
CAN bus uzerinden calisan OBD-II (ISO 15765), modern araclarda en yaygin
teshis protokoludur.

#### OBD-II CAN Mesaj Yapisi

```
  Teshis Istegi (Tester -> ECU):
  ID: 0x7DF (broadcast) veya 0x7E0-0x7E7 (belirli ECU)
  DLC: 8
  Data: [Byte Sayisi] [Servis Modu] [PID] [00] [00] [00] [00] [00]

  Teshis Yaniti (ECU -> Tester):
  ID: 0x7E8-0x7EF
  DLC: 8
  Data: [Byte Sayisi] [Servis Modu+0x40] [PID] [Veri...] [00] [00]
```

#### Ornek OBD-II Sorgulari

```
  Motor Devri (RPM) Sorgulama:
  TX: ID=0x7DF, Data=[02][01][0C][00][00][00][00][00]
  RX: ID=0x7E8, Data=[04][41][0C][1A][F8][00][00][00]
  RPM = (0x1A * 256 + 0xF8) / 4 = (26*256 + 248) / 4 = 1754 RPM

  Arac Hizi Sorgulama:
  TX: ID=0x7DF, Data=[02][01][0D][00][00][00][00][00]
  RX: ID=0x7E8, Data=[03][41][0D][3C][00][00][00][00]
  Hiz = 0x3C = 60 km/h

  Motor Sogutma Suyu Sicakligi:
  TX: ID=0x7DF, Data=[02][01][05][00][00][00][00][00]
  RX: ID=0x7E8, Data=[03][41][05][7B][00][00][00][00]
  Sicaklik = 0x7B - 40 = 123 - 40 = 83°C
```

### SAE J1939 (Agir Vasita Protokolu)

Kamyon, otobus ve is makinelerinde kullanilan CAN tabanli protokol.
29-bit extended ID kullanir ve 250 Kbit/s hizda calisir.

```
  J1939 29-bit ID Yapisi:
  +----------+----+--------+----------------+
  | Priority | DP |   PGN  | Source Address |
  |  (3 bit) |(1b)|(16 bit)|    (8 bit)     |
  +----------+----+--------+----------------+
```

---

## Pin Diyagrami ve Baglanti Semasi

### STM32F407VG CAN Pin Haritalamasi

STM32F407VG'de CAN pinleri alternatif fonksiyon olarak yapilandirilir:

```
  CAN1 Pin Secenekleri:
  +--------+--------+--------+
  | Fonk.  | Secenek 1 | Secenek 2 |
  +--------+-----------+-----------+
  | CAN1_RX| PA11      | PB8  | PD0  |  <- Bu projede PD0
  | CAN1_TX| PA12      | PB9  | PD1  |  <- Bu projede PD1
  +--------+-----------+------+------+

  CAN2 Pin Secenekleri:
  +--------+-----------+------+
  | Fonk.  | Secenek 1 | Secenek 2 |
  +--------+-----------+-----------+
  | CAN2_RX| PB5       | PB12 |
  | CAN2_TX| PB6       | PB13 |
  +--------+-----------+------+
```

### Bu Projenin Baglanti Semasi

```
  STM32F407VG Discovery Board
  +---------------------------+
  |                           |
  |  PD0 (CAN1_RX) --------+ |-------> CAN Transceiver RXD
  |  PD1 (CAN1_TX) --------+ |-------> CAN Transceiver TXD
  |                           |
  |  PA2 (USART2_TX) ------+ |-------> USB-TTL TX (Debug)
  |  PA3 (USART2_RX) ------+ |-------> USB-TTL RX (Debug)
  |                           |
  |  PD12 (LED Yesil)  [TX]  |  TX mesaj gonderildiginde yanar
  |  PD13 (LED Turuncu)[RX]  |  RX mesaj alindiginda yanar
  |  PD14 (LED Kirmizi)[ERR] |  Hata durumunda yanar
  |  PD15 (LED Mavi)   [HB]  |  Heartbeat - sistem calisiyor
  |                           |
  |  PA0  (User Button)      |  Mod degistirme butonu
  |                           |
  +---------------------------+
            |     |
           GND   3.3V
```

### Tam Devre Semasi

```
                      3.3V
                       |
                      [4.7K]  Pull-up (opsiyonel)
                       |
  STM32        +-------+-------+     CAN Bus
  PD1(TX)----->| TXD       CANH|---->----+----[120R]----+
               |   SN65HVD230  |         |              |
  PD0(RX)<----| RXD       CANL|---->----+----[120R]----+
               |               |         |              |
               | VCC  GND  Rs  |    Diger CAN Dugumu    |
               +--|----|----|--+         |              |
                  |    |    |            +--------------+
                3.3V  GND  GND
                           (Yuksek hiz modu)

  USB-TTL Donusturucu (Debug):
  PA2(TX) -----> RX (USB-TTL)
  PA3(RX) <----- TX (USB-TTL)
  GND ---------- GND
```

---

## STM32CubeMX Yapilandirmasi

### Adim 1: Yeni Proje Olusturma

1. STM32CubeMX'i acin
2. "New Project" > MCU secimi: **STM32F407VGTx**
3. Veya board secimi: **STM32F4DISCOVERY**

### Adim 2: Saat Yapilandirmasi (Clock Configuration)

```
  HSE (8 MHz) -> PLL -> SYSCLK = 168 MHz

  AHB Prescaler  = /1  -> HCLK  = 168 MHz
  APB1 Prescaler = /4  -> PCLK1 = 42 MHz  (CAN1/CAN2 bu bus'ta)
  APB2 Prescaler = /2  -> PCLK2 = 84 MHz
```

**Onemli**: CAN modulu APB1 bus'a bagli oldugundan, APB1 saat frekansini
(42 MHz) baud rate hesaplamasinda kullanin.

### Adim 3: CAN1 Yapilandirmasi

1. **Connectivity** > **CAN1** > **Activated** isaretleyin
2. Pin Atamasi:
   - CAN1_RX: **PD0**
   - CAN1_TX: **PD1**
3. Parameter Settings:
   ```
   Prescaler:           6
   Time Quanta in BS1:  11
   Time Quanta in BS2:  2
   ReSynchronization JW: 1
   Operating Mode:      Normal

   Hesaplama:
   Baud Rate = 42MHz / (6 * (1+11+2)) = 500 Kbit/s
   Sample Point = (1+11)/14 = 85.7%
   ```

4. **NVIC Settings**:
   - CAN1 RX0 interrupt: **Enabled**
   - CAN1 RX1 interrupt: **Enabled**
   - CAN1 TX interrupt: **Enabled**
   - CAN1 SCE interrupt: **Enabled**

### Adim 4: USART2 Yapilandirmasi (Debug)

1. **Connectivity** > **USART2** > Mode: **Asynchronous**
2. Pin Atamasi: PA2 (TX), PA3 (RX)
3. Parameter Settings:
   ```
   Baud Rate:    115200
   Word Length:  8 Bits
   Stop Bits:    1
   Parity:       None
   ```

### Adim 5: GPIO Yapilandirmasi

1. **LED Pinleri** (Output):
   - PD12: GPIO_Output (Yesil LED - TX)
   - PD13: GPIO_Output (Turuncu LED - RX)
   - PD14: GPIO_Output (Kirmizi LED - Error)
   - PD15: GPIO_Output (Mavi LED - Heartbeat)

2. **Buton Pini** (Input):
   - PA0: GPIO_Input (User Button)

### Adim 6: Kod Uretimi

1. **Project** > **Settings**:
   - Toolchain/IDE: **Makefile** veya **STM32CubeIDE**
   - Project Name: **CAN_Bus_Network**
2. **Generate Code** butonuna basin

---

## Test Proseduru

### Test 1: Loopback Modu Testi (Harici Donanim Gerekmez)

Bu test, CAN modulunun dahili loopback modunda calismasini dogrular.
Gonderilen mesajlar dogrudan alim FIFO'suna yonlendirilir.

```
Adimlar:
1. Projeyi derleyin ve karta yukleyin
2. UART terminal baglantisi kurun (115200 baud)
3. Terminal uzerinden "MODE LOOPBACK" komutu gonderin
4. "SEND 100 8 DEADBEEFCAFEBABE" komutu ile test mesaji gonderin
5. Gonderilen mesajin geri alindigini dogrulayin

Beklenen Cikti:
  [CAN] Loopback mode activated
  [CAN TX] ID=0x100 DLC=8 Data=DE AD BE EF CA FE BA BE
  [CAN RX] ID=0x100 DLC=8 Data=DE AD BE EF CA FE BA BE
  [CAN] TX Success - Mailbox: 0
```

### Test 2: Iki Dugum Haberlesme Testi

Iki STM32F407 kart veya bir STM32 kart ile bir CAN adaptoru kullanilir.

```
Gerekli Malzemeler:
- 2x STM32F407VG Discovery Board
- 2x CAN Transceiver modulu (SN65HVD230 veya MCP2551)
- 2x 120 ohm sonlandirma direnci
- Baglanti kablolari

Baglanti:
  Kart 1                    Kart 2
  PD1 -> TXD   CANH ---+--- CANH   TXD <- PD1
  PD0 <- RXD   CANL ---+--- CANL   RXD -> PD0
                  |     |     |
                [120R]  |   [120R]
                  |     |     |
                 GND   GND   GND

Adimlar:
1. Kart 1'i TX moduna ayarlayin: "MODE TX"
2. Kart 2'yi RX moduna ayarlayin: "MODE RX"
3. Kart 1'den mesaj gonderin: "SEND 100 4 01020304"
4. Kart 2'de mesajin alindigini dogrulayin
5. "STATS" komutu ile istatistikleri kontrol edin
```

### Test 3: Filtre Testi

```
Adimlar:
1. Alici karta filtre ekleyin:
   "FILTER 0 100 7F0 MASK"     (0x100-0x10F araligini kabul et)
2. Verici karttan farkli ID'ler ile mesaj gonderin:
   "SEND 100 2 0102"           -> Kabul edilmeli
   "SEND 105 2 0304"           -> Kabul edilmeli
   "SEND 200 2 0506"           -> Reddedilmeli
3. Alici kartta sadece filtrelenen mesajlarin geldigini dogrulayin
```

### Test 4: Hata Senaryolari

```
Test 4a: Bus-Off Kurtarma
1. CAN bus'i fiziksel olarak kesin
2. Mesaj gondermeye calisin
3. Hata sayaclarini izleyin: "ERRORS"
4. Bus-off durumuna gecisi gozlemleyin
5. CAN bus'i tekrar baglayin
6. Otomatik kurtarmayi dogrulayin

Test 4b: Baud Rate Uyumsuzlugu
1. Kart 1: 500 Kbit/s, Kart 2: 250 Kbit/s
2. Iletisim hatasini gozlemleyin
3. Her iki karti ayni baud rate'e ayarlayin
4. Basarili iletisimi dogrulayin
```

### Test 5: Performans Testi

```
Adimlar:
1. TX modunda surekli mesaj gonderimi baslayin
2. 1000 mesaj gonderin ve sureyi olcun
3. Kayip mesaj olup olmadigini kontrol edin: "STATS"
4. Farkli baud rate'lerde tekrarlayin

Beklenen Sonuclar (500 Kbit/s):
- Maksimum mesaj hizi: ~5000 msg/s (standart 8-byte mesaj)
- Kayip mesaj: 0
- Bus kullanim orani: ~%70-80
```

---

## Sorun Giderme

### Sik Karsilasilan Sorunlar

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| CAN baslatma hatasi | Yanlis saat yapilandirmasi | APB1 saatini kontrol edin |
| Mesaj gonderilemiyor | Sonlandirma direnci eksik | 120R direnc ekleyin |
| Mesaj alinmiyor | Filtre yapilandirmasi yanlis | Tum mesajlari kabul eden filtre ekleyin |
| Bus-off durumu | Fiziksel baglanti hatasi | Kablolari ve transceiver'i kontrol edin |
| ACK hatasi | Bus'ta baska dugum yok | Loopback modu kullanin veya 2. dugum ekleyin |
| CRC hatasi | Sinyal butunlugu bozuk | Kablo uzunlugunu ve sonlandirmayi kontrol edin |
| Baud rate uyumsuzlugu | Farkli prescaler degerleri | Tum dugumlerde ayni baud rate ayarlayin |

### Debug Komutlari

```
UART uzerinden kullanilabilir komutlar:

  MODE LOOPBACK  - Loopback test moduna gec
  MODE TX        - Verici moduna gec
  MODE RX        - Alici moduna gec
  SEND id dlc data - CAN mesaji gonder
  FILTER bank id mask type - Filtre ekle
  STATS          - Iletisim istatistiklerini goster
  ERRORS         - Hata sayaclarini goster
  BAUD rate      - Baud rate degistir (125/250/500/1000)
```

---

## Kaynaklar ve Referanslar

1. **STM32F407 Reference Manual** (RM0090) - ST Microelectronics
   - Bolum 32: bCAN (Controller Area Network)
2. **STM32F4 HAL CAN Driver** - STM32Cube HAL Documentation
3. **ISO 11898-1:2015** - CAN Data Link Layer
4. **ISO 11898-2:2016** - CAN Physical Layer (High Speed)
5. **ISO 15765-2** - CAN Transport Protocol (ISO-TP)
6. **SAE J1939** - Heavy Duty Vehicle Communication
7. **Bosch CAN Specification 2.0** - Orijinal CAN protokol spesifikasyonu

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. STM32 HAL kutuphaneleri
ST Microelectronics lisansi altindadir.

---

**Yazar**: STM32 CAN Bus Egitim Projesi
**Tarih**: 2026
**Platform**: STM32F407VG Discovery Board
**Gelistirme Ortami**: STM32CubeIDE / Keil MDK / Makefile
