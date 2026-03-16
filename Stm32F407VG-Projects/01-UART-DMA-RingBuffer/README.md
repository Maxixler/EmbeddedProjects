# STM32F407VG - UART DMA Ring Buffer Projesi

## Proje Ozeti

Bu proje, STM32F407VG Discovery gelistirme kartinda **UART haberlesme protokolunun DMA (Direct Memory Access) ve Ring Buffer (Dairesel Tampon)** veri yapisi kullanilarak verimli bir sekilde gerceklenmesini gostermektedir. Proje, gomulu sistemlerde yuksek performansli seri haberlesme icin endustri standardinda bir yaklasim sunmaktadir.

### Projenin Amaci

- CPU'yu mesgul etmeden yuksek hizda UART veri transferi gerceklestirmek
- Degisken uzunluktaki veri paketlerini kayip olmadan almak
- Thread-safe (is parcacigi guvenli) ring buffer ile veri yonetimi saglamak
- AT komut tabanli bir haberlesme protokolu uygulamak

---

## Teorik Arka Plan

### 1. UART (Universal Asynchronous Receiver/Transmitter) Protokolu

UART, iki cihaz arasinda asenkron seri haberlesme saglayan bir protokoldur. "Asenkron" olmasi, veri transferi icin ayri bir clock (saat) hattina ihtiyac duyulmadigi anlamina gelir. Bunun yerine, her iki taraf da onceden anlasilmis bir baud rate (bit hizi) kullanir.

#### UART Cerceve Yapisi

```
     ___     ___ ___ ___ ___ ___ ___ ___ ___     ___     ___
    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
    | I |   | S | D | D | D | D | D | D | D | D | P | S | I |
    | D |   | T | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | A | T | D |
    | L |   | A |   |   |   |   |   |   |   |   | R | O | L |
    | E |   | R |   |   |   |   |   |   |   |   | I | P | E |
    |   |   | T |   |   |   |   |   |   |   |   | T |   |   |
    |___|   |___|___|___|___|___|___|___|___|___|_Y_|___|___|

    IDLE : Hat bos durumu (logic HIGH)
    START: Baslangic biti (logic LOW) - 1 bit
    D0-D7: Veri bitleri (LSB first) - 8 veya 9 bit
    PARITY: Eslik biti (opsiyonel) - 0 veya 1 bit
    STOP : Durus biti (logic HIGH) - 1 veya 2 bit
```

#### Baud Rate Hesaplama

Baud rate, saniyede iletilen sembol (bit) sayisidir:

```
USARTDIV = f_CK / (8 x (2 - OVER8) x Baud_Rate)

Ornek (115200 baud, 84 MHz APB1 clock):
USARTDIV = 84000000 / (16 x 115200) = 45.5729
Mantissa = 45 (0x2D)
Fraction = 0.5729 x 16 = 9.17 ~= 9
BRR = 0x2D9
```

### 2. DMA (Direct Memory Access) Nedir?

DMA, CPU'nun muedahalesine gerek kalmadan, cevre birimleri ile bellek arasinda (veya bellek ile bellek arasinda) dogrudan veri transferi yapan bir donanim birimidir.

#### DMA Olmadan (Polling/Interrupt):

```
  CPU                Bellek              UART
   |                   |                   |
   |<------ oku -------|                   |
   |------- yaz ------>|                   |  Her byte icin
   |<----- oku --------|                   |  CPU mesgul
   |                   |------- gonder --->|
   |                   |                   |
```

#### DMA ile:

```
  CPU                DMA                Bellek              UART
   |                  |                   |                   |
   |-- konfig et ---->|                   |                   |
   |  (baska is yap)  |<------ oku -------|                   |
   |  (baska is yap)  |                   |------- gonder --->|
   |  (baska is yap)  |<------ oku -------|                   |
   |  (baska is yap)  |                   |------- gonder --->|
   |<-- TC kesme ------|                   |                   |
   |                  |                   |                   |
```

#### STM32F407VG DMA Ozellikleri

STM32F407VG'de 2 adet DMA kontrolcusu bulunur:

| Ozellik | DMA1 | DMA2 |
|---------|------|------|
| Stream Sayisi | 8 (Stream 0-7) | 8 (Stream 0-7) |
| Kanal Sayisi | Her stream icin 8 kanal (Ch0-Ch7) | Her stream icin 8 kanal (Ch0-Ch7) |
| Veri Genisligi | 8/16/32 bit | 8/16/32 bit |
| FIFO | Her stream icin 4-word FIFO | Her stream icin 4-word FIFO |
| Oncelik | 4 seviye (Low-Medium-High-Very High) | 4 seviye |

**Bu projede kullanilan DMA konfigurasyon:**

| Fonksiyon | DMA | Stream | Kanal | Mod |
|-----------|-----|--------|-------|-----|
| USART2_RX | DMA1 | Stream 5 | Channel 4 | Circular (Dairesel) |
| USART2_TX | DMA1 | Stream 6 | Channel 4 | Normal |

### 3. Ring Buffer (Dairesel Tampon) Veri Yapisi

Ring buffer, sabit boyutlu bir dizinin baslangic ve bitis noktalarini birbirine baglayarak olusturulan FIFO (First-In-First-Out) veri yapisidir.

```
    Bos Ring Buffer (size=8):        Veri yazildiktan sonra:

    +---+---+---+---+---+---+---+---+    +---+---+---+---+---+---+---+---+
    |   |   |   |   |   |   |   |   |    | A | B | C | D |   |   |   |   |
    +---+---+---+---+---+---+---+---+    +---+---+---+---+---+---+---+---+
      ^                                    ^               ^
      |                                    |               |
    head=0                               tail=0          head=4
    tail=0                               (okuma)         (yazma)
    count=0                              count=4

    Bazi veriler okuduktan sonra:     Buffer sarma (wrap-around):

    +---+---+---+---+---+---+---+---+    +---+---+---+---+---+---+---+---+
    |   |   | C | D |   |   |   |   |    | H | I |   |   | E | F | G |   |
    +---+---+---+---+---+---+---+---+    +---+---+---+---+---+---+---+---+
              ^       ^                            ^       ^
              |       |                            |       |
            tail=2  head=4                       head=2  tail=4
            count=2                              count=5
```

#### Ring Buffer Avantajlari

1. **Sabit bellek kullanimi**: Dinamik bellek tahsisi gerektirmez
2. **O(1) islem suresi**: Okuma ve yazma islemleri sabit zamanli
3. **Dogal FIFO davranisi**: Veri sirasi korunur
4. **Wrap-around**: Buffer sonuna ulasildiginda otomatik olarak basa doner
5. **Gomulu sistemler icin ideal**: malloc/free gerektirmez, bellek parcalanmasina yol acmaz

### 4. Polling vs Interrupt vs DMA Karsilastirmasi

| Ozellik | Polling | Interrupt | DMA |
|---------|---------|-----------|-----|
| **CPU Kullanimi** | %100 (surekli kontrol) | Dusuk (sadece kesme aninda) | Minimum (sadece konfigurasyon) |
| **Gecikme (Latency)** | Degisken | Dusuk | En dusuk |
| **Veri Kaybi Riski** | Yuksek (CPU mesgulse) | Dusuk | En dusuk |
| **Karmasiklik** | En basit | Orta | Yuksek |
| **Throughput** | Dusuk | Orta | Yuksek |
| **Enerji Tuketimi** | Yuksek | Orta | Dusuk |
| **Coklu Transfer** | Zor | Orta | Kolay |
| **Gercek Zamanlilik** | Zayif | Iyi | En iyi |
| **Kullanim Alani** | Basit, dusuk hiz | Genel amacli | Yuksek hiz, buyuk veri |

#### Neden DMA + Ring Buffer?

- **Yuksek baud rate'lerde** (>= 115200) polling ile veri kaybi yasanir
- **Interrupt ile** her byte icin kesme olusur, yuksek hizda CPU bant genisligi harcanir
- **DMA ile** veriler otomatik olarak bellek tamponuna aktarilir
- **Ring buffer ile** CPU, veriyi istendigi zaman isleyebilir
- **IDLE line detection** ile degisken uzunluktaki paketler tespit edilir

---

## STM32F407VG UART ve DMA Donanim Bloklari

### USART Register'lari (Yazmaclari)

#### USART_SR (Status Register) - Durum Yazmaci

```
Bit 31:10  Rezerve
Bit 9      CTS   : CTS bayragi
Bit 8      LBD   : LIN break algilama
Bit 7      TXE   : TX tampon bos (1 = bos, veri yazilabilir)
Bit 6      TC    : Iletim tamamlandi (1 = tum veri gonderildi)
Bit 5      RXNE  : RX tampon dolu degil (1 = veri var, okunabilir)
Bit 4      IDLE  : IDLE hat algilandi (1 = hat bos)
Bit 3      ORE   : Overrun hatasi (veri kaybi)
Bit 2      NF    : Gurultu hatasi
Bit 1      FE    : Cerceveleme hatasi
Bit 0      PE    : Eslik hatasi
```

#### USART_DR (Data Register) - Veri Yazmaci

```
Bit 31:9   Rezerve
Bit 8:0    DR    : Veri degeri (8 veya 9 bit)
                   Yazma: TX tamponuna veri yazar
                   Okuma: RX tamponundan veri okur
```

#### USART_BRR (Baud Rate Register) - Baud Rate Yazmaci

```
Bit 31:16  Rezerve
Bit 15:4   DIV_Mantissa : USARTDIV tam kismi
Bit 3:0    DIV_Fraction : USARTDIV kesirli kismi
```

#### USART_CR1 (Control Register 1) - Kontrol Yazmaci 1

```
Bit 15     OVER8  : Oversampling modu (0=16x, 1=8x)
Bit 13     UE     : USART etkinlestirme
Bit 12     M      : Kelime uzunlugu (0=8bit, 1=9bit)
Bit 10     PCE    : Eslik kontrolu etkinlestirme
Bit 9      PS     : Eslik secimi (0=cift, 1=tek)
Bit 7      TXEIE  : TXE kesme etkinlestirme
Bit 6      TCIE   : TC kesme etkinlestirme
Bit 5      RXNEIE : RXNE kesme etkinlestirme
Bit 4      IDLEIE : IDLE kesme etkinlestirme
Bit 3      TE     : Verici etkinlestirme
Bit 2      RE     : Alici etkinlestirme
```

#### USART_CR3 (Control Register 3) - Kontrol Yazmaci 3

```
Bit 7      DMAT   : DMA verici etkinlestirme
Bit 6      DMAR   : DMA alici etkinlestirme
```

### DMA Register'lari

#### DMA_SxCR (Stream x Configuration Register)

```
Bit 27:25  CHSEL  : Kanal secimi (0-7)
Bit 17:16  PL     : Oncelik seviyesi (00=Low, 11=VeryHigh)
Bit 14:13  MSIZE  : Bellek veri boyutu (00=byte, 01=half-word, 10=word)
Bit 12:11  PSIZE  : Cevre birimi veri boyutu
Bit 10     MINC   : Bellek adresi artirma (1=arttir)
Bit 9      PINC   : Cevre birimi adresi artirma (0=artirma)
Bit 8      CIRC   : Dairesel mod (1=dairesel)
Bit 7:6    DIR    : Transfer yonu (00=P->M, 01=M->P, 10=M->M)
Bit 4      TCIE   : Transfer tamamlama kesme etkinlestirme
Bit 3      HTIE   : Yari transfer kesme etkinlestirme
Bit 0      EN     : Stream etkinlestirme
```

#### DMA_SxNDTR (Stream x Number of Data to Transfer Register)

```
Bit 15:0   NDT    : Transfer edilecek veri sayisi
                    Circular modda otomatik yeniden yuklenir
                    Normal modda 0'a ulasinca transfer durur
```

#### DMA_SxPAR (Stream x Peripheral Address Register)

```
Bit 31:0   PAR    : Cevre birimi adresi (ornek: &USART2->DR)
```

#### DMA_SxM0AR (Stream x Memory 0 Address Register)

```
Bit 31:0   M0A    : Bellek adresi (ornek: rx_dma_buffer)
```

---

## Pin Baglanti Semasi

### STM32F407VG Discovery Board - USB-UART Baglantisi

```
    STM32F407VG Discovery Board
    +--------------------------------------------------+
    |                                                  |
    |  +----------+          +------------------+      |
    |  |          |   PA2    |                  |      |
    |  |  USART2  |--------->| TX (USART2_TX)   |----->|----> USB-TTL TX
    |  |          |   PA3    |                  |      |
    |  |          |<---------| RX (USART2_RX)   |<-----|<---- USB-TTL RX
    |  |          |          |                  |      |
    |  +----------+          +------------------+      |
    |                                                  |
    |  +----------+                                    |
    |  |   LED    |  PD12 = Yesil LED  (Veri alimi)    |
    |  |  Blogu   |  PD13 = Turuncu LED (Veri gonderim)|
    |  |          |  PD14 = Kirmizi LED (Hata)         |
    |  |          |  PD15 = Mavi LED (Sistem aktif)    |
    |  +----------+                                    |
    |                                                  |
    +--------------------------------------------------+

    USB-TTL Donusturucu Baglantisi:
    +------------------+          +------------------+
    | STM32 Discovery  |          | USB-TTL (CP2102/ |
    |                  |          | FT232/CH340)     |
    |   PA2 (TX) ------+--------->| RX               |
    |   PA3 (RX) ------+<--------| TX               |
    |   GND -----------+----------| GND              |
    |                  |          |                  |
    +------------------+          +------------------+

    NOT: ST-Link uzerindeki VCP (Virtual COM Port) kullanilacaksa
    ek bir USB-TTL donusturucuye gerek yoktur. ST-Link v2 uzerinde
    PA2/PA3 pinleri zaten VCP'ye baglidir.
```

### DMA Veri Akis Semasi

```
                   DMA1 Stream 5 (RX)
    +--------+    +--------+    +-----------+    +-------------+
    | USART2 |--->| DMA    |--->| RX DMA    |--->| RX Ring     |
    | RX Pin |    | Engine |    | Buffer    |    | Buffer      |
    | (PA3)  |    | (HW)   |    | (256B)    |    | (1024B)     |
    +--------+    +--------+    +-----------+    +------+------+
                                                        |
                                 IDLE/HT/TC             |
                                 Interrupt              v
                                                 +------+------+
                                                 | Uygulama    |
                                                 | Katmani     |
                                                 | (main.c)    |
                                                 +------+------+
                                                        |
                                                        v
                   DMA1 Stream 6 (TX)            +------+------+
    +--------+    +--------+    +-----------+    | TX Ring     |
    | USART2 |<---| DMA    |<---| TX DMA    |<---| Buffer      |
    | TX Pin |    | Engine |    | Buffer    |    | (1024B)     |
    | (PA2)  |    | (HW)   |    | (256B)    |    +-------------+
    +--------+    +--------+    +-----------+
                                 TC Interrupt
```

---

## CubeMX / CubeIDE Konfigurasyon Adimlari

### Adim 1: Yeni Proje Olusturma

1. STM32CubeIDE'yi acin
2. **File -> New -> STM32 Project** secin
3. **MCU/MPU Selector** sekmesinde `STM32F407VGTx` secin
4. Proje adini girin: `UART_DMA_RingBuffer`
5. **Targeted Language**: C
6. **Targeted Binary Type**: Executable

### Adim 2: Clock Konfigurasyonu (RCC)

1. **Pinout & Configuration -> System Core -> RCC**
2. HSE: **Crystal/Ceramic Resonator** secin
3. **Clock Configuration** sekmesine gecin
4. Ayarlar:
   - Input frequency: **8 MHz** (Discovery board kristali)
   - PLL Source: **HSE**
   - PLLM: **8** (VCO input = 1 MHz)
   - PLLN: **336** (VCO output = 336 MHz)
   - PLLP: **/2** (SYSCLK = 168 MHz)
   - PLLQ: **7** (USB clock = 48 MHz)
   - AHB Prescaler: **/1** (HCLK = 168 MHz)
   - APB1 Prescaler: **/4** (PCLK1 = 42 MHz)
   - APB2 Prescaler: **/2** (PCLK2 = 84 MHz)

### Adim 3: USART2 Konfigurasyonu

1. **Pinout & Configuration -> Connectivity -> USART2**
2. Mode: **Asynchronous**
3. Parameter Settings:
   - Baud Rate: **115200**
   - Word Length: **8 Bits**
   - Stop Bits: **1**
   - Parity: **None**
   - Data Direction: **Receive and Transmit**
   - Over Sampling: **16 Samples**
4. DMA Settings sekmesi:
   - **Add** -> USART2_RX -> DMA1 Stream 5
     - Direction: Peripheral To Memory
     - Priority: High
     - Mode: **Circular**
     - Data Width: Byte / Byte
     - Increment Address: Memory = checked
   - **Add** -> USART2_TX -> DMA1 Stream 6
     - Direction: Memory To Peripheral
     - Priority: Medium
     - Mode: **Normal**
     - Data Width: Byte / Byte
     - Increment Address: Memory = checked
5. NVIC Settings:
   - USART2 global interrupt: **Enabled**
   - DMA1 stream5 global interrupt: **Enabled**
   - DMA1 stream6 global interrupt: **Enabled**

### Adim 4: GPIO Konfigurasyonu (LED'ler)

1. Pinout gorunumunde asagidaki pinleri **GPIO_Output** olarak ayarlayin:
   - **PD12**: LD4 (Yesil)
   - **PD13**: LD3 (Turuncu)
   - **PD14**: LD5 (Kirmizi)
   - **PD15**: LD6 (Mavi)
2. Her pin icin:
   - Output Level: Low
   - Mode: Push Pull
   - Pull-up/Pull-down: No pull
   - Speed: Low

### Adim 5: NVIC Oncelikleri

1. **System Core -> NVIC**
2. Priority Group: **4 bits for pre-emption priority**
3. Oncelikler:
   - DMA1 Stream5 (RX): Pre-emption = 1
   - DMA1 Stream6 (TX): Pre-emption = 2
   - USART2: Pre-emption = 1

### Adim 6: Kod Uretme

1. **Project -> Generate Code** (veya Alt+K)
2. Uretilen koda ring_buffer ve uart_dma dosyalarini ekleyin

---

## Kodun Calisma Mantigi

### Genel Akis Diyagrami

```
                    +------------------+
                    |   Sistem Baslat  |
                    +--------+---------+
                             |
                    +--------v---------+
                    | Clock 168MHz     |
                    | GPIO Init        |
                    | UART+DMA Init    |
                    +--------+---------+
                             |
                    +--------v---------+
                    | DMA RX Circular  |
                    | Modda Baslatilir |
                    +--------+---------+
                             |
                +------------v------------+
                |     Ana Dongu (while)   |
                +------------+------------+
                             |
                  +----------v----------+
                  | RX Ring Buffer'da   |   Hayir
                  | veri var mi?        +----------+
                  +----------+----------+          |
                             | Evet                |
                  +----------v----------+          |
                  | Veriyi oku          |          |
                  +----------+----------+          |
                             |                     |
                  +----------v----------+          |
                  | Komut tamamlandi mi?|          |
                  | ('\r' veya '\n')    |          |
                  +----------+----------+          |
                             | Evet                |
                  +----------v----------+          |
                  | Komutu isle         |          |
                  | (AT komut parser)   |          |
                  +----------+----------+          |
                             |                     |
                  +----------v----------+          |
                  | Yaniti TX Ring      |          |
                  | Buffer'a yaz       |          |
                  +----------+----------+          |
                             |                     |
                  +----------v----------+          |
                  | DMA TX baslatilir   |          |
                  | (eger aktif degilse)|          |
                  +----------+----------+          |
                             |                     |
                             +<--------------------+
                             |
                  +----------v----------+
                  | LED durumlarini     |
                  | guncelle           |
                  +----------+----------+
                             |
                             +-----> Ana donguye don
```

### RX Veri Alma Mekanizmasi (DMA + IDLE)

```
    Zaman -->

    UART RX Hatti:
    __|XXXXX|___|XXXXXXXX|___|XXX|___________
      ^     ^   ^        ^   ^   ^
      |     |   |        |   |   |
      |     |   |        |   |   IDLE algilandi
      |     |   |        |   Son byte
      |     |   |        IDLE algilandi (2. paket sonu)
      |     |   Yeni paket baslangici
      |     IDLE algilandi (1. paket sonu)
      Ilk paket baslangici

    DMA Circular Buffer:
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    |H |e |l |l |o |W |o |r |l |d |! |A |B |C |  |  |
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15

    IDLE/HT/TC kesmeleri tetiklendiginde:
    1. DMA_SxNDTR okunur (kalan transfer sayisi)
    2. Yeni veri pozisyonu hesaplanir
    3. Yeni veriler RX Ring Buffer'a kopyalanir
    4. Onceki pozisyon guncellenir
```

### TX Veri Gonderme Mekanizmasi

```
    1. Uygulama -> TX Ring Buffer'a yaz
    2. DMA TX bos mu kontrol et
    3. Ring Buffer'dan DMA TX tamponuna kopyala
    4. DMA TX transferini baslat
    5. DMA TC kesmesi -> Daha fazla veri var mi kontrol et
    6. Varsa 3. adima don, yoksa DMA TX'i durdur
```

---

## Test Proseduru

### Gerekli Araclar

1. STM32F407VG Discovery Board
2. USB-TTL donusturucu (veya ST-Link VCP)
3. Terminal programi (PuTTY, Tera Term, veya CoolTerm)
4. USB kablosu (micro-USB)

### Terminal Ayarlari

| Parametre | Deger |
|-----------|-------|
| Port | COMx (cihaz yoneticisinden kontrol edin) |
| Baud Rate | 115200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| New Line | CR+LF veya CR |

### Test Adimlari

#### Test 1: Temel Baglanti

1. Karti programlayin ve sifirlama yapin
2. Terminal'de asagidaki mesaji gormelisiniz:

```
========================================
  UART DMA Ring Buffer Demo
  STM32F407VG Discovery Board
  Baud: 115200, Clock: 168 MHz
========================================
System ready. Type 'AT' to test.
>
```

3. Mavi LED (PD15) yanip sonmelidir (sistem aktif gostergesi)

#### Test 2: AT Komut Testi

Terminal'e asagidaki komutlari yazip Enter'a basin:

| Komut | Beklenen Yanit |
|-------|---------------|
| `AT` | `OK` |
| `AT+LED=ON` | `LED ON` + PD12 yanar |
| `AT+LED=OFF` | `LED OFF` + PD12 soner |
| `AT+TEMP` | `TEMP=xx.x C` (ic sicaklik degeri) |
| `AT+INFO` | Sistem bilgileri (clock, uptime, buffer) |
| `AT+STATS` | Haberlesme istatistikleri |
| `HELLO` | `ECHO: HELLO` (bilinmeyen komutlar echo edilir) |

#### Test 3: Yuksek Hiz Testi

1. Terminal programindan buyuk bir metin dosyasi gonderin (1KB+)
2. Tum verinin eksiksiz echo edildigini dogrulayin
3. Buffer overflow olmamalidir

#### Test 4: Hata Yonetimi

1. Kablo baglantisini gecici olarak kesin -> Kirmizi LED (PD14) yanmali
2. Yanlis baud rate ile baglanti deneyin -> Framing error
3. Buffer'i doldurun -> Uygun hata mesaji

### Beklenen LED Davranislari

| LED | Renk | Anlam |
|-----|------|-------|
| PD12 | Yesil | Kullanici kontrollu (AT+LED komutu) |
| PD13 | Turuncu | Veri transferi gostergesi (kisa yanip sonme) |
| PD14 | Kirmizi | Hata durumu |
| PD15 | Mavi | Sistem heartbeat (500ms aralikla yanip sonme) |

---

## Olasi Sorunlar ve Cozumleri

### Sorun 1: UART'tan Hic Veri Alinmiyor

**Belirtiler**: Terminal'de hicbir cikti gorulmuyor.

**Kontrol Listesi**:
- [ ] TX/RX kablolari capraz bagli mi? (TX->RX, RX->TX)
- [ ] GND baglantisi var mi?
- [ ] Baud rate ayari dogru mu? (her iki tarafta 115200)
- [ ] Dogru COM portu secildi mi?
- [ ] USART2 clock'u etkinlestirildi mi? (`__HAL_RCC_USART2_CLK_ENABLE()`)
- [ ] GPIO pinleri dogru Alternate Function'da mi? (AF7)
- [ ] UART Enable (UE) biti set edildi mi?

### Sorun 2: Bozuk Karakterler Aliniyor

**Olasi Nedenler**:
1. **Baud rate uyumsuzlugu**: APB1 clock'u kontrol edin (42 MHz olmali)
2. **Clock konfigurasyonu hatasi**: HSE/PLL ayarlarini dogrulayin
3. **Parity/Stop bit uyumsuzlugu**: Her iki tarafta ayni olmali

### Sorun 3: DMA Transfer Tamamlanmiyor

**Kontrol Listesi**:
- [ ] DMA clock etkinlestirildi mi? (`__HAL_RCC_DMA1_CLK_ENABLE()`)
- [ ] Dogru stream ve kanal secildi mi?
- [ ] NDTR (veri sayisi) 0'dan buyuk mu?
- [ ] DMA stream etkinlestirildi mi? (EN biti)
- [ ] UART'ta DMA modu etkinlestirildi mi? (DMAT/DMAR bitleri)

### Sorun 4: Veri Kaybi / Overrun Hatasi

**Nedenler**:
- Ring buffer boyutu yetersiz
- Uygulama katmani veriyi yeterince hizli okumuyor
- DMA kesmesi cok gec isleniyor

**Cozumler**:
1. Ring buffer boyutunu artirin (ornek: 1024 -> 2048)
2. Kesme onceliklerini ayarlayin
3. UART hata bayraklarini temizleyin

### Sorun 5: Hard Fault / Bellek Erisim Hatasi

**Olasi Nedenler**:
1. DMA buffer'i SRAM disinda (CCM RAM'de DMA calismaz!)
2. Hatali pointer kullanimi
3. Stack overflow

**Cozum**: DMA buffer'larini global olarak tanimlayin, CCM RAM yerine normal SRAM kullaniladigindan emin olun.

---

## Mimari Notlar

### Bellek Haritalamasi

```
    Flash (1MB)     : 0x0800_0000 - 0x080F_FFFF  (Kod)
    SRAM1 (112KB)   : 0x2000_0000 - 0x2001_BFFF  (Veri + DMA Buffer)
    SRAM2 (16KB)    : 0x2001_C000 - 0x2001_FFFF  (Veri)
    CCM RAM (64KB)  : 0x1000_0000 - 0x1000_FFFF  (Sadece CPU erisimi, DMA ERISEMEZ!)
```

**ONEMLI**: DMA buffer'lari **SRAM1** veya **SRAM2**'de olmalidir. CCM RAM'e DMA erisimi yoktur!

### Interrupt Oncelik Sirasi

```
    Oncelik 0 (En yuksek) : SysTick (HAL_Delay icin)
    Oncelik 1              : DMA1_Stream5 (USART2_RX) + USART2 Global
    Oncelik 2              : DMA1_Stream6 (USART2_TX)
    Oncelik 15 (En dusuk) : Kullanilmiyor
```

---

## Kaynaklar ve Referanslar

### Resmi Dokumantasyon

1. **RM0090** - STM32F405/407/415/417 Reference Manual
   - Chapter 30: USART (Universal synchronous asynchronous receiver transmitter)
   - Chapter 10: DMA Controller
   - [ST Website](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

2. **DS8626** - STM32F407VG Datasheet
   - Pin assignments, electrical characteristics
   - [ST Website](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)

3. **UM1472** - STM32F4DISCOVERY User Manual
   - Board layout, schematic, jumper settings
   - [ST Website](https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf)

4. **AN3116** - STM32 USART Application Note
   - UART/USART kullanim kilavuzu

### HAL Kutuphanesi

5. **UM1725** - STM32Cube HAL and LL drivers for STM32F4
   - HAL fonksiyonlari referansi

### Teknik Makaleler

6. **Tilen Majerle** - STM32 UART DMA RX/TX
   - [GitHub](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx)
   - DMA + IDLE line detection en iyi uygulamalari

7. **Ring Buffer (Circular Buffer)** - Wikipedia
   - [Wikipedia](https://en.wikipedia.org/wiki/Circular_buffer)

### Araclar

8. **STM32CubeIDE** - Entegre gelistirme ortami
   - [ST Website](https://www.st.com/en/development-tools/stm32cubeide.html)

9. **STM32CubeMX** - Konfigurasyon araci
   - [ST Website](https://www.st.com/en/development-tools/stm32cubemx.html)

---

## Lisans

Bu proje egitim ve portfolyo amaciyla gelistirilmistir. MIT Lisansi ile lisanslanmistir.

## Yazar

STM32F407VG Gomulu Sistemler Portfolyo Projesi - Proje 01
