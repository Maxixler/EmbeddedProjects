# STM32F407VG ADC-DMA-FFT Projesi

## Proje Amaci

Bu proje, STM32F407VG mikrodenetleyicisi kullanarak analog sinyallerin orneklenmesi (ADC),
verimli veri transferi (DMA) ve frekans analizi (FFT) islemlerini gerceklestirmektedir.
Sinyal isleme zincirinin tamamini gomulu sistem uzerinde calistirarak gercek zamanli
spektrum analizi yapmak amaclanmaktadir.

---

## 1. ADC (Analog-Digital Converter) Teorisi

### 1.1 SAR (Successive Approximation Register) Mimarisi

STM32F407VG'nin ADC birimi SAR (Ardisik Yaklasim Kaydedici) mimarisini kullanir.
Bu yontem, analog sinyali dijital degere donusturmek icin ikili arama (binary search)
algoritmasina benzer bir yaklasim kullanir.

**SAR ADC Calisma Prensibi:**

```
Analog Giris --> Sample & Hold --> Karsilastirici --> SAR Mantigi --> Dijital Cikis
                                        ^
                                        |
                                   DAC (Dahili)
```

1. **Ornekleme ve Tutma (Sample & Hold):** Analog sinyal belirli bir anda orneklenir
   ve bir kapasitorde tutulur.
2. **Karsilastirma:** Tutulan deger, dahili DAC cikisi ile karsilastirilir.
3. **Ardisik Yaklasim:** MSB'den LSB'ye dogru her bit icin:
   - Bit '1' yapilir, DAC cikisi uretilir
   - Karsilastirici sonucuna gore bit '1' kalir veya '0' yapilir
   - Bir sonraki bite gecilir
4. **Sonuc:** N-bit cozunurluk icin N saat cevriminde donusum tamamlanir.

**12-bit SAR ornegi (Vref = 3.3V, Vin = 2.0V):**

```
Adim 1: MSB = 1 -> DAC = 2048 -> 1.65V < 2.0V -> Bit = 1
Adim 2: Bit10 = 1 -> DAC = 3072 -> 2.475V > 2.0V -> Bit = 0
Adim 3: Bit9 = 1 -> DAC = 2560 -> 2.0625V > 2.0V -> Bit = 0
...
Sonuc: 0b100110011010 = 2458 -> (2458/4095) * 3.3V = 1.979V
```

### 1.2 Kuantalama (Quantization)

Analog sinyalin dijital degere donusturulmesinde kuantalama hatasi olusur.

**Kuantalama Parametreleri:**

| Parametre | Formul | 12-bit (Vref=3.3V) |
|-----------|--------|---------------------|
| LSB | Vref / (2^N) | 0.806 mV |
| Kuantalama Hatasi | +/- 0.5 LSB | +/- 0.403 mV |
| Dinamik Aralik | 20*log10(2^N) | 72.25 dB |
| Olculebilir Min. Sinyal | 1 LSB | 0.806 mV |

**Dijital Deger Hesaplama:**

```
Dijital_Deger = (Vin / Vref) * (2^N - 1)

Ornek: Vin = 1.65V, Vref = 3.3V, N = 12
Dijital_Deger = (1.65 / 3.3) * 4095 = 2047.5 -> 2048
```

### 1.3 SNR (Signal-to-Noise Ratio) ve ENOB

**Ideal SNR Formulü:**

```
SNR_ideal = 6.02 * N + 1.76 dB

12-bit icin: SNR = 6.02 * 12 + 1.76 = 74 dB
```

**ENOB (Effective Number of Bits):**

Gercek ADC'nin performansini olcen parametredir. Gurultu, dogrusal olmama
ve diger hata kaynaklari nedeniyle ENOB her zaman nominal cozunurlukten dusuktur.

```
ENOB = (SINAD - 1.76) / 6.02

STM32F407 icin tipik ENOB: ~10.5 bit (12-bit nominal)
```

**Diger Onemli Parametreler:**
- **THD (Total Harmonic Distortion):** Harmonik bozulmanin toplam olcusu
- **SFDR (Spurious Free Dynamic Range):** En buyuk sinyal ile en buyuk sahte sinyal arasi fark
- **INL (Integral Non-Linearity):** Transfer fonksiyonunun ideal dogrudan sapmas
- **DNL (Differential Non-Linearity):** Ardisik kodlar arasi adim buyuklugu farki

---

## 2. STM32F407 ADC Ozellikleri

### 2.1 Genel Ozellikler

STM32F407VG, 3 adet bagimsiz ADC birimine sahiptir:

| Ozellik | Deger |
|---------|-------|
| ADC Sayisi | 3 (ADC1, ADC2, ADC3) |
| Cozunurluk | 6, 8, 10, 12 bit (ayarlanabilir) |
| Maksimum Ornekleme Hizi | 2.4 MSPS (tek ADC) |
| Triple Interleaved Mod | 7.2 MSPS (3 ADC birlikte) |
| Kanal Sayisi | 16 harici + 3 dahili |
| Giris Araligi | 0V - VDDA (tipik 3.3V) |
| Donusum Suresi | 3 + N saat cevrimi (N = ornekleme suresi) |
| ADC Saat Frekansi | Maksimum 36 MHz (APB2/2, /4, /6, /8) |
| Tetikleme Kaynaklari | Timer, EXTI, yazilim |
| DMA Destegi | Evet (her ADC icin) |

### 2.2 Triple Interleaved Mod

Uc ADC biriminin ayni kanali sirayla orneklemesiyle elde edilen yuksek hizli moddur:

```
Zaman -->
ADC1: |--S1--|      |--S4--|      |--S7--|
ADC2:    |--S2--|      |--S5--|      |--S8--|
ADC3:       |--S3--|      |--S6--|      |--S9--|

Etkin ornekleme hizi = 3 x tek ADC hizi = 7.2 MSPS
```

### 2.3 Ornekleme Suresi Secimi

STM32F407 ADC icin ornekleme suresi 8 farkli degerden secilebilir:

| Secim | Saat Cevrimi | 30 MHz'de Sure |
|-------|-------------|----------------|
| 000 | 3 cycles | 0.1 us |
| 001 | 15 cycles | 0.5 us |
| 010 | 28 cycles | 0.93 us |
| 011 | 56 cycles | 1.87 us |
| 100 | 84 cycles | 2.8 us |
| 101 | 112 cycles | 3.73 us |
| 110 | 144 cycles | 4.8 us |
| 111 | 480 cycles | 16 us |

**Toplam Donusum Suresi:**

```
T_conv = (Ornekleme Suresi + 12) * T_ADC_CLK

Ornek: 15 cycle ornekleme, 30 MHz ADC clock
T_conv = (15 + 12) / 30MHz = 0.9 us -> ~1.11 MSPS
```

---

## 3. ADC Register Yapisi

### 3.1 ADC_CR1 (Control Register 1)

```
Bit 25:24  RES[1:0]     - Cozunurluk secimi
                           00: 12-bit (15 ADCCLK cevrimi)
                           01: 10-bit (13 ADCCLK cevrimi)
                           10: 8-bit  (11 ADCCLK cevrimi)
                           11: 6-bit  (9 ADCCLK cevrimi)
Bit 26     OVRIE         - Overrun kesme izni
Bit 8      SCAN          - Scan modu (coklu kanal icin)
Bit 5      EOCIE         - End of conversion kesme izni
Bit 11:8   DISCNUM[3:0]  - Discontinuous kanal sayisi
```

### 3.2 ADC_CR2 (Control Register 2)

```
Bit 0      ADON         - ADC acma/kapama
Bit 1      CONT         - Surekli donusum modu
Bit 8      DMA          - DMA istegi izni
Bit 9      DDS          - DMA Disable Selection
Bit 10     EOCS         - End of conversion secimi
Bit 11     ALIGN        - Veri hizalama (0: sag, 1: sol)
Bit 27:24  EXTSEL[3:0]  - Harici tetik secimi
                          0110: Timer 2 TRGO
                          1001: Timer 3 TRGO
Bit 29:28  EXTEN[1:0]   - Harici tetik kenar secimi
                          00: Devre disi
                          01: Yukselen kenar
                          10: Dusen kenar
                          11: Her iki kenar
Bit 30     SWSTART      - Yazilimla donusum baslatma
```

### 3.3 ADC_SMPR1/SMPR2 (Sample Time Registers)

```
ADC_SMPR1: Kanal 10-18 icin ornekleme suresi
ADC_SMPR2: Kanal 0-9 icin ornekleme suresi

Her kanal icin 3 bit:
  000: 3 cycles    001: 15 cycles
  010: 28 cycles   011: 56 cycles
  100: 84 cycles   101: 112 cycles
  110: 144 cycles  111: 480 cycles
```

### 3.4 ADC_SQR1/SQR2/SQR3 (Sequence Registers)

```
ADC_SQR1:
  Bit 23:20  L[3:0]  - Dizideki toplam donusum sayisi (0-15 -> 1-16 donusum)
  Bit 19:15  SQ16    - 16. sira kanal numarasi
  ...

ADC_SQR3:
  Bit 4:0    SQ1     - 1. sira kanal numarasi
  Bit 9:5    SQ2     - 2. sira kanal numarasi
  ...
```

### 3.5 ADC_DR (Data Register)

```
Bit 15:0   DATA[15:0]  - Donusum sonucu (sag hizali)
                         12-bit: 0x000 - 0xFFF (0 - 4095)
```

### 3.6 ADC_CCR (Common Control Register)

```
Bit 17:16  ADCPRE[1:0]  - ADC onbolme secimi
                          00: PCLK2 / 2
                          01: PCLK2 / 4
                          10: PCLK2 / 6
                          11: PCLK2 / 8
Bit 22     TSVREFE       - Sicaklik sensoru ve Vrefint izni
Bit 23     VBATE         - VBAT kanal izni
Bit 4:0    MULTI[4:0]    - Coklu ADC modu secimi
```

---

## 4. DMA ile ADC Kullanimi

### 4.1 DMA Temelleri

DMA (Direct Memory Access), CPU'yu mesgul etmeden cevresel birimlerden
bellek transferi yapar. ADC ile kullanildiginda, donusum sonuclari
otomatik olarak bellek dizisine aktarilir.

```
ADC Data Register --> DMA Stream --> Bellek Dizisi (Buffer)
     (Kaynak)         (Transfer)       (Hedef)

CPU bu sirada baska islemler yapabilir!
```

### 4.2 DMA2 Stream Atamasi (ADC icin)

| ADC | DMA | Stream | Channel |
|-----|-----|--------|---------|
| ADC1 | DMA2 | Stream 0 | Channel 0 |
| ADC1 | DMA2 | Stream 4 | Channel 0 |
| ADC2 | DMA2 | Stream 2 | Channel 1 |
| ADC2 | DMA2 | Stream 3 | Channel 1 |
| ADC3 | DMA2 | Stream 0 | Channel 2 |
| ADC3 | DMA2 | Stream 1 | Channel 2 |

### 4.3 Circular (Dairesel) Mod

Circular modda DMA, buffer sonuna ulastiginda otomatik olarak
buffer basindan devam eder. Bu sayede surekli veri toplama mumkun olur.

```
Buffer:  [0] [1] [2] [3] [4] [5] [6] [7]
          ^                               ^
          |                               |
        Basi                            Sonu
          <-------- DMA yazma yonu -------->
          Sona ulasinca basa doner (circular)
```

### 4.4 Double Buffering (Cift Tamponlama)

Half-complete ve transfer-complete kesmelerini kullanarak
cift tamponlama gerceklestirilir:

```
Buffer: [--- Yarim 1 ---][--- Yarim 2 ---]
         Half Complete     Transfer Complete

Durum 1: DMA Yarim-1'e yazar  | CPU Yarim-2'yi isler
Durum 2: DMA Yarim-2'ye yazar | CPU Yarim-1'i isler

Bu sayede veri kaybi olmadan surekli islem yapilir.
```

**Kesme Mekanizmasi:**
- **HTIF (Half Transfer Interrupt):** Buffer'in ilk yarisi doldu
- **TCIF (Transfer Complete Interrupt):** Buffer'in tamami doldu (basa dondü)

```
Zaman -->
DMA:  [Yarim1 doldur][Yarim2 doldur][Yarim1 doldur][Yarim2 doldur]
CPU:                  [Yarim1 isle  ][Yarim2 isle  ][Yarim1 isle  ]
Kesme:       HT             TC            HT             TC
```

---

## 5. Ornekleme Teorisi

### 5.1 Nyquist-Shannon Ornekleme Teoremi

Bir analog sinyalin kayipsiz olarak yeniden olusturulabilmesi icin
ornekleme frekansi, sinyalin en yuksek frekans bileseninin en az
2 katinda olmalidir.

```
fs >= 2 * fmax

fs:   Ornekleme frekansi (Hz)
fmax: Sinyaldeki en yuksek frekans bileseni (Hz)
```

**Nyquist Frekansi:** fn = fs / 2

Orneklenebilecek en yuksek frekans, Nyquist frekansina esittir.

**Ornekler:**

| Uygulama | fmax | Minimum fs | Tipik fs |
|----------|------|-----------|----------|
| Ses (konusma) | 4 kHz | 8 kHz | 8 kHz |
| Muzik (CD) | 20 kHz | 40 kHz | 44.1 kHz |
| Titresim | 5 kHz | 10 kHz | 25.6 kHz |
| Guc kalitesi | 3 kHz | 6 kHz | 12.8 kHz |

### 5.2 Aliasing (Ortusmeli Bozulma)

Nyquist kosulu saglanmazsa, yuksek freansli sinyaller dusuk freansli
sinyaller olarak gorunur. Bu geri donusumsuz bir bozulmadır.

```
Aliased Frekans = |f_sinyal - k * fs|   (k: en yakin tamsayi)

Ornek: fs = 10 kHz, f_sinyal = 12 kHz
Alias = |12000 - 1 * 10000| = 2 kHz (YANLIS frekans gorunur!)
```

```
Gercek sinyal: 12 kHz ~~~~~~
                              \
Ornekleme: 10 kHz              > Aliasing!
                              /
Gorunen sinyal: 2 kHz  ------
```

### 5.3 Anti-Aliasing Filtresi

ADC oncesinde yerlestirilmesi gereken analog alcak geciren filtredir.
Nyquist frekansinin uzerindeki tum bilesenleri bastirmalidir.

```
                    Anti-Aliasing        Ornekleme
Analog Sinyal --> [ Alcak Geciren ] --> [ ADC ] --> Dijital Veri
                    Filtre (AAF)
                    fc < fs/2
```

**Basit RC Alcak Geciren Filtre:**

```
fc = 1 / (2 * pi * R * C)

Ornek: fs = 10 kHz -> fc = 5 kHz (Nyquist)
       Pratikte fc = 4 kHz (guvenlik marji ile)
       R = 1 kOhm, C = 39 nF -> fc = 4.08 kHz
```

**Filtre Duzen Secimi:**
- 1. duzen: -20 dB/dekad (basit ama yetersiz)
- 2. duzen: -40 dB/dekad (makul)
- 4. duzen: -80 dB/dekad (iyi bastirma)
- Oversampling + dijital filtre: En iyi sonuc

---

## 6. Pencere Fonksiyonlari ve Spektral Sizinti

### 6.1 Spektral Sizinti (Spectral Leakage)

FFT, sinyalin periyodik oldugunu varsayar. Eger ornekleme penceresi
sinyalin tam periyodunu icermezse, FFT sonucunda spektral sizinti olusur.

```
Tam periyod:     |~~~~~|~~~~~|~~~~~|  -> Tek frekans cizgisi
Eksik periyod:   |~~~~~|~~~~~|~~~  |  -> Yayilmis frekans (sizinti)
```

Pencere fonksiyonlari, sinyalin baslangic ve bitis noktalarinda
genlik degerini sifira yaklastirarak bu sorunu azaltir.

### 6.2 Pencere Fonksiyonlari

#### Rectangular (Dikdortgen) Pencere

```
w[n] = 1,  0 <= n <= N-1

Ozellikler:
- Ana lob genisligi: En dar (2*fs/N)
- Yan lob seviyesi: -13 dB (en kotu)
- Kullanim: Periyodik sinyaller (tam periyod yakalandiginda)
```

#### Hanning (Hann) Penceresi

```
w[n] = 0.5 * (1 - cos(2*pi*n / (N-1)))

Ozellikler:
- Ana lob genisligi: 4*fs/N
- Yan lob seviyesi: -31 dB
- Yan lob dusus hizi: -18 dB/oktav
- Kullanim: Genel amacli, titresim analizi
```

#### Hamming Penceresi

```
w[n] = 0.54 - 0.46 * cos(2*pi*n / (N-1))

Ozellikler:
- Ana lob genisligi: 4*fs/N
- Yan lob seviyesi: -42 dB
- Yan lob dusus hizi: -6 dB/oktav
- Kullanim: Frekans olcumu, daha iyi yan lob bastirma
```

#### Blackman Penceresi

```
w[n] = 0.42 - 0.5*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1))

Ozellikler:
- Ana lob genisligi: 6*fs/N
- Yan lob seviyesi: -58 dB
- Kullanim: Harmonik analiz, dusuk seviyeli bilesenlerin tespiti
```

### 6.3 Pencere Karsilastirmasi

```
                 Ana Lob     Yan Lob     Frekans      Genlik
Pencere          Genisligi   Seviyesi    Dogrulugu    Dogrulugu
-----------------------------------------------------------------
Rectangular      En dar      -13 dB      En iyi       Kotu
Hanning          Orta        -31 dB      Iyi          Iyi
Hamming          Orta        -42 dB      Iyi          Iyi
Blackman         Genis       -58 dB      Orta         En iyi
```

---

## 7. FFT Algoritmasi

### 7.1 DFT (Discrete Fourier Transform)

Ayrik Fourier Donusumu, zaman domenindeki N noktayi frekans domenine donusturur.

```
         N-1
X[k] =  SUM  x[n] * e^(-j*2*pi*k*n/N)    k = 0, 1, ..., N-1
         n=0

Burada:
  x[n] : Zaman domenindeki n. ornek
  X[k] : Frekans domenindeki k. bilesen
  N    : Toplam ornek sayisi
  j    : Sanal birim (sqrt(-1))
  e^(-j*theta) = cos(theta) - j*sin(theta)  (Euler formulü)
```

**DFT Karmasikligi:** O(N^2) - Her frekans bileseni icin N carpma ve toplama.

N = 1024 icin: 1024 * 1024 = ~1 milyon islem -> YAVAS!

### 7.2 Cooley-Tukey FFT Algoritmasi

FFT, DFT'nin verimli hesaplanmasi icin gelistirilen algoritmalar ailesidir.
En yaygin kullanilani 1965 yilinda Cooley ve Tukey tarafindan yayinlanan
Radix-2 DIT (Decimation in Time) algoritmasidir.

**Temel Fikir:** N noktali DFT'yi, N/2 noktali iki DFT'ye bolerek
hesaplama karmasikligini azaltmak.

```
N noktali DFT = Cift indisli N/2 DFT + Tek indisli N/2 DFT

X[k] = E[k] + W_N^k * O[k]           k = 0, ..., N/2-1
X[k+N/2] = E[k] - W_N^k * O[k]       k = 0, ..., N/2-1

Burada:
  E[k] = Cift indisli orneklerin DFT'si
  O[k] = Tek indisli orneklerin DFT'si
  W_N^k = e^(-j*2*pi*k/N) = "Twiddle factor"
```

### 7.3 Radix-2 Kelebek (Butterfly) Yapisi

```
    a -----> [+] -----> A = a + W*b
              X
    b --> [W] [+] ----> B = a - W*b

    W = e^(-j*2*pi*k/N)  (Twiddle factor)
```

**FFT Asamalari (N=8 ornegi):**

```
Giris (bit-reversed)     Asama 1      Asama 2      Asama 3      Cikis
    x[0] ------>          *            *            *         --> X[0]
    x[4] ------>          *            *            *         --> X[1]
    x[2] ------>          *            *            *         --> X[2]
    x[6] ------>          *            *            *         --> X[3]
    x[1] ------>          *            *            *         --> X[4]
    x[5] ------>          *            *            *         --> X[5]
    x[3] ------>          *            *            *         --> X[6]
    x[7] ------>          *            *            *         --> X[7]

Toplam asama sayisi: log2(N) = log2(8) = 3
Her asamada: N/2 kelebek islemi
```

### 7.4 Karmasiklik Karsilastirmasi

```
Algoritma     Islem Sayisi        N=1024           N=4096
-----------------------------------------------------------------
DFT           O(N^2)              1,048,576        16,777,216
FFT           O(N*log2(N))        10,240           49,152
Hizlanma                          ~102x            ~341x
```

---

## 8. ARM CMSIS-DSP Kutuphanesi

### 8.1 Genel Bakis

CMSIS-DSP, ARM Cortex-M islemciler icin optimize edilmis dijital sinyal isleme
kutuphanesidir. FPU (Floating Point Unit) ve SIMD komutlarini kullanarak
yuksek performans saglar.

**Kullanilan Fonksiyonlar:**

| Fonksiyon | Aciklama |
|-----------|----------|
| `arm_rfft_fast_f32()` | Gercek degerli FFT (float32) |
| `arm_cmplx_mag_f32()` | Karmasik buyukluk hesaplama |
| `arm_max_f32()` | Dizideki maksimum deger bulma |
| `arm_mean_f32()` | Ortalama hesaplama |
| `arm_rms_f32()` | RMS deger hesaplama |

### 8.2 arm_rfft_fast_f32 Kullanimi

```c
arm_rfft_fast_instance_f32 fft_instance;

/* FFT yapisini baslat (N = 1024) */
arm_rfft_fast_init_f32(&fft_instance, 1024);

/* FFT hesapla */
/* Giris: fft_input[1024] (gercek degerler) */
/* Cikis: fft_output[1024] (karmasik degerler: R0,I0,R1,I1,...) */
arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

/* Buyukluk hesapla */
/* Cikis: magnitude[512] */
arm_cmplx_mag_f32(fft_output, magnitude, 512);
```

**Onemli Notlar:**
- Giris dizisi FFT sirasinda degistirilir (in-place)
- Cikis dizisi karmasik formattadir: [Re0, Im0, Re1, Im1, ...]
- N noktali gercek FFT -> N/2 + 1 frekans bileseni uretir
- Desteklenen N degerleri: 32, 64, 128, 256, 512, 1024, 2048, 4096

---

## 9. Frekans Cozunurluğu ve Spektrum Analizi

### 9.1 Frekans Cozunurlugu

```
df = fs / N

fs: Ornekleme frekansi (Hz)
N:  FFT noktasi sayisi
df: Frekans cozunurlugu (Hz)

Ornek: fs = 10 kHz, N = 1024
df = 10000 / 1024 = 9.77 Hz
```

**Her FFT kutusunun (bin) frekans degeri:**

```
f[k] = k * df = k * fs / N    k = 0, 1, ..., N/2

k=0:   DC bileseni (0 Hz)
k=1:   df Hz (temel frekans cozunurlugu)
k=N/2: fs/2 Hz (Nyquist frekansi)
```

### 9.2 Buyukluk (Magnitude) ve Faz (Phase)

FFT cikisi karmasik sayilardir: X[k] = Re[k] + j*Im[k]

```
Buyukluk: |X[k]| = sqrt(Re[k]^2 + Im[k]^2)
Faz:      phi[k] = atan2(Im[k], Re[k])
```

### 9.3 dB Olcegi ve Guc Spektral Yogunlugu

**Genlik Spektrumu (dB):**

```
X_dB[k] = 20 * log10(|X[k]| / X_ref)

X_ref: Referans deger (genellikle maksimum deger veya 1.0)
```

**Guc Spektral Yogunlugu (PSD):**

```
PSD[k] = |X[k]|^2 / (N * fs)    [V^2/Hz]

PSD_dB[k] = 10 * log10(PSD[k])  [dB/Hz]
```

**Parseval Teoremi:**

```
Zaman domenindeki toplam enerji = Frekans domenindeki toplam enerji

N-1                    N-1
SUM |x[n]|^2  =  1/N * SUM |X[k]|^2
n=0                    k=0
```

---

## 10. Pratik Uygulamalar

### 10.1 Ses Analizi

- Frekans araligi: 20 Hz - 20 kHz
- Ornekleme: 44.1 kHz veya 48 kHz
- FFT boyutu: 1024 veya 2048
- Pencere: Hanning
- Kullanim: Muzik enstrumani tanima, ses komutu, gurultu analizi

### 10.2 Titresim Analizi

- Frekans araligi: 1 Hz - 10 kHz
- Ornekleme: 25.6 kHz (tipik)
- FFT boyutu: 2048 veya 4096
- Pencere: Hanning veya Flat-top
- Kullanim: Makine saglik izleme, rulman ariza tespiti, dengesizlik

### 10.3 Guc Kalitesi Analizi

- Temel frekans: 50 Hz (Turkiye) veya 60 Hz
- Ornekleme: 6.4 kHz veya 12.8 kHz
- FFT boyutu: 1024
- Pencere: Rectangular (senkron ornekleme ile)
- Olcumler: THD, harmonik bilesenleri, guc faktoru
- Standartlar: IEC 61000-4-7

### 10.4 Bu Projede

- Ornekleme: 1 kHz - 100 kHz (ayarlanabilir)
- FFT boyutu: 256, 512, 1024, 2048, 4096 (ayarlanabilir)
- Pencere: Rectangular, Hanning, Hamming, Blackman
- Cikis: Spektrum, tepe degerler, THD, istatistikler

---

## 11. Donanim Baglantilari

### 11.1 Pin Baglantilari

```
STM32F407VG Discovery Board
============================

Analog Giris:
  PA0 (ADC1_CH0) <--- Sinyal kaynagi (0-3.3V)
                      |
                      +--- [Anti-aliasing RC filtresi]
                      |       R = 1kOhm, C = 100nF
                      |       fc = 1.59 kHz
                      |
                      +--- GND (sinyal referansi)

UART2 (Debug/Komut):
  PA2 (USART2_TX) ---> USB-UART donusturucu RX
  PA3 (USART2_RX) <--- USB-UART donusturucu TX
  GND              --- USB-UART donusturucu GND

LED Gostergeler:
  PD12 (Yesil LED)  - Sistem hazir / bos
  PD13 (Turuncu LED) - ADC ornekleme aktif
  PD14 (Kirmizi LED) - FFT isleme aktif
  PD15 (Mavi LED)    - Veri gonderme aktif

Test Sinyali (opsiyonel):
  DAC1_OUT (PA4) ---> PA0 (test sinyali uretimi)
```

### 11.2 Anti-Aliasing Filtre Devresi

```
Sinyal  ---[1kOhm]---+--- PA0 (ADC giris)
Kaynagi               |
                    [100nF]
                       |
                      GND

fc = 1 / (2 * pi * 1000 * 100e-9) = 1.59 kHz
```

### 11.3 Test Sinyali Uretimi

Harici sinyal jeneratoru veya dahili DAC kullanilabilir:

```
Fonksiyon Jeneratoru Ayarlari:
- Dalga formu: Sinus
- Frekans: 100 Hz - 5 kHz
- Genlik: 0 - 3.3V (DC offset ile)
- Offset: 1.65V (orta nokta)
```

---

## 12. STM32CubeMX Yapilandirmasi

### 12.1 Saat Yapilandirmasi

```
HSE: 8 MHz (Discovery board kristali)
PLL: HSE -> /8 -> *336 -> /2 = 168 MHz (SYSCLK)
AHB:  168 MHz
APB1: 42 MHz  (Timer2, USART2)
APB2: 84 MHz  (ADC1)
```

### 12.2 ADC1 Yapilandirmasi

```
Mode: Independent
Clock Prescaler: PCLK2 / 4 = 21 MHz
Resolution: 12-bit
Scan Conversion Mode: Disabled (tek kanal)
Continuous Conversion: Disabled (timer tetiklemeli)
DMA Continuous Requests: Enabled
External Trigger: Timer 2 TRGO Event
External Trigger Edge: Rising Edge
Channel: IN0 (PA0)
Sampling Time: 15 cycles
Rank: 1
```

### 12.3 DMA2 Yapilandirmasi

```
DMA Request: ADC1
Stream: DMA2 Stream 0
Channel: Channel 0
Direction: Peripheral to Memory
Mode: Circular
Data Width: Half Word (16-bit) -> Half Word
Increment: Peripheral Fixed, Memory Increment
FIFO: Disabled (Direct mode)
Priority: High
Interrupts: Half Transfer + Transfer Complete
```

### 12.4 Timer 2 Yapilandirmasi

```
Clock Source: Internal (APB1 * 2 = 84 MHz)
Prescaler: 0 (84 MHz)
Period: (84000000 / fs) - 1
  fs = 10 kHz -> Period = 8399
  fs = 44.1 kHz -> Period = 1904
  fs = 100 kHz -> Period = 839
Trigger Output: Update Event (TRGO)
```

### 12.5 USART2 Yapilandirmasi

```
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 bits
Stop Bits: 1
Parity: None
Hardware Flow Control: None
```

---

## 13. Test Proseduru

### 13.1 Temel Fonksiyon Testi

1. Projeyi derleyin ve karta yukleyin
2. UART terminali acin (115200 baud)
3. Yesil LED'in yandigini dogrulayin (sistem hazir)
4. PA0'a 1 kHz sinus sinyali uygulayin (1.65V offset, 1V p-p)
5. Terminal uzerinden `START` komutu gonderin
6. Spektrum ciktisinda 1 kHz'de tepe deger gorun

### 13.2 Frekans Dogrulugu Testi

```
Test Sinyalleri:
1. 100 Hz sinus  -> Olculen: ~100 Hz (+/- df)
2. 1 kHz sinus   -> Olculen: ~1000 Hz (+/- df)
3. 2.5 kHz sinus -> Olculen: ~2500 Hz (+/- df)

df = fs/N = 10000/1024 = 9.77 Hz (beklenen hata)
```

### 13.3 THD Testi

```
1. Saf sinus sinyali uygula -> THD < 1% olmali
2. Kare dalga uygula -> THD > 40% olmali (harmonikler)
3. Ucgen dalga uygula -> THD ~ 12% olmali
```

### 13.4 Pencere Fonksiyonu Testi

```
1. Rectangular pencere ile frekans arasinda sinyal uygula
   -> Spektral sizinti gorun
2. Hanning pencere ile ayni sinyal
   -> Azalmis sizinti gorun
3. Blackman pencere ile ayni sinyal
   -> En az sizinti gorun
```

### 13.5 UART Komutlari

```
Komut Formati: KOMUT PARAMETRE\r\n

Komutlar:
  FFT 256|512|1024|2048|4096  - FFT boyutunu ayarla
  FS 1000-100000              - Ornekleme frekansini ayarla (Hz)
  WIN RECT|HANN|HAMM|BLACK    - Pencere fonksiyonunu sec
  MODE SPEC|PEAK|RAW|STATS    - Cikis modunu sec
  START                       - Surekli modu baslat
  STOP                        - Surekli modu durdur
  SINGLE                      - Tek atisli olcum
  HELP                        - Komut listesini goster
```

### 13.6 Cikis Formatlari

**Spectrum Modu (MODE SPEC):**
```
--- SPECTRUM ---
BIN,FREQ_HZ,MAG_DB
0,0.00,-45.2
1,9.77,-42.1
2,19.53,-38.5
...
102,996.09,0.0
103,1005.86,-3.2
...
--- END ---
```

**Peak Modu (MODE PEAK):**
```
--- PEAKS ---
PEAK1: 1000.0 Hz, -0.5 dB
PEAK2: 2000.0 Hz, -25.3 dB
PEAK3: 3000.0 Hz, -35.1 dB
THD: 3.45%
SNR: 62.3 dB
--- END ---
```

**Stats Modu (MODE STATS):**
```
--- STATS ---
Fs: 10000 Hz
N: 1024
df: 9.77 Hz
Window: HANNING
Peak Freq: 1000.0 Hz
Peak Mag: -0.5 dB
RMS: 0.452 V
DC Offset: 1.648 V
THD: 3.45%
SNR: 62.3 dB
--- END ---
```

---

## 14. Performans Notlari

### 14.1 Islem Sureleri (168 MHz, FPU aktif)

| Islem | N=256 | N=1024 | N=4096 |
|-------|-------|--------|--------|
| Pencere uygulama | ~15 us | ~60 us | ~240 us |
| FFT hesaplama | ~45 us | ~250 us | ~1.2 ms |
| Buyukluk hesaplama | ~10 us | ~40 us | ~160 us |
| Tepe bulma | ~5 us | ~20 us | ~80 us |
| **Toplam** | **~75 us** | **~370 us** | **~1.7 ms** |

### 14.2 Bellek Kullanimi

| Tampon | N=1024 | Aciklama |
|--------|--------|----------|
| ADC DMA buffer | 4096 B | 2*N * 2 byte (uint16) |
| FFT giris | 4096 B | N * 4 byte (float32) |
| FFT cikis | 4096 B | N * 4 byte (float32) |
| Buyukluk | 2048 B | N/2 * 4 byte (float32) |
| Pencere katsayilari | 4096 B | N * 4 byte (float32) |
| **Toplam** | **~18 KB** | (128 KB SRAM mevcut) |

### 14.3 Maksimum Gercek Zamanli Ornekleme

```
Islem suresi < Veri toplama suresi olmali

N=1024, islem ~370 us
Veri toplama suresi = N / fs = 1024 / fs

fs_max (gercek zamanli) = N / T_islem
  = 1024 / 370e-6 = ~2.77 MHz (teorik)

Pratikte fs = 100 kHz ile rahat calisilir.
```

---

## 15. Sorun Giderme

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| ADC deger 0 | Pin baglantisi yok | PA0 baglantisini kontrol et |
| ADC deger 4095 | Asiri gerilim | 0-3.3V araligini kontrol et |
| Yanlis frekans | Aliasing | fs'yi arttir veya AAF ekle |
| Yayilmis tepe | Spektral sizinti | Pencere fonksiyonu degistir |
| Gurultulu spektrum | Topraklama | GND baglantisini iyilestir |
| DMA hatasi | Yanlis yapilandirma | Stream/Channel kontrol et |
| Dusuk SNR | Gurultulu kaynak | Korumali kablo kullan |

---

## Kaynaklar

1. STM32F407 Reference Manual (RM0090)
2. STM32F4 HAL Driver Documentation
3. ARM CMSIS-DSP Library Documentation
4. Oppenheim & Willsky - Signals and Systems
5. Smith - The Scientist and Engineer's Guide to DSP
6. STM32 ADC Application Notes (AN2834, AN3116)

---

*Bu proje, STM32F407VG Discovery board uzerinde gelistirilmis ve test edilmistir.*
