# STM32F407VG Proje 2: PID Motor Hiz Kontrolu

## Proje Ozeti

Bu proje, STM32F407VG mikrodenetleyici kullanarak bir DC motorun hizinin PID (Proportional-Integral-Derivative) kontrol algoritmasi ile hassas bir sekilde kontrol edilmesini amaclamaktadir. Encoder geri beslemesi ile gercek zamanli hiz olcumu yapilarak, kapali cevrim kontrol sistemi olusturulmaktadir.

### Ozellikler

- **PID Kontrol Algoritmasi**: Kp, Ki, Kd parametreleri ile ayarlanabilir kontrol
- **Quadrature Encoder Okuma**: TIM3 encoder modunda 4x cozunurluk
- **PWM Motor Surme**: TIM1 ile 20kHz PWM uretimi
- **UART Arayuzu**: Gercek zamanli parametre ayarlama ve veri izleme
- **Anti-Windup Korumasi**: Integral sarma onleme mekanizmasi
- **Guvenlik Ozellikleri**: Asiri akim korumasi, motor tikanma tespiti, acil durdurma
- **Veri Akisi Modu**: PC uzerinde grafik cizimi icin CSV formatta veri aktarimi

---

## Icerik Tablosu

1. [PID Kontrol Teorisi](#1-pid-kontrol-teorisi)
2. [DC Motor Modelleme](#2-dc-motor-modelleme)
3. [Encoder Calisma Prensibi](#3-encoder-calisma-prensibi)
4. [STM32F407VG Timer Modulleri](#4-stm32f407vg-timer-modulleri)
5. [H-Bridge Motor Surucu Devresi](#5-h-bridge-motor-surucu-devresi)
6. [Pin Baglanti Semasi](#6-pin-baglanti-semasi)
7. [CubeMX Konfigurasyon Adimlari](#7-cubemx-konfigurasyon-adimlari)
8. [PID Ayarlama (Tuning) Proseduru](#8-pid-ayarlama-tuning-proseduru)
9. [Anti-Windup Stratejileri](#9-anti-windup-stratejileri)
10. [Yazilim Mimarisi](#10-yazilim-mimarisi)
11. [UART Komut Arayuzu](#11-uart-komut-arayuzu)
12. [Test Proseduru](#12-test-proseduru)
13. [Sorun Giderme](#13-sorun-giderme)

---

## 1. PID Kontrol Teorisi

### 1.1 Giris

PID kontrolor, endustride en yaygin kullanilan geri beslemeli kontrol algoritmasidir. Bir sistemin
cikisini (olculen deger) istenilen degere (setpoint) ulastirmak icin kontrol sinyali uretir. PID
kontrolor uc temel bilesenden olusur: Oransal (P), Integral (I) ve Turev (D).

### 1.2 Temel PID Denklemi

Surekli zamanda PID kontrolorun matematiksel ifadesi:

```
u(t) = Kp * e(t) + Ki * integral(e(t)dt) + Kd * de(t)/dt
```

Burada:
- `u(t)` : Kontrol sinyali (motor PWM duty cycle)
- `e(t)` : Hata sinyali = Setpoint - Olculen Deger
- `Kp`   : Oransal kazanc
- `Ki`   : Integral kazanc
- `Kd`   : Turev kazanc

### 1.3 Ayrik Zamanda PID (Dijital Implementasyon)

Mikrodenetleyicide PID algoritmasi ayrik zamanda calistirilir. Ornekleme periyodu `dt` ile:

```
e[k] = setpoint - measurement[k]

P_term = Kp * e[k]
I_term = Ki * sum(e[i] * dt, i=0..k)
D_term = Kd * (e[k] - e[k-1]) / dt

u[k] = P_term + I_term + D_term
```

Bu projede ornekleme frekansi 100Hz (dt = 10ms) olarak secilmistir. Bu deger DC motor kontrol
uygulamalari icin yeterli bant genisligi saglamaktadir.

### 1.4 Oransal Terim (P)

Oransal terim, mevcut hataya oranli bir kontrol sinyali uretir:

```
P_out = Kp * e(t)
```

**Ozellikleri:**
- Hata buyudukce kontrol sinyali de buyur
- Hizli yanit saglar ancak tek basina kararli hal hatasini (steady-state error) sifira indiremez
- Cok buyuk Kp degerleri sistem kararsizligina (osilasyon) neden olur
- Cok kucuk Kp degerleri yavas yanit verir

**Motor kontrolunde P terimi:**
Ornegin hedef hiz 1000 RPM, mevcut hiz 800 RPM ise:
```
e = 1000 - 800 = 200 RPM
P_out = Kp * 200
```
Kp = 0.5 icin P_out = 100 birimlik kontrol sinyali uretilir.

### 1.5 Integral Terim (I)

Integral terim, gecmis hatalarin toplamina oranli kontrol sinyali uretir:

```
I_out = Ki * integral(e(t)dt)  (surekli zaman)
I_out = Ki * sum(e[k] * dt)    (ayrik zaman)
```

**Ozellikleri:**
- Kararli hal hatasini sifira indirir (offset eliminasyonu)
- Yavas etkili bir terimdir, zamanla birikir
- Cok buyuk Ki degerleri asiri sallanima (overshoot) ve kararsizliga neden olur
- **Integral Wind-up Problemi:** Motor fiziksel sinirlara ulastiginda integral terimi birikmeye
  devam eder, bu da motor tekrar kontrol araligina dondugunde asiri tepkiye neden olur

**Ayrik zamanda integral guncelleme:**
```c
integral_sum += error * dt;
I_out = Ki * integral_sum;
```

### 1.6 Turev Terim (D)

Turev terim, hatanin degisim hizina oranli kontrol sinyali uretir:

```
D_out = Kd * de(t)/dt          (surekli zaman)
D_out = Kd * (e[k] - e[k-1]) / dt  (ayrik zaman)
```

**Ozellikleri:**
- Gelecekteki hatayi tahmin eder (ongorucu etki)
- Asiri sallanmayi (overshoot) azaltir
- Sistemi daha kararli hale getirir
- Gurultuye (noise) cok duyarlidir - filtreleme gerektirir
- **Derivative Kick Problemi:** Setpoint ani olarak degistiginde turev terimi cok buyuk deger
  uretir. Bu problemi cozmek icin turev, hata yerine olcum uzerinden hesaplanir:

```c
// YANLIS: Derivative kick olusur
D_out = Kd * (error - prev_error) / dt;

// DOGRU: Derivative on measurement
D_out = -Kd * (measurement - prev_measurement) / dt;
```

### 1.7 Turev Filtresi

Turev terimi yuksek frekansi gurultuyu yukseltir. Bu nedenle bir alcak geciren filtre uygulanir:

```
Birinci dereceden alcak geciren filtre:

filtered_derivative = alpha * raw_derivative + (1 - alpha) * prev_filtered_derivative

alpha = dt / (tau + dt)

tau: Filtre zaman sabiti (tipik olarak 1/(2*pi*fc))
fc : Kesim frekansi (tipik olarak ornekleme frekansinin 1/10'u)
```

Bu projede fc = 10Hz secilmistir (ornekleme frekansi 100Hz oldugu icin).

```
tau = 1 / (2 * pi * 10) = 0.01592 s
alpha = 0.01 / (0.01592 + 0.01) = 0.386
```

---

## 2. DC Motor Modelleme

### 2.1 Elektriksel Model

DC motorun armutur devresi asagidaki denklemle modellenir:

```
V(t) = R * i(t) + L * di(t)/dt + Ke * omega(t)

Burada:
V(t)     : Uygulanan gerilim [V]
R        : Armutur direnci [Ohm]
L        : Armutur enduktansi [H]
i(t)     : Armutur akimi [A]
Ke       : Geri EMK sabiti [V/(rad/s)]
omega(t) : Acisal hiz [rad/s]
```

### 2.2 Mekanik Model

Motorun mekanik denklemi:

```
J * d(omega)/dt = Kt * i(t) - B * omega(t) - T_load

Burada:
J      : Rotor ataleti [kg*m^2]
Kt     : Tork sabiti [N*m/A]
B      : Viskoz surtunme katsayisi [N*m*s/rad]
T_load : Yuk torku [N*m]
```

**Not:** Ideal bir DC motorda Ke = Kt (SI birimlerinde) olur.

### 2.3 Transfer Fonksiyonu

Laplace donusumu uygulandiginda motor transfer fonksiyonu:

```
           Kt
G(s) = ----------------
       (Ls + R)(Js + B) + Ke*Kt

Basitlestirmek icin L << R kabulu ile (kucuk motorlar):

         Kt / (R*B + Ke*Kt)
G(s) = ----------------------
        (R*J)/(R*B + Ke*Kt) * s + 1

         Km
G(s) = --------
        Tm*s + 1

Burada:
Km = Kt / (R*B + Ke*Kt)   : Motor kazanci
Tm = R*J / (R*B + Ke*Kt)  : Mekanik zaman sabiti
```

### 2.4 Tipik DC Motor Parametreleri (12V, 300RPM)

| Parametre | Deger | Birim |
|-----------|-------|-------|
| Nominal Gerilim | 12 | V |
| Bos Calisma Hizi | 300 | RPM |
| Bos Calisma Akimi | 0.15 | A |
| Nominal Akim | 1.5 | A |
| Tikanma Akimi | 5.0 | A |
| Armutur Direnci | 2.4 | Ohm |
| Armutur Enduktansi | 1.2 | mH |
| Tork Sabiti | 0.038 | N*m/A |
| Geri EMK Sabiti | 0.038 | V/(rad/s) |
| Rotor Ataleti | 1.5e-5 | kg*m^2 |

---

## 3. Encoder Calisma Prensibi

### 3.1 Incremental Encoder (Artimsal Enkoder)

Incremental encoder, motorun mil donusunu dijital darbelere donusturen bir sensordur.
Iki fazli (A ve B) cikis sinyali uretir. Bu sinyaller birbirine gore 90 derece (quadrature)
faz farkina sahiptir.

### 3.2 Quadrature Encoding

```
Ileri Yonde Donus (CW):

Faz A:  |---|   |---|   |---|   |---|
        |   |   |   |   |   |   |   |
   _____|   |___|   |___|   |___|   |_____

Faz B:    |---|   |---|   |---|   |---|
          |   |   |   |   |   |   |   |
   _______|   |___|   |___|   |___|   |___

A fazinin yukselen kenarinda B fazi LOW ise -> ILERI
A fazinin yukselen kenarinda B fazi HIGH ise -> GERI
```

### 3.3 Cozunurluk Modlari

Encoder okuma uc farkli modda yapilabilir:

| Mod | Cozunurluk | Aciklama |
|-----|------------|----------|
| 1x  | CPR        | Yalnizca A fazinin yukselen kenari |
| 2x  | 2 x CPR    | A fazinin her iki kenari |
| 4x  | 4 x CPR    | A ve B fazinin her iki kenari |

**Bu projede 4x mod kullanilmaktadir.**

Ornek: 400 CPR encoder ile:
- 1x mod: 400 pulse/rev
- 2x mod: 800 pulse/rev
- 4x mod: 1600 pulse/rev

### 3.4 Hiz Hesaplama

Encoder kullanarak hiz iki yontemle hesaplanabilir:

**Yontem 1: Darbe Sayma (Frequency Method)**
```
RPM = (delta_count / (CPR * 4)) * (60 / dt)

Burada:
delta_count : dt suresi icinde okunan darbe sayisi degisimi
CPR         : Encoder'in devir basina darbe sayisi
4           : 4x quadrature carpani
dt          : Ornekleme periyodu [s]
```

**Yontem 2: Periyot Olcme (Period Method)**
```
RPM = 60 / (T * CPR * 4)

T: Ardisik iki darbe arasindaki sure [s]
```

Yuksek hizlarda Yontem 1 daha dogrudur. Dusuk hizlarda Yontem 2 tercih edilir.
Bu projede Yontem 1 kullanilmaktadir cunku kontrol dongusu sabit frekansta (100Hz) calismaktadir.

### 3.5 Pozisyon Takibi ve Tasma (Overflow) Yonetimi

STM32'nin timer sayaci 16-bit oldugu icin (0-65535 arasi), yuksek CPR encoder'larda
sayac tasabilir. Bu sorunu cozmek icin yazilimda 32-bit genisletilmis pozisyon sayaci
kullanilmaktadir:

```c
// Timer update interrupt'ta:
if (direction == FORWARD) {
    overflow_count++;
} else {
    overflow_count--;
}

// Toplam pozisyon:
int32_t position = (overflow_count * 65536) + timer_counter;
```

---

## 4. STM32F407VG Timer Modulleri

### 4.1 Timer Genel Bakis

STM32F407VG'de 14 adet timer bulunmaktadir:

| Timer | Tip | Bus | Frekans | Kanal | Ozellik |
|-------|-----|-----|---------|-------|---------|
| TIM1  | Advanced | APB2 | 168 MHz | 4 | PWM, Complementary, Dead-time |
| TIM2  | General | APB1 | 84 MHz | 4 | 32-bit, Encoder |
| TIM3  | General | APB1 | 84 MHz | 4 | 16-bit, Encoder |
| TIM4  | General | APB1 | 84 MHz | 4 | 16-bit, Encoder |
| TIM5  | General | APB1 | 84 MHz | 4 | 32-bit |
| TIM6  | Basic | APB1 | 84 MHz | 0 | DAC trigger, Interrupt |
| TIM7  | Basic | APB1 | 84 MHz | 0 | DAC trigger, Interrupt |
| TIM8  | Advanced | APB2 | 168 MHz | 4 | PWM, Complementary |
| TIM9-14 | Simple | Cesitli | Cesitli | 1-2 | Basit PWM/Capture |

### 4.2 PWM Uretimi (TIM1)

TIM1 advanced timer olarak PWM uretimi icin kullanilir. 20kHz PWM frekansi hedeflenmektedir.

**Hesaplama:**
```
PWM Frekansi = Timer_Clock / ((PSC + 1) * (ARR + 1))

Timer_Clock = 168 MHz (APB2)
Hedef Frekans = 20 kHz

PSC = 0 (prescaler kullanmiyoruz)
ARR = (168,000,000 / 20,000) - 1 = 8399

Duty Cycle = CCR / ARR * 100%

%50 duty icin: CCR = 4200
%100 duty icin: CCR = 8400
```

### 4.3 Timer Registerlari - PWM Konfigurasyonu

#### TIMx_CR1 (Control Register 1)
```
Bit 7  : ARPE  = 1  -> Auto-reload preload enable
Bit 6:5: CMS   = 00 -> Edge-aligned mode
Bit 4  : DIR   = 0  -> Upcounter
Bit 0  : CEN   = 1  -> Counter enable

Register Degeri: 0x0081
```

#### TIMx_CCMR1 (Capture/Compare Mode Register 1)
```
PWM Mode 1 icin (CH1):
Bit 6:4: OC1M  = 110 -> PWM mode 1 (active when CNT < CCR)
Bit 3  : OC1PE = 1   -> Output compare preload enable
Bit 1:0: CC1S  = 00  -> Output mode

Register Degeri: 0x0068
```

#### TIMx_CCER (Capture/Compare Enable Register)
```
Bit 3  : CC1NP = 0  -> OC1N active high
Bit 2  : CC1NE = 0  -> OC1N disabled (complementary output)
Bit 1  : CC1P  = 0  -> OC1 active high
Bit 0  : CC1E  = 1  -> OC1 output enabled

Register Degeri: 0x0001
```

#### TIMx_ARR (Auto-Reload Register)
```
ARR = 8399 -> 20kHz PWM frekansi
```

#### TIMx_CCR1 (Capture/Compare Register 1)
```
CCR1 = 0..8399 -> %0..%100 duty cycle
```

#### TIMx_BDTR (Break and Dead-Time Register) - Yalnizca TIM1/TIM8
```
Bit 15 : MOE  = 1    -> Main output enable
Bit 7:0: DTG  = 0x20 -> Dead-time = 32 * (1/168MHz) = ~190ns

Register Degeri: 0x8020
```

### 4.4 Encoder Modu (TIM3)

TIM3, encoder modunda calistirilir. A fazı TI1'e (PA6), B fazı TI2'ye (PA7) baglanir.

#### TIMx_SMCR (Slave Mode Control Register)
```
Bit 2:0: SMS = 011 -> Encoder mode 3 (her iki kanalin her iki kenari)

Bu mod 4x cozunurluk saglar.
```

#### TIMx_CCMR1 (Encoder modu icin)
```
Bit 9:8  : CC2S  = 01 -> IC2 mapped to TI2
Bit 7:4  : IC2F  = 0110 -> Input filter (gurultu filtreleme)
Bit 1:0  : CC1S  = 01 -> IC1 mapped to TI1
Bit 7:4  : IC1F  = 0110 -> Input filter

Register Degeri: 0x6161
```

#### TIMx_CCER (Encoder modu icin)
```
Bit 5  : CC2P = 0 -> TI2 non-inverted (yonu belirler)
Bit 1  : CC1P = 0 -> TI1 non-inverted

Register Degeri: 0x0000 (varsayilan polarite)
```

#### TIMx_ARR (Encoder modu icin)
```
ARR = 0xFFFF (65535) -> Maksimum sayac araligi

16-bit sayac tam aralikta kullanilir.
Tasma interrupt'i ile 32-bit pozisyon takibi yapilir.
```

### 4.5 Kontrol Dongusu Zamanlayicisi (TIM6)

TIM6 basic timer, PID kontrol dongusunu 100Hz frekansta tetiklemek icin kullanilir.

```
Timer_Clock = 84 MHz (APB1, timer carpani ile 2x = 84MHz)
Hedef Frekans = 100 Hz

PSC = 839   -> 84MHz / 840 = 100kHz
ARR = 999   -> 100kHz / 1000 = 100Hz

Kesme periyodu = 10ms
```

---

## 5. H-Bridge Motor Surucu Devresi

### 5.1 L298N Motor Surucu

L298N, cift H-bridge motor surucudur. Bir DC motorun her iki yonde surmesine olanak tanir.

**Temel Ozellikleri:**
- Calisma gerilimi: 5V - 35V
- Maksimum akim: 2A (kanal basina)
- Mantik seviyesi: 5V (STM32 3.3V ile uyumlu - seviye cevirici gerekebilir)
- Dahili diyotlar (geri EMK korumasi)

**Kontrol Tablosu:**

| IN1 | IN2 | ENA (PWM) | Motor Durumu |
|-----|-----|-----------|--------------|
| 0   | 0   | X         | Serbest (Coast) |
| 1   | 0   | PWM       | Ileri Yonde |
| 0   | 1   | PWM       | Geri Yonde |
| 1   | 1   | X         | Fren (Brake) |

### 5.2 TB6612FNG Motor Surucu (Alternatif)

TB6612FNG, L298N'e gore daha verimli (MOSFET tabanli) bir motor surucudur.

**Temel Ozellikleri:**
- Calisma gerilimi: 2.5V - 13.5V
- Maksimum akim: 1.2A (surekli), 3.2A (tepe)
- Mantik seviyesi: 2.7V - 5.5V (STM32 3.3V ile dogrudan uyumlu)
- Dusuk ON direnci: 0.5 Ohm

**Kontrol Tablosu:**

| AIN1 | AIN2 | PWMA | Motor Durumu |
|------|------|------|--------------|
| 0    | 0    | X    | Serbest |
| 1    | 0    | PWM  | Ileri |
| 0    | 1    | PWM  | Geri |
| 1    | 1    | X    | Fren |
| X    | X    | 0    | Serbest |

### 5.3 Dead-Time (Olu Zaman) Onemi

H-bridge devresinde ust ve alt anahtarlar asla ayni anda iletimde olmamalidir. Aksi takdirde
"shoot-through" (kisa devre) olusur ve buyuk akim akar. Dead-time, her iki anahtarin da
kapali oldugu kisa bir suredir.

```
Tipik Dead-Time: 100ns - 1us

TIM1 BDTR register'i ile donanim tabanli dead-time eklenebilir:

Dead-time = DTG[7:0] * t_DTS

t_DTS = 1 / TIM1_Clock = 1 / 168MHz = ~5.95ns

DTG = 0x20 (32 desimal) icin:
Dead-time = 32 * 5.95ns = ~190ns
```

---

## 6. Pin Baglanti Semasi

### 6.1 ASCII Baglanti Diyagrami

```
    STM32F407VG Discovery Board
    +---------------------------+
    |                           |
    |  PA8  (TIM1_CH1) ------->|---> H-Bridge ENA (PWM)
    |  PB12 (GPIO_OUT) ------->|---> H-Bridge IN1
    |  PB13 (GPIO_OUT) ------->|---> H-Bridge IN2
    |                           |
    |  PA6  (TIM3_CH1) <-------|<--- Encoder A
    |  PA7  (TIM3_CH2) <-------|<--- Encoder B
    |                           |
    |  PA2  (USART2_TX) ------>|---> USB-UART TX
    |  PA3  (USART2_RX) <------|<--- USB-UART RX
    |                           |
    |  PD12 (LED_GREEN)  [O]   |  Calisma gostergesi
    |  PD13 (LED_ORANGE) [O]   |  Hedef RPM ulasildi
    |  PD14 (LED_RED)    [O]   |  Hata / Asiri akim
    |  PD15 (LED_BLUE)   [O]   |  (Rezerv)
    |                           |
    |  PA0  (ADC1_IN0) <-------|<--- Akim Sensoru (ACS712)
    |                           |
    +---------------------------+

    H-Bridge (L298N)
    +---------------------------+
    |                           |
    |  ENA <--- PA8 (PWM)       |
    |  IN1 <--- PB12            |
    |  IN2 <--- PB13            |
    |                           |
    |  OUT1 -------+            |
    |  OUT2 ----+  |            |
    |           |  |            |
    |  VCC: 12V |  |            |
    |  GND: GND |  |            |
    +-----------|--|------------+
                |  |
          +-----|--|-----+
          |   M O T O R  |
          |   +Encoder   |
          |              |
          |  M+ <-- OUT1 |
          |  M- <-- OUT2 |
          |  A  --> PA6  |
          |  B  --> PA7  |
          |  VCC --> 3.3V|
          |  GND --> GND |
          +--------------+
```

### 6.2 Detayli Pin Tablosu

| STM32 Pin | Fonksiyon | Yonu | Baglanti | Aciklama |
|-----------|-----------|------|----------|----------|
| PA8  | TIM1_CH1 | Output | H-Bridge ENA | 20kHz PWM |
| PB12 | GPIO_Output | Output | H-Bridge IN1 | Yon kontrol 1 |
| PB13 | GPIO_Output | Output | H-Bridge IN2 | Yon kontrol 2 |
| PA6  | TIM3_CH1 | Input | Encoder A | Quadrature faz A |
| PA7  | TIM3_CH2 | Input | Encoder B | Quadrature faz B |
| PA2  | USART2_TX | Output | USB-UART RX | Debug/Komut |
| PA3  | USART2_RX | Input | USB-UART TX | Debug/Komut |
| PA0  | ADC1_IN0 | Input | ACS712 OUT | Akim olcumu |
| PD12 | GPIO_Output | Output | LED (Board) | Yesil - Calisma |
| PD13 | GPIO_Output | Output | LED (Board) | Turuncu - Hedefe ulasildi |
| PD14 | GPIO_Output | Output | LED (Board) | Kirmizi - Hata |
| PD15 | GPIO_Output | Output | LED (Board) | Mavi - Rezerv |

### 6.3 Guc Baglantilari

```
+12V Guc Kaynagi
     |
     +--- L298N VCC (Motor Gucu)
     |
     +--- 7805 Regulator --> +5V --> L298N Mantik
     |
    GND --- STM32 GND --- L298N GND --- Encoder GND --- Guc Kaynagi GND
                |
                +--- USB ile 3.3V (STM32 kartinin kendi regulatoru)

ONEMLI: Tum GND'ler ortak olmalidir!
```

---

## 7. CubeMX Konfigurasyon Adimlari

### 7.1 Yeni Proje Olusturma

1. STM32CubeMX'i acin
2. "New Project" secin
3. MCU olarak **STM32F407VGTx** secin
4. "Start Project" tiklayin

### 7.2 Saat Konfigurasyonu (Clock Configuration)

1. **RCC** sekmesine gidin
2. HSE: Crystal/Ceramic Resonator secin
3. Clock Configuration sekmesinde:
   - HSE = 8 MHz (Discovery board kristali)
   - PLL_M = 8
   - PLL_N = 336
   - PLL_P = 2
   - System Clock = 168 MHz
   - APB1 Prescaler = 4 -> APB1 = 42 MHz (Timer clock = 84 MHz)
   - APB2 Prescaler = 2 -> APB2 = 84 MHz (Timer clock = 168 MHz)

### 7.3 TIM1 Konfigurasyonu (PWM)

1. **TIM1** sekmesine gidin
2. Clock Source: Internal Clock
3. Channel 1: PWM Generation CH1
4. Parameter Settings:
   - Prescaler (PSC): 0
   - Counter Mode: Up
   - Counter Period (ARR): 8399
   - Auto-Reload Preload: Enable
   - CH1 Mode: PWM mode 1
   - CH1 Pulse (CCR): 0
   - CH1 Polarity: High
5. BDTR Settings:
   - Dead Time: 32 (190ns)
   - MOE (Main Output Enable): Enable

### 7.4 TIM3 Konfigurasyonu (Encoder)

1. **TIM3** sekmesine gidin
2. Combined Channels: Encoder Mode
3. Parameter Settings:
   - Encoder Mode: Encoder Mode TI1 and TI2
   - Counter Period (ARR): 65535
   - Polarity: Rising (her iki kanal)
   - IC Filter: 6 (gurultu filtreleme)
4. NVIC Settings:
   - TIM3 global interrupt: Enable (tasma tespiti icin)

### 7.5 TIM6 Konfigurasyonu (Kontrol Dongusu)

1. **TIM6** sekmesine gidin
2. Activated: Isaretleyin
3. Parameter Settings:
   - Prescaler (PSC): 839
   - Counter Period (ARR): 999
   - Auto-Reload Preload: Enable
4. NVIC Settings:
   - TIM6 DAC global interrupt: Enable

### 7.6 USART2 Konfigurasyonu

1. **USART2** sekmesine gidin
2. Mode: Asynchronous
3. Parameter Settings:
   - Baud Rate: 115200
   - Word Length: 8 Bits
   - Parity: None
   - Stop Bits: 1
4. DMA Settings (opsiyonel):
   - USART2_TX: Memory To Peripheral, Increment Memory
   - USART2_RX: Peripheral To Memory, Increment Memory
5. NVIC Settings:
   - USART2 global interrupt: Enable

### 7.7 GPIO Konfigurasyonu

1. **PB12**: GPIO_Output, Push-Pull, No Pull, High Speed -> IN1
2. **PB13**: GPIO_Output, Push-Pull, No Pull, High Speed -> IN2
3. **PD12**: GPIO_Output, Push-Pull, No Pull, Low Speed -> LED Green
4. **PD13**: GPIO_Output, Push-Pull, No Pull, Low Speed -> LED Orange
5. **PD14**: GPIO_Output, Push-Pull, No Pull, Low Speed -> LED Red
6. **PD15**: GPIO_Output, Push-Pull, No Pull, Low Speed -> LED Blue

### 7.8 ADC1 Konfigurasyonu (Akim Olcumu)

1. **ADC1** sekmesine gidin
2. IN0: Enable
3. Parameter Settings:
   - Resolution: 12 bits
   - Scan Conversion Mode: Disable
   - Continuous Conversion Mode: Disable
   - Sampling Time: 84 Cycles

### 7.9 Proje Uretimi

1. Project Manager sekmesine gidin
2. Project Name: PID-Motor-Control
3. Toolchain/IDE: Makefile (veya kullandiginiz IDE)
4. Generate Code tiklayin

---

## 8. PID Ayarlama (Tuning) Proseduru

### 8.1 Ziegler-Nichols Yontemi

Ziegler-Nichols, deneysel olarak PID parametrelerini belirlemek icin kullanilan klasik bir
yontemdir. Iki varyanti vardir:

#### 8.1.1 Adim Yaniti Yontemi (Acik Cevrim)

1. PID kontroloru devre disi birakin (acik cevrim)
2. Motora basamak giris uygulayın (%50 duty cycle)
3. Hiz yanitini kaydedin
4. Yanit egrisinden su parametreleri belirleyin:
   - **L** (Dead Time): Giristen ilk tepkiye kadar gecen sure
   - **T** (Zaman Sabiti): %63'e ulasma suresi
   - **K** (Sistem Kazanci): Kararli hal degeri / giris degeri

```
PID Parametreleri (Ziegler-Nichols Adim Yaniti):

| Kontrolor | Kp          | Ti        | Td       |
|-----------|-------------|-----------|----------|
| P         | T/(K*L)     | -         | -        |
| PI        | 0.9*T/(K*L) | L/0.3     | -        |
| PID       | 1.2*T/(K*L) | 2*L       | 0.5*L    |

Ki = Kp / Ti
Kd = Kp * Td
```

#### 8.1.2 Kritik Kazanc Yontemi (Kapali Cevrim)

1. Ki = 0, Kd = 0 yapin (sadece P kontrolor)
2. Kp degerini yavas yavas artirin
3. Sistem kararli osilasyon yapana kadar devam edin
4. Bu noktadaki degerleri kaydedin:
   - **Ku** (Kritik Kazanc): Osilasyona neden olan Kp degeri
   - **Pu** (Kritik Periyot): Osilasyonun periyodu

```
PID Parametreleri (Ziegler-Nichols Kritik Kazanc):

| Kontrolor | Kp      | Ti      | Td       |
|-----------|---------|---------|----------|
| P         | 0.5*Ku  | -       | -        |
| PI        | 0.45*Ku | Pu/1.2  | -        |
| PID       | 0.6*Ku  | Pu/2    | Pu/8     |

Ki = Kp / Ti
Kd = Kp * Td
```

### 8.2 Manuel Ayarlama Yontemi

Ziegler-Nichols iyi bir baslangic noktasi verir ancak genellikle ince ayar gerektirir.
Manuel ayarlama adimlari:

**Adim 1: Sadece P ile baslayin**
```
Ki = 0, Kd = 0
Kp = kucuk bir degerle baslayin (ornegin 0.5)
Hedef RPM'e bir basamak giris uygulayın
Kp'yi artirin: Daha hizli yanit, ancak overshoot artar
Kabul edilebilir overshoot (%10-20) elde edene kadar ayarlayin
```

**Adim 2: I terimini ekleyin**
```
Kucuk bir Ki degeri ekleyin (ornegin Kp/100)
Kararli hal hatasinin sifira indigini dogrulayin
Ki'yi artirin: Daha hizli offset duzeltme, ancak osilasyon riski
Asiri osilasyon baslarsa Ki'yi azaltin
```

**Adim 3: D terimini ekleyin**
```
Kucuk bir Kd degeri ekleyin (ornegin Kp/10)
Overshoot'un azaldigini dogrulayin
Kd'yi artirin: Daha az overshoot, ancak gurultu hassasiyeti artar
Gurultu nedeniyle motor titremeye baslarsa Kd'yi azaltin
```

### 8.3 Cohen-Coon Yontemi

Cohen-Coon yontemi, Ziegler-Nichols'a alternatif olarak ozellikle olu zamani buyuk sistemler
icin daha iyi sonuc verir:

```
K: Sistem kazanci
T: Zaman sabiti
L: Olu zaman
r = L/T

PID Parametreleri:
Kp = (1/(K*r)) * (4/3 + r/4)
Ti = L * (32 + 6*r) / (13 + 8*r)
Td = L * 4 / (11 + 2*r)
```

### 8.4 Yazilim ile Otomatik Ayarlama (Bu Projede)

UART arayuzu uzerinden gercek zamanli ayarlama yapilabilir:

```
1. UART terminalini acin (115200 baud)
2. Baslangic degerleri ile baslayin:
   SET KP 1.0
   SET KI 0.0
   SET KD 0.0

3. Veri akisini baslatın:
   STREAM ON

4. Hedef RPM belirleyin:
   SET RPM 500

5. Gelen verileri PC'de grafik olarak cizin
   (Python matplotlib veya SerialPlot kullanabilirsiniz)

6. Kp, Ki, Kd degerlerini interaktif olarak ayarlayin

7. Optimum parametreleri buldugunuzda kodda varsayilan degerleri guncelleyin
```

---

## 9. Anti-Windup Stratejileri

### 9.1 Integral Wind-up Problemi

Motor fiziksel sinirlarına ulastiginda (ornegin PWM %100), kontrol cikisi doyuma girer.
Ancak hata hala mevcutsa integral terimi birikmeye devam eder. Motor normal calisma
araligina dondugunde, biriken integral degeri asiri buyuk kontrol sinyali olusturur ve
buyuk overshoot'a neden olur.

```
Ornek Senaryo:
- Hedef: 1000 RPM, Motor maks: 800 RPM (yuk altinda)
- Hata surekli: 1000 - 800 = 200
- Integral birikir: 200, 400, 600, 800, 1000...
- Yuk kaldirildiginda motor asiri hizlanir cunku integral cok buyuktur
```

### 9.2 Clamping (Sinirlandirma) Yontemi

Bu projede kullanilan yontem: Cikis doyuma girdiyse integral guncellenmez.

```c
// PID hesaplamasi
float output = P_term + I_term + D_term;

// Cikis doyumda mi kontrol et
bool is_saturated = (output > max_output) || (output < min_output);

// Integral guncelleme: Sadece doyumda degilse veya
// hata integral'i doyumdan cikartacak yondeyse guncelle
if (!is_saturated || (error * I_term < 0)) {
    integral_sum += error * dt;
}

// Cikisi sinirla
if (output > max_output) output = max_output;
if (output < min_output) output = min_output;
```

### 9.3 Back-Calculation Yontemi (Alternatif)

```
Anti-windup kazanci: Ka = 1/Ti (veya kullanici tanimlı)

Doyum farki: e_s = u_saturated - u_unsaturated

Integral duzeltme: I_term += (Ki * error + Ka * e_s) * dt
```

### 9.4 Integral Sinirlandirma (Basit Yontem)

```c
// Integral terimine dogrudan sinir koy
integral_sum += error * dt;
if (integral_sum > integral_max) integral_sum = integral_max;
if (integral_sum < integral_min) integral_sum = integral_min;
```

---

## 10. Yazilim Mimarisi

### 10.1 Modul Yapisi

```
02-PID-Motor-Control/
|
|-- README.md                  <- Bu dosya
|
|-- inc/
|   |-- pid_controller.h       <- PID algoritma arayuzu
|   |-- motor_driver.h         <- Motor surucu arayuzu
|   |-- encoder.h              <- Encoder arayuzu
|
|-- src/
|   |-- pid_controller.c       <- PID implementasyonu
|   |-- motor_driver.c         <- Motor surucu implementasyonu
|   |-- encoder.c              <- Encoder implementasyonu
|   |-- main.c                 <- Ana uygulama
```

### 10.2 Kontrol Dongusu Akis Diyagrami

```
                +------------------+
                |  TIM6 Interrupt  |
                |    (100Hz)       |
                +--------+---------+
                         |
                         v
                +--------+---------+
                | Encoder Oku      |
                | (TIM3 Counter)   |
                +--------+---------+
                         |
                         v
                +--------+---------+
                | RPM Hesapla      |
                | delta_count/dt   |
                +--------+---------+
                         |
                         v
    Setpoint ---->  +----+----+
    (UART'tan)      |  PID    |
                    | Compute |
                    +----+----+
                         |
                    Control Signal
                         |
                         v
                +--------+---------+
                | Motor Driver     |
                | PWM + Direction  |
                +--------+---------+
                         |
                         v
                +--------+---------+
                |    DC Motor      |
                |   + Encoder      |
                +------------------+
```

### 10.3 Kesme (Interrupt) Oncelikleri

| Kesme | Oncelik | Aciklama |
|-------|---------|----------|
| TIM6_DAC_IRQn | 2 | PID kontrol dongusu (100Hz) |
| TIM3_IRQn | 1 | Encoder tasma sayaci |
| USART2_IRQn | 3 | UART komut isleme |
| ADC_IRQn | 4 | Akim olcumu (opsiyonel) |

---

## 11. UART Komut Arayuzu

### 11.1 Desteklenen Komutlar

| Komut | Aciklama | Ornek |
|-------|----------|-------|
| SET RPM <deger> | Hedef hizi ayarla | SET RPM 1000 |
| SET KP <deger> | Oransal kazanc ayarla | SET KP 1.5 |
| SET KI <deger> | Integral kazanc ayarla | SET KI 0.1 |
| SET KD <deger> | Turev kazanc ayarla | SET KD 0.05 |
| GET STATUS | Mevcut durumu oku | GET STATUS |
| STREAM ON | Veri akisini baslat | STREAM ON |
| STREAM OFF | Veri akisini durdur | STREAM OFF |
| STOP | Acil durdurma | STOP |
| RESET PID | PID'i sifirla | RESET PID |

### 11.2 Cikis Formati

**GET STATUS yaniti:**
```
--- Motor Status ---
Setpoint: 1000 RPM
Current:  987 RPM
Error:    13 RPM
Duty:     67.5%
PID: Kp=1.50 Ki=0.10 Kd=0.05
State: RUNNING
--------------------
```

**STREAM ON ciktisi (CSV formati):**
```
time_ms,setpoint,rpm,duty,error
10,1000,0,100.0,1000
20,1000,45,98.5,955
30,1000,120,95.2,880
40,1000,230,89.1,770
...
```

Bu CSV verisi Python ile kolayca gorsellestirilebilir:
```python
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ser = serial.Serial('COM3', 115200)
# ... gercek zamanli grafik cizdirme kodu ...
```

---

## 12. Test Proseduru

### 12.1 Donanim Testi

**Adim 1: Guc Kaynagi Kontrolu**
- 12V guc kaynagini multimetre ile dogrulayin
- STM32 kartinin USB ile beslendigini kontrol edin
- GND baglantilarinin dogru yapildigini dogrulayin

**Adim 2: Motor Surucu Testi**
- PWM olmadan IN1=1, IN2=0 yapin -> Motor donmeli
- IN1=0, IN2=1 yapin -> Motor ters donmeli
- IN1=1, IN2=1 yapin -> Motor durmali (fren)
- IN1=0, IN2=0 yapin -> Motor serbest donmeli

**Adim 3: Encoder Testi**
- Motoru elle cevirin
- TIM3->CNT degerinin degistigini dogrulayin
- Ileri ve geri cevirdiginde sayacin arttigini/azaldigini kontrol edin
- UART uzerinden encoder degerlerini okuyun

**Adim 4: PWM Testi**
- Osiloskop ile PA8 pinindeki PWM sinyalini olcun
- Frekans: 20kHz olmali
- Duty cycle: Yazilimda ayarladiginiz deger olmali

### 12.2 Yazilim Testi

**Test 1: Acik Cevrim Yaniti**
```
1. PID'i devre disi birakin
2. Dogrudan PWM degeri ayarlayin (%25, %50, %75, %100)
3. Her deger icin kararli hal RPM'ini kaydedin
4. Motor kazancini hesaplayin: K = RPM / PWM_duty
```

**Test 2: Basamak Yaniti (Step Response)**
```
1. PID parametrelerini ayarlayin
2. STREAM ON komutu ile veri akisini baslatin
3. SET RPM 500 komutu gonderin
4. Yaniti kaydedin ve analiz edin:
   - Yukselme suresi (Rise Time): %10'dan %90'a ulasma suresi
   - Oturma suresi (Settling Time): %2 banda girme suresi
   - Asma orani (Overshoot): Tepe deger / Hedef deger
   - Kararli hal hatasi (Steady-State Error)
```

**Test 3: Bozucu Etki Reddi (Disturbance Rejection)**
```
1. Motor sabit hizda calisirken elle yuk uygulayin
2. PID'in hizi tekrar hedefe getirmesini gozleyin
3. Toparlanma suresini olcun
```

**Test 4: Setpoint Degisimi**
```
1. SET RPM 300 -> bekleyin -> SET RPM 800 -> bekleyin -> SET RPM 500
2. Her gecis icin basamak yaniti performansini degerlendirin
```

**Test 5: Guvenlik Testleri**
```
1. Motor miline asiri yuk uygulayin -> Tikanma tespiti calismali
2. Cok yuksek RPM hedefi verin -> Maks RPM siniri calismali
3. STOP komutu gonderin -> Motor hemen durmali
4. Guc kaynagini kesin -> Motor serbest durmali
```

### 12.3 Performans Kriterleri

| Parametre | Hedef Deger | Kabul Edilebilir Aralik |
|-----------|-------------|------------------------|
| Yukselme Suresi | < 200 ms | < 500 ms |
| Oturma Suresi | < 500 ms | < 1000 ms |
| Asma Orani | < %10 | < %20 |
| Kararli Hal Hatasi | 0 RPM | +/- 5 RPM |
| Bozucu Reddi | < 300 ms | < 500 ms |

---

## 13. Sorun Giderme

### 13.1 Sik Karsilasilan Sorunlar

**Sorun: Motor donmuyor**
- PWM sinyalini osiloskop ile kontrol edin
- H-bridge guc baglantisini kontrol edin
- IN1/IN2 sinyal seviyelerini kontrol edin
- Motor baglantisini dogrudan 12V ile test edin

**Sorun: Encoder okumuyor**
- Encoder guc baglantisini kontrol edin (3.3V veya 5V)
- Encoder cikis sinyallerini osiloskop ile kontrol edin
- TIM3 encoder modunun dogru konfigure edildigini dogrulayin
- Input filter degeri cok yuksekse hizli sinyalleri kacirir

**Sorun: PID kararsiz (osilasyon)**
- Kp degerini azaltin
- Kd degerini artirin
- Ornekleme frekansini kontrol edin (100Hz yeterli mi?)
- Mekanik gevselik (backlash) olup olmadigini kontrol edin

**Sorun: Integral windup**
- Anti-windup kodunun aktif oldugunu dogrulayin
- Integral sinirlarini kontrol edin
- Ki degerini azaltin

**Sorun: Motor titresiyor (yuksek frekansta)**
- Kd degerini azaltin
- Turev filtresinin aktif oldugunu dogrulayin
- PWM frekansini kontrol edin (20kHz altinda motor sesi olur)
- Encoder baglantilarinda parazit olup olmadigini kontrol edin

**Sorun: Kararli hal hatasi sifirlanmiyor**
- Ki degerinin sifirdan buyuk oldugunu dogrulayin
- Anti-windup cok agresif olabilir, sinirlari genisletin
- Encoder cozunurlugu dusuk hizlarda yetersiz olabilir

### 13.2 Debug Ipuclari

1. **UART Debug**: Tum degiskenleri UART uzerinden yazdirin
2. **LED Gostergeler**: Farkli durumlari LED'lerle kodlayin
3. **Osiloskop**: PWM sinyali ve encoder sinyallerini gorsel olarak inceleyin
4. **Data Logging**: STREAM modu ile verileri kaydedin ve PC'de analiz edin
5. **Breakpoint**: Debugger ile kritik noktalarda durdurun ve degiskenleri inceleyin

### 13.3 Performans Optimizasyonu

- **Sabit noktali aritmetik**: Float yerine Q16.16 formatinda hesaplama yaparak
  islemci yukunu azaltabilirsiniz (bu projede float kullanilmistir, STM32F4'un FPU'su
  oldugu icin performans kabul edilebilir seviyededir)
- **DMA ile UART**: Buyuk veri aktarimlarinda DMA kullanarak CPU yukunu azaltin
- **Lookup Table**: Trigonometrik veya karmasik hesaplamalarda tablo kullanin

---

## Lisans

Bu proje egitim amacli gelistirilmistir. MIT Lisansi ile lisanslanmistir.

## Yazar

STM32F407VG Gomulu Sistemler Portfoy Projesi - Proje 2

---

*Bu dokuman, PID motor kontrol sisteminin tum yonlerini kapsamli sekilde aciklamaktadir.
Herhangi bir sorunuz veya oneriniz icin issue acabilirsiniz.*
