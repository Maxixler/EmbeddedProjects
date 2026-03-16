# Proje 3: I2C-MPU6050-KalmanFilter

## STM32F407VG ile MPU6050 IMU Sensoer Okuma ve Kalman Filtresi ile Sensor Fuzyonu

---

## Icindekiler

1. [Proje Ozeti](#1-proje-ozeti)
2. [I2C Protokolu Detayli Aciklamasi](#2-i2c-protokolu-detayli-aciklamasi)
3. [STM32F407VG I2C Donanim Blogu](#3-stm32f407vg-i2c-donanim-blogu)
4. [MPU6050 Sensor Detaylari](#4-mpu6050-sensor-detaylari)
5. [Accelerometer ile Aci Hesaplama](#5-accelerometer-ile-aci-hesaplama)
6. [Gyroscope ile Aci Hesaplama](#6-gyroscope-ile-aci-hesaplama)
7. [Kalman Filtresi Teorisi](#7-kalman-filtresi-teorisi)
8. [Complementary Filter vs Kalman Filter](#8-complementary-filter-vs-kalman-filter)
9. [Donanim Baglantilari](#9-donanim-baglantilari)
10. [CubeMX Konfigurasyonu](#10-cubemx-konfigurasyonu)
11. [Yazilim Mimarisi](#11-yazilim-mimarisi)
12. [Kalibrasyon Proseduru](#12-kalibrasyon-proseduru)
13. [Test Proseduru](#13-test-proseduru)
14. [UART Komut Arayuzu](#14-uart-komut-arayuzu)
15. [Performans Analizi](#15-performans-analizi)
16. [Sorun Giderme](#16-sorun-giderme)
17. [Kaynaklar](#17-kaynaklar)

---

## 1. Proje Ozeti

Bu proje, STM32F407VG mikrodenetleyicisi kullanarak InvenSense MPU6050 6 eksenli
IMU (Inertial Measurement Unit - Ataletsel Olcum Birimi) sensorunden I2C protokolu
ile ivmeolcer (accelerometer) ve jiroskop (gyroscope) verilerini okur. Okunan ham
veriler uzerinde Kalman filtresi uygulanarak gurultuden arindirilmis, drift
sorunlarindan kurtulmus hassas roll (yuvarlama) ve pitch (yunuslama) acilari
hesaplanir.

### Projenin Temel Ozellikleri

- **I2C Haberlesme**: 400 kHz Fast Mode I2C ile MPU6050 okuma
- **14 Byte Burst Read**: Tek islemde accelerometer, sicaklik ve gyroscope verileri
- **Kalman Filtresi**: 2 durumlu (aci ve bias) Kalman filtresi ile sensor fuzyonu
- **Complementary Filter**: Karsilastirma icin tamamlayici filtre implementasyonu
- **200 Hz Ornekleme**: TIM7 timer kesmesi ile hassas zamanlama
- **UART Cikis**: 115200 baud ile CSV formatinda veri aktarimi
- **Komut Arayuzu**: Kalibrasyon, parametre ayari ve mod secimi
- **Kalibrasyon**: Otomatik offset hesaplama rutini
- **Self-Test**: MPU6050 dahili self-test ozelligi destegi

### Hedef Kitle

Bu proje asagidaki konularda deneyim kazanmak isteyenler icindir:
- Gomulu sistemlerde I2C haberlesme protokolu
- IMU sensor okuma ve veri isleme
- Kalman filtresi teorisi ve pratik uygulamasi
- Sensor fuzyonu (sensor fusion) teknikleri
- Gercel zamanli sinyal isleme

---

## 2. I2C Protokolu Detayli Aciklamasi

### 2.1 I2C Nedir?

I2C (Inter-Integrated Circuit), Philips Semiconductor (simdi NXP) tarafindan 1982
yilinda gelistirilen, iki telli senkron seri haberlesme protokoludur. Ayni bus
uzerinde birden fazla master ve slave cihazin bagli olabilmesine olanak tanir.

### 2.2 Fiziksel Katman

I2C bus'i iki hat kullanir:
- **SDA (Serial Data)**: Cift yonlu veri hatti
- **SCL (Serial Clock)**: Master tarafindan uretilen saat sinyali

Her iki hat da acik-kollektorlu (open-drain) cikislar ile surulur ve harici
pull-up direncler (tipik olarak 4.7 kOhm) ile VDD'ye cekilir. Bu sayede:
- Herhangi bir cihaz hatti LOW'a cekebilir
- Hic bir cihaz cekmezse hat HIGH kalir (pull-up ile)
- Wired-AND mantigi olusur, bu da collision detection'i mumkun kilar

```
VDD (3.3V)
 |         |
[4.7k]    [4.7k]
 |         |
 +--SDA    +--SCL
 |         |
[Master]--[Slave1]--[Slave2]--...
```

### 2.3 Hiz Modlari

| Mod              | Maksimum Hiz | Kullanim Alani        |
|------------------|-------------|------------------------|
| Standard Mode    | 100 kbit/s  | Genel amacli           |
| Fast Mode        | 400 kbit/s  | Sensorler, EEPROM      |
| Fast Mode Plus   | 1 Mbit/s    | Yuksek hiz gerektiren  |
| High Speed Mode  | 3.4 Mbit/s  | Ozel uygulamalar       |

Bu projede **Fast Mode (400 kHz)** kullanilmaktadir. MPU6050 sensoru 400 kHz'e
kadar destekler.

### 2.4 Start ve Stop Kosullari

I2C haberlesme, START ve STOP kosullari ile baslar ve biter:

**START Kosulu (S)**:
- SCL HIGH iken SDA'nin HIGH'dan LOW'a gecmesi
- Bus'un mesgul oldugunu isaret eder
- Sadece master uretebilir

```
SDA: ____
          \___
SCL: ________
          (START)
```

**STOP Kosulu (P)**:
- SCL HIGH iken SDA'nin LOW'dan HIGH'a gecmesi
- Bus'un serbest oldugunu isaret eder
- Sadece master uretebilir

```
SDA:      ___
    ____/
SCL: ________
         (STOP)
```

**REPEATED START (Sr)**:
- Stop gondermeden yeni bir Start kosulu olusturma
- Bus'u serbest birakmadan yeni bir transfer baslatma
- Register okuma islemlerinde cok onemlidir (yazma + okuma birlesmesi)

### 2.5 Adres Yapisi

#### 7-Bit Adresleme

En yaygin kullanilan mod. Ilk byte'ta 7 bitlik adres ve 1 bitlik R/W biti bulunur:

```
Bit:  [A6][A5][A4][A3][A2][A1][A0][R/W]
       |<---  7-bit adres  --->|
       R/W = 0: Yazma (Master -> Slave)
       R/W = 1: Okuma (Slave -> Master)
```

MPU6050 icin:
- AD0 = LOW  -> Adres = 0x68 (1101000) -> Yazma: 0xD0, Okuma: 0xD1
- AD0 = HIGH -> Adres = 0x69 (1101001) -> Yazma: 0xD2, Okuma: 0xD3

#### 10-Bit Adresleme

Daha fazla cihaz adreslemek icin kullanilir. Iki byte'lik adres yapisi:
```
Birinci byte: [1][1][1][1][0][A9][A8][R/W]
Ikinci byte:  [A7][A6][A5][A4][A3][A2][A1][A0]
```

### 2.6 ACK ve NACK Mekanizmasi

Her 8 bitlik veri transferinden sonra alici taraf bir onay (acknowledge) biti gonderir:

**ACK (Acknowledge)**:
- Alici SDA'yi LOW'a ceker (9. clock darbesinde)
- Verinin basariyla alinidigini gosterir

**NACK (Not Acknowledge)**:
- Alici SDA'yi HIGH birakir (9. clock darbesinde)
- Asagidaki durumlarda olusur:
  - Slave adresi bus uzerinde mevcut degil
  - Slave veri almaya hazir degil
  - Master son byte'i aldigini belirtmek istiyor (okuma sonlandirma)

```
Veri Byte'i:     [D7][D6][D5][D4][D3][D2][D1][D0][ACK/NACK]
                 |<--------  8 bit veri  -------->| 9. bit
```

### 2.7 Clock Stretching

Slave cihaz, veriyi islemeye zamana ihtiyac duydugunda SCL hattini LOW'da tutarak
master'i bekletebilir. Bu mekanizmaya "clock stretching" denir.

- Slave SCL'yi LOW tutar
- Master SCL'yi HIGH yapmak ister ama hat LOW'da kalir
- Master bekler
- Slave hazir oldugunda SCL'yi serbest birakir
- Master devam eder

MPU6050 clock stretching kullabilir, bu nedenle I2C konfigurasyonunda buna
dikkat edilmelidir.

### 2.8 Tipik I2C Register Okuma Silinsilesi

MPU6050'den bir register okumak icin:

```
[S][Slave Addr + W][ACK][Register Addr][ACK][Sr][Slave Addr + R][ACK][Data][NACK][P]

Adim adim:
1. START kosulu
2. Slave adresi + Write bit (0xD0) gonder, ACK bekle
3. Okunacak register adresini gonder, ACK bekle
4. REPEATED START kosulu
5. Slave adresi + Read bit (0xD1) gonder, ACK bekle
6. Veriyi oku, son byte'ta NACK gonder
7. STOP kosulu
```

### 2.9 Burst Read (Coklu Okuma)

MPU6050 otomatik register artirma (auto-increment) destekler. Tek bir okuma
isleminde ardisik register'lari okuyabilirsiniz:

```
[S][0xD0][ACK][0x3B][ACK][Sr][0xD1][ACK][D0][ACK][D1][ACK]...[D13][NACK][P]

Register 0x3B'den baslayarak 14 byte okuma:
- Byte 0-1:  ACCEL_XOUT (H,L)
- Byte 2-3:  ACCEL_YOUT (H,L)
- Byte 4-5:  ACCEL_ZOUT (H,L)
- Byte 6-7:  TEMP_OUT (H,L)
- Byte 8-9:  GYRO_XOUT (H,L)
- Byte 10-11: GYRO_YOUT (H,L)
- Byte 12-13: GYRO_ZOUT (H,L)
```

Bu yontem ile I2C bus uzerindeki trafik minimuma indirilir ve tum eksen
verileri neredeyse ayni anda okunmus olur (veri tutarliligi).

---

## 3. STM32F407VG I2C Donanim Blogu

### 3.1 Genel Bakis

STM32F407VG mikrodenetleyicisi 3 adet I2C birimi icerir (I2C1, I2C2, I2C3).
Bu projede **I2C1** kullanilmaktadir.

I2C1 pin atamasi:
- **PB6**: I2C1_SCL (AF4)
- **PB7**: I2C1_SDA (AF4)

### 3.2 I2C Register Haritasi

#### I2C_CR1 (Control Register 1) - Offset: 0x00

| Bit | Alan       | Aciklama                                    |
|-----|-----------|----------------------------------------------|
| 15  | SWRST     | Yazilim reset (1: reset, 0: normal)          |
| 13  | ALERT     | SMBus alert                                  |
| 12  | PEC       | Packet Error Checking transfer               |
| 11  | POS       | ACK/PEC pozisyonu (2 byte okuma icin)        |
| 10  | ACK       | Acknowledge enable (1: ACK dondu)            |
| 9   | STOP      | Stop uretimi (1: Stop kosulu olustur)        |
| 8   | START     | Start uretimi (1: Start kosulu olustur)      |
| 7   | NOSTRETCH | Clock stretching devre disi (slave modda)    |
| 6   | ENGC      | General call enable                          |
| 5   | ENPEC     | PEC enable                                   |
| 4   | ENARP     | ARP enable                                   |
| 3   | SMBTYPE   | SMBus type                                   |
| 1   | SMBUS     | SMBus mode                                   |
| 0   | PE        | Peripheral enable (1: I2C aktif)             |

#### I2C_CR2 (Control Register 2) - Offset: 0x04

| Bit   | Alan     | Aciklama                                      |
|-------|---------|------------------------------------------------|
| 12    | LAST    | DMA son transfer                               |
| 11    | DMAEN   | DMA istek enable                               |
| 10    | ITBUFEN | Buffer kesme enable                            |
| 9     | ITEVTEN | Event kesme enable                             |
| 8     | ITERREN | Error kesme enable                             |
| 5:0   | FREQ    | Peripheral clock frekansi (MHz, 2-42 arasi)   |

FREQ alani APB1 bus frekansini MHz cinsinden icerir. STM32F407VG'de APB1
frekansi genellikle 42 MHz'dir, dolayisiyla FREQ = 42.

#### I2C_SR1 (Status Register 1) - Offset: 0x14

| Bit | Alan       | Aciklama                                    |
|-----|-----------|----------------------------------------------|
| 15  | SMBALERT  | SMBus alert                                  |
| 14  | TIMEOUT   | Timeout veya Tlow hatasi                     |
| 12  | PECERR    | PEC hatasi                                   |
| 11  | OVR       | Overrun/underrun hatasi                      |
| 10  | AF        | Acknowledge failure                          |
| 9   | ARLO      | Arbitration lost                             |
| 8   | BERR      | Bus hatasi                                   |
| 7   | TxE       | Veri register bos (gonderme)                 |
| 6   | RxNE      | Veri register dolu (alma)                    |
| 4   | STOPF     | Stop algilandi                               |
| 3   | ADD10     | 10-bit header gonderildi                     |
| 2   | BTF       | Byte transfer tamamlandi                     |
| 1   | ADDR      | Adres gonderildi/eslesti                     |
| 0   | SB        | Start bit (Start kosulu olusturuldu)         |

#### I2C_SR2 (Status Register 2) - Offset: 0x18

| Bit   | Alan       | Aciklama                                  |
|-------|-----------|-------------------------------------------|
| 15:8  | PEC       | Packet error checking register            |
| 7     | DUALF     | Dual flag (ikinci adres eslesmesi)        |
| 6     | SMBHOST   | SMBus host header                         |
| 5     | SMBDEFALT | SMBus device default address              |
| 4     | GENCALL   | General call address alindi               |
| 2     | TRA       | Transmitter/receiver (1: transmitter)     |
| 1     | BUSY      | Bus mesgul                                |
| 0     | MSL       | Master/slave (1: master)                  |

**Onemli Not**: ADDR bayragi temizlemek icin SR1 ve ardindan SR2 okunmalidir.
Bu donanim tasarimi geregi boyledir.

#### I2C_DR (Data Register) - Offset: 0x10

8 bitlik veri register'i. Gondermede yazilir, almada okunur.

#### I2C_CCR (Clock Control Register) - Offset: 0x1C

| Bit   | Alan   | Aciklama                                      |
|-------|-------|------------------------------------------------|
| 15    | F/S   | Mod secimi (0: Standard, 1: Fast)              |
| 14    | DUTY  | Fast Mode duty cycle (0: 2:1, 1: 16:9)        |
| 11:0  | CCR   | Clock control register degeri                  |

Fast Mode 400 kHz icin CCR hesaplama:
- APB1 = 42 MHz, T_high = T_low (duty = 0)
- CCR = F_pclk1 / (2 * F_i2c) = 42000000 / (2 * 400000) = 52.5 -> 53

#### I2C_TRISE (Rise Time Register) - Offset: 0x20

Maksimum yukselme suresi konfigurasyonu:
- Standard Mode: TRISE = (F_pclk1 / 1000000) + 1 = 43
- Fast Mode: TRISE = (F_pclk1 * 300 / 1000000000) + 1 = 13

### 3.3 HAL Kutuphanesi I2C Fonksiyonlari

STM32 HAL kutuphanesi I2C islemlerini kolaylastirir:

```c
/* Register yazma */
HAL_I2C_Mem_Write(hi2c, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);

/* Register okuma */
HAL_I2C_Mem_Read(hi2c, DevAddr, MemAddr, MemAddrSize, pData, Size, Timeout);

/* Cihaz hazir mi kontrolu */
HAL_I2C_IsDeviceReady(hi2c, DevAddr, Trials, Timeout);
```

---

## 4. MPU6050 Sensor Detaylari

### 4.1 Genel Ozellikler

MPU6050, InvenSense (simdi TDK) tarafindan uretilen 6 eksenli IMU sensorudur:
- 3 eksen ivmeolcer (accelerometer)
- 3 eksen jiroskop (gyroscope)
- Dahili sicaklik sensoru
- Dahili DMP (Digital Motion Processor)
- 16-bit ADC cozunurluk
- I2C (400 kHz) ve SPI arayuzu
- Besleme gerilimi: 2.375V - 3.46V
- Boyut: 4mm x 4mm x 0.9mm QFN paketi

### 4.2 Olcum Araliklari

#### Accelerometer Araliklari

| Ayar       | Aralik    | Hassasiyet (LSB/g) | Cozunurluk     |
|-----------|-----------|---------------------|-----------------|
| FS_SEL=0  | +/- 2g    | 16384               | 0.061 mg/LSB   |
| FS_SEL=1  | +/- 4g    | 8192                | 0.122 mg/LSB   |
| FS_SEL=2  | +/- 8g    | 4096                | 0.244 mg/LSB   |
| FS_SEL=3  | +/- 16g   | 2048                | 0.488 mg/LSB   |

#### Gyroscope Araliklari

| Ayar       | Aralik       | Hassasiyet (LSB/dps) | Cozunurluk       |
|-----------|--------------|----------------------|-------------------|
| FS_SEL=0  | +/- 250 dps  | 131.0                | 0.00763 dps/LSB  |
| FS_SEL=1  | +/- 500 dps  | 65.5                 | 0.01527 dps/LSB  |
| FS_SEL=2  | +/- 1000 dps | 32.8                 | 0.03049 dps/LSB  |
| FS_SEL=3  | +/- 2000 dps | 16.4                 | 0.06098 dps/LSB  |

### 4.3 Onemli Register Haritasi

| Register Adresi | Isim                 | Aciklama                         |
|----------------|----------------------|-----------------------------------|
| 0x0D           | SELF_TEST_X          | X ekseni self-test                |
| 0x0E           | SELF_TEST_Y          | Y ekseni self-test                |
| 0x0F           | SELF_TEST_Z          | Z ekseni self-test                |
| 0x10           | SELF_TEST_A          | Accelerometer self-test           |
| 0x19           | SMPLRT_DIV           | Ornekleme hizi bolucusu           |
| 0x1A           | CONFIG               | DLPF ve FSYNC konfigurasyonu      |
| 0x1B           | GYRO_CONFIG          | Gyro aralik ve self-test          |
| 0x1C           | ACCEL_CONFIG         | Accel aralik ve self-test         |
| 0x23           | FIFO_EN              | FIFO enable register              |
| 0x37           | INT_PIN_CFG          | Interrupt pin konfigurasyonu      |
| 0x38           | INT_ENABLE           | Interrupt enable                  |
| 0x3A           | INT_STATUS           | Interrupt status                  |
| 0x3B - 0x40   | ACCEL_XOUT_H - _L    | Accelerometer verileri            |
| 0x41 - 0x42   | TEMP_OUT_H - _L      | Sicaklik verisi                   |
| 0x43 - 0x48   | GYRO_XOUT_H - _L     | Gyroscope verileri                |
| 0x6A           | USER_CTRL            | Kullanici kontrol register        |
| 0x6B           | PWR_MGMT_1           | Guc yonetimi 1                    |
| 0x6C           | PWR_MGMT_2           | Guc yonetimi 2                    |
| 0x72 - 0x73   | FIFO_COUNT_H - _L    | FIFO sayaci                       |
| 0x74           | FIFO_R_W             | FIFO okuma/yazma                  |
| 0x75           | WHO_AM_I             | Cihaz kimlik register (0x68)      |

### 4.4 Guc Yonetimi (PWR_MGMT_1 - 0x6B)

| Bit | Alan        | Aciklama                                    |
|-----|------------|----------------------------------------------|
| 7   | DEVICE_RESET| Cihaz reset (1 yazilinca tum register reset) |
| 6   | SLEEP      | Uyku modu (1: uyku, 0: aktif)               |
| 5   | CYCLE      | Periyodik uyandirma modu                     |
| 3   | TEMP_DIS   | Sicaklik sensoru devre disi                  |
| 2:0 | CLKSEL     | Clock kaynagi secimi                         |

CLKSEL degerleri:
- 0: Dahili 8 MHz RC osilatoru
- 1: PLL, X ekseni gyro referansi (onerilir)
- 2: PLL, Y ekseni gyro referansi
- 3: PLL, Z ekseni gyro referansi

### 4.5 Digital Low-Pass Filter (DLPF)

CONFIG register'i (0x1A) ile DLPF konfigurasyonu:

| DLPF_CFG | Accel BW (Hz) | Gyro BW (Hz) | Ornekleme (kHz) |
|----------|--------------|---------------|------------------|
| 0        | 260          | 256           | 8                |
| 1        | 184          | 188           | 1                |
| 2        | 94           | 98            | 1                |
| 3        | 44           | 42            | 1                |
| 4        | 21           | 20            | 1                |
| 5        | 10           | 10            | 1                |
| 6        | 5            | 5             | 1                |

Bu projede DLPF_CFG = 3 (44 Hz bant genisligi) kullanilmaktadir.
200 Hz ornekleme hizi icin uygundur.

### 4.6 Ornekleme Hizi

Ornekleme hizi formuluu:
```
Sample_Rate = Gyroscope_Output_Rate / (1 + SMPLRT_DIV)
```

DLPF etkin oldugunda (DLPF_CFG != 0): Gyroscope_Output_Rate = 1 kHz
200 Hz icin: SMPLRT_DIV = (1000 / 200) - 1 = 4

### 4.7 FIFO Tampon

MPU6050, 1024 byte FIFO tampon icerir. Bu tampon sensore verilerini biriktirir
ve mikroislemcinin surekli sorgulamasi gereksinimini azaltir. Ancak bu projede
dogrudan register okuma kullanilmaktadir (daha dusuk gecikme icin).

### 4.8 Sicaklik Hesaplama

Sicaklik register'indan (0x41-0x42) okunan ham deger:
```
Sicaklik (C) = (TEMP_OUT / 340.0) + 36.53
```

---

## 5. Accelerometer ile Aci Hesaplama

### 5.1 Temel Prensibi

Ivmeolcer, yercekim ivmesini olcer. Cihaz hareketsiz durumda iken yalnizca
yercekimi kuvveti (g = 9.81 m/s^2) etki eder. Bu kuvvetin X, Y ve Z
eksenlerindeki bilesenleri cihazin egim acisini hesaplamak icin kullanilir.

### 5.2 Roll ve Pitch Hesaplama

**Roll (X ekseni etrafinda donme)**:
```
roll = atan2(Ay, sqrt(Ax^2 + Az^2))
```

**Pitch (Y ekseni etrafinda donme)**:
```
pitch = atan2(-Ax, sqrt(Ay^2 + Az^2))
```

Burada Ax, Ay, Az ivmeolcerden okunan normalize edilmis degerlerdir (g cinsinden).

### 5.3 Alternatif Formul (atan2 ile)

Daha stabil bir hesaplama:
```
roll  = atan2(Ay, Az)  * 180.0 / PI
pitch = atan2(-Ax, sqrt(Ay^2 + Az^2)) * 180.0 / PI
```

### 5.4 Gimbal Lock Problemi

Pitch acisi +/- 90 dereceye yaklastiginda (cihaz dikey konumda) "gimbal lock"
sorunu ortaya cikar:
- sqrt(Ay^2 + Az^2) sifira yaklasir
- atan2 fonksiyonu kararsiz hale gelir
- Roll acisi anlamini yitirir

Bu sorunun cozumleri:
1. Quaternion temsili kullanmak
2. Calismasi beklenen aci araligini sinirlandirmak
3. Kucuk bir epsilon degeri eklemek: sqrt(Ay^2 + Az^2 + epsilon)

### 5.5 Ivmeolcerin Avantajlari ve Dezavantajlari

**Avantajlar**:
- Uzun vadede drift yoktur
- Mutlak aci referansi saglar (yercekimine gore)
- Statik durumlarda dogru sonuc verir

**Dezavantajlar**:
- Titresim ve lineer ivmelenmeye cok duyarli
- Kisa vadede gurultulu
- Dinamik hareketlerde yanlis aci hesaplar
- Yaw acisi hesaplanamaz (manyetometre gerekir)

---

## 6. Gyroscope ile Aci Hesaplama

### 6.1 Temel Prensibi

Jiroskop, acisal hizi (derece/saniye) olcer. Aci elde etmek icin acisal hiz
zamana gore integre edilmelidir:

```
aci(t) = aci(t-1) + gyro_rate * dt
```

### 6.2 Euler Integrasyonu

En basit integrasyon yontemi:
```c
angle += gyro_rate * dt;  /* dt = ornekleme periyodu (saniye) */
```

Bu projede dt = 1/200 = 0.005 saniye (200 Hz ornekleme).

### 6.3 Drift Problemi

Jiroskop verileri uzerinden yapilan integrasyon zamanla "drift" sorununa
yol acar:

- **Bias Drift**: Sifir noktasi kaymasindan kaynaklanir
- **Random Walk**: Beyaz gurultu integrasyonundan kaynaklanir
- **Sicaklik Etkisi**: Sicaklik degisimi bias'i degistirir

Tipik MPU6050 drift degerleri:
- Bias kararsizligi: ~20 dps (power-on, kalibrasyon oncesi)
- Kalibrasyon sonrasi: ~1-5 dps kalan offset
- Gurultu yogunlugu: 0.005 dps/sqrt(Hz)

Drift etkileri:
- 1 dakikada birkacel derecelik hata birikmesi
- 10 dakikada onlarca derecelik hata
- Uzun sureli kullanim icin kabul edilemez

### 6.4 Jiroskopin Avantajlari ve Dezavantajlari

**Avantajlar**:
- Titresime duyarsiz
- Kisa vadede cok hassas
- Hizli tepki suresi
- Tum uc eksende olcum yapabilir (yaw dahil)

**Dezavantajlar**:
- Drift sorunu (uzun vadede hata birikimi)
- Mutlak referans yok
- Bias kalibrasyonu gerektirir
- Sicaklik degisiminden etkilenir

---

## 7. Kalman Filtresi Teorisi

### 7.1 Nedir?

Kalman filtresi, 1960 yilinda Rudolf E. Kalman tarafindan gelistirilen, optimal
durum tahmin algoritmasidir. Gurultulu olcumlerden bir sistemin gercek durumunu
en iyi sekilde tahmin eder.

### 7.2 Neden Kalman Filtresi?

Ivmeolcer ve jiroskop verilerinin birlestirilmesi (sensor fuzyonu) gereklidir:
- Ivmeolcer: Uzun vadede dogru, kisa vadede gurultulu
- Jiroskop: Kisa vadede hassas, uzun vadede driftli

Kalman filtresi bu iki sensorun gucluu yanlarini birlestirerek her iki
sorundan da arinmis bir tahmin uretir.

### 7.3 Durum Vektoru (State Vector)

Bu projede 2 durumlu Kalman filtresi kullanilmaktadir:

```
x = [aci    ]   Durum vektoru
    [bias   ]   (jiroskop bias'i)
```

- **aci**: Tahmin edilen gercek aci (derece)
- **bias**: Jiroskop bias hatasi (derece/saniye)

### 7.4 Sistem Modeli

**Durum Gecis Modeli (State Transition)**:
```
aci(k)  = aci(k-1) + (gyro_rate - bias(k-1)) * dt
bias(k) = bias(k-1)
```

Matris formunda:
```
x(k) = F * x(k-1) + B * u(k)

F = [1  -dt]    B = [dt]    u = gyro_rate
    [0   1 ]        [0 ]
```

**Olcum Modeli (Measurement)**:
```
z(k) = H * x(k) + v(k)

H = [1  0]    z = accelerometer_angle
```

### 7.5 Kovaryans Matrisleri

**P - Hata Kovaryans Matrisi (2x2)**:
```
P = [P00  P01]
    [P10  P11]
```
Tahmin belirsizligini temsil eder. Baslangicta buyuk degerlerle baslatilir.

**Q - Proses Gurultu Kovaryans Matrisi (2x2)**:
```
Q = [Q_angle    0      ]
    [0          Q_bias  ]
```
- Q_angle: Aci proses guruultusuu (tipik: 0.001)
- Q_bias: Bias proses guruultusuu (tipik: 0.003)

Dusuk Q degerleri -> Modele daha cok guven (daha yavas tepki)
Yuksek Q degerleri -> Olcume daha cok guven (daha hizli tepki, daha gurultulu)

**R - Olcum Gurultu Kovaryans (skaler)**:
- R_measure: Ivmeolcer olcum gurultusu (tipik: 0.03)

Dusuk R -> Ivmeolcere daha cok guven (daha hizli tepki, titresime duyarli)
Yuksek R -> Jirosopa daha cok guven (daha yavas tepki, daha puruzsuz)

### 7.6 Kalman Filtresi Adimlari

#### Adim 1: Tahmin (Prediction / Time Update)

```
1a. Durum tahmini:
    aci_tahmin  = aci + (gyro_rate - bias) * dt
    bias_tahmin = bias  (bias sabit varsayilir)

1b. Hata kovaryans tahmini:
    P00 = P00 + dt * (dt*P11 - P01 - P10 + Q_angle)
    P01 = P01 - dt * P11
    P10 = P10 - dt * P11
    P11 = P11 + Q_bias * dt
```

#### Adim 2: Guncelleme (Update / Measurement Update)

```
2a. Inovasyon (olcum hatasi):
    y = accel_angle - aci_tahmin

2b. Inovasyon kovaryansii:
    S = P00 + R_measure

2c. Kalman kazanci (Kalman Gain):
    K0 = P00 / S
    K1 = P10 / S

2d. Durum guncellemesi:
    aci  = aci_tahmin  + K0 * y
    bias = bias_tahmin + K1 * y

2e. Kovaryans guncellemesi:
    P00 = P00 - K0 * P00
    P01 = P01 - K0 * P01
    P10 = P10 - K1 * P00
    P11 = P11 - K1 * P01
```

### 7.7 Kalman Kazanci (Kalman Gain) Yorumu

K degeri 0 ile 1 arasinda degisir:
- **K = 0**: Tamamen modele guven (jiroskop)
- **K = 1**: Tamamen olcume guven (ivmeolcer)
- **K = 0.5**: Her ikisine esit guven

Filtre zamanla optimal K degerini otomatik hesaplar.

### 7.8 Parametre Ayarlama Rehberi

| Parametre  | Dusuk Deger Etkisi        | Yuksek Deger Etkisi       |
|-----------|--------------------------|---------------------------|
| Q_angle   | Yavas tepki, puruzsuz     | Hizli tepki, gurultulu    |
| Q_bias    | Bias yavas duzeltilir     | Bias hizli duzeltilir     |
| R_measure | Ivmeolcere guven artar    | Jirosopa guven artar      |

Onerilir baslangic degerleri:
- Q_angle = 0.001
- Q_bias = 0.003
- R_measure = 0.03

---

## 8. Complementary Filter vs Kalman Filter

### 8.1 Complementary Filter (Tamamlayici Filtre)

En basit sensor fuzyon yontemidir:

```
angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
```

- alpha: Filtre katsayisi (tipik: 0.96 - 0.98)
- Yuksek gecisli filtre (jiroskop) + alcak gecisli filtre (ivmeolcer)

### 8.2 Karsilastirma

| Ozellik               | Complementary Filter   | Kalman Filter          |
|-----------------------|------------------------|------------------------|
| Hesaplama karmasikligi | Dusuk (1 satirlik)     | Orta (matris islemleri)|
| Parametre sayisi      | 1 (alpha)              | 3 (Q_angle, Q_bias, R)|
| Optimal mi?           | Hayir                  | Evet (Gauss gurultu)   |
| Bias tahmini          | Yok                    | Var (durum degiskeni)  |
| Adaptif mi?           | Hayir                  | Evet (kovaryans)       |
| Yaklasma suresi       | Hemen                  | Birkacel iterasyon     |
| CPU kullanimi         | Cok dusuk              | Dusuk                  |
| Hassasiyet            | Iyi                    | Cok iyi                |
| Implementasyon        | Cok kolay              | Orta                   |

### 8.3 Ne Zaman Hangisi?

**Complementary Filter Tercih Edin**:
- CPU kaynaklari cok sinirli
- Hizli prototipleme
- Hassasiyet kritik degil
- Ogrenme amacli

**Kalman Filter Tercih Edin**:
- Yuksek hassasiyet gerekli
- Bias kompanzasyonu onemli
- Degisen gurultu seviyelerine adaptasyon
- Profesyonel uygulamalar

---

## 9. Donanim Baglantilari

### 9.1 Pin Baglanti Semasi

```
STM32F407VG                    MPU6050 Modulu
+---------------+             +---------------+
|               |             |               |
|  PB6 (SCL) ---+---[4.7k]---+--- SCL        |
|               |      |     |               |
|  PB7 (SDA) ---+---[4.7k]---+--- SDA        |
|               |      |     |               |
|  3.3V --------+------+-----+--- VCC        |
|               |             |               |
|  GND ---------+-------------+--- GND        |
|               |             |               |
|  PA2 (TX) ----+--- UART2   |  AD0 --- GND  |
|  PA3 (RX) ----+--- UART2   |  INT --- (NC) |
|               |             |               |
+---------------+             +---------------+

Not: Bircok MPU6050 modulunde pull-up direncler
dahili olarak mevcuttur. Ayrica eklemeniz
gerekmeyebilir.
```

### 9.2 Baglanti Detaylari

| STM32F407VG Pin | Fonksiyon    | MPU6050 Pin | Aciklama              |
|----------------|-------------|-------------|------------------------|
| PB6            | I2C1_SCL    | SCL         | I2C saat hatti         |
| PB7            | I2C1_SDA    | SDA         | I2C veri hatti         |
| 3.3V           | VCC         | VCC         | Guc beslemesi          |
| GND            | GND         | GND         | Referans               |
| PA2            | USART2_TX   | -           | UART veri cikisi       |
| PA3            | USART2_RX   | -           | UART veri girisi       |

### 9.3 Dikkat Edilecek Noktalar

1. **Gerilim Seviyesi**: MPU6050 3.3V ile beslenmelidir. 5V besleme MPU6050'ye
   zarar verebilir. Bircok breakout modulu dahili 3.3V regulator icerir.

2. **Pull-Up Direncler**: I2C hatlari icin 4.7 kOhm pull-up direncler
   gereklidir. Modullerin cogu dahili pull-up icerir.

3. **AD0 Pini**: I2C adresini belirler. GND'ye baglanirsa adres 0x68,
   VCC'ye baglanirsa adres 0x69 olur.

4. **Bypass Kapasitorler**: VCC ile GND arasina 100 nF seramik kapasitor
   eklemek gurultuyu azaltir.

5. **Kablo Uzunlugu**: I2C hatlari mumkun oldugunca kisa tutulmalidir.
   30 cm'den uzun kablolar sorun yaratabilir.

---

## 10. CubeMX Konfigurasyonu

### 10.1 Sistem Clock Konfigurasyonu

```
HSE: 8 MHz (harici kristal)
PLL Carpanlari:
  PLLM = 8
  PLLN = 336
  PLLP = 2
  PLLQ = 7
Sistem Clock: 168 MHz
AHB Prescaler: 1 (HCLK = 168 MHz)
APB1 Prescaler: 4 (PCLK1 = 42 MHz)
APB2 Prescaler: 2 (PCLK2 = 84 MHz)
```

### 10.2 I2C1 Konfigurasyonu

```
Mode: I2C
Speed Mode: Fast Mode
I2C Speed Frequency: 400 kHz
Clock No Stretch Mode: Disabled
Analog Filter: Enabled
SCL Pin: PB6
SDA Pin: PB7
GPIO Pull-Up: Enabled (harici pull-up varsa Internal pull-up devre disi)
```

### 10.3 USART2 Konfigurasyonu

```
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Stop Bits: 1
Parity: None
Hardware Flow Control: None
TX Pin: PA2
RX Pin: PA3
```

### 10.4 TIM7 Konfigurasyonu

200 Hz kesme icin:
```
Prescaler: 8399 (84000000 / 8400 = 10000 Hz - Not: TIM7 APB1'de,
           ancak timer clock = 2 * APB1 = 84 MHz)
Counter Period: 49 (10000 / 50 = 200 Hz)
Auto-Reload Preload: Enabled
Interrupt: Enabled (TIM7 global interrupt)

Gercek hesaplama:
Timer Clock = 84 MHz (APB1 timer clock)
Kesme Frekansi = 84000000 / ((8399+1) * (49+1)) = 84000000 / 420000 = 200 Hz
```

### 10.5 GPIO Konfigurasyonu (Debug LED'leri)

```
PD12: GPIO_Output (Yesil LED - Sistem calisma gostergesi)
PD13: GPIO_Output (Turuncu LED - Veri okuma gostergesi)
PD14: GPIO_Output (Kirmizi LED - Hata gostergesi)
PD15: GPIO_Output (Mavi LED - Kalibrasyon gostergesi)
```

### 10.6 NVIC Konfigurasyonu

```
TIM7 Global Interrupt: Enabled, Preemption Priority = 1
I2C1 Event Interrupt: Enabled, Preemption Priority = 0 (opsiyonel, polling modunda devre disi)
USART2 Global Interrupt: Enabled, Preemption Priority = 2
```

---

## 11. Yazilim Mimarisi

### 11.1 Dosya Yapisi

```
03-I2C-MPU6050-KalmanFilter/
|-- README.md              Bu dosya
|-- inc/
|   |-- mpu6050.h          MPU6050 surucu header
|   |-- kalman_filter.h    Kalman filtre header
|-- src/
|   |-- main.c             Ana uygulama
|   |-- mpu6050.c          MPU6050 surucu implementasyonu
|   |-- kalman_filter.c    Kalman filtre implementasyonu
|-- docs/                  Ek dokumantasyon
```

### 11.2 Veri Akisi

```
MPU6050 Sensor
     |
     | (I2C - 14 byte burst read)
     v
Ham Veri Okuma (mpu6050.c)
     |
     | (Endian donusumu, olceklendirme)
     v
Kalibrasyon Offset Cikarma
     |
     +-------+-------+
     |               |
     v               v
Ivmeolcer        Jiroskop
(g cinsinden)    (dps cinsinden)
     |               |
     v               |
atan2() ile          |
Aci Hesaplama        |
     |               |
     v               v
     +-------+-------+
             |
             v
    Kalman Filtresi (kalman_filter.c)
             |
             v
    Filtrelenmis Roll/Pitch
             |
             v
    UART Cikis (CSV format)
```

### 11.3 Zamanlama

```
TIM7 Kesmesi (200 Hz = 5 ms periyot)
     |
     v
MPU6050 Okuma (~500 us, 14 byte I2C)
     |
     v
Kalman Filtre Hesaplama (~50 us)
     |
     v
UART Gonderme (~200 us, 50 byte)
     |
     v
Toplam: ~750 us < 5000 us (5 ms periyot)
CPU Kullanimi: ~%15
```

---

## 12. Kalibrasyon Proseduru

### 12.1 Neden Kalibrasyon Gerekli?

Her MPU6050 sensorunde uretim surecinden kaynaklanan offset (sifir noktasi
kayma) hatasi vardir. Kalibrasyon bu offset'leri olcer ve kompanze eder.

### 12.2 Kalibrasyon Adimlari

1. **Hazirliks**:
   - Sensoru duz ve titresimden uzak bir yuzeye yerlestirin
   - X ve Y eksenleri yatay, Z ekseni dikey olmali
   - Sensor tamamen hareketsiz olmali

2. **Kalibrasyon Baslatma**:
   - UART uzerinden 'C' komutu gonderin
   - Veya sistem baslangicinda otomatik kalibrasyon

3. **Veri Toplama**:
   - 1000 adet olcum alinir (yaklasik 5 saniye)
   - Her olcumde 6 eksen verisi kaydedilir

4. **Offset Hesaplama**:
   ```
   accel_x_offset = ortalama(accel_x_olcumleri) - 0     // X yatay: 0g beklenir
   accel_y_offset = ortalama(accel_y_olcumleri) - 0     // Y yatay: 0g beklenir
   accel_z_offset = ortalama(accel_z_olcumleri) - 16384 // Z dikey: 1g beklenir
   gyro_x_offset  = ortalama(gyro_x_olcumleri)  - 0     // Hareketsiz: 0 dps
   gyro_y_offset  = ortalama(gyro_y_olcumleri)  - 0     // Hareketsiz: 0 dps
   gyro_z_offset  = ortalama(gyro_z_olcumleri)  - 0     // Hareketsiz: 0 dps
   ```

5. **Dogrulama**:
   - Kalibrasyon sonrasi degerler kontrol edilir
   - Ivmeolcer: X,Y ~ 0g, Z ~ 1g
   - Jiroskop: X,Y,Z ~ 0 dps

### 12.3 Kalibrasyon Kalitesi

Iyi kalibrasyon icin:
- Sensoru hareket ettirmeyin
- Titresim kaynaklarindan uzak tutun
- Oda sicakliginda kalibrasyon yapin
- Duz bir referans yuzey kullanin
- Minimum 500 ornek alin

---

## 13. Test Proseduru

### 13.1 Baglanti Testi

1. WHO_AM_I register'ini okuyun (0x75)
2. Beklenen deger: 0x68
3. Eger farkli bir deger okunuyorsa:
   - I2C baglantilarini kontrol edin
   - Pull-up direnleri kontrol edin
   - AD0 pin durumunu kontrol edin

### 13.2 Statik Test

1. Sensoru duz yuzey uzerine yerlestirin
2. Roll ve Pitch acilari 0 dereceye yakin olmali (+/- 2 derece)
3. Z ekseni ivmeolcer degeri +1g (+/- 0.05g) olmali
4. Jiroskop degerleri 0 dps (+/- 1 dps) olmali
5. En az 1 dakika boyunca drift gozlemleyin

### 13.3 Dinamik Test

1. Sensoru yavasa 90 derece yana yatirin
2. Roll acisinin ~90 dereceye ciktigini dogrulayin
3. Sensoru geri duz konuma getirin
4. Roll acisinin ~0 dereceye dondugunu dogrulayin
5. Hizli hareketlerde asiri sapma (overshoot) olmadigini kontrol edin
6. Ayni testi pitch ekseni icin tekrarlayin

### 13.4 Karsilastirmali Test

1. Kalman filtresi ve complementary filtre cikislarini ayni anda gozlemleyin
2. Her iki filtrenin de benzer sonuclar urettigini dogrulayin
3. Titresim ortaminda Kalman filtresinin daha iyi sonuc verdigini gozlemleyin
4. Uzun sureli testte jiroskop drift'inin her iki filtre ile de
   kompanze edildigini dogrulayin

### 13.5 Self-Test

MPU6050 dahili self-test ozelligi icerir:
1. GYRO_CONFIG ve ACCEL_CONFIG register'larinda self-test bitleri etkinlestirilir
2. Self-test etkin ve devre disi olcumler karsilastirilir
3. Fark, fabrika trim degerleri ile karsilastirilir
4. Belirtilen tolerans icinde ise sensor saglikli

### 13.6 Stres Testi

1. Sensoru hizli ileri-geri sallayarak yuksek donme hizlarini test edin
2. Ani durdurmalarda filtrenin toparlanma suresini olcun
3. Vibrasyon motoru yakinina yerlestirerek titresim dayanikliligi test edin
4. Sicaklik degisimi sirasinda drift davranisini gozlemleyin

---

## 14. UART Komut Arayuzu

### 14.1 Kullanilabilir Komutlar

| Komut | Aciklama                                        |
|-------|--------------------------------------------------|
| C     | Kalibrasyon baslatma                             |
| S     | Self-test calistirma                             |
| R     | Ham veri moduna gecis                            |
| K     | Kalman filtre moduna gecis                       |
| F     | Complementary filtre moduna gecis                |
| D     | Veri cikisi durdurma/baslatma                    |
| Q     | Q_angle degerini artir                           |
| q     | Q_angle degerini azalt                           |
| W     | Q_bias degerini artir                            |
| w     | Q_bias degerini azalt                            |
| E     | R_measure degerini artir                         |
| e     | R_measure degerini azalt                         |
| P     | Guncel parametreleri yazdir                      |
| H     | Yardim menusunu yazdir                           |

### 14.2 Cikis Formati

CSV formatinda veri cikisi (seri terminal veya veri loglama icin):

```
timestamp_ms,roll_kalman,pitch_kalman,roll_comp,pitch_comp,ax,ay,az,gx,gy,gz,temp
```

Ornek:
```
1000,2.34,-1.56,2.31,-1.58,0.01,-0.02,1.00,0.12,-0.08,0.03,25.4
1005,2.35,-1.55,2.32,-1.57,0.01,-0.02,1.00,0.11,-0.09,0.02,25.4
```

---

## 15. Performans Analizi

### 15.1 CPU Kullanimi

| Islem                    | Sure (yaklasik) | Yuzde    |
|--------------------------|-----------------|----------|
| I2C 14 byte okuma        | 500 us          | %10      |
| Kalman filtre (2 eksen)  | 50 us           | %1       |
| Complementary filtre     | 10 us           | %0.2     |
| UART cikis               | 200 us          | %4       |
| Toplam                   | 760 us          | ~%15     |
| Bosta kalma              | 4240 us         | ~%85     |

### 15.2 Bellek Kullanimi

| Bilesin           | RAM (byte) | Flash (byte) |
|-------------------|-----------|--------------|
| MPU6050 surucusu   | 120       | 2048         |
| Kalman filtre (x2) | 80        | 1024         |
| UART tampon        | 256       | 512          |
| Main degiskenleri  | 64        | 4096         |
| **Toplam**         | **~520**  | **~7680**    |

### 15.3 Hassasiyet

| Metrik                    | Kalman Filter | Comp. Filter |
|--------------------------|---------------|--------------|
| Statik hata (RMS)        | < 0.5 derece  | < 1 derece   |
| Dinamik hata (RMS)       | < 2 derece    | < 3 derece   |
| Yaklasma suresi          | ~1 saniye     | Aninda       |
| Drift (10 dk)            | < 0.1 derece  | < 0.5 derece |

---

## 16. Sorun Giderme

### 16.1 I2C Haberlesme Sorunlari

| Sorun                          | Olasi Neden                      | Cozum                           |
|-------------------------------|----------------------------------|----------------------------------|
| WHO_AM_I okunamiyor           | Yanlis baglanti                  | SDA/SCL hatlarini kontrol edin  |
| HAL_TIMEOUT hatasi            | Pull-up direnc eksik             | 4.7k pull-up ekleyin            |
| HAL_BUSY hatasi               | I2C bus kilitlenmis              | Bus recovery uygulayinn         |
| Yanlis veri                   | Endian donusum hatasi            | MSB/LSB sirasini kontrol edin   |
| Aralikli baglanti kopuklugu   | Gevsek kablo                     | Lehim veya breadboard kontrol   |

### 16.2 Sensor Veri Sorunlari

| Sorun                    | Olasi Neden              | Cozum                          |
|-------------------------|--------------------------|--------------------------------|
| Surekli 0 degeri        | Sensor uyku modunda      | PWR_MGMT_1 sleep biti temizle  |
| Dalgali degerler        | Kalibrasyon yapilmamis   | Kalibrasyon rutini calistir    |
| Yanlis aci              | Sensor ters takili       | Eksen yonlerini kontrol et     |
| Asiri drift             | Sicaklik etkisi          | Ortam sicakligi stabilize et   |
| Gurultulu veri          | EMI / titresim           | Kablo kisa tut, filtre ekle    |

### 16.3 I2C Bus Recovery

I2C bus kilitlendiginde (SDA LOW'da kalmis):

```c
/* SCL hattinda 9 clock darbesi gonder */
/* Bu islemi GPIO modunda yapmak gerekir */
1. SCL ve SDA pinlerini GPIO output olarak yapilandir
2. SDA'yi HIGH yap
3. SCL'de 9 clock darbesi uret (her biri ~5 us)
4. STOP kosulu olustur
5. Pinleri tekrar I2C AF moduna al
6. I2C perifer'ini reinit et
```

---

## 17. Kaynaklar

### 17.1 Veri Sayfalari ve Referans Kilavuzlari

1. **MPU6050 Register Map and Descriptions** - InvenSense, Rev 4.2
   - MPU-6000 and MPU-6050 Register Map and Descriptions
   - Tum register adresleri, bit tanimlari ve islevleri

2. **MPU-6000/MPU-6050 Product Specification** - InvenSense, Rev 3.4
   - Elektriksel ozellikler, hassasiyet, gurultu seviyeleri
   - Mutlak maksimum degerler

3. **STM32F407VG Reference Manual** - STMicroelectronics, RM0090
   - I2C perifer detaylari (Bolum 27)
   - Timer detaylari (Bolum 18)

4. **STM32F407VG Datasheet** - STMicroelectronics
   - Pin konfigurasyonu, alternate function tablosu

### 17.2 Teknik Makaleler

5. **An Introduction to the Kalman Filter** - Greg Welch, Gary Bishop
   - University of North Carolina at Chapel Hill
   - Kalman filtresi temel teorisi

6. **A Practical Approach to Kalman Filter and How to Implement It**
   - TKJ Electronics blog
   - MPU6050 icin Kalman filtre implementasyonu

### 17.3 Yararli Araclar

7. **STM32CubeMX** - Konfiguerasyon araci
8. **Serial Plot / SerialPlot** - Seri port uzerinden gercek zamanli grafik
9. **PuTTY / Tera Term** - Seri terminal programlari
10. **Python + Matplotlib** - Veri analizi ve goruntueleme

### 17.4 Ilgili Projeler

- Proje 01: UART-DMA-RingBuffer (UART temelleri)
- Proje 02: PID-Motor-Control (Kontrol teorisi temelleri)

---

## Lisans

Bu proje egitim amaciyla gelistirilmistir. MIT lisansi altinda serbestce
kullanilabilir.

## Yazar

STM32F407VG Gomulu Sistemler Proje Serisi - Proje 3

---

*Bu dokuman, I2C protokolu, MPU6050 sensor kullanimi ve Kalman filtresi
konularinda kapsamli bir referans kaynak olarak hazirlanmistir. Projede
kullanilan tum kod ve konfigurasyonlar detayli olarak aciklanmistir.*
