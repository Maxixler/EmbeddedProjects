# STM32F407VG Embedded Systems Projects

Bu repo, **STM32F407VG Discovery** kartı üzerinde geliştirilen ileri seviye gömülü sistem projelerini içermektedir.

## Donanım Bilgisi

| Özellik | Detay |
|---------|-------|
| MCU | STM32F407VGT6 (ARM Cortex-M4F) |
| Clock | 168 MHz (HSE 8 MHz) |
| Flash | 1 MB |
| SRAM | 192 KB (128 KB + 64 KB CCM) |
| FPU | Single-precision Floating Point Unit |
| DMA | 2x DMA controller, 16 stream |
| ADC | 3x 12-bit ADC (2.4 MSPS) |
| Timer | 14x Timer (Advanced, GP, Basic) |
| UART | 4x USART + 2x UART |
| SPI | 3x SPI |
| I2C | 3x I2C |
| CAN | 2x CAN 2.0B |
| USB | USB OTG FS + HS |

## Projeler

| # | Proje | Konu | Zorluk |
|---|-------|------|--------|
| 01 | [UART-DMA-RingBuffer](./01-UART-DMA-RingBuffer) | UART + DMA + Circular Buffer ile verimli seri haberleşme | Orta-İleri |
| 02 | [PID-Motor-Control](./02-PID-Motor-Control) | PWM ile DC Motor PID Hız Kontrolü + Encoder okuma | İleri |
| 03 | [I2C-MPU6050-KalmanFilter](./03-I2C-MPU6050-KalmanFilter) | MPU6050 IMU okuma + Kalman Filtresi ile açı hesaplama | İleri |
| 04 | [FreeRTOS-TaskScheduler](./04-FreeRTOS-TaskScheduler) | FreeRTOS ile çoklu görev, kuyruk, semafor, mutex | İleri |
| 05 | [CAN-Bus-Network](./05-CAN-Bus-Network) | CAN 2.0B protokolü ile çoklu düğüm haberleşmesi | İleri |
| 06 | [ADC-DMA-FFT](./06-ADC-DMA-FFT) | Yüksek hızlı ADC örnekleme + DMA + FFT frekans analizi | İleri |

## Geliştirme Ortamı

- **IDE:** STM32CubeIDE / Keil MDK-ARM
- **HAL Kütüphanesi:** STM32 HAL Driver
- **Debugger:** ST-Link V2 (Discovery üzerinde mevcut)
- **Seri Terminal:** PuTTY / Tera Term / CoolTerm

## Derleme ve Yükleme

Her proje klasöründe ayrı bir `README.md` dosyası bulunmaktadır. Projeyi derleyip yüklemek için:

1. STM32CubeIDE'de `File > New > STM32 Project` ile yeni proje oluşturun
2. MCU olarak **STM32F407VGTx** seçin
3. İlgili proje klasöründeki `src/` ve `inc/` dosyalarını projenize kopyalayın
4. `.ioc` konfigürasyon ayarlarını her projenin `README.md` dosyasında bulabilirsiniz
5. Projeyi derleyip ST-Link ile karta yükleyin

## Pin Bağlantı Haritası

Her projenin kendi `README.md` dosyasında detaylı pin bağlantı şeması bulunmaktadır.

## Lisans

Bu projeler eğitim amaçlı oluşturulmuştur. MIT Lisansı ile lisanslanmıştır.
