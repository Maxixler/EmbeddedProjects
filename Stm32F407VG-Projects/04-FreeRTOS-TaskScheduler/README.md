# Proje 4: FreeRTOS Task Scheduler - STM32F407VG

## Genel Bakis

Bu proje, STM32F407VG mikrodenetleyicisi uzerinde **FreeRTOS** gercek zamanli isletim sistemi (RTOS)
kullanarak coklu gorev (multi-task) tabanli bir **sensor veri toplama ve izleme sistemi** gerceklestirmektedir.
Proje; task yonetimi, kuyruklar (queues), semaforlar (semaphores), mutex'ler, yazilim zamanlayicilari
(software timers) ve olay gruplari (event groups) gibi temel RTOS kavramlarini pratikte gostermektedir.

Sistem, birden fazla sensorden (sicaklik, isik) veri okuyarak bu verileri isler, filtreleyerek ekrana
(UART uzerinden) gonderir, alarm durumlarini yonetir ve sistem sagligini izler. Tum bu islemler
oncelik tabanli preemptive scheduling ile es zamanli olarak yurutulur.

---

## Icerik Tablosu

1. [RTOS Nedir?](#1-rtos-nedir)
2. [Bare-Metal vs RTOS Karsilastirmasi](#2-bare-metal-vs-rtos-karsilastirmasi)
3. [FreeRTOS Mimarisi](#3-freertos-mimarisi)
4. [Task Yasam Dongusu](#4-task-yasam-dongusu)
5. [Task Oncelik Sistemi ve Preemptive Scheduling](#5-task-oncelik-sistemi-ve-preemptive-scheduling)
6. [IPC Mekanizmalari](#6-ipc-mekanizmalari)
7. [Priority Inversion ve Deadlock](#7-priority-inversion-ve-deadlock)
8. [Stack Overflow Algilama](#8-stack-overflow-algilama)
9. [Bellek Yonetimi](#9-bellek-yonetimi)
10. [STM32 + FreeRTOS Entegrasyonu](#10-stm32--freertos-entegrasyonu)
11. [FreeRTOSConfig.h Parametreleri](#11-freertoscconfigh-parametreleri)
12. [CubeMX Konfigurasyon](#12-cubemx-konfigurasyon)
13. [Runtime Statistics Izleme](#13-runtime-statistics-izleme)
14. [Proje Dosya Yapisi](#14-proje-dosya-yapisi)
15. [Sistem Tasarimi](#15-sistem-tasarimi)
16. [Donanim Baglantilari](#16-donanim-baglantilari)
17. [Derleme ve Yukleme](#17-derleme-ve-yukleme)
18. [Test ve Dogrulama](#18-test-ve-dogrulama)
19. [Sorun Giderme](#19-sorun-giderme)
20. [Ileri Duzey Konular](#20-ileri-duzey-konular)
21. [Kaynaklar](#21-kaynaklar)

---

## 1. RTOS Nedir?

**RTOS (Real-Time Operating System)**, gercek zamanli uygulamalar icin tasarlanmis bir isletim
sistemidir. Geleneksel isletim sistemlerinden farki, belirli zaman kisitlamalari icinde gorevleri
tamamlamayi **garanti** etmesidir.

### RTOS'un Temel Ozellikleri

- **Deterministic (Belirlenebilir) Davranis**: Bir gorev ne kadar surede tamamlanacak,
  onceden bilinebilir. Worst-case execution time (WCET) hesaplanabilir.

- **Preemptive Scheduling**: Yuksek oncelikli bir gorev hazir oldugunda, dusuk oncelikli
  gorev kesilir ve CPU yuksek oncelikli goreve verilir.

- **Multitasking**: Birden fazla gorev, CPU zamanini paylasarak es zamanli calisir gibi
  gorunur. Tek cekirdekli islemcilerde bu, hizli context switching ile saglanir.

- **Inter-Task Communication (IPC)**: Gorevler arasi veri paylasimi ve senkronizasyon icin
  kuyruklar, semaforlar, mutex'ler gibi mekanizmalar saglar.

- **Kaynak Yonetimi**: Paylasilan kaynaklara (UART, SPI, I2C vb.) erisimi guvenli sekilde
  yonetir.

### Neden RTOS Kullanilir?

Gomulu sistemlerde RTOS kullanmanin baslica nedenleri:

1. **Karmasik sistemlerde kod organizasyonu**: Her islem bagimsiz bir gorev olarak yazilir.
2. **Zamanlama garantisi**: Kritik gorevlerin zamaninda calistirilmasi saglanir.
3. **Olceklenebilirlik**: Yeni ozellikler kolayca yeni gorevler olarak eklenir.
4. **Bakim kolayligi**: Modular yapi, kodun anlasilmasini ve bakimini kolaylastirir.
5. **Yeniden kullanilabilirlik**: Gorevler bagimsiz moduller olarak baska projelerde kullanilabilir.

### FreeRTOS Hakkinda

FreeRTOS, dunyada en yaygin kullanilan gercek zamanli isletim sistemidir:

- **Acik kaynak** ve ucretsiz (MIT lisansi)
- **35+** mikrodenetleyici mimarisini destekler
- **Kucuk bellek ayak izi**: ~6-12 KB ROM, ~0.5 KB RAM (minimum)
- **Amazon Web Services (AWS)** tarafindan desteklenir
- **MISRA-C** uyumlu kod tabani
- **Sertifikalandirilabilir**: SAFERTOS turevi, IEC 61508, ISO 26262 sertifikali

---

## 2. Bare-Metal vs RTOS Karsilastirmasi

Bare-metal programlama ile RTOS tabanli programlama arasindaki temel farklar:

| Ozellik | Bare-Metal | RTOS (FreeRTOS) |
|---------|-----------|-----------------|
| **Yapi** | Super loop (while(1)) | Gorev tabanli (task-based) |
| **Zamanlama** | Polling veya kesme tabanli | Preemptive scheduler |
| **Determinizm** | Karmasik sistemlerde zor | Oncelik tabanli garanti |
| **Multitasking** | Manuel state machine | Otomatik context switching |
| **IPC** | Global degiskenler, flagler | Queue, Semaphore, Mutex |
| **Zamanlayici** | Timer kesmesi ile manuel | vTaskDelay, Software Timer |
| **Stack** | Tek stack | Her gorev icin ayri stack |
| **Bellek** | Statik, onceden belirlenmis | Dinamik veya statik tahsis |
| **Kod karmasikligi** | Basit sistemlerde dusuk | Her durumda modular |
| **Olceklenebilirlik** | Sinirli | Yuksek |
| **Hata izolasyonu** | Zor | Stack overflow detection |
| **ROM kullanimi** | Minimum | +6-12 KB kernel |
| **RAM kullanimi** | Minimum | +0.5 KB + task stack'leri |
| **Ogrenme egrisi** | Dusuk | Orta |
| **Debug zorlugu** | Basit | RTOS-aware debugger gerekli |
| **Uygun oldugu yer** | Basit, tek islemli sistemler | Karmasik, cok islemli sistemler |

### Bare-Metal Yaklasim (Ornek)

```c
/* Super loop yaklasimi - tum islemler tek dongu icinde */
while (1) {
    if (sensor_timer_flag) {
        read_sensors();           /* 5ms suruyor */
        sensor_timer_flag = 0;
    }
    if (display_timer_flag) {
        update_display();         /* 20ms suruyor */
        display_timer_flag = 0;
    }
    if (uart_data_ready) {
        process_uart_data();      /* 2ms suruyor */
        uart_data_ready = 0;
    }
    /* Problem: display_update sirasinda sensor okumasi gecikebilir! */
}
```

### RTOS Yaklasimi (Ornek)

```c
/* Her islem bagimsiz bir gorev olarak calisir */
void Sensor_Task(void *params) {
    for (;;) {
        read_sensors();
        vTaskDelay(pdMS_TO_TICKS(100));  /* 100ms periyot */
    }
}

void Display_Task(void *params) {
    for (;;) {
        update_display();
        vTaskDelay(pdMS_TO_TICKS(500));  /* 500ms periyot */
    }
}

/* Sensor okumasi ASLA display guncellenmesinden etkilenmez! */
```

---

## 3. FreeRTOS Mimarisi

### 3.1 Scheduler (Zamanlayici)

FreeRTOS scheduler, hangi gorev'in CPU'yu kullanacagina karar verir. STM32'de
**preemptive scheduling** kullanilir:

```
Scheduler Karar Mantigi:
1. En yuksek oncelikli READY gorev sec
2. Ayni oncelikteki gorevler arasinda Round-Robin uygula
3. Hicbir gorev hazir degilse Idle Task calistir
```

Scheduler, su durumlarda devreye girer:
- **SysTick kesmesi** (her tick'te - varsayilan 1ms)
- **taskYIELD()** cagrisi
- **Bir gorev bloke oldugunda** (vTaskDelay, xQueueReceive vb.)
- **Bir gorev baska bir gorevi uyandirdiginda**

### 3.2 TCB (Task Control Block)

Her gorev icin bir TCB yapisi tutulur. Bu yapi su bilgileri icerir:

```
TCB (Task Control Block)
+---------------------------+
| Stack Pointer (pxTopOfStack)
| Gorev Onceligi (uxPriority)
| Stack Baslangic Adresi
| Gorev Adi (pcTaskName)
| Durum Listesi Ogesi
| Olay Listesi Ogesi
| Stack Derinligi
| Mutex Sahiplik Bilgisi
| ...
+---------------------------+
```

### 3.3 Idle Task (Bos Gorev)

Scheduler tarafindan otomatik olusturulur ve en dusuk oncelige (0) sahiptir:

- Silinmis gorevlerin kaynaklarini temizler
- `vApplicationIdleHook()` callback'ini cagirabilir
- CPU'yu dusuk guc moduna almak icin idealdir
- **ASLA bloke olmamalidir**

### 3.4 Tick Interrupt (SysTick)

FreeRTOS'un kalp atisi SysTick kesmesidir:

```
SysTick Kesmesi (her 1ms'de bir)
    |
    +-> xTaskIncrementTick()
    |     |
    |     +-> Gecikme sayaclarini guncelle
    |     +-> Bloke gorevleri kontrol et
    |     +-> Round-robin zamanlama (time slice)
    |
    +-> Context switch gerekli mi?
          |
          +-> Evet: portYIELD_FROM_ISR() -> PendSV
          +-> Hayir: Devam et
```

### 3.5 Cortex-M4 Ozel Kesmeler

FreeRTOS, ARM Cortex-M4 uzerinde uc ozel kesme kullanir:

| Kesme | Oncelik | Gorevi |
|-------|---------|--------|
| **SysTick** | En dusuk | Tick sayaci, zamanlama |
| **PendSV** | En dusuk | Context switching |
| **SVCall** | - | Scheduler baslatma |

> **ONEMLI**: SysTick ve PendSV oncelikleri **en dusuk** olmalidir (STM32'de en yuksek
> sayi = en dusuk oncelik). Bu, diger kesmelerin RTOS tarafindan geciktirilmemesini saglar.

---

## 4. Task Yasam Dongusu

Bir FreeRTOS gorevi su durumlardan birinde bulunur:

```
                    xTaskCreate()
                         |
                         v
                    +---------+
                    |  READY  |<-----------+
                    +---------+            |
                         |                 |
                   Scheduler sec           | Olay/Sure doldu
                         |                 |
                         v                 |
                    +---------+       +---------+
                    | RUNNING |------>| BLOCKED |
                    +---------+       +---------+
                      |    |          (vTaskDelay,
                      |    |          xQueueReceive,
                      |    |          xSemaphoreTake)
                      |    |
                      |    +-------->+-----------+
                      |              | SUSPENDED |
                      |              +-----------+
                      |              (vTaskSuspend)
                      |                   |
                      |    vTaskResume()   |
                      |<------------------+
                      |
                      v
                   vTaskDelete()
                      |
                      v
                  [SILINDI]
```

### Durum Aciklamalari

| Durum | Aciklama | Ornek |
|-------|----------|-------|
| **Ready** | Calisabilir durumda, scheduler'in secmesini bekliyor | Baska yuksek oncelikli gorev calisiyor |
| **Running** | Su anda CPU'yu kullaniyor | Aktif gorev |
| **Blocked** | Bir olay veya sure bekliyor | `vTaskDelay()`, `xQueueReceive()` |
| **Suspended** | Tamamen durdurulmus, sadece `vTaskResume()` ile uyanir | Kullanici tarafindan durdurulmus |

### Onemli Kurallar

1. **Herhangi bir anda sadece bir gorev Running durumundadir** (tek cekirdekli islemcide)
2. **Blocked gorev CPU tuketmez** - scheduler bu gorevi atlar
3. **Suspended gorev hicbir olay ile uyanmaz** - sadece acik `vTaskResume()` cagrisi
4. **Idle task hicbir zaman Blocked veya Suspended olmamalidir**

---

## 5. Task Oncelik Sistemi ve Preemptive Scheduling

### Oncelik Seviyeleri

FreeRTOS'ta oncelikler 0'dan `configMAX_PRIORITIES - 1`'e kadar numaralandirilir:

```
Oncelik 4 (En yuksek): Sensor_Task     - Kritik veri toplama
Oncelik 3            : Processing_Task - Veri isleme
Oncelik 2            : Display_Task    - Ekran guncelleme
Oncelik 1            : Logger_Task     - Log kaydi
Oncelik 0 (En dusuk) : Monitor_Task    - Sistem izleme
                       Idle Task       - Arka plan (otomatik)
```

> **Not**: Bu projede `configMAX_PRIORITIES = 5` olarak ayarlanmistir.

### Preemptive Scheduling Nasil Calisir?

```
Zaman ----->

Oncelik 4: Sensor_Task
           [RUN]                    [RUN]
              |                        |
Oncelik 3:   | Processing_Task        |
              |  [RUN]                 | [RUN]
              |     |                  |    |
Oncelik 2:   |     | Display_Task     |    |
              |     |  [RUN]........   |    | [RUN]
              |     |                  |    |
Oncelik 1:   |     |                  |    |
              |     | Logger_Task      |    |
              |     |    [BLOCKED]     |    |   [RUN]
```

**Aciklama**: Sensor_Task en yuksek oncelikli gorevdir. Hazir oldugunda, diger tum
gorevler kesilir (preempt edilir) ve Sensor_Task calisir. Islemi bitirip bloke
oldugunda, bir sonraki en yuksek oncelikli hazir gorev calisir.

### Time Slicing (Zaman Dilimleme)

Ayni oncelikteki gorevler icin Round-Robin zamanlama kullanilir:

```
configTICK_RATE_HZ = 1000 (1ms tick)

Ayni oncelikteki Task_A ve Task_B:
[Task_A][Task_B][Task_A][Task_B][Task_A]...
  1ms     1ms     1ms     1ms     1ms
```

---

## 6. IPC Mekanizmalari

### 6.1 Queue (Kuyruk)

Gorevler arasi veri aktarimi icin FIFO (First-In-First-Out) tampon:

```c
/* Kuyruk olusturma */
QueueHandle_t xQueue = xQueueCreate(10, sizeof(SensorData_t));

/* Veri gonderme (uretici gorev) */
SensorData_t data = {25.5, 1024};
xQueueSend(xQueue, &data, pdMS_TO_TICKS(100));

/* Veri alma (tuketici gorev) */
SensorData_t received;
xQueueReceive(xQueue, &received, portMAX_DELAY);
```

**Ozellikler**:
- Birden fazla uretici ve tuketici destekler
- Kuyruk doluysa/bossa, gorev otomatik bloke olur
- Kesme icerisinden `xQueueSendFromISR()` kullanilir
- Deger kopyalama ile calisir (referans degil)

**Bu projede kullanim**: Sensor_Task -> (sensorDataQueue) -> Processing_Task -> (displayQueue) -> Display_Task

### 6.2 Binary Semaphore (Ikili Semafor)

Iki gorev/kesme arasinda senkronizasyon icin kullanilir:

```c
/* Semafor olusturma */
SemaphoreHandle_t xSem = xSemaphoreCreateBinary();

/* ISR'dan semafor verme (sinyal gonderme) */
void DMA_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Gorevde semafor bekleme */
xSemaphoreTake(xSem, portMAX_DELAY);  /* DMA tamamlanana kadar bekle */
```

**Bu projede kullanim**: DMA tamamlanma kesmesi -> Binary Semaphore -> Sensor_Task

### 6.3 Counting Semaphore (Sayici Semafor)

Birden fazla kaynak veya olayun sayilmasi icin:

```c
/* 5 adet kaynak icin sayici semafor */
SemaphoreHandle_t xCountSem = xSemaphoreCreateCounting(5, 5);

/* Kaynak al (sayac azalir) */
xSemaphoreTake(xCountSem, portMAX_DELAY);
/* ... kaynak kullan ... */
xSemaphoreGive(xCountSem);  /* Kaynak birak (sayac artar) */
```

### 6.4 Mutex (Mutual Exclusion)

Paylasilan kaynaklara guvenli erisim icin:

```c
/* Mutex olusturma */
SemaphoreHandle_t xMutex = xSemaphoreCreateMutex();

/* Kritik bolge - UART erisimi */
if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    /* UART'a veri gonder - bu bolge korunmus */
    HAL_UART_Transmit(&huart2, data, len, 100);
    xSemaphoreGive(xMutex);
}
```

**Mutex vs Binary Semaphore farklari**:

| Ozellik | Mutex | Binary Semaphore |
|---------|-------|-----------------|
| **Sahiplik** | Var (alan gorev birakmali) | Yok |
| **Priority Inheritance** | Var | Yok |
| **Kullanim amaci** | Karsilikli dislanma | Senkronizasyon |
| **ISR'dan kullanilabilir mi?** | Hayir | Evet |
| **Recursive alma** | Recursive Mutex ile | Hayir |

**Bu projede kullanim**: Display_Task ve Logger_Task ayni UART'i paylasir -> Mutex korumasi

### 6.5 Event Groups (Olay Gruplari)

Birden fazla olayin kombinasyonunu beklemek icin:

```c
/* Olay grubu olusturma */
EventGroupHandle_t xEventGroup = xEventGroupCreate();

/* Bit tanimlari */
#define TEMP_ALARM_BIT    (1 << 0)  /* Bit 0: Sicaklik alarmi */
#define LIGHT_ALARM_BIT   (1 << 1)  /* Bit 1: Isik alarmi */
#define SYSTEM_ERROR_BIT  (1 << 2)  /* Bit 2: Sistem hatasi */

/* Olay bitini set etme */
xEventGroupSetBits(xEventGroup, TEMP_ALARM_BIT);

/* Birden fazla olayi bekleme */
EventBits_t bits = xEventGroupWaitBits(
    xEventGroup,
    TEMP_ALARM_BIT | LIGHT_ALARM_BIT,  /* Beklenecek bitler */
    pdTRUE,                             /* Okuyunca temizle */
    pdFALSE,                            /* ANY bit yeterli (OR) */
    pdMS_TO_TICKS(1000)                 /* Timeout */
);
```

**Bu projede kullanim**: Processing_Task alarm durumlarini set eder, Logger_Task bu olaylari izler.

### 6.6 Task Notifications (Gorev Bildirimleri)

En hizli ve hafif IPC mekanizmasi:

```c
/* Bildirim gonderme */
xTaskNotifyGive(xTaskHandle);

/* Bildirim bekleme */
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

/* Deger ile bildirim */
xTaskNotify(xTaskHandle, value, eSetValueWithOverwrite);
```

**Avantajlar**:
- Ek RAM gerektirmez (TCB icerisindedir)
- Queue'dan ~%45 daha hizli
- Semaphore/Event Group yerine kullanilabilir

**Sinirlamalar**:
- Sadece tek bir gorev alici olabilir
- Tamponlama yok (sadece son deger)

---

## 7. Priority Inversion ve Deadlock

### 7.1 Priority Inversion (Oncelik Terslenmesi)

Dusuk oncelikli gorev mutex tutarken, yuksek oncelikli gorev beklemek zorunda kalir:

```
Problem Senaryosu:

Yuksek (H): ........[BLOCKED - mutex bekliyor]...........[RUN]
Orta   (M): ....[RUN - H'yi engelliyor!]................
Dusuk  (L): [RUN - mutex tutuyor].........[mutex birakti]

L mutex'i tutuyor, H bekliyor, ama M araya girerek L'nin calismasini engelliyor!
H, M'den daha dusuk etkin oncelikle calisiyor = PRIORITY INVERSION
```

**Cozum: Priority Inheritance (Oncelik Mirasi)**

FreeRTOS Mutex'leri otomatik priority inheritance destekler:

```
Cozum:

Yuksek (H): ........[BLOCKED].........[RUN]
Orta   (M): ....[READY - calisamaz!]........[RUN]
Dusuk  (L): [RUN+H onceligi].[mutex birakti]

L, H'nin oncelik seviyesine yukseltilir -> M araya giremez!
```

### 7.2 Deadlock (Kilitlenme)

Iki gorev birbirinin tuttugu mutex'i bekler:

```
Task_A: mutex_1 al -> mutex_2 al (BLOCKED - Task_B tutuyor)
Task_B: mutex_2 al -> mutex_1 al (BLOCKED - Task_A tutuyor)

Her iki gorev de sonsuza kadar bekler = DEADLOCK!
```

**Onleme Stratejileri**:
1. **Mutex'leri her zaman ayni sirada al** (en onemli kural)
2. **Timeout kullan**: `xSemaphoreTake(mutex, pdMS_TO_TICKS(100))` - sonsuza kadar bekleme
3. **Tek mutex kullan**: Mumkunse birden fazla mutex yerine tek mutex
4. **Mutex tutma suresini minimize et**: Kritik bolgeyi kisa tut

---

## 8. Stack Overflow Algilama

Her gorev kendi stack alanina sahiptir. Yanlis boyutlandirma, stack overflow'a neden olur.

### Method 1: Basit Kontrol

```c
#define configCHECK_FOR_STACK_OVERFLOW 1
```
Task context switch sirasinda stack pointer kontrol edilir. Hizli ama her overflow'u yakalayamayabilir.

### Method 2: Pattern Kontrol (Bu projede kullanilan)

```c
#define configCHECK_FOR_STACK_OVERFLOW 2
```
Stack'in son 20 byte'i bilinen bir pattern ile doldurulur. Context switch'te bu pattern kontrol edilir.
Daha guvenilir ama biraz daha yavas.

### Overflow Callback

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    /* KRITIK HATA! Sistemi guvenli duruma getir */
    __disable_irq();
    /* Hata LED'ini yak, UART'a hata mesaji gonder */
    for (;;);  /* Burada kal - sistem guvenli degil */
}
```

### Stack Boyutu Hesaplama

```
Gerekli Stack = Fonksiyon degiskenleri
              + Ic ice fonksiyon cagrilari (call depth)
              + Context save (Cortex-M4: 17 register = 68 byte)
              + Kesme icin ek alan
              + %20-30 guvenlik marji
```

Bu projede stack boyutlari:

| Gorev | Stack (word) | Stack (byte) | Aciklama |
|-------|-------------|-------------|----------|
| Sensor_Task | 256 | 1024 | ADC + DMA islemleri |
| Processing_Task | 512 | 2048 | Filtre + hesaplama |
| Display_Task | 256 | 1024 | UART formatlama |
| Logger_Task | 256 | 1024 | Log formatlama |
| Monitor_Task | 512 | 2048 | Runtime stats |

---

## 9. Bellek Yonetimi

FreeRTOS 5 farkli heap yonetim semasi sunar:

### heap_1.c
- **Sadece tahsis**: `pvPortMalloc()` var, `vPortFree()` yok
- **Fragmentasyon**: Yok
- **Kullanim**: Gorevler bir kez olusturulup silinmiyorsa
- **Avantaj**: En basit, deterministic

### heap_2.c
- **Tahsis ve serbest birakma**: Her ikisi de var
- **Fragmentasyon**: Olabilir (best-fit algoritmasi)
- **Kullanim**: Ayni boyutta bloklar tekrar tekrar tahsis ediliyorsa
- **Avantaj**: Basit, ayni boyut bloklarda iyi

### heap_3.c
- **Standard malloc/free wrapper**: Thread-safe wrapper
- **Fragmentasyon**: C kutuphanesine bagli
- **Kullanim**: Mevcut malloc/free kullanilmak istenirse
- **Avantaj**: Mevcut kodu degistirme gerekmiyor

### heap_4.c (Bu projede kullanilan)
- **Tahsis ve serbest birakma**: Best-fit + bitisik bloklari birlestirme
- **Fragmentasyon**: Minimum (coalescence)
- **Kullanim**: Farkli boyutlarda bloklar tahsis/serbest birakiliyorsa
- **Avantaj**: Genel amacli, en yaygin tercih

### heap_5.c
- **heap_4 + coklu bellek bolgesi**: Ayri RAM bolgeleri kullanilabilir
- **Fragmentasyon**: Minimum
- **Kullanim**: RAM birden fazla adres bolgesi varsa (dahili + harici RAM)
- **Avantaj**: Esnek bellek yerlestirme

### Bellek Yonetimi Karsilastirma Tablosu

| Ozellik | heap_1 | heap_2 | heap_3 | heap_4 | heap_5 |
|---------|--------|--------|--------|--------|--------|
| malloc | Evet | Evet | Evet | Evet | Evet |
| free | Hayir | Evet | Evet | Evet | Evet |
| Coalescence | - | Hayir | - | Evet | Evet |
| Coklu bolge | Hayir | Hayir | Hayir | Hayir | Evet |
| Deterministic | Evet | Kismi | Hayir | Kismi | Kismi |
| Karmasiklik | Dusuk | Dusuk | Dusuk | Orta | Yuksek |

---

## 10. STM32 + FreeRTOS Entegrasyonu

### 10.1 SysTick Yapilandirmasi

STM32F407VG'de SysTick, FreeRTOS tick kaynagi olarak kullanilir:

```
System Clock: 168 MHz
SysTick Clock: 168 MHz (AHB clock)
Tick Rate: 1000 Hz (1ms periyot)
SysTick Reload: 168000000 / 1000 - 1 = 167999
```

### 10.2 Interrupt Priority Yapilandirmasi

STM32F407VG, 4-bit oncelik kullanir (16 seviye, 0-15):

```
NVIC Oncelik Yapisi (4 bit, preemption only):

Oncelik 0-3:  FreeRTOS'un YONETEMEYECEGI kesmeler
              (configMAX_SYSCALL_INTERRUPT_PRIORITY = 5)
              Ornek: Acil guvenlik kesmeleri

Oncelik 4:    *** SINIR ***

Oncelik 5-14: FreeRTOS API kullanabilecek kesmeler
              Ornek: DMA, UART, ADC kesmeleri
              Bu kesmelerde xSemaphoreGiveFromISR() vb. kullanilabilir

Oncelik 15:   SysTick ve PendSV (en dusuk oncelik)
              configKERNEL_INTERRUPT_PRIORITY = 15 << 4
```

> **KRITIK KURAL**: `configMAX_SYSCALL_INTERRUPT_PRIORITY` degerinden DAHA YUKSEK
> (numerik olarak daha dusuk) oncelikli kesmelerde FreeRTOS API fonksiyonlari
> **KESINLIKLE** cagrilmamalidir! Aksi halde sistem coker.

### 10.3 PendSV ve Context Switching

Context switching PendSV kesmesi uzerinden gerceklestirilir:

```
1. SysTick/API -> Context switch gerekli
2. PendSV kesmesi tetiklenir (en dusuk oncelik)
3. Tum bekleyen kesmeler tamamlanir
4. PendSV calisir:
   a. Mevcut gorev register'larini stack'e kaydet
   b. Stack pointer'i TCB'ye kaydet
   c. Yeni gorev'in stack pointer'ini TCB'den yukle
   d. Register'lari stack'ten geri yukle
5. Yeni gorev devam eder
```

### 10.4 SVCall

`vTaskStartScheduler()` cagrildiginda, ilk gorev SVCall kesmesi ile baslatilir.
Bu sadece bir kez, scheduler baslatilirken kullanilir.

### 10.5 FreeRTOS Handler Mapping

STM32'de FreeRTOS handler'lari su sekilde eslenir:

```c
/* stm32f4xx_it.c veya FreeRTOSConfig.h icinde */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
```

---

## 11. FreeRTOSConfig.h Parametreleri

### Temel Parametreler

| Parametre | Deger | Aciklama |
|-----------|-------|----------|
| `configUSE_PREEMPTION` | 1 | Preemptive scheduling aktif |
| `configCPU_CLOCK_HZ` | 168000000 | STM32F407 saat frekansi |
| `configTICK_RATE_HZ` | 1000 | 1ms tick periyodu |
| `configMAX_PRIORITIES` | 5 | 0-4 arasi oncelik seviyeleri |
| `configMINIMAL_STACK_SIZE` | 128 | Minimum stack boyutu (word) |
| `configTOTAL_HEAP_SIZE` | 32768 | 32 KB heap alani |
| `configMAX_TASK_NAME_LEN` | 16 | Gorev adi maksimum uzunlugu |

### Ozellik Parametreleri

| Parametre | Deger | Aciklama |
|-----------|-------|----------|
| `configUSE_MUTEXES` | 1 | Mutex destegi aktif |
| `configUSE_COUNTING_SEMAPHORES` | 1 | Sayici semafor destegi |
| `configUSE_QUEUE_SETS` | 0 | Queue set destegi |
| `configUSE_TASK_NOTIFICATIONS` | 1 | Task notification destegi |
| `configUSE_TIMERS` | 1 | Software timer destegi |
| `configUSE_EVENT_GROUPS` | 1 | Event group destegi |

### Hook Fonksiyonlari

| Parametre | Deger | Aciklama |
|-----------|-------|----------|
| `configUSE_IDLE_HOOK` | 1 | Idle hook fonksiyonu aktif |
| `configUSE_TICK_HOOK` | 0 | Tick hook fonksiyonu pasif |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | Bellek hatasi hook'u aktif |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | Stack overflow Method 2 |

### Runtime Statistics

| Parametre | Deger | Aciklama |
|-----------|-------|----------|
| `configGENERATE_RUN_TIME_STATS` | 1 | CPU kullanim istatistikleri |
| `configUSE_TRACE_FACILITY` | 1 | Trace destegi |
| `configUSE_STATS_FORMATTING_FUNCTIONS` | 1 | vTaskGetRunTimeStats() |

---

## 12. CubeMX Konfigurasyon

STM32CubeMX ile projeyi olusturmak icin asagidaki ayarlari yapin:

### 12.1 RCC Ayarlari

```
High Speed Clock (HSE): Crystal/Ceramic Resonator (8 MHz)
PLL Kaynak: HSE
PLLM: 8
PLLN: 336
PLLP: 2
PLLQ: 7
System Clock: 168 MHz
AHB Prescaler: 1 (168 MHz)
APB1 Prescaler: 4 (42 MHz)
APB2 Prescaler: 2 (84 MHz)
```

### 12.2 GPIO Ayarlari

```
PD12 - GPIO_Output (LED Yesil  - Heartbeat)
PD13 - GPIO_Output (LED Turuncu - Sicaklik Alarmi)
PD14 - GPIO_Output (LED Kirmizi - Isik Alarmi)
PD15 - GPIO_Output (LED Mavi   - Sistem Durumu)
PA0  - GPIO_Input  (Kullanici Butonu - B1)
```

### 12.3 ADC1 Ayarlari

```
ADC1 Channel 0 (PA0): Sicaklik Sensoru (veya potansiyometre)
ADC1 Channel 1 (PA1): Isik Sensoru (LDR)
Resolution: 12-bit
Scan Mode: Enabled
Continuous Conversion: Disabled
DMA: DMA2 Stream 0, Circular mode
End of Conversion: End of sequence
```

### 12.4 USART2 Ayarlari

```
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Stop Bits: 1
Parity: None
Flow Control: None
TX Pin: PA2
RX Pin: PA3
```

### 12.5 TIM5 Ayarlari (Runtime Stats)

```
Clock Source: Internal Clock
Prescaler: 167 (168MHz / 168 = 1MHz -> daha sonra 10kHz icin)
Counter Period: 0xFFFFFFFF (32-bit free-running)
Auto-reload: Enabled
```

> **Not**: TIM5, FreeRTOS runtime istatistikleri icin 10 kHz'de serbest calisan
> bir sayac olarak kullanilir. Bu, tick hizinin (1 kHz) en az 10 kati olmalidir.

### 12.6 NVIC Ayarlari

```
DMA2 Stream 0 (ADC):  Preemption Priority 6
USART2:               Preemption Priority 7
TIM5:                 Preemption Priority 8

SysTick:              Preemption Priority 15 (otomatik)
PendSV:               Preemption Priority 15 (otomatik)
```

### 12.7 FreeRTOS Ayarlari (CubeMX)

CubeMX'te Middleware -> FreeRTOS sekmesinde:
- Interface: CMSIS_V2 (veya native FreeRTOS API)
- Bu projede **native FreeRTOS API** kullanilmaktadir

---

## 13. Runtime Statistics Izleme

### 13.1 CPU Kullanim Olcumu

FreeRTOS, her gorev'in ne kadar CPU zamani kullandigini olcebilir:

```
TIM5 (10 kHz, 100us cozunurluk)
    |
    +-> Her context switch'te: mevcut gorev'in sayaci guncellenir
    |
    +-> vTaskGetRunTimeStats() ile rapor alinir:

        Gorev Adi        Toplam Sure    Yuzde(%)
        ------------------------------------------
        Sensor_Task      12345          8%
        Processing_Task  45678          30%
        Display_Task     23456          15%
        Logger_Task      5678           3%
        Monitor_Task     2345           1%
        IDLE             63498          43%
```

### 13.2 Stack Kullanim Izleme

```c
/* Her gorev'in kullanilmayan stack miktarini ogrenme */
UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
/* Donen deger word cinsindendir (4 byte) */
/* Deger 0'a yaklasiyorsa stack overflow riski var! */
```

### 13.3 Heap Kullanim Izleme

```c
/* Mevcut bos heap miktari */
size_t xFreeHeap = xPortGetFreeHeapSize();

/* Simdiye kadarki minimum bos heap miktari */
size_t xMinFreeHeap = xPortGetMinimumEverFreeHeapSize();
```

### 13.4 Monitor_Task Ciktisi (UART)

```
========== SYSTEM MONITOR ==========
[Uptime: 00:05:23]

--- Stack Usage (words remaining) ---
Sensor_Task:      156 / 256
Processing_Task:  289 / 512
Display_Task:     178 / 256
Logger_Task:      198 / 256
Monitor_Task:     334 / 512

--- Heap Usage ---
Total:   32768 bytes
Free:    18432 bytes (56%)
Min Free: 16384 bytes

--- CPU Usage ---
Sensor_Task:       8.2%
Processing_Task:  12.5%
Display_Task:      5.1%
Logger_Task:       2.3%
Monitor_Task:      1.8%
IDLE:             70.1%

--- Alarms ---
Temperature: NORMAL
Light:       ALARM (Low light detected)
System:      OK
=====================================
```

---

## 14. Proje Dosya Yapisi

```
04-FreeRTOS-TaskScheduler/
|
+-- README.md                    # Bu dokuman
|
+-- inc/
|   +-- FreeRTOSConfig.h        # FreeRTOS yapilandirma dosyasi
|   +-- app_tasks.h             # Gorev tanimlari ve prototipleri
|
+-- src/
|   +-- app_tasks.c             # Gorev implementasyonlari
|   +-- main.c                  # Ana program, cevrebirimleri init
|
+-- Middlewares/
|   +-- FreeRTOS/               # FreeRTOS kaynak dosyalari
|       +-- Source/
|           +-- tasks.c
|           +-- queue.c
|           +-- list.c
|           +-- timers.c
|           +-- event_groups.c
|           +-- stream_buffer.c
|           +-- portable/
|           |   +-- GCC/ARM_CM4F/
|           |       +-- port.c
|           |       +-- portmacro.h
|           +-- MemMang/
|               +-- heap_4.c
|
+-- Drivers/                    # STM32 HAL surculeri (CubeMX tarafindan olusturulur)
    +-- CMSIS/
    +-- STM32F4xx_HAL_Driver/
```

---

## 15. Sistem Tasarimi

### 15.1 Gorev Akis Diyagrami

```
  +------------------+
  |   Sensor_Task    |  (Oncelik 4, 100ms)
  | ADC + DMA okuma  |
  +--------+---------+
           |
           | sensorDataQueue (10 eleman)
           v
  +------------------+
  | Processing_Task  |  (Oncelik 3)
  | Filtre + Alarm   |
  +---+---------+----+
      |         |
      |         | Event Group (alarm bitleri)
      |         v
      |    +-----------+
      |    |Logger_Task|  (Oncelik 1)
      |    | Olay logu |
      |    +-----+-----+
      |          |
      | displayQueue    | (ayni UART mutex)
      v          v
  +------------------+
  | Display_Task     |  (Oncelik 2, 500ms)
  | UART formatla   |----> UART2 (Mutex ile korunmus)
  +------------------+

  +------------------+
  | Monitor_Task     |  (Oncelik 0, 1000ms)
  | Sistem izleme    |----> UART2 (Mutex ile korunmus)
  +------------------+

  +------------------+
  | LED_Timer        |  (Software Timer, 500ms)
  | Heartbeat LED    |
  +------------------+
```

### 15.2 Senkronizasyon Nesneleri

| Nesne | Tur | Kullanim |
|-------|-----|----------|
| sensorDataQueue | Queue (10 eleman) | Sensor -> Processing veri aktarimi |
| displayQueue | Queue (5 eleman) | Processing -> Display veri aktarimi |
| dmaSemaphore | Binary Semaphore | DMA tamamlanma sinyali |
| uartMutex | Mutex | UART erisim korumasi |
| systemEventGroup | Event Group | Alarm ve durum bitleri |

### 15.3 Veri Akisi

```
ADC (CH0, CH1)
    |
    v (DMA transfer)
    |
    +-> DMA ISR: xSemaphoreGiveFromISR(dmaSemaphore)
    |
    v
Sensor_Task: xSemaphoreTake(dmaSemaphore)
    |         Ham ADC degerlerini oku
    |         SensorData_t yapisi doldur
    |
    +-> xQueueSend(sensorDataQueue, &data)
    |
    v
Processing_Task: xQueueReceive(sensorDataQueue)
    |              Moving average filtre uygula
    |              Esik degeri kontrol
    |              Alarm durumu belirle
    |
    +-> xQueueSend(displayQueue, &processed)
    +-> xEventGroupSetBits(alarm bitleri)
    |
    v
Display_Task: xQueueReceive(displayQueue)
    |           xSemaphoreTake(uartMutex)
    |           Formatli UART ciktisi
    |           xSemaphoreGive(uartMutex)
    |
    v
UART2 -> PC (115200 baud)
```

---

## 16. Donanim Baglantilari

### STM32F407VG Discovery Board

```
+------------------------------------------+
|          STM32F407VG Discovery            |
|                                          |
|  LED'ler (dahili):                       |
|    PD12 - Yesil LED  (Heartbeat)         |
|    PD13 - Turuncu LED (Sicaklik Alarmi)  |
|    PD14 - Kirmizi LED (Isik Alarmi)      |
|    PD15 - Mavi LED (Sistem Durumu)       |
|                                          |
|  Buton (dahili):                         |
|    PA0  - User Button (B1)              |
|                                          |
|  ADC Girisleri:                          |
|    PA0  - ADC1_CH0 (Sicaklik/Pot)       |
|    PA1  - ADC1_CH1 (Isik/LDR)          |
|                                          |
|  UART (ST-Link uzerinden):              |
|    PA2  - USART2_TX                     |
|    PA3  - USART2_RX                     |
|                                          |
+------------------------------------------+
```

### Harici Baglanti Semasi

```
Sicaklik Sensoru (veya 10K Pot):
  +3.3V ---[Sensor/Pot]--- PA0 --- GND

Isik Sensoru (LDR):
  +3.3V ---[10K]---+--- PA1
                    |
                  [LDR]
                    |
                   GND

UART Baglantisi (USB-TTL donusturucu):
  PA2 (TX) ---> RX (USB-TTL)
  PA3 (RX) <--- TX (USB-TTL)
  GND      ---- GND
```

> **Not**: Discovery board kullaniliyorsa, UART ST-Link uzerinden zaten baglidir.
> PA2/PA3 pinleri ST-Link Virtual COM Port olarak kullanilir.

---

## 17. Derleme ve Yukleme

### STM32CubeIDE ile

1. File -> New -> STM32 Project
2. MCU: STM32F407VGTx sec
3. Middleware: FreeRTOS ekle
4. Proje dosyalarini kopyala
5. Build Project (Ctrl+B)
6. Debug/Run (F11/F8)

### Makefile ile (GCC ARM)

```bash
# Gerekli araclar
arm-none-eabi-gcc --version
st-flash --version

# Derleme
make all

# Yukleme
make flash

# Debug
make debug
```

### Komut Satiri Derleme Ornegi

```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
    -DSTM32F407xx -DUSE_HAL_DRIVER \
    -Iinc -IDrivers/CMSIS/Include -IDrivers/STM32F4xx_HAL_Driver/Inc \
    -IMiddlewares/FreeRTOS/Source/include \
    -IMiddlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F \
    -Os -Wall -fdata-sections -ffunction-sections \
    -c src/main.c -o build/main.o
```

---

## 18. Test ve Dogrulama

### 18.1 Fonksiyonel Testler

| Test | Yontem | Beklenen Sonuc |
|------|--------|----------------|
| Task olusturma | UART ciktisi | Tum gorevler basariyla olusturuldu |
| Sensor okuma | ADC degerini degistir | Kuyruktan dogru deger alinir |
| Filtre dogrulama | Bilinen giris uygula | Hareketli ortalama dogru hesaplanir |
| Alarm testi | Esik degerini as | LED yanar, log kaydedilir |
| Mutex testi | Eslesik UART erisimi | Karisik karakter olmaz |
| Stack monitor | Tum gorevleri calistir | High water mark pozitif |
| Heap monitor | Uzun sure calistir | Bellek sizintisi yok |

### 18.2 Stres Testleri

1. **Yuksek frekans sensor okuma**: 100ms -> 10ms'ye indir
2. **Kuyruk tasmasi**: Kuyruk boyutunu kucult, davranisi gozle
3. **Mutex yarisi**: Display ve Logger gorevlerini hizlandir
4. **Stack siniri**: Stack boyutlarini kucult, overflow algilama test

### 18.3 UART Terminal Izleme

```
Tera Term veya PuTTY ayarlari:
- Port: COMx (Device Manager'dan kontrol edin)
- Baud Rate: 115200
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Flow Control: None
```

---

## 19. Sorun Giderme

### Sik Karsilasilan Sorunlar

| Sorun | Olasi Neden | Cozum |
|-------|-------------|-------|
| Hard Fault | ISR'da yanlis API | FromISR varyantlarini kullan |
| Hard Fault | Stack overflow | Stack boyutunu artir |
| Gorev calismiyor | Oncelik hatasi | Oncelikleri kontrol et |
| Kuyruk verisi kayip | Kuyruk dolu | Boyutu artir veya timeout ekle |
| UART karisik cikti | Mutex eksik | Tum UART erisimlerini mutex ile koru |
| Sistem donuyor | Deadlock | Mutex alma sirasini duzenle |
| Yanlis CPU istatistigi | Timer hatasi | TIM5 konfigurasyonunu kontrol et |
| malloc basarisiz | Heap yetersiz | configTOTAL_HEAP_SIZE artir |

### Debug Ipuclari

1. **Segger SystemView**: Gercek zamanli gorev izleme araci
2. **FreeRTOS+Trace**: Ticari trace araci (Percepio)
3. **UART Debug**: Her gorev basinda/sonunda mesaj yazdir
4. **LED Debug**: Kritik noktalarda LED toggle
5. **Breakpoint**: RTOS-aware debugger kullan (STM32CubeIDE destekler)

### configASSERT Kullanimi

```c
/* FreeRTOSConfig.h icinde */
#define configASSERT(x) if ((x) == 0) { \
    taskDISABLE_INTERRUPTS(); \
    for (;;); \
}
```

Bu makro, FreeRTOS icerisindeki hatalari yakalamak icin kritiktir. Yanlis
parametre, gecersiz oncelik degeri veya uyumsuz interrupt priority gibi
hatalari aninda tespit eder.

---

## 20. Ileri Duzey Konular

### 20.1 Low Power Entegrasyonu

```c
/* Idle hook ile dusuk guc modu */
void vApplicationIdleHook(void) {
    /* WFI (Wait For Interrupt) komutu */
    __WFI();
    /* Sonraki kesmeye kadar CPU durur, guc tuketimi azalir */
}
```

### 20.2 Task Notification ile Hizli Senkronizasyon

Task notification, binary semaphore'dan ~%45 daha hizlidir:

```c
/* ISR'dan bildirim */
void DMA_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Gorevde bekleme */
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
```

### 20.3 Stream Buffer ve Message Buffer

FreeRTOS v10+ ile gelen yeni IPC mekanizmalari:

```c
/* Tek uretici - tek tuketici senaryolari icin optimize */
StreamBufferHandle_t xStreamBuffer = xStreamBufferCreate(1024, 1);

/* Veri gonderme */
xStreamBufferSend(xStreamBuffer, data, len, portMAX_DELAY);

/* Veri alma */
xStreamBufferReceive(xStreamBuffer, buffer, sizeof(buffer), portMAX_DELAY);
```

### 20.4 MPU (Memory Protection Unit) Destegi

STM32F407'nin MPU'su FreeRTOS ile kullanilabilir:

- Her gorev icin bellek erisim kisitlamalari tanimlanir
- Bir gorev baska bir gorev'in stack alanina yazamaz
- Cekirdek modu ve kullanici modu ayrimi
- `configENABLE_MPU` ile aktiflestirilir

### 20.5 Symmetric Multiprocessing (SMP)

FreeRTOS v11+ ile SMP destegi eklenmistir (STM32H7 gibi cift cekirdekli islemciler icin).
STM32F407 tek cekirdekli oldugu icin bu ozellik burada gecerli degildir, ancak bilgi
amaciyla belirtilmistir.

### 20.6 FreeRTOS + TCP/IP Stack

FreeRTOS+TCP, RTOS ile entegre TCP/IP stack saglar:
- Sifir kopyalama (zero-copy) destegi
- DHCP, DNS, HTTP, MQTT protokolleri
- STM32F407'nin Ethernet MAC birimi ile kullanilabilir

### 20.7 Watchdog Entegrasyonu

```c
/* Her gorev periyodik olarak watchdog'u besler */
void Sensor_Task(void *params) {
    for (;;) {
        /* ... islem ... */
        xEventGroupSetBits(xWdtEventGroup, SENSOR_TASK_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Watchdog gorevi tum bitleri kontrol eder */
void Watchdog_Task(void *params) {
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(xWdtEventGroup,
            ALL_TASK_BITS, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
        if (bits == ALL_TASK_BITS) {
            HAL_IWDG_Refresh(&hiwdg);  /* Tum gorevler saglikli */
        }
        /* Timeout olursa watchdog reset yapar */
    }
}
```

---

## 21. Kaynaklar

### Resmi Dokumanlar
- [FreeRTOS Resmi Sitesi](https://www.freertos.org/)
- [FreeRTOS API Referansi](https://www.freertos.org/a00106.html)
- [STM32F407 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/rm0090.pdf)
- [STM32F407 Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [ARM Cortex-M4 Technical Reference](https://developer.arm.com/documentation/100166/latest)

### Kitaplar
- "Mastering the FreeRTOS Real Time Kernel" - Richard Barry (ucretsiz PDF)
- "The Definitive Guide to ARM Cortex-M3 and Cortex-M4 Processors" - Joseph Yiu
- "Real-Time Operating Systems for ARM Cortex-M Microcontrollers" - Jonathan Valvano

### Video Kaynaklar
- Shawn Hymel - "Introduction to RTOS" (Digi-Key YouTube)
- FastBit Embedded Brain Academy - "Mastering RTOS" (Udemy)
- Controllers Tech - STM32 FreeRTOS Tutorials (YouTube)

### Araclar
- [Segger SystemView](https://www.segger.com/products/development-tools/systemview/) - Gercek zamanli gorev izleme
- [Percepio Tracealyzer](https://percepio.com/tracealyzer/) - RTOS trace analiz araci
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Gelistirme ortami
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) - Konfigurasyon araci

### Topluluk
- [FreeRTOS Forum](https://forums.freertos.org/)
- [ST Community](https://community.st.com/)
- [Stack Overflow - FreeRTOS Tag](https://stackoverflow.com/questions/tagged/freertos)
- [Reddit - r/embedded](https://www.reddit.com/r/embedded/)

---

## Lisans

Bu proje egitim amaclidir. FreeRTOS, MIT lisansi altinda dagitilmaktadir.
STM32 HAL kutuphaneleri, ST Microelectronics lisansi altindadir.

---

**Yazar**: Gomulu Sistemler Portfolyo Projesi
**Tarih**: 2026
**Platform**: STM32F407VG Discovery Board
**RTOS**: FreeRTOS v10.x
**IDE**: STM32CubeIDE
