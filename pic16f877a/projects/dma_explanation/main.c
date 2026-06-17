/**
 * @file main.c
 * @brief Direkt Hafıza Erişimi (DMA) Açıklaması PIC16F877A için
 * @details Bu proje, DMA (Direkt Hafıza Erişimi) kavramını ve onu blok diyagramı açıklamasıyla işletimini açıklar.
 *          Not: PIC16F877A mikrokontrolörü DONANIMSAL DMA MODÜLÜNA SAHİBİ DEĞİLDİR. Bu dosya, DMA teorisini
 *          anlamak ve eğer uygulansaydı nasıl çalışacağını göstermek veya kesmeler ve dikkatli zamanlama kullanarak
 *          DMA benzeri davranışı simüle etmek için eğitim amaçlı hazırlanmıştır.
 *
 *          DMA Blok Diyagramı Açıklaması:
 *          - DMA Kontrolörü: CPU müdahalesi olmadan hafıza transferlerini yönetir
 *          - Adres Registerleri: Kaynak ve hedef adres işaretçileri
 *          - Sayaç Registeri: Transfer edilecek bayt/kelime sayısı
 *          - Kontrol Registeri: Transfer modu, öncelik, kesmeler, vb.
 *          - Veri Tamponu: Transfer edilen veriyi geçici olarak tutan register
 *
 *          Tipik DMA İşlemi:
 *          1. CPU, DMA kontrolörüne kaynak adres, hedef adres, aktarım sayısı ve kontrol ayarlarını programma eder.
 *          2. DMA kontrolörü, CPU'dan otobüs kontrolünü ister (otobüs müktesebeti).
 *          3. Otobüs verildiğinde, DMA kontrolörü transferi gerçekleştirir:
 *             - Kaynak adresinden veri okunur
 *             - Hedef adresine veri yazılır
 *             - Adresler artırılır/azaltılır ve sayaç azaltılır
 *          4. Sayaç sıfıra ulaşıncaya kadar işlem tekrar edilir.
 *          5. DMA kontrolörü, tamamlanışı CPU'ya kesme ile bildirir.
 *
 *          Sınavda sorulabilecek DMA Transfer Türleri:
 *          - Hafıza dan Hafıza'ya
 *          - Hafızadan Çevre Birimine'ye (örn. UART, ADC)
 *          - Çevre Biriminden Hafıza'ya
 *          - Çevre Biriminden Çevre Birimine'e
 *
 *          Transfer Modları:
 *          - Patlamalı Mod: Otobüsü bırakmadan tüm blok transfer gerçekleştirilir
 *          - Otobüs Çalma: Bir bayt/kelime transfer edilir ve otobüs bırakılarak CPU çalıştırabilir
 *          - Şeffaf: Sadece CPU boş cyklileri sırasında transfer gerçekleştirilir
 *
 *          PIC16F877A'da donanımsal DMA olmamasına rağmen, CPU-verimli veri transferleri şu yollarla elde edilebilir:
 *          - Kesmeyle çalışan çevresel modüller (örn. SPI, UART tamponlarıyla)
 *          - Hariç veri transferi için Paralel Escave Portu (PSP) kullanımı (DMA olamasını da)
 *          - Kritik zamanlı işlemler için kesmelerle dikkatli döngü zamanlama
 *
 * @author Claude tarafından oluşturuldu
 * @date 2026-06-07
 */

#include <pic16f877a.h>

// Konfigürasyon bits
#pragma config FOSC = HS        // Yüksek hızlı Osilatör
#pragma config WDTE = OFF       // Watchdog Timer devre dışı
#pragma config PWRTE = OFF      // Power-up Timer devre dışı
#pragma config BOREN = OFF      // Brown-out Reset devre dışı
#pragma config LVP = OFF        // Düşük voltajlı Programlama devre dışı
#pragma config CPD = OFF        // Veri EEPROM Bellek Kod Koruması kapalı
#pragma config WRT = OFF        // Flash Program Bellek Yazma Koruması kapalı
#pragma config CP = OFF         // Flash Program Bellek Kod Koruması kapalı

#define _XTAL_FREQ 8000000     // Gecikme makroları için osilatör frekansını tanımlayın

/**
 * @brief CPU kullanarak bir hafıza bloğu kopyalamasını simüle eder (DMA değil, ancak kavramı gösterir)
 * @details Bu fonksiyon, bir hafıza bloğunu kaynaktan hedefe kopyalar.
 *          Gerçek bir DMA sisteminde, bu transfer CPU müdahalesi olmadan donanım tarafından yapılırdı.
 *          Burada, CPU döngüyü çalıştırır.
 *
 *          Bu işlemi daha DMA benzeri hale getirmek için, kopyalamanın gerektiği zaman tetiklemek
 *          için kesmeler kullanılabilirdi, ancak asıl kopyalama yine CPU tarafından yapılacaktı.
 *
 * @param src  Kaynak hafıza bloğunun pointer'ı
 * @param dst  Hedef hafıza bloğunun pointer'ı
 * @param len  Kopyalanacak bayt sayısı
 */
void memcpy_simulated(unsigned char *src, unsigned char *dst, unsigned int len) {
    while (len--) {
        *dst++ = *src++;
    }
}

/**
 * @brief Kesme kullanarak DMA gibi davranış gösteren UART transfer ayarlama örneği
 * @details Bu fonksiyon, UART'ı kesmeyle asenkron alım için yapılandırır ve
 *          CPU'nun başka işler yapmasına izin verirken baytlarının alınmasını sağlar.
 *          UART modülü gelen veriyi tamponlar (RCREG'de) ve bir bayt alındığında
 *          kesme bayrağını ayarlar. Bu, DMA olmasa da, seri alınmanın zaman-kritik kısmını
 *          donanıma yükler.
 */
void UART_Init_DMA_Style(void) {
    // TX/RX pinleri yapılandırılır
    TRISC6 = 0; // TX pini çıkış olarak
    TRISC7 = 1; // RX pini giriş olarak

    // 9600 baud için SPBRG ayarlanır (Fosc = 8MHz)
    // BRGH = 0, BRG16 = 0 -> SPBRG = ([Fosc/(64*(Baud+1))]-1)
    // SPBRG = ([8000000/(64*97)]-1) = ([8000000/6208]-1) = [128.86]-1 = 127.86 -> 128
    SPBRG = 128;
    BRGH = 0;   // Düşük hız
    BRG16 = 0;  // 8-bit baud rate üreticisi

    // Asenkron seri port etkinleştirilir
    SYNC = 0;
    SPEN = 1;   // Seri port etkinleştirilir

    // Alım etkinleştirilir
    CREN = 1;   // Sürekli alım etkinleştirilir

    // UART alım kesmesi etkinleştirilir
    RCIE = 1;   // UART alım kesmesi etkinleştirilir
    PEIE = 1;   // Periferik kesmeler etkinleştirilir
    GIE = 1;    // Küresel kesmeler etkinleştirilir
}

/**
 * @brief UART alım kesme servis rutini (DMA veri yakalamasını simüle eder)
 * @details UART üzerinden bir bayt alındığında bu ISR çağrılır. ISR,
 *          alınan baytı bir倫 tamponuna depolar. Gerçek bir DMA sisteminde,
 *          donanım baytı CPU müdahalesi olmadan doğrudan belleğe transfer ederdi.
 *          Burada CPU transferini yine halleder, ancak zaman-kritik alım kısmı
 *          UART donanımı tarafından halledilir.
 */
void interrupt ISR(void) {
    static unsigned int rx_index = 0;
    static unsigned char rx_buffer[64]; // Örnek tampon boyutu

    if (PIR1bits.RCIF) { // UART alım kesme bayrağı
        if (rx_index < sizeof(rx_buffer)) {
            rx_buffer[rx_index++] = RCREG; // Alınan baytı depola
        }
        // Tampon overflow olursa, sarabilir veya atabiliriz; basitlik içinde overflow'u görmezden geliyoruz
        PIR1bits.RCIF = 0; // Kesme bayrağı temizlenir
    }
    // Başka kesme kaynakları eklenebilir (örn. Timer0, ADC)
}

// Ana fonksiyon
void main() {
    unsigned char src[16] = "Hello, World!";
    unsigned char dst[16];

    // Kesmeyle çalışan alım (DMA-tipi kavram) için UART'ı başlat
    UART_Init_DMA_Style();

    // CPU-tabanlı hafıza kopyalama örneği (DMA değil)
    memcpy_simulated(src, dst, 16);

    while(1) {
        // Ana döngü: CPU, UART alımı kesmelerle arka planda olurken
        // diğer görevleri yapabilir (gerçek DMA değil, ancak verimli)
        // Örneğin, bir LED blinkleyebilir veya başka veri işleyebilir
        __delay_ms(500);
        // RD0'i LED'e bağlı varsayarak durumunu tersine çevir (PORTD yapılandırılmış olsun)
        // NOT: PORTD pinlerini çıkış yapmanız gerekebilir
        // TRISD = 0x00; // Bu satırı başlangıçta yapabilirsiniz
        // LATD0 = ~LATD0; // RD0'yi toggle
    }
}