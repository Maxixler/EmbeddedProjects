/**
 * @file main.c
 * @brief PIC16F877A için CCP Yakalama Modu ile Frekans Ölçümü
 * @details Bu proje, CCP1 modülünün yakalama modunda kullanılarak frekans ölçümünü gösterir.
 *          CCP1 modülü, RC2/CCP1 pine uygulanan giriş sinyalinin her yükselen kenarında
 *          Timer1 değerini yakalar. İki ardışık yükselen kenar arasındaki süreyi (periyot)
 *          ölçerek frekansı f = 1/T formülüyle hesaplayabiliriz.
 *
 *          PROJECTS.md'deki formül:
 *          Periyot(T) = t2 - t1
 *          Frekans f = 1/T
 *          Elde edilen periyot değeri komut işleme süresi (Tcy) ile çarpılarak
 *          gerçek zamana çevrilmelidir.
 *
 *          Tcy = 4 * Tosc = 4/Fosc (komut devre zamanı)
 *
 *          Adımlar:
 *          1. Timer1'i dahili saat olarak yapılandır (referans saatimiz olur)
 *          2. CCP1 modülünü her yükselen kenarda yakalama modunda yapılandır
 *          3. İlk yakalama sırasında Timer1 değerini t1 olarak sakla
 *          4. İkinci yakalama sırasında Timer1 değerini t2 olarak sakla ve periyot hesapla
 *          5. Periyot = (t2 - t1) * Tcy
 *          6. Frekans = 1 / Periyot
 *
 * @author Claude tarafından üretildi
 * @date 2026-06-07
 */

#include <pic16f877a.h>

// Configuration bits
#pragma config FOSC = HS        // Yüksek hızlı Osilatör
#pragma config WDTE = OFF       // Watchdog Timer devre dışı
#pragma config PWRTE = OFF      // Power-up Timer devre dışı
#pragma config BOREN = OFF      // Brown-out Reset devre dışı
#pragma config LVP = OFF        // Düşük voltajlı Programlama devre dışı
#pragma config CPD = OFF        // Veri EEPROM Bellek Kod Koruması kapalı
#pragma config WRT = OFF        // Flash Program Belleği Yazma Koruması kapalı
#pragma config CP = OFF         // Flash Program Belleği Kod Koruması kapalı

#define _XTAL_FREQ 8000000     // Gecikme makroları için osilatör frekansı tanımla

// Timer1'i dahili saat (dahili saat kaynağı) olarak başlatan fonksiyon
void Timer1_Init(void) {
    // Timer1'i dahili saat kaynağı olarak yapılandır
    T1CONbits.TMR1CS = 0;      // Timer1 Saat Seçimi = dahili saat (Fosc/4)
    T1CONbits.T1SYNC = 0;      // Harici saat girişi senkronizasyonu (dahili saat için ilgisiz)
    T1CONbits.T1CKPS0 = 0;     // Öölçekleyici 1:1
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.TMR1ON = 1;      // Timer1'i etkinleştir

    // Timer1 registrelerini temizle
    TMR1H = 0;
    TMR1L = 0;
}

// CCP1 modülünü yakalama modunda başlatan fonksiyon
void CCP1_Capture_Init(void) {
    // RC2/CCP1 pine giriş olarak yapılandır
    TRISC2 = 1;                // RC2 giriş

    // CCP1 modülünü her yükselen kenarda yakalama modu için yapılandır
    CCP1M3 = 0;
    CCP1M2 = 0;
    CCP1M1 = 1;
    CCP1M0 = 1;                // 0b0001 = Yakalama modu, her yükselen kenar

    // CCP1 kesme bayrağını temizle
    PIR1bits.CCP1IF = 0;

    // CCP1 kesmesini etkinleştir
    PIE1bits.CCP1IE = 1;
}

// Ana fonksiyon
void main() {
    unsigned int t1 = 0, t2 = 0;
    unsigned int period_ticks = 0;
    float period_seconds = 0;
    float frequency = 0;
    unsigned char capture_count = 0;

    // G/Ç yapılandır
    TRISB = 0x00;              // PORTB çıkış (hata ayıklama/görüntüleme için)
    PORTB = 0x00;

    // Modülleri başlat
    Timer1_Init();
    CCP1_Capture_Init();

    // Genel ve çevresel kesmeleri etkinleştir
    INTCONbits.PEIE = 1;       // Çevresel kesmeleri etkinleştir
    INTCONbits.GIE = 1;        // Genel kesmeleri etkinleştir

    while(1) {
        // Ana döngü diğer görevler yapabilir
        // Örneğin, LCD'de frekansı göster veya UART ile gönder
        // Şimdilik PORTB'de bir aktivite gösterelim
        PORTB = 0xFF;          // PORTB'deki tüm LED'leri kısa süre 동안 yak
        __delay_ms(100);
        PORTB = 0x00;          // PORTB'daki tüm LED'leri söndür
        __delay_ms(900);
    }
}

/**
 * @brief Kesme Servis Rutini
 * @details Bu ISR, CCP1 yakalama kesmesini işler. Giriş sinyalinin her steigende kenarında,
 *          Timer1 değerini yakalar. İlk yakalama t1'i, ikinci yakalama t2'yi saklar.
 *          İki yakalama tamamlandıktan sonra periyot ve frekans hesaplanır.
 */
void interrupt ISR(void) {
    if (PIR1bits.CCP1IF && PIE1bits.CCP1IE) { // CCP1 kesme bayrağı ayarlandı mı kontrol et
        // Yakalanan Timer1 değerini oku (CCPR1H:CCP1R)
        unsigned int captured_time = (CCPR1H << 8) | CCPR1L;

        if (capture_count == 0) {
            // İlk yakalama - t1'i sakla
            t1 = captured_time;
            capture_count = 1;
        } else if (capture_count == 1) {
            // İkinci yakalama - t2'yi sakla ve hesapla
            t2 = captured_time;

            // Timer1 tiks cinsinden periyot hesapla
            // Timer1 overflow durumu için kontrol et
            if (t2 >= t1) {
                period_ticks = t2 - t1;
            } else {
                // Yakalamalar arasında Timer1 overflow oluştu
                period_ticks = (0xFFFF - t1) + t2 + 1;
            }

            // Periyot tiks'ını saniyeye çevir
            // Tcy = 4 * Tosc = 4/Fosc (komut devre zamanı)
            // Saniye cinsinden periyot = period_ticks * Tcy
            period_seconds = period_ticks * (4.0 / _XTAL_FREQ);

            // Frekansı hesapla
            if (period_seconds > 0) {
                frequency = 1.0 / period_seconds;
            } else {
                frequency = 0; // Sıfıra bölmeyi önle
            }

            // İsteğe bağlı: frekans değerini burada kullan
            // Örneğin, LCD'de göster, UART ile gönder vb.
            // Gösterim amacıyla, frekansa göre bir LED yanıp söndür
            // (Bu sadece örnek kod - gerekçeye göre ayarla)

            // Yeni ölçüm için sıfırla
            capture_count = 0;
            // Seçenekle, yeni ölçümü hemen başlatmak için
            // capture_count = 1 ve t1 = t2 ayarlayabiliyoruz
        }

        // CCP1 kesme bayrağını temizle
        PIR1bits.CCP1IF = 0;
    }
    // Gerekiyorsa diğer kesme kaynaklarını ekle
}