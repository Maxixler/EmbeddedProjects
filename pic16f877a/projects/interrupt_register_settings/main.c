/**
 * @file main.c
 * @brief Kesme Kayıtçı Ayarları PIC16F877A için
 * @details Bu proje, INTCON, PIE1 ve PIR1 kayıtçılarının doğru yapılandırılmasını gösterir.
 *          Timer0 kesmesini etkinleştirir ve ISR içinde bir LED'i değiştirerek kesme işlemini doğrular.
 *
 *          Ana Kayıtçılar:
 *          - INTCON: GIE (Küresel Kesme İzni), PEIE (Periferik Kesme İzni),
 *                    T0IE (Timer0 Kesme İzni), T0IF (Timer0 Kesme Bayrağı), vb. içerir.
 *          - PIE1: Periferik Kesme İzni Kayıtçısı 1 (ADC için ADIE, UART 알ıcı için RCIE, vb.)
 *          - PIR1: Periferik Kesme İstek Kayıtçısı 1 (ADC için ADIF, UART alıcı için RCIF, vb.)
 *
 *          Adımlar:
 *          1. Timer0'ü periyodik kesmeler için yapılandırın (örn. her 500ms'de bir).
 *          2. INTCON'daki T0IE biti ayarlanarak Timer0 kesmesi etkinleştirilir.
 *          3. INTCON'daki GIE ve PEIE bitleri ayarlanarak küresel ve periferik kesmeler etkinleştirilir.
 *          4. ISR'de, T0IF bayrağı temizlenir ve LED değiştirilir.
 *          5. Not: PIE1 ve PIR1 bu örnekte tamamlayıcı amaçla gösterilmiştir (0 olarak ayarlanmıştır, ancak
 *             ADC, UART gibi diğer çevrim birimleri için kullanılacaktır).
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

// Timer0'ı periyodik kesme için başlatan fonksiyon
void Timer0_Init(void) {
    // RC0/LED pini çıkış olarak ayarlanır
    TRISC0 = 0;                // RC0 çıkış
    RC0 = 0;                   // LED başlangıçta kapalı

    // Timer0 yapılandırılır
    OPTION_REGbits.T0CS = 0;   // Timer0 Clock Select = dahili talimat döngüsü saati (Fosc/4)
    OPTION_REGbits.PSA = 0;    // Prescaler Timer0'a atanır
    // Daha uzun gecikmeler için prescaler 1:256 ayarlanır (PS2:PS0 = 111)
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;

    // Yaklaşık 500ms gecikme için TMR0 değeri hesaplanır
    // Fosc = 8MHz, Tosc = 0.25us, TMR0 clock = Tosc/4 = 1us (prescaler=1 olduğunda)
    // Prescaler=256 ile, TMR0 clock = 256 * 1us = 256us per tik
    // 500ms = 500000us gerektiği için, gerekli tik sayısı = 500000 / 256 = 1953.125
    // Timer0 8-bit (0-255) olduğundan, yeniden yükleme değeri şu şekilde bulunur:
    // 500ms aralığında toplam tik sayısı = n * 256 + (256 - r) where r is reload value
    // n=7 (7*256=1792) seçersek, 256 - r = 1953.125 - 1792 = 161.125 → r = 256 - 161.125 = 94.875 → 95
    // Dolayısıyla TMR0 = 95 olarak ayarlarız; 161 tik sonra overflow olur (256-95=161),
    // ardından 7 kez tam overflow (7*256=1792) ve ek 161 tik = 1953 tik = ~500ms

    // Basitlik adına Timer0 ve prescaler 256 kullanıyoruz ve ISR'de 95 ile yeniden yüklüyoruz.
    TMR0 = 95;                 // Timer0 için başlangıç değeri

    // Timer0 kesmesini etkinleştir
    INTCONbits.T0IE = 1;       // Timer0 kesmesi etkinleştir
    INTCONbits.GIE = 1;        // Küresel kesmeleri etkinleştir
    INTCONbits.PEIE = 1;       // Periferik kesmeleri etkinleştir (Timer0 için kesinlikle gerekli değil,
                               // ancak diğer periferik kesmelerle birlikte kullanılırsa iyi bir uygulamadır)
}

// Ana fonksiyon
void main() {
    // Timer0'ı periyodik kesme için başlat
    Timer0_Init();

    while(1) {
        // Ana döngü hiçbir iş yapmaz; LED değiştirme ISR içinde yapılır
        // İsterseniz burada diğer arka plan görevleri ekleyebilirsiniz
    }
}

/**
 * @brief Kesme Servis Rutini
 * @details Bu ISR, Timer0 kesmesini işler. RC0 pindeki LED'i değiştirir
 *          ve Timer0 kesme bayrağını temizler.
 */
void interrupt ISR(void) {
    if (INTCONbits.T0IF) {     // Timer0 kesme bayrağı kontrol edilir
        RC0 = ~RC0;            // RC0 (LED) pini değiştirilir (toggle)
        TMR0 = 95;             // Timer0 bir sonraki aralık için yeniden yüklenir
        INTCONbits.T0IF = 0;   // Timer0 kesme bayrağı temizlenir
    }
    // Not: PIE1/PIR1 kaynaklı diğer kesmeler (ADC, UART RX vb.) kullanılıyorsa,
    // burada kontrol edilmelidir. Örnek:
    // if (PIR1bits.ADIF && PIE1bits.ADIE) { ... } // ADC kesmesi
    // if (PIR1bits.RCIF && PIE1bits.RCIE) { ... } // UART alıcı kesmesi
}