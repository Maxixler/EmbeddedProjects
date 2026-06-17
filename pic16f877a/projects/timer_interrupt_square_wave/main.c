/**
 * @file main.c
 * @brief Timer Kesmesi Kare Dalga Üretimi PIC16F877A için
 * @details Bu proje, Timer0 kesmesi kullanarak kare dalga üretilmesini gösterir.
 *          Çıkış pini (örneğin, RC0) Timer0 ISR'sinde değiştirilerek istenen frekansda
 *          bir kare dalga oluşturulur.
 *
 *          Kullanılan Formül (PROJECTS.md'den):
 *          TMRx_reg_degeri = Timer_max - (Gecikme * Fosc) / (Prescale * 4)
 *
 *          Timer0 (8-bit) için: Timer_max = 256
 *          Timer1 (16-bit) için: Timer_max = 65536
 *
 *          Örnek: 1kHz kare dalga üretmek için (periyot = 1ms, her 0.5ms'te bir değiştirme)
 *          Fosc = 8MHz ve prescaler = 32 ile:
 *          TMR0_değeri = 256 - (0.0005 * 8000000) / (4 * 32)
 *                     = 256 - (4000) / (128)
 *                     = 256 - 31.25 = 224.75 -> 225 (yaklaşık)
 *
 *          Not: Basitlik için Timer0 kullanıyoruz, ancak daha uzun gecikmeler için Timer1 daha iyi olur.
 *          Bu örnek, Timer0 ile prescaler 32 kullanarak ~0.5ms değiştirme periyodu oluşturur.
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

// Timer0'ı kare dalga üretimi için başlatan fonksiyon
void Timer0_Init(void) {
    // RC0 pini (kare dalga çıkışı için) çıkış olarak ayarlanır
    TRISC0 = 0;                // RC0 çıkış
    RC0 = 0;                   // Düşük seviyede başlat

    // Timer0 yapılandırılır
    OPTION_REGbits.T0CS = 0;   // Timer0 Clock Select = dahili talimat döngüsü saati (Fosc/4)
    OPTION_REGbits.PSA = 0;    // Prescaler Timer0'a atanır
    OPTION_REGbits.PS2 = 0;    // Prescaler oranı seçme bitleri
    OPTION_REGbits.PS1 = 0;    // 000 = 1:2
    OPTION_REGbits.PS0 = 1;    // Aslında, daha iyi gecikme için 1:32 kullanalım: PS2:PS0 = 100 (1:32)
    // Düzeltme: 1:32 için, PS2=1, PS1=0, PS0=0
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;

    // 0.5ms delay için TMR0 değeri hesaplanır (1kHz kare dalga için yarı periyot)
    // TMR0_değeri = 256 - (gecikme * Fosc) / (4 * prescaler)
    // gecikme = 0.0005s, Fosc = 8000000, prescaler = 32
    // TMR0_değeri = 256 - (0.0005 * 8000000) / (4 * 32)
    //            = 256 - (4000) / (128)
    //            = 256 - 31.25 = 224.75 -> 225
    TMR0 = 225;                // Timer0 için başlangıç değeri

    // Timer0 kesmesini etkinleştir
    INTCONbits.T0IE = 1;       // Timer0 kesmesi etkinleştir
    INTCONbits.GIE = 1;        // Küresel kesmeleri etkinleştir
    INTCONbits.PEIE = 1;       // Periferik kesmeleri etkinleştir (T0 için kesinlikle gerekli değil ama iyi uygulama)
}

// Timer0 kesme servis rutini
void interrupt ISR(void) {
    if (INTCONbits.T0IF) {     // Timer0 kesme bayrağı kontrol edilir
        RC0 = ~RC0;            // RC0 pini değiştirilir (toggle)
        TMR0 = 225;            // Timer0 bir sonraki aralık için yeniden yüklenir
        INTCONbits.T0IF = 0;   // Timer0 kesme bayrağı temizlenir
    }
}

// Ana fonksiyon
void main() {
    // Timer0'ı kare dalga üretimi için başlat
    Timer0_Init();

    while(1) {
        // Ana döngü - burada başka bir şey yapılması gerekmez, kare dalga kesme tarafından üretilir
        // İsterseniz burada diğer görevler ekleyebilirsiniz
    }
}