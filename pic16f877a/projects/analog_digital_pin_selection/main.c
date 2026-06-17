/**
 * @file main.c
 * @brief Analog/Dijital Pin Seçimi PIC16F877A için
 * @details Bu proje, PIC16F877A pinlerini TRIS kayıtları ve ADCON1 kaydı (özellikle PCFG3:PCFG0 bitleri)
 *          kullanarak analog veya dijital olarak nasıl yapılandırılacağını gösterir.
 *
 *          Temel Kavramlar:
 *          1. TRIS Kayıtları: Pin yönünü (giriş/çıkış) kontrol eder
 *             - TRISx = 0 -> Pin ÇIKIŞ olarak yapılandırılır
 *             - TRISx = 1 -> Pin GİRİŞ olarak yapılandırılır
 *
 *          2. ADCON1 Kaydı: Bağlantı noktası yapılandırmasını ve voltaj referansını kontrol eder
 *             - PCFG3:PCFG0 bitleri (ADCON1<3:0>) pinlerin analog değilse dijital olmasını belirler
 *             - Farklı PCFG değerleri farklı sayıda analog giriş kanalı ayırır
 *
 *          Yaygın PCFG Yapılandırmaları (ADCON1<3:0>):
 *          - 0000: Tüm 8 kanal (AN0-AN7) analog
 *          - 0001: AN0-AN5 analog, AN6-AN7 dijital
 *          - 0010: AN0-AN3 analog, AN4-AN7 dijital
 *          - 0011: AN0-AN2 analog, AN3-AN7 dijital
 *          - 0100: AN0-AN1 analog, AN2-AN7 dijital
 *          - 0101: AN0 analog, AN1-AN7 dijital
 *          - 0110: Tüm pinler dijital (analog giriş yok)
 *          - 1000: AN0-AN2 analog, AN3-AN7 dijital, Vref- = AN2, Vref+ = AN3
 *          - 1001: AN0-AN2 analog, AN3-AN7 dijital, Vref- = AN2, Vref+ = VDD
 *          - 1010: AN0-AN1 analog, AN2-AN7 dijital, Vref- = AN1, Vref+ = AN2
 *          - 1011: AN0-AN1 analog, AN2-AN7 dijital, Vref- = AN1, Vref+ = VDD
 *          - 1100: AN0 analog, AN1-AN7 dijital, Vref- = AN0, Vref+ = AN1
 *          - 1101: AN0 analog, AN1-AN7 dijital, Vref- = AN0, Vref+ = VDD
 *          - 1110: AN0 analog, AN1-AN7 dijital, Vref- = VSS, Vref+ = VDD (VARSAYILAN)
 *          - 1111: Tüm pinler dijital
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

// Analog okuma için ADC modülünü başlatan fonksiyon
void ADC_Init(void) {
    // ADC aşağıdaki gösterime göre yapılandırılacak
    ADCON0 = 0x00;             // Başlangıçta ADC kapalı
}

// Belirtilen kanaldan (0-7) analog değeri okuma fonksiyonu
unsigned int ADC_Read(unsigned char kanal) {
    if (kanal > 7) return 0; // Geçersiz kanal

    // ADCON0'ı yapılandırır: Kanal ve saat seçimi
    ADCON0 &= 0b11000100;      // Kanal seçimi ve ADON bitlerini temizler
    ADCON0 |= (kanal << 2);    // ADC kanalı seçer (CHS2:CHS0)
    ADCON0 |= 0b00000001;      // ADC modülünü açar (ADON=1)

    // Zakış gecikmesi
    __delay_us(20);

    // Dönüştürmeyi başlatır
    GO_nDONE = 1;

    // Dönüştürmenin tamamlanmasını bekler
    while(GO_nDONE);

    // 10-bit sonucu döndürür
    return ((ADRESH << 8) + ADRESL);
}

// Ana fonksiyon: çeşitli pin yapılandırmalarını gösterir
void main() {
    unsigned int analog_degeri;
    float voltaj;

    // Örnek 1: AN0'u analog giriş olarak yapılandır (varsayılan PCFG=1110 kullanılarak)
    // Bu, gücün alınmasının yapılandırmasıdır
    TRISA0 = 1;                // RA0/AN0 giriş olarak
    // ADCON1 varsayılan olarak 0b00001110 (PCFG3:PCFG0 = 1110)
    // Bu anlamına geliyor: AN0 analog, AN1-AN7 dijital, Vref- = VSS, Vref+ = VDD

    // Örnek 2: AN0-AN3 analog, AN4-AN7 dijital olarak yapılandır (PCFG=0010)
    // Bu yapılandırmayı kullanmak için aşağıdaki satırı açın
    // ADCON1 = 0b00000010;     // PCFG3:PCFG0 = 0010

    // Örnek 3: Tüm pinleri dijital olarak yapılandır (PCFG=0110 veya 1111)
    // Bu yapılandırmayı kullanmak için aşağıdaki satırlardan birini açın
    // ADCON1 = 0b00000110;     // PCFG3:PCFG0 = 0110 - Tüm pinler dijital
    // ADCON1 = 0b00001111;     // PCFG3:PCFG0 = 1111 - Tüm pinler dijital

    // Örnek 4: AN0-AN5 analog, AN6-AN7 dijital olarak yapılandır (PCFG=0001)
    // Bu yapılandırmayı kullanmak için aşağıdaki satırı açın
    // ADCON1 = 0b00000001;     // PCFG3:PCFG0 = 0001

    // ADC'yi başlat
    ADC_Init();

    // PORTB'yi dijital çıkış olarak yapılandırır (LED bar grafiği gösterimi için)
    TRISB = 0x00;              // PORTB çıkış
    PORTB = 0x00;              // Başlangıçta tüm LEDler kapalı

    // PORTC pinlerini analog/dijital karışım gösterimi için yapılandırır
    // RC0'u dijital çıkış, RC1'i dijital giriş olarak kullanalım ve
    // komşu pinler analog olduğu hâlde bunların dijital olarak kullanıldığını gösterelim
    TRISC0 = 0;                // RC0 çıkış (LED)
    TRISC1 = 1;                // RC1 giriş (buton)
    TRISC2 = 0;                // RC2 çıkış (PWM/CCP1 örneği)

    while(1) {
        // AN0'dan analog değeri okur (analog olarak yapılandırılmışsa)
        analog_degeri = ADC_Read(0); // Kanal 0 (AN0) okunur

        // Voltaja çevirir (Vref = 5.0V varsayılır)
        voltaj = (analog_degeri * 5.0) / 1023.0;

        // PORTB'de analog okuma basit LED bar grafiği
        // 0-1023 aralığını 0-8 LED'e eşler
        unsigned char led_seviyesi = analog_degeri / 128; // 1024/8 = 128
        if (led_seviyesi > 8) led_seviyesi = 8;
        PORTB = (1 << led_seviyesi) - 1; // LED'leri 0'dan led_seviyesi-1'e kadar yakar

        // PORTC'de dijital I/O gösterimi
        // RC0 LED'i analog değere göre blinkler (daha yüksek voltaj için daha hızlı)
        RC0 = 1;               // RC0 LED'ini yakar
        __delay_ms(100 + (analog_degeri / 10)); // Analog değere orantılı gecikme
        RC0 = 0;               // RC0 LED'ini söndürür
        __delay_ms(100 + (analog_degeri / 10));

        // RC1'deki dijital girişi okur (buton)
        // Bir butonun RC1 ile zemin arasında bağlandığını varsayalım
        if (RC1 == 0) {        // Buton basılmış (aktif düşük)
            // Buton basıldığında RC2'yi değiştir
            RC2 = ~RC2;
            __delay_ms(200);   // Debouncing gecikmesi
        }

        // Okumadan önce kısa bir gecikme
        __delay_ms(10);
    }
}