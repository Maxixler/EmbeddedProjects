/**
 * @file main.c
 * @brief Kesme Servis Rutini Uygulaması PIC16F877A için
 * @details Bu proje, ADC dönüşümü tamamlandı (ADIF) ve Timer0 taşma (TMR0IF) olayları için
 *          kesme servis rutinlerinin (ISR) nasıl uygulanacağını gösterir.
 *          ISR, ilgili kesme bayraklarını temizler ve uygun eylemleri gerçekleştirir.
 *
 *          Temel yönler:
 *          - ADC Kesmesi: ADC dönüşümü tamamlandığında ADIF bayrağı ayarlanır.
 *            ISR, ADC sonucunu okur ve ADIF bayrağını temizler.
 *          - Timer0 Kesmesi: Timer0 taşma olduğunda TMR0IF bayrağı ayarlanır.
 *            ISR, bir LED'i değiştirir ve TMR0IF bayrağını temizler.
 *          - Uygun kesme yapılandırması: GIE, PEIE, ADIE, T0IE bitlerinin ayarlanması.
 *
 *          Bu, PROJECTS.md'deki rehberliği uygular:
 *          * "Kesme Alt Programı: ADC çevrimi bittiğinde (ADIF) veya Timer taştığında (TMR0IF)
 *            bayrakları temizleyen kesme alt programı tasarımları istenecektir."
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

// ADC'yi başlatan fonksiyon
void ADC_Init(void) {
    // ADCON1'i yapılandırır: AN0 analog giriş, diğerleri dijital, Vref = VDD/VSS
    ADCON1 = 0b00001110;       // PCFG3:PCFG0 = 1110

    // ADCON0'ı yapılandırır: Kanal 0 (AN0) seçilir, ADC saatí = Fosc/8
    ADCON0 = 0b00000001;       // CHS2:CHS0 = 000 (AN0), ADCS1:ADCS0 = 00 (Fosc/2) - aslında Fosc/8 kullanacağız
    // Düzeltme: Fosc=8MHz için Tad >= 0.7us gerekir. Fosc/8 = 1us -> Tad = 2us (ACQT=000) kabul edilebilir.
    ADCON0 = 0b00000001;       // ADCS bitlerini aşağıda düzgün ayarlayacağız

    // ADC saatini Fosc/8 olarak ayarlar (ADCS1:ADCS0 = 10)
    ADCON0 &= 0b11110011;      // ADCS1:ADCS0 bitlerini temizler
    ADCON0 |= 0b00001000;      // ADCS1:ADCS0 = 10 (Fosc/8) ayarlar
}

// Timer0'ı periyodik kesme için başlatan fonksiyon
void Timer0_Init(void) {
    // Timer0 yapılandırılır
    OPTION_REGbits.T0CS = 0;   // Timer0 Clock Select = dahili talimat döngüsü saati (Fosc/4)
    OPTION_REGbits.PSA = 0;    // Prescaler Timer0'a atanır
    // Uygun gecikme için prescaler 1:64 (PS2:PS0 = 010) ayarlanır
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 0;

    // Başlangıç değeri (main veya ISR içinde ayarlanacak)
    TMR0 = 0;
}

// Ana fonksiyon
void main() {
    unsigned int adc_sonucu = 0;

    // I/O yapılandırılır
    TRISC0 = 0;                // RC0 çıkış (Timer0 kesmesi için LED)
    TRISA0 = 1;                // RA0/AN0 giriş (analog)
    RC0 = 0;                   // LED başlangıçta kapalı

    // Modülleri başlatır
    ADC_Init();
    Timer0_Init();

    // ADC kesmesini yapılandırır
    PIR1bits.ADIF = 0;         // ADC kesme bayrağını temizler
    PIE1bits.ADIE = 1;         // ADC kesmesini etkinleştirir

    // Timer0 kesmesini yapılandırır
    INTCONbits.T0IF = 0;       // Timer0 kesme bayrağını temizler
    INTCONbits.T0IE = 1;       // Timer0 kesmesini etkinleştirir

    // Küresel ve periferik kesmeleri etkinleştirir
    INTCONbits.PEIE = 1;       // Periferik kesmeleri etkinleştirir
    INTCONbits.GIE = 1;        // Küresel kesmeleri etkinleştirir

    // İlk ADC dönüşümünü başlatır
    ADCON0bits.ADON = 1;       // ADC modülünü açar
    GO_nDONE = 1;              // ADC dönüşümünü başlatır

    while(1) {
        // Ana döngü başka işler yapabilir
        // Gösterim amacıyla sadece gecikme yapıyor ve canlı olduğunu gösteriyoruz
        __delay_ms(100);

        // İsteğe bağlı: ADC sonucunu burada okuyup kullanabilirsiniz
        // (ancak bu değer ISR içinde de mevcuttur)
        // Bu, ana döngünün ISR tarafından ayarlanan değere erişebildiğini gösterir
    }
}

/**
 * @brief Kesme Servis Rutini
 * @details Bu ISR, hem ADC dönüşümü tamamlandı (ADIF) hem de Timer0 taşma (TMR0IF)
 *          kesmelerini işler. İlgili bayrakları temizler ve uygun eylemleri gerçekleştirir:
 *          - ADIF için: ADC sonucunu okur ve bir değişkene saklar
 *          - TMR0IF için: RC0 pindeki LED'i değiştirir ve Timer0'ı yeniden yükler
 */
void interrupt ISR(void) {
    // ADC kesmesini işler
    if (PIR1bits.ADIF && PIE1bits.ADIE) {
        // ADC sonucunu okur (10-bit değer ADRESH:ADRESL)
        unsigned int adc_degeri = (ADRESH << 8) + ADRESL;

        // Sonucu gerektiği şekilde saklar veya işler
        // Gerçek bir uygulamada, ölçekleyebilir, diziye saklayabilirsiniz.
        // Bu örnekte sadece okunduğunu kabul ediyoruz
        // (Gerekirse genel bir değişkene atayabilirsiniz)

        // ADC kesme bayrağını temizler
        PIR1bits.ADIF = 0;

        // Sürekli örnekleme için sonraki ADC dönüşümünü başlatır
        GO_nDONE = 1;
    }

    // Timer0 kesmesini işler
    if (INTCONbits.T0IF && INTCONbits.T0IE) {
        // RC0 pindeki LED'i değiştirir
        RC0 = ~RC0;

        // Timer0 kesme bayrağını temizler
        INTCONbits.T0IF = 0;

        // Timer0'ı sonraki periyod için yeniden yükler (isteğe bağlı, uygulamaya göre değişir)
        // Periyodik kesmeler için yeniden yükleme gerekir
        TMR0 = 100; // Örnek yeniden yükleme değeri: prescaler=64, Fosc=8MHz ile ~10ms
                    // Gerçek periode yüklenen değere bağlıdır
    }

    // Not: Diğer kesme kaynakları gerektiği şekilde buraya eklenebilir
    // (örn. UART, RB port değişikliği, vb.)
}