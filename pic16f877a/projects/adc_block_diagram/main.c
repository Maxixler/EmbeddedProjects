/**
 * @file main.c
 * @brief ADC Blok Diyagramı Açıklaması ve Çözünürlük Hesabı PIC16F877A için
 * @details Bu proje, ADC modülünün işletimini şu adımlarla gösterir:
 *          1. ADC modülünün yapılandırılması (ADCON0, ADCON1 registers)
 *          2. AN0 pininden analog giriş okunması
 *          3. ADC okunmasının gerçek voltaja dönüştürülmesi
 *          4. ADC blok diyagramı kavramlarının açıklaması: örnekleme, nicemleme, vb.
 *
 *          ADC Blok Diyagramı Açıklaması:
 *          - Örnekleme ve Tutma: Analog giriş gerilimini yakalar ve tutar
 *          - Nicemleme: Tutulan gerilimi, çözünürlük bit sayısı (n) bilgisiyle 2^n diskrete seviyeye dönüştürür
 *          - Kodlama: Nicemleme seviyesini temsil eden ikili kodu çıkarır
 *
 *          Kayıt Etkileri:
 *          - ADCON0: ADC işlemini kontrol eder (ADC AÇIK/KAPALI, kanal seçimi, dönüştürme başlatma)
 *          - ADCON1: Bağlantı noktalarını analog/dijital olarak yapılandırır, voltaj referans seçimi
 *
 *          Çözünürlük Hesabı:
 *          Çözünürlük (bit) = log2(diskret seviye sayısı)
 *          PIC16F877A için: 10-bit ADC -> 1024 diskret seviye (0 ile 1023 arası)
 *
 *          Voltaj Hesabı:
 *          Ölçülen Voltaj = (ADC_Okuma * Vref) / 1023
 *          Burada Vref referans voltajıdır (genellikle VDD veya harici referans)
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

// LCD bağlantıları (LCD kullanılıyorsa)
// LCD'ye bağlıysanız aşağıdaki satırları açın ve ayarlayın
// #define LCD_RS RD0
// #define LCD_EN RD1
// #define LCD_D4 RD2
// #define LCD_D5 RD3
// #define LCD_D6 RD4
// #define LCD_D7 RD5

// ADC modülünü başlatan fonksiyon
void ADC_Init(void) {
    // ADCON1'i yapılandırır: AN0 analog giriş olarak, diğerleri dijital, Vref = VDD/VSS
    // PCFG3:PCFG0 = 1110 -> AN0 analog, AN1-AN7 dijital, Vref+ = VDD, Vref- = VSS
    ADCON1 = 0b00001110;       // AN0 analog, diğerleri dijital, Vref = VDD/VSS

    // ADCON0'ı yapılandırır: Kanal 0 (AN0) seçilir, ADC başlangıçta kapalı
    // ADCS1:ADCS0 = 10 -> Tad için Fosc/8 (Tad'en 0.7us olduğu gerekli)
    // Fosc = 8MHz ile, Fosc/8 = 1us -> Tad = 1us (geçerli)
    ADCON0 = 0b00000001;       // Kanal 0 (AN0), ADON=0 şimdilik

    // İsteğe bağlı: Zakış zamanı seçimi
    // ACQT2:ACQT0 = 001 -> 2 Tad zakış zamanı
}

// Belirtilen kanaldan ADC değeri okuma fonksiyonu
unsigned int ADC_Read(unsigned char kanal) {
    if (kanal > 7) return 0; // Geçersiz kanal

    // ADC kanalı seçilir
    ADCON0 &= 0b11000100;      // Kanal seçimi bitleri temizlenir
    ADCON0 |= (kanal << 2);    // İstenen kanal ayarlanır

    // Dönüştürme öncesi zakış gerekli olduğu için pequeños bir gecikme eklenir
    __delay_us(20);            // Zakış için küçük delay

    // Dönüştürme başlatılır
    GO_nDONE = 1;              // ADC dönüştürme başlat

    // Dönüştürmenin tamamlanmasını bekler
    while(GO_nDONE);           // GO_nDONE 0 olana kadar bekle

    // 10-bit sonuç döndürülür
    return ((ADRESH << 8) + ADRESL);
}

// ADC okumasını voltaja dönüştürme fonksiyonu
float ADC_To_Voltage(unsigned int adc_degeri, float vref) {
    // Formül: Voltaj = (ADC_Değeri * Vref) / 1023
    // 1023 kullanılır çünkü 10-bit ADC 0-1023 arasındaki değerleri verir (1024 adım)
    return (adc_degeri * vref) / 1023.0;
}

// İsteğe bağlı: Basit LCD fonksiyonları (LCD varsa)
// LCD'ye bağlıysanız bu fonksiyonları uygulayabilirsiniz
/*
void LCD_Init(void) {
    // 4-bit mod инициализация
    TRISD = 0x00;              // PORTD çıkış
    __delay_ms(20);
    // ... LCD инициализация komutları
}

void LCD_Clear(void) {
    // Ekran temizleme komutu gönder
}

void LCD_Set_Cursor(unsigned char satir, unsigned char sütun) {
    // Kursör pozisyonunu ayarla
}

void LCD_Write_String(char *str) {
    // LCD'ye string yaz
}

void LCD_Write_Float(float num, unsigned char ondalikli_basamak) {
    // LCD'ye float sayı yaz
}
*/

// Ana fonksiyon
void main() {
    unsigned int adc_degeri;
    float voltaj;
    const float Vref = 5.0;    // VDD = 5V referans olarak kabul ediliyor

    // ADC'yi başlat
    ADC_Init();
    ADCON0bits.ADON = 1;       // ADC modülü açılır

    // İsteğe bağlı: LCD kullanılıyorsa başlat
    // TRISD = 0x00;            // PORTD çıkış için LCD
    // LCD_Init();
    // LCD_Clear();
    // LCD_Write_String("ADC Voltaj Metresi");
    // __delay_ms(1000);
    // LCD_Clear();

    while(1) {
        // AN0 kanalı (kanal 0)から ADC değeri okunur
        adc_degeri = ADC_Read(0);

        // Voltaja dönüştürülür
        voltaj = ADC_To_Voltage(adc_degeri, Vref);

        /*
        // LCD'de gösterim (eğer LCD varsa)
        LCD_Set_Cursor(0, 0);
        LCD_Write_String("AD=");
        LCD_Write_Float((float)adc_degeri, 0);
        LCD_Write_String("  ");

        LCD_Set_Cursor(1, 0);
        LCD_Write_String("V=");
        LCD_Write_Float(voltaj, 2);
        LCD_Write_String("V  ");
        */

        // Okumadan önce bekleme eklenir
        __delay_ms(500);
    }
}