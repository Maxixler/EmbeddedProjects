/**
 * @file main.c
 * @brief PWM Frekans Üretimi PIC16F877A için
 * @details Bu proje, CCP1 modülünü PWM modunda kullanarak belirli bir frekansla PWM sinyali üretilmesini gösterir.
 *          Örnek, 8MHz osilatör frekansı verildiğinde istenen PWM frekansı (örneğin 5kHz) için PR2 değerinin nasıl hesaplanacağını gösterir.
 *
 *          Kullanılan Formüller:
 *          PWM Periyodu = [(PR2) + 1] * 4 * Tosc * TMR2 Prescale Değeri
 *          PWM Frekansı = 1 / PWM Periyodu
 *
 *          8MHz osilatör ve prescaler = 1 için:
 *          Tosc = 1 / Fosc = 0.25 us
 *          PWM Periyodu = (PR2 + 1) * 4 * 0.25 us * 1 = (PR2 + 1) * 1 us
 *          PWM Frekansı (Hz) = 1 / [(PR2 + 1) * 1e-6] = 1e6 / (PR2 + 1)
 *          Bu yüzden, PR2 = (1e6 / PWM_Frekansı) - 1
 *
 *          Örnek: 5kHz PWM için, PR2 = (1e6 / 5000) - 1 = 200 - 1 = 199
 *
 *          Çözünürlük (bit cinsinden) = log2(Fosc / (PWM_Frekansı * Prescaler * 4))
 *          8MHz, 5kHz, prescaler=1 için: log2(8000000/(5000*4)) = log2(400) ≈ 8.64 bit
 *
 *          PWM Çözünürlük hesabı, kullanılabilir discrete duty cycle adım sayısını gösterir.
 *          Daha yüksek çözünürlük, duty cycle üzerinde daha hassas kontrol sağlar ancak maksimum frekansı sınırlar.
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

// PWM başlatma fonksiyonu
void PWM_Init(long frekans) {
    // TRISC pini (CCP1) çıkış olarak ayarlanır
    TRISC2 = 0;                // RC2/CCP1 pini çıkış olarak

    // İstenen frekans için PR2 değeri hesaplanır
    // PR2 = (_XTAL_FREQ / (frekans * 4 * TMR2_PRESCALER)) - 1
    // Prescaler = 1 kullanılıyor (T2CKPS1:T2CKPS0 = 00)
    unsigned int PR2_degeri = (_XTAL_FREQ / (frekans * 4)) - 1;

    // Timer2 yapılandırılır
    PR2 = PR2_degeri;          // PWM periyodu ayarlanır
    T2CKPS0 = 0;               // Prescaler 1:1
    T2CKPS1 = 0;
    TMR2ON = 1;                // Timer2 aktifleştirilir

    // CCP1 modülü PWM modu için yapılandırılır
    CCP1M3 = 1;                // PWM modu
    CCP1M2 = 1;
    CCP1M1 = 0;
    CCP1M0 = 0;

    // Başlangıçta duty cycle %50 olarak ayarlanır
    // Duty cycle = (CCPR1L:CCP1CON<5:4>) * Tosc * TMR2 Prescaler
    // %50 için: CCPR1L = (PR2 + 1) / 2
    CCPR1L = (PR2_degeri + 1) / 2;
    CCP1CONbits.DC1B = 0;      // Duty cycle bitleri 1:0 temizlenir
}

// PWM duty cycle ayarlama fonksiyonu (0-100%)
void PWM_Set_Duty(unsigned int duty_yuzde) {
    if (duty_yuzde > 100) return;  // Geçerli aralığı sınırla

    // Yüzden değerinden duty cycle değeri hesaplanır
    // Duty cycle = (CCPR1L:CCP1CON<5:4>) * Tosc * TMR2 Prescaler
    // Maksimum duty cycle değeri = PR2 + 1
    unsigned int duty_degeri = ((PR2 + 1) * duty_yuzde) / 100;

    // Duty cycle değeri CCPR1L ve DC1B bite uygulanır
    CCPR1L = duty_degeri >> 2;          // Üst 8 bit
    CCP1CONbits.DC1B = duty_degeri & 0x03; // Alt 2 bit
}

// Ana fonksiyon
void main() {
    // 5kHz PWM için başlatılır (motor kontrolü, LED düzeltme için yaygın)
    PWM_Init(5000);            // 5kHz PWM sinyali üretilir

    // Örnek: Başlatıldıktan sonra duty cycle %75 olarak ayarlanır
    PWM_Set_Duty(75);

    while(1) {
        // Ana döngü - PWM arka planda sürekli çalışır
        // Burada sensör girdisine göre duty cycle dinamik olarak değiştirilebilir.
        // Örneğin, bir potansiyometreyi okuyarak LED parlaklığı ayarlanabilir:
        // unsigned int pot_degeri = Oku_ADC(0); // AN0'u oku
        // unsigned int duty = (pot_degeri * 100) / 1023; // 0-100%'e dönüştür
        // PWM_Set_Duty(duty);
    }
}