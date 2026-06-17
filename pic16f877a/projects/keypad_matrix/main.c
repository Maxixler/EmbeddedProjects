/**
 * @file main.c
 * @brief Matris Tuş Takımı Tarama PIC16F877A için
 * @details Bu proje, PORTB'ye bağlı 4x4 matris tuş takımının nasıl taranacağını gösterir.
 *          Satırlar RB0-RB3 (çıkışlar) ile, sütunlar RB4-RB7 (çekim-up direnciyle girişler) ile bağlanır.
 *          Tuş takımı, her satırı sırayla düşük yaparak ve sütunları okunarak taranır.
 *          Debouncing (tıklama önleme) bir gecikme kullanılarak uygulanmıştır.
 *
 *          Tuş Eşleştirme (4x4):
 *            {'1','2','3','A'},
 *            {'4','5','6','B'},
 *            {'7','8','9','C'},
 *            {'*','0','#','D'}
 *
 *          3x4 bir tuş takımı için, son satırı çıkar ve tuş eşlemesini buna göre ayarla.
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

// Tuş takımını başlatan fonksiyon
void Keypad_Init(void) {
    // PORTB yapılandırılır: RB0-RB3 çıkış (satırlar), RB4-RB7 giriş (sütunlar)
    TRISB = 0xF0;              // 0b11110000: RB4-RB7 giriş, RB0-RB3 çıkış
    PORTB = 0xFF;              // Başlangıçta tüm pinler yüksek seviyede (satırlar yüksek, sütunlar çekilmiş)
}

/**
 * @brief Tuş takımını tarar ve basılan tuşu döndürür.
 * @return Basılan tuşun ASCII karakteri, veya tuş yoksa 0.
 * @note Bu fonksiyon debouncing kullanır ve 4x4 tuş takımı varsayar.
 *       3x4 için, tuş eşlemesi ve döngü sınırlarını ayarla.
 */
char Keypad_GetKey(void) {
    const char keymap[4][4] = {
        {'1','2','3','A'},
        {'4','5','6','B'},
        {'7','8','9','C'},
        {'*','0','#','D'}
    };
    unsigned char row, col;

    // İlk olarak, herhangi bir tuşa basılıp basılmadığı kontrol edilir (herhangi bir sütun düşük mü?)
    if ((PORTB & 0xF0) == 0xF0)
        return 0;   // Tuş basılmamış (tüm sütunlar yüksek)

    // Debouncing için bekle (20ms)
    __delay_ms(20);

    // Debouncing sonrası tekrar kontrol et
    if ((PORTB & 0xF0) == 0xF0)
        return 0;   // Hala tuş yok, possibly gürültü

    // Şimdi her satırı tara
    for (row = 0; row < 4; row++) {
        // Mevcut satırı düşük yap, diğerlerini yüksek tut
        // Satır bitlerini (RB0-RB3) temizle ve mevcut satırı 0 yap
        PORTB = (PORTB & 0xF0) | (0x0F & ~(1 << row));

        // Sinyalin yerleşmesini bekle
        __delay_us(50);

        // Sütunları oku (RB4-RB7)
        col = (PORTB & 0xF0) >> 4; // Sütun bitlerini bit 0-3'e kaydır

        // Herhangi bir sütun düşükse, bu satırda tuş basılmış demektir
        if (col != 0x0F) { // Tüm sütunlar yüksek değil
            // Hangi sütunun düşük olduğunu bul (tek tuş basılı varsayılır)
            for (unsigned char c = 0; c < 4; c++) {
                if (!(col & (1 << c))) {
                    // Tuş bulundu: row, column c
                    return keymap[row][c];
                }
            }
        }
    }

    // Buraya ulaşılırsa bir sorun oldu (tuş basılıysa olmamalı)
    return 0;
}

// Ana fonksiyon
void main() {
    unsigned char key;
    // RC0'nun bir LED çıkışı olarak kullanılması, tuş basıldığında yanmasını gösterir
    TRISC0 = 0;    // RC0 çıkış
    RC0 = 0;       // Başlangıçta LED kapalı

    // Tuş takımını başlat
    Keypad_Init();

    while(1) {
        // Tuş takımını tara
        key = Keypad_GetKey();

        // Bir tuşa basıldıysa LED'i yak, aksi takdirde söndür
        if (key) {
            RC0 = 1;   // Tuş basıldığında LED yanar
            // İsteğe bağlı: burada tuşu işleyebilirsiniz (örn. UART ile gönder, tampona ekle)
            // Örnek:
            //   UART_Write(key); // UART fonksiyonunuz varsa
        } else {
            RC0 = 0;   // Tuş yokken LED söner
        }

        // Fazla CPU kullanımını önlemek için küçük gecikme
        __delay_ms(10);
    }
}