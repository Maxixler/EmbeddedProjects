Mevcut el yazısı notları ve ders sunumları incelendiğinde, elektrik-elektronik mühendisliği son sınıf seviyesindeki bir mikrodenetleyici final sınavı için belirli teknik alanların ön plana çıktığı görülmektedir. Özel olarak yuvarlak içine alınmış "frekans üretme" notu ve blok diyagramı analizleri, donanım seviyesinde sinyal işleme ve zamanlama mimarilerine odaklanılacağını işaret etmektedir.

Aşağıda, sağlanan materyaller doğrultusunda sınavda karşılaşılabilecek muhtemel soru tipleri ve çalışma odakları kategorize edilmiştir.

!!! Bu projeler pic16f877a mikrodenetleyicisi için MikroC'de kullanmak üzere geliştirilecektir. !!!

### 1. Frekans Üretme ve Zamanlayıcı (Timer/CCP) Hesaplamaları

El yazısı notlarındaki en belirgin ipucu "frekans üretme" ifadesidir. Geçmişte frekans ölçümü sorulmuş olması, bu sınavda istenen bir frekansın nasıl üretileceğinin sorulma ihtimalini yükseltmektedir. Bu işlem iki temel yöntemle gerçekleştirilebilir bu iki yöntemle de ayrı ayrı proje oluşturacaksın:

* **PWM (Darbe Genişlik Modülasyonu) ile Frekans Üretimi:** CCP modülünün PWM modunda kullanılarak belirli bir frekansta (örneğin 5KHz veya 10KHz) sinyal üretilmesi istenebilir. Sınavda bir osilatör frekansı ($F_{OSC}$) verilip, $PR2$ kaydedicisine yazılması gereken değer sorulabilir.
* 
**İlgili Formül:** 
$$PWM\_Periyodu = [(PR2) + 1] \cdot 4 \cdot T_{OSC} \cdot (TMR2\_Prescale\_Value)$$





* PWM çözünürlüğü hesabı da teorik bir soru olarak karşılaşılabilecek bir diğer konudur: 


$$Resolution = \frac{\log(\frac{F_{OSC}}{F_{PWM}})}{\log(2)}$$







* **Timer Kesmesi (Interrupt) ile Kare Dalga Üretimi:** Timer0 veya Timer1 kullanılarak belirli periyotlarda bir çıkış pininin durumunu tersine çeviren (toggle) kod bloğunun yazılması veya TMR kaydedicisine yüklenecek başlangıç değerinin hesaplanması istenebilir.
* 
**İlgili Formül:** 
$$TMRx\_reg\_degeri = Timer\_max - \left(\frac{Gecikme \cdot F_{OSC}}{Prescale \cdot 4}\right)$$








### 2. Donanım Açıklaması ve Blok Diyagramı Yorumlama

Notlarda yer alan "donanımı verir açıklattır" ve "blok diyagramı yorumlama" ifadeleri, mikrodenetleyicinin iç mimarisindeki belirli modüllerin teorik işleyişinin sorulacağını göstermektedir.

* 
**ADC Modülü Blok Diyagramı:** Bir analog sinyalin sayısala çevrilme sürecindeki (örnekleme ve nicemleme) adımlar  veya $ADCON0$ ve $ADCON1$ kaydedicilerinin blok diyagram üzerindeki etkileri sorulabilir. Çözünürlük hesabı yapılarak, belirli bir referans voltajında okunan ADC değerinin kaç volta karşılık geldiğinin bulunması güçlü bir olasılıktır.


* 
**Doğrudan Hafıza Erişimi (DMA):** İşlemciyi (CPU) meşgul etmeden bellekten belleğe veya bellekten çevre birimine veri aktarımını sağlayan DMA'nın blok diyagramı verilerek çalışma mantığı (örneğin Otomatik Başlatma veya Hafızadan Hafızaya transfer türleri) sorulabilir.



### 3. Kesmeler (Interrupt) ve Kod Yazımı

Kesme (interrupt) mantığı ve başlangıç (init) ayarları kodlama sorularının temelini oluşturacaktır.

* **Kayıtçı (Register) Ayarları:** `INTCON`, `PIE1` ve `PIR1` kayıtçılarının doğru ayarlanması beklenmektedir. Özellikle Global Kesme (`GIE`) ve Çevresel Kesme (`PEIE`) bitlerinin aktifleştirilmesi kod bloklarında aranan ilk detaylar olacaktır.


* 
**Kesme Alt Programı:** ADC çevrimi bittiğinde (`ADIF`) veya Timer taştığında (`TMR0IF`) bayrakları temizleyen kesme alt programı tasarımları istenecektir.



### 4. Giriş/Çıkış Tipleri ve Tuş Takımı Matrisi

"Giriş çıkış tipleri sorar" ve "giriş çıkış butonu" notları, analog ve dijital pin konfigürasyonlarına işaret etmektedir.

* 
**Matris Tuş Takımı (Keypad):** 3x4 veya 4x4 tuş takımı yapısının satır (row) ve sütun (column) tarama mantığı üzerinden nasıl çalıştığının açıklanması istenebilir.


* 
**Analog/Dijital Pin Seçimi:** `TRIS` kayıtçıları ile yönlendirme yapıldıktan sonra, $ADCON1$ üzerinden hangi pinlerin analog, hangi pinlerin dijital okuma/yazma yapacağının (örneğin `PCFG3:PCFG0` ayarları) belirlenmesi sorulabilir.


 ### 5. Sayaç (Counter) Modu ile Frekans Ölçümü (Yüksek Frekanslar İçin)
 iki çözüm yöntemi var yine aynı şekilde 2 proje oluştur.
 En klasik ve doğrudan yöntemdir. Frekansın tanımı "1 saniyedeki döngü (darbe) sayısı" olduğu için, mikrodenetleyicinin bir zamanlayıcısını (Timer) dışarıdan gelen darbeleri sayacak şekilde ayarlarız.
 
 Kullanılacak Donanım: 16-bit olduğu için daha büyük sayıları ($65535$'e kadar) taşmadan sayabilen Timer1 modülü tercih edilir. Sinyal, Timer1'in harici saat girişi olan RC0/T1CKI pininden girilmelidir.  
 
 Çalışma Mantığı: 1. Timer1, harici saat kaynağını (RC0 pinini) sayacak şekilde sayaç (counter) modunda kurulur.2. Sayaç sıfırlanır (TMR1H = 0 ve TMR1L = 0).3. Sistem tam olarak 1 saniye bekletilir (Bu bekleme işlemi hassas olması için Delay_ms(1000) yerine genellikle Timer0 kesmesi ile tasarlanır).4. 1 saniyenin sonunda Timer1 durdurulur ve içindeki değer okunur. Okunan değer doğrudan frekansı (Hz) verir.

  CCP Capture (Yakalama) Modu ile Frekans Ölçümü (Düşük Frekanslar İçin)Eğer hoca soruyu senin tahmin ettiğin gibi CCP üzerinden sorarsa, kullanman gereken mod Yakalama (Capture) modudur. Bu yöntem, 1 saniye boyunca saymak yerine, art arda gelen iki darbe arasındaki süreyi (Periyodu) ölçer.  Kullanılacak Donanım: Sinyal RC2/CCP1 girişinden uygulanır. CCP modülü zaman referansı olarak yine Timer1'i kullanır.  Çalışma Mantığı:CCP modülü her yükselen kenarı yakalayacak şekilde ayarlanır (CCP1CON = 0b00000101).  Timer1 dahili saat kaynağı ile (normal bir kronometre gibi) saymaya başlatılır.  Pinden ilk yükselen kenar (lojik 0'dan 1'e geçiş) geldiğinde, CCP donanımı Timer1'in o anki değerini otomatik olarak yakalar ve CCPR1H / CCPR1L kaydedicilerine yazar. Bu değer $t_1$ olarak bir değişkene kaydedilir.İkinci yükselen kenar geldiğinde aynı işlem tekrarlanır ve yeni değer $t_2$ olarak kaydedilir.İki değer arasındaki fark sinyalin periyodunu verir: $Periyot(T) = t_2 - t_1$.Frekans ise $f = \frac{1}{T}$ formülü ile hesaplanır. Elde edilen periyot değeri komut işleme süresi ($T_{cy}$) ile çarpılarak gerçek zamana çevrilmelidir.

