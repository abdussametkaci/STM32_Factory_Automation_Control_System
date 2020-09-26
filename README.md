# Fabrika Otomasyonu Kontrol Sistemi
## Proje Sonunda Ortaya Çıkacak Ürünün Özellikleri:
Proje bir fabrika bandının genel çalışma presibine dayanarak yapılmıştır. İki adet butonumuz vardır ve bu butonlardan
birisi fabrikayı açıp kapamak için, diğeri ise ürün bandına bir eklemek için kulllanılmştır. NTC sensöründen 0.5 saniye
aralıklarla sıcaklık ölçümü yapılmaktadır. Birinci ürün grubu sıcak, ikinci ürün grubu soğuktur. Fabrika açıldığında led
yanar ve hangi grup ürün geldiyse onu temsil eden led yanmaktadır. TIMER birimleri kullanılarak 5 saniyede bir
ürünlerin adedi gönderilmektedir. Eğer 10 saniye boyunca hiç ürün okunmazsa ya da limit (199) aşılırsa hata mesajları
gösterilmektedir.Projede stm32’den bilgisayara USART ile mesaj gönderilmesi istenmiştir.Ancak biz USB TTL
kullanımında sıkıntı yaşadığımız için stm32’nin içindeki 2 tane USART birimi arasında haberleşme yaptık.

## Projede Kullanılacak Malzemeler ve Özellikleri:
 4 adet led

    o Fabrikanın açık olup olmadığını göstermek için,
    o Hangi grup ürün geldiğini göstermek için (2 led),
    o Toplam ürün sayısı aşımlarını göstermek için kullanılmıştır.
    
 NTC sensörü : Sıcaklığa bağlı olarak ADC değeri okumak için kullanılmıştır. Bu projede stm32f103c8t6 modeli
kullanılmıştır ve bu model sadece 12 bit çözünürlük desteklemektedir. Projede ise 6 bit istenmektedir ve gelen
ADC değeri 6 bite dönüştürülmüştür. Bunun için aşağıdaki formül kullanılmıştır:
    o (2
6 – 1) x 12 bitlik adc değeri / (2
12 – 1)
 2 adet buton
    o Fabrikayı açmak/kapatmak için
    o Ürün eklemek için kullanlmıştır.
 2 adet seven segment gösterge : Toplam ürün adedini göstermek amacıyla kullanılmıştır.
 2 adet BC337 (BJT NPN) transistör : Hangi seven segment göstergenin yanacağını seçmek için anahtarlama
yapmak amacıyla kullanılmıştır.
 TIMER birimi prescaler ve counter periyot değerlerini hesaplamak için kullandığımız formül:
