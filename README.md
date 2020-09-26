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
• 4 adet led

    o Fabrikanın açık olup olmadığını göstermek için,
    o Hangi grup ürün geldiğini göstermek için (2 led),
    o Toplam ürün sayısı aşımlarını göstermek için kullanılmıştır.
    
• NTC sensörü : Sıcaklığa bağlı olarak ADC değeri okumak için kullanılmıştır. Bu projede stm32f103c8t6 modeli
kullanılmıştır ve bu model sadece 12 bit çözünürlük desteklemektedir. Projede ise 6 bit istenmektedir ve gelen
ADC değeri 6 bite dönüştürülmüştür. Bunun için aşağıdaki formül kullanılmıştır:

![image](https://user-images.githubusercontent.com/61049743/94344563-f429bd00-0028-11eb-98ce-1b6f9cbdf4e7.png)
    
• 2 adet buton

    o Fabrikayı açmak/kapatmak için
    o Ürün eklemek için kullanlmıştır.
    
• 2 adet seven segment gösterge : Toplam ürün adedini göstermek amacıyla kullanılmıştır.

• 2 adet BC337 (BJT NPN) transistör : Hangi seven segment göstergenin yanacağını seçmek için anahtarlama
yapmak amacıyla kullanılmıştır.

• TIMER birimi prescaler ve counter periyot değerlerini hesaplamak için kullandığımız formül:

![image](https://user-images.githubusercontent.com/61049743/94344607-394def00-0029-11eb-8a89-d3b0ca152319.png)

## Kullanılan Malzemelerin Çalışması için STM CubeMX ayarları:
![image](https://user-images.githubusercontent.com/61049743/94344657-731ef580-0029-11eb-8f8a-bd0c290ab0c6.png)

PA0 – PA6 arasındaki pinler Seven Segment göstergeler
için kullanılacaktır.

PA7 pini NTC sensöründen analog değer okumak için
kullanılacaktır.

PB0 ve PB1 transistörlerde anahtarlama yapmak için
kullanılacaktır.

PB10 ve PB11 USART3 haberleşme için kullanılacaktır.
Bu USART birimi mesaj alantaraf olacaktır.

PA9 ve PA10 USART1 haberleşme için kullanılacaktır.
Bu USART birimi mesaj gönderen taraf olacaktır.

PB12 ürün sayısını artıracak buton için kullanılacaktır.

PB13 fabrikayı açıp kapatacak buton için kullanılacaktır.

PB7 fabrikanın aktif olup olmadığını gösteren led için
kullanılacaktır.

PB8 eklenen sıcak ürün olduğunu gösteren led için
kullanılacaktır.

PB3 eklenen soğuk ürün olduğunu göstermek için
kullanılacaktır.

PB5 ürün aşım ledi olarak kullanılacaktır.

PA13 ve PA14 ST-Link V2 ile kod yüklemek amacıyla
kullanılacaktır.

PD0 ve PD1 harici osilatörden sinyal almak için
kullanılacaktır.

PB4 TIMER 3 birimi için kullanılacaktır ve ADC değerinin
okunacağı periyodu belirler.

PB6 TIMER 4 birimi için kullanılacaktır ve USART
haberleşmenin yapılacağı periyodu belirler.

USART1 ve USART3 ayarları aşağıdaki gibi yapılmıştır:

![image](https://user-images.githubusercontent.com/61049743/94344776-49b29980-002a-11eb-9a2a-3d5a369d932e.png)

UART1, 115200 bps olacak, data uzunluğu 8 bit olacak
şekilde konfigüre edilmiştir.

Interrupt aktif edilmiştir.

TIMER 3 ayarları aşağıdaki gibi yapılmıştır:

![image](https://user-images.githubusercontent.com/61049743/94344809-82eb0980-002a-11eb-8314-76334a74819b.png)

![image](https://user-images.githubusercontent.com/61049743/94344823-9b5b2400-002a-11eb-83ea-06d637d8139c.png)

Timer 3 birimi, 2 Hz frekansta (0.5 saniye periyot)
çalışacak şekilde ayarlanmıştır. Ayarlar yapılırken STM
saat hızı (16 MHZ) dikkate alınmıştır.

Bu birimde channel 1 kullanılacaktır ve global interrupt
aktif edilmiştir.

TIMER 4 ayarları aşağıdaki gibi yapılmıştır:

![image](https://user-images.githubusercontent.com/61049743/94344848-d9f0de80-002a-11eb-8414-fd6c46244e53.png)

Timer 4 birimi, 0.2 Hz frekansta (5 saniye periyot)
çalışacak şekilde ayarlanmıştır. Ayarlar yapılırken STM
saat hızı (16 MHZ) dikkate alınmıştır.

Bu birimde channel 1 kullanılacaktır ve global interrupt
aktif edilmiştir.

ADC1 ayarları aşağıdaki gibidir:
