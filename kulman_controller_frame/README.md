# Arac_Controller_Frame
English version is below.

Bu pakat Aracın genel güdümcü döngüsünden sorumludur. Benzetim yada gerçek kulmandan gelen algılayıcı verilerini alt düzey güdümcüye iletmek, deviteç girdilerini alt düzey güdümcüden benzerime yada gerçek kulmana iletmek, ve oyun kolu verisini devinim tasarımcısına iletmek gibi görevleri vardır.


Burada ayarlanmasi gereken onemli degiskenler.

time_step : arac_controller_frame.launch icinde yer alir ve gudumcunun 1 dongusunun suresini verilerir.
parameters.yaml : her turlu denetleyici ve durum kestirimci degiskenlerini icerir
