# STM32 Modbus RTU Slave - Algoritma Akışı
# STM32 Modbus RTU Slave - Algorithm Flow

---

## 🇹🇷 Türkçe

Bu yazılım, bir RTOS (İşletim Sistemi) kullanmadan çalışır.
Sistem, kesme (interrupt) güdümlü bir modeli ve engellemesiz (non-blocking) bir ana döngüyü temel alır.
Tüm zamanlama, 1 milisaniyede bir çalışan SysTick kesmesi ile yönetilir.
SysTick, farklı görevler için periyodik bayraklar (flag) ayarlar (örneğin 10ms, 40ms, 750ms).
Ana `while(1)` döngüsü, bu bayrakları sürekli olarak kontrol eder.
Bir bayrak ayarlandığında, ana döngü ilgili kısa görevi (örn: IMU okuma, ADC hesaplama) çalıştırır.
Uygulama, Modbus ve Kesme katmanları birbirinden bağımsızdır.
Tüm katmanlar arasındaki veri alışverişi, Modbus bellek dizileri (Registerlar, Coiller) üzerinden yapılır.
Uygulama döngüsü, sensör verilerini (ADC, IMU) okur ve bu verileri "Input Register" dizilerine yazar.
Ayrıca sistem durumlarını (hatalar, limitler) "Discrete Input" dizilerine yazar.
Bir Modbus çerçevesi (frame) geldiğinde, UART "Receive-to-Idle" kesmesi tetiklenir.
Bu kesme, Modbus protokol işlemcisini başlatır.
İşlemci, CRC'yi doğrular ve fonksiyon kodunu yorumlar.
Eğer bir "yazma" isteği ise (örn: Holding Register), veriyi ilgili dizilere yazar.
Eğer bir "okuma" isteği ise, veriyi ilgili dizilerden okur ve yanıtı oluşturur.
Ana döngü, Modbus tarafından yazılan "Coil" ve "Holding Register" dizilerini de okur.
Bu dizilerdeki veriyi (örn: PWM değeri, LED durumu) donanıma uygular.
Donanım (PWM, GPIO) yalnızca değer önceki durumdan farklıysa güncellenir.

---

## 🇬🇧 English

This software operates without an RTOS (Operating System).
The system is based on an interrupt-driven model and a non-blocking main loop.
All timing is managed by the SysTick interrupt, which runs every 1 millisecond.
SysTick sets periodic flags for different tasks (e.g., 10ms, 40ms, 750ms).
The main `while(1)` loop continuously checks these flags.
When a flag is set, the main loop executes the corresponding short task (e.g., read IMU, calculate ADC).
The Application, Modbus, and Interrupt layers are independent of each other.
Data exchange between all layers happens via the Modbus memory arrays (Registers, Coils).
The application loop reads sensor data (ADC, IMU) and writes this data into the "Input Register" arrays.
It also writes system statuses (faults, limits) into the "Discrete Input" arrays.
When a Modbus frame arrives, the UART "Receive-to-Idle" interrupt is triggered.
This interrupt initiates the Modbus protocol processor.
The processor validates the CRC and interprets the function code.
If it is a "write" request (e.g., Holding Register), it writes data to the respective arrays.
If it is a "read" request, it reads data from the arrays and builds the response.
The main loop also reads the "Coil" and "Holding Register" arrays written by Modbus.
It applies this data (e.g., PWM value, LED state) to the hardware.
The hardware (PWM, GPIO) is only updated if the value is different from the previous state.