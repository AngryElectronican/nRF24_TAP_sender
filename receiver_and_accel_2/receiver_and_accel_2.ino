#include <SPI.h>                          // Подключаем библиотеку SPI
#include <nRF24L01.h>                     // Подключаем библиотеку nRF24L01 
#include <RF24.h>                         // Подключаем библиотеку RF24 

uint8_t data[3];                              // Создаём массив для приёма данных
uint32_t received = 0;
RF24 radio(9, 10);                        // Указываем номера выводов nRF24L01+ (CE, CSN)
uint8_t address[] = {0xCC, 0xAA, 0x35};

void setup(void) {

  Serial.begin(9600);
  delay(1000);                            // Ждем 1с
    radio.begin();                          // Инициируем работу nRF24L01+
    radio.setChannel(100);                  // Указываем канал передачи (от 0 до 126)
    radio.setDataRate(RF24_1MBPS);         // Указываем скорость передачи (250KBPS, 1MBPS, 2MBPS)
    radio.setPALevel(RF24_PA_HIGH);         // Указываем мощность передатчика (MIN=-18dBm, LOW=-12dBm, HIGH=-6dBm, MAX=0dBm)
    radio.openReadingPipe(1, address); // Задаем идентификатор для передачи данных данных
    radio.startListening();  
    Serial.println("Radio init!");
    radio.setPayloadSize(32);
}

void loop(void)
{

if(!radio.isChipConnected()) //ошибка или чип не присоединен
  Serial.println("Radio failure!");
  
  if (radio.available()) {
    while (radio.available()) {
      radio.read(data, sizeof(data));
}


    Serial.println("Received!");
    for (int i = 0; i < sizeof(data); i++) {
      Serial.println(data[i]);
    }
    
    if (data[1] == 255) Serial.println("TAP!");
  } //radio.available endif
}
