#include <timer_class.h>
#include <SPI.h>                        // Подключаем библиотеку SPI
#include <nRF24L01.h>                   // Подключаем библиотеку nRF24L01 
#include <RF24.h>                       // Подключаем библиотеку RF24 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

byte data[3] = {0, 0, 0};                 // Создаём массив для приёма данных
RF24 radio(9, 10);                      // Указываем номера выводов nRF24L01+ (CE, CSN)
uint8_t adress[] = {0xCC, 0xAA, 0x35};

Timer timer1(10, SECS);

#define INT1_pin 5
#define ADC_pin A0
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup(void) {
  pinMode(ADC_pin, INPUT);
  Serial.begin(9600);
  radio.begin();                        // Инициируем работу nRF24L01+
  radio.setChannel(100);                // Указываем канал передачи (от 0 до 126)
  radio.setDataRate (RF24_1MBPS);       // Указываем скорость передачи (250KBPS, 1MBPS, 2MBPS)
  radio.setPALevel(RF24_PA_HIGH);       // Указываем мощность передатчика (MIN=-18dBm, LOW=-12dBm, HIGH=-6dBm, MAX=0dBm)
  radio.openWritingPipe(adress);// Задаем адрес для передачи данных данных
  radio.setPayloadSize(32);
  //radio.setRetries(uint8 delay, uint8 count)
  //delay How long to wait between each retry, in multiples of 250us, max is 15. 0 means 250us, 15 means 4000us.
  //count How many retries before giving up, max 15
  if (!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
  pinMode(INT1_pin, INPUT_PULLUP);
  accel.setRange(ADXL345_RANGE_2_G);
  accel.writeRegister(ADXL345_REG_THRESH_TAP,  0x40);
  accel.writeRegister(ADXL345_REG_DUR,  0x0F);
  accel.writeRegister(ADXL345_REG_TAP_AXES,  0x01);
  accel.writeRegister(ADXL345_REG_DATA_FORMAT,  accel.readRegister(ADXL345_REG_DATA_FORMAT) | (1 << 5));
  accel.writeRegister(ADXL345_REG_INT_ENABLE,  (1 << 6));
}

void loop(void)
{
  if (Serial.available() > 0 and Serial.read() == 's' ) { // when 's' typed
    //check errors
    if (!radio.isChipConnected()) {
      Serial.println("Transmitter is not connected!");
    }

    if (accel.getDeviceID() != 0xE5) {
      Serial.println("Axel error!");
    }
    
    data[0] = accel.getDeviceID();
    data[1] = 0;
    data[2] = map(analogRead(ADC_pin), 0, 1023, 0, 100); //adc info
    sensors_event_t event;
    accel.getEvent(&event);
    
    if (digitalRead(INT1_pin) != HIGH) {
      accel.readRegister(ADXL345_REG_INT_SOURCE);
      data[1] = 255;
      Serial.print("TAP ");
      Serial.println(digitalRead(INT1_pin));
    }
    
    // add ack?
    radio.write(&data, sizeof(data));               // Отправляем данные из массива data

    while (!radio.txStandBy(1000)) {
      Serial.println("Transmitter error!");
      //переделать под попытки переподключиться
    }
    
    Serial.println(analogRead(ADC_pin));
    Serial.print("Sended ID and TAP status: ");
          Serial.print("acsel int ");
      Serial.println(digitalRead(INT1_pin));
    for (int i = 0; i < sizeof(data); i++) {
      Serial.println(data[i]);
    }
    Serial.println();
    delay(50);
  }

}
