#include <SPI.h>                        // Подключаем библиотеку SPI
#include <nRF24L01.h>                   // Подключаем библиотеку nRF24L01 
#include <RF24.h>                       // Подключаем библиотеку RF24 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LowPower.h>
#include <Adafruit_ADXL345_U.h>

byte data[32] = {0};                 // TAP, ADC_HIGH, ADC_LOW
byte data_test[3] = {255,255,255}; 
RF24 radio(9, 10);                      // Указываем номера выводов nRF24L01+ (CE, CSN)
uint8_t address[] = {0xCC, 0xAA, 0x35, 0x77, 0xBB};

#define INT1_PIN 2
#define ADC_PIN A0

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
uint16_t adc=0;

void wakeUp(){
    //accel.readRegister(ADXL345_REG_INT_SOURCE);
    detachInterrupt(0);
    data[0]=255;
    ADCSRA|=(1<<ADEN);
    adc=analogRead(ADC_PIN);
    for(uint8_t i=0;i<7;i++){
      adc=(adc+analogRead(ADC_PIN))/2;
    }
    data[1]=highByte(adc);
    data[2]=lowByte(adc);
    radio.powerUp();
    radio.write(data,  sizeof(data));
    data[0]=0;
    radio.powerDown();
}

void setup(void){
    pinMode(ADC_PIN, INPUT);
    radio.begin();                        // Инициируем работу nRF24L01+
    radio.setChannel(100);                // Указываем канал передачи (от 0 до 126)
    radio.setDataRate (RF24_1MBPS);       // Указываем скорость передачи (250KBPS, 1MBPS, 2MBPS)
    radio.setPALevel(RF24_PA_HIGH);       // Указываем мощность передатчика (MIN=-18dBm, LOW=-12dBm, HIGH=-6dBm, MAX=0dBm)
    radio.openWritingPipe(address);       // Задаем адрес для передачи данных данных
    radio.setPayloadSize(32);
    radio.stopListening(); 
    radio.powerDown();
    pinMode(INT1_PIN,INPUT);
    //Serial.print("Accel setting");
    accel.begin();
    accel.setRange(ADXL345_RANGE_2_G);
    accel.writeRegister(ADXL345_REG_THRESH_TAP,  0x40);
    accel.writeRegister(ADXL345_REG_DUR,  0xFF);
    accel.writeRegister(ADXL345_REG_TAP_AXES,  0x01);
    accel.writeRegister(ADXL345_REG_DATA_FORMAT,  accel.readRegister(ADXL345_REG_DATA_FORMAT) | (1 << 5));
    accel.writeRegister(ADXL345_REG_INT_MAP,  (0 << 6));
    accel.writeRegister(ADXL345_REG_INT_ENABLE,  (1 << 6));
    radio.powerDown();
    attachInterrupt(0, wakeUp, LOW);
    sei();
}

void loop(void){
    for(uint8_t i=0;i<1;i++){
      attachInterrupt(0, wakeUp, LOW);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);//64 sec sleep
      accel.readRegister(ADXL345_REG_INT_SOURCE);
      attachInterrupt(0, wakeUp, LOW);
    }
    ADCSRA|=(1<<ADEN);
    for(uint8_t i=0;i<8;i++){
      adc=analogRead(ADC_PIN);
    }
    data[1]=highByte(adc);
    data[2]=lowByte(adc);
    radio.powerUp();
    radio.write(data,  sizeof(data));
    data[0]=0;
    radio.powerDown();
}
