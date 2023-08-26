#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>
#include <SharpIR.h>
#include <ControleMotores.h>
#include <ControleSensores.h>
#include "veml6040.h"
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
// set the pins to shutdown
#define SHT_LOX1 26
#define SHT_LOX2 40
#define SHT_LOX3 28
#define SHT_LOX4 43


// objects for the vl53l0x
Adafruit_VL53L0X sensorLaserFrente = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserDireita = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserRampa = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserEsquerda = Adafruit_VL53L0X();


//Inicializar sensor sharp da frente
#define sensorSharpFrente A14
#define sensorSharpDireita A15
#define sensorSharpEsquerda A13

//Ligar leds vermelhos da placa seguidor de linha
#define ledVermelho1 53
#define ledVermelho2 50
#define ledVerde1 49
#define ledVerde2 51
//Definir pino do sensor de inclinação
#define sensorRampa 30
//Definir pino para receber o PULLUP da garra
#define identificadorBolinhas 25
//Botao de calibrar para os sensores de linha
#define botaoCalibrar 52

//Inicializar os motores principais
ControleMotores motores;
#define quantidadeSensores 7
//Instancia sensores de linha usando a biblioteca 'ControleSensores':
const byte PORTAS_ANALOGICOS[quantidadeSensores] = { A0, A1, A2, A3, A4, A6, A5 };
ControleSensores sensores(PORTAS_ANALOGICOS, quantidadeSensores, botaoCalibrar);
//Instancias dos sensores veml6040;
VEML6040 sensorRGB;
short corteMinVml[2] = { 2000, 3000 }, corteMaxVml[2] = { 3000, 4500 };
short testeVerdeEsquerda = 0, testeVerdeDireita = 0;

//Inicializacao do LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  beginServos();
  lcd.init();       // Serve para iniciar a comunicação com o display já conectado
  lcd.backlight();  // Serve para ligar a luz do display
  lcd.clear();      // Serve para limpar a tela do display

  //Inicializar sensor Sharp da frente
  pinMode(sensorSharpFrente, INPUT);
  pinMode(sensorSharpDireita, INPUT);
  pinMode(sensorSharpEsquerda, INPUT);

  //Definir pinos dos leds
  pinMode(ledVermelho1, OUTPUT);
  pinMode(ledVermelho2, OUTPUT);
  pinMode(ledVerde1, OUTPUT);
  pinMode(ledVerde2, OUTPUT);
  ledVerde();

  //Iniciar e configurar VEML6040
  selecionarDispositivo(0);
  sensorRGB.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);

  selecionarDispositivo(1);
  sensorRGB.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);

  //Definir pinos para identificar as bolinhas
  pinMode(identificadorBolinhas, INPUT_PULLUP);
  //Definir servos da garra

  selecionarDispositivo(7);
  scrollText("OBR-2208", 270);
  lcd.clear();

  iniciarSensorLaser();
  garraInicial();


  pinMode(identificadorBolinhas, INPUT_PULLUP);
  selecionarDispositivo(7);
  lcd.print("Calibrando...");
  Serial.print("Programa Iniciado!!!");
  sensores.calibrar();
  delay(500);
  lcd.clear();
  selecionarDispositivo(7);
  lcd.print("Calibrado!!!");
  delay(500);
  lcd.clear();
  motores.frente();
}
// void calibrarI2c(){
//   short menoresValores[2] = {999, 999}, maioresValores[2] = {0,0};
//   for(byte i=0: i<2;i++){

//   }
// }
void scrollText(String texto, int delayTime) {
  for (byte i = 0; i < 16; i++) {
    selecionarDispositivo(7);
    lcd.clear();
    lcd.setCursor(i, 0);
    lcd.print(texto);  // Exibe os próximos 16 caracteres
    delay(delayTime);
  }
}
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!sensorLaserFrente.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!sensorLaserDireita.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if (!sensorLaserRampa.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1)
      ;
  }
  
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX2
  if (!sensorLaserEsquerda.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot four VL53L0X"));
    while (1)
      ;
  }
}

void beginServos() {

#define Frequencia 50  // VALOR DA FREQUENCIA DO SERVO

  pwm.begin();                 // INICIA O OBJETO PWM
  pwm.setPWMFreq(Frequencia);  // DEFINE A FREQUENCIA DE TRABALHO DO SERVO
}
void writeServos(int nServo, int posicao) {
#define SERVOMIN 100  // VALOR PARA UM PULSO MAIOR QUE 1 mS
#define SERVOMAX 300  // VALOR PARA UM PULSO MENOR QUE 2 mS

  if (nServo == 2) {
#define SERVOMAX 83 // VALOR PARA UM PULSO MENOR QUE 2 mS
  }

  short pos = map(posicao, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(nServo, 0, pos);
}

short leituraSensorLaser(Adafruit_VL53L0X sensorLaser) {
  VL53L0X_RangingMeasurementData_t measure;
  sensorLaser.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // if not out of range
    return (measure.RangeMilliMeter);
  }
}
void iniciarSensorLaser() {

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));


  Serial.println(F("Starting..."));
  setID();
}
void ledVerde() {
  digitalWrite(ledVermelho1, 0);
  digitalWrite(ledVermelho2, 0);
  digitalWrite(ledVerde1, 1);
  digitalWrite(ledVerde2, 1);
}
void ledVermelho() {
  digitalWrite(ledVermelho1, 1);
  digitalWrite(ledVermelho2, 1);
  digitalWrite(ledVerde1, 0);
  digitalWrite(ledVerde2, 0);
}
void selecionarDispositivo(uint8_t id) {
  Wire.beginTransmission(0x70);  // A0= LOW; A1= LOW; A2= LOW
  Wire.write(1 << id);
  Wire.endTransmission();
}
short getGreen(uint8_t portaI2c) {
  selecionarDispositivo(portaI2c);
  return sensorRGB.getGreen();
}
void debugVeml6040() {
  ledVerde();
  selecionarDispositivo(0);
  Serial.print((String)sensorRGB.getRed() + "\t");
  Serial.print((String)sensorRGB.getGreen() + "\t");
  Serial.print((String)sensorRGB.getBlue() + "\t");
  Serial.print((String)sensorRGB.getWhite() + "\t\t");
  selecionarDispositivo(1);
  Serial.print((String)sensorRGB.getRed() + "\t");
  Serial.print((String)sensorRGB.getGreen() + "\t");
  Serial.print((String)sensorRGB.getBlue() + "\t");
  Serial.println((String)sensorRGB.getWhite());
}
void debugSensoresLinha() {
  for (byte i = 0; i < 7; i++) {
    Serial.print((String)sensores.leitura(i) + "\t");
  }
  Serial.println();
}
void debugSensoresLaser() {
  Serial.print((String) leituraSensorLaser(sensorLaserFrente) + "\t");Serial.print((String) leituraSensorLaser(sensorLaserRampa) + "\t");Serial.print((String) leituraSensorLaser(sensorLaserEsquerda) + "\t");Serial.println(leituraSensorLaser(sensorLaserDireita));
}
void debugSensoresSharp() {
  Serial.print((String)leituraSharp(sensorSharpEsquerda) + "\t");
  Serial.print((String)leituraSharp(sensorSharpDireita) + "\t");
  Serial.println((String)leituraSharp(sensorSharpFrente));
}
bool testeVerde(uint8_t portaI2c) {
  if (portaI2c == 0) {
    return getGreen(portaI2c) >= corteMinVml[0] && getGreen(portaI2c) <= corteMaxVml[0];
  }
  return getGreen(portaI2c) >= corteMinVml[1] && getGreen(portaI2c) <= corteMaxVml[1];
}

void fazerCurva(void (ControleMotores::*direcaoCurva)(), bool verde, byte indiceSensorLado, byte indiceSensorCurva) {
  motores.frente();
  motores.acionar(255, 255, 200);
  if (verde) {
    motores.acionar(255, 255, 160);
  }

  //Vai virar de acordo com a função passada como parametro
  (motores.*direcaoCurva)();
  //Vai virar para esquerda/direita enquanto o sensor do meio for menor que 600
  if (verde) {
    motores.acionar(255, 255, 650);
  }
  //Vai virar pra esquerda/direita enquanto o sensor do meio for maior que 350
  motores.acionarEnquantoBranco(sensores, indiceSensorCurva, 700);

  //motores.acionarEnquantoBranco(sensores, indiceSensorLado, 750);
  //Andar pra frente enquanto os sensores = {0 1 0 0 0 1 0} for menor que 600
  motores.frente();
  motores.acionar(255, 255, 85);

  lcd.clear();
}

float leituraSharp(byte portaSensor) {
  return 13 * pow(analogRead(portaSensor) * 0.0048828125, -1);
}
void desviarObstaculo() {
  byte target = 15;
  short erro = 0;
  float P = 3.5;
  short velocidadeAtual = 150;
  motores.tras();
  motores.acionar(255, 255, 1);
  while (leituraSharp(sensorSharpFrente) <= 11)
    ;
  motores.esquerda();
  motores.acionar(255, 255, 800);
  int tempoInicial = millis();
  while (true) {
    if (millis() - tempoInicial >= 5000) {
      if (sensores.leitura(0) <= 500) {
        break;
      }
    }
    erro = (leituraSharp(sensorSharpDireita) - target) * P;
    motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
  }
  motores.frente();
  motores.acionar(255, 255, 350);
  motores.esquerda();
  motores.acionarEnquantoBranco(sensores, 2, 500);
  motores.tras();
  motores.acionar(255, 255, 200);
}
void seguirLinha(bool testeVerdes = true) {
  motores.acionar(240, 240, 40);
  while (sensores.leitura(1) <= 500 && sensores.leitura(2) <= 500 && sensores.leitura(4) <= 500 && sensores.leitura(5) <= 500) {
    motores.acionar(115, 115, 40);
    Serial.println("Interseção");
    if (testeVerde(0) && testeVerde(1) && testeVerdes) {
      selecionarDispositivo(7);
      lcd.print("Beco sem saida");
      while (sensores.leitura(1) <= 500 && sensores.leitura(2) <= 500 && sensores.leitura(4) <= 500 && sensores.leitura(5) <= 500)
        ;
      motores.esquerda();
      motores.acionar(255, 255, 1600);
      motores.acionarEnquantoBranco(sensores, 2, 500);
      lcd.clear();
    }
  }
  while ((sensores.leitura(0) <= 500 || sensores.leitura(1) <= 500 || sensores.leitura(2) <= 650) && (sensores.leitura(4) >= 550 && sensores.leitura(5) >= 550 && sensores.leitura(6) >= 550)) {
    // Serial.println("Direita");
    motores.direita();
    if (sensores.leitura(1) <= 650 && sensores.leitura(0) >= 650 && testeVerde(1) && testeVerdes) {
      selecionarDispositivo(7);
      lcd.print("Verde Direita");
      Serial.println("Verde Direita");
      testeVerdeDireita = 0;
      fazerCurva(&ControleMotores::direita, true, 0, 4);
      motores.esquerda();
      motores.acionar(115, 115, 100);
    }
  }
  while ((sensores.leitura(4) <= 650 || sensores.leitura(5) <= 500 || sensores.leitura(6) <= 500) && (sensores.leitura(0) >= 550 && sensores.leitura(1) >= 550 && sensores.leitura(2) >= 550)) {
    // Serial.println("Esquerda");
    motores.esquerda();
    if (sensores.leitura(5) <= 650 && sensores.leitura(6) >= 650 && testeVerde(0) && testeVerdes) {
      selecionarDispositivo(7);
      lcd.print("Verde Esquerda");
      Serial.println("Verde Esquerda");
      testeVerdeEsquerda = 0;
      fazerCurva(&ControleMotores::esquerda, true, 0, 2);
      motores.direita();
      motores.acionar(115, 115, 100);
    }
  }

  lcd.clear();
  motores.frente();
}
void garraInicial() {
  writeServos(0, 180);
  writeServos(1, 0);
  delay(1000);
  writeServos(2, 180);
  delay(1000);
}
void pegarObjeto(bool cubo = false) {
  motores.tras();
  motores.acionar(255, 255, 150);
  motores.parado();
  writeServos(2, 0);
  delay(1000);
  motores.frente();
  motores.acionar(255, 255, 60);
  motores.parado();
  writeServos(0, 55);
  writeServos(1, 125);
  if (cubo) {
    writeServos(0, 40);
    writeServos(1, 140);
  }
  delay(1000);
}
void pegarCubo() {
  pegarObjeto(true);
  writeServos(2, 180);
  delay(10000);
  writeServos(0, 180);
  writeServos(1, 180);
  delay(1000);
  garraInicial();
}
void pegarBolinha() {
  pegarObjeto(false);
  if (!digitalRead(identificadorBolinhas)) {
    Serial.println("Bolinha Viva");
  } else {
    Serial.println("Bolinha Morta");
  }
  garraInicial();
}
void resgate() {
  byte target = 90;
  short erro = 0, P = 3;
  short velocidadeAtual = 180;
  Adafruit_VL53L0X sensorLaserPID = sensorLaserDireita;
  byte sensorSharpArenas = sensorSharpDireita;
  void (ControleMotores::*direcaoCurva)() = &ControleMotores::esquerda;
  if (leituraSharp(sensorSharpEsquerda) < leituraSharp(sensorSharpDireita)) {
    P *= -1;
    sensorLaserPID = sensorLaserEsquerda;
    sensorSharpArenas = sensorSharpEsquerda;
    direcaoCurva = &ControleMotores::direita;
  }
  while (true) {
    for (byte i = 0; i < 4; i++) {
      while (leituraSharp(sensorSharpFrente) >= 7) {
        erro = (leituraSensorLaser(sensorLaserPID) - target) * P;
        motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
        // if (leituraSensorLaser(indiceSensorLaser) <= 180) {
        //   pegarBolinhasLaterais(direcaoCurva);
        // }
        Serial.print((String) (velocidadeAtual - erro) + "\t");Serial.println((String) leituraSensorLaser(sensorLaserPID) + "\t");
      }
      (motores.*direcaoCurva)();
      motores.acionar(255, 255, 1050);
    }
  }
}

void pegarBolinhasLaterais(void (ControleMotores::*direcaoCurva)()) {
  motores.parado();
  motores.tras();
  motores.acionar(255, 255, 90);
  (motores.*direcaoCurva)();
  motores.acionar(255, 255, 575);
  motores.parado();
  delay(2000);
  if (direcaoCurva == &ControleMotores::esquerda) {
    motores.direita();
  } else {
    motores.esquerda();
  }
  motores.acionar(255, 255, 585);
}
void loop() {
  //--------------------------VALENDO--------------------------
  // seguirLinha();
  // if (leituraSensorLaser(1) <= 75) {
  //   motores.frente();
  //   motores.acionar(255, 255, 50);
  //   motores.parado();
  //   if (leituraSensorLaser(2) <= 75) {
  //     if (leituraSharp(sensorSharpFrente) <= 5) {
  //       selecionarDispositivo(7);
  //       lcd.print("Obstaculo");
  //       desviarObstaculo();
  //     } else {
  //       Serial.println("Rampa");
  //       selecionarDispositivo(7);
  //       lcd.print("Rampa");
  //       seguirLinha(false);
  //       delay(1000);
  //     }
  //   } else {
  //     Serial.println("Cubo" + (String)leituraSensorLaser(1));
  //     selecionarDispositivo(7);
  //     lcd.print("Cubo");
  //     pegarCubo();
  //     lcd.clear();
  //   }
  // }
  //--------------------------VALENDO--------------------------
  // resgate();
  // // debugVeml6040();
  // // debugSensoresLinha();
  debugSensoresLaser();
  // debugSensoresSharp();
  // pegarBolinha();
}