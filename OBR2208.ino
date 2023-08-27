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
const byte pinosLox[4] = { SHT_LOX1, SHT_LOX2, SHT_LOX3, SHT_LOX4 };


// objects for the vl53l0x
Adafruit_VL53L0X sensorLaserFrente = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserDireita = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserRampa = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLaserEsquerda = Adafruit_VL53L0X();
const Adafruit_VL53L0X listaSensoresLaser[4] = { sensorLaserFrente, sensorLaserDireita, sensorLaserRampa, sensorLaserEsquerda };

//Inicializar sensor sharp da frente
#define sensorSharpFrente A14
#define sensorSharpDireita A15
#define sensorSharpEsquerda A13
#define sensorSharpGarra A9

//Ligar leds vermelhos da placa seguidor de linha
#define ledVermelho1 53
#define ledVermelho2 50
#define ledVerde1 49
#define ledVerde2 51

//Definir pino para receber o PULLUP da garra
#define identificadorBolinhas 25
//Botao de calibrar para os sensores de linha
#define botaoCalibrar 52

//Inicializar os motores principais
ControleMotores motores;

//Quantidade de sensores usados para seguir linha
#define quantidadeSensores 7
const byte PORTAS_ANALOGICOS[quantidadeSensores] = { A0, A1, A2, A3, A4, A6, A5 };
//Instancia sensores de linha usando a biblioteca 'ControleSensores':
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
  pinMode(sensorSharpGarra, INPUT);

  //Definir pino para identificar se a bolinha é viva ou morta
  pinMode(identificadorBolinhas, INPUT_PULLUP);

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

  // selecionarDispositivo(7);
  // scrollText("OBR-2208", 270);
  // lcd.clear();

  iniciarSensorLaser();
  garraInicial();

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
}
void scrollText(String texto, int delayTime) {
  for (byte i = 0; i < 16; i++) {
    selecionarDispositivo(7);
    lcd.clear();
    lcd.setCursor(i, 0);
    lcd.print(texto);  // Exibe os próximos 16 caracteres
    delay(delayTime);
  }
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
void setID() {
  // all reset
  for (byte i : pinosLox) {
    digitalWrite(i, LOW);
  }
  delay(10);
  // all unreset
  for (byte i : pinosLox) {
    digitalWrite(i, HIGH);
  }
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX3, LOW);

  for (byte i = 0; i < 4; i++) {
    if (!listaSensoresLaser[i].begin(pinosLox[i])) {
      Serial.println("Failed to boot " + (String)(i + 1) + " VL53L0X");
      while (1)
        ;
    }
    delay(10);
    if (i == 3) { break; }
    digitalWrite((pinosLox[i + 1]), HIGH);
    delay(10);
  }
}
void iniciarSensorLaser() {
  for (byte i : pinosLox) {
    pinMode(i, OUTPUT);
  }

  Serial.println(F("Shutdown pins inited..."));
  for (byte i : pinosLox) {
    digitalWrite(i, LOW);
  }

  Serial.println(F("Both in reset mode...(pins are low)"));


  Serial.println(F("Starting..."));
  setID();
}
void beginServos() {

#define Frequencia 50  // VALOR DA FREQUENCIA DO SERVO

  pwm.begin();                 // INICIA O OBJETO PWM
  pwm.setPWMFreq(Frequencia);  // DEFINE A FREQUENCIA DE TRABALHO DO SERVO
}
void writeServos(int nServo, int posicao) {
#define SERVOMIN 120
#define SERVOMAX 300
  if (nServo == 14) {
#define SERVOMIN 83  // VALOR PARA UM PULSO MENOR QUE 2 mS
#define SERVOMAX 400
  }

  short pos = map(posicao, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(nServo, 0, pos);
}
void garraAberta() {
  writeServos(0, 115);
  writeServos(1, 20);
}
void garraFechada(bool cubo = false) {
  if (cubo) {
    writeServos(0, 40);
    writeServos(1, 80);
    delay(1000);
  } else {
    short posicaoGarraDireita = 20;
    for (byte i = 115; i < 115; i += 15) {
      writeServos(0, i);
      if (posicaoGarraDireita != 80) {
        writeServos(1, posicaoGarraDireita += 15);
      }
      delay(100);
    }
  }
}
void garraBaixa() {
  writeServos(14, 180);
  delay(1000);
}
void garraAlta() {
  writeServos(14, 0);
  delay(1000);
}
void garraInicial(bool resgate = false) {
  garraAberta();
  if (resgate) {
    garraBaixa();
  } else {
    garraAlta();
  }
}
void pegarObjeto(bool cubo = false) {
  garraAberta();
  garraBaixa();
  motores.frente();
  motores.acionar(150, 150, 120);
  motores.parado();
  garraFechada(cubo);
  garraAlta();
  motores.tras();
  motores.acionar(150, 150, 120);
  motores.parado();
}
void pegarCubo() {
  pegarObjeto(true);
}
void pegarBolinha() {
  pegarObjeto();
  if (!digitalRead(identificadorBolinhas)) {
    writeServos(0, 115);
    writeServos(1, 100);
    delay(1000);
  } else {
    writeServos(0, 30);
    writeServos(1, 20);
    delay(1000);
  }
  garraAberta();
  garraBaixa();
}
void selecionarDispositivo(uint8_t id) {
  Wire.beginTransmission(0x70);  // A0= LOW; A1= LOW; A2= LOW
  Wire.write(1 << id);
  Wire.endTransmission();
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
  Serial.print((String)leituraSensorLaser(sensorLaserFrente) + "\t");
  Serial.print((String)leituraSensorLaser(sensorLaserRampa) + "\t");
  Serial.print((String)leituraSensorLaser(sensorLaserEsquerda) + "\t");
  Serial.println(leituraSensorLaser(sensorLaserDireita));
}
void debugSensoresSharp() {
  Serial.print((String)leituraSharp(sensorSharpEsquerda) + "\t");
  Serial.print((String)leituraSharp(sensorSharpDireita) + "\t");
  Serial.print((String)leituraSharp(sensorSharpFrente) + "\t");
  Serial.println((String)leituraSharp(sensorSharpGarra));
}
short getGreen(uint8_t portaI2c) {
  selecionarDispositivo(portaI2c);
  return sensorRGB.getGreen();
}
short leituraSensorLaser(Adafruit_VL53L0X sensorLaser) {
  VL53L0X_RangingMeasurementData_t measure;
  sensorLaser.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // if not out of range
    return (measure.RangeMilliMeter);
  }
  return 8000;
}
float leituraSharp(byte portaSensor) {
  return 13 * pow(analogRead(portaSensor) * 0.0048828125, -1);
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
void resgate() {
  garraInicial(true);
  byte target = 130;
  float P = -5.5;
  short erro = 0, velocidadeAtual = 180, curvas = 0, posicoesArenas[3] = { 0, 0, 0 };
  Adafruit_VL53L0X sensorLaserPID = sensorLaserEsquerda, sensorLaserBolinhas = sensorLaserDireita;
  byte sensorSharpArenas = sensorSharpEsquerda;
  void (ControleMotores::*direcaoCurva)() = &ControleMotores::direita, (ControleMotores::*curvaOposta)() = &ControleMotores::esquerda;
  if (leituraSharp(sensorSharpEsquerda) < leituraSharp(sensorSharpDireita)) {
    P *= -1;
    sensorLaserPID = sensorLaserDireita;
    sensorLaserBolinhas = sensorLaserEsquerda;
    sensorSharpArenas = sensorSharpDireita;
    direcaoCurva = &ControleMotores::esquerda;
    curvaOposta = &ControleMotores::direita;
  }
  // motores.frente();
  // motores.acionar(255,255,800);
  // motores.esquerda();
  // motores.acionar(255,255,800);
  motores.frente();
  motores.acionar(255, 255, 1);
  Serial.println("ENTRANDO NO RESGATE");
  while (leituraSensorLaser(sensorLaserPID) >= target);
  motores.acionar(255, 255, 600);
  while (true) {
    long tempo = millis();
    while (leituraSharp(sensorSharpFrente) >= 8.50) {
      if (leituraSharp(sensorSharpArenas) >= 16.5) {
        long tempoSeguirParedes = millis();
        while (millis() - tempoSeguirParedes <= 1500) {
          erro = (leituraSensorLaser(sensorLaserPID) - target) * P;
          motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
        }
        pegarBolinha();
        (motores.*direcaoCurva)();
        motores.acionar(255, 255, 1200);
        pegarBolinha();
        motores.tras();
        motores.acionar(255, 255, 600);
        //entregarBolinhas
        for (byte i = 0; i <= 2; i++) {
          motores.frente();
          delay(200);
          motores.tras();
          delay(200);
        }
        motores.frente();
        motores.acionar(255, 255, 600);
        (motores.*curvaOposta)();
        motores.acionar(255, 255, 1100);
        tempoSeguirParedes = millis();
        while (millis() - tempoSeguirParedes <= 4000) {
          erro = (leituraSensorLaser(sensorLaserPID) - target) * P;
          motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
        }
        tempoSeguirParedes = 0;
        curvas++;
        switch (curvas) {
          case 1:
            posicoesArenas[0] = 1;
            break;
          case 2:
            posicoesArenas[1] = 1;
            break;
          case 3:
            posicoesArenas[2] = 1;
            break;
        }
      }
      if(leituraSensorLaser(sensorLaserBolinhas) <= 150){
        pegarBolinhasLaterais(direcaoCurva);
      }
      if (curvas == 3) {
        long tempoSeguirParedes = millis();
        while (millis() - tempoSeguirParedes <= 1800) {
          erro = (leituraSensorLaser(sensorLaserPID) - target) * P;
          motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
        }
        pegarBolinha();
        writeServos(14, 170);
        garraAberta();
        (motores.*direcaoCurva)();
        motores.acionar(255, 255, 1300);
        motores.frente();
        motores.acionar(255, 255, 3000);
        if (posicoesArenas[0] == 1) {
          motores.esquerda();
          motores.acionar(255, 255, 850);
          motores.frente();
          while (leituraSharp(sensorSharpGarra) >= 5)
            ;
          pegarBolinha();
          writeServos(14, 170);
          garraAberta();
          motores.direita();
          motores.acionar(255, 255, 2500);
          motores.tras();
          motores.acionar(255, 255, 600);
          //entregarBolinhas
          for (byte i = 0; i <= 2; i++) {
            motores.frente();
            delay(200);
            motores.tras();
            delay(200);
          }
          if (posicoesArenas[1] == 1) {
            motores.esquerda();
            motores.acionar(255, 255, 600);
            motores.frente();
            while (leituraSharp(sensorSharpGarra) >= 5)
              ;
            pegarBolinha();
            writeServos(14, 170);
            garraAberta();
            motores.direita();
            motores.acionar(255, 255, 2500);
            motores.tras();
            motores.acionar(255, 255, 600);
            //entregarBolinhas
            for (byte i = 0; i <= 2; i++) {
              motores.frente();
              delay(200);
              motores.tras();
              delay(200);
            }
          } else {
            motores.frente();
            while (leituraSharp(sensorSharpGarra) >= 5)
              ;
            pegarBolinha();
            writeServos(14, 170);
            garraAberta();
            motores.direita();
            motores.acionar(255, 255, 2500);
            motores.tras();
            motores.acionar(255, 255, 700);
            //entregarBolinhas
            for (byte i = 0; i <= 2; i++) {
              motores.frente();
              delay(200);
              motores.tras();
              delay(200);
            }
          }
        } else if (posicoesArenas[1] == 1) {
          motores.direita();
          motores.acionar(255, 255, 850);
        }
      }
      erro = (leituraSensorLaser(sensorLaserPID) - target) * P;
      motores.acionar(velocidadeAtual + erro, velocidadeAtual - erro);
    }
    motores.tras();
    motores.acionar(255, 255, 475);
    motores.parado();
    pegarBolinha();
    (motores.*direcaoCurva)();
    motores.acionar(255, 255, 850);
    motores.frente();
    motores.acionar(255, 255, 900);
    (motores.*direcaoCurva)();
    motores.acionar(255, 255, 420);
    motores.parado();
    curvas++;
  }
}

void pegarBolinhasLaterais(void (ControleMotores::*direcaoCurva)()) {
  motores.parado();
  motores.tras();
  motores.acionar(255, 255, 200);
  (motores.*direcaoCurva)();
  motores.acionar(255, 255, 1);
  long tempoGiro = millis();
  while (leituraSharp(sensorSharpGarra) >= 9 || millis() - tempoGiro >= 1300);
  long tempoParada = millis();
  motores.parado();
  motores.frente();
  motores.acionar(255, 255, 450);
  motores.parado();
  pegarBolinha();
  motores.tras();
  motores.acionar(255, 255, 450);
  motores.parado();
  if (direcaoCurva == &ControleMotores::esquerda) {
    motores.direita();
  } else {
    motores.esquerda();
  }
  motores.acionar(255, 255, tempoParada - tempoGiro);
  motores.parado();
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
  resgate();
  // // debugVeml6040();
  // // debugSensoresLinha();
  // debugSensoresLaser();
  // debugSensoresSharp();
  // pegarBolinha();
}