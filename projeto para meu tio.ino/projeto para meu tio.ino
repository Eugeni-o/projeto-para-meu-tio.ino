// Inclusão das Bibliotecas
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Arduino.h>
#include <EEPROM.h>

// regras
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/50, /* data=*/48, /* CS=*/46, /* reset=*/52);
Adafruit_TCS34725 sensTCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X);

//definir pinos
#define buzzer 41
#define Ledluz 13
#define luzfundLCD 2
#define LDR A14
#define botCima 49
#define botBaixo 51
#define botDir 53
#define botEsq 47
#define botEnt 45
#define botCan 43
#define rele1 44
#define rele2 42
#define rele3 40
#define rele4 38
#define rele1C 39
#define rele2C 37
#define rele3C 36
#define rele4C 35

//outars variaveis
#define MPU 0x68

//variaveis de atuadores e sensores
float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;
float AccX_offset, AccY_offset, AccZ_offset;
float GyrX_offset, GyrY_offset, GyrZ_offset;
uint16_t r1, g1, b1, c1, TC1, LUX1;
uint16_t r2, g2, b2, c2, TC2, LUX2;
uint16_t r3, g3, b3, c3, TC3, LUX3;
uint16_t r4, g4, b4, c4, TC4, LUX4;
uint16_t controle_dia_R, controle_dia_G, controle_dia_B;
uint16_t controle_noite_R, controle_noite_G, controle_noite_B;
int senCor;
bool botbeep;

//calibragem da tela
const int numReadings = 10;    // Defina o tamanho da média móvel
int LDRReadings[numReadings];  // Armazena as últimas leituras do LDR
int LDRIndex = 0;              // Índice para armazenar a próxima leitura do LDR
int LDRf;
int atraso;
char buffer[12];

//configurações
int minBrilho;
bool luzAuto;

//menus
int menu = 1;
//menus da tela
bool menuAtivo = true;
bool menuConfigAtivo = false;
bool menuCalibragemAtivo = false;
bool menuAtrasoAtivo = false;
bool luzFundominAtivo = false;
bool luzAutoAtivo = false;
bool calibibralAnguloAtivo = false;
bool sombeepAtivo = false;
bool calibibralCorAtivo = false;
bool senCorAtivo = false;

/* endereços de eepron:
0 = luz de fundo minimo
1 = atraso
2 = luz auto
3-6 = AccX_offset
7-10 = AccY_offset
11-14 = AccZ_offset
15-18 = GyrX_offset
19-22 = GyrY_offset
23-26 = GyrZ_offset
27 = botbeep
28-31 = controle_dia_R
32-35 = controle_dia_G
36-39 = controle_dia_B
40-43 = controle_noite_R
44-47 = controle_noite_G
48-51 = controle_noite_B
52 = sensibiidade da cor
*/
// funções
void verificarRele (int nomeDoRele, int nomeRetornoRele, bool estado) {

}
bool estaDentroDoIntervalo(uint16_t r, uint16_t g, uint16_t b, uint16_t controle_R, uint16_t controle_G, uint16_t controle_B, int sensibilidade) {
  return (r >= controle_R - sensibilidade && r <= controle_R + sensibilidade) && (g >= controle_G - sensibilidade && g <= controle_G + sensibilidade) && (b >= controle_B - sensibilidade && b <= controle_B + sensibilidade);
}
void atualizarRele() {
  int sensibilidade = 10;  // Exemplo de valor para sensibilidade

  // Verifica o sensor 1
  if (estaDentroDoIntervalo(r1, g1, b1, controle_dia_R, controle_dia_G, controle_dia_B, sensibilidade) || estaDentroDoIntervalo(r1, g1, b1, controle_noite_R, controle_noite_G, controle_noite_B, sensibilidade)) {
    digitalWrite(rele1, LOW);
  } else {
    digitalWrite(rele1, HIGH);
  }

  // Verifica o sensor 2
  if (estaDentroDoIntervalo(r2, g2, b2, controle_dia_R, controle_dia_G, controle_dia_B, sensibilidade) || estaDentroDoIntervalo(r2, g2, b2, controle_noite_R, controle_noite_G, controle_noite_B, sensibilidade)) {
    digitalWrite(rele2, LOW);
  } else {
    digitalWrite(rele2, HIGH);
  }

  // Verifica o sensor 3
  if (estaDentroDoIntervalo(r3, g3, b3, controle_dia_R, controle_dia_G, controle_dia_B, sensibilidade) || estaDentroDoIntervalo(r3, g3, b3, controle_noite_R, controle_noite_G, controle_noite_B, sensibilidade)) {
    digitalWrite(rele3, LOW);
  } else {
    digitalWrite(rele3, HIGH);
  }

  // Verifica o sensor 4
  if (estaDentroDoIntervalo(r4, g4, b4, controle_dia_R, controle_dia_G, controle_dia_B, sensibilidade) || estaDentroDoIntervalo(r4, g4, b4, controle_noite_R, controle_noite_G, controle_noite_B, sensibilidade)) {
    digitalWrite(rele4, LOW);
  } else {
    digitalWrite(rele4, HIGH);
  }
}
void esperarBotao(char botao) {
  while (true) {
    if (digitalRead(botao)) {
      break;
    }
    delay(20);
  }
}
void beep() {
  if (botbeep) {
    tone(buzzer, 1200);
    delay(50);
    noTone(buzzer);
  }
}
void EEPROM_writeFloat(int address, float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(float); i++) {
    EEPROM.write(address + i, *p++);
  }
}
float EEPROM_readFloat(int address) {
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(float); i++) {
    *p++ = EEPROM.read(address + i);
  }
  return value;
}
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}
void InicioSensesor() {
  // Inicializa o MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Configura Giroscópio para fundo de escala desejado
  /*
    Wire.write(0b00000000); // fundo de escala em +/-250°/s
    Wire.write(0b00001000); // fundo de escala em +/-500°/s
    Wire.write(0b00010000); // fundo de escala em +/-1000°/s
    Wire.write(0b00011000); // fundo de escala em +/-2000°/s
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0b00000000);  // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();
  // Configura Acelerometro para fundo de escala desejado
  /*
      Wire.write(0b00000000); // fundo de escala em +/-2g
      Wire.write(0b00001000); // fundo de escala em +/-4g
      Wire.write(0b00010000); // fundo de escala em +/-8g
      Wire.write(0b00011000); // fundo de escala em +/-16g
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000000);  // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();
  //Inicializa o TCS34725 1
  TCA9548A(6);
  if (sensTCS.begin()) {
    Serial.println("Sensor 1 conectado");
  } else {  //Se o sensor não conectou , mostra a mensagem de erro
    Serial.println("Sensor 1 com problema");
  }
  //Inicializa o TCS34725 2
  TCA9548A(2);
  if (sensTCS.begin()) {
    Serial.println("Sensor 2 conectado");
  } else {  //Se o sensor não conectou , mostra a mensagem de erro
    Serial.println("Sensor 2 com problema");
  }
  //Inicializa o TCS34725 3
  TCA9548A(3);
  if (sensTCS.begin()) {
    Serial.println("Sensor 3 conectado");
  } else {  //Se o sensor não conectou , mostra a mensagem de erro
    Serial.println("Sensor 3 com problema");
  }
  //Inicializa o TCS34725 4
  TCA9548A(1);
  if (sensTCS.begin()) {
    Serial.println("Sensor 4 conectado");
  } else {  //Se o sensor não conectou , mostra a mensagem de erro
    Serial.println("Sensor 4 com problema");
  }
}
void LerSensores() {
  int ciclos = 0;
  //chamar sensores
  // Comandos para iniciar transmissão de dados
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // Solicita os dados ao sensor

  // Armazena o valor dos sensores nas variaveis correspondentes
  AccX = Wire.read() << 8 | Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AccY = Wire.read() << 8 | Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ = Wire.read() << 8 | Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = Wire.read() << 8 | Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyrX = Wire.read() << 8 | Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyrY = Wire.read() << 8 | Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyrZ = Wire.read() << 8 | Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Imprime na Serial os valores obtidos
  /* Alterar divisão conforme fundo de escala escolhido:
      Acelerômetro
      +/-2g = 16384
      +/-4g = 8192
      +/-8g = 4096
      +/-16g = 2048

      Giroscópio
      +/-250°/s = 131
      +/-500°/s = 65.6
      +/-1000°/s = 32.8
      +/-2000°/s = 16.4

      Contas necessarias
  */
  ciclos++;
  if (ciclos >= 50) {
    TCA9548A(6);  //
    sensTCS.getRawData(&r1, &g1, &b1, &c1);
    TC1 = sensTCS.calculateColorTemperature(r1, g1, b1);
    LUX1 = sensTCS.calculateLux(r1, g1, b1);
    TCA9548A(2);  //
    sensTCS.getRawData(&r2, &g2, &b2, &c2);
    TC2 = sensTCS.calculateColorTemperature(r2, g2, b2);
    LUX2 = sensTCS.calculateLux(r2, g2, b2);
    TCA9548A(3);  //
    sensTCS.getRawData(&r3, &g3, &b3, &c3);
    TC3 = sensTCS.calculateColorTemperature(r3, g3, b3);
    LUX3 = sensTCS.calculateLux(r3, g3, b3);
    TCA9548A(1);  //
    sensTCS.getRawData(&r4, &g4, &b4, &c4);
    TC4 = sensTCS.calculateColorTemperature(r4, g4, b4);
    LUX4 = sensTCS.calculateLux(r4, g4, b4);
    ciclos == 0;
  }
  AccX = AccX / 16384;
  AccY = AccY / 16384;
  AccZ = AccZ / 16384;
  GyrX = GyrX / 131;
  GyrY = GyrY / 131;
  GyrZ = GyrZ / 131;
  Temp = Temp / 340.00 + 36.53;
  AccX -= AccX_offset;
  AccY -= AccY_offset;
  AccZ -= AccZ_offset;
  GyrX -= GyrX_offset;
  GyrY -= GyrY_offset;
  GyrZ -= GyrZ_offset;
}
void Configpinos() {
  u8g2.begin(/*Select=*/49, /*Right/Next=*/53, /*Left/Prev=*/47, /*Up=*/49, /*Down=*/51, /*Home/Cancel=*/43);
  pinMode(luzfundLCD, OUTPUT);
  pinMode(Ledluz, OUTPUT);
  pinMode(botCima, INPUT);
  pinMode(botBaixo, INPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(rele3, OUTPUT);
  pinMode(rele4, OUTPUT);
}
void UIcontrole() {
  //Serial.print(" menu:");
  //Serial.print(menu);
  if (luzAuto == 1) {
    //brilho linear
    // Leitura do LDR
    LDRf = 1023 - analogRead(LDR);

    // Adiciona a nova leitura ao array e remove a mais antiga
    LDRReadings[LDRIndex] = LDRf;
    LDRIndex = (LDRIndex + 1) % numReadings;

    // Calcula a média das últimas leituras
    int LDRAverage = 0;
    for (int i = 0; i < numReadings; i++) {
      LDRAverage += LDRReadings[i];
    }
    LDRAverage /= numReadings;

    // Mapeia a média para o intervalo de brilho
    int brightness = map(LDRAverage, 0, 1023, 0, 255);

    // Limita o brilho mínimo
    if (brightness < minBrilho) {
      brightness = minBrilho;
    }

    // Define o brilho do LCD
    analogWrite(luzfundLCD, brightness);
  } else if (luzAuto == 0) {
    analogWrite(luzfundLCD, minBrilho);
  }
  if (menuAtivo) {
    if (menu <= 0) {
      menu = 3;
    } else if (menu == 1) {
      UIvisual(1, 0, 1);
    } else if (menu == 2) {
      UIvisual(1, 0, 2);
      if (digitalRead(botEnt)) {
        beep();
        menu = 1;
        menuCalibragemAtivo = true;
        menuAtivo = false;
      }
    } else if (menu == 3) {
      UIvisual(1, 0, 3);
      if (digitalRead(botEnt)) {
        beep();
        menu = 1;
        menuConfigAtivo = true;
        menuAtivo = false;
      }
    } else if (menu >= 4) {
      menu = 1;
    }
  } else if (menuConfigAtivo) {
    if (menu <= 0) {
      menu = 5;
    } else if (menu == 1) {
      UIvisual(3, 0, 1);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menu = 1;
        menuConfigAtivo = false;
        menuAtrasoAtivo = true;
      }
    } else if (menu == 2) {
      UIvisual(3, 0, 2);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menu = 1;
        menuConfigAtivo = false;
        luzFundominAtivo = true;
      }
    } else if (menu == 3) {
      UIvisual(3, 0, 3);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menu = 1;
        menuConfigAtivo = false;
        luzAutoAtivo = true;
      }
    } else if (menu == 4) {
      UIvisual(3, 0, 4);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menu = 1;
        menuConfigAtivo = false;
        sombeepAtivo = true;
      }
    } else if (menu == 5) {
      UIvisual(3, 0, 5);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menu = 1;
        menuConfigAtivo = false;
        senCorAtivo = true;
      }
    } else if (menu >= 6) {
      menu = 1;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      menu = 1;
      menuConfigAtivo = false;
      menuAtivo = true;
    }
  } else if (menuCalibragemAtivo) {
    if (menu <= 0) {
      menu = 3;
    } else if (menu == 1) {
      UIvisual(2, 0, 1);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menuCalibragemAtivo = false;
        calibibralAnguloAtivo = true;
        menu = 1;
      }
    } else if (menu == 2) {
      UIvisual(2, 0, 2);
      if (digitalRead(botEnt)) {
        beep();
        delay(atraso);
        menuCalibragemAtivo = false;
        calibibralCorAtivo = true;
        menu = 1;
      }
    } else if (menu >= 3) {
      menu = 1;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      menu = 1;
      menuCalibragemAtivo = false;
      menuAtivo = true;
    }
  } else if (luzFundominAtivo) {
    UIvisual(3, 2, 1);
    if (minBrilho <= -1) {
      minBrilho = 255;
    } else if (digitalRead(botDir)) {
      beep();
      minBrilho++;
      delay(atraso);
    } else if (digitalRead(botEsq)) {
      beep();
      minBrilho--;
      delay(atraso);
    } else if (minBrilho >= 256) {
      minBrilho = 0;
    }
    String(minBrilho).toCharArray(buffer, sizeof(buffer));
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      EEPROM.write(0, minBrilho);
      menu = 1;
      menuConfigAtivo = true;
      luzFundominAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      minBrilho = EEPROM.read(0);
      menu = 1;
      menuConfigAtivo = true;
      luzFundominAtivo = false;
    }
  } else if (menuAtrasoAtivo) {
    UIvisual(3, 1, 1);
    if (atraso <= -1) {
      atraso = 255;
    } else if (digitalRead(botDir)) {
      beep();
      atraso++;
      delay(atraso);
    } else if (digitalRead(botEsq)) {
      beep();
      atraso--;
      delay(atraso);
    } else if (atraso >= 10000) {
      atraso = 0;
    }
    String(atraso).toCharArray(buffer, sizeof(buffer));
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      EEPROM.write(1, atraso);
      menu = 1;
      menuConfigAtivo = true;
      menuAtrasoAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      atraso = EEPROM.read(1);
      menu = 1;
      menuConfigAtivo = true;
      menuAtrasoAtivo = false;
    }
  } else if (luzAutoAtivo) {
    UIvisual(3, 3, 1);
    if (digitalRead(botDir)) {
      beep();
      luzAuto = 1;
      delay(atraso);
    } else if (digitalRead(botEsq)) {
      beep();
      luzAuto = 0;
      delay(atraso);
    }
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      EEPROM.write(2, luzAuto);
      menu = 1;
      luzAutoAtivo = false;
      menuConfigAtivo = true;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      luzAuto = EEPROM.read(2);
      menu = 1;
      luzAutoAtivo = false;
      menuConfigAtivo = true;
    }
  } else if (sombeepAtivo) {
    UIvisual(3, 4, 1);
    if (digitalRead(botDir)) {
      beep();
      botbeep = 1;
      delay(atraso);
    } else if (digitalRead(botEsq)) {
      beep();
      botbeep = false;
      delay(atraso);
    }
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      EEPROM.write(27, botbeep);
      menu = true;
      menuConfigAtivo = true;
      sombeepAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      botbeep = EEPROM.read(27);
      menu = 1;
      menuConfigAtivo = true;
      sombeepAtivo = false;
    }
  } else if (calibibralAnguloAtivo) {
    UIvisual(2, 1, 1);
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      AccX = AccX_offset;
      AccY = AccY_offset;
      AccZ = AccZ_offset;
      GyrX = GyrX_offset;
      GyrY = GyrY_offset;
      GyrZ = GyrZ_offset;
      EEPROM_writeFloat(3, AccX_offset);
      EEPROM_writeFloat(7, AccY_offset);
      EEPROM_writeFloat(11, AccZ_offset);
      EEPROM_writeFloat(15, GyrX_offset);
      EEPROM_writeFloat(19, GyrY_offset);
      EEPROM_writeFloat(23, GyrZ_offset);
      menu = 1;
      menuCalibragemAtivo = true;
      calibibralAnguloAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      menu = 1;
      menuCalibragemAtivo = true;
      calibibralAnguloAtivo = false;
    }
  } else if (calibibralCorAtivo) {
    UIvisual(2, 1, 2);
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(5, 6, "Calibrando cor modo dia");
      } while (u8g2.nextPage());
      r4 = controle_dia_R;
      g4 = controle_dia_G;
      b4 = controle_dia_B;
      delay(1000);
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(5, 6, "Calibrando cor modo dia");
        u8g2.drawStr(5, 15, "Calibrando cor modo noite");
      } while (u8g2.nextPage());
      digitalWrite(Ledluz, HIGH);
      r4 = controle_noite_R;
      g4 = controle_noite_G;
      b4 = controle_noite_B;
      delay(1000);
      do {
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(5, 6, "Calibrando cor modo dia");
        u8g2.drawStr(5, 15, "Calibrando cor modo noite");
        u8g2.drawStr(5, 24, "Salvando");
        digitalWrite(Ledluz, LOW);
      } while (u8g2.nextPage());
      EEPROM_writeFloat(28, controle_dia_R);
      EEPROM_writeFloat(32, controle_dia_G);
      EEPROM_writeFloat(36, controle_dia_B);
      EEPROM_writeFloat(40, controle_noite_R);
      EEPROM_writeFloat(44, controle_noite_G);
      EEPROM_writeFloat(48, controle_noite_B);
      delay(500);
      do {
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(5, 6, "Calibrando cor modo dia");
        u8g2.drawStr(5, 15, "Calibrando cor modo noite");
        u8g2.drawStr(5, 24, "Salvando");
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(5, 60, "PRONTO");
        delay(1000);
      } while (u8g2.nextPage());
      menu = 1;
      menuCalibragemAtivo = true;
      calibibralCorAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      menu = 1;
      menuCalibragemAtivo = true;
      calibibralCorAtivo = false;
    }
  } else if (senCorAtivo) {
    UIvisual(3, 5, 1);
    if (senCor <= -1) {
      senCor = 255;
    } else if (digitalRead(botDir)) {
      beep();
      senCor++;
      delay(atraso);
    } else if (digitalRead(botEsq)) {
      beep();
      senCor--;
      delay(atraso);
    } else if (senCor >= 256) {
      senCor = 0;
    }
    String(senCor).toCharArray(buffer, sizeof(buffer));
    if (digitalRead(botEnt)) {
      beep();
      delay(atraso);
      EEPROM.write(52, senCor);
      menu = 1;
      menuConfigAtivo = true;
      senCorAtivo = false;
    }
    if (digitalRead(botCan)) {
      beep();
      delay(atraso);
      atraso = EEPROM.read(52);
      menu = 1;
      menuConfigAtivo = true;
      senCorAtivo = false;
    }
  }
  //deteção de botões
  if (digitalRead(botBaixo)) {
    beep();
    menu++;
    delay(atraso);
  } else if (digitalRead(botCima)) {
    beep();
    menu--;
    delay(atraso);
  }
}
void UIvisual(uint8_t menu1, uint8_t menu2, uint8_t opcao) {
  Serial.print(" menu1:");
  Serial.print(menu1);
  Serial.print(" menu2:");
  Serial.print(menu2);
  Serial.print(" opcao");
  Serial.println(opcao);
  if (menu1 == 1 && menu2 == 0 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 7, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Corte de Linha");
      u8g2.drawStr(5, 16, "Calibragem");
      u8g2.drawStr(5, 25, "Ajustes");
    } while (u8g2.nextPage());
  } else if (menu1 == 1 && menu2 == 0 && opcao == 2) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 7, "Corte de Linha");
      u8g2.drawButtonUTF8(5, 16, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Calibragem");
      u8g2.drawStr(5, 25, "Ajustes");
    } while (u8g2.nextPage());
  } else if (menu1 == 1 && menu2 == 0 && opcao == 3) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 7, "Corte de Linha");
      u8g2.drawStr(5, 16, "Calibragem");
      u8g2.drawButtonUTF8(5, 25, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Ajustes");
    } while (u8g2.nextPage());
  }


  else if (menu1 == 2 && menu2 == 0 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Calibragem");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Calibrar giroscopio");
      u8g2.drawStr(5, 29, "Calibrar sensoers de cor");
    } while (u8g2.nextPage());
  } else if (menu1 == 2 && menu2 == 0 && opcao == 2) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Calibragem");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 20, "Calibrar giroscopio");
      u8g2.drawButtonUTF8(5, 29, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Calibrar sensoers de cor");
    } while (u8g2.nextPage());
  }

  else if (menu1 == 2 && menu2 == 1 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Calibrar sensor");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Calibrar giroscópio");
      u8g2.drawStr(5, 29, "pare em uma superficie");
      u8g2.drawStr(5, 38, "plana e aperte enter");
      u8g2.drawLine(0, 43, 128, 43);
      u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
      u8g2.drawStr(23, 55, "Calibar");
      u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
      u8g2.drawStr(86, 55, "Cancelar");
    } while (u8g2.nextPage());

  } else if (menu1 == 2 && menu2 == 1 && opcao == 2) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Calibrar sensor");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Cor para desativar setor");
      u8g2.drawStr(5, 29, "Pare em um ambiente");
      u8g2.drawStr(5, 38, "escuro e aperte enter");
      u8g2.drawLine(0, 43, 128, 43);
      u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
      u8g2.drawStr(23, 55, "Calibar");
      u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
      u8g2.drawStr(86, 55, "Cancelar");
    } while (u8g2.nextPage());

  }



  else if (menu1 == 3 && menu2 == 0 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Ajustes");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Atraso de aperto");
      u8g2.drawStr(5, 29, "Luz de fundo minimo");
      u8g2.drawStr(5, 38, "Brilho automaico");
      u8g2.drawStr(5, 47, "Beep");
      u8g2.drawStr(5, 56, "Sensibilidade da cor");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 0 && opcao == 2) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Ajustes");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 20, "Atraso de aperto");
      u8g2.drawButtonUTF8(5, 29, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Luz de fundo minimo");
      u8g2.drawStr(5, 38, "Brilho automaico");
      u8g2.drawStr(5, 47, "Beep");
      u8g2.drawStr(5, 56, "Sensibilidade da cor");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 0 && opcao == 3) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Ajustes");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 20, "Atraso de aperto");
      u8g2.drawStr(5, 29, "Luz de fundo minimo");
      u8g2.drawButtonUTF8(5, 38, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Brilho automaico");
      u8g2.drawStr(5, 47, "Beep");
      u8g2.drawStr(5, 56, "Sensibilidade da cor");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 0 && opcao == 4) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Ajustes");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 20, "Atraso de aperto");
      u8g2.drawStr(5, 29, "Luz de fundo minimo");
      u8g2.drawStr(5, 38, "Brilho automaico");
      u8g2.drawButtonUTF8(5, 47, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Beep");
      u8g2.drawStr(5, 56, "Sensibilidade da cor");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 0 && opcao == 5) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Ajustes");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 20, "Atraso de aperto");
      u8g2.drawStr(5, 29, "Luz de fundo minimo");
      u8g2.drawStr(5, 38, "Brilho automaico");
      u8g2.drawStr(5, 47, "Beep");
      u8g2.drawButtonUTF8(5, 56, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Sensibilidade da cor");
    } while (u8g2.nextPage());
  }



  else if (menu1 == 3 && menu2 == 1 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "atraso de aperto");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "atraso de aperto  < >");
      u8g2.drawStr(5, 29, buffer);
      u8g2.drawLine(0, 43, 128, 43);
      u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
      u8g2.drawStr(23, 55, "Salvar");
      u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
      u8g2.drawStr(86, 55, "Cancelar");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 2 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Luz de fundo minimo");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Luz de fundo minimo  < >");
      u8g2.drawStr(5, 29, buffer);
      u8g2.drawLine(0, 43, 128, 43);
      u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
      u8g2.drawStr(23, 55, "Salvar");
      u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
      u8g2.drawStr(86, 55, "Cancelar");
    } while (u8g2.nextPage());
  } else if (menu1 == 3 && menu2 == 3 && opcao == 1) {
    if (luzAuto == 0) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(5, 8, "Brilho automatico");
        u8g2.drawLine(0, 11, 128, 11);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Brilho Automatico  < >");
        u8g2.drawStr(5, 29, "Desativado");
        u8g2.drawLine(0, 43, 128, 43);
        u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
        u8g2.drawStr(23, 55, "Salvar");
        u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
        u8g2.drawStr(86, 55, "Cancelar");
      } while (u8g2.nextPage());
    }
    if (luzAuto == 1) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(5, 8, "Brilho automatico");
        u8g2.drawLine(0, 11, 128, 11);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Brilho Automatico  < >");
        u8g2.drawStr(5, 29, "Ativado");
        u8g2.drawLine(0, 43, 128, 43);
        u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
        u8g2.drawStr(23, 55, "Salvar");
        u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
        u8g2.drawStr(86, 55, "Cancelar");
      } while (u8g2.nextPage());
    }
  } else if (menu1 == 3 && menu2 == 4 && opcao == 1) {
    if (botbeep == 0) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(5, 8, "Beep");
        u8g2.drawLine(0, 11, 128, 11);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Beep  < >");
        u8g2.drawStr(5, 29, "Desativado");
        u8g2.drawLine(0, 43, 128, 43);
        u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
        u8g2.drawStr(23, 55, "Salvar");
        u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
        u8g2.drawStr(86, 55, "Cancelar");
      } while (u8g2.nextPage());
    }
    if (botbeep == 1) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(5, 8, "Beep");
        u8g2.drawLine(0, 11, 128, 11);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Beep  < >");
        u8g2.drawStr(5, 29, "Ativado");
        u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
        u8g2.drawStr(23, 55, "Salvar");
        u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
        u8g2.drawStr(86, 55, "Cancelar");
      } while (u8g2.nextPage());
    }
  } else if (menu1 == 3 && menu2 == 5 && opcao == 1) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(5, 8, "Sensibilidade da cor");
      u8g2.drawLine(0, 11, 128, 11);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawButtonUTF8(5, 20, U8G2_BTN_INV, u8g2.getDisplayWidth() - 5 * 2, 5, 1, "Sensibilidade da cor  < >");
      u8g2.drawStr(5, 29, buffer);
      u8g2.drawLine(0, 43, 128, 43);
      u8g2.drawButtonUTF8(5, 55, U8G2_BTN_BW2, 0, 1, 1, "Ent");
      u8g2.drawStr(23, 55, "Salvar");
      u8g2.drawButtonUTF8(68, 55, U8G2_BTN_BW2, 0, 1, 1, "Can");
      u8g2.drawStr(86, 55, "Cancelar");
    } while (u8g2.nextPage());
  }
}
void setup() {
  beep();
  Configpinos();
  Wire.setClock(400000);
  Serial.begin(9600);
  InicioSensesor();
  minBrilho = EEPROM.read(0);
  atraso = EEPROM.read(1);
  luzAuto = EEPROM.read(2);
  botbeep = EEPROM.read(27);
  senCor = EEPROM.read(52);
  AccX_offset = EEPROM_readFloat(3);
  AccY_offset = EEPROM_readFloat(7);
  AccZ_offset = EEPROM_readFloat(11);
  GyrX_offset = EEPROM_readFloat(15);
  GyrY_offset = EEPROM_readFloat(19);
  GyrZ_offset = EEPROM_readFloat(23);
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2, HIGH);
  digitalWrite(rele3, HIGH);
  digitalWrite(rele4, HIGH);
  digitalWrite(Ledluz, LOW);
}
void loop() {
  LerSensores();
  UIcontrole();
}