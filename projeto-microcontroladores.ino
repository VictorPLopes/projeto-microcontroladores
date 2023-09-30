/*
Instituto Federal De Educação, Ciência e Tecnologia de São Paulo – Campus Piracicaba
Curso: Engenharia da Computação

PROJETO DE MICROCONTROLADORES: COLETA DE DADOS DE VARIÁVEIS CLIMÁTICAS PARA AGRICULTURA DE PRECISÃO

Autores:
    - André Lisboa Augusto | andre.lisboa@aluno.ifsp.edu.br
    - Marcos Henrique Maimoni Campanella | marcos.campanella@aluno.ifsp.edu.br
    - Rodolfo Henrique Raymundo Engelmann | rodolfo.engelmann@aluno.ifsp.edu.br
    - Victor Probio Lopes | victor.probio@aluno.ifsp.edu.br

Trabalho entregue ao professor Dr. Pablo Rodrigo de Souza, pertencente à disciplina "Microcontroladores"
com o objetivo deprojetar e construir um dispositivo para coleta de variáveis climáticas
para agricultura de pressão utilizando o microcontrolador ESP32.

Piracicaba, 2023
*/



// BIBLIOTECAS

#include <Wire.h> // Biblioteca para comunicação I2C
#include <LiquidCrystal_I2C.h> // Biblioteca para controle do LCD
#include <Adafruit_Sensor.h> // Biblioteca para sensores
#include <DHT_U.h> // Biblioteca para sensor DHT
#include <DHT.h> // Biblioteca para sensor DHT
#include <Adafruit_BMP280.h> //Biblioteca para sensor BMP

// DEFINIÇÕES

// Diretivas define para atribuição dos nomes dos pinos de E/S
#define BOTAO 5 // Pino do botão de seleção no ESP32 (D5 - GPIO5)
//#define BOTAO 14 // Pino do botão de seleção no ESP8266 (D5 - GPIO14)
#define DHT_PINO 4 // Pino do sensor DHT11 no ESP32 (D4 - GPIO4)
//#define DHT_PINO 2 // Pino do sensor DHT11 no ESP8266 (D4 - GPIO2)
#define LCD_ENDERECO 0x27 // Endereço I2C padrão do LCD

#define POS_GRAUS 0 // Posição do caractere de graus (°) no LCD
#define POS_ATIL 1 // Posição do caractere de a com til (ã) no LCD


// Filtro global para o botão de seleção de modo
#define FILTRO 500


// CONSTANTES
const float pressaoMar = 1013.25; // Pressão atmosférica ao nível do mar em hPa


// VARIÁVEIS GLOBAIS

LiquidCrystal_I2C lcd(LCD_ENDERECO, 16, 2); // Criação do objeto lcd da classe LiquidCrystal_I2C - Endereço I2C do LCD: 0x27 | Número de colunas: 16 | Número de linhas: 2

DHT dht(DHT_PINO, DHT11); // Criação do objeto dht da classe DHT - Pino do DHT: 4 | Tipo: DHT11

Adafruit_BMP280 bmp; // Criação do objeto bmp da classe Adafruit_BMP280

volatile byte modoSelecionado = 1; // 0 = Temperatura, 1 = Umidade, 2 = Pressão, 3 = Altitude

unsigned long tUltInt0 = 0; // Variável que armazena o tempo da última interrupção do botão de seleção de modo

float temperatura = 0; // Temperatura em graus Celsius - valor temporário de teste
float umidade = 0; // Umidade relativa do ar em %
float pressao = 0; // Pressão atmosférica em hPa - valor temporário de teste
float altitude = 0; // Altitude em metros - valor temporário de teste

byte graus[8] = { // Vetor de bytes que armazena o caractere de graus (°) para ser escrito no LCD
	0b00000,
	0b01110,
	0b01010,
	0b01110,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

byte aTil[8] = { // Vetor de bytes que armazena o caractere de a com til (ã) para ser escrito no LCD
	0b01110,
	0b00000,
	0b01110,
	0b00001,
	0b01111,
	0b10001,
	0b01111,
	0b00000
};


// FUNÇÕES

// Função que mede a umidade usando o sensor DHT11
void medeUmidade() {
    float umiAux = dht.readHumidity(); // Lê a umidade relativa do ar em % do sensor DHT11
    float tempAux = dht.readTemperature(); // Lê a temperatura em °C do sensor DHT11

    while (isnan(umiAux) || isnan(tempAux)) { // Verifica se houve erro na leitura do sensor DHT11
        Serial.println("Erro na leitura do DHT11");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro no DHT11!");
        dht.begin(); // Reinicia o sensor DHT11
        delay(100); // Delay de 100 ms
        umiAux = dht.readHumidity(); // Tenta ler a umidade novamente
        tempAux = dht.readTemperature(); // Tenta ler a temperatura novamente
    }
    umidade = umiAux; // Atualiza o valor da umidade

    Serial.print("Umidade (Ur%) = "); // Imprime o valor da umidade no monitor serial
    Serial.println(umidade);
    Serial.print("Temperatura - DHT11 (°C) = "); // Imprime o valor da temperatura no monitor serial
    Serial.println(tempAux);
}

// Função que mede a pressão e altitude usando o sensor BMP280
void medePressaoAltitude() {
    float pressaoAux = bmp.readPressure(); // Lê a pressão atmosférica em hPa do sensor BMP280
    float altitudeAux = bmp.readAltitude(pressaoMar); // Lê a altitude em metros do sensor BMP280
    float tempAux = bmp.readTemperature(); // Lê a temperatura em °C do sensor BMP280

    while (isnan(pressaoAux) || isnan(altitudeAux) || isnan(tempAux)) { // Verifica se houve erro na leitura do sensor BMP280
        Serial.println("Erro na leitura do BMP280");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro no BMP280!");
        bmp.begin(BMP280_ADDRESS_ALT); // Reinicia o sensor BMP280
        delay(100); // Delay de 100 ms
        pressaoAux = bmp.readPressure(); // Tenta ler a pressão novamente
        altitudeAux = bmp.readAltitude(pressaoMar);
        tempAux = bmp.readTemperature(); // Tenta ler a temperatura novamente
    }

    pressao = pressaoAux; // Atualiza o valor da pressão
    altitude = altitudeAux; // Atualiza o valor da altitude

    Serial.print("Pressão (atm) = "); // Imprime o valor da pressão no monitor serial
    Serial.println(pressao);
    Serial.print("Altitude - BMP (m) = "); // Imprime o valor da altitude no monitor serial
    Serial.println(altitudeAux);
    Serial.print("Temperatura - BMP280 (°C) = "); // Imprime o valor da temperatura no monitor serial
    Serial.println(tempAux);
}

// Função que mede a temperatura usando o sensor LM35
void medeTemperatura() {
    // Implementar
}

// Código necessário para gravar a interrupção na memória ram do ESP8266
void ICACHE_RAM_ATTR selecionaModo();

// Função acionada pela interrupção do botão de seleção de modo que alterna entre os modos de operação
void selecionaModo() {
    if ((millis() - tUltInt0) > FILTRO){ // Verifica se o tempo desde a última interrupção é maior que o filtro
        modoSelecionado = (modoSelecionado + 1) % 4; // Incrementa o modo selecionado e faz o módulo 4 para que o valor fique entre 0 e 3
        tUltInt0 = millis(); // Atualiza o tempo da última interrupção
    }
    //escreveLCD(); // Escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado | NÃO FUNCIONA
}

// Função que escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
void escreveLCD() {
    lcd.clear(); // Limpa o LCD
    switch (modoSelecionado) { // Verifica o modo selecionado
    case 0: // Modo de temperatura
        lcd.setCursor(2, 0);
        lcd.print("Temperatura");
        lcd.setCursor(4, 1);
        lcd.print("T=");
        medeTemperatura(); // Atualiza o valor da temperatura
        lcd.print(temperatura);
        lcd.write(POS_GRAUS); // Escreve o caractere de graus (°) no LCD
        lcd.print("C");
        break;
    case 1: // Modo de umidade
        lcd.setCursor(4, 0);
        lcd.print("Umidade");
        lcd.setCursor(3, 1);
        lcd.print("Ur%=");
        medeUmidade(); // Atualiza o valor da umidade
        lcd.print(umidade);
        break;
    case 2: // Modo de pressão
        lcd.setCursor(4, 0);
        lcd.print("Press");
        lcd.write(POS_ATIL); // Escreve o caractere de a com til (ã) no LCD
        lcd.print("o");
        lcd.setCursor(1, 1);
        lcd.print("P=");
        medePressaoAltitude(); // Atualiza o valor da pressão
        lcd.print(pressao);
        lcd.print("hPa");
        break;
    default: // Modo de altitude
        lcd.setCursor(4, 0);
        lcd.print("Altitude");
        lcd.setCursor(3, 1);
        lcd.print("A=");
        medePressaoAltitude(); // Atualiza o valor da altitude
        lcd.print(altitude);
        lcd.print("m");
        break;
    }
}

// Função que inicializa o ESP32
void setup() {
    // Configuração dos pinos como entrada ou saída
    pinMode(BOTAO, INPUT_PULLUP);

    // Inicializa a USART
    Serial.begin(9600);

    // Configuração do display LCD
    lcd.init(); // Inicialização do LCD
    lcd.backlight(); // Liga o backlight do LCD
    lcd.createChar(POS_GRAUS, graus); // Cria o caractere de graus (°) no LCD
    lcd.createChar(POS_ATIL, aTil); // Cria o caractere de a com til (ã) no LCD

    // Configuração do sensor DHT11
    dht.begin();
    
    // Configuração do sensor BMP280
    while(!bmp.begin(BMP280_ADDRESS_ALT)) { // Verifica se o sensor BMP280 foi encontrado
        Serial.println("Sensor não localizado");
        lcd.setCursor(0, 0);
        lcd.print("Erro no BMP280!");
    }
    lcd.clear(); // Limpa o LCD, para os casos em que houve erro na inicialização do BMP280
    lcd.setCursor(0, 0); // Posiciona o cursor na primeira coluna da primeira linha

    // Configuração da interrupção do botão de seleção de modo
    attachInterrupt(digitalPinToInterrupt(BOTAO), selecionaModo, FALLING);

}

// Função que executa o loop principal do programa
void loop() {
    /*
    if (!((millis() - tUltInt0) % 15000)) // Verifica se o tempo desde a última interrupção é múltiplo de 15 segundos
        selecionaModo(); // Se sim, chama a função de seleção de modo (para que o modo seja alterado automaticamente a cada 15 segundos)
    */
    escreveLCD(); // Escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
    delay(1000); // Delay de 1 segundo
}
