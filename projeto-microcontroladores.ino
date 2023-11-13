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
#include <WiFi.h> //Biblioteca para a conexão WiFi
#include <NTPClient.h> //Biblioteca para TimeStamp

// DEFINIÇÕES

// Diretivas define para atribuição dos nomes dos pinos de E/S
#define BOTAO 5 // Pino do botão de seleção no ESP32 (D5 - GPIO5)
//#define BOTAO 14 // Pino do botão de seleção no ESP8266 (D5 - GPIO14)
#define DHT_PINO 4 // Pino do sensor DHT11 no ESP32 (D4 - GPIO4)
#define LM35_PINO 35 // Pino de leitura analógica do sensor LM35
//#define DHT_PINO 2 // Pino do sensor DHT11 no ESP8266 (D4 - GPIO2)

#define LCD_ENDERECO 0x27 // Endereço I2C padrão do LCD

#define ledWiFi 23 //Led que indica se a conexão WiFi foi feita

#define POS_GRAUS 0 // Posição do caractere de graus (°) no LCD
#define POS_ATIL 1 // Posição do caractere de a com til (ã) no LCD


// Filtro global para o botão de seleção de modo
#define FILTRO 500


// CONSTANTES
const float pressaoMar = 1013.25; // Pressão atmosférica ao nível do mar em 
const float vDiodosGlobal = 0.45; // Tensão nos diodos do ESP32
const char* ssid = "Senha: 12345678";
const char* password = "12345678";

// VARIÁVEIS GLOBAIS

//Configuração para acessar o horário no servidor utilizando o protocolo NTP
WiFiUDP ntpUDP;                     
NTPClient timeClient(ntpUDP,"pool.ntp.org");  //pool.ntp.org é o endereço do servidor

LiquidCrystal_I2C lcd(LCD_ENDERECO, 16, 2); // Criação do objeto lcd da classe LiquidCrystal_I2C - Endereço I2C do LCD: 0x27 | Número de colunas: 16 | Número de linhas: 2

DHT dht(DHT_PINO, DHT11); // Criação do objeto dht da classe DHT - Pino do DHT: 4 | Tipo: DHT11

Adafruit_BMP280 bmp; // Criação do objeto bmp da classe Adafruit_BMP280

volatile byte modoSelecionado = 1; // 0 = Temperatura, 1 = Umidade, 2 = Pressão, 3 = Altitude

unsigned long tUltInt0 = 0; // Variável que armazena o tempo da última interrupção do botão de seleção de modo

unsigned long timeStamp=0; //Variável para armazenar a epoca

double temperatura = 0; // Temperatura em graus Celsius - valor temporário de teste
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
        pressaoAux = bmp.readPressure() / 100; // Tenta ler a pressão novamente
        altitudeAux = bmp.readAltitude(pressaoMar);
        tempAux = bmp.readTemperature(); // Tenta ler a temperatura novamente
    }

    pressao = pressaoAux; // Atualiza o valor da pressão
    altitude = altitudeAux; // Atualiza o valor da altitude

    Serial.print("Pressão (hPa) = "); // Imprime o valor da pressão no monitor serial
    Serial.println(pressao);
    Serial.print("Altitude - BMP (m) = "); // Imprime o valor da altitude no monitor serial
    Serial.println(altitudeAux);
    Serial.print("Temperatura - BMP280 (°C) = "); // Imprime o valor da temperatura no monitor serial
    Serial.println(tempAux);
}

// Fução que converte a leitura analógica em tensão
double calculaTensao(double leitura) {
    return 0.00080488*leitura + 0.13169092; // Ajuste linear
    // return (-1)*0.00000006*(leitura*leitura) + 0.00102569*leitura + 0.02256359; // Ajuste com polinômio de 2º Grau
}

// Função que mede a temperatura usando o sensor LM35
void medeTemperatura(float vDiodos, int nMedidas) { // vDiodos é a tensão nos diodos do ESP32; nMedidas é o número de leituras analógicas a serem feitas
    // Valor cumulativo das leituras
    int n = 0;
    for (int i = 0; i < nMedidas; i++) {
        n += analogRead(LM35_PINO);
        delay(20);
    }
    
    // Media das leituras
    double nMedia = n/nMedidas; // Valor médio das leituras

    Serial.print("Valor de n (leitura analógica):"); // Imprime o valor de n no monitor serial
    Serial.println(nMedia);

    // Calcula a tensão
    double tensao = calculaTensao(nMedia); // Calcula a tensão a partir do valor médio das leituras, considerando o ajuste linear
    Serial.print("Tensão (leitura analógica):"); // Imprime o valor da tensão no monitor serial
    Serial.println(tensao);

    // Calcula a temperatura
    double tempAux = (tensao - vDiodos) / 0.01; // Tensão de saída do LM35 = 10 mV/°C
    Serial.print("Temperatura - LM35 (°C) = "); // Imprime o valor da temperatura no monitor serial
    Serial.println(tempAux);


    // Se a nova temperatura for consideravelmente diferente da anterior, atualiza seu valor
    if ((-0.2 > (temperatura - tempAux)) || ((temperatura - tempAux) > 0.2))
        temperatura = tempAux; // Atualiza o valor da temperatura
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
        medeTemperatura(vDiodosGlobal, 10); // Atualiza o valor da temperatura
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


//Função para inicializar a rede WiFi
void iniciaWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando na rede WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  digitalWrite(ledWiFi, HIGH);
}

//Verifica se o esp32 está conectado na rede WiFi
void checaWiFi() {
  if ((WiFi.status() != WL_CONNECTED)) {
    digitalWrite(ledWiFi, LOW);
    Serial.println("Reconectando na rede WiFi...");
    WiFi.disconnect();
    iniciaWiFi();
  }
}
void checaTimeStamp(){
  timeClient.update();                 // Obtém o horário do servidor NTP
  timeStamp=timeClient.getEpochTime(); // Quando for converter o timeStamp para data e horário no Python, precisa somar 10800 para ajustar o fuso
  Serial.print("HORARIO: ");
  Serial.println(timeClient.getFormattedTime());

  Serial.print("HORA: ");
  Serial.println(timeClient.getHours());

  Serial.print("MINUTOS: ");
  Serial.println(timeClient.getMinutes());

  Serial.print("SEGUNDOS: ");
  Serial.println(timeClient.getSeconds());

  Serial.print("DIA DA SEMANA: ");
  byte dia = timeClient.getDay();
  switch (dia){
    case 0:
      Serial.print("Domingo");
      break;
    case 1:
      Serial.print("Segunda-Feira");
      break;
    case 2:
      Serial.print("Terca-Feira");
      break;
    case 3:
      Serial.print("Quarta-Feira");
      break;
    case 4:
      Serial.print("Quinta-Feira");
      break;
    case 5:
      Serial.print("Sexta-Feira");
      break;
    case 6:
      Serial.print("Sabado");
      break;     
  }                           
  Serial.print("Epoca (Segundos desde 01/01/1970): ");
  Serial.println(timeStamp);
  Serial.println();
}

// Função que inicializa o ESP32
void setup() {
    // Configuração dos pinos como entrada ou saída
    pinMode(BOTAO, INPUT_PULLUP);

    // Inicializa a USART
    Serial.begin(9600);

    // Configuração da conexão WiFi
    pinMode(ledWiFi, OUTPUT);
    digitalWrite(ledWiFi, LOW);

    //Inicializa conexão servidor NTP
    timeClient.begin();
    timeClient.setTimeOffset(-10800); //Correção do fuso horário para o horário de Brasilia
  
    //Inicializa WiFi
    iniciaWiFi();

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
  checaWiFi();
  checaTimeStamp();
    /*
    if (!((millis() - tUltInt0) % 15000)) // Verifica se o tempo desde a última interrupção é múltiplo de 15 segundos
        selecionaModo(); // Se sim, chama a função de seleção de modo (para que o modo seja alterado automaticamente a cada 15 segundos)
    */
    escreveLCD(); // Escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
    delay(1000); // Delay de 1 segundo
}
