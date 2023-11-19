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
#include <time.h> //Biblioteca para TimeStamp
#include <PubSubClient.h> //Biblioteca para MQTT


// DEFINIÇÕES

// Diretivas define para atribuição dos nomes dos pinos de E/S
#define BOTAO 5 // Pino do botão de seleção no ESP32 (D5 - GPIO5)
#define DHT_PINO 4 // Pino do sensor DHT11 no ESP32 (D4 - GPIO4)
#define LM35_PINO 35 // Pino de leitura analógica do sensor LM35
#define LED_WIFI 23 // Led que indica se a conexão WiFi foi feita
#define IRRIGACAO 27 // Pino da irrigação
#define SEMEADURA 26 // Pino da semeadura

#define LCD_ENDERECO 0x27 // Endereço I2C padrão do LCD

// Diretivas define para atribuição das posições dos caracteres especiais no LCD
#define POS_GRAUS 0 // Posição do caractere de graus (°) no LCD
#define POS_ATIL 1 // Posição do caractere de a com til (ã) no LCD

// Filtro global para o botão de seleção de modo
#define FILTRO 500


// CONSTANTES

// Constnantess para medidas de variáveis climáticas
const float pressaoMar = 1013.25; // Pressão atmosférica ao nível do mar em 
const float vDiodosGlobal = 0.45; // Tensão nos diodos do ESP32

// Constantes para configurar a conexão WiFi
const String ssid = "WiFi_GrupoZ"; // Nome da rede WiFi
const String password = "12345678"; // Senha da rede WiFi

// Constantes para configurar a conexão com o servidor NTP
const String ntpServer = "pool.ntp.org"; // Endereço do servidor NTP
const int gmt_menos3 = -10800; //Fuso horário de Brasilia em segundos

// Constantes para configurar a conexão com o broker MQTT
const String brokerMqtt = "broker.hivemq.com"; // Endereço do broker MQTT
const int portaMqtt = 1883; // Porta do broker MQTT
const String idMqtt = "dadosClimaticosGrupoZ"; // ID MQTT (para identificar a sessão)


// VARIÁVEIS E OBJETOS GLOBAIS

// Configuração para acessar o horário no servidor utilizando o protocolo NTP
WiFiUDP ntpUDP; // Criação do objeto ntpUDP da classe WiFiUDP
NTPClient timeClient(ntpUDP, ntpServer.c_str()); // pool.ntp.org é o endereço do servidor; conversão para "const char*
bool ntpStatus = false; // Variável que indica se o NTP está inicializado
unsigned long timeStamp = 0; // Variável para armazenar a epoca

// Criação do objeto da classe WiFiClient para conexão com o broker MQTT por meio de um endereço e uma porta
WiFiClient espClient; // Criação do objeto espClient da classe WiFiClient
PubSubClient mqttClient(espClient); // Criação do objeto client da classe PubSubClient

// Criação do objeto lcd da classe LiquidCrystal_I2C - Endereço I2C do LCD: 0x27 | Número de colunas: 16 | Número de linhas: 2
LiquidCrystal_I2C lcd(LCD_ENDERECO, 16, 2);

// Criação do objeto dht da classe DHT - Pino do DHT: 4 | Tipo: DHT11
DHT dht(DHT_PINO, DHT11);

// Criação do objeto bmp da classe Adafruit_BMP280
Adafruit_BMP280 bmp;

// Variável que armazena o modo de operação selecionado
volatile byte modoSelecionado = 1; // 0 = Temperatura, 1 = Umidade, 2 = Pressão, 3 = Altitude, 4 = Data e Hora

// Variável que armazena o tempo da última interrupção do botão de seleção de modo
unsigned long tUltInt0 = 0;

// Variável que armazena o tempo da última atualização dos dados
unsigned long tUltAtualizacao = 0;

// Variáveis que armazenam os valores de temperatura, umidade, pressão e altitude
float temperatura = 0, // Temperatura em graus Celsius
      umidade = 0, // Umidade relativa do ar em %
      pressao = 0, // Pressão atmosférica em hPa
      altitude = 0; // Altitude em metros

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
    float pressaoAux = bmp.readPressure()/100.0; // Lê a pressão atmosférica em hPa do sensor BMP280
    float altitudeAux = bmp.readAltitude(pressaoMar); // Lê a altitude em metros do sensor BMP280
    float tempAux = bmp.readTemperature(); // Lê a temperatura em °C do sensor BMP280

    while (isnan(pressaoAux) || isnan(altitudeAux) || isnan(tempAux)) { // Verifica se houve erro na leitura do sensor BMP280
        Serial.println("Erro na leitura do BMP280");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro no BMP280!");
        bmp.begin(BMP280_ADDRESS_ALT); // Reinicia o sensor BMP280
        delay(100); // Delay de 100 ms
        pressaoAux = bmp.readPressure()/100.0; // Tenta ler a pressão novamente
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

// Código necessário para gravar a interrupção na memória ram do ESP8266 (não é necessário para o ESP32)
void ICACHE_RAM_ATTR selecionaModo();

// Função acionada pela interrupção do botão de seleção de modo que alterna entre os modos de operação
void selecionaModo() {
    if ((millis() - tUltInt0) > FILTRO){ // Verifica se o tempo desde a última interrupção é maior que o filtro
        modoSelecionado = (modoSelecionado + 1) % 5; // Incrementa o modo selecionado e faz o módulo 4 para que o valor fique entre 0 e 3
        tUltInt0 = millis(); // Atualiza o tempo da última interrupção
    }
}

// Função que escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
void escreveLCD() {
    //lcd.clear(); // Limpa o LCD
    switch (modoSelecionado) { // Verifica o modo selecionado
    case 0: // Modo de temperatura
        lcd.setCursor(0, 0);
        lcd.print("  Temperatura   ");
        lcd.setCursor(0, 1);
        lcd.print("   T=");
        lcd.print(temperatura);
        lcd.write(POS_GRAUS); // Escreve o caractere de graus (°) no LCD
        lcd.print("C   ");
        break;
    case 1: // Modo de umidade
        lcd.setCursor(0, 0);
        lcd.print("    Umidade     ");
        lcd.setCursor(0, 1);
        lcd.print("   Ur%=");
        lcd.print(umidade);
        lcd.print("    ");
        break;
    case 2: // Modo de pressão
        lcd.setCursor(0, 0);
        lcd.print("    Press");
        lcd.write(POS_ATIL); // Escreve o caractere de a com til (ã) no LCD
        lcd.print("o     ");
        lcd.setCursor(0, 1);
        lcd.print("  P=");
        lcd.print(pressao);
        lcd.print("hPa ");
        break;
    case 4: // Modo de altitude
        lcd.setCursor(0, 0);
        lcd.print("    Altitude    ");
        lcd.setCursor(0, 1);
        lcd.print("   A=");
        lcd.print(altitude);
        lcd.print("m    ");
        break;
    default: // Modo de data e hora
        lcd.setCursor(0, 0);
        if (WiFi.status() == WL_CONNECTED) { // Se o ESP32 estiver conectado na rede WiFi
            lcd.print("Data: ");
            lcd.print(epochToDDMMYYYY(timeStamp, gmt_menos3));
            lcd.setCursor(0, 1);
            lcd.print(" Hora: ");
            lcd.print(timeClient.getFormattedTime());
            lcd.print(" ");
        }
        else { // Se o ESP32 não estiver conectado na rede WiFi
            lcd.print("      WiFi      ");
            lcd.setCursor(0, 1);
            lcd.print("  Desconectado  ");
        }
        break;
    }
}

// Função que se conecta na rede WiFi
void conectaWiFi(String ssid, String password) {
    WiFi.mode(WIFI_STA); // Configura o ESP32 como cliente WiFi
    WiFi.begin(ssid, password); // Conecta na rede WiFi
}


// Função para inicializar a rede WiFi
void iniciaWiFi() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Conectando WiFi");
    lcd.setCursor(0, 1);
    lcd.print("...");

    Serial.println("Conectando na rede WiFi...");
    Serial.println(ssid);

    conectaWiFi(ssid, password); // Conecta na rede WiFi

    unsigned long t0 = millis(); // Armazena o tempo atual em milissegundos
    while ((WiFi.status() != WL_CONNECTED) && ((millis() - t0) < 15000)) // Verifica se o ESP32 está conectado na rede WiFi ou se o tempo desde o início da conexão é maior que 30 segundos
        Serial.print('.');
    if (WiFi.status() != WL_CONNECTED) { // Se o ESP32 não estiver conectado na rede WiFi após 30 segundos
        Serial.println("\nFalha na conexão WiFi");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro no WiFi!");

        delay(2000);
    }
    else {
        Serial.print("\nConectado na rede WiFi ");
        Serial.println(WiFi.localIP()); // Imprime o endereço IP do ESP32 no monitor serial

        digitalWrite(LED_WIFI, HIGH); // Acende o LED que indica que o ESP32 está conectado na rede WiFi

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi conectado!");

        delay(2000);
    }
}

// Verifica se o ESP32 está conectado na rede WiFi
bool checaWiFi() {
    if (WiFi.status() != WL_CONNECTED) { // Se o ESP32 não estiver conectado na rede WiFi
        digitalWrite(LED_WIFI, LOW); // Apaga o LED que indica que o ESP32 está conectado na rede WiFi

        Serial.println("Reconectando na rede WiFi...");

	WiFi.disconnect();
        conectaWiFi(ssid, password); // Conecta na rede WiFi
    }
    return WiFi.status() == WL_CONNECTED; // Retorna se o ESP32 está conectado na rede WiFi
}

// Função de callback para o recebimento de mensagens no broker MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Mensagem recebida no tópico: ");
    Serial.println(topic);

    Serial.print("Mensagem: ");
    for (int i = 0; i < length; i++)
        Serial.print((char)payload[i]);
    Serial.println();

    // Controle da mensagem recebida
    String topico = String(topic); // Converte o tópico recebido para String

    if(topico.equals("GrupoZ_irrigacao")) { // Se a mensagem for para controlar a irrigação
        if((char)payload[0] == '1') { // Se o payload for igual a 1
            digitalWrite(IRRIGACAO, HIGH); // Liga a irrigação
            Serial.println("Irrigação ligada");
        }
        else if((char)payload[0] == '0') { // Se o payload for igual a 0
            digitalWrite(IRRIGACAO, LOW); // Desliga a irrigação
            Serial.println("Irrigação desligada");
        }
    }
    else if(topico.equals("GrupoZ_semeadura")) { // Se a mensagem for para controlar a semeadura
        if((char)payload[0] == '1') { // Se o payload for igual a 1
            digitalWrite(SEMEADURA, HIGH); // Liga a semeadura
            Serial.println("Semeadura ligada");
        }
        else if((char)payload[0] == '0') { // Se o payload for igual a 0
            digitalWrite(SEMEADURA, LOW); // Desliga a semeadura
            Serial.println("Semeadura desligada");
        }
    }
}

// Função para conectar no broker MQTT
bool conectaMqtt(String endereco, int porta, String id) {
    mqttClient.setServer(endereco.c_str(), porta); // Configura o endereço e a porta do broker MQTT
    mqttClient.setCallback(callback); // Configura a função de callback para o recebimento de mensagens no broker MQTT
    return mqttClient.connect(id.c_str()); // Conecta no broker MQTT
}

// Função para inicializar a conexão com o broker MQTT
void iniciaMqtt() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Conectando MQTT");
    lcd.setCursor(0, 1);
    lcd.print("...");

    Serial.println("Conectando no broker MQTT...");
    Serial.println(brokerMqtt);

    unsigned long t0 = millis(); // Armazena o tempo atual em milissegundos
    while (!conectaMqtt(brokerMqtt, portaMqtt, idMqtt) && ((millis() - t0) < 15000)) { // Verifica se o ESP32 está conectado no broker MQTT ou se o tempo desde o início da conexão é maior que 30 segundos
        Serial.print('.');
    }
    if (!mqttClient.connected()) { // Se o ESP32 não estiver conectado no broker MQTT após 15 segundos
        Serial.println("\nFalha na conexão MQTT");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro no MQTT!");
        lcd.setCursor(0, 1);
        lcd.print("Envio indisp.");

        delay(2000);
    }
    else {
        Serial.println("\nConectado no broker MQTT");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MQTT conectado!");

        delay(2000);
    }
}

// Verifica se o ESP32 está conectado no broker MQTT
bool checaMqtt() {
    if (!mqttClient.connected()) { // Se o ESP32 não estiver conectado no broker MQTT
        Serial.println("Reconectando no broker MQTT...");

        conectaMqtt(brokerMqtt, portaMqtt, idMqtt); // Conecta no broker MQTT
    }
    return mqttClient.connected(); // Retorna se o ESP32 está conectado no broker MQTT
}

// Função que converte de epoch para DD/MM/AAAA
String epochToDDMMYYYY(unsigned long epoch, int fuso) {
    unsigned long dataFuso = epoch + fuso; // Soma o fuso horário em segundos ao epoch
    struct tm *ptm = gmtime((time_t *)&dataFuso); // Converte o epoch para a struct tm
    return String(ptm->tm_mday) + "/" + String(ptm->tm_mon + 1) + "/" + String(ptm->tm_year + 1900); // Retorna a data no formato DD/MM/AAAA
}

// Função para checar o horário no servidor NTP
void checaTimeStamp() {
    if (!ntpStatus) { // Se o NTP não estiver inicializado
        timeClient.begin(); // Inicializa o NTP
        timeClient.setTimeOffset(gmt_menos3); //Correção do fuso horário para o horário de Brasilia
    }

    timeClient.update(); // Obtém o horário do servidor NTP
    timeStamp = timeClient.getEpochTime(); // Quando for converter o timeStamp para data e horário no Python, precisa somar 10800 para ajustar o fuso

    Serial.println("\n--------------------------------------------------------------------------------");


    Serial.print("HORARIO: ");
    Serial.println(timeClient.getFormattedTime());

    Serial.print("HORA: ");
    Serial.println(timeClient.getHours());

    Serial.print("MINUTOS: ");
    Serial.println(timeClient.getMinutes());

    Serial.print("SEGUNDOS: ");
    Serial.println(timeClient.getSeconds());

    Serial.print("DATA: ");
    Serial.println(epochToDDMMYYYY(timeStamp, gmt_menos3));

    Serial.print("DIA DA SEMANA: ");
    byte dia = timeClient.getDay();
    switch (dia) {
        case 0:
            Serial.println("Domingo");
            break;
        case 1:
            Serial.println("Segunda-Feira");
            break;
        case 2:
            Serial.println("Terça-Feira");
            break;
        case 3:
            Serial.println("Quarta-Feira");
            break;
        case 4:
            Serial.println("Quinta-Feira");
            break;
        case 5:
            Serial.println("Sexta-Feira");
            break;
        case 6:
            Serial.println("Sábado");
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
    Serial.flush(); // Limpa o buffer da USART
    
    // Configuração do display LCD
    Serial.println("Inicializando LCD...");
    lcd.init(); // Inicialização do LCD
    lcd.backlight(); // Liga o backlight do LCD
    lcd.createChar(POS_GRAUS, graus); // Cria o caractere de graus (°) no LCD
    lcd.createChar(POS_ATIL, aTil); // Cria o caractere de a com til (ã) no LCD

    // Configuração da conexão WiFi
    pinMode(LED_WIFI, OUTPUT);
    digitalWrite(LED_WIFI, LOW);

    //Inicializa WiFi
    Serial.println("Inicializando WiFi...");
    iniciaWiFi();

    //Inicializa conexão servidor NTP
    if (WiFi.status() == WL_CONNECTED) { // Se o ESP32 estiver conectado na rede WiFi
        Serial.println("Inicializando conexão com servidor NTP...");
        timeClient.begin();
        timeClient.setTimeOffset(gmt_menos3); //Correção do fuso horário para o horário de Brasilia
        ntpStatus = true;
        
        Serial.println("Inicializando MQTT...");
        iniciaMqtt();
    }

    // Configuração do sensor DHT11
    Serial.println("Inicializando DHT11...");
    dht.begin();
    
    // Configuração do sensor BMP280
    Serial.println("Inicializando BMP280...");
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
    // Verifica se já passou 1 segundo desde a última atualização
    unsigned long tAtual = millis(); // Armazena o tempo atual em milissegundos
    if (tAtual - tUltAtualizacao > 1000) { // Verifica se o tempo desde a última atualização do LCD é maior que 1 segundo
        tUltAtualizacao = tAtual; // Atualiza o tempo da última atualização do LCD

        // Medições
        medeTemperatura(vDiodosGlobal, 10); // Atualiza o valor da temperatura
        medeUmidade(); // Atualiza o valor da umidade
        medePressaoAltitude(); // Atualiza o valor da pressão

        // Rede
        if (checaWiFi()) { // Verifica se o ESP32 está conectado na rede WiFi
            checaTimeStamp(); // Verifica o horário no servidor NTP
            if (checaMqtt()) { // Verifica se o ESP32 está conectado no broker MQTT
                Serial.println("Publicando dados no broker MQTT...");

                // Converte o valor da temperatura para char[]
                char temperaturaChar[10];
                dtostrf(temperatura, 6, 3, temperaturaChar);

                // Converte o valor da umidade para char[]
                char umidadeChar[10];
                dtostrf(umidade, 6, 3, umidadeChar);
                
                // Converte o valor da pressão para char[]
                char pressaoChar[10];
                dtostrf(pressao, 6, 3, pressaoChar);
                
                // Converte o valor da altitude para char[]
                char altitudeChar[10];
                dtostrf(altitude, 6, 3, altitudeChar);
                
                // Publica os dados como uma string formatada no tópico "GrupoZ_time_temp_umi_press_alt"
                mqttClient.publish("GrupoZ_time_temp_umi_press_alt", (String(timeStamp) + ";" + String(temperaturaChar) + ";" + String(umidadeChar) + ";" + String(pressaoChar) + ";" + String(altitudeChar)).c_str(), true);

                // Publica os dados separadamente nos tópicos "GrupoZ_time", "GrupoZ_temp", "GrupoZ_umi", "GrupoZ_press" e "GrupoZ_alt"
                mqttClient.publish("GrupoZ_time", String(timeStamp).c_str(), true);
                mqttClient.publish("GrupoZ_temp", temperaturaChar, true);
                mqttClient.publish("GrupoZ_umi", umidadeChar, true);
                mqttClient.publish("GrupoZ_press", pressaoChar, true);
                mqttClient.publish("GrupoZ_alt", altitudeChar, true);

                Serial.println("Dados publicados no broker MQTT com sucesso!");

                mqttClient.loop(); // Mantém a conexão com o broker MQTT
            }
        }
    }
    
    escreveLCD(); // Escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
}