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

// DEFINIÇÕES

// Diretivas define para atribuição dos nomes dos pinos de E/S
#define BOTAO 5

// Filtro global para o botão de seleção de modo
#define FILTRO 100


// VARIÁVEIS GLOBAIS

LiquidCrystal_I2C lcd(0x27, 16, 2); // Criação do objeto lcd da classe LiquidCrystal_I2C - Endereço I2C do LCD: 0x27 | Número de colunas: 16 | Número de linhas: 2

byte modoSelecionado = 0; // 0 = Temperatura, 1 = Umidade, 2 = Pressão, 3 = Altitude

unsigned long tUltInt0 = 0; // Variável que armazena o tempo da última interrupção do botão de seleção de modo

float temperatura = 25.38; // Temperatura em graus Celsius - valor temporário de teste
float umidade = 50.78; // Umidade relativa do ar em % - valor temporário de teste
float pressao = 1013.25; // Pressão atmosférica em hPa - valor temporário de teste
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


// FUNÇÕES

// Função acionada pela interrupção do botão de seleção de modo que alterna entre os modos de operação
void selecionaModo() {
    modoSelecionado = (modoSelecionado + 1) % 4; // Incrementa o modo selecionado e faz o módulo 4 para que o valor fique entre 0 e 3
}

void selecionaModoFiltro() {
    if ((millis() - tUltInt0) > FILTRO){
        selecionaModo();
        tUltInt0 = millis();
    }
}

// Função que escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
void escreveLCD() {
    lcd.clear(); // Limpa o LCD
    switch (modoSelecionado) {
    case 0: // Modo de temperatura
        lcd.setCursor(2, 0);
        lcd.print("Temperatura");
        lcd.setCursor(4, 1);
        lcd.print("T=");
        lcd.print(temperatura);
        lcd.write(0);
        lcd.print("C");
        break;
    case 1: // Modo de umidade
        lcd.setCursor(4, 0);
        lcd.print("Umidade");
        lcd.setCursor(4, 1);
        lcd.print("Ur%=");
        lcd.print(umidade);
        break;
    case 2: // Modo de pressão
        lcd.setCursor(4, 0);
        lcd.print("Pressao");
        lcd.setCursor(3, 1);
        lcd.print("P=");
        lcd.print(pressao);
        lcd.print("hPa");
        break;
    default: // Modo de altitude
        lcd.setCursor(4, 0);
        lcd.print("Altitude");
        lcd.setCursor(4, 1);
        lcd.print("A=");
        lcd.print(altitude);
        lcd.print("m");
        break;
    }
}

void setup() {
    // Configuração dos pinos como entrada ou saída
    pinMode(BOTAO, INPUT_PULLUP);

    // Configuração do display LCD
    lcd.init(); // Inicialização do LCD
    lcd.backlight(); // Liga o backlight do LCD
    lcd.createChar(0, graus); // Cria o caractere de graus (°) no LCD
    lcd.setCursor(0, 0); // Posiciona o cursor na primeira coluna da primeira linha

    // Configuração da interrupção do botão de seleção de modo
    attachInterrupt(digitalPinToInterrupt(BOTAO), selecionaModoFiltro, FALLING);
}

void loop() {
    escreveLCD();
    delay(1000);
}