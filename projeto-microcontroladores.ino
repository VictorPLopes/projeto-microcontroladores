/*
Instituto Federal De Educação, Ciência e Tecnologia de São Paulo – Campus Piracicaba
Curso: Engenharia da Computação

PROJETO DE MICROCONTROLADORES: COLETA DE DADOS DE VARIÁVEIS CLIMÁTICAS PARA AGRICULTURA DE PRECISÃO

Autores:
    - André Lisboa Augusto | andre.lisboa@aluno.ifsp.edu.br
    - Marcos Henrique Maimoni Campanella | marcos.campanella@aluno.ifsp.edu.br
    - Rodolfo Henrique Raymundo Engelmann | rodolfo.engelmann@aluno.ifsp.edu.br
    - Victor Probio Lopes | victor.probio@aluno.ifsp.edu.br

Trabalho entregue ao professor Dr. Pablo Rodrigo de Souza, pertencente à disciplina “Microcontroladores”
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

LiquidCrystal_I2C lcd(0x3f, 16, 2); // Criação do objeto lcd da classe LiquidCrystal_I2C

byte modoSelecionado = 0; // 0 = Temperatura, 1 = Umidade, 2 = Pressão

unsigned long tUltInt0 = 0; // Variável que armazena o tempo da última interrupção do botão de seleção de modo

float temperatura = 28.82; // Temperatura em graus Celsius - valor temporário de teste
float umidade = 50.78; // Umidade relativa do ar em % - valor temporário de teste
float pressao = 1013.25; // Pressão atmosférica em hPa - valor temporário de teste

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
    modoSelecionado = (modoSelecionado + 1) % 3; // Incrementa o modo selecionado e faz o módulo 3 para que o valor fique entre 0 e 2
}

void selecionaModoFiltro() {
    if ((millis() - tUltInt0) > FILTRO){
        selecionaModo();
        tUltInt0 = millis();
    }
}

// Função que escreve os valores de temperatura, umidade e pressão no LCD conforme o modo selecionado
void escreveLCD() {
    switch (modoSelecionado) {
    case 0: // Modo de temperatura
        lcd.setCursor(0, 2);
        lcd.print("Temperatura");
        lcd.setCursor(1, 4);
        lcd.print("T=");
        lcd.print(temperatura);
        lcd.write(0);
        lcd.print("C");
        break;
    case 1: // Modo de umidade
        lcd.setCursor(0, 4);
        lcd.print("Umidade");
        lcd.setCursor(1, 4);
        lcd.print("Ur%=");
        lcd.print(umidade);
        break;
    case 2: // Modo de pressão
        lcd.setCursor(0, 4);
        lcd.print("Pressao");
        lcd.setCursor(1, 3);
        lcd.print("P=");
        lcd.print(pressao);
        lcd.print("hPa");
        break;
    default: // Inutilizado, pois o modo selecionado nunca será diferente de 0, 1 ou 2
        break;
    }
}

void setup() {
    // Configuração dos pinos como entrada ou saída
    pinMode(BOTAO, INPUT_PULLUP);

    // Configuração do display LCD
    lcd.begin(16, 2);
    lcd.createChar(0, graus);
    lcd.setCursor(0, 0);

    // Configuração da interrupção do botão de seleção de modo
    attachInterrupt(digitalPinToInterrupt(BOTAO), selecionaModoFiltro, FALLING);
}

void loop() {
    escreveLCD();
    delay(1000);
}