#include <stdio.h>
#include <stdlib.h>
#define FOSC 16000000U  
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1
#define sensor_ultrassonico (1 << PC3) //definindo o sensor ultrassonico no PORT3
#define botao_sensor_gotas (1 << PB2) //sensor de gotas(resistor pull-up)
#define buzzer_alarme (1<<PD2) //alarme
#define motor (1 << PB3)  
#define AMOSTRAS 50       

char resposta = 0;
int cont = 0;     
bool entrada_de_dados = true;
unsigned int Leitura_AD;
unsigned int x = 0;
unsigned int volume = 0;
unsigned int minutos = 0;
float hora = 0;
float fluxo_definido = 0; 
float potencia = 0;
float tensao;  
float fluxo_real = 0;  
float DC = 0; //define a velocidade do motor - PWM
float erro;       

char msg_tx[20];
char msg_rx[32];
int pos_msg_rx = 0;
int tamanho_msg_rx = 3;  
void UART_Init(unsigned int ubrr);
void UART_Transmit(char *dados);

int main() {

  //Configurando a parte da UART
  UART_Init(MYUBRR);

  //Declarando as saídas
  DDRD = buzzer_alarme ; 
  PORTD &= ~buzzer_alarme; // Para o buzzer começar desligando
  DDRB = motor;
  PORTD &= ~ motor;

  //Interrupção do botão
  PCICR = (1 << PCIE0);  //Interrupção habilitada no Portal B
  PCMSK0 = botao_sensor_gotas;  //Define o pino que a interrupção será gerada   

  //Configurar a conversão AD para o sensor ultrasônico
  ADMUX = (0 << REFS1) + (1 << REFS0);
  ADCSRA = (1 << ADEN) + (1 << ADPS2) + (1 << ADPS1) + (1 << ADPS0);
  ADCSRB = 0;

  //Inturrupção configurado no modo comparador
  TCCR0A = (1 << WGM01);   //configura o modo para comparador
  OCR0A = 249; //250 contagens de 4 us, o que gera uma interrupção a cada 1000 us
  TIMSK0 = (1 << OCIE0A);  

  //Configurando o PWM para setar a velocidade no motor 
  TCCR2A = (1 << COM0A1) + (0 << COM0A0) + (1 << WGM01) + (1 << WGM00); //Configura o modo não invertido e Fast Pwm
  TCCR2B = (1 << CS02) + (1 << CS00);  //Prescaler de 1024 - PWM, para diminuir a oscilação do motor;
  OCR2A = 0; //Inicia a velocidade em zero

  //Interrupção gerada devido as bolhas pelo sensor ultrassonico
  PCICR = (1 << PCIE1); //Interrupção habilitada no portal C
  PCMSK1 = sensor_ultrassonico; //Define o pino que irá gerar a interrupção

  sei();  //Habilita a interrupção global

  while (1) {
    if (entrada_de_dados) {
      UART_Transmit("Entre com o volume: \n");
      while (x == 0) {
        volume = (msg_rx[0] - 48) * 100 + (msg_rx[1] - 48) * 10 + (msg_rx[2] - 48) * 1;
        _delay_ms(200);
        if (volume >= 0 && volume <= 999) {
          x = 1;
        }
      }
      itoa(volume, msg_tx, 10);
      UART_Transmit(msg_tx);
      UART_Transmit("\n");
      msg_rx[0] = 0;
      msg_rx[1] = 0;
      msg_rx[2] = 0;

      UART_Transmit("Entre com o Tempo de Infusão em minutos: \n");
      while (x == 1) {
        minutos = (msg_rx[0] - 48) * 100 + (msg_rx[1] - 48) * 10 + (msg_rx[2] - 48) * 1;
        _delay_ms(200);
        if ((minutos >= 0) && (minutos <= 999)) {
          x = 2;
        }
      }
      
      hora = minutos / 60.0;  //converter os minutos em hora
      itoa(minutos,msg_tx,10);
      UART_Transmit(msg_tx);
      UART_Transmit("\n");
      itoa(hora,msg_tx,10);
      UART_Transmit(msg_tx);
      UART_Transmit("\n");
      msg_rx[0] = 0;
      msg_rx[1] = 0;
      msg_rx[2] = 0;

      fluxo_definido = (volume / hora);       
      potencia = (fluxo_definido / 450) * 100;  //potencia (fluxo maximo é 450)
      entrada_de_dados = false;
    }
    else{
        erro = ((fluxo_real - fluxo_definido) / fluxo_definido) * 100; //Diferenca em porcentagem do fluxo real para o fluxo definido
        UART_Transmit("O erro em porcentagem: \n");
        itoa(erro, msg_tx, 10);  //printa o erro na serial
        UART_Transmit(msg_tx);
        UART_Transmit(" \n");
        x = 0;  //após mostrar deve entrar com os dados novamente
        entrada_de_dados = true; 
      }
    
  }
}

  //Interrupção de interrupção do Sensor de gotas
  ISR(PCINT0_vect) {
    short int BOTAO = PINB & botao_sensor_gotas;
    bool flag = true;

    if (BOTAO == 1 && flag == true) {
      TCCR0B = (1 << CS02) + (1 << CS00); //Prescaler de 1024 - Contador de gotas
      flag == false;
    }
    
    if(BOTAO == 1){  
      TCCR0B = 0; //Voltando o prescaler para 0 para reiniciar a contagem
      fluxo_real = (0.05 / (cont * 36000000));  //converte os segundos em hora (mili segundos)
      cont = 0;
      TCCR2B = (1 << CS02) + (1 << CS00);
    }
  }
  //Interrupção da primeira gota
  ISR(TIMER0_COMPA_vect) {
    cont++;
  }
  //Interrupção gerada pelo sensor ultrasônico
  ISR(PCINT1_vect) {

    ADMUX = sensor_ultrassonico ;  //definido que irá ler o sensor ultrassonico

    //Leitura do ADC (Com média)
    unsigned int SomaLeitura = 0, MediaLeitura;
    for (int i = 0; i < AMOSTRAS; i++) {

      ADCSRA |= (1 << ADSC);  //Inicia a conversão
      while ((ADCSRA & (1 << ADSC)) == (1 << ADSC));  //Esperar a conversão
      Leitura_AD = ADC;
      SomaLeitura += Leitura_AD;
      
    }

    MediaLeitura = SomaLeitura / AMOSTRAS;

    tensao = ((MediaLeitura * 5.0) / 1023.0);  //Cálculo da Tensão

    //Verificando se há existencia de bolhas (largura da tubulação sobre a distancia total (5/20 * 5) - como é inversamente proporcional a leitura será maior que 1.25V
    if (tensao > 1.25) {
      PORTD |= buzzer_alarme; //buzzer será acionado quando detectado bolha
      DC = 0; //desliga o motor
      OCR2A = int(DC);//seta o valor 0 no motor
    }
    else{
      PORTD &= ~buzzer_alarme; //desliga o buzzer
    }
  }


void UART_Transmit(char *dados) {
  while (*dados != 0) {
    while (!(UCSR0A & (1 << UDRE0)));  
    UDR0 = *dados;
    dados++;
  }
}
void UART_Init(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
ISR(USART_RX_vect) {
  msg_rx[pos_msg_rx++] = UDR0;
  if (pos_msg_rx == tamanho_msg_rx)
    pos_msg_rx = 0;
}
