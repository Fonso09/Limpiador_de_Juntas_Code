#include <stm32f7xx.h>
#include <unordered_map>
using namespace std;
void SysTick_Init(void);                   //funciones para los delays
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void RCC_SETUP();
void GPIO_SETUP();
void USART_SETUP();
int USART3_SendChar(int value);
void TIM_Config();
void carro_adelante(int potencia);
void carro_atras(int potencia);
void paro_emergencia();
void Test_Serial();
void Test_Timer();
int in_adelante=0;
int in_atras=0;
int in_E_stop=0;
unordered_map<char, int> potePWM;




int main(){
    //Inicializar funciones y los setups 
	RCC_SETUP();
    GPIO_SETUP();
    SysTick_Init();	
    USART_SETUP();
    TIM_Config();
    potePWM['A'] = 5000;
    potePWM['M'] = 12000;
    potePWM['B'] = 20000;

    while(1){
        //Test_Serial();
        //Test_Timer();

        
        //Entradas 
        in_adelante= (GPIOF->IDR)& (1<<13);
        in_atras= (GPIOF->IDR)& (1<<14);
        in_E_stop= (GPIOC->IDR)& (1<<13);
        //GPIOB->ODR &= ~0xFFFF;
        //GPIOE->ODR &= ~0xFFFF;

        //Seleccionar funcion del carro, de manera manual con botones fisicos:
        if(in_E_stop==(1<<13))
        {
            paro_emergencia();
        } else{
            if((in_atras==(1<<14))&&(in_adelante!=(1<<13))){
                carro_adelante(1000);
            }
            if((in_atras!=(1<<14))&&(in_adelante==(1<<13))){
                carro_atras(1000);
            }
        }
    }
}

void RCC_SETUP(){
	RCC-> AHB1ENR |= 0xFF; //Activa los puertos de la A a la F
    RCC->APB1ENR |= (1<<18)|(1<<3); //habilita el USART3 y Habilita TIM5

}
void GPIO_SETUP(){
    GPIOC->MODER |= (0<<26); // PC13 es el boton de Usuario que sera el boton para el Paro de Emergencia, Input
    GPIOC->PUPDR |= (2<<26); // pull down el pin PC13

    GPIOE->MODER |= (1<<14)|(1<<16)|(1<<18)|(1<<20); //PE7, PE8, PE9, PE10 como salida para el control del puente H
    GPIOB->MODER |= (1<<14)|(1<<28); //LEDS PRUEBA de la placa PB7 y PB14

    GPIOF->MODER |= (0<<26)|(0<<28); //PF13 y PF14 como entradas para el avanzado y retroceso manual
    GPIOF->PUPDR |= (2<<26)|(2<<28); //PF13 y PF14 pull down

    GPIOD->MODER |= 0xA0000;  //Pines a usar por el USART3 PD8(TX) y PD9(RX)
    GPIOD->AFR[1] |= 0x77; // Viendo el Datasheet se puede observar que el PD8 7 PD9 para USART3 se debe activar el AF7 y estan en la parte high del AFR

    //GPIOC->MODER |= (2<<22)|(2<<20);  //Pines a usar por el USART3 PC10(TX) y PC11(RX)
    //GPIOC->AFR[1] |= 0x7700; // Viendo el Datasheet se puede observar que el PC10 y PC11 para USART3 se debe activar el AF7 y estan en la parte high del AFR

    GPIOA->MODER |=(2<<0)|(2<<2)|(2<<4)|(2<<6); //Modo alternante para el TIM5|PWM, PA0, PA1, PA2, PA3
    GPIOA->AFR[0] |= 0x2222; //AF2 para el TIM5 de PA0, PA1, PA2 y PA3

}
void USART_SETUP(){
    USART3->BRR |= 0x683; //BAUD RATE 9600
    USART3->CR1 |= ((0x2D) | (0<<15)); //no hay oversampling del baud rate, se activa USART EN, Reciever y Transmiter EN, RX INTERRUPT EN
    NVIC_EnableIRQ(USART3_IRQn); //nombre en tabla del NVIC, interrupción
}
void TIM_Config(){
    TIM5->PSC = 16-1;  // Para tener 50Hz
    TIM5->ARR = 20000-1; //ARR
    TIM5->DIER |= 0x1; //Habilita Interrupcion
    TIM5->EGR |= 0x1; //Reinicio del conteo
	TIM5->CCMR1|=0X6060; //Modo PWM Tim5  CH1 y 2 - Como salida 00 en CC1S
	TIM5->CCMR2|=0X6060; //Modo PWM Tim5  CH3 y 4 - Como salida 00 en CC1S
	TIM5->CCER|=(1<<0) | (1<<4)|(1<<8) | (1<<12);  //Habilitar salida de comparacion y captura
    TIM5->CR1|=0X1; //Habilitar conteo
	NVIC_EnableIRQ(TIM5_IRQn); //Por si queremos la interrupci?n que igual trabaja con el ARR
        
}
void carro_adelante(int potencia){
    GPIOB->ODR &= ~0xFFFF;
    GPIOE->ODR &= ~0xFFFF;
    GPIOE->ODR |= (1<<7)|(1<<8)|(0<<9)|(0<<10);
    GPIOB->ODR |= (1<<7)|(0<<14);
    TIM5->CCR1=potencia; //PA0
}
void carro_atras(int potencia){
    GPIOE->ODR &= ~0xFFFF;
    GPIOB->ODR &= ~0xFFFF;
    GPIOE->ODR |= (0<<7)|(0<<8)|(1<<9)|(1<<10);
    GPIOB->ODR |= (0<<7)|(1<<14);
    TIM5->CCR1=potencia; //PA0
}
void paro_emergencia(){
    GPIOE->ODR &= ~0xFFFF;
    GPIOB->ODR &= ~0xFFFF;
    SysTick_Wait1ms(10000);
}
int USART3_SendChar(int value){
    USART3->TDR = value;
   while(!(USART3->ISR & USART_ISR_TXE));
   return 0;
}
void Test_Serial(){
	SysTick_Wait1ms(1500);	//pobjeto prueba para saber si la comunicacion serial esta sirviendo
	USART3_SendChar('P');    
}
void Test_Timer(){
    TIM5->CCR1=5000; //PA0
	TIM5->CCR2=1000; //PA1
	TIM5->CCR3=1500; //PA2
	TIM5->CCR4=2000; //PA3
}

// funciones de los delays
void SysTick_Init(void) {
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}
void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0);
}
void SysTick_Wait1ms(uint32_t delay) {
    for (uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16000);
    }
}
//interrupciones
extern "C"{
    void TIM5_IRQHandler(void){
        TIM5->SR &=~(1UL<<0);
        GPIOB->ODR^=(1<<0);   
    }

    void USART3_IRQHandler(void){
        while(USART3->ISR & USART_ISR_RXNE){
            // lo que quiere que se haga cuando se reciba un dato por partde de la comunicación serial
            char rx = (char)(USART3->RDR & 0xFF);
            switch(rx){
                case 'A':
                    carro_adelante(potePWM[rx]);
                    break; 
                case 'B':
                    carro_atras(potePWM[rx]);
                    break;
                default:
                    GPIOE->ODR &= ~0xFFFF;
                    GPIOB->ODR &= ~0xFFFF;

            }
        }
    }
}