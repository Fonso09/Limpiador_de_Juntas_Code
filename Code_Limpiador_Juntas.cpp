#include <stm32f7xx.h>
void SysTick_Init(void);                   //funciones para los delays
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void RCC_SETUP();
void GPIO_SETUP();
void USART_SETUP();
int USART3_SendChar(int value);
void carro_adelante();
void carro_atras();
void paro_emergencia();
int in_adelante=0;
int in_atras=0;
int in_E_stop=0;

int main(){
    //Inicializar funciones y los setups 
	RCC_SETUP();
    GPIO_SETUP();
    SysTick_Init();	
    USART_SETUP();

   

    while(1){
		SysTick_Wait1ms(1500);	//pobjeto prueba para saber si la comunicacion serial esta sirviendo
		USART3_SendChar('P');
        
        //Entradas 
        in_adelante= (GPIOF->IDR)& (1<<13);
        in_atras= (GPIOF->IDR)& (1<<14);
        in_E_stop= (GPIOC->IDR)& (1<<13);
        GPIOB->ODR &= ~0xFFFF;
        GPIOE->ODR &= ~0xFFFF;

        //Seleccionar funcion del carro, de manera manual con botones físicos:
        if(in_E_stop==(1<<13))
        {
            paro_emergencia();
        } else{
            if((in_atras==(1<<14))&&(in_adelante!=(1<<13))){
                carro_adelante();
            }
            if((in_atras!=(1<<14))&&(in_adelante==(1<<13))){
                carro_atras();
            }
        }
        


    }


}

void RCC_SETUP(){
	RCC-> AHB1ENR |= 0xFF; //Activa los puertos de la A a la F
    RCC->APB1ENR |= (1<<18); //habilita el USART3

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

}
void USART_SETUP(){
    USART3->BRR |= 0x683; //BAUD RATE 9600
    USART3->CR1 |= ((0x2D) | (0<<15)); //no hay oversampling del baud rate, se activa USART EN, Reciever y Transmiter EN, RX INTERRUPT EN
    NVIC_EnableIRQ(USART3_IRQn); //nombre en tabla del NVIC, interrupción
}
void carro_adelante(){
    GPIOB->ODR &= ~0xFFFF;
    GPIOE->ODR |= (1<<7)|(1<<8)|(0<<9)|(0<<10);
    GPIOB->ODR |= (1<<7)|(0<<14);
}
void carro_atras(){
    GPIOB->ODR &= ~0xFFFF;
    GPIOE->ODR |= (0<<7)|(0<<8)|(1<<9)|(1<<10);
    GPIOB->ODR |= (0<<7)|(1<<14);
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
    void USART3_IRQHandler(void){
        while(USART3->ISR & USART_ISR_RXNE){
            // lo que quiere que se haga cuando se reciba un dato por partde de la comunicación serial
            char rx = (char)(USART3->RDR & 0xFF);
            switch(rx){
                case 'A':
                    carro_adelante();
                    break;
                case 'B':
                    carro_atras();
                    break;
                //default:
                    //GPIOE->ODR &= ~0xFFFF;

            }
        }
    }
}