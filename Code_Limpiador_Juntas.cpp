#include <stm32f7xx.h>
void SysTick_Init(void);                   //funciones para los delays
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void RCC_SETUP();
void GPIO_SETUP();
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

   

    while(1){
        //Entradas 
        in_adelante= (GPIOF->IDR)& (1<<13);
        in_atras= (GPIOF->IDR)& (1<<14);
        in_E_stop= (GPIOC->IDR)& (1<<13);
        GPIOB->ODR &= ~0xFFFF;
        GPIOE->ODR &= ~0xFFFF;

        //Seleccionar funcion del carro:
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

}
void GPIO_SETUP(){
    GPIOC->MODER |= (0<<26); // PC13 es el boton de Usuario que sera el boton para el Paro de Emergencia, Input
    GPIOC->PUPDR |= (2<<26); // pull down el pin PC13

    GPIOE->MODER |= (1<<14)|(1<<16)|(1<<18)|(1<<20); //PE7, PE8, PE9, PE10 como salida para el control del puente H
    GPIOB->MODER |= (1<<14)|(1<<28); //LEDS PRUEBA de la placa PB7 y PB14

    GPIOF->MODER |= (0<<26)|(0<<28); //PF13 y PF14 como entradas para el avanzado y retroceso manual
    GPIOF->PUPDR |= (2<<26)|(2<<28); //PF13 y PF14 pull down

}   
void carro_adelante(){
    GPIOE->ODR |= (1<<7)|(1<<8)|(0<<9)|(0<<10);
}
void carro_atras(){
    GPIOE->ODR |= (0<<7)|(0<<8)|(1<<9)|(1<<10);
}
void paro_emergencia(){
    GPIOE->ODR &= ~0xFFFF;
    SysTick_Wait1ms(5000);
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