  #include <libopencm3/stm32/rcc.h>
  #include <libopencm3/stm32/gpio.h>
  #include <libopencm3/stm32/timer.h>
  #include <libopencm3/stm32/f3/nvic.h>//linea agregada
  #include "delay.h"
  #include <limits.h>
  #include <stdbool.h>
  #include <libopencm3/stm32/usart.h>

//pwm-related timer configuration
  #define SYSFREQ     64000000 //168MHz
  #define PWMFREQ        16000  //32000
  #define PWMFREQ_F       ((float )(PWMFREQ)) //32000.0f
  #define PRESCALE        1                           //freq_CK_CNT=freq_CK_PSC/(PSC[15:0]+1)
  #define PWM_PERIOD_ARR  (SYSFREQ/( PWMFREQ*(PRESCALE+1) ))
 


static void usart_setup(void)
{
        /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
        rcc_periph_clock_enable(RCC_USART2);
        rcc_periph_clock_enable(RCC_GPIOA);

        /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
        gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

        /* Setup UART parameters. */
        usart_set_baudrate(USART2, 115200);
        usart_set_databits(USART2, 8);
        usart_set_stopbits(USART2, USART_STOPBITS_1);
        usart_set_mode(USART2, USART_MODE_TX_RX);
        usart_set_parity(USART2, USART_PARITY_NONE);
        usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

        /* Finally enable the USART. */
        usart_enable(USART2);
}


void leds_init(void) {//poner pines de leds como salidas
	  	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);
		  	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8| GPIO9| GPIO10| GPIO11| GPIO12| GPIO13| GPIO14| GPIO15);
			  }
 
  void system_init(void) {//frecuencia reloj 64 Mhz  encender pines
	      rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
	          leds_init();
		    }
 
  float duty=0.5;
 
  void tim_init(void)
  {
	TIM6_CR1 = 0b0000000000000000;//reset Ctrl reg timer 6
	TIM6_CR1 |= 0b0000000010000100;//habilitar funciones de timer 6 sin encenderlo
	TIM6_CR2 = 0b0000000000000000;//no lo vamos a usar como master de otros timers...


    /* Enable TIM1 clock. and Port E clock (for outputs) */
    rcc_periph_clock_enable(RCC_TIM6);
    rcc_periph_clock_enable(RCC_GPIOE);
 
    //Set TIM1 channel (and complementary) output to alternate function push-pull'.
    //f4 TIM1=> GIO9: CH1, GPIO11: CH2, GPIO13: CH3
    //f4 TIM1=> GIO8: CH1N, GPIO10: CH2N, GPIO12: CH3N
//    gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9);
//    gpio_set_af(GPIOE, GPIO_AF2, GPIO9);
 
    /* Reset TIM1 peripheral. */
//    rcc_periph_reset_pulse(TIM1);
	rcc_periph_reset_pulse(TIM6);
//    timer_reset(TIM1); //Depreciado
 
    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     */
//    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, //For dead time and filter sampling, not important for now.
//                   TIM_CR1_CMS_CENTER_3,  //TIM_CR1_CMS_EDGE
                   //TIM_CR1_CMS_CENTER_1
                   //TIM_CR1_CMS_CENTER_2
                   //TIM_CR1_CMS_CENTER_3 la frequencia del pwm se divide a la mitad.
//                   TIM_CR1_DIR_UP);
 
TIM6_PSC = 0b1111100111111111; //prescalador en 63999 para que f sea 100 Hz y periodo 1 ms
TIM6_PSC = 0b0000000000000000; //no prescalador


//    timer_set_prescaler(TIM1, PRESCALE); //1 = disabled (max speed)
//   timer_set_repetition_counter(TIM1, 0); //disabled
//    timer_enable_preload(TIM1);
//  timer_continuous_mode(TIM1);
 
    /* Period (32kHz). */

TIM6_ARR = 0b1111101000000000;//64000 es el tope de cuenta para haber contado 1 ms sin prescalador

//    timer_set_period(TIM1, PWM_PERIOD_ARR); //ARR (value compared against main counter to reload counter aka: period of counter)
 
    /* Configure break and deadtime. */
//    timer_set_enabled_off_state_in_idle_mode(TIM1);
//    timer_set_enabled_off_state_in_run_mode(TIM1);
//    timer_disable_break(TIM1);
//    timer_set_break_polarity_high(TIM1);
//    timer_disable_break_automatic_output(TIM1);
//    timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);
 
    /* Disable outputs. */
//    timer_disable_oc_output(TIM1, TIM_OC1);
//    timer_disable_oc_output(TIM1, TIM_OC1N);
//    timer_disable_oc_output(TIM1, TIM_OC2);
//    timer_disable_oc_output(TIM1, TIM_OC2N);
//    timer_disable_oc_output(TIM1, TIM_OC3);
//    timer_disable_oc_output(TIM1, TIM_OC3N);
 
    /* -- OC1 and OC1N configuration -- */
    /* Configure global mode of line 1. */
//    timer_enable_oc_preload(TIM1, TIM_OC1);//INVestigar
//    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);//investigar
    /* Configure OC1. */
//    timer_set_oc_polarity_high(TIM1, TIM_OC1);
//    timer_set_oc_idle_state_unset(TIM1, TIM_OC1); //When idle (braked) put 0 on output
    /* Configure OC1N. */
//    timer_set_oc_polarity_high(TIM1, TIM_OC1N);
//    timer_set_oc_idle_state_unset(TIM1, TIM_OC1N);
    /* Set the capture compare value for OC1. */
//    timer_set_oc_value(TIM1, TIM_OC1, PWM_PERIOD_ARR*duty);//duty_cycle*pwm_period_ARR);


	TIM6_DIER |= 1<<8;//permite actualizaci[on de eventos


    /* ARR reload enable. */
//    timer_enable_preload(TIM1); puesto arriba en pin 7 para tim6
 
    /*
     * Enable preload of complementary channel configurations and
     * update on COM event.
     */
//    timer_disable_preload_complementry_enable_bits(TIM1);
 
    /* Enable outputs in the break subsystem. */
//    timer_enable_break_main_output(TIM1);
 
    /* Generate update event to reload all registers before starting*/
//    timer_generate_event(TIM1, TIM_EGR_UG);

	TIM6_EGR |= 1;

//    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
//    timer_enable_oc_output(TIM1, TIM_OC1 );
//    timer_disable_oc_output (TIM1, TIM_OC1N);
 
    /* Counter enable. */
//    timer_enable_counter(TIM1);
 
	TIM6_CR1 |= 0b0000000000000001;//encender contador
    //enable capture compare interrupt
//    timer_enable_update_event(TIM1);
 
//    timer_enable_irq(TIM1, TIM_DIER_UIE);
//    nvic_enable_irq(NVIC_TIM1_UP_TIM16_IRQ);
  }

//  void tim1_up_tim16_isr(void) {
int c =0;

  void tim6_up_tim16_isr(void) {
    // Clear the update interrupt flag
    timer_clear_flag(TIM6,  TIM_SR_UIF);
//    gpio_toggle(GPIOE, GPIO11);	/* LED on/off */
	c++;
	if(c==1000)
	art_send_blocking(usart, '\r');
	pio_toggle(GPIOE, GPIO11);
	c=0;
  }

int main(void)
	  {
	system_init();
	tim_init();
        int i;
  	while (1){
	gpio_toggle(GPIOE, GPIO12);	/* LED on/off */
	for (i = 0; i < 2000000; i++) /* Wait a bit. */
	__asm__("nop");
	 	 }
      	  }
