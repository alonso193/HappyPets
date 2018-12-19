  #include <libopencm3/stm32/rcc.h>
  #include <libopencm3/stm32/gpio.h>
  #include <libopencm3/stm32/timer.h>
  #include <libopencm3/stm32/f3/nvic.h>//linea agregada
  #include "happydogs.h"
  #include <limits.h>
  #include <stdbool.h>


//pwm-related timer configuration
  #define SYSFREQ     64000000 //168MHz
  #define PWMFREQ        16000  //32000
  #define PERIOD	64000 // para que cuente 1 ms
  #define PWMFREQ_F       ((float )(PWMFREQ)) //32000.0f
  #define PRESCALE        1                           //freq_CK_CNT=freq_CK_PSC/(PSC[15:0]+1)
 // #define PWM_PERIOD_ARR  (SYSFREQ/( PWMFREQ*(PRESCALE+1) ))
   #define PWM_PERIOD_ARR  PERIOD// si no sirve usar formula de arriba y bajar valor en PERIOD



//mis variables
#define unidadesAguaEnTanque 1000
#define unidadesComidaEnTanque 24000
#define razonDispensadoAgua 15 // ml/ms
#define razonDispensadoComida 2 // unidadesDeComidaCuantica/ms
#define puertoMotores GPIOA //y Leds
#define puertoComunicaciones GPIOC
#define puertoBotones GPIOE

//pines
int motorComida;
int motorAgua;
int ledComida;
int ledAgua;
int ledConfiguracion;

int comidaBaja;
int aguaBaja;
int comandoDispensarComida;
int comandoDispensarAgua;

int botonComida;
int botonAgua;
int botonAceptar;
int tiempo;
bool configLista = false;
bool existeConfiguracion = false;

//varibles
int disComAnt = 0;
int disAguAnt = 0;
int tiempoVaciadoAgua = 41;// s
int tiempoVaciadoComida = 24;
int tiempoDispensadoAgua = 10;
int tiempoDispensadoComida = 4;
int porcionesComida = 24/4;
int porcionesAgua = 40/10;

int porcionesComidaDispensada = 0;
int porcionesAguaDispensada = 0;


int contadorDelays = 0;





void leds_init(void) {
	  	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);
		  	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8| GPIO9| GPIO10| GPIO11| GPIO12| GPIO13| GPIO14| GPIO15);
			  }
 
  void system_init(void) {
	      rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
	          leds_init();
		    }
 
  void tim_init(void)
  {
    /* Enable TIM1 clock. and Port E clock (for outputs) */
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_GPIOE);
 
    //Set TIM1 channel (and complementary) output to alternate function push-pull'.
    //f4 TIM1=> GIO9: CH1, GPIO11: CH2, GPIO13: CH3
    //f4 TIM1=> GIO8: CH1N, GPIO10: CH2N, GPIO12: CH3N
    gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9);
    gpio_set_af(GPIOE, GPIO_AF2, GPIO9);
 
    /* Reset TIM1 peripheral. */
    rcc_periph_reset_pulse(TIM1);
//    timer_reset(TIM1); //Depreciado
 
    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, //For dead time and filter sampling, not important for now.
                   //TIM_CR1_CMS_CENTER_3,
		     TIM_CR1_CMS_EDGE,// para que sea modo sierra
                   //TIM_CR1_CMS_CENTER_1
                   //TIM_CR1_CMS_CENTER_2
                   //TIM_CR1_CMS_CENTER_3 la frequencia del pwm se divide a la mitad.
                   TIM_CR1_DIR_UP);//timer set mode (TIMX, prescalador, alineamiento, direccion de conteo)
 
    timer_set_prescaler(TIM1, PRESCALE); //1 = disabled (max speed)
    timer_set_repetition_counter(TIM1, 0); //disabled
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);
 
    /* Period (32kHz). */
    timer_set_period(TIM1, PWM_PERIOD_ARR); //ARR (value compared against main counter to reload counter aka: period of counter) //lo puse en 64000 para que cuente un ms
 
    /* Configure break and deadtime. */
    timer_set_enabled_off_state_in_idle_mode(TIM1);
    timer_set_enabled_off_state_in_run_mode(TIM1);
    timer_disable_break(TIM1);
    timer_set_break_polarity_high(TIM1);
    timer_disable_break_automatic_output(TIM1);
    timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);
 
    /* Disable outputs. */
    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_disable_oc_output(TIM1, TIM_OC1N);
    timer_disable_oc_output(TIM1, TIM_OC2);
    timer_disable_oc_output(TIM1, TIM_OC2N);
    timer_disable_oc_output(TIM1, TIM_OC3);
    timer_disable_oc_output(TIM1, TIM_OC3N);
 
    /* -- OC1 and OC1N configuration -- */
    /* Configure global mode of line 1. */
    timer_enable_oc_preload(TIM1, TIM_OC1);//INVestigar
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE);// TIM_OCM_PWM1);//
    /* Configure OC1. */
    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC1); //When idle (braked) put 0 on output
    /* Configure OC1N. */
    timer_set_oc_polarity_high(TIM1, TIM_OC1N);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC1N);
    /* Set the capture compare value for OC1. */
    timer_set_oc_value(TIM1, TIM_OC1, PWM_PERIOD_ARR);//duty_cycle*pwm_period_ARR);//dejemoslo como el periodo a ver si funca
 
    /* ARR reload enable. */
    timer_enable_preload(TIM1);
 
    /*
     * Enable preload of complementary channel configurations and
     * update on COM event.
     */
    timer_disable_preload_complementry_enable_bits(TIM1);
 
    /* Enable outputs in the break subsystem. */
    timer_enable_break_main_output(TIM1);
 
    /* Generate update event to reload all registers before starting*/
    timer_generate_event(TIM1, TIM_EGR_UG);
 
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE);//toggle en vez de pwm1
    timer_enable_oc_output(TIM1, TIM_OC1 );
    timer_disable_oc_output (TIM1, TIM_OC1N);
 
    /* Counter enable. */
    timer_enable_counter(TIM1);
 
    //enable capture compare interrupt
    timer_enable_update_event(TIM1);
 
    timer_enable_irq(TIM1, TIM_DIER_UIE);//habilita interrupciones
    nvic_enable_irq(NVIC_TIM1_UP_TIM16_IRQ);//habilita interrupcion de usuario para el timer 1 en ese modo, ese macro no existe para otros timers

  }

  void tim1_up_tim16_isr(void) {
    // Clear the update interrupt flag
    timer_clear_flag(TIM1,  TIM_SR_UIF);
    gpio_toggle(GPIOE, GPIO12);	/* LED on/off */
    contadorDelays++;
  }

  void delay(int delay){
	while(contadorDelays < delay){};
	contadorDelays = 0;
  }

  void dispensarComida(){
		gpio_set(puertoMotores, ledComida | motorComida);
		delay(tiempoDispensadoComida);
		gpio_clear(puertoMotores, ledComida | motorComida);
		porcionesComidaDispensada++;
		if(porcionesComida-porcionesComidaDispensada < 3){gpio_set(puertoComunicaciones, comidaBaja);
		delay(500);
		gpio_clear(puertoComunicaciones, comidaBaja);
		}
	}

  void dispensarAgua(){
		gpio_set(puertoMotores, ledAgua | motorAgua);
		delay(tiempoDispensadoAgua);
		gpio_clear(puertoMotores, ledAgua | motorAgua);
		porcionesAguaDispensada++;
		if(porcionesAgua-porcionesAguaDispensada < 3){gpio_set(puertoComunicaciones, aguaBaja);
		delay(500);
		gpio_clear(puertoComunicaciones, aguaBaja);
	}


	}

  void configuracionAlimentador()
	{
		gpio_set(puertoMotores, ledConfiguracion);

		while(!configLista){
			while(gpio_get(puertoBotones, botonComida)){
				gpio_set(puertoMotores, motorComida | ledComida);
				//contarTiempo;
			}
			gpio_clear(puertoMotores, motorComida | ledComida);
			//dispensar comida
			//contar tiempo que boton es presionado
			tiempoDispensadoComida += tiempo;
			if(gpio_get(puertoBotones, botonAceptar))configLista = true;
			if(gpio_get(puertoBotones, botonAgua))tiempoDispensadoComida = 0;
		
			}
		porcionesComida = unidadesComidaEnTanque/(tiempoDispensadoComida*razonDispensadoComida);

		tiempo = 0;
		configLista = false;

		while(!configLista){
			while(gpio_get(puertoBotones, botonAgua)){
				gpio_set(puertoMotores, motorAgua | ledAgua);
				//contarTiempo;
			}
			gpio_clear(puertoMotores, motorAgua | ledAgua);

			//contar tiempo que boton es presionado
			tiempoDispensadoAgua += tiempo;
			if(gpio_get(puertoBotones, botonAceptar))configLista = true;
			if(gpio_get(puertoBotones, botonComida))tiempoDispensadoAgua = 0;
		
		}
		porcionesAgua = unidadesAguaEnTanque/(tiempoDispensadoAgua*razonDispensadoAgua);

		gpio_clear(puertoMotores, ledConfiguracion);
	
	}

int main(void)
	  {
	system_init();
	tim_init();
	if(!existeConfiguracion)configuracionAlimentador();
  	while (1){
	if(gpio_get(puertoComunicaciones, comandoDispensarComida) && disComAnt == 0)dispensarComida();
	if(gpio_get(puertoComunicaciones, comandoDispensarAgua) && disAguAnt == 0)dispensarAgua();
	if(gpio_get(puertoBotones, botonComida)){porcionesComidaDispensada = 0;}
	if(gpio_get(puertoBotones, botonAgua)){porcionesAguaDispensada = 0;}

	disComAnt = gpio_get(puertoComunicaciones, comandoDispensarComida);
	disAguAnt = gpio_get(puertoComunicaciones, comandoDispensarAgua);

	 	 }
      	  }
