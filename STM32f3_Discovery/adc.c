/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
codigo para en sensado de masa con dos sensores uno usando el ADC1 y otro usando el ADC2
Los sensores van a los pines:

adc1: pin  stm
adc2: pin  stm
*/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#define LBLUE GPIOE, GPIO8
#define LRED GPIOE, GPIO9
#define LORANGE GPIOE, GPIO10
#define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
#define LRED2 GPIOE, GPIO13
#define LORANGE2 GPIOE, GPIO14
#define LGREEN2 GPIOE, GPIO15

#define LD4 GPIOE, GPIO8
#define LD3 GPIOE, GPIO9
#define LD5 GPIOE, GPIO10
#define LD7 GPIOE, GPIO11
#define LD9 GPIOE, GPIO12
#define LD10 GPIOE, GPIO13
#define LD8 GPIOE, GPIO14
#define LD6 GPIOE, GPIO15


static void adc_setup(void)
{
	//ADC
	rcc_periph_clock_enable(RCC_ADC12);
	rcc_periph_clock_enable(RCC_GPIOA);
	//ADC
	//void gpio_set_output_options(uint32_t gpioport, uint8_t otype, uint8_t speed, uint16_t gpios)
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);//configura el pin 0 del puerto A, ADC1_IN1
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);//pin 1 de puerto A, ADC1_IN2
	gpio_mode_setup(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);//pin 4 de puerto F, ADC_IN5
	adc_power_off(ADC1);
	adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC1);//ADC realiza lectura de pin o pines y se detiene

//	adc_set_continuous_conversion_mode(ADC1);//realiza lecturas repetitivamente (yo agregué)

	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);//se dejarán ceros a la izquierda en el rejistro datos a la dercha
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_61DOT5CYC);
//	uint8_t channel_array[] = { 0, 2 ,0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0}; /* ADC1_IN1 (PA0) */ ///cambio aqui
	uint8_t channel_array[] = {1}; /* ADC1_IN1 (PA0) */ ///cambio aqui
	adc_set_regular_sequence(ADC1, 1, channel_array);//secuencia a leer los canales del ADC cambio aqui, está en adc_common_v2_multi.c pero no entiendo por qué hace |=
	adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);//pone resolución en 12 bits
	adc_power_on(ADC1);//enciende ADC

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");

}

static void adc_setup2(void)
{
	//ADC
//	rcc_periph_clock_enable(RCC_ADC12); ya se hizo en la otra func
//	rcc_periph_clock_enable(RCC_GPIOA); ya se hizo también
	//ADC
	//void gpio_set_output_options(uint32_t gpioport, uint8_t otype, uint8_t speed, uint16_t gpios)
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);//configura el pin 4 del puerto A, ADC2_IN1
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);//pin 1 de puerto A, ADC1_IN2
//	gpio_mode_setup(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);//pin 4 de puerto F, ADC_IN5
	adc_power_off(ADC2);
	adc_set_clk_prescale(ADC2, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC2);//ADC realiza lectura de pin o pines y se detiene

//	adc_set_continuous_conversion_mode(ADC1);//realiza lecturas repetitivamente (yo agregué)

	adc_disable_external_trigger_regular(ADC2);
	adc_set_right_aligned(ADC2);//se dejarán ceros a la izquierda en el rejistro datos a la dercha
	/* We want to read the temperature sensor, so we have to enable it. */
	//adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_61DOT5CYC);
//	uint8_t channel_array[] = { 0, 2 ,0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0}; /* ADC1_IN1 (PA0) */ ///cambio aqui
	uint8_t channel_array[] = {1}; /* ADC1_IN1 (PA0) */ ///cambio aqui
	adc_set_regular_sequence(ADC2, 1, channel_array);//secuencia a leer los canales del ADC cambio aqui, está en adc_common_v2_multi.c pero no entiendo por qué hace |=
	adc_set_resolution(ADC2, ADC_CFGR1_RES_12_BIT);//pone resolución en 12 bits
	adc_power_on(ADC2);//enciende ADC

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");

}



static void adc_setup3(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);//configura el pin 1 del puerto B, ADC3_IN1
	adc_power_off(ADC3);
	adc_set_clk_prescale(ADC3, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC3);//ADC realiza lectura de pin o pines y se detiene
	adc_disable_external_trigger_regular(ADC3);
	adc_set_right_aligned(ADC3);//se dejarán ceros a la izquierda en el rejistro datos a la dercha
	adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_61DOT5CYC);//frecuencia de lectura
	uint8_t channel_array[] = {1}; /* ADC3_IN1 (PB1) */ ///cambio aqui
	adc_set_regular_sequence(ADC3, 1, channel_array);//secuencia a leer los canales del ADC cambio aqui, está en adc_common_v2_multi.c pero no entiendo por qué hace |=
	adc_set_resolution(ADC3, ADC_CFGR1_RES_12_BIT);//pone resolución en 12 bits
	adc_power_on(ADC3);//enciende ADC
	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");
}


static void adc_setup4(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO12);//configura el pin 4 del puerto A, ADC2_IN1
	adc_power_off(ADC4);
	adc_set_clk_prescale(ADC4, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC4);//ADC realiza lectura de pin o pines y se detiene
	adc_disable_external_trigger_regular(ADC4);
	adc_set_right_aligned(ADC4);//se dejarán ceros a la izquierda en el rejistro datos a la dercha
	adc_set_sample_time_on_all_channels(ADC4, ADC_SMPR_SMP_61DOT5CYC);
	uint8_t channel_array[] = {1}; /* ADC1_IN1 (PA0) */ ///cambio aqui
	adc_set_regular_sequence(ADC4, 1, channel_array);//secuencia a leer los canales del ADC cambio aqui, está en adc_common_v2_multi.c pero no entiendo por qué hace |=
	adc_set_resolution(ADC4, ADC_CFGR1_RES_12_BIT);//pone resolución en 12 bits
	adc_power_on(ADC4);//enciende ADC
	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);/// para usar puerto B
//	rcc_periph_clock_enable(RCC_GPIOA);/// para puerto b

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

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
}

static void my_usart_print_int(uint32_t usart, int16_t value)
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	if (value == 0) {
		usart_send_blocking(usart, '0');
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits-1; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}
//nodemcu.readthedocs.io/en/dev/	while (!(adc_eoc(ADC2)/*and entre ADC_ISR(adc) y ADC_ISR_EOC */));


int main(void)
{
	uint16_t temp;

	clock_setup();
	gpio_setup();
	adc_setup();
	adc_setup2();
//	adc_setup3();
//	adc_setup4();
	usart_setup();
		
	uint16_t sensado = 0;
	uint16_t sensado3 = 0;
	uint16_t sensado4 = 0;

	float voltajeSTM = 3.28;
	int resistenciaDiv = 3300;
	float valorPre;
	float resistencia;
	float conductancia;
	float fuerza;
	int peso;


	while (1) {
		adc_start_conversion_regular(ADC1);// void pone un bit en ADC_CR para empezar muestreo de canales, ADC_CD(adc) |= ADC_CR_ADSTART bitwise OR deja lo demás intacto se supone
		adc_start_conversion_regular(ADC2);// void pone un bit en ADC_CR para empezar muestreo de canales, ADC_CD(adc) |= ADC_CR_ADSTART bitwise OR deja lo demás intacto se supone
//adc_start_conversion_regular(ADC3);
//adc_start_conversion_regular(ADC4);
		while (!(adc_eoc(ADC1)/*and entre ADC_ISR(adc) y ADC_ISR_EOC */));
		while (!(adc_eoc(ADC2)/*and entre ADC_ISR(adc) y ADC_ISR_EOC */));
//		while (!(adc_eoc(ADC3)));
//		while (!(adc_eoc(ADC4)));

		temp=adc_read_regular(ADC1);//retorna un uint32 en reg ADC_DR(adc) los 12 bits leidos estan en la mitad baja del reg acomodados hacia la derecha según se configuró arriba.
		sensado =adc_read_regular(ADC2);//retorna un uint32 en reg ADC_DR(adc) los 12 bits leidos estan en la mitad baja del reg acomodados hacia la derecha según se configuró arriba.

//		sensado3 =adc_read_regular(ADC3);
//		sensado4 =adc_read_regular(ADC4);

		valorPre = sensado*voltajeSTM/4096;//sparkfun estos valores no me sirven, calibrar con balanza 
		resistencia = resistenciaDiv*(voltajeSTM/sensado -1);
		conductancia = 1/resistencia;
		if(resistencia <= 600){
			fuerza = (conductancia-0.00075)/0.00000032639;
		}
		else
			fuerza = conductancia/0.000000642857;

		peso = (int)fuerza;
//		sensado=adc_read_regular(ADC1);
 		gpio_port_write(GPIOE, temp << 4);
		my_usart_print_int(USART2, temp);//envia el dato por usart 2
		my_usart_print_int(USART2, sensado);//envia el dato por usart 
//		my_usart_print_int(USART2, sensado3);//envia el dato por usart 2
//		my_usart_print_int(USART2, sensado4);//envia el dato por usart 2

//		my_usart_print_int(USART2, peso);//envia el dato por usart 2



//		my_usart_print_int(USART2, sensado[0]);
//		my_usart_print_int(USART2, sensado[1]);
//		my_usart_print_int(USART2, sensado[2]);
	}

	return 0;
}

