//#ifndef TIMER_H
//#define TIMER_H

 
  void leds_init(void);

  void tim_init(void);
 
  void system_init(void);

//  void tim1_up_tim16_isr(void);

  void delay(int delay);

  void dispensarComida(void);
  void dispensarAgua(void);
  void configuracionAlimentador(void);

//#endif
