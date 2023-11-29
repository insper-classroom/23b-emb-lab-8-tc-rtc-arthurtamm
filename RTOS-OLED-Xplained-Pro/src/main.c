#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreLed3;
SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xSemaphoreSeconds;
SemaphoreHandle_t xSemaphoreSetHour;
SemaphoreHandle_t xSemaphoreSetMin;

/* LEDs */
#define LED_PIO         PIOC
#define LED_PIO_ID      ID_PIOC
#define LED_PIO_IDX	8
#define LED_IDX_MASK    (1<<LED_PIO_IDX)

#define LED1_PIO PIOA // LED1
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1u << LED1_PIO_IDX)

#define LED2_PIO PIOC // LED2
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1u << LED2_PIO_IDX)

#define LED3_PIO PIOB // LED3	
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1u << LED3_PIO_IDX)

/* Botao da placa */
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define BUT1_PIO PIOD  // Botão 1
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX	28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC // Botão 2
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO PIOA // Botão 3
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

void LED_init(int estado);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pisca_led(int n, int t);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTT_Handler(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow \n");
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o código para irq de segundo vem aqui
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(xSemaphoreSeconds, &xHigherPriorityTaskWoken);
      rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(xSemaphoreLed3, &xHigherPriorityTaskWoken);
    }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		RTT_init(1000, 4000, RTT_MR_ALMIEN);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
   }  
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
}

void TC3_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 0);

	/** Muda o estado do LED (pisca) **/
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreLed3, &xHigherPriorityTaskWoken);
  tc_stop(TC1, 0);
}

void but_callback(void) {}

void but1_callback(void){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
}

void but2_callback(void){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreSetHour, &xHigherPriorityTaskWoken);
}

void but3_callback(void){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreSetMin, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  // gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  // gfx_mono_draw_string("oii", 0, 20, &sysfont);
  LED_init(0); 
  BUT_init();

  /** Configura RTC */                                                                            
  calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};                                            
  RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);                                              
                                                                                                  
  /* Leitura do valor atual do RTC */           
  uint32_t current_hour, current_min, current_sec;
  uint32_t current_year, current_month, current_day, current_week;
  rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

  RTT_init(1000, 4000, RTT_MR_ALMIEN);

  TC_init(TC0, ID_TC1, 1, 4);
	tc_start(TC0, 1);

  TC_init(TC1, ID_TC3, 0, 0.05);
  
  char hour[4], min[4], sec[4];
  
  sprintf(hour, "%d", current_hour);
  sprintf(min, "%d", current_min);
  sprintf(sec, "%d", current_sec);

  gfx_mono_draw_string(hour,  GFX_MONO_LCD_WIDTH/3, GFX_MONO_LCD_HEIGHT/2, &sysfont);
  gfx_mono_draw_string(min, GFX_MONO_LCD_WIDTH/3 + 20 , GFX_MONO_LCD_HEIGHT/2, &sysfont);
  gfx_mono_draw_string(sec, GFX_MONO_LCD_WIDTH/3 + 40, GFX_MONO_LCD_HEIGHT/2, &sysfont);

  int time_changed = 0;

  for (;;) {

    if (xSemaphoreTake(xSemaphoreBut1, 0)) {
			// rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
      // rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
      tc_start(TC1, 0);
		}

    if(xSemaphoreTake(xSemaphoreLed3, 0)){   
      printf("piscando led\n");                                                                              
      pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
      delay_ms(1000);
      pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
      delay_ms(100);
    }

    if(xSemaphoreTake(xSemaphoreRTT, 0)){
      pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
    }
    
    if(xSemaphoreTake(xSemaphoreSeconds, 0)){
      rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
      sprintf(hour, "%d", current_hour);
      sprintf(min, "%d", current_min);
      sprintf(sec, "%d", current_sec);
      time_changed = 1;
      
    }

    if(xSemaphoreTake(xSemaphoreSetHour, 0)){
      rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
      current_hour = (current_hour + 1) % 24;
      rtc_set_time(RTC, current_hour, current_min, current_sec);
      sprintf(hour, "%d", current_hour);
      sprintf(min, "%d", current_min);
      sprintf(sec, "%d", current_sec);
      time_changed = 1;
    }

    if(xSemaphoreTake(xSemaphoreSetMin, 0)){
      rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
      current_min = (current_min + 1) % 60;
      rtc_set_time(RTC, current_hour, current_min, current_sec);
      sprintf(hour, "%d", current_hour);
      sprintf(min, "%d", current_min);
      sprintf(sec, "%d", current_sec);
      time_changed = 1;
    }

    if(time_changed){
      gfx_mono_draw_filled_rect(0, 0, GFX_MONO_LCD_WIDTH, GFX_MONO_LCD_HEIGHT, GFX_PIXEL_CLR);
      gfx_mono_draw_string(hour,  GFX_MONO_LCD_WIDTH/3, GFX_MONO_LCD_HEIGHT/2, &sysfont);
      gfx_mono_draw_string(min, GFX_MONO_LCD_WIDTH/3 + 20 , GFX_MONO_LCD_HEIGHT/2, &sysfont);
      gfx_mono_draw_string(sec, GFX_MONO_LCD_WIDTH/3 + 40, GFX_MONO_LCD_HEIGHT/2, &sysfont);
      time_changed = 0;
    }
  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  /* Configure console UART. */
  stdio_serial_init(CONF_UART, &uart_serial_options);

  /* Specify that stdout should not be buffered. */
  setbuf(stdout, NULL);
}

static void BUT_init(void) {
  /* configura prioridae */
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4);

  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_SetPriority(BUT1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT2_PIO_ID);
  NVIC_SetPriority(BUT2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT3_PIO_ID);
  NVIC_SetPriority(BUT3_PIO_ID, 4);

  /* conf bot�o como entrada */
  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

  pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
  pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
  pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
  pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

  pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
  pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

  pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE, but_callback);
  pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
  pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
  pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio,mask);
}

void pisca_led (int n, int t) {
    for (int i=0;i<n;i++){
      pio_clear(LED_PIO, LED_IDX_MASK);
      delay_ms(t);
      pio_set(LED_PIO, LED_IDX_MASK);
      delay_ms(t);
    }
}

/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado) {
  pmc_enable_periph_clk(LED_PIO_ID);
  pio_set_output(LED_PIO, LED_IDX_MASK, !estado, 0, 0 );

  pmc_enable_periph_clk(LED1_PIO_ID);
  pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, !estado, 0, 0 );

  pmc_enable_periph_clk(LED2_PIO_ID);
  pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, !estado, 0, 0 );

  pmc_enable_periph_clk(LED3_PIO_ID);
  pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, !estado, 0, 0 );
};

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  configure_console();

  /* Disable the watchdog */                                                                      
  WDT->WDT_MR = WDT_MR_WDDIS;

    /* Attempt to create a semaphore. */
  xSemaphoreBut1 = xSemaphoreCreateBinary();
  if (xSemaphoreBut1 == NULL)
    printf("falha em criar o semaforo do botao 3\n");
  
  xSemaphoreLed3 = xSemaphoreCreateBinary();
  if (xSemaphoreLed3 == NULL)
    printf("falha em criar o semaforo RTC\n");

  xSemaphoreRTT = xSemaphoreCreateBinary();
  if (xSemaphoreRTT == NULL)
    printf("falha em criar o semaforo RTT\n");
	
  xSemaphoreSeconds = xSemaphoreCreateBinary();
  if (xSemaphoreSeconds == NULL)
    printf("falha em criar o semaforo seconds\n");

  xSemaphoreSetHour = xSemaphoreCreateBinary();
  if (xSemaphoreSetHour == NULL)
    printf("falha em criar o semaforo set hour\n");

  xSemaphoreSetMin = xSemaphoreCreateBinary();
  if (xSemaphoreSetMin == NULL)
    printf("falha em criar o semaforo set min\n");

  /* Create task to control oled */
  if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL,
                  TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create oled task\r\n");
  }                                                                

  vTaskStartScheduler();

  while (1) {
    pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);                                                                                          
  }
}
