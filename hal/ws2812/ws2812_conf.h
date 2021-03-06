// Pin, timer and dma are all connected, check them all if you change one.
#define WS2812_LED_N    110 // Number of LEDs
#define PORT_WS2812     GPIOA
#define PIN_WS2812      8
#define WS2812_TIM_N    1  // timer, 1-11
#define WS2812_TIM_CH   0  // timer channel, 0-3
#define WS2812_DMA_STREAM STM32_DMA_STREAM_ID(1, 5)  // DMA stream for TIMx_UP (look up in reference manual under DMA Channel selection)
#define WS2812_DMA_CHANNEL 5                  // DMA channel for TIMx_UP
