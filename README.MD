# ADC_DualModeInterleaved Example from STM32CubeIDE for Nucleo-H745 board. 

This project is derived from original example project and modified for specific use. 
Project is not created from CubeMX GUI. But code is written in CubeMX Style. So Adding 
new feature is a managed manual work. Most ADC configs are defined in main.h file. 
GPIO Pins PA6 (Signal) and PA7 (Sawtooth) are configured for analog inputs. 

### main.h
ADCx is ADC1 
```C
#define ADCx_CHANNELa                   ADC_CHANNEL_3 // was 18 PA4 , Now PA6 
```

