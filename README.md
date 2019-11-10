## P600EncoderEmulation
Code is written for STM32F407VG.
## Configuration

### LEDs
- Green - **PD12**
- Orange - **PD13**
- Red - **PD14**
- Blue - **PD15**

### ADC
ADC1 and ADC2 work in dual mode to measure voltage on both pins at the same time. Input frequency is 84MHz and is delivered by PCLK2. Prescaller divide input frequency 8 times, so frequency for ADC is 10.5MHz. 
- ADC1_IN1 - **PA1**
- ADC2_IN2 - **PA2**

Sampling time is 480 cycles for both channels. It gives sampling rate 21 875Hz.
DMA is active in mode 2. It means ADC2 data take the upper half-word and ADC1 data take the lower half-word of the transferred data.

### UART
Baudrate 115200, 8bits, 1 stop, parirty 0.
- PD5 - **TX**
- PD6 - **RX**