## P600EncoderEmulation
Code is written for STM32F407VG.
## Configuration

### LEDs
- Green - **PD12**
- Orange - **PD13**
- Red - **PD14**
- Blue - **PD15**

### ADC
Used ADC1 to measure volatge on IN1 and IN2. Beetwen measurement is a little delay (~14us). Input frequency is 84MHz and is delivered by PCLK2. Prescaller divide input frequency 8 times, so frequency for ADC is 10.5MHz. 
- ADC1_IN1 - **PA1** - connect positive voltage of motor
- ADC1_IN2 - **PA2** - connect negative voltage of motor

Sampling time is 144 cycles for both channels. It gives sampling rate 72 916Hz
DMA 2 Stream 0 transfer data from ADC to memory in circular mode and half-word. 

### UART
Baudrate 115200, 8bits, 1 stop, parirty 0.
- TX - **PD5**
- RX - **PD6** 

### TIM7
Used for generate A/B encoder signal. TIM7 is clocked by APB1 bus, so input frequency is 84MHz. Prescaler is set to 7 its give 10 500 000Hz. ARR register could be from 1 to 65535. Maximum encoder frequency should be 40kHz at ARR = 262.

Pinout:
- Signal A - **PD12**
- Signal B - **PD13**

### TIM6
Used to check motor input volatage on ADC1 and correct encoder output frequency. Loop frequency is 16kHz.
PSC 1311 and ARR 3 gives 16 006Hz