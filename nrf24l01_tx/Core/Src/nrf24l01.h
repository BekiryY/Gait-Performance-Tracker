#ifndef NRF24L01_H_
#define NRF24L01_H_

#include "main.h"
// main.h dosyasi CubeIDE projelerinde standarttir ve
// dogru HAL kutuphanesini (stm32f4xx_hal.h veya f3xx) otomatik cagirir.

/* --- ENUMLAR (Ayar Secenekleri) --- */
typedef enum {
    NRF24_DR_1MBPS   = 0,
    NRF24_DR_2MBPS   = (1 << 3),
    NRF24_DR_250KBPS = (1 << 5)
} NRF24_DataRate_t;

typedef enum {
    NRF24_PA_MIN   = 0,
    NRF24_PA_LOW   = (1 << 1),
    NRF24_PA_HIGH  = (2 << 1),
    NRF24_PA_MAX   = (3 << 1)
} NRF24_PowerLevel_t;

typedef enum {
    NRF24_CRC_OFF    = 0,
    NRF24_CRC_8      = (1 << 3),
    NRF24_CRC_16     = (1 << 3) | (1 << 2)
} NRF24_CRC_Length_t;

typedef enum {
    NRF24_TX_OK,
    NRF24_TX_MAX_RT,
    NRF24_TX_ERROR
} NRF24_TX_Result_t;

/* --- FONKSIYONLAR --- */

// *** EN ONEMLI DEGISIKLIK BURADA ***
// Artık Init fonksiyonuna hangi SPI ve hangi Pinleri kullandıgımızı soyluyoruz.
void NRF24_Init(SPI_HandleTypeDef *hspi,
                GPIO_TypeDef *CE_Port, uint16_t CE_Pin,
                GPIO_TypeDef *CSN_Port, uint16_t CSN_Pin);

// Diger fonksiyonlar aynen kaliyor
void NRF24_SetRFChannel(uint8_t channel);
void NRF24_SetDataRate(NRF24_DataRate_t dataRate);
void NRF24_SetOutputPower(NRF24_PowerLevel_t powerLevel);
void NRF24_SetCRCLength(NRF24_CRC_Length_t length);
void NRF24_SetPayloadSize(uint8_t payloadSize);
void NRF24_SetAutoAck(uint8_t state);
void NRF24_SetTXAddress(uint8_t* pAddress);
void NRF24_SetRXAddress_P0(uint8_t* pAddress);

// TX (Verici)
void NRF24_SetTXMode(void);
NRF24_TX_Result_t NRF24_Transmit(uint8_t* pData, uint8_t size);

// RX (Alici)
void NRF24_SetRXMode(void);
void NRF24_StartListening(void);
void NRF24_StopListening(void);
uint8_t NRF24_IsDataAvailable(uint8_t* pPipeNum);
void NRF24_Receive(uint8_t* pData, uint8_t size);

// Yardimci
uint8_t NRF24_GetStatus(void);
void NRF24_ClearInterrupts(void);
void NRF24_FlushTX(void);
void NRF24_FlushRX(void);

#endif /* NRF24L01_H_ */
