#include "nrf24l01.h"

// --- Hardware Variables (Private) ---
// Bu degiskenler Init fonksiyonu ile doldurulacak
static SPI_HandleTypeDef *NRF_SPI;
static GPIO_TypeDef      *NRF_CE_PORT;
static uint16_t           NRF_CE_PIN;
static GPIO_TypeDef      *NRF_CSN_PORT;
static uint16_t           NRF_CSN_PIN;

/* --- REGISTER ADRESLERI --- */
#define NRF24_CMD_R_REGISTER       0x00
#define NRF24_CMD_W_REGISTER       0x20
#define NRF24_CMD_R_RX_PAYLOAD     0x61
#define NRF24_CMD_W_TX_PAYLOAD     0xA0
#define NRF24_CMD_FLUSH_TX         0xE1
#define NRF24_CMD_FLUSH_RX         0xE2
#define NRF24_CMD_NOP              0xFF

#define NRF24_REG_CONFIG           0x00
#define NRF24_REG_EN_AA            0x01
#define NRF24_REG_EN_RXADDR        0x02
#define NRF24_REG_SETUP_AW         0x03
#define NRF24_REG_SETUP_RETR       0x04
#define NRF24_REG_RF_CH            0x05
#define NRF24_REG_RF_SETUP         0x06
#define NRF24_REG_STATUS           0x07
#define NRF24_REG_RX_ADDR_P0       0x0A
#define NRF24_REG_TX_ADDR          0x10
#define NRF24_REG_RX_PW_P0         0x11

#define NRF24_STATUS_RX_DR         (1 << 6)
#define NRF24_STATUS_TX_DS         (1 << 5)
#define NRF24_STATUS_MAX_RT        (1 << 4)
#define NRF24_CONFIG_PWR_UP        (1 << 1)
#define NRF24_CONFIG_PRIM_RX       (1 << 0)


/* --- LOW LEVEL HELPERS --- */
static void CE_Set(void)   { HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET); }
static void CE_Reset(void) { HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET); }
static void CSN_Set(void)  { HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET); }
static void CSN_Reset(void){ HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET); }

static uint8_t SPI_Byte(uint8_t data) {
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(NRF_SPI, &data, &rxData, 1, 1000);
    return rxData;
}

static void WriteReg(uint8_t reg, uint8_t data) {
    CSN_Reset();
    SPI_Byte(NRF24_CMD_W_REGISTER | reg);
    SPI_Byte(data);
    CSN_Set();
}

static uint8_t ReadReg(uint8_t reg) {
    uint8_t data;
    CSN_Reset();
    SPI_Byte(NRF24_CMD_R_REGISTER | reg);
    data = SPI_Byte(0xFF);
    CSN_Set();
    return data;
}

static void WriteRegMulti(uint8_t reg, uint8_t* pData, uint8_t size) {
    CSN_Reset();
    SPI_Byte(NRF24_CMD_W_REGISTER | reg);
    for(uint8_t i=0; i<size; i++) SPI_Byte(pData[i]);
    CSN_Set();
}

static void WriteCmd(uint8_t cmd) {
    CSN_Reset();
    SPI_Byte(cmd);
    CSN_Set();
}

/* --- BASLATMA (INITIALIZATION) --- */
void NRF24_Init(SPI_HandleTypeDef *hspi,
                GPIO_TypeDef *CE_Port, uint16_t CE_Pin,
                GPIO_TypeDef *CSN_Port, uint16_t CSN_Pin)
{
    // Donanim bilgilerini kaydet
    NRF_SPI = hspi;
    NRF_CE_PORT = CE_Port;
    NRF_CE_PIN = CE_Pin;
    NRF_CSN_PORT = CSN_Port;
    NRF_CSN_PIN = CSN_Pin;

    // Baslangic durumlari
    CE_Reset();
    CSN_Set();
    HAL_Delay(100);

    // Temel Ayarlar
    WriteReg(NRF24_REG_CONFIG, 0x08);     // CRC Enable
    WriteReg(NRF24_REG_EN_AA, 0x3F);      // Auto-Ack
    WriteReg(NRF24_REG_EN_RXADDR, 0x03);  // Pipe 0 ve 1
    WriteReg(NRF24_REG_SETUP_AW, 0x03);   // 5 Byte Adres
    WriteReg(NRF24_REG_SETUP_RETR, 0x2F); // 750us, 15 retry

    // Varsayilanlar
    NRF24_SetRFChannel(90);
    NRF24_SetDataRate(NRF24_DR_1MBPS);
    NRF24_SetOutputPower(NRF24_PA_MAX);
    NRF24_SetCRCLength(NRF24_CRC_16);

    NRF24_ClearInterrupts();
    NRF24_FlushRX();
    NRF24_FlushTX();

    // Power Up
    uint8_t config = ReadReg(NRF24_REG_CONFIG);
    if (!(config & NRF24_CONFIG_PWR_UP)) {
        WriteReg(NRF24_REG_CONFIG, config | NRF24_CONFIG_PWR_UP);
        HAL_Delay(2);
    }
}

/* --- AYAR FONKSIYONLARI --- */
void NRF24_SetRFChannel(uint8_t channel) {
    if (channel > 125) channel = 125;
    WriteReg(NRF24_REG_RF_CH, channel);
}

void NRF24_SetDataRate(NRF24_DataRate_t dataRate) {
    uint8_t setup = ReadReg(NRF24_REG_RF_SETUP);
    setup &= ~((1 << 5) | (1 << 3));
    setup |= dataRate;
    WriteReg(NRF24_REG_RF_SETUP, setup);
}

void NRF24_SetOutputPower(NRF24_PowerLevel_t powerLevel) {
    uint8_t setup = ReadReg(NRF24_REG_RF_SETUP);
    setup &= ~((1 << 2) | (1 << 1));
    setup |= powerLevel;
    WriteReg(NRF24_REG_RF_SETUP, setup);
}

void NRF24_SetCRCLength(NRF24_CRC_Length_t length) {
    uint8_t config = ReadReg(NRF24_REG_CONFIG);
    config &= ~((1 << 3) | (1 << 2));
    config |= length;
    WriteReg(NRF24_REG_CONFIG, config);
}

void NRF24_SetPayloadSize(uint8_t payloadSize) {
    WriteReg(NRF24_REG_RX_PW_P0, payloadSize);
}

void NRF24_SetAutoAck(uint8_t state) {
    WriteReg(NRF24_REG_EN_AA, state ? 0x3F : 0x00);
}

void NRF24_SetTXAddress(uint8_t* pAddress) {
    WriteRegMulti(NRF24_REG_TX_ADDR, pAddress, 5);
    WriteRegMulti(NRF24_REG_RX_ADDR_P0, pAddress, 5);
}

void NRF24_SetRXAddress_P0(uint8_t* pAddress) {
    WriteRegMulti(NRF24_REG_RX_ADDR_P0, pAddress, 5);
}

/* --- VERICI (TX) --- */
void NRF24_SetTXMode(void) {
    CE_Reset();
    uint8_t config = ReadReg(NRF24_REG_CONFIG);
    config &= ~NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_PWR_UP;
    WriteReg(NRF24_REG_CONFIG, config);
}

NRF24_TX_Result_t NRF24_Transmit(uint8_t* pData, uint8_t size) {
    uint8_t status;
    CE_Reset();
    CSN_Reset();
    SPI_Byte(NRF24_CMD_W_TX_PAYLOAD);
    for(uint8_t i=0; i<size; i++) SPI_Byte(pData[i]);
    CSN_Set();

    CE_Set();
    for(volatile int i=0; i<100; i++);
    CE_Reset();

    uint32_t start = HAL_GetTick();
    while (1) {
        status = NRF24_GetStatus();
        if (status & NRF24_STATUS_TX_DS) {
            NRF24_ClearInterrupts();
            return NRF24_TX_OK;
        }
        if (status & NRF24_STATUS_MAX_RT) {
            NRF24_ClearInterrupts();
            NRF24_FlushTX();
            return NRF24_TX_MAX_RT;
        }
        if (HAL_GetTick() - start > 100) {
            NRF24_FlushTX();
            return NRF24_TX_ERROR;
        }
    }
}

/* --- ALICI (RX) --- */
void NRF24_SetRXMode(void) {
    CE_Reset();
    uint8_t config = ReadReg(NRF24_REG_CONFIG);
    config |= NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_PWR_UP;
    WriteReg(NRF24_REG_CONFIG, config);
    CE_Set();
}

void NRF24_StartListening(void) { CE_Set(); }
void NRF24_StopListening(void)  { CE_Reset(); }

uint8_t NRF24_IsDataAvailable(uint8_t* pPipeNum) {
    uint8_t status = NRF24_GetStatus();
    if (status & NRF24_STATUS_RX_DR) {
        if (pPipeNum) *pPipeNum = (status >> 1) & 0x07;
        return 1;
    }
    return 0;
}

void NRF24_Receive(uint8_t* pData, uint8_t size) {
    CSN_Reset();
    SPI_Byte(NRF24_CMD_R_RX_PAYLOAD);
    for(uint8_t i=0; i<size; i++) pData[i] = SPI_Byte(0xFF);
    CSN_Set();
    WriteReg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
}

/* --- YARDIMCI --- */
uint8_t NRF24_GetStatus(void) {
    uint8_t status;
    CSN_Reset();
    status = SPI_Byte(NRF24_CMD_NOP);
    CSN_Set();
    return status;
}

void NRF24_ClearInterrupts(void) { WriteReg(NRF24_REG_STATUS, 0x70); }
void NRF24_FlushTX(void) { WriteCmd(NRF24_CMD_FLUSH_TX); }
void NRF24_FlushRX(void) { WriteCmd(NRF24_CMD_FLUSH_RX); }
