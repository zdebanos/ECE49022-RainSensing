
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) { while (1) {} }

/* LCD pins */
#define LCD_RS_PORT GPIOB
#define LCD_RS_PIN  GPIO_PIN_5
#define LCD_E_PORT  GPIOB
#define LCD_E_PIN   GPIO_PIN_4
#define LCD_D4_PORT GPIOC
#define LCD_D4_PIN  GPIO_PIN_7
#define LCD_D5_PORT GPIOB
#define LCD_D5_PIN  GPIO_PIN_6
#define LCD_D6_PORT GPIOA
#define LCD_D6_PIN  GPIO_PIN_7
#define LCD_D7_PORT GPIOA
#define LCD_D7_PIN  GPIO_PIN_6

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

static void LCD_PulseEnable(void) {
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void LCD_WriteNibble(uint8_t nibble) {
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (nibble >> 0) & 1);
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (nibble >> 1) & 1);
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (nibble >> 2) & 1);
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (nibble >> 3) & 1);
    LCD_PulseEnable();
}

static void LCD_SendByte(uint8_t byte, uint8_t isData) {
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, isData ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_WriteNibble(byte >> 4);
    LCD_WriteNibble(byte & 0x0F);
    HAL_Delay(2);
}

static void LCD_Init(void) {
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_E_PORT,  LCD_E_PIN,  GPIO_PIN_RESET);
    LCD_WriteNibble(0x03); HAL_Delay(5);
    LCD_WriteNibble(0x03); HAL_Delay(1);
    LCD_WriteNibble(0x03); HAL_Delay(1);
    LCD_WriteNibble(0x02); HAL_Delay(1);
    LCD_SendByte(0x28, 0);
    LCD_SendByte(0x0C, 0);
    LCD_SendByte(0x06, 0);
    LCD_SendByte(0x01, 0);
    HAL_Delay(2);
}

static void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendByte(addr, 0);
}

static void LCD_Print(const char *str) {
    while (*str) LCD_SendByte((uint8_t)*str++, 1);
}

static void LCD_Clear(void) {
    LCD_SendByte(0x01, 0);
    HAL_Delay(2);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } else if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

static void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 9;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan1);
}

static void MX_CAN2_Init(void) {
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 9;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = ENABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan2);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* LCD pins on PA6, PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LCD pins on PB4, PB5, PB6 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LCD pin on PC7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int main(void) {
    HAL_Init();
    MX_GPIO_Init();
    LCD_Init();

    LCD_SetCursor(0, 0);
    LCD_Print("CAN Starting...");

    MX_CAN1_Init();
    MX_CAN2_Init();

    /* CAN2 receive filter */
    CAN_FilterTypeDef filter;
    filter.FilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("Waiting for RX");

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x123;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint8_t TxData[8] = "HELLO!!!";
    uint32_t TxMailbox;
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (1) {
        /* Send on CAN1 every second */
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

        HAL_Delay(100);

        /* Check CAN2 for received message */
        if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
            if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                char line1[17];
                char line2[17];

                /* Line 1: ID and DLC */
                snprintf(line1, sizeof(line1), "ID:0x%03lX DLC:%lu",
                    RxHeader.StdId, RxHeader.DLC);

                /* Line 2: data as text */
                snprintf(line2, sizeof(line2), "%.8s", (char*)RxData);

                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_Print(line1);
                LCD_SetCursor(1, 0);
                LCD_Print(line2);
            }
        }

        HAL_Delay(900);
    }
}
