#include "Usart1_MaixCam.h"
#include "Drv_Uart.h"
#include <stdio.h>

#define MAIXCAM_BAUDRATE       115200U
#define MAIXCAM_FRAME_HEADER   0x45U
#define MAIXCAM_FRAME_TAIL     0x46U
#define MAIXCAM_QUERY_BUF_SIZE 32U

typedef enum
{
    MAIXCAM_RX_WAIT_HEADER = 0,
    MAIXCAM_RX_READ_DATA,
    MAIXCAM_RX_WAIT_TAIL
} MaixCamRxState;

static volatile u8 maixcam_data_ready = RESET;
static u8 maixcam_data[MAIXCAM_RESPONSE_DATA_LENGTH];
static MaixCamRxState maixcam_rx_state = MAIXCAM_RX_WAIT_HEADER;
static u8 maixcam_rx_index = 0;
static u8 maixcam_rx_checksum = 0;

extern u8 U1RxInCnt;
extern u8 U1RxoutCnt;

void MaixCam_Init(void)
{
    DrvUart1Init(MAIXCAM_BAUDRATE);
    MaixCam_ClearRxState();
}

void MaixCam_DataAnl(u8 com_data)
{
    if (maixcam_data_ready != RESET)
    {
        return;
    }

    switch (maixcam_rx_state)
    {
    case MAIXCAM_RX_WAIT_HEADER:
        if (com_data == MAIXCAM_FRAME_HEADER)
        {
            maixcam_rx_index = 0;
            maixcam_rx_checksum = 0;
            maixcam_rx_state = MAIXCAM_RX_READ_DATA;
        }
        break;

    case MAIXCAM_RX_READ_DATA:
        maixcam_data[maixcam_rx_index] = com_data;
        if (maixcam_rx_index < (MAIXCAM_RESPONSE_DATA_LENGTH - 1U))
        {
            maixcam_rx_checksum += com_data;
        }

        maixcam_rx_index++;
        if (maixcam_rx_index >= MAIXCAM_RESPONSE_DATA_LENGTH)
        {
            maixcam_rx_state = MAIXCAM_RX_WAIT_TAIL;
        }
        break;

    case MAIXCAM_RX_WAIT_TAIL:
        if (com_data == MAIXCAM_FRAME_TAIL &&
            maixcam_data[MAIXCAM_RESPONSE_DATA_LENGTH - 1U] ==
                maixcam_rx_checksum)
        {
            maixcam_data_ready = SET;
            maixcam_rx_state = MAIXCAM_RX_WAIT_HEADER;
        }
        else if (com_data == MAIXCAM_FRAME_HEADER)
        {
            maixcam_rx_index = 0;
            maixcam_rx_checksum = 0;
            maixcam_rx_state = MAIXCAM_RX_READ_DATA;
        }
        else
        {
            maixcam_rx_state = MAIXCAM_RX_WAIT_HEADER;
        }
        break;

    default:
        maixcam_rx_state = MAIXCAM_RX_WAIT_HEADER;
        maixcam_rx_index = 0;
        maixcam_rx_checksum = 0;
        break;
    }
}

u8 MaixCam_GetDataFlag(void)
{
    if (maixcam_data_ready != RESET)
    {
        maixcam_data_ready = RESET;
        return SET;
    }

    return RESET;
}

void MaixCam_GetData(u8 *store_array)
{
    u8 i;

    if (store_array == 0)
    {
        return;
    }

    for (i = 0; i < MAIXCAM_RESPONSE_DATA_LENGTH; i++)
    {
        store_array[i] = maixcam_data[i];
    }
}

void MaixCam_ClearRxState(void)
{
    maixcam_data_ready = RESET;
    maixcam_rx_state = MAIXCAM_RX_WAIT_HEADER;
    maixcam_rx_index = 0;
    maixcam_rx_checksum = 0;

    /* Discard bytes already queued by the USART1 interrupt. */
    U1RxoutCnt = U1RxInCnt;
}

void MaixCam_SendGridQuery(u8 grid_column, u8 grid_row)
{
    char query[MAIXCAM_QUERY_BUF_SIZE];
    int query_length;
    u8 checksum;

    if (grid_column < 1U || grid_column > 9U ||
        grid_row < 1U || grid_row > 7U)
    {
        return;
    }

    MaixCam_ClearRxState();

    /*
     * Keep the checksum calculation identical to the supplied MaixCam
     * protocol implementation: numeric column/row values are added to
     * the two field letters.
     */
    checksum = (u8)('A' + grid_column + 'B' + grid_row);
    query_length = snprintf(query,
                            sizeof(query),
                            "#GRID,A%dB%d*%02X\r\n",
                            (int)grid_column,
                            (int)grid_row,
                            (unsigned int)checksum);

    if (query_length <= 0)
    {
        return;
    }
    if (query_length >= (int)sizeof(query))
    {
        query_length = sizeof(query) - 1;
    }

    DrvUart1SendBuf((u8 *)query, (u8)query_length);
}
