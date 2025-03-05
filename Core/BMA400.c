#include "BMA400.h"
#include "main.h"
#include "memory.h"


#define BMA400_SPI_NUM  1  // SPI1
#if (BMA400_SPI_NUM == 1)
extern SPI_HandleTypeDef hspi1;
#define BMA400_SPI  (&hspi1)
#else
extern SPI_HandleTypeDef hspi2;
#define BMA400_SPI  (&hspi2)
#endif

#define BMA400_12BIT         0
#if (BMA400_12BIT == 1)
#define BMA400_DATA_SIZE     6
#else
#define BMA400_DATA_SIZE     3
#endif

#define BMA400_BUFFER_SIZE  (11*(1+BMA400_DATA_SIZE))  // 11 samples (1 header, 3 or 6 bytes data)
#define BMA400_WINDOW_SIZE  32
#define BMA400_SPI_TIMEOUT  2

#define BMA400_WRITE  0x00
#define BMA400_READ   0x80
#define BMA_CHIPID    0x90

#define BMA400_REG_CHIP_ID       0x00
#define BMA400_REG_ACC_LSB       0x04
#define BMA400_REG_FIFO_LENGTH0  0x12
#define BMA400_REG_FIFO_LENGTH1  0x13
#define BMA400_REG_FIFO_DATA     0x14
#define BMA400_REG_ACC_CONFIG0   0x19
#define BMA400_REG_ACC_CONFIG1   0x1A
#define BMA400_REG_ACC_CONFIG2   0x1B
#define BMA400_REG_FIFO_CONFIG0  0x26
#define BMA400_REG_FIFO_CONFIG1  0x27
#define BMA400_REG_FIFO_CONFIG2  0x28
#define BMA400_REG_CMD           0x7E

#define BMA400_CONFIG_SLEEP       0x00
#define BMA400_CONFIG_PWR_LOW     0x01
#define BMA400_CONFIG_PWR_NORMAL  0x02
#define BMA400_CONFIG_ODR_100     (0x08 << 0)
#define BMA400_CONFIG_ODR_200     (0x09 << 0)
#define BMA400_CONFIG_ODR_400     (0x0a << 0)
#define BMA400_CONFIG_ODR_800     (0x0b << 0)
#define BMA400_CONFIG_OSR_HIGH    (0x03 << 4)
#define BMA400_CONFIG_RANGE_2G    (0x00 << 6)
#define BMA400_CONFIG_RANGE_4G    (0x01 << 6)
#define BMA400_CONFIG_RANGE_8G    (0x02 << 6)
#define BMA400_CONFIG_RANGE_16G   (0x03 << 6)
#define BMA400_CONFIG_FILT1_BW_HIGH  (0 << 7)
#define BMA400_CONFIG_FILT1_BW_LOW   (1 << 7)

// FIFO config
#define BMA400_FIFO_CONFIG_8BIT   (1 << 4)
#define BMA400_FIFO_CONFIG_12BIT  (0 << 4)
#define BMA400_FIFO_CONFIG_X_EN   (1 << 5)
#define BMA400_FIFO_CONFIG_Y_EN   (1 << 6)
#define BMA400_FIFO_CONFIG_Z_EN   (1 << 7)
#define BMA400_FIFO_CONFIG_FILT1  (0 << 3)
#define BMA400_FIFO_CONFIG_FILT2  (1 << 3)
#define BMA400_CMD_RESET          0xB6
#define BMA400_CMD_FIFO_FLUSH     0xB0

// FIFO header frames
#define BMA400_FIFO_MODE_MASK     (0x03 << 6)
#define BMA400_FIFO_AXES_MASK     (0x07 << 1)
#define BMA400_FIFO_PARAM_MASK    (0x1F << 1)
#define BMA400_FIFO_X_EN          (1 << 1)
#define BMA400_FIFO_Y_EN          (1 << 2)
#define BMA400_FIFO_Z_EN          (1 << 3)
#define BMA400_FIFO_12BIT         (1 << 4)
#define BMA400_FIFO_8BIT          (0 << 4)
#define BMA400_FIFO_TIME          (1 << 5)
#define BMA400_FIFO_MODE_DATA     (0x02 << 6)
#define BMA400_FIFO_FRAME_EMPTY   (BMA400_FIFO_MODE_DATA)
#define BMA400_FIFO_FRAME_TIME    (BMA400_FIFO_MODE_DATA | BMA400_FIFO_TIME)
#if (BMA400_12BIT == 1)
#define BMA400_FIFO_FRAME_DATA    (0x80 | BMA400_FIFO_12BIT | BMA400_FIFO_Z_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_X_EN)
#else
#define BMA400_FIFO_FRAME_DATA    (0x80 | BMA400_FIFO_8BIT | BMA400_FIFO_Z_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_X_EN)
#endif


static uint8_t bma400_detected = 0;
static uint8_t buffer_index = 0;
static uint16_t std_dev_x = 0;
static uint16_t std_dev_y = 0;
static uint16_t std_dev_z = 0;
static volatile uint8_t bma400_spi_buffer[BMA400_BUFFER_SIZE] = { 0 };
#if (BMA400_12BIT == 1)
static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;
static int16_t accel_x_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int16_t accel_y_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int16_t accel_z_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int32_t accel_sum_x = 0;
static int32_t accel_sum_y = 0;
static int32_t accel_sum_z = 0;
#else
static int8_t accel_x = 0;
static int8_t accel_y = 0;
static int8_t accel_z = 0;
static int8_t accel_x_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int8_t accel_y_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int8_t accel_z_buffer[BMA400_WINDOW_SIZE] = { 0 };
static int16_t accel_sum_x = 0;
static int16_t accel_sum_y = 0;
static int16_t accel_sum_z = 0;
#endif


static inline void bma400_nss(uint8_t value)
{
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, value);
}

static void flush_spi_rx(void)
{
    __IO uint8_t tmpreg;
    while (BMA400_SPI->Instance->SR & SPI_FLAG_FRLVL)
    {
        tmpreg = BMA400_SPI->Instance->DR; // flush SPI rx fifo
        UNUSED(tmpreg);
    }
}


void bma400_init(void)
{
    BMA400_SPI->State = HAL_SPI_STATE_READY;
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN); // disable DMA
    MODIFY_REG(SPI1_MOSI_GPIO_Port->MODER, GPIO_MODE << (SPI1_MOSI_Pin_Number * 2), MODE_AF << (SPI1_MOSI_Pin_Number * 2)); // set MOSI pin to SPI
    // NSS rising edge to switch to SPI mode
    bma400_nss(GPIO_PIN_SET);
    // dummy tx of 4 bytes to give time for the sensor to switch to SPI mode
    HAL_SPI_Transmit(BMA400_SPI, (uint8_t*)bma400_spi_buffer, 4, BMA400_SPI_TIMEOUT);
    // read chip id
    bma400_nss(GPIO_PIN_RESET);
    bma400_detected = 0;
    bma400_spi_buffer[0] = BMA400_READ | BMA400_REG_CHIP_ID;
    flush_spi_rx(); // flush SPI rx fifo
    HAL_SPI_TransmitReceive(BMA400_SPI, (uint8_t*)bma400_spi_buffer, (uint8_t*)bma400_spi_buffer, 3, BMA400_SPI_TIMEOUT);
    bma400_nss(GPIO_PIN_SET);
    if (bma400_spi_buffer[2] == BMA_CHIPID)
    {
        bma400_spi_buffer[0] = BMA400_WRITE | BMA400_REG_ACC_CONFIG0;
        bma400_spi_buffer[1] = BMA400_CONFIG_FILT1_BW_HIGH | BMA400_CONFIG_PWR_NORMAL;
        bma400_spi_buffer[2] = BMA400_WRITE | BMA400_REG_ACC_CONFIG1;
        bma400_spi_buffer[3] = BMA400_CONFIG_RANGE_2G | BMA400_CONFIG_OSR_HIGH | BMA400_CONFIG_ODR_100;
        bma400_spi_buffer[4] = BMA400_WRITE | BMA400_REG_FIFO_CONFIG0;
#if (BMA400_12BIT == 1)
        bma400_spi_buffer[5] = BMA400_FIFO_CONFIG_12BIT | BMA400_FIFO_CONFIG_Z_EN | BMA400_FIFO_CONFIG_Y_EN | BMA400_FIFO_CONFIG_X_EN | BMA400_FIFO_CONFIG_FILT1;
#else
        bma400_spi_buffer[5] = BMA400_FIFO_CONFIG_8BIT | BMA400_FIFO_CONFIG_Z_EN | BMA400_FIFO_CONFIG_Y_EN | BMA400_FIFO_CONFIG_X_EN | BMA400_FIFO_CONFIG_FILT1;
#endif
        bma400_nss(GPIO_PIN_RESET);
        HAL_SPI_Transmit(BMA400_SPI, (uint8_t*)bma400_spi_buffer, 6, BMA400_SPI_TIMEOUT);
        bma400_nss(GPIO_PIN_SET);
        // init SPI rx DMA
        BMA400_SPI->hdmarx->Instance->CCR |= DMA_MINC_ENABLE | DMA_PERIPH_TO_MEMORY | DMA_IT_TC; // enable DMA rx transfer complete interrupt
        BMA400_SPI->hdmarx->Instance->CPAR = (uint32_t)&BMA400_SPI->Instance->DR;
        bma400_detected = 1;
    }
}


uint8_t bma400_get_chipid(void)
{
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN);  // disable DMA
    flush_spi_rx(); // flush SPI rx fifo
    bma400_spi_buffer[0] = BMA400_READ | BMA400_REG_CHIP_ID;
    BMA400_SPI->State = HAL_SPI_STATE_READY;  // set state so HAL function doesn't fail
    bma400_nss(GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMA400_SPI, (uint8_t*)bma400_spi_buffer, (uint8_t*)bma400_spi_buffer, 3, BMA400_SPI_TIMEOUT);
    bma400_nss(GPIO_PIN_SET);
    return bma400_spi_buffer[2];
}


uint16_t bma400_get_fifo_length(void)
{
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN);  // disable DMA
    flush_spi_rx(); // flush SPI rx fifo
    bma400_spi_buffer[0] = BMA400_READ | BMA400_REG_FIFO_LENGTH0;
    BMA400_SPI->State = HAL_SPI_STATE_READY;   // set state so HAL function doesn't fail
    bma400_nss(GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMA400_SPI, (uint8_t*)bma400_spi_buffer, (uint8_t*)bma400_spi_buffer, 4, BMA400_SPI_TIMEOUT);
    bma400_nss(GPIO_PIN_SET);
    return (bma400_spi_buffer[3] << 8) + bma400_spi_buffer[2];
}


uint16_t bma400_read_fifo(void)
{
    if (bma400_detected == 0)
    {
        return 0;
    }
#if (BMA400_12BIT == 1)
    int16_t accel_x_tmp = 0;
    int16_t accel_y_tmp = 0;
    int16_t accel_z_tmp = 0;
#else
    int8_t accel_x_tmp = 0;
    int8_t accel_y_tmp = 0;
    int8_t accel_z_tmp = 0;
#endif
    uint8_t header_index = 0;
    while (header_index < (BMA400_BUFFER_SIZE - BMA400_DATA_SIZE))
    {
        uint8_t header = bma400_spi_buffer[header_index];
        if ((header & (BMA400_FIFO_MODE_MASK | BMA400_FIFO_PARAM_MASK)) == BMA400_FIFO_FRAME_DATA) // data frame
        {
#if (BMA400_12BIT == 1)
            accel_x_tmp = (bma400_spi_buffer[header_index + 2] << 4) | bma400_spi_buffer[header_index + 1];
            accel_y_tmp = (bma400_spi_buffer[header_index + 4] << 4) | bma400_spi_buffer[header_index + 3];
            accel_z_tmp = (bma400_spi_buffer[header_index + 6] << 4) | bma400_spi_buffer[header_index + 5];
            if (accel_x_tmp & 2048)
                accel_x_tmp -= 4096;
            if (accel_y_tmp & 2048)
                accel_y_tmp -= 4096;
            if (accel_z_tmp & 2048)
                accel_z_tmp -= 4096;
#else
            accel_x_tmp = (int8_t)bma400_spi_buffer[header_index + 1];
            accel_y_tmp = (int8_t)bma400_spi_buffer[header_index + 2];
            accel_z_tmp = (int8_t)bma400_spi_buffer[header_index + 3];
#endif
            // update sum for mean, add new value and substract oldest one
            accel_sum_x += accel_x_tmp - accel_x_buffer[buffer_index];
            accel_sum_y += accel_y_tmp - accel_y_buffer[buffer_index];
            accel_sum_z += accel_z_tmp - accel_z_buffer[buffer_index];

            // save new value in circular buffer
            accel_x_buffer[buffer_index] = accel_x_tmp;
            accel_y_buffer[buffer_index] = accel_y_tmp;
            accel_z_buffer[buffer_index] = accel_z_tmp;
            buffer_index++;
            if (buffer_index >= BMA400_WINDOW_SIZE)
            {
                buffer_index = 0;
            }
            header_index += 1 + BMA400_DATA_SIZE;
        }
        else if ((header & (BMA400_FIFO_MODE_MASK | BMA400_FIFO_AXES_MASK)) == BMA400_FIFO_FRAME_EMPTY)  // empty frame
        {
            break;
        }
        else if ((header & (BMA400_FIFO_MODE_MASK | BMA400_FIFO_PARAM_MASK)) == BMA400_FIFO_TIME) // time frame
        {
            header_index += 4;
        }
        else
        {
            break;
        }
    }
    int16_t accel_mean_x = accel_sum_x / BMA400_WINDOW_SIZE;
    int16_t accel_mean_y = accel_sum_y / BMA400_WINDOW_SIZE;
    int16_t accel_mean_z = accel_sum_z / BMA400_WINDOW_SIZE;

    accel_x = accel_x_tmp - accel_mean_x;
    accel_y = accel_y_tmp - accel_mean_y;
    accel_z = accel_z_tmp - accel_mean_z;

    // calculate standard deviation
    uint32_t std_dev_x_tmp = 0;
    uint32_t std_dev_y_tmp = 0;
    uint32_t std_dev_z_tmp = 0;
    for (uint8_t i = 0; i < BMA400_WINDOW_SIZE; i++)
    {
        std_dev_x_tmp += (accel_x_buffer[i] - accel_mean_x)*(accel_x_buffer[i] - accel_mean_x);
        std_dev_y_tmp += (accel_y_buffer[i] - accel_mean_y)*(accel_y_buffer[i] - accel_mean_y);
        std_dev_z_tmp += (accel_z_buffer[i] - accel_mean_z)*(accel_z_buffer[i] - accel_mean_z);
    }
    std_dev_x = bma400_newton_sqrt(std_dev_x_tmp, std_dev_x); // horizontal right->left axis
    std_dev_y = bma400_newton_sqrt(std_dev_y_tmp, std_dev_y); // vertical top->bottom axis
    std_dev_z = bma400_newton_sqrt(std_dev_z_tmp, std_dev_z); // horizontal back->front axis
    vibration = std_dev_x + std_dev_y + std_dev_z;
    // the sensor data is not converted to g and the last division is skipped
    // so to convert the end result to g:
    //   - divide by 2^(data bits - 2) (= 64 for 8 bit, 1024 for 12 bit), for the sensor data conversion to g
    //   - divide by sqrt(BMA400_WINDOW_SIZE), to correct the missing divide

    return vibration;
}


/*
 * Initiate a FIFO read using SPI DMA
 **/
void bma400_read_fifo_start(void)
{
    if (bma400_detected == 0)
    {
        return;
    }
    // disable DMA
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN);
    CLEAR_BIT(BMA400_SPI->hdmarx->Instance->CCR, DMA_CCR_EN);
    // set up address and size
    BMA400_SPI->hdmarx->Instance->CNDTR = BMA400_BUFFER_SIZE;
    BMA400_SPI->hdmarx->Instance->CMAR = (uint32_t)bma400_spi_buffer;
    // send FIFO read command (2 bytes are sent)
    bma400_nss(GPIO_PIN_RESET);
    BMA400_SPI->Instance->DR = BMA400_READ | BMA400_REG_FIFO_DATA;
    // disable SPI, wait tx buffer and SPI busy
    uint32_t tickstart = HAL_GetTick();
    while (((BMA400_SPI->Instance->SR & SPI_FLAG_FTLVL) || (BMA400_SPI->Instance->SR & SPI_FLAG_BSY)) && (HAL_GetTick() - tickstart) <  BMA400_SPI_TIMEOUT);
    CLEAR_BIT(BMA400_SPI->Instance->CR1, SPI_CR1_SPE);   // disable spi
    flush_spi_rx(); // flush SPI rx fifo
    // enable DMA
    SET_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(BMA400_SPI->hdmarx->Instance->CCR, DMA_CCR_EN);
    // set SPI to read-only to continue to output SCK
    SET_BIT(BMA400_SPI->Instance->CR1, SPI_CR1_RXONLY);
    // enable SPI
    SET_BIT(BMA400_SPI->Instance->CR1, SPI_CR1_SPE);
}

uint16_t bma400_get_dma_count(void)
{
    return BMA400_SPI->hdmarx->Instance->CNDTR;
}

uint8_t bma400_get_sensor_detected(void)
{
    return bma400_detected;
}


void bma400_fifo_flush(void)
{
    if (bma400_detected == 0)
    {
        return;
    }
    bma400_spi_buffer[0] = BMA400_WRITE | BMA400_REG_CMD;
    bma400_spi_buffer[1] = BMA400_CMD_FIFO_FLUSH;
    BMA400_SPI->State = HAL_SPI_STATE_READY;
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN); // disable DMA
    bma400_nss(GPIO_PIN_RESET);
    HAL_SPI_Transmit(BMA400_SPI, (uint8_t*)bma400_spi_buffer, 2, BMA400_SPI_TIMEOUT);
    bma400_nss(GPIO_PIN_SET);

    accel_sum_x = 0;
    accel_sum_y = 0;
    accel_sum_z = 0;
    std_dev_x = 0;
    std_dev_y = 0;
    std_dev_z = 0;
    buffer_index = 0;
    for (uint8_t i = 0; i < BMA400_WINDOW_SIZE; i++)
    {
        accel_x_buffer[i] = 0;
        accel_y_buffer[i] = 0;
        accel_z_buffer[i] = 0;
    }

    // read next fifo data (this will clear bma400_spi_buffer)
    bma400_read_fifo_start();
}


void bma400_sleep_mode(void)
{
    bma400_spi_buffer[0] = BMA400_WRITE | BMA400_REG_ACC_CONFIG0;
    bma400_spi_buffer[1] = BMA400_CONFIG_SLEEP;
    BMA400_SPI->State = HAL_SPI_STATE_READY;
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN); // disable DMA
    bma400_nss(GPIO_PIN_RESET);
    HAL_SPI_Transmit(BMA400_SPI, (uint8_t*)bma400_spi_buffer, 2, BMA400_SPI_TIMEOUT);
    // set MOSI to low by switching mode to GPIO output
    MODIFY_REG(SPI1_MOSI_GPIO_Port->MODER, GPIO_MODE << (SPI1_MOSI_Pin_Number * 2), MODE_OUTPUT << (SPI1_MOSI_Pin_Number * 2));
    SPI1_MOSI_GPIO_Port->BRR = SPI1_MOSI_Pin;
}


void bma400_normal_mode(void)
{
    bma400_spi_buffer[0] = BMA400_WRITE | BMA400_REG_ACC_CONFIG0;
    bma400_spi_buffer[1] = BMA400_CONFIG_FILT1_BW_HIGH | BMA400_CONFIG_PWR_NORMAL;
    BMA400_SPI->State = HAL_SPI_STATE_READY;
    CLEAR_BIT(BMA400_SPI->Instance->CR2, SPI_CR2_RXDMAEN); // disable DMA
    bma400_nss(GPIO_PIN_RESET);
    HAL_SPI_Transmit(BMA400_SPI, (uint8_t*)bma400_spi_buffer, 2, BMA400_SPI_TIMEOUT);
    bma400_nss(GPIO_PIN_SET);
}


// calculate square root using Newton's method with only one iteration
// since we use it on mean values which don't change fast, the error is small
// tests showed a max error of 2 compared to math.h sqrt() function
uint16_t bma400_newton_sqrt(uint32_t value, uint16_t prev)
{
    if (prev >= 2)
    {
        return (prev + (value / prev)) / 2;
    }
    return (1 + value / 4); // set prev to 2 so result is not 0
}


// check accel data integrity
uint8_t check_accel_data(void)
{
    uint8_t pass_zero = 0;
    uint8_t pass_ff_x = 0;
    uint8_t pass_ff_y = 0;
    uint8_t pass_ff_z = 0;
    for (uint8_t i = 0; i < BMA400_WINDOW_SIZE; i++)
    {
        // test if data is not all zeros
        if (accel_x_buffer[i] || accel_y_buffer[i] || accel_z_buffer[i])
        {
            pass_zero = 1;
        }
        // test if data is not all 0xFF
        if ((uint8_t)accel_x_buffer[i] != 0xFF)
        {
            pass_ff_x = 1;
        }
        if ((uint8_t)accel_y_buffer[i] != 0xFF)
        {
            pass_ff_y = 1;
        }
        if ((uint8_t)accel_z_buffer[i] != 0xFF)
        {
            pass_ff_z = 1;
        }
        if (pass_zero && pass_ff_x && pass_ff_y && pass_ff_z)
        {
            return 1;
        }
    }
    return 0;
}
