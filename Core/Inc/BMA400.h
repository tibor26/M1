#ifndef __BMA400_H
#define __BMA400_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void bma400_init(void);
uint8_t bma400_get_chipid(void);
uint16_t bma400_get_fifo_length(void);
void bma400_read_fifo_start(void);
uint16_t bma400_get_dma_count(void);
uint8_t bma400_get_sensor_detected(void);
uint16_t bma400_read_fifo(void);
void bma400_fifo_flush(void);
void bma400_sleep_mode(void);
void bma400_normal_mode(void);
uint16_t bma400_newton_sqrt(uint32_t value, uint16_t prev);
uint8_t check_accel_data(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMA400_H */
