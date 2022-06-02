/*
 * sensors.h
 *
 *  Created on: 13 avr. 2022
 *      Author: Alaa_YAKHNI
 */



#ifndef MAIN_SENSORS_H_
#define MAIN_SENSORS_H_

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "stts751_reg.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18         /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define STTS751_ADDR_7BITS 0x4a
#define LPS22HH_ADDR_7BITS 0x5D
#define HTS221_ADDR_7BITS 0x5F

#define SENSOR_BUS I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;


stmdev_ctx_t dev_ctx;
stmdev_ctx_t dev_ctxp,dev_hum;
axis1bit16_t data_raw_temperature;
float temperature_degC;

 int32_t i2c_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size);
 int32_t i2c_master_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size);
 esp_err_t i2c_master_init(void);
void get_temp_task(void *args);
void init_stts751();

// privates variables de capteur LPS22HH
 uint32_t data_raw_pressure;
 float pressure_hPa;
uint8_t whoamI, rst;
int32_t i2c_LPS22HH_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size);
int32_t i2c_LPS22H_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size);
void lps22hh_init();
void get_press(void *args);

// privates variables de capteur hts221
 int16_t data_raw_humidity;
 float humidity_perc;
 typedef struct {
   float x0;
   float y0;
   float x1;
   float y1;
 } lin_t;
 lin_t lin_hum;



 uint8_t whoamI1, rst1;
 int32_t i2c_hts221_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size);
 int32_t i2c_hts221_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size);

 void hts221_init();
 void get_hum(void *args);
#endif /* MAIN_SENSORS_H_ */


