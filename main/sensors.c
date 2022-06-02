/*
 * sensors.c
 *
 *  Created on: 13 avr. 2022
 *      Author: Alaa_YAKHNI
 */
#include "sensors.h"



 int32_t i2c_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (STTS751_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (STTS751_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave memory like device,
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
 * --------|---------------------------|---------------------------------------|----------------------|------|
 *
 */
 int32_t i2c_master_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (STTS751_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
 esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // Pullup resistors are already present on X-NUCLEO-IKS01A3
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
 void get_temp_task(void *args)
{
	/* Read output only if not busy */
	uint8_t flag;

 while (1) {
	stts751_flag_busy_get(&dev_ctx, &flag);
	if (flag)
	{
	  /* Read temperature data */
	  memset(data_raw_temperature.u8bit, 0, sizeof(int16_t));
	  stts751_temperature_raw_get(&dev_ctx, &data_raw_temperature.i16bit);
	  temperature_degC = stts751_from_lsb_to_celsius(data_raw_temperature.i16bit);

	  printf("Temperature [degC]:%3.2f\r\n\n", temperature_degC);
	  vTaskDelay(pdMS_TO_TICKS( 1000 ));
	}
   }
}

void init_stts751()
{
	 /* This acts as the entry point of ST's STTS751 driver */
	    dev_ctx.write_reg = i2c_master_write_slave;
	    dev_ctx.read_reg = i2c_master_read_slave;
	    dev_ctx.i2c_number = SENSOR_BUS;

	    /* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
	    float temperature_high_limit = 49.5f;
	    stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

	    float temperature_low_limit = -4.5f;
	    stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

	    stts751_pin_event_route_set(&dev_ctx,  PROPERTY_ENABLE);

	      /* Set Output Data Rate */
	    stts751_temp_data_rate_set(&dev_ctx, STTS751_TEMP_ODR_250mHz);

	      /* Set Resolution */
	    stts751_resolution_set(&dev_ctx, STTS751_11bit);
}






/// FOn
int32_t i2c_LPS22HH_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
   if (size == 0) {
       return ESP_OK;
   }
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (LPS22HH_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (LPS22HH_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
   if (size > 1) {
       i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}

/**
* @brief Test code to write esp-i2c-slave
*        Master device write data to slave memory like device,
*        the data will be stored in slave buffer.
*        We can read them out from slave buffer.
*
* __________________________________________________________________________________________________________
* | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
* --------|---------------------------|---------------------------------------|----------------------|------|
*
*/
int32_t i2c_LPS22HH_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (LPS22HH_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
   i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}



//// fonctions de LPS22HH///
void lps22hh_init()
{


	  /* Initialize mems driver interface */
	  dev_ctxp.write_reg =i2c_LPS22HH_write_slave ;
	  dev_ctxp.read_reg = i2c_LPS22HH_read_slave;
	  dev_ctxp.i2c_number= SENSOR_BUS;
	  /* Initialize platform specific hardware */

	  /* Check device ID */
	  whoamI = 0;
	  lps22hh_device_id_get(&dev_ctxp, &whoamI);



	  /* Restore default configuration */
	  lps22hh_reset_set(&dev_ctxp, PROPERTY_ENABLE);

	  do {
	    lps22hh_reset_get(&dev_ctxp, &rst);
	  } while (rst);

	  /* Enable Block Data Update */
	  lps22hh_block_data_update_set(&dev_ctxp, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  lps22hh_data_rate_set(&dev_ctxp, LPS22HH_10_Hz_LOW_NOISE);

}

void get_press(void *args)
{
	  lps22hh_reg_t reg;
	while (1) {
	    /* Read output only if new value is available */
	    lps22hh_read_reg(&dev_ctxp, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	    if (reg.status.p_da) {
	      memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	      lps22hh_pressure_raw_get(&dev_ctxp, &data_raw_pressure);
	      pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
	      printf( "pressure [hPa]:%6.2f\r\n", pressure_hPa);
	      vTaskDelay(pdMS_TO_TICKS( 1000 ));
	    }
}
}




/////////////////////////////////////
int32_t i2c_hts221_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
   if (size == 0) {
       return ESP_OK;
   }
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (HTS221_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (HTS221_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
   if (size > 1) {
       i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}

/**
* @brief Test code to write esp-i2c-slave
*        Master device write data to slave memory like device,
*        the data will be stored in slave buffer.
*        We can read them out from slave buffer.
*
* __________________________________________________________________________________________________________
* | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
* --------|---------------------------|---------------------------------------|----------------------|------|
*
*/
int32_t i2c_hts221_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (HTS221_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
   i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}

float linear_interpolation(lin_t *lin, int16_t x)
{
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
}


void hts221_init()
{
	 dev_hum.write_reg = i2c_hts221_write_slave;
	  dev_hum.read_reg = i2c_hts221_read_slave;
	  dev_hum.i2c_number = SENSOR_BUS;
	  /* Check device ID */
	  whoamI1 = 0;
	  hts221_device_id_get(&dev_hum, &whoamI1);


	  /* Read humidity calibration coefficient */

	  hts221_hum_adc_point_0_get(&dev_hum, &lin_hum.x0);
	  hts221_hum_rh_point_0_get(&dev_hum, &lin_hum.y0);
	  hts221_hum_adc_point_1_get(&dev_hum, &lin_hum.x1);
	  hts221_hum_rh_point_1_get(&dev_hum, &lin_hum.y1);

	  /* Enable Block Data Update */
	  hts221_block_data_update_set(&dev_hum, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  hts221_data_rate_set(&dev_hum, HTS221_ODR_1Hz);
	  /* Device power on */
	  hts221_power_on_set(&dev_hum, PROPERTY_ENABLE);
}
void get_hum(void *args)
{
	while (1) {
	    /* Read output only if new value is available */
	    hts221_reg_t reg;
	    hts221_status_get(&dev_hum, &reg.status_reg);

	    if (reg.status_reg.h_da) {
	      /* Read humidity data */
	      memset(&data_raw_humidity, 0x00, sizeof(int16_t));
	      hts221_humidity_raw_get(&dev_hum, &data_raw_humidity);
	      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity);

	      if (humidity_perc < 0) {
	        humidity_perc = 0;
	      }

	      if (humidity_perc > 100) {
	        humidity_perc = 100;
	      }

	      printf( "Humidity [%%]:%3.2f\r\n", humidity_perc);
	      vTaskDelay(pdMS_TO_TICKS( 1000 ));
	    }

}
}
