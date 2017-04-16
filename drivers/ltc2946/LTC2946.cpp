/****************************************************************************
 *
 *   Copyright (C) 2016 Bharath Ramaswamy. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <string.h>
#include "DriverFramework.hpp"
#include "LTC2946.hpp"
#ifdef __QURT
#include "dev_fs_lib_i2c.h"
#endif

// APM has two LTC chips, one connected to battery, one connected to 5V supply. Also two different
// current sensing resistors are used to measure the corresponding currents (for APM), and one for
// ESC.

#define LTC2946_BUS_FREQUENCY_IN_KHZ 400
#define LTC2946_TRANSFER_TIMEOUT_IN_USECS 9000

#define LTC2946_BUF_SIZE    32


using namespace DriverFramework;

int LTC2946::configure()
{
  int result;

  uint8_t CTRLA = LTC2946_CHANNEL_CONFIG_V_C_3|LTC2946_SENSE_PLUS|LTC2946_OFFSET_CAL_EVERY|LTC2946_ADIN_GND;  //! Set Control A register to default value.

  uint8_t CTRLB = LTC2946_DISABLE_ALERT_CLEAR&LTC2946_DISABLE_SHUTDOWN&LTC2946_DISABLE_CLEARED_ON_READ&LTC2946_DISABLE_STUCK_BUS_RECOVER&LTC2946_ENABLE_ACC&LTC2946_DISABLE_AUTO_RESET; //! Set Control B Register to default

  result = _writeReg(LTC2946_CTRLA_REG, &CTRLA, sizeof(CTRLA));

  if (result != 0) {
	  DF_LOG_ERR(" 1: i2c_write_reg(0x00, &CTRLA, 1)) failed");
	  return -EIO;
  }

  result = _writeReg(LTC2946_CTRLB_REG, &CTRLB, sizeof(CTRLB));

  if (result != 0) {
	  DF_LOG_ERR(" 1: i2c_write_reg(0x00, &CTRLA, 1)) failed");
	  return -EIO;
  }

  return 0;
}


int LTC2946::ltc2946_init()
{
	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.board_voltage_V = 0.0f;
	m_sensor_data.board_current_A = 0.0f;

	m_sensor_data.esc_voltage_V = 0.0f;
	m_sensor_data.esc_current_A = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

	m_synchronize.unlock();

	int result;

	uint8_t CTRLA = LTC2946_CHANNEL_CONFIG_V_C_3|LTC2946_SENSE_PLUS|LTC2946_OFFSET_CAL_EVERY|LTC2946_ADIN_GND;  //! Set Control A register to default value.

/*
	if (configure()!=0)
	{
		DF_LOG_ERR("ltc2946: configure failed!");
		return -1;
	}
*/

	//  DF_LOG_ERR("...LTC2946_CTRLA_REG: CTRLA= %x ", CTRLA);

	  result = _writeReg(LTC2946_CTRLA_REG, &CTRLA, sizeof(CTRLA));

	  if (result != 0) {
		  DF_LOG_ERR(" 1: i2c_write_reg(0x00, &CTRLA, 1)) failed");
		  return -EIO;
	  }

	  uint8_t CTRLB = LTC2946_DISABLE_ALERT_CLEAR&LTC2946_DISABLE_SHUTDOWN&LTC2946_DISABLE_CLEARED_ON_READ&LTC2946_DISABLE_STUCK_BUS_RECOVER&LTC2946_ENABLE_ACC&LTC2946_DISABLE_AUTO_RESET; //! Set Control B Register to default

	  result = _writeReg(LTC2946_CTRLB_REG, &CTRLB, sizeof(CTRLB));

	  if (result != 0) {
		  DF_LOG_ERR(" 2: i2c_write_reg(0x01, &CTRLB, 1)) failed");
		  return -EIO;
	  }

	DF_LOG_INFO("ltc2946: initialization successful!");

	usleep(1000);
	return 0;
}


int LTC2946::start()
{
//	DF_LOG_ERR("--------------------  LTC2946::start()  --------------------");
	int result;

	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error:LTC2946 Unable to open the device path: %s", m_dev_path);
		goto exit;
	}

	DF_LOG_ERR("LTC2946: Open the device path: %s", m_dev_path);

	/* Configure the I2C bus parameters for the LTC2946 sensor. */
	result = _setSlaveConfig(LTC2946_I2C_ADDRESS, LTC2946_BUS_FREQUENCY_IN_KHZ, LTC2946_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	/* Initialize the sensor for active and continuous operation. */
	result = ltc2946_init();

	if (result != 0) {
		DF_LOG_ERR("error: sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

exit:
/*
	if (result != 0) {
		devClose();
	}
*/
/*
	if (result != 0) {
		DF_LOG_ERR("error: Failed to start ISL");
		I2CDevObj::stop();
	}
*/
	return result;
}


int LTC2946::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}
	return 0;
}


void LTC2946::_measure(void)
{
	/* Read the data from the LTC2946 sensor. */

	int result;

	m_id.dev_id_s.address = LTC2946_I2C_ADDRESS_5V;
	// Set LTC2946 I2C address to Board measure
	result = _setSlaveConfig(LTC2946_I2C_ADDRESS_5V, LTC2946_BUS_FREQUENCY_IN_KHZ, LTC2946_TRANSFER_TIMEOUT_IN_USECS);
/*
	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}
*/

	m_sensor_data.read_counter++;

//	DF_LOG_ERR("LTC2946::_measure(void)");

	// Read raw voltage measurement from 0x1E register (2 bytes)
	uint8_t vraw[2];

	result = _readReg(LTC2946_VIN_MSB_REG, vraw, sizeof(vraw)); // 0x1E

	if (result < 0) {
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();

		return;
	}

	uint8_t iraw[2];

	result = _readReg(LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14

	if (result < 0) {
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();
		return;
	}

/*
	if (LTC2946::i2c_read_reg(LTC2946_VIN_MSB_REG,vraw,2)) return;
	// Read raw current measurement from 0x14 register (2 bytes)
	uint8_t iraw[2];
	if (LTC2946::i2c_read_reg(LTC2946_DELTA_SENSE_MSB_REG,iraw,2)) return;
*/

	uint16_t board_volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];  //MSB first
	board_volt16 >>= 4;                                     //data is 12 bit and left-aligned
//	float v_now = volt16/4095.0 * 102.4;                //102.4V is maximum voltage on this input

	uint16_t board_curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];  //MSB first
	board_curr16 >>= 4;                                     //data is 12 bit and left-aligned


//	float r_sense = R_SENSE_APM_5V;
//	float i_now = curr16/4095.0 * 0.1024 / r_sense;     //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

	/* Measure V.BAT */
	// Set LTC2946 I2C address to V.BAT measure


	m_id.dev_id_s.address = LTC2946_I2C_ADDRESS_VBATT;
	result = _setSlaveConfig(LTC2946_I2C_ADDRESS_VBATT, LTC2946_BUS_FREQUENCY_IN_KHZ, LTC2946_TRANSFER_TIMEOUT_IN_USECS);

	result = _readReg(LTC2946_VIN_MSB_REG, vraw, sizeof(vraw)); // 0x1E

	if (result < 0) {
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();

		vraw[0] = 0x00;
		vraw[1] = 0x00;
		//return;
	}

	result = _readReg(LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14

	if (result < 0) {
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();

		iraw[0] = 0x00;
		iraw[1] = 0x00;
		//return;
	}

	uint16_t esc_volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];  //MSB first
	esc_volt16 >>= 4;                                     //data is 12 bit and left-aligned
//	float esc_v_now = volt16/4095.0 * 102.4;                //102.4V is maximum voltage on this input

	uint16_t esc_curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];  //MSB first
	esc_curr16 >>= 4;                                     //data is 12 bit and left-aligned

	m_synchronize.lock();


	m_sensor_data.board_voltage_V = board_volt16;
	m_sensor_data.board_current_A = board_curr16;

	m_sensor_data.esc_voltage_V = esc_volt16;
	m_sensor_data.esc_current_A = esc_curr16;


//	r_sense = R_SENSE_APM_VBATT;
//	float esc_i_now = curr16/4095.0 * 0.1024 / r_sense;     //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

/*
	m_synchronize.lock();

	if (m_sensor_data.read_counter == 1) {
		m_sensor_data.board_voltage = v_now;       //if first time, initialize voltage to value now
		m_sensor_data.board_current = i_now;
		m_sensor_data.esc_voltage = esc_v_now;     //if first time, initialize voltage to value now
		m_sensor_data.esc_current = esc_i_now;
	}

	m_sensor_data.board_voltage = m_sensor_data.board_voltage * 0.01 + esc_v_now * 0.99;         //filter voltage
	m_sensor_data.board_current = m_sensor_data.board_current * 0.01 + esc_i_now * 0.99;         //filter current

	m_sensor_data.esc_voltage = m_sensor_data.esc_voltage * 0.01 + v_now * 0.99;                 //filter voltage
	m_sensor_data.esc_current = m_sensor_data.esc_current * 0.01 + i_now * 0.99;                 //filter current


	if (m_sensor_data.read_counter % 250 == 0)
	{
		DF_LOG_ERR("...............LTC2946::_measure voltage = %f, current %f , %d",
				m_sensor_data.board_voltage,
				m_sensor_data.board_current,
				m_sensor_data.read_counter );

		DF_LOG_ERR("...............LTC2946::    _measure ESC = %f, current %f , %d",
				m_sensor_data.esc_voltage,
				m_sensor_data.esc_current,
				m_sensor_data.read_counter );

	}
*/
	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();

}


int LTC2946::_publish(struct ltc2946_sensor_data &data)
{
	// TBD
	return -1;
}


void LTC2946::printValues(struct ltc2946_sensor_data &data)
{
	DF_LOG_INFO("Voltage = %f V, current %f A", data.board_voltage_V, data.board_current_A);
}

/*
int LTC2946::getSensorData(DevHandle &h, struct ltc2946_sensor_data &out_data,
		bool is_new_data_required)
{
	LTC2946 *me = DevMgr::getDevObjByHandle<LTC2946>(h);
	int ret = -1;

	if (me != nullptr) {
		me->m_synchronize.lock();

		if (is_new_data_required) {
			me->m_synchronize.waitOnSignal(0);
		}

		out_data = me->m_sensor_data;
		me->m_synchronize.unlock();
		ret = 0;
	}

	return ret;
}
*/
