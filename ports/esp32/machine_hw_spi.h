#ifndef MACHINE_HW_SPI_H_
#define MACHINE_HW_SPI_H_

#include "py/obj.h"
#include "driver/spi_master.h"

#define MP_HW_SPI_MAX_XFER_BYTES (4092)
#define MP_HW_SPI_MAX_XFER_BITS (MP_HW_SPI_MAX_XFER_BYTES * 8) // Has to be an even multiple of 8

const spi_device_handle_t spi_device_handle_from_mp_obj(mp_obj_t o);

#endif /* MACHINE_HW_SPI_H_ */