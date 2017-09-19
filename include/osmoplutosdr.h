/*
* libosmoplutosdr
* Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
* Copyright (C) 2017 by Hoernchen <la@tfc-server.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LIBOSMOPLUTOSDR_H
#define LIBOSMOPLUTOSDR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "osmoplutosdr_export.h"
#include <stdint.h>

typedef struct plutosdr_dev plutosdr_dev_t;
typedef void (*plutosdr_read_async_cb_t)(unsigned char* buf, int32_t len, void* ctx);

PLUTOSDR_API uint32_t plutosdr_get_device_count(void);
PLUTOSDR_API int plutosdr_open(plutosdr_dev_t** dev, uint32_t index);
PLUTOSDR_API int plutosdr_close(plutosdr_dev_t* dev);
PLUTOSDR_API int plutosdr_cancel_async(plutosdr_dev_t* dev);
PLUTOSDR_API int plutosdr_read_async(plutosdr_dev_t* dev, plutosdr_read_async_cb_t cb, void* ctx, uint32_t buf_num, uint32_t buf_len);
PLUTOSDR_API int plutosdr_wait_async(plutosdr_dev_t* dev, plutosdr_read_async_cb_t cb, void* ctx);

PLUTOSDR_API void plutosdr_bufstream_enable(plutosdr_dev_t* dev, uint32_t enable);
PLUTOSDR_API void plutosdr_set_rfbw(plutosdr_dev_t* dev, uint32_t rfbw_hz);
PLUTOSDR_API void plutosdr_set_sample_rate(plutosdr_dev_t* dev, uint32_t sampfreq_hz);
PLUTOSDR_API void plutosdr_set_rxlo(plutosdr_dev_t* dev, uint64_t rfbw_hz);
PLUTOSDR_API void plutosdr_set_gainctl_manual(plutosdr_dev_t* dev);
PLUTOSDR_API void plutosdr_set_gain_mdb(plutosdr_dev_t* dev, int32_t gain_in_millib);

#ifdef __cplusplus
}
#endif

#endif // LIBOSMOPLUTOSDR_H
