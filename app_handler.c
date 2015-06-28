/*
 * Copyright (c) 2012-2014, hiro99ma
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *         this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *         this list of conditions and the following disclaimer
 *         in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

/**************************************************************************
 * include
 **************************************************************************/

#include <string.h>

#include "boards.h"
#include "app_handler.h"
#include "app_ble.h"
#include "drivers.h"

#include "app_error.h"
#include "app_trace.h"


/**************************************************************************
 * macro
 **************************************************************************/

/**************************************************************************
 * declaration
 **************************************************************************/

/**************************************************************************
 * prototype
 **************************************************************************/

/**************************************************************************
 * public function
 **************************************************************************/

/**@brief エラーハンドラ
 *
 * APP_ERROR_HANDLER()やAPP_ERROR_CHECK()から呼び出される(app_error.h)。
 * 引数は見ず、ASSERT LEDを点灯させて処理を止める。
 *
 * @param[in] error_code  エラーコード
 * @param[in] line_num    エラー発生行(__LINE__など)
 * @param[in] p_file_name エラー発生ファイル(__FILE__など)
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    //ASSERT LED点灯
    led_on(LED_PIN_NO_ASSERT);

#if 0
    //リセットによる再起動
    NVIC_SystemReset();
#else
    //留まる
    while(1) {
        __WFI();
    }
#endif
}


/**@brief システムイベント発生
 *
 * SoCでイベントが発生した場合にコールバックされる。
 *
 *
 * @param[in]   sys_evt   enum NRF_SOC_EVTS型(NRF_EVT_xxx). nrf_soc.hに定義がある.
 *      - NRF_EVT_HFCLKSTARTED
 *      - NRF_EVT_POWER_FAILURE_WARNING
 *      - NRF_EVT_FLASH_OPERATION_SUCCESS
 *      - NRF_EVT_FLASH_OPERATION_ERROR
 *      - NRF_EVT_RADIO_BLOCKED
 *      - NRF_EVT_RADIO_CANCELED
 *      - NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN
 *      - NRF_EVT_RADIO_SESSION_IDLE
 *      - NRF_EVT_RADIO_SESSION_CLOSED
 */
void main_sys_evt_dispatch(uint32_t sys_evt)
{
	/* 上位のハンドラを呼ぶようにしておくか？ */
	/* とりあえず放置しておく */
}


/**************************************************************************
 * private function
 **************************************************************************/

