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
#include "boards.h"
#include "drivers.h"
#include "app_ble.h"
#include "main.h"

#include "app_error.h"

#include "softdevice_handler_appsh.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"

#include "app_trace.h"


/**************************************************************************
 * macro
 **************************************************************************/
#define ARRAY_SIZE(array)               (sizeof(array) / sizeof(array[0]))

/** Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEAD_BEEF                       (0xDEADBEEF)

/*
 * Timer
 */
/** Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_PRESCALER             (0)

/** BLEが使用するタイマ数(BLEを使うなら1、使わないなら0) */
#define APP_TIMER_NUM_BLE               (1)

/** ユーザアプリで使用するタイマ数 */
#define APP_TIMER_NUM_USERAPP           (0)

/** 同時に生成する最大タイマ数 */
#define APP_TIMER_MAX_TIMERS            (APP_TIMER_NUM_BLE+APP_TIMER_NUM_USERAPP)

/** Size of timer operation queues. */
#define APP_TIMER_OP_QUEUE_SIZE         (4)

/*
 * Scheduler
 */
// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */

/** Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE                (10)



/**************************************************************************
 * declaration
 **************************************************************************/

/**************************************************************************
 * prototype
 **************************************************************************/

static void gpio_init(void);
static void timers_init(void);
//static void timers_start(void);
static void scheduler_init(void);
static void softdevice_init(void);

static void sys_evt_dispatch(uint32_t sys_evt);


/**************************************************************************
 * public function
 **************************************************************************/

void drv_init(void)
{
    gpio_init();
    timers_init();      //app_button_init()やble_conn_params_init()よりも前に呼ぶこと!
                        //呼ばなかったら、NRF_ERROR_INVALID_STATE(8)が発生する。
    scheduler_init();
    softdevice_init();
}


void drv_event_exec(void)
{
    uint32_t err_code;

    //スケジュール済みイベントの実行(mainloop内で呼び出す)
    app_sched_execute();
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**********************************************
 * LED
 **********************************************/

/**
 * @brief LED点灯
 *
 * @param[in]   pin     対象PIN番号
 */
void led_on(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_clear(pin);
}


/**
 * @brief LED消灯
 *
 * @param[in]   pin     対象PIN番号
 */
void led_off(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_set(pin);
}


/**************************************************************************
 * private function
 **************************************************************************/

/**********************************************
 * GPIO
 **********************************************/

/**
 * @brief GPIO初期化
 */
static void gpio_init(void)
{
	/* output */
    nrf_gpio_cfg_output(LED_PIN_NO_ADVERTISING);
    nrf_gpio_cfg_output(LED_PIN_NO_CONNECTED);
    nrf_gpio_cfg_output(LED_PIN_NO_ASSERT);

	/* 初期値 */
	nrf_gpio_pin_set(LED_PIN_NO_ADVERTISING);
	nrf_gpio_pin_set(LED_PIN_NO_CONNECTED);
	nrf_gpio_pin_set(LED_PIN_NO_ASSERT);
}


/**********************************************
 * タイマ
 **********************************************/

/**
 * @brief タイマ初期化
 *
 * タイマ機能を初期化する。
 * 調べた範囲では、以下の機能を使用する場合には、その初期化よりも前に実行しておく必要がある。
 *  - ボタン
 *  - BLE
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(0, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
//    APP_TIMER_INIT(0, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

#if 0
    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
     */
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
}

#if 0
/**
 * @brief タイマ開始
 */
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer. */
    uint32_t err_code;

    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
#endif


/**********************************************
 * Scheduler
 **********************************************/

/**
 * @brief スケジューラ初期化
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**********************************************
 * BLE : Advertising
 **********************************************/

/**
 * @brief BLEスタック初期化
 *
 * @detail BLE関連の初期化を行う。
 *      -# SoftDeviceハンドラ初期化
 *      -# システムイベントハンドラ初期化
 */
static void softdevice_init(void)
{
    uint32_t err_code;

    /*
     * SoftDeviceの初期化
     *      スケジューラの使用：あり
     */
//    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, true);
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, NULL);

    /* システムイベントハンドラの設定 */
    err_code = softdevice_sys_evt_handler_set(main_sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);


    /* BLEイベントハンドラの設定 */
    {
        err_code = softdevice_ble_evt_handler_set(app_ble_evt_dispatch);
        APP_ERROR_CHECK(err_code);
    }
}

