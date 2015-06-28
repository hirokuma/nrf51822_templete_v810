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
#include "nrf_soc.h"

#include "boards.h"
#include "app_ble.h"
#include "drivers.h"
#include "app_handler.h"

#include "app_error.h"
#include "app_timer.h"

#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advertising.h"

#include "ble_ios.h"

#include "app_trace.h"


/**************************************************************************
 * macro
 **************************************************************************/
#define ARRAY_SIZE(array)               (sizeof(array) / sizeof(array[0]))

/** Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEAD_BEEF                       (0xDEADBEEF)


/**
 * Include or not the service_changed characteristic.
 * if not enabled, the server's database cannot be changed for the lifetime of the device
 */
#define IS_SRVC_CHANGED_CHARACT_PRESENT (0)


/*
 * デバイス名
 *   UTF-8かつ、\0を含まずに20文字以内(20byte?)
 */
#define GAP_DEVICE_NAME                 "BLE_TEMPLATE"
//                                      "12345678901234567890"


/*
 * Appearance設定
 *  コメントアウト時はAppearance無しにするが、おそらくSoftDeviceがUnknownにしてくれる。
 *
 * Bluetooth Core Specification Supplement, Part A, Section 1.12
 * Bluetooth Core Specification 4.0 (Vol. 3), Part C, Section 12.2
 * https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
 * https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v7.x.x/doc/7.2.0/s110/html/a00837.html
 */

/*
 * BLE : Advertising
 */
/* Advertising間隔[msec単位] */
#define APP_ADV_INTERVAL                (1000)

/* Advertisingタイムアウト時間[sec単位] */
#define APP_ADV_TIMEOUT_IN_SECONDS      (60)

/* Advertising抑制CH */
//#define APP_ADV_DISABLE_CH37
//#define APP_ADV_DISABLE_CH38
//#define APP_ADV_DISABLE_CH39


#define GAP_USE_APPEARANCE		BLE_APPEARANCE_UNKNOWN

/*
 * Peripheral Preferred Connection Parameters(PPCP)
 *   パラメータの意味はCore_v4.1 p.2537 "4.5 CONNECTION STATE"を参照
 *
 * connInterval : Connectionイベントの送信間隔(7.5msec～4sec)
 * connSlaveLatency : SlaveがConnectionイベントを連続して無視できる回数
 * connSupervisionTimeout : Connection時に相手がいなくなったとみなす時間(100msec～32sec)
 *                          (1 + connSlaveLatency) * connInterval * 2以上
 *
 * note:
 *      connSupervisionTimeout * 8 >= connIntervalMax * (connSlaveLatency + 1)
 */
/* 最小時間[msec単位] */
#define CONN_MIN_INTERVAL               (500)

/* 最大時間[msec単位] */
#define CONN_MAX_INTERVAL               (1000)

/* slave latency */
#define CONN_SLAVE_LATENCY              (0)

/* connSupervisionTimeout[msec単位] */
#define CONN_SUP_TIMEOUT                (4000)

/** sd_ble_gap_conn_param_update()を実行してから初回の接続イベントを通知するまでの時間[msec単位] */
/*
 * sd_ble_gap_conn_param_update()
 *   Central role時:
 *      Link Layer接続パラメータ更新手続きを初期化する。
 *
 *   Peripheral role時:
 *      L2CAPへ連絡要求(corresponding L2CAP request)を送信し、Centralからの手続きを待つ。
 *      接続パラメータとしてNULLが指定でき、そのときはPPCPキャラクタリスティックが使われる。
 *      ということは、Peripheralだったらsd_ble_gap_conn_param_update()は呼ばずに
 *      sd_ble_gap_ppcp_set()を呼ぶということでもよいということか？
 */
#define CONN_FIRST_PARAMS_UPDATE_DELAY  (5000)

/** sd_ble_gap_conn_param_update()を呼び出す間隔[msec単位] */
#define CONN_NEXT_PARAMS_UPDATE_DELAY   (30000)

/** Connectionパラメータ交換を諦めるまでの試行回数 */
#define CONN_MAX_PARAMS_UPDATE_COUNT    (3)

/*
 * BLE : Security
 */
/** ペアリング要求からのタイムアウト時間[sec] */
#define SEC_PARAM_TIMEOUT               (30)

/** 1:Bondingあり 0:なし */
#define SEC_PARAM_BOND                  (0)

/** 1:ペアリング時の認証あり 0:なし */
#define SEC_PARAM_MITM                  (0)

/** IO能力
 * - BLE_GAP_IO_CAPS_DISPLAY_ONLY     : input=無し/output=画面
 * - BLE_GAP_IO_CAPS_DISPLAY_YESNO    : input=Yes/No程度/output=画面
 * - BLE_GAP_IO_CAPS_KEYBOARD_ONLY    : input=キーボード/output=無し
 * - BLE_GAP_IO_CAPS_NONE             : input/outputなし。あるいはMITM無し
 * - BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY : input=キーボード/output=画面
 */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE

/** 1:OOB認証有り 0:なし */
#define SEC_PARAM_OOB                   (0)

/** 符号化鍵サイズ:最小byte(7～) */
#define SEC_PARAM_MIN_KEY_SIZE          (7)

/** 符号化鍵サイズ:最大byte(min～16) */
#define SEC_PARAM_MAX_KEY_SIZE          (16)


//設定値のチェック
#if (APP_ADV_INTERVAL < 20)
#error connInterval(Advertising) too small.
#elif (APP_ADV_INTERVAL < 100)
#warning connInterval(Advertisin) is too small in non-connectable mode
#elif (10240 < APP_ADV_INTERVAL)
#error connInterval(Advertising) too large.
#endif  //APP_ADV_INTERVAL

#if (APP_ADV_TIMEOUT_IN_SECONDS == BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED)
//OK
#elif (BLE_GAP_ADV_TIMEOUT_LIMITED_MAX < APP_ADV_TIMEOUT_IN_SECONDS)
#warning Advertising Timeout is too large in limited discoverable mode
#endif  //APP_ADV_TIMEOUT_IN_SECONDS

#if (CONN_MIN_INTERVAL * 10 < 75)
#error connInterval_Min(Connection) too small.
#elif (4000 < CONN_MIN_INTERVAL)
#error connInterval_Min(Connection) too large.
#endif  //CONN_MIN_INTERVAL

#if (CONN_MAX_INTERVAL * 10 < 75)
#error connInterval_Max(Connection) too small.
#elif (4000 < CONN_MAX_INTERVAL)
#error connInterval_Max(Connection) too large.
#endif  //CONN_MAX_INTERVAL

#if (CONN_MAX_INTERVAL < CONN_MIN_INTERVAL)
#error connInterval_Max < connInterval_Min
#endif  //connInterval Max < Min

#if (BLE_GAP_CP_SLAVE_LATENCY_MAX < CONN_SLAVE_LATENCY)
#error connSlaveLatency too large.
#endif  //CONN_SLAVE_LATENCY

#if (CONN_SUP_TIMEOUT < 100)
#error connSupervisionTimeout too small.
#elif (32000 < CONN_SUP_TIMEOUT)
#error connSupervisionTimeout too large.
#endif  //CONN_SUP_TIMEOUT
#if (CONN_SUP_TIMEOUT <= (1 + CONN_SLAVE_LATENCY) * (CONN_MAX_INTERVAL * 2))
#error connSupervisionTimeout too small in manner 1.
#endif
#if (CONN_SUP_TIMEOUT * 4 <= CONN_MAX_INTERVAL * (CONN_SLAVE_LATENCY + 1))
#error connSupervisionTimeout too small in manner 2.
#endif


/**************************************************************************
 * declaration
 **************************************************************************/

/** Handle of the current connection. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;

#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                        m_dfus;     /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT

static ble_ios_t                        m_ios;


/**************************************************************************
 * prototype
 **************************************************************************/

static void ble_stack_init(void);

#ifdef BLE_DFU_APP_SUPPORT
static void dfu_reset_prepare(void)
#endif

static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);

static void ble_evt_handler(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);


static void svc_ios_handler_in(ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length);
static void svc_ios_handler_out(ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length);


/**************************************************************************
 * public function
 **************************************************************************/

void app_ble_init(void)
{
	ble_stack_init();
}


/**
 * @brief Advertising開始
 */
void app_ble_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = MSEC_TO_UNITS(APP_ADV_INTERVAL, UNIT_0_625_MS);
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
#ifdef APP_ADV_DISABLE_CH37
	adv_params.channel_mask.ch_37_off = 1;
#endif	//APP_ADV_DISABLE_CH37
#ifdef APP_ADV_DISABLE_CH38
	adv_params.channel_mask.ch_38_off = 1;
#endif	//APP_ADV_DISABLE_CH38
#ifdef APP_ADV_DISABLE_CH39
	adv_params.channel_mask.ch_39_off = 1;
#endif	//APP_ADV_DISABLE_CH39

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    led_on(LED_PIN_NO_ADVERTISING);

    app_trace_log("advertising start\r\n");
}


#ifdef BLE_DFU_APP_SUPPORT
void app_ble_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
    led_off(LED_PIN_NO_ADVERTISING);

    app_trace_log("advertising stop\r\n");
}
#endif // BLE_DFU_APP_SUPPORT

int app_ble_is_connected(void)
{
	return m_conn_handle != BLE_CONN_HANDLE_INVALID;
}

void app_ble_nofify(const uint8_t *p_data, uint16_t length)
{
	//ble_ios_notify(&m_ios, p_data, length);
}


/**
 * @brief BLEイベントハンドラ
 *
 * @details BLEスタックイベント受信後、メインループのスケジューラから呼ばれる。
 *
 * @param[in]   p_ble_evt   BLEスタックイベント
 */
void app_ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);

    //I/O Service
    ble_ios_on_ble_evt(&m_ios, p_ble_evt);

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
}


/**************************************************************************
 * private function
 **************************************************************************/


/**********************************************
 * BLE : Advertising
 **********************************************/

/**
 * @brief BLEスタック初期化
 *
 * @detail BLE関連の初期化を行う。
 *      -# SoftDeviceハンドラ初期化
 *      -# システムイベントハンドラ初期化
 *      -# BLEスタック有効化
 *      -# BLEイベントハンドラ設定
 *      -# デバイス名設定
 *      -# Appearance設定(GAP_USE_APPEARANCE定義時)
 *      -# PPCP設定
 *      -# Service初期化
 *      -# Advertising初期化
 *      -# Connection初期化
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    /* BLEスタックの有効化 */
    {
        ble_enable_params_t ble_enable_params;

        memset(&ble_enable_params, 0, sizeof(ble_enable_params));
        ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
        err_code = sd_ble_enable(&ble_enable_params);
        APP_ERROR_CHECK(err_code);
    }

    /* デバイス名設定 */
    {
        //デバイス名へのWrite Permission(no protection, open link)
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                            (const uint8_t *)GAP_DEVICE_NAME,
                                            strlen(GAP_DEVICE_NAME));
        APP_ERROR_CHECK(err_code);
    }

#ifdef GAP_USE_APPEARANCE
    /* Appearance設定 */
    err_code = sd_ble_gap_appearance_set(GAP_USE_APPEARANCE);
    APP_ERROR_CHECK(err_code);
#endif  //GAP_USE_APPEARANCE

    /*
     * Peripheral Preferred Connection Parameters(PPCP)
     * ここで設定しておくと、Connection Parameter Update Reqを送信せずに済むらしい。
     */
    {
        ble_gap_conn_params_t   gap_conn_params = {0};

        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(CONN_MIN_INTERVAL, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(CONN_MAX_INTERVAL, UNIT_1_25_MS);
        gap_conn_params.slave_latency     = CONN_SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT, UNIT_10_MS);

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);
    }

    /*
     * Service初期化
     */
    {
        ble_ios_init_t ios_init;

        ios_init.evt_handler_in = svc_ios_handler_in;
        //ios_init.evt_handler_out = svc_ios_handler_out;
        ios_init.len_in = 64;
        ios_init.len_out = 32;
        ble_ios_init(&m_ios, &ios_init);
    }

    /*
     * Advertising初期化
     */
    {
        ble_uuid_t adv_uuids[] = { { IOS_UUID_SERVICE, m_ios.uuid_type } };
        ble_advdata_t advdata;
        ble_advdata_t scanrsp;

        memset(&advdata, 0, sizeof(advdata));
        memset(&scanrsp, 0, sizeof(scanrsp));

        /*
         * ble_advdata_name_type_t (ble_advdata.h)
         *
         * BLE_ADVDATA_NO_NAME    : デバイス名無し
         * BLE_ADVDATA_SHORT_NAME : デバイス名あり «Shortened Local Name»
         * BLE_ADVDATA_FULL_NAME  : デバイス名あり «Complete Local Name»
         *
         * https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
         * https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v7.x.x/doc/7.2.0/s110/html/a01015.html#ga03c5ccf232779001be9786021b1a563b
         */
        advdata.name_type = BLE_ADVDATA_FULL_NAME;

        /*
         * Appearanceが含まれるかどうか
         */
#ifdef GAP_USE_APPEARANCE
        advdata.include_appearance = true;
#else   //GAP_USE_APPEARANCE
        advdata.include_appearance = false;
#endif  //GAP_USE_APPEARANCE
        /*
         * Advertisingフラグの設定
         * CSS_v4 : Part A  1.3 FLAGS
         * https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v7.x.x/doc/7.2.0/s110/html/a00802.html
         *
         * BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE = BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE | BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
         *      BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE : LE Limited Discoverable Mode
         *      BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED : BR/EDR not supported
         */
        advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;    //探索時間に制限あり

        /* SCAN_RSPデータ設定 */
        scanrsp.uuids_complete.uuid_cnt = ARRAY_SIZE(adv_uuids);
        scanrsp.uuids_complete.p_uuids  = adv_uuids;

        err_code = ble_advdata_set(&advdata, &scanrsp);
        APP_ERROR_CHECK(err_code);
    }

    /*
     * Connection初期化
     */
    {
        ble_conn_params_init_t cp_init = {0};

		/* APP_TIMER_PRESCALER = 0 */
        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(CONN_FIRST_PARAMS_UPDATE_DELAY, 0);
        cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(CONN_NEXT_PARAMS_UPDATE_DELAY, 0);
        cp_init.max_conn_params_update_count   = CONN_MAX_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = conn_params_evt_handler;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
    }

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    {
        ble_dfu_init_t   dfus_init;

        // Initialize the Device Firmware Update Service.
        memset(&dfus_init, 0, sizeof(dfus_init));

        dfus_init.evt_handler    = dfu_app_on_dfu_evt;
        dfus_init.error_handler  = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
        dfus_init.evt_handler    = dfu_app_on_dfu_evt;
        dfus_init.revision       = DFU_REVISION;

        err_code = ble_dfu_init(&m_dfus, &dfus_init);
        APP_ERROR_CHECK(err_code);

        dfu_app_reset_prepare_set(dfu_reset_prepare);
    }
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}


/** @snippet [DFU BLE Reset prepare] */
#ifdef BLE_DFU_APP_SUPPORT
static void dfu_reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else {
        // If not connected, then the device will be advertising. Hence stop the advertising.
        ble_advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT


/**********************************************
 * BLE : Connection
 **********************************************/

/**
 * @brief Connectionパラメータモジュールイベントハンドラ
 *
 * @details Connectionパラメータモジュールでアプリに通知するイベントが発生した場合に呼ばれる。
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Connectionパラメータモジュールから受信したイベント
 */
static void conn_params_evt_handler(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**
 * @brief Connectionパラメータエラーハンドラ
 *
 * @param[in]   nrf_error   エラーコード
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**********************************************
 * BLE stack
 **********************************************/

/**
 * @brief BLEスタックイベントハンドラ
 *
 * @param[in]   p_ble_evt   BLEスタックイベント
 */
static void ble_evt_handler(ble_evt_t *p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
	bool                             master_id_matches;
	ble_gap_sec_kdist_t *            p_distributed_keys;
    ble_gap_enc_info_t               *p_enc_info;
	ble_gap_irk_t *                  p_id_info;
	ble_gap_sign_info_t *            p_sign_info;

    static ble_gap_enc_key_t         m_enc_key;           /**< Encryption Key (Encryption Info and Master ID). */
    static ble_gap_id_key_t          m_id_key;            /**< Identity Key (IRK and address). */
    static ble_gap_sign_info_t       m_sign_key;          /**< Signing Key (Connection Signature Resolving Key). */
    static ble_gap_sec_keyset_t      sec_key = {.keys_periph = {&m_enc_key, &m_id_key, &m_sign_key}};


    switch (p_ble_evt->header.evt_id) {
    /*************
     * GAP event
     *************/

    //接続が成立したとき
    case BLE_GAP_EVT_CONNECTED:
        app_trace_log("BLE_GAP_EVT_CONNECTED\r\n");
        led_on(LED_PIN_NO_CONNECTED);
        led_off(LED_PIN_NO_ADVERTISING);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    //相手から切断されたとき
    //必要があればsd_ble_gatts_sys_attr_get()でSystem Attributeを取得し、保持しておく。
    //保持したSystem Attributeは、EVT_SYS_ATTR_MISSINGで返すことになる。
    case BLE_GAP_EVT_DISCONNECTED:
        app_trace_log("BLE_GAP_EVT_DISCONNECTED\r\n");
        led_off(LED_PIN_NO_CONNECTED);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        app_ble_start();
        break;

    //SMP Paring要求を受信したとき
    //sd_ble_gap_sec_params_reply()で値を返したあと、SMP Paring Phase 2に状態遷移する
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        app_trace_log("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
        {
            ble_gap_sec_params_t sec_param;
            sec_param.bond         = SEC_PARAM_BOND;
            sec_param.mitm         = SEC_PARAM_MITM;
            sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
            sec_param.oob          = SEC_PARAM_OOB;
            sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
            sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                               BLE_GAP_SEC_STATUS_SUCCESS,
                                               &sec_param,
                                               &sec_key);
            APP_ERROR_CHECK(err_code);
        }
        break;

    //Just Works(Bonding有り)の場合、SMP Paring Phase 3のあとでPeripheral Keyが渡される。
    //ここではPeripheral Keyを保存だけしておき、次のBLE_GAP_EVT_SEC_INFO_REQUESTで処理する。
    case BLE_GAP_EVT_AUTH_STATUS:
        app_trace_log("BLE_GAP_EVT_AUTH_STATUS\r\n");
        m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
        break;

    //SMP Paringが終わったとき？
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        app_trace_log("BLE_GAP_EVT_SEC_INFO_REQUEST\r\n");
		master_id_matches  = memcmp(&p_ble_evt->evt.gap_evt.params.sec_info_request.master_id,
		                            &m_enc_key.master_id,
		                            sizeof(ble_gap_master_id_t)) == 0;
		p_distributed_keys = &m_auth_status.kdist_periph;

		p_enc_info  = (p_distributed_keys->enc  && master_id_matches) ? &m_enc_key.enc_info : NULL;
		p_id_info   = (p_distributed_keys->id   && master_id_matches) ? &m_id_key.id_info   : NULL;
		p_sign_info = (p_distributed_keys->sign && master_id_matches) ? &m_sign_key         : NULL;
		err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, p_id_info, p_sign_info);
        APP_ERROR_CHECK(err_code);
        break;

    //Advertisingか認証のタイムアウト発生
    case BLE_GAP_EVT_TIMEOUT:
        app_trace_log("BLE_GAP_EVT_TIMEOUT\r\n");
        switch (p_ble_evt->evt.gap_evt.params.timeout.src) {
        case BLE_GAP_TIMEOUT_SRC_ADVERTISING: //Advertisingのタイムアウト
            /* Advertising LEDを消灯 */
            led_off(LED_PIN_NO_ADVERTISING);

            /* System-OFFにする(もう戻ってこない) */
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_TIMEOUT_SRC_SECURITY_REQUEST:  //Security requestのタイムアウト
            break;

        default:
            break;
        }
        break;

    /*********************
     * GATT Server event
     *********************/

    //接続後、Bondingした相手からSystem Attribute要求を受信したとき
    //System Attributeは、EVT_DISCONNECTEDで保持するが、今回は保持しないのでNULLを返す。
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        app_trace_log("BLE_GATTS_EVT_SYS_ATTR_MISSING\r\n");
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0,
        			BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}


/**********************************************
 * BLE : Services
 **********************************************/

/**
 * @brief I/Oサービスイベントハンドラ
 *
 * @param[in]   p_ios   I/Oサービス構造体
 * @param[in]   p_value 受信バッファ
 * @param[in]   length  受信データ長
 */
static void svc_ios_handler_in(ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length)
{
    app_trace_log("svc_ios_handler_in\r\n");
}

/**
 * @brief I/Oサービスイベントハンドラ
 *
 * @param[in]   p_ios   I/Oサービス構造体
 * @param[in]   p_value 受信バッファ
 * @param[in]   length  受信データ長
 */
static void svc_ios_handler_out(ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length)
{
    app_trace_log("svc_ios_handler_out\r\n");
}

