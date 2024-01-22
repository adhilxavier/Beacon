
/**

* Copyright (c) 2012 - 2021, Nordic Semiconductor ASA

*

* All rights reserved.

*

* Redistribution and use in source and binary forms, with or without modification,

* are permitted provided that the following conditions are met:

*

* 1. Redistributions of source code must retain the above copyright notice, this

*    list of conditions and the following disclaimer.

*

* 2. Redistributions in binary form, except as embedded into a Nordic

*    Semiconductor ASA integrated circuit in a product or a software update for

*    such product, must reproduce the above copyright notice, this list of

*    conditions and the following disclaimer in the documentation and/or other

*    materials provided with the distribution.

*

* 3. Neither the name of Nordic Semiconductor ASA nor the names of its

*    contributors may be used to endorse or promote products derived from this

*    software without specific prior written permission.

*

* 4. This software, with or without modification, must only be used with a

*    Nordic Semiconductor ASA integrated circuit.

*

* 5. Any software provided in binary form under this license must not be reverse

*    engineered, decompiled, modified and/or disassembled.

*

* THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS

* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES

* OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE

* DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE

* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR

* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE

* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)

* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT

* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT

* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*

*/

/* Attention!

* To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile

* qualification listings, this section of source code must not be modified.

*/

#include "sdk_common.h"

#include "UartService.h"

#include "app_error.h"

#include <string.h>

#include "ble_srv_common.h"

#include "nrf_ble_qwr.h"
 
 
//static nrf_ble_qwr_t m_qwr;

NRF_BLE_QWR_DEF(m_qwr);

/**@brief Function for handling the Connect event.

*

* @param[in]   p_hrs       Heart Rate Service structure.

* @param[in]   p_ble_evt   Event received from the BLE stack.

*/

static void OnConnect(_sUartService * psUartService, ble_evt_t const * p_ble_evt)

{

    uint32_t err_code;
 
    psUartService->usConnHdl = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, p_ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

}
 
 
/**@brief Function for handling the Disconnect event.
*
*
*
* @param[in]   p_hrs       Heart Rate Service structure.
*
* @param[in]   p_ble_evt   Event received from the BLE stack.
*
*/

static void OnDisconnect(_sUartService * psUartService, ble_evt_t const * p_ble_evt)

{

    UNUSED_PARAMETER(p_ble_evt);

    psUartService->usConnHdl = BLE_CONN_HANDLE_INVALID;

}
 
 
/**@brief Function for handling write events to the Heart Rate Measurement characteristic.
*
*
*
* @param[in]   p_hrs         Heart Rate Service structure.
*
* @param[in]   p_evt_write   Write event received from the BLE stack.
*
*/

static void OnTxCharaCCCDWrite(_sUartService * psUartService, ble_gatts_evt_write_t const * p_evt_write)

{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (psUartService->ServiceEvtHandler != NULL)
        {
            _sUartServiceEvtType sEvtType;
 
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                sEvtType.eEvtType = UART_SVC_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                sEvtType.eEvtType = UART_SVC_EVT_NOTIFICATION_DISABLED;
            }

            psUartService->ServiceEvtHandler(psUartService, &sEvtType);
        }
    }
}
 
 
/**@brief Function for handling the Write event.

*

* @param[in]   p_hrs       Heart Rate Service structure.

* @param[in]   p_ble_evt   Event received from the BLE stack.

*/

static void OnWrite(_sUartService * psUartService, ble_evt_t const * p_ble_evt)

{

    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
 
    if (p_evt_write->handle == psUartService->sTxHandle.cccd_handle)

    {

        OnTxCharaCCCDWrite(psUartService, p_evt_write);

    }

}
 
 
void UartServiceOnBleEvt(ble_evt_t const * p_ble_evt, void * p_context)

{

    uint32_t err_code;

    _sUartService *psUartService = (_sUartService *) p_context;

    _sUartServiceEvtType SvcEvt =  {0};

    ble_gap_conn_params_t   gap_conn_params;
 
    SvcEvt.eEvtType = UART_SVC_EVT_NONE;

    switch (p_ble_evt->header.evt_id)

    {

        case BLE_GAP_EVT_CONNECTED:

            OnConnect(psUartService, p_ble_evt);

            SvcEvt.eEvtType = UART_SVC_EVT_CONNECTED;

            psUartService->ServiceEvtHandler(psUartService, &SvcEvt);

            break;
 
        case BLE_GAP_EVT_DISCONNECTED:

            OnDisconnect(psUartService, p_ble_evt);

            SvcEvt.eEvtType = UART_SVC_EVT_DISCONNECTED;

            psUartService->ServiceEvtHandler(psUartService, &SvcEvt);

            break;
 
        case BLE_GATTS_EVT_WRITE:

            OnWrite(psUartService, p_ble_evt);
 
 
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:

        {

           // NRF_LOG_DEBUG("PHY update request.");

            ble_gap_phys_t const phys =

            {

                .rx_phys = BLE_GAP_PHY_AUTO,

                .tx_phys = BLE_GAP_PHY_AUTO,

            };

            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);

            APP_ERROR_CHECK(err_code);

        } break;            
 
        case BLE_GATTS_EVT_TIMEOUT:

            // Disconnect on GATT Server timeout event.

            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,

                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        default:

            // No implementation needed.

            break;

    }

}
 
uint32_t UartServiceInit(_sUartService * psUartService, const _sUartServiceInit *psUartServiceInit)
{

    uint32_t              err_code;

    ble_uuid_t            ble_uid;

    uint8_t               ucUuidType;

    ble_add_char_params_t add_char_params;

    nrf_ble_qwr_init_t qwr_init = {0};
 
    // Initialize service structure

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);

    APP_ERROR_CHECK(err_code);
 
    psUartService->ServiceEvtHandler  = psUartServiceInit->ServiceEvtHandler;

    psUartService->usConnHdl          = BLE_CONN_HANDLE_INVALID;

    // Add service

    ucUuidType =     BLE_UUID_TYPE_VENDOR_BEGIN;

    ble_uuid128_t base_uuid = {UART_SERVICE_UUID};

 
    err_code = sd_ble_uuid_vs_add(&base_uuid, &ucUuidType);

    VERIFY_SUCCESS(err_code);
 
    ble_uid.uuid = SERVICE_UUID;

    ble_uid.type = BLE_UUID_TYPE_BLE;
 
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,

                                        &ble_uid,

                                        &psUartService->usServiceHdl);
 
    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }
 
    // Add heart rate measurement characteristic

    memset(&add_char_params, 0, sizeof(add_char_params));
 
    add_char_params.uuid              = TX_CHARA_UUID;

    add_char_params.uuid_type         = BLE_UUID_TYPE_BLE;

    add_char_params.init_len          = sizeof(uint8_t);

    add_char_params.max_len           = 264;

    add_char_params.char_props.read   = 1;

    add_char_params.char_props.notify = 1;
 
    add_char_params.read_access       = SEC_OPEN;

    add_char_params.cccd_write_access = SEC_OPEN;
 
    err_code = characteristic_add(psUartService->usServiceHdl, &add_char_params, &(psUartService->sTxHandle));

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
 
    return NRF_SUCCESS;

}
 
 
uint32_t DoNotification(_sUartService * psUartService, uint8_t *pucNotifyByf)

{

    uint32_t err_code;
    const uint8_t sampleBuf[2] = {'A','B'};
 
    // Send value if connected and notifying

    if (psUartService->usConnHdl != BLE_CONN_HANDLE_INVALID)

    {

        uint16_t               len;

        uint16_t               hvx_len;

        ble_gatts_hvx_params_t hvx_params;
 
      //  len     = hrm_encode(p_hrs, heart_rate, encoded_hrm);

        hvx_len = 264;
        //len = 10;
 
        memset(&hvx_params, 0, sizeof(hvx_params));
 
        hvx_params.handle = psUartService->sTxHandle.value_handle;

        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

        hvx_params.offset = 0;

        hvx_params.p_len  = &hvx_len;

        hvx_params.p_data = pucNotifyByf;
 
        err_code = sd_ble_gatts_hvx(psUartService->usConnHdl, &hvx_params);

        if ((err_code == NRF_SUCCESS) && (hvx_len != len))

        {

            err_code = NRF_ERROR_DATA_SIZE;

        }

    }

    else

    {

        err_code = NRF_ERROR_INVALID_STATE;

    }
 
    return err_code;

}

/**
 * @brief Pushes a byte into the circular buffer.
 * @param[in] c Circular buffer.
 * @param[in] data Byte to push into the buffer.
 * @return 0 on success, -1 on failure.
*/
int CircularBufPush(_sCircularBuf *c, uint8_t data) 
{
    int next;
    next = c->head + 1;

    if (next >= c->maxlen)
    {
        next = 0;
    }
    if (next == c->tail)
    {
        return -1;
    }

    c->buffer[c->head] = data;
    c->head = next;

    return 0;

}

/**
 * @brief Pops a byte from the circular buffer.
 * @param[in] c Circular buffer.
 * @param[in] data Byte to pop from the buffer.
 * @return 0 on success, -1 on failure.
*/
int CircularBufPop(_sCircularBuf *c, uint8_t *data) {

    int next;
 
    if (c->head == c->tail)
    {
        return -1;
    }
    next = c->tail + 1;

    if (next >= c->maxlen)
    {
        next = 0;
    }
    *data = c->buffer[c->tail];
    c->tail = next;
    return 0;
}