
#include "sdk_common.h"
#include "ble_lbs_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"
#include "UartSvcClient.h"


#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */


/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    _sUartSvcClient *psUartSvcClient = (_sUartSvcClient *)p_ctx;

    //NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (psUartSvcClient->ErrorHdlr != NULL)
    {
        psUartSvcClient->ErrorHdlr(nrf_error);
    }
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function uses the Handle Value Notification received from the SoftDevice
 *          and checks whether it is a notification of Button state from the peer. If
 *          it is, this function decodes the state of the button and sends it to the
 *          application.
 *
 * @param[in] p_ble_lbs_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(_sUartSvcClient *psUartSvcClient, ble_evt_t const * p_ble_evt)
{
    // Check if the event is on the link for this instance.
    if (psUartSvcClient->usConnHdl != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if this is a Button notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == psUartSvcClient->sUartSvcCliHdlDb.usTxCharaHandle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len)
        {
            _sUartSvcClientEvent sUartSvcClientEvent;

            sUartSvcClientEvent.EvtType              = UART_DATA_RCV_NOTIFICATION;
            sUartSvcClientEvent.usConnHdl            = psUartSvcClient->usConnHdl;
            memcpy(sUartSvcClientEvent.params.sRcvdData.ucRcvdData, 
                  p_ble_evt->evt.gattc_evt.params.hvx.data[0],
                  p_ble_evt->evt.gattc_evt.params.hvx.len);
            psUartSvcClient->EvtHdlr(psUartSvcClient, &sUartSvcClientEvent);
        }
    }
}


/**@brief Function for handling the Disconnected event received from the SoftDevice.
 *
 * @details This function checks whether the disconnect event is happening on the link
 *          associated with the current instance of the module. If the event is happening, the function sets the instance's
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_lbs_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(_sUartSvcClient *psUartSvcClient, ble_evt_t const * p_ble_evt)
{
    if (psUartSvcClient->usConnHdl == p_ble_evt->evt.gap_evt.conn_handle)
    {
        psUartSvcClient->usConnHdl                            = BLE_CONN_HANDLE_INVALID;
        psUartSvcClient->sUartSvcCliHdlDb.usTxCharaCCCDHandle = BLE_GATT_HANDLE_INVALID;
        psUartSvcClient->sUartSvcCliHdlDb.usTxCharaHandle     = BLE_GATT_HANDLE_INVALID;
    }
}


void UartSvcClientOnDbDiscEvent(_sUartSvcClient *psUartSvcClient, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the LED Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == SERVICE_UUID &&
        p_evt->params.discovered_db.srv_uuid.type == psUartSvcClient->ucUUIDType)
    {
        _sUartSvcClientEvent sUartSvcClientEvent;

        sUartSvcClientEvent.EvtType    = UART_SERVICE_DISCOVERY_EVT_COMPLETE;
        sUartSvcClientEvent.usConnHdl = p_evt->conn_handle;

        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            switch (p_char->characteristic.uuid.uuid)
            {
                // case LBS_UUID_LED_CHAR:
                //     evt.params.peer_db.led_handle = p_char->characteristic.handle_value;
                //     break;
                case TX_CHARA_UUID:
                    sUartSvcClientEvent.params.HdlDb.usTxCharaHandle  = p_char->characteristic.handle_value;
                    sUartSvcClientEvent.params.HdlDb.usTxCharaCCCDHandle = p_char->cccd_handle;
                    break;

                default:
                    break;
            }
        }

        //NRF_LOG_DEBUG("LED Button Service discovered at peer.");
        //If the instance was assigned prior to db_discovery, assign the db_handles
        if (psUartSvcClient->usConnHdl != BLE_CONN_HANDLE_INVALID)
        {
            if ((psUartSvcClient->sUartSvcCliHdlDb.usTxCharaHandle     == BLE_GATT_HANDLE_INVALID)&&
                (psUartSvcClient->sUartSvcCliHdlDb.usTxCharaCCCDHandle == BLE_GATT_HANDLE_INVALID))
            {
                psUartSvcClient->sUartSvcCliHdlDb = sUartSvcClientEvent.params.HdlDb;
            }
        }

        psUartSvcClient->EvtHdlr(psUartSvcClient, &sUartSvcClientEvent);

    }
}


uint32_t UartSvcClientInit(_sUartSvcClient *psUartSvcClient, _sUartSvcClientInit * psUartSvcClientInit)
{
    uint32_t      err_code;
    ble_uuid_t    UartSvcUuid;
    ble_uuid128_t lbs_base_uuid = {UART_SERVICE_UUID};

    VERIFY_PARAM_NOT_NULL(psUartSvcClient);
    VERIFY_PARAM_NOT_NULL(psUartSvcClientInit);
    VERIFY_PARAM_NOT_NULL(psUartSvcClientInit->EvtHdlr);
    VERIFY_PARAM_NOT_NULL(psUartSvcClientInit->p_gatt_queue);

    psUartSvcClient->sUartSvcCliHdlDb.usTxCharaCCCDHandle = BLE_GATT_HANDLE_INVALID;
    psUartSvcClient->sUartSvcCliHdlDb.usTxCharaHandle     = BLE_GATT_HANDLE_INVALID;
    psUartSvcClient->usConnHdl                            = BLE_CONN_HANDLE_INVALID;
    psUartSvcClient->EvtHdlr                              = psUartSvcClientInit->EvtHdlr;
    psUartSvcClient->p_gatt_queue                         = psUartSvcClientInit->p_gatt_queue;
    psUartSvcClient->ErrorHdlr                            = psUartSvcClientInit->ErrorHdlr;

    err_code = sd_ble_uuid_vs_add(&lbs_base_uuid, &psUartSvcClient->ucUUIDType);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    UartSvcUuid.type = psUartSvcClient->ucUUIDType;
    UartSvcUuid.uuid = SERVICE_UUID;

    return ble_db_discovery_evt_register(&UartSvcUuid);
}

void UartSvcClientOnbleEvt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    _sUartSvcClient * psUartSvcClient = (_sUartSvcClient *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(psUartSvcClient, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(psUartSvcClient, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for configuring the CCCD.
 *
 * @param[in] p_ble_lbs_c Pointer to the LED Button Client structure.
 * @param[in] enable      Whether to enable or disable the CCCD.
 *
 * @return NRF_SUCCESS if the CCCD configure was successfully sent to the peer.
 */
static uint32_t cccd_configure(_sUartSvcClient *psUartSvcClient, bool enable)
{
    //NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
    //              psUartSvcClient->sUartSvcCliHdlDb.usTxCharaCCCDHandle,
    //              psUartSvcClient->usConnHdl);

    nrf_ble_gq_req_t cccd_req;
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    uint8_t          cccd[WRITE_MESSAGE_LENGTH];

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = psUartSvcClient;
    cccd_req.params.gattc_write.handle   = psUartSvcClient->sUartSvcCliHdlDb.usTxCharaCCCDHandle;
    cccd_req.params.gattc_write.len      = WRITE_MESSAGE_LENGTH;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(psUartSvcClient->p_gatt_queue, &cccd_req, psUartSvcClient->usConnHdl);
}


uint32_t UartSvcClientTxNotiFicationEnable(_sUartSvcClient *psUartSvcClient)
{
    VERIFY_PARAM_NOT_NULL(psUartSvcClient);

    if (psUartSvcClient->usConnHdl == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure(psUartSvcClient,
                          true);
}


// uint32_t ble_lbs_led_status_send(ble_lbs_c_t * p_ble_lbs_c, uint8_t status)
// {
//     VERIFY_PARAM_NOT_NULL(p_ble_lbs_c);

//     if (p_ble_lbs_c->conn_handle == BLE_CONN_HANDLE_INVALID)
//     {
//         return NRF_ERROR_INVALID_STATE;
//     }

//     NRF_LOG_DEBUG("Writing LED status 0x%x", status);

//     nrf_ble_gq_req_t write_req;

//     memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

//     write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
//     write_req.error_handler.cb            = gatt_error_handler;
//     write_req.error_handler.p_ctx         = p_ble_lbs_c;
//     write_req.params.gattc_write.handle   = p_ble_lbs_c->peer_lbs_db.led_handle;
//     write_req.params.gattc_write.len      = sizeof(status);
//     write_req.params.gattc_write.p_value  = &status;
//     write_req.params.gattc_write.offset   = 0;
//     write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD; 

//     return nrf_ble_gq_item_add(p_ble_lbs_c->p_gatt_queue, &write_req, p_ble_lbs_c->conn_handle);
// }

uint32_t UartSvcClientHandlesAssign(_sUartSvcClient *psUartSvcClient,
                                  uint16_t         usConnHdl,
                                  const _sUartSvcCliHdlDb *psUartSvcCliHdlDb)
{
    VERIFY_PARAM_NOT_NULL(psUartSvcClient);

    psUartSvcClient->usConnHdl = usConnHdl;
    if (psUartSvcCliHdlDb != NULL)
    {
        psUartSvcClient->sUartSvcCliHdlDb = *psUartSvcCliHdlDb;
    }
    return nrf_ble_gq_conn_handle_register(psUartSvcClient->p_gatt_queue, usConnHdl);
}

