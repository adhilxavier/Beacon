
#ifndef UART_SVC_C_H__
#define UART_SVC_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_lbs_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define UART_SVC_C_DEF(_name)                                                                        \
static _sUartSvcClient _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     2,                                                   \
                     UartSvcClientOnbleEvt, &_name)

/**@brief   Macro for defining multiple ble_lbs_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define UART_SVC_C_ARRAY_DEF(_name, _cnt)                                                            \
static _sUartSvcClient _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      2,                                                  \
                      UartSvcClientOnbleEvt, &_name, _cnt)


#define UART_SERVICE_UUID   {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define SERVICE_UUID        0x6695
#define TX_CHARA_UUID       0x1408
#define RX_CHARA_UUID       0x2000

/**@brief LBS Client event type. */
typedef enum
{
    UART_SERVICE_DISCOVERY_EVT_COMPLETE = 1,  /**< Event indicating that the LED Button Service was discovered at the peer. */
    UART_DATA_RCV_NOTIFICATION,      /**< Event indicating that a notification of the LED Button Button characteristic was received from the peer. */
    UART_SVC_DISCONNECTED
} UartSvcCltEvtType;

/**@brief Structure containing the Button value received from the peer. */
typedef struct __sRCvdData
{
    uint8_t ucRcvdData[247];  /**< Button Value. */
} _sRCvdData;

/**@brief Structure containing the handles related to the LED Button Service found on the peer. */
typedef struct __sUartSvcCliHdlDb
{
    uint16_t usTxCharaCCCDHandle;  /**< Handle of the CCCD of the Button characteristic. */
    uint16_t usTxCharaHandle;       /**< Handle of the Button characteristic as provided by the SoftDevice. */
} _sUartSvcCliHdlDb;

/**@brief LED Button Event structure. */
typedef struct __sUartSvcClientEvent
{
    UartSvcCltEvtType EvtType;    /**< Type of the event. */
    uint16_t          usConnHdl; /**< Connection handle on which the event occured.*/
    union
    {
        _sRCvdData            sRcvdData;          /**< Button value received. This is filled if the evt_type is @ref UART_DATA_RCV_NOTIFICATION. */
        _sUartSvcCliHdlDb     HdlDb;         /**< Handles related to the LED Button Service found on the peer device. This is filled if the evt_type is @ref UART_SERVICE_DISCOVERY_EVT_COMPLETE.*/
    } params;
} _sUartSvcClientEvent;

// Forward declaration of the ble_lbs_c_t type.
typedef struct __sUartSvcClient _sUartSvcClient;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* UartSvcClientEvtHandler) (_sUartSvcClient * sUartSvcClient, _sUartSvcClientEvent *pEvent);

/**@brief LED Button Client structure. */
struct __sUartSvcClient
{
    uint16_t                  usConnHdl;   /**< Connection handle as provided by the SoftDevice. */
    _sUartSvcCliHdlDb         sUartSvcCliHdlDb;   /**< Handles related to LBS on the peer. */
    UartSvcClientEvtHandler   EvtHdlr;   /**< Application event handler to be called when there is an event related to the LED Button service. */
    ble_srv_error_handler_t   ErrorHdlr; /**< Function to be called in case of an error. */
    uint8_t                   ucUUIDType;     /**< UUID type. */
    nrf_ble_gq_t            * p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
};

/**@brief LED Button Client initialization structure. */
typedef struct __sUartSvcClientInit
{
    UartSvcClientEvtHandler   EvtHdlr;   /**< Event handler to be called by the LED Button Client module when there is an event related to the LED Button Service. */
    nrf_ble_gq_t            * p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
    ble_srv_error_handler_t   ErrorHdlr; /**< Function to be called in case of an error. */
} _sUartSvcClientInit;


/**@brief Function for initializing the LED Button client module.
 *
 * @details This function registers with the Database Discovery module for the
 *          LED Button Service. The module looks for the presence of a LED Button Service instance
 *          at the peer when a discovery is started.
 *
 * @param[in] sUartSvcClient      Pointer to the LED Button client structure.
 * @param[in] p_ble_lbs_c_init Pointer to the LED Button initialization structure that contains the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. 
 * @retval    err_code    Otherwise, this function propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t UartSvcClientInit(_sUartSvcClient *psUartSvcClient, _sUartSvcClientInit * psUartSvcClientInit);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function handles the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the LED Button Client module, the function uses the event's data to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the LED button client structure.
 */
void UartSvcClientOnbleEvt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for requesting the peer to start sending notification of the Button
 *        Characteristic.
 *
 * @details This function enables to notification of the Button at the peer
 *          by writing to the CCCD of the Button characteristic.
 *
 * @param[in] p_ble_lbs_c Pointer to the LED Button Client structure.
 *
 * @retval  NRF_SUCCESS 			If the SoftDevice has been requested to write to the CCCD of the peer.
 * @retval  NRF_ERROR_INVALID_STATE If no connection handle has been assigned (@ref ble_lbs_c_handles_assign).
 * @retval  NRF_ERROR_NULL 			If the given parameter is NULL.
 * @retval  err_code				Otherwise, this API propagates the error code returned by function
 *          						@ref nrf_ble_gq_item_add.
 */
uint32_t UartSvcClientTxNotiFicationEnable(_sUartSvcClient *psUartSvcClient);


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details Call this function when you get a callback event from the Database Discovery module. This
 *          function handles an event from the Database Discovery module, and determines whether it
 *          relates to the discovery of LED Button service at the peer. If it does, this function calls the
 *          application's event handler to indicate that the LED Button service was discovered
 *          at the peer. The function also populates the event with service-related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_lbs_c Pointer to the LED Button client structure.
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 */
void UartSvcClientOnDbDiscEvent(_sUartSvcClient *psUartSvcClient, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning handles to this instance of lbs_c.
 *
 * @details Call this function when a link has been established with a peer to associate the link
 *          to this instance of the module. This makes it possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_lbs_c    Pointer to the LED Button client structure instance for associating the link.
 * @param[in] conn_handle    Connection handle to associate with the given LED Button Client Instance.
 * @param[in] p_peer_handles LED Button Service handles found on the peer (from @ref UART_SERVICE_DISCOVERY_EVT_COMPLETE event).
 *
 * @retval NRF_SUCCESS If the status was sent successfully.
 * @retval err_code    Otherwise, this API propagates the error code returned by function
 *                     @ref nrf_ble_gq_item_add.
 *
 */
uint32_t UartSvcClientHandlesAssign(_sUartSvcClient *psUartSvcClient,
                                  uint16_t         usConnHdl,
                                  const _sUartSvcCliHdlDb *psUartSvcCliHdlDb);


/**@brief Function for writing the LED status to the connected server.
 *
 * @param[in] p_ble_lbs_c Pointer to the LED Button client structure.
 * @param[in] status      LED status to send.
 *
 * @retval NRF_SUCCESS If the status was sent successfully.
 * @retval err_code    Otherwise, this API propagates the error code returned by function
 *                     @ref nrf_ble_gq_conn_handle_register.
//  */
// uint32_t UartSvcClientWriteToChara(_sUartSvcClient * sUartSvcClient, uint8_t status);


#ifdef __cplusplus
}
#endif

#endif // BLE_LBS_C_H__

/** @} */

