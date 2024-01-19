

#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif


#define UART_SERVICE_UUID   {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define SERVICE_UUID        0x6695
#define TX_CHARA_UUID       0x1408
#define RX_CHARA_UUID       0x2000

/**@brief Uart Service event type. */
typedef enum __eUartServiceEvt
{
    UART_SVC_EVT_NONE,
    UART_SVC_EVT_NOTIFICATION_ENABLED,   /**< Heart Rate value notification enabled event. */
    UART_SVC_EVT_NOTIFICATION_DISABLED,   /**< Heart Rate value notification disabled event. */
    UART_SVC_EVT_CONNECTED,
    UART_SVC_EVT_DISCONNECTED
}_eUartServiceEvt;

/**@brief Heart Rate Service event. */
typedef struct __sUartServiceEvtType
{
    _eUartServiceEvt  eEvtType;    /**< Type of event. */
} _sUartServiceEvtType;

// Forward declaration of the ble_hrs_t type.
typedef struct __sUartService _sUartService;

/**@brief Heart Rate Service event handler type. */
typedef void (*UartServiceEvtHandler) (_sUartService *psUartService, _sUartServiceEvtType *pEvtType);

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct __sUartServiceInit
{
    UartServiceEvtHandler        ServiceEvtHandler;                                    /**< Event handler to be called for handling events in the Heart Rate Service. */
} _sUartServiceInit;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
typedef struct __sUartService
{
    UartServiceEvtHandler        ServiceEvtHandler;     /**< Event handler to be called for handling events */
    uint16_t                     usServiceHdl;          /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     sTxHandle;              /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     sRxHandle;              /**< Handles related to the Body Sensor Location characteristic. */
    uint16_t                     usConnHdl;          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */                                  /**< Current maximum HR measurement length, adjusted according to the current ATT MTU. */
}_sUartService;


/**@brief Function for initializing the  Service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t UartServiceInit(_sUartService *psUartService, _sUartServiceInit const *psServiceInit);


///**@brief Function for handling the GATT module's events.
// *
// * @details Handles all events from the GATT module of interest to the Heart Rate Service.
// *
// * @param[in]   p_hrs      Heart Rate Service structure.
// * @param[in]   p_gatt_evt  Event received from the GATT module.
// */
//void UartServiceOnGattEvt(_sUartService *psUartService, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Heart Rate Service structure.
 */
void UartServiceOnBleEvt(ble_evt_t const * p_ble_evt, void * p_context);


///**@brief Function for sending heart rate measurement if notification has been enabled.
// *
// * @details The application calls this function after having performed a heart rate measurement.
// *          If notification has been enabled, the heart rate measurement data is encoded and sent to
// *          the client.
// *
// * @param[in]   p_hrs                    Heart Rate Service structure.
// * @param[in]   heart_rate               New heart rate measurement.
// *
// * @return      NRF_SUCCESS on success, otherwise an error code.
// */
uint32_t DoNotification(_sUartService * psUartService, uint16_t heart_rate);




#ifdef __cplusplus
}
#endif

#endif // UART_SERVICE_H

/** @} */
