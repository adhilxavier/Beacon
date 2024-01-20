#include "ButtonHandler.h"

static _eDeviceState DeviceState = DEV_IDLE;

/**
* Getting device state
*/
_eDeviceState *GetDeviceState()
{
  return &DeviceState;
}

/**
* Set device state
*/
void SetDeviceState(_eDeviceState DevState)
{
  DeviceState = DevState;
}

/**
* Button interrrupt handler
*/
void ButtonIntHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (pin == BUTTON_1)
  {
    if (DeviceState == DEV_IDLE || DeviceState == DEV_BEACON)
    {
      DeviceState = DEV_CONN_ADV;
    }
    else 
    {
      DeviceState = DEV_IDLE;
    }
  }
  else if (pin == BUTTON_2)
  { 
    if (DeviceState == DEV_IDLE || DeviceState == DEV_CONN)
    {
      DeviceState = DEV_BEACON_ADV; 
    }
    else
    {
      DeviceState = DEV_IDLE;
    }    
  }
}

/**
* GPIO initialization
*/
void GPIOInit(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, ButtonIntHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);

    err_code = nrf_drv_gpiote_in_init(BUTTON_2, &in_config, ButtonIntHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_2, true);
}