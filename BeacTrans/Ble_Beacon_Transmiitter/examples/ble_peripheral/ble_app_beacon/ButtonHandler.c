#include "ButtonHandler.h"

_eDeviceState DeviceState = DEV_IDLE;

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
  if (DeviceState == DEV_IDLE)
  {
    if (pin == BUTTON_1)
    {
      SetDeviceState(DEV_CONN);
    }
    else if (pin == BUTTON_2)
    {
      SetDeviceState(DEV_BEACON);      
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

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, ButtonIntHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);

    err_code = nrf_drv_gpiote_in_init(BUTTON_2, &in_config, ButtonIntHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_2, true);
}