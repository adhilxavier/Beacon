#include "nrf_drv_gpiote.h"

#define BUTTON_1       11
#define BUTTON_2       12
#define BUTTON_3       24
#define BUTTON_4       25

typedef  enum __eDeviceState
{
    DEV_IDLE,
    DEV_BEACON,
    DEV_CONN
}_eDeviceState; 

void ButtonIntHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void GPIOInit(void);
_eDeviceState *GetDeviceState();
void SetDeviceState(_eDeviceState DevState);