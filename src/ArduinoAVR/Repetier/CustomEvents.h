#ifndef CUSTOM_EVENTS_H_INCLUDED
#define CUSTOM_EVENTS_H_INCLUDED

#define EVENT_INITIALIZE {UM2LED_init();}
#define EVENT_WAITING_HEATER(id) {UM2LED_waitingheater(id);}
#define EVENT_HEATING_FINISHED(id) {UM2LED_heatingfinished(id);}
#define EVENT_TIMER_100MS {UM2LED_update();}

void UM2LED_init();
void UM2LED_update();
void UM2LED_waitingheater(int8_t id);
void UM2LED_heatingfinished(int8_t id);
bool UM2LED_endstop(bool force);

#endif //CUSTOM_EVENTS_H_INCLUDED
