
/*
  Makers Tool Works RGB LED I2C control
  I2C Device ID = 21

  Repetier-Firmware port/rewrite by exuvo 2015-05-17
  Contributed to UM2 by OhmEye October 2014
  */

// option switches
#define UM2LED_cool EXTRUDER_FAN_COOL_TEMP      // The temp at which the hotend is considered cooled down and safe
#define UM2LED_swing 4                          // how far off before the temperature is not considered 'at temp' in degrees C
#define UM2LED_endstoptimer 3                   // how many seconds to display endstop status

// patterns                 R   G   B
#define UM2led_ready        0, 30,  0    // Printer Ready
#define UM2led_startup     30, 30, 30    // Printer startup
#define UM2led_temphit     40, 40, 40    // Hotend is at target temp
#define UM2led_templow     40,  0, 40    // Hotend heater is slightly lower than target temp
#define UM2led_temphigh    40,  0,  0    // Hotend heater is slightly higher than target temp
#define UM2led_heateroff    0,  0, 40    // Hotend heater is off but still hot
#define UM2led_heating0     0,  0, 50    // Hotend heating up <10%
#define UM2led_heating1     0,  0,100    // Hotend heating up <20%
#define UM2led_heating2     0, 50,100    // Hotend heating up <30%
#define UM2led_heating3     0,100,100    // Hotend heating up <40%
#define UM2led_heating4     0,100, 50    // Hotend heating up <50%
#define UM2led_heating5   100,100,  0    // Hotend heating up <60%
#define UM2led_heating6   100, 50,  0    // Hotend heating up <70%
#define UM2led_heating7   100,  0,100    // Hotend heating up <80%
#define UM2led_heating8   100,  0, 50    // Hotend heating up <90%
#define UM2led_heating9   100,  0,  0    // Hotend heating up <100%

#define UM2LED_ADDRESS 0b1100000

// Pattern Selection Table for defaults that must not be changed
#define UM2led_nochange 	1	// Reserved for no change to LED Strip

#ifndef UINT8_MAX
#define UINT8_MAX 255
#endif // UINT8_MAX


union patterncode {  // access a pattern both as 32 bits and as array of 4 uint8_ts.
  uint32_t value;
  uint8_t part[4];
};

patterncode UM2LED_lastpattern;
uint16_t UM2LED_timer;
bool UM2LED_starup;
uint8_t waitingForHeaterIndex = UINT8_MAX;
float waitingForHeaterStartC;

uint32_t UM2LED_convert(uint8_t red, uint8_t green, uint8_t blue) {
  patterncode pc;
  pc.part[0] = 0;
  pc.part[1] = red;
  pc.part[2] = green;
  pc.part[3] = blue;
  return pc.value;
}

void UM2LED_write(uint8_t addr, uint8_t data) {
  HAL::i2cStart((UM2LED_ADDRESS << 1) + I2C_WRITE);
  HAL::i2cWrite(addr);
  HAL::i2cWrite(data);
  HAL::i2cStop();
}

void UM2LED_init() {
  HAL::i2cInit(400000L);
  UM2LED_endstop(true);

  UM2LED_write(0, 0x80);//MODE1
  UM2LED_write(1, 0x1C);//MODE2
  UM2LED_write(2, 0);//PWM0
  UM2LED_write(3, 0);//PWM1
  UM2LED_write(4, 0);//PWM2
  UM2LED_write(5, 0x00);//PWM3
  UM2LED_write(6, 0xFF);//GRPPWM
  UM2LED_write(7, 0x00);//GRPFREQ
  UM2LED_write(8, 0xAA);//LEDOUT
}

// send pattern frame via I2C
void UM2LED_write(uint8_t red, uint8_t green, uint8_t blue, uint16_t timer) {
  uint32_t converted = UM2LED_convert(red, green, blue);

  if(converted != UM2LED_lastpattern.value) {  // don't send sequential identical patterns
    UM2LED_lastpattern.value = converted;  // update states

    UM2LED_write(2, red);
    UM2LED_write(3, green);
    UM2LED_write(4, blue);

    if(timer) {
      UM2LED_timer = timer * 10U;
    }
  }
}

bool UM2LED_endstop(bool force) {
  uint8_t endx=0, endy=0, endz=0;

  Endstops::update();

  #if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
    if(Endstops::xMin()) endx |= 1;
  #endif
  #if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
    if(Endstops::yMin()) endy |= 1;
  #endif
  #if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
    if(Endstops::zMin()) endz |= 1;
  #endif
  #if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
    if(Endstops::xMax()) endx |= 1;
  #endif
  #if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
    if(Endstops::yMax()) endy |= 1;
  #endif
  #if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
    if(Endstops::zMax()) endz |= 1;
  #endif

  if(force || endx || endy || endz) {
    UM2LED_write(endx, endy, endz, UM2LED_endstoptimer);

    if(endx || endy || endz) {
        return true;
    }
  }

  return false;
}

void UM2LED_waitingheater(int8_t id){
  if(id == -1){
    waitingForHeaterIndex = NUM_TEMPERATURE_LOOPS - 1;
  } else if (id >= 0 && id < NUM_TEMPERATURE_LOOPS) {
    waitingForHeaterIndex = id;
  } else {
   //Error
   waitingForHeaterIndex = UINT8_MAX;
  }

  if(waitingForHeaterIndex < NUM_TEMPERATURE_LOOPS){
    waitingForHeaterStartC = tempController[waitingForHeaterIndex]->currentTemperatureC;
  }
}

void UM2LED_heatingfinished(int8_t id){
  waitingForHeaterIndex = UINT8_MAX;
}

//does percentile display between start temp and target temp while a heater is heating up
bool UM2LED_temp() {
	if(waitingForHeaterIndex >= NUM_TEMPERATURE_LOOPS){
    return false;
	}

  float target = tempController[waitingForHeaterIndex]->targetTemperatureC;
  float current = tempController[waitingForHeaterIndex]->currentTemperatureC;
  float start = waitingForHeaterStartC;

  uint8_t pattern = (10.0f * (1.0f - ((target - current) / (target - start))));

  switch(pattern){
    default:
    case 0:
      UM2LED_write(UM2led_heating0, 0);
      break;
    case 1:
      UM2LED_write(UM2led_heating1, 0);
      break;
    case 2:
      UM2LED_write(UM2led_heating2, 0);
      break;
    case 3:
      UM2LED_write(UM2led_heating3, 0);
      break;
    case 4:
      UM2LED_write(UM2led_heating4, 0);
      break;
    case 5:
      UM2LED_write(UM2led_heating5, 0);
      break;
    case 6:
      UM2LED_write(UM2led_heating6, 0);
      break;
    case 7:
      UM2LED_write(UM2led_heating7, 0);
      break;
    case 8:
      UM2LED_write(UM2led_heating8, 0);
      break;
    case 9:
      UM2LED_write(UM2led_heating9, 0);
      break;
  }

  return true;
}

//Called every 100ms
void UM2LED_update() {

  if(UM2LED_temp() || UM2LED_endstop(false) || UM2LED_lastpattern.part[0] == UM2led_nochange) {
    return;
  }

  if(UM2LED_timer > 0){
    UM2LED_timer--;
    return;
  }

  float currentTempC =  0;
  float targetTempC = 0;

  for(uint8_t i=0; i < NUM_EXTRUDER; i++){
    float target = tempController[i]->targetTemperatureC;
    float current = tempController[i]->currentTemperatureC;

    if(target > targetTempC || (targetTempC == 0 && target == 0 && current > currentTempC)){
      currentTempC = current;
      targetTempC = target;
    }
  }

  if(targetTempC == 0) {
    if(currentTempC > UM2LED_cool) { // heater is off but still warm
      UM2LED_write(UM2led_heateroff, 0);
    } else {
      UM2LED_write(UM2led_ready, 0);
    }

  } else {
    uint8_t swing = abs(targetTempC - currentTempC); // how far off from target temp we are

    if(swing < UM2LED_swing * 2) {                  // if within double the swing threshold
      if(swing < UM2LED_swing) {
        UM2LED_write(UM2led_temphit, 0);  // close to target temp, so consider us 'at temp'
      } else {
        if(currentTempC >= targetTempC) {
          UM2LED_write(UM2led_temphigh, 0);   // temp high, heater is off
        } else {
          UM2LED_write(UM2led_templow, 0);    // temp low, heater is on
        }
      }
    }
  }
}
