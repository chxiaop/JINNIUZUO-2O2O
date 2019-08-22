#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"

#define init_speed 0      
#define low_speed 300  //Ĭ������Ϊ25~27m/s
#define energy_speed 410   //������������Ϊ27.5~29.5m/s
#define high_speed 350
typedef enum
{
  FRIC_SPEED_DEFAULT = 0x00,
  FRIC_SPEED_ENERGY  = 0x01,
  FRIC_SPEED_VISION  = 0x02,
	FRIC_SPEED_STOP    = 0x03,
	FRIC_SPEED_HIGH    = 0x04,
	FRIC_SPEED_LOW     = 0x05,
} fric_speed_e;

void FricMotor_init(void);
void fricmotor_status(void);
void FricGunControl(uint16_t pwm1,uint16_t pwm2);
void laser_on(void);
void laser_off(void);
#endif
