#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <arm_math.h>
#include <leds.h>
#include "sensors/proximity.h"

#include <main.h>
#include <audio_processing.h>

#define IR_SIDE_STUCK 10
#define IR_FRONT_STUCK 30
#define IR_FRONT_LEAVE 40
#define IR_OBSTACLE 30

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static bool obstacle;
static float left_motor_sound;
static float right_motor_sound;
static float left_motor_ir;
static float right_motor_ir;

void set_motor_sound(float left, float right){
	left_motor_sound = left;
	right_motor_sound = right;
}

static THD_WORKING_AREA(waAlignSound, 128);
static THD_FUNCTION(ThdAlignSound, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1) {
    	mic_start(&AlignSound);
    	chThdSleepMilliseconds(100);
    }
}

static THD_WORKING_AREA(waMove, 128);
static THD_FUNCTION(ThdMove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1) {
    	if (obstacle == 1) {
    		left_motor_set_speed(left_motor_ir);
    		right_motor_set_speed(right_motor_ir);
    		chThdSleepMilliseconds(100);
    	} else {
    		left_motor_set_speed(left_motor_sound);
    		right_motor_set_speed(right_motor_sound);
    		chThdSleepMilliseconds(100);
    	}
    }
}

static THD_WORKING_AREA(waThdLeave, 128);
static THD_FUNCTION(ThdLeave, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	int stuck = 0;

    while(1){
    	if(get_prox(2)>IR_SIDE_STUCK && get_prox(5)>IR_SIDE_STUCK && get_prox(0)>IR_FRONT_STUCK
    			&& get_prox(1) >IR_FRONT_STUCK && get_prox(6) >IR_FRONT_STUCK && get_prox(7) > IR_FRONT_STUCK){
    		while(1){
    		    left_motor_set_speed(-500);
    		    right_motor_set_speed(500);
    		    set_led(LED1, 1);
    		    set_led(LED3, 1);
    		    set_led(LED5, 1);
    		    set_led(LED7, 1);
    		}
    		chThdSleepMilliseconds(50);
    	}else if(get_prox(0) > IR_FRONT_LEAVE && get_prox(1) > IR_FRONT_LEAVE
    			&& get_prox(6) > IR_FRONT_LEAVE && get_prox(7) > IR_FRONT_LEAVE){
       			stuck++;
       			systime_t time;
       			time = chVTGetSystemTime()+ 1300;
       			while(chVTGetSystemTime() != time){
       				left_motor_set_speed(-500);
       				right_motor_set_speed(500);
       			}

       			time = chVTGetSystemTime()+ 2000;
       			while(chVTGetSystemTime() != time){
       				left_motor_set_speed(300);
       				right_motor_set_speed(800);
       			}

       		chThdSleepMilliseconds(500);
       	}
       	chThdSleepMilliseconds(500);
    }
}


static THD_WORKING_AREA(waThdObstacle, 128);
static THD_FUNCTION(ThdObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    left_motor_ir = 0;
    right_motor_ir = 0;

    while(1){

    	if(get_prox(0)>IR_OBSTACLE || get_prox(6)>IR_OBSTACLE || get_prox(7)>IR_OBSTACLE || get_prox(1)>IR_OBSTACLE){
    		obstacle = 1;
    		left_motor_ir = 700 - get_prox(0)*2 - get_prox(1)*2;
    		right_motor_ir = 700 - get_prox(7)*2 - get_prox(6)*2;
    		chThdSleepMilliseconds(50);
    	} else {
    		obstacle = 0;
    		left_motor_ir = 0;
    		right_motor_ir = 0;
    		chThdSleepMilliseconds(50);
    	}
    }
}

int main(void) {
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    mpu_init();

    motors_init();
    proximity_start();

	chThdCreateStatic(waThdLeave, sizeof(waThdLeave), NORMALPRIO+3, ThdLeave, NULL);
    chThdCreateStatic(waThdObstacle, sizeof(waThdObstacle), NORMALPRIO+2, ThdObstacle, NULL);
    chThdCreateStatic(waAlignSound, sizeof(waAlignSound), NORMALPRIO+1, ThdAlignSound, NULL);
    chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, ThdMove, NULL);

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
