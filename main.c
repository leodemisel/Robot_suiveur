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

#define IR_SIDE_STUCK 	10
#define IR_FRONT_STUCK 	30
#define IR_FRONT_LEAVE 	40
#define IR_OBSTACLE 	30
#define ZERO			0

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

void set_all_leds(){
	set_led(LED1, 1);
	set_led(LED3, 1);
	set_led(LED5, 1);
	set_led(LED7, 1);
}

static THD_WORKING_AREA(waThdStuck, 128);
static THD_FUNCTION(ThdStuck, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	int stuck = 0;

    while(1){
    	if(get_prox(2)>IR_SIDE_STUCK && get_prox(5)>IR_SIDE_STUCK && get_prox(0)>IR_FRONT_STUCK
    			&& get_prox(1) >IR_FRONT_STUCK && get_prox(6) >IR_FRONT_STUCK && get_prox(7) > IR_FRONT_STUCK){//en cas de cul-de-sac
    		while(1){//mode "panique" indiquant qu'il faut reset le robot
    		    left_motor_set_speed(-500);
    		    right_motor_set_speed(500);
    		    set_all_leds();
    		}
    		chThdSleepMilliseconds(50);


    	}else if(get_prox(0) > IR_FRONT_LEAVE && get_prox(1) > IR_FRONT_LEAVE //en cas de petite impasse
    			&& get_prox(6) > IR_FRONT_LEAVE && get_prox(7) > IR_FRONT_LEAVE){
       			stuck++;
       			systime_t time;
       			time = chVTGetSystemTime()+ 1300;
       			while(chVTGetSystemTime() != time){//tourne 180 degrés
       				left_motor_set_speed(-500);
       				right_motor_set_speed(500);
       			}

       			time = chVTGetSystemTime()+ 2000;
       			while(chVTGetSystemTime() != time){//sort de l'impasse
       				left_motor_set_speed(300);
       				right_motor_set_speed(800);
       			}

       		chThdSleepMilliseconds(500);
       	}
       	chThdSleepMilliseconds(500);
    }
}


static THD_WORKING_AREA(waThdWalls, 128);
static THD_FUNCTION(ThdWalls, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	if(get_prox(0)>IR_OBSTACLE || get_prox(6)>IR_OBSTACLE || get_prox(7)>IR_OBSTACLE || get_prox(1)>IR_OBSTACLE){
    		//if (les capteurs ir sont trop proches d'un obstacle)
    		obstacle = 1; // update valeur du booléen obstacle

    		//update valeurs des vitesses des moteurs
    		left_motor_ir = 700 - get_prox(0)*2 - get_prox(1)*2;
    		right_motor_ir = 700 - get_prox(7)*2 - get_prox(6)*2;
    		chThdSleepMilliseconds(50);
    	} else {
    		//mise à zero des valeurs
    		obstacle = 0;
    		left_motor_ir = ZERO;
    		right_motor_ir = ZERO;
    		chThdSleepMilliseconds(50);
    	}
    }
}

static THD_WORKING_AREA(waThdSound, 128);
static THD_FUNCTION(ThdSound, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1) {
    	mic_start(&processAudioData); // voir audio_processing.c
    	chThdSleepMilliseconds(100);
    }
}

static THD_WORKING_AREA(waThdMotors, 128);
static THD_FUNCTION(ThdMotors, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1) {
    	if (obstacle == 1) { //si obstacle alors les valeurs du thread Walls priment sur celles du thread Sound
    		left_motor_set_speed(left_motor_ir);
    		right_motor_set_speed(right_motor_ir);
    		chThdSleepMilliseconds(100);
    	} else { //sinon on utilise les valeurs pour se diriger vers le son
    		left_motor_set_speed(left_motor_sound);
    		right_motor_set_speed(right_motor_sound);
    		chThdSleepMilliseconds(100);
    	}
    }
}

int main(void) {

	//initialisation périphériques
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    proximity_start();

    //initialisation variables
    left_motor_ir = ZERO;
    right_motor_ir = ZERO;

    //threads et priorités
	chThdCreateStatic(waThdStuck, sizeof(waThdStuck), NORMALPRIO+3, ThdStuck, NULL);
    chThdCreateStatic(waThdWalls, sizeof(waThdWalls), NORMALPRIO+2, ThdWalls, NULL);
    chThdCreateStatic(waThdSound, sizeof(waThdSound), NORMALPRIO+1, ThdSound, NULL);
    chThdCreateStatic(waThdMotors, sizeof(waThdMotors), NORMALPRIO, ThdMotors, NULL);

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
