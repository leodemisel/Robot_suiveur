#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <audio/play_melody.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <leds.h>
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include <motors_control.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

static bool obstacle;
static float left_motor_sound;
static float right_motor_sound;
static float left_motor_ir;
static float right_motor_ir;

void set_motor_sound(float left, float right){
	left_motor_sound = left;
	right_motor_sound = right;
}

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
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
    systime_t time;
    time = chVTGetSystemTime();

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
    	if(get_prox(2)>10 && get_prox(5)>10 && get_prox(0)>30 && get_prox(1) >30 && get_prox(6) >30 && get_prox(7) > 30){
    		while(1){
    		    rotate_left();
    		    set_led(LED1, 1);
    		    set_led(LED3, 1);
    		    set_led(LED5, 1);
    		    set_led(LED7, 1);
    		}
    		chThdSleepMilliseconds(50);
    	}else if(get_prox(0) > 40 && get_prox(1) > 40 && get_prox(6) > 40 && get_prox(7) > 40){
       			stuck++;
       			systime_t time;
       			time = chVTGetSystemTime()+ 1300;
       			while(chVTGetSystemTime() != time){
       				rotate_left();
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

    	if(get_prox(0)>30 || get_prox(6)>30 || get_prox(7)>30 || get_prox(1)>30){
    		set_led(LED1, 1);
    		obstacle = 1;
    		left_motor_ir = 700 - get_prox(0)*2 - get_prox(1)*2;
    		right_motor_ir = 700 - get_prox(7)*2 - get_prox(6)*2;
    		chThdSleepMilliseconds(50);
    	} else {
    		set_led(LED1, 0);
    		obstacle = 0;
    		left_motor_ir = 0;
    		right_motor_ir = 0;
    		chThdSleepMilliseconds(50);
    	}
    }
}

int main(void) {
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	//test
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();
    proximity_start();
    imu_start();

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
