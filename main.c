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
#include <sensors/proximity.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

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


static THD_WORKING_AREA(waThdFrontLed, 128);
static THD_FUNCTION(ThdFrontLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        palTogglePad(GPIOD, GPIOD_LED_FRONT);
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

static THD_WORKING_AREA(wabouge, 128);
static THD_FUNCTION(bouge, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1) {
    	mic_start(&processAudioData);
        }
}

static THD_WORKING_AREA(waThdBodyLed, 128);
static THD_FUNCTION(ThdBodyLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        palTogglePad(GPIOB, GPIOB_LED_BODY);
        set_led(LED5,2);
        chThdSleepMilliseconds(500);
        palTogglePad(GPIOB, GPIOB_LED_BODY);
        set_led(LED5,0);
        chThdSleepMilliseconds(500);
    }
}

static THD_WORKING_AREA(waThdObstacle, 128);
static THD_FUNCTION(ThdObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

		if(get_prox(0) <= 140 && get_prox(7) <= 140){
			left_motor_set_speed(-500);
		} if(get_prox(0) <= 140 && get_prox(1) <= 140){
			left_motor_set_speed(-500);
		} else{
			left_motor_set_speed(500);
			right_motor_set_speed(500);
		}

    }
}


int main(void)
{
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
    //calibrate_ir();


//chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO +1, ThdFrontLed, NULL);
chThdCreateStatic(waThdBodyLed, sizeof(waThdBodyLed), NORMALPRIO, ThdBodyLed, NULL);
chThdCreateStatic(wabouge, sizeof(wabouge), NORMALPRIO , bouge, NULL);
//chThdCreateStatic(waThdObstacle, sizeof(waThdObstacle), NORMALPRIO , ThdObstacle, NULL);

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
