#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_Align(float* dataLeft, float* dataRight, float* dataFront, float* dataBack){

	float max_norm[4] = {MIN_VALUE_THRESHOLD, MIN_VALUE_THRESHOLD, MIN_VALUE_THRESHOLD, MIN_VALUE_THRESHOLD};
					//= {max_norm_left, max_norm_right, max_norm_front, max_norm_back};
	uint16_t max_mic [2] = {0};

	//search for the highest norms
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(dataLeft[i] > max_norm[0]){
			max_norm[0] = dataLeft[i];
		}
		if(dataRight[i] > max_norm[1]){
			max_norm[1] = dataRight[i];
		}
		if(dataFront[i] > max_norm[2]){
			max_norm[2] = dataFront[i];
		}
		if(dataBack[i] > max_norm[3]){
			max_norm[3] = dataBack[i];
		}
	}
	// search for the highest norm
	for(uint16_t i = 1; i < 4; i++){
			if (max_norm[i] > max_norm[max_mic[0]]){
				max_mic[1] = max_mic[0];
				max_mic[0] = i;
			}
		}

	float vitesse = max_norm[max_mic[0]]/100;

	if (vitesse > 150){
		clear_leds();
		if(max_norm[2] > max_norm[3]){ //Sound from front
			set_led(LED1, 1);
			if (max_norm[0] > max_norm[1]){//Son plus fort à gauche qu'à droite
				set_led(LED7, 1);
				left_motor_set_speed(900*max_norm[1]/max_norm[0]);
				right_motor_set_speed(900);
			} else {//Son plus fort à droite qu'à gauche
				set_led(LED3, 1);
				left_motor_set_speed(900);
				right_motor_set_speed(900*max_norm[0]/max_norm[1]);
			}
		} else { //Sound from back
			set_led(LED5, 1);
			if (max_norm[0] > max_norm[1]){//Son plus fort à gauche qu'à droite
				set_led(LED7, 1);
				left_motor_set_speed(-900);
				right_motor_set_speed(900);
			} else {//Son plus fort à droite qu'à gauche
				set_led(LED3, 1);
				left_motor_set_speed(900);
				right_motor_set_speed(-900);
			}
		}
	} else {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void AlignSound(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_Align(micLeft_output, micRight_output, micFront_output, micBack_output);

	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
