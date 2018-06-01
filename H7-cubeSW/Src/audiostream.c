/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"

#define AUDIO_FRAME_SIZE     32
#define HALF_BUFFER_SIZE      AUDIO_FRAME_SIZE * 2 //number of samples per half of the "double-buffer" (twice the audio frame size because there are interleaved samples for both left and right channels)
#define AUDIO_BUFFER_SIZE     AUDIO_FRAME_SIZE * 4 //number of samples in the whole data structure (four times the audio frame size because of stereo and also double-buffering/ping-ponging)

// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS



ALIGN_32BYTES (int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);
ALIGN_32BYTES (int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);


int16_t inBuffer[HALF_BUFFER_SIZE];
int16_t outBuffer[HALF_BUFFER_SIZE];

uint16_t* adcVals;

uint8_t buttonAPressed = 0;

float sample = 0.0f;

float adcx[8];

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

#define NUM_RAMP N_RAMP

tCompressor* compressor;

tRamp* ramp[NUM_RAMP];

tNeuron* neuron;

typedef enum _NeuronParam
{
	TIMESTEP=0,CURRENT,SODIUM_INACT,SODIUM_ACT,POTASSIUM,CAPACITANCE,V1,V3,NUMPARAMS
} NeuronParam;

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, &randomNumber);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;

	neuron = tNeuronInit();

	for (int i = 0; i < NUM_RAMP; i++)
	{
		ramp[i] = tRampInit(12.0f, 1);
	}

	/*
	compressor = tCompressorInit();
	compressor->M = 1.0f;
	compressor->T = 0.0f;
	compressor->tauAttack = 25.0f;
	compressor->tauRelease = 250.0f;
	compressor->R = 12.0f;
	compressor->W = 6.0f;
*/
	compressor = tCompressorInit();
	compressor->M = 0.0f;
	compressor->T = -1.0f;
	compressor->tauAttack = 20.0f;
	compressor->tauRelease = 250.0f;
	compressor->R = 3.0f;
	compressor->W = 6.0f;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
	

}

float tempVal = 0.0f;
uint16_t frameCounter = 0;

void audioFrame(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int16_t current_sample = 0;

	//tRampSetDest(ramp[TIMESTEP], 1.0f / ((adcVals[8]*INV_TWO_TO_16 * 128.0f) * 2.0f + 1.0f));
	//tNeuronSetTimeStep(neuron, 1.0f / (adcVals[8]*INV_TWO_TO_16 * 128.0f * 2.0f + 1.0f));

	/*
	//tNeuronSetTimeStep(neuron, 1.0f / ((float)adcVals[0]*INV_TWO_TO_16 * 128.0f * 2.0f + 1.0f));
	//tRampSetDest(ramp[TIMESTEP], 1.0f / ((adcx[TIMESTEP] * 128.0f) * 2.0f + 1.0f));
	tempVal = (adcx[TIMESTEP] * 128.0f * 2.0f + 1.0f) + (adcVals[8]*INV_TWO_TO_16 * 128.0f * 2.0f + 1.0f);
	if (tempVal < 10.0f)
	{
		tempVal = 10.0f;
	}
	tRampSetDest(ramp[TIMESTEP], 1.0f / tempVal);

	tempVal = (50.0f + ((adcx[CURRENT] + ((float)adcVals[9] * INV_TWO_TO_16)) * 128.0f) * 1.5f);
	tRampSetDest(ramp[CURRENT], tempVal);
	//tNeuronSetCurrent(neuron, 50.0f + (adcx[CURRENT] * 128.0f) * 1.5f);

	tRampSetDest(ramp[SODIUM_INACT], -adcx[SODIUM_INACT] + (-1 * ((float)adcVals[10])));
	//tNeuronSetL(neuron, -adcx[SODIUM_INACT]);
	//tRampSetDest(ramp[1], cval*2.0f);

	tRampSetDest(ramp[SODIUM_ACT], 128.0f + ((adcx[SODIUM_ACT] + ((float)adcVals[11] * INV_TWO_TO_16)) * 128.0f)  * 3.0f);
	//tNeuronSetN(neuron, 128.0f + (adcx[SODIUM_ACT] * 128.0f)  * 3.0f);

	tRampSetDest(ramp[POTASSIUM], adcx[POTASSIUM] * 80.0f - 20.0f);
	//tNeuronSetK(neuron, adcx[POTASSIUM] * 80.0f - 20.0f);

	tRampSetDest(ramp[CAPACITANCE], adcx[CAPACITANCE] * 2.0f + 0.01f);
	//tNeuronSetC(neuron, adcx[CAPACITANCE] * 2.0f + 0.01);

	tRampSetDest(ramp[V1], (adcx[V1] * 128.0f)*2.0f - 128.0f);
	//tNeuronSetV1(neuron, (adcx[V1] * 128.0f)*2.0f - 128.0f);

	tRampSetDest(ramp[V3], (adcx[V3] * 128.0f)*2.0f - 128.0f);
	//tNeuronSetV3(neuron, (adcx[V3] * 128.0f)*2.0f - 128.0f);


	 */
	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
		buttonCheck();
	}


	tempVal = (adcx[TIMESTEP] * 128.0f * 2.0f + 1.0f) + (adcVals[8]*INV_TWO_TO_16 * 128.0f * 2.0f + 1.0f);
	if (tempVal < 10.0f)
	{
		tempVal = 10.0f;
	}
	tRampSetDest(ramp[TIMESTEP], 1.0f / tempVal);
	tempVal = (50.0f + ((adcx[CURRENT] + ((float)adcVals[9] * INV_TWO_TO_16)) * 128.0f) * 1.5f);
	tRampSetDest(ramp[CURRENT], tempVal);
	tempVal = (adcx[SODIUM_INACT] + ((float)adcVals[10] * INV_TWO_TO_16));
	tRampSetDest(ramp[SODIUM_INACT], -1.0f * tempVal);
	tempVal = adcx[SODIUM_ACT] + ((float)adcVals[11] * INV_TWO_TO_16);
	tRampSetDest(ramp[SODIUM_ACT], 128.0f + (tempVal * 128.0f)  * 3.0f);
	tRampSetDest(ramp[POTASSIUM], adcx[POTASSIUM] * 80.0f - 20.0f);
	tRampSetDest(ramp[CAPACITANCE], adcx[CAPACITANCE] * 2.0f + 0.01f);
	tRampSetDest(ramp[V1], (adcx[V1] * 128.0f)*2.0f - 128.0f);
	tRampSetDest(ramp[V3], (adcx[V3] * 128.0f)*2.0f - 128.0f);
	for (int i = 0; i < NUMPARAMS; i++)
	{
		//dropping the resolution of the knobs to allow for stable positions (by making the ADC only 8 bit)
		adcx[i] = adcVals[i] / 256 * INV_TWO_TO_8;
	}
	
	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0) {
			current_sample = (int16_t)(audioTickL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		}
		else
		{
			//current_sample = (int16_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		}
		audioOutBuffer[buffer_offset + i] = current_sample;
	}
}

float currentFreq = 1.0f;



float audioTickL(float audioIn) 
{
	    tNeuronSetTimeStep(neuron, tRampTick(ramp[TIMESTEP]));

		tNeuronSetCurrent(neuron, tRampTick(ramp[CURRENT]));

		tNeuronSetL(neuron, tRampTick(ramp[SODIUM_INACT]));

		tNeuronSetN(neuron, tRampTick(ramp[SODIUM_ACT]));

		tNeuronSetK(neuron, tRampTick(ramp[POTASSIUM]));

		tNeuronSetC(neuron, tRampTick(ramp[CAPACITANCE]));

		tNeuronSetV1(neuron, tRampTick(ramp[V1]));

		tNeuronSetV3(neuron, tRampTick(ramp[V3]));

/*
	tNeuronSetCurrent(neuron, 0.5f);

	tNeuronSetL(neuron, 0.5f);

	tNeuronSetN(neuron,0.5f);

	tNeuronSetK(neuron, 0.5f);

	tNeuronSetC(neuron, 0.5f);

	tNeuronSetV1(neuron, 0.5f);

	tNeuronSetV3(neuron, 0.5f);
*/
	sample =  tNeuronTick(neuron);
	if (isnan(sample))
	{
		sample = 0.0f;
	}
	sample = tCompressorTick(compressor, sample);
	if (isnan(sample))
	{
		sample = 0.0f;
	}
	sample *= 0.95f;
	return sample;
}

float audioTickR(float audioIn) 
{
	return sample;
}

void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

	for (int i = 0; i < 2; i++)
	{
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 40))
	  {
		  buttonCounters[i]++;
	  }
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 40))
	  {
		  if (buttonValues[i] == 1)
		  {
			  buttonPressed[i] = 1;
		  }
		  buttonValuesPrev[i] = buttonValues[i];
		  buttonCounters[i] = 0;
	  }
	}

	if (buttonPressed[0] == 1)
	{

			neuron->mode++;
			neuron->voltage = 0.0f;
			if (neuron->mode > 2)
			{
				neuron->mode = 0;
			}
			if (neuron->mode == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else if (neuron->mode == 1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else if (neuron->mode == 2)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}
			buttonPressed[0] = 0;
	}
	if (buttonPressed[1] == 1)
	{

			neuron->filterPlacement++;
			neuron->voltage = 0.0f;
			if (neuron->filterPlacement > 1)
			{
				neuron->filterPlacement = 0;
			}
			if (neuron->filterPlacement == AfterFeedback)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			}
			else if (neuron->filterPlacement == InFeedback)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			}

			buttonPressed[1] = 0;
	}
}


void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{

	audioFrame(HALF_BUFFER_SIZE);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{

	audioFrame(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);;
}

