/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"
#include "leaf.h"

#define ADCJoyY 0
#define ADCJoyX 1
#define ADCBreath 2
#define ADCPedal 3
#define ADCKnob 4
#define ADCSlide 5


#define NUM_HARMONICS 16.0f

#define ACOUSTIC_DELAY				132 //caluclate new acoustic delay if the plastic tube length changes

#define NUM_BANDPASSES 100


// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS


int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

uint16_t* adcVals;

#define SAMPLE_BUFFER_SIZE 20000
float buffer1[SAMPLE_BUFFER_SIZE] __ATTR_RAM_D2;
float buffer2[SAMPLE_BUFFER_SIZE] __ATTR_RAM_D2;
float sample = 0.0f;

float fundamental_hz = 58.27;
float fundamental_cm;
float fundamental_m = 2.943195469366741f;
float inv_fundamental_m;
float cutoff_offset;
long long click_counter = 0;

tSVF bandPasses[NUM_BANDPASSES];

tDelayL delay;

uint16_t Distance;

#define CROSSFADE_SAMPLES 32


const int SYSTEM_DELAY = 2*AUDIO_FRAME_SIZE + ACOUSTIC_DELAY;

float slideLengthPreRamp;

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

float LN2;
float amp_mult = 1.5f;

float valPerM;
float mPerVal;


tSawtooth osc;
tRamp adc[6];
tRamp slideRamp;
tRamp finalFreqRamp;

tSVF filter1;
tSVF filter2;

tCycle sine;

tRamp qRamp;

tRamp correctionRamp;
tDelayL correctionDelay;

tCompressor compressor;

tHighpass breathMicHP;
tEnvelopeFollower follower;
tCrusher crush;

tHighpass dcBlock;

tFBleveller leveller;
tPwrFollow follow;

tSimpleLivingString string;


float breath_baseline = 0.0f;
float breath_mult = 0.0f;
float testVal = 0.0f;
uint16_t knobValue;

uint16_t slideValue;

float slideLengthDiff = 0;
float slideLength = 0;

float fundamental = 0.0f;
float customFundamental = 48.9994294977f;
float position = 0.f;
float firstPositionValue = 0.f;


float harmonicHysteresis = 0.6f;


float knobValueToUse = 0.0f;

float floatHarmonic, floatPeak, intPeak, mix;
float intHarmonic;

int flip = 1;
int envTimeout;


FTMode ftMode = FTFeedback;
float val;

float breath = 0.0f;
float rampedBreath = 0.0f;

float slide_tune = 1.0f;

int hysteresisKnob = 0;
int hysteresisAmount = 512;

int octave = 3;
float octaveTransp[7] = { 0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f};

float slidePositions[8] = {0.0f, 0.09f, 0.17f, 0.27f, 0.34f, 0.39f, 0.47f, 0.7f};
float slideLengthM = 0.0f;
float slideLengthChange = 0.0f;
float oldSlideLengthM = 0.0f;
float newTestDelay = 0.0f;
float oldIntHarmonic = 0.0f;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

float audioTickSynth(float audioIn);
float audioTickFeedback(float audioIn);
float audioTickSynthL(float audioIn);
float audioTickSynthR(float audioIn);
float audioTickFeedbackL(float audioIn);
float audioTickFeedbackR(float audioIn);
void metering(float audioIn);
static void calculatePeaks(void);
static int additionalDelay(float Tfreq);

//ADC values are =
// [0] = joystick x
// [1] = breath
// [2] = open
// [3] = joy Y
// [4] = pedal
// [5] = knob



void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library.
	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, &randomNumber);

	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;

	tCycle_init(&sine);
	tCycle_setFreq(&sine, 220.0f);

	tSawtooth_init(&osc);
	tSawtooth_setFreq(&osc, 200.f);

	tHighpass_init(&breathMicHP, 500.0f);
	tFBleveller_init(&leveller, 0.0f, 0.01f, 0.01f, 1);
	tPwrFollow_init(&follow, .06f);
	tSimpleLivingString_init(&string, 440.0f, 9000.0f, 100.0f, 0.0f, 0.01f, 0.2f, 1);

	for (int i = 0; i < NUM_BANDPASSES; i++)
	{
		tSVF_init(&bandPasses[i], SVFTypeBandpass, ((randomNumber() + 1.0f) * 5000.0f) + 40.0f, 500.0f);
	}

	tDelayL_init(&delay, 20000.0f, 40000.0f);
	tDelayL_setDelay (&delay, 20000.0f);
	tRamp_init(&correctionRamp, 10, 1);
	// 16000 was max delay length in OOPS, can change this once we start using this with the fbt
	tDelayL_init(&correctionDelay, 0, 16000);

	tHighpass_init(&dcBlock, 100.0f);
	tRamp_init(&qRamp, 10, 1);

	tRamp_init(&adc[ADCJoyY], 5, 1);
	tRamp_init(&adc[ADCKnob], 20, AUDIO_FRAME_SIZE);
	tRamp_init(&adc[ADCPedal], 20, 1);
	tRamp_init(&adc[ADCBreath], 10, 1);
	tRamp_init(&adc[ADCSlide], 10, AUDIO_FRAME_SIZE);
	tRamp_init(&adc[ADCJoyX], 20, 1);
	/*
	compressor = tCompressorInit();


	compressor->M = 24.0f;
	compressor->W = 24.0f;//24
	compressor->T = -24.0f;//24
	compressor->R = 3.f ; //3
	compressor->tauAttack = 0.0f ;//1
	compressor->tauRelease = 0.0f;//1
	*/

	tEnvelopeFollower_init(&follower, 0.05f, 0.99999f);

	tRamp_init(&slideRamp, 5, 1);
	tRamp_init(&finalFreqRamp, 5, 1);

	breath_baseline = ((adcVals[ADCBreath] * INV_TWO_TO_16) + 0.1f);
	breath_mult = 1.0f / (1.0f-breath_baseline);

	//valPerM = 1430.0f;// / powf(2.0f,SLIDE_BITS);
	//mPerVal = 1.0f/valPerM;

	tCrusher_init(&crush);
	// right shift 4 because our valPerM measure is originally from 12 bit data. now we are using 16 bit adc for controller input, so scaling was all off.
	//firstPositionValue = adcVals[ADCSlide] >> 4;
	tRamp_setVal(&slideRamp, firstPositionValue);

	tSVF_init(&filter1, SVFTypeBandpass, 2000.0f, 1000.0f);

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	//filter2 = tSVFInit(SVFTypeBandpass, 2000.0f, 1000.0f);

}
//prevDelayVal = 0.0f;
uint16_t currentBP = 0;

float gain1;
float gain2;
int crossfade;
float detectorVal;

int detectorCountdown;

int whichBuffer = 0;
int resetPhasor = 0;
int recWriteIndex = 0;
uint32_t loopLength = 1000;
uint32_t phasor = 0;


void audioFrame(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int32_t current_sample = 0;


	tRamp_setDest(&adc[ADCPedal], (adcVals[ADCPedal] * INV_TWO_TO_16));
	tRamp_setDest(&adc[ADCKnob], (adcVals[ADCKnob] * INV_TWO_TO_16));
	tRamp_setDest(&adc[ADCJoyY], 1.0f - ((adcVals[ADCJoyY] * INV_TWO_TO_16) - 0.366f) * 3.816f);
	tRamp_setDest(&adc[ADCJoyX], 1.0f - ((adcVals[ADCJoyX] * INV_TWO_TO_16) - 0.366f) * 3.816f);
	tRamp_setDest(&adc[ADCSlide], (float)Distance);

	position = tRamp_tick(&adc[ADCSlide]);

	//slideLengthDiff = (position - firstPositionValue) * mPerVal * slide_tune;
	//slideLengthM = (position - firstPositionValue) * mPerVal;

	slideLengthPreRamp = fundamental_m + (position * .001f); // mm to M is /1000
	tRamp_setDest(&slideRamp, slideLengthPreRamp);

	if (ftMode == FTFeedback)
	{
		for (i = 0; i < (HALF_BUFFER_SIZE); i++)
		{
			if ((i & 1) == 0)
			{
				current_sample = (int32_t)(audioTickFeedbackR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
			}
			else
			{
				current_sample = (int32_t)(audioTickFeedbackL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
			}

			audioOutBuffer[buffer_offset + i] = current_sample;
		}
	}

	else
	{
		for (i = 0; i < (HALF_BUFFER_SIZE); i++)
		{
			if ((i & 1) == 0)
			{
				current_sample = (int32_t)(audioTickSynthR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
			}
			else
			{
				current_sample = (int32_t)(audioTickSynthL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
			}

			audioOutBuffer[buffer_offset + i] = current_sample;
		}
	}



}

float Lin = 0.0f;
float Rin = 0.0f;

float audioTickFeedbackL(float audioIn)
{
	Lin = audioIn * 1.0f;
	metering(audioIn * 1.0f);
	///float pedal = tRamp_tick(&adc[ADCPedal]);
	//pedal = LEAF_clip(0.0f, pedal - 0.05f, 1.0f);

	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRamp_setDest(&adc[ADCBreath], breath);

	rampedBreath = tRamp_tick(&adc[ADCBreath]);
	//sample = tSawtooth_tick(&osc) * rampedBreath;

	calculatePeaks();


	sample = Lin;

	//tCycle_setFreq(&sine, tRamp_tick(&adc[ADCSlide]) + 50.0f);

	testVal = LEAF_clip(0.0, ((tRamp_tick(&adc[ADCKnob]) * 1.2f) - .1f), 1.0f);


	//sample = audioIn * testVal * 30.0f;
	//sample = tHighpass_tick(&dcBlock, sample * 2.0f);

	tFBleveller_setTargetLevel(&leveller, testVal * rampedBreath);

	sample = LEAF_softClip(sample * testVal * rampedBreath, .6f);
	float myQ = LEAF_clip(0.75f, ((tRamp_tick(&adc[ADCJoyX]) * 10.0f) + 0.5f), 10.0f);
	tSVF_setQ(&filter1, myQ);
	tSVF_setFreq(&filter1, floatPeak);
	sample = tSVF_tick(&filter1, sample);
	sample = tFBleveller_tick(&leveller, sample);



	// Delay correction
		newTestDelay = (float) additionalDelay(intPeak) + 3.0f;//+ (pedal * 16.0f);// + delayValueCorrection(slideLengthM);
		tRamp_setDest(&correctionRamp, newTestDelay);

		tDelayL_setDelay(&correctionDelay, tRamp_tick(&correctionRamp));

		sample = tDelayL_tick(&correctionDelay, sample);


	/*
	sample = 0.0f;

	calculatePeaks();

	tRampSetDest(finalFreqRamp, intPeak);

	knob = tRampTick(adc[ADCKnob]);

	Q = OOPS_clip(0.5f, (knob - 0.1) * 300.0f, 300.0);

	tRampSetDest(qRamp, Q);

	tSVFSetQ(filter1, tRampTick(qRamp));


	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRampSetDest(adc[ADCBreath], breath);

	rampedBreath = tRampTick(adc[ADCBreath]);

	tSVFSetFreq(filter1, tRampTick(finalFreqRamp));

	sample = tSVFTick(filter1, audioIn);

	//sample = tCompressorTick(compressor, sample);

	// Delay correction
	newTestDelay = (float) additionalDelay(intPeak) + (pedal * 256.0f);// + delayValueCorrection(slideLengthM);
	tRampSetDest(correctionRamp, newTestDelay);

	tDelayLSetDelay(correctionDelay, tRampTick(correctionRamp));

	sample = tDelayLTick(correctionDelay, sample);


	sample *= rampedBreath;

	//sample *= pedal;

	//sample = OOPS_clip(-1.0f, sample * 20.0f, 1.0f);
	//sample *= 0.1f;
*/
	//sample = tCycle_tick(&sine) * 0.5f * pedal;
	//sample = audioIn;
	//sample = 0.0f;
	//sample *= 0.9f;
	//sample *= tHighpass_tick(&breathMicHP, Rin * 1.0f);
	//sample = 	tCycle_tick(&sine) * testVal;
		sample *= 1.0f;
		float crushVal = (tRamp_tick(&adc[ADCJoyX]));
		tCrusher_setQuality(&crush, crushVal);
		tCrusher_setRound(&crush, crushVal);
		tCrusher_setSamplingRatio(&crush, crushVal);
		sample = tCrusher_tick(&crush, sample);
		//sample = audioTickSynthL(sample) * 0.5f;
		sample *= rampedBreath;
	sample = 	LEAF_clip(-1.0f, sample, 1.0f);

	return sample;

}

float audioTickFeedbackR(float audioIn)
{
	Rin = audioIn;
	//sample = audioIn;
	//sample = 0.0f;
	return sample;
}

float slideLengthPostRamp;

float audioTickSynthL(float audioIn)
{

	sample = 0.0f;


	float pedal = tRamp_tick(&adc[ADCPedal]);
	pedal = LEAF_clip(0.0f, pedal - 0.05f, 1.0f);

	tSawtooth_setFreq(&osc, 200);
	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRamp_setDest(&adc[ADCBreath], breath);

	rampedBreath = tRamp_tick(&adc[ADCBreath]);

	calculatePeaks();


	//sample = audioIn;

	float myFreq = floatPeak;

	//tSawtooth_setFreq(&osc, myFreq);

	testVal = LEAF_clip(0.0f, ((tRamp_tick(&adc[ADCKnob]) * 2.0f) - .1f), 1.0f);

	//sample = (tSawtooth_tick(&osc) * testVal) * rampedBreath;
	//sample = audioIn * testVal * 30.0f;
	tSimpleLivingString_setTargetLev(&string, rampedBreath);
	tSimpleLivingString_setDampFreq(&string, testVal * 6000.0f + 50.0f);
	tSimpleLivingString_setFreq(&string, myFreq);
	sample = (tSimpleLivingString_tick(&string, Rin * 0.1f));
	float crushVal = (tRamp_tick(&adc[ADCJoyX]));
	tCrusher_setQuality(&crush, crushVal);
	tCrusher_setRound(&crush, crushVal);
	tCrusher_setSamplingRatio(&crush, crushVal);
	sample = tCrusher_tick(&crush, sample);
	sample = tHighpass_tick(&dcBlock, (sample * .9f) + (Rin * 0.1f));
	return sample;
}


float audioTickSynthR(float audioIn)
{
	Rin = audioIn;
	return sample;
}





////////////////////////






//ADC values are =
// [0] = joystick
// [1] = knob
// [2] = pedal
// [3] = breath
// [4] = slide




void metering(float audioIn)
{

	float myLevel = tPwrFollow_tick(&follow, audioIn);
	if (myLevel > .99f)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //led white
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //led white
	}
	if (myLevel > .1f)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //led Green
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //led Green
	}
	if (myLevel > .3f)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //led amber
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //led amber
	}
}


static void calculatePeaks(void)
{
	slideLengthPostRamp = tRamp_tick(&slideRamp);
	float x = 12.0f * logf(slideLengthPostRamp / fundamental_m) * INV_LOG2;
	fundamental = (fundamental_hz * 1.0f) * powf(2.0f, (-x * INV_TWELVE));

	floatHarmonic = tRamp_tick(&adc[ADCJoyY]) * 2.0f - 1.0f;
	floatHarmonic = (floatHarmonic < 0.0f) ? 1.0f : (floatHarmonic * NUM_HARMONICS + 1.0f);

	if (((floatHarmonic - intHarmonic) > (harmonicHysteresis)) || ((floatHarmonic - intHarmonic) < ( -1.0f * harmonicHysteresis)))
	{
		intHarmonic = (uint16_t) (floatHarmonic + 0.5f);
	}

	floatPeak = fundamental * floatHarmonic * octaveTransp[octave];
	intPeak = fundamental * intHarmonic * octaveTransp[octave];
}

float delayCor[8][16] = {{0.0f, 0.0f, 43.0f, 56.0f, 48.0f, 32.0f, 18.0f, 26.0f, 23.0f, 14.0f, 11.0f, 5.0f, 1.0f, 0.0f, 0.0f, 7.0f},
													{0.0f, 0.0f, 24.0f, 39.0f, 41.0f, 31.0f, 17.0f, 1.0f, 104.0f, 100.0f, 93.0f, 74.0f, 4.0f, 66.0f, 3.0f, 63.0f},
													{0.0f, 0.0f, 56.0f, 82.0f, 69.0f, 39.0f, 29.0f, 23.0f, 121.0f, 97.0f, 179.0f, 80.0f, -5.0f, 76.0f, -4.0f, 0.0f},
													{0.0f, 0.0f, 1.0f, 18.0f, 38.0f, 20.0f, 9.0f, 123.0f, 109.0f, -1.0f, 93.0f, 69.0f, 65.0f, 140.0f, 1.0f, 67.0f},
													{0.0f, 0.0f, 6.0f, 37.0f, 43.0f, 30.0f, 10.0f, 4.0f, 117.0f, 99.0f, 93.0f, 56.0f, 74.0f, 54.0f, 1.0f, 132.0f},
													{0.0f, 0.0f, 159.0f, 133.0f, 117.0f, 95.0f, 56.0f, 151.0f, 120.0f, 108.0f, 104.0f, 103.0f, 103.0f, 22.0f, 87.0f, 78.0f},
													{0.0f, 0.0f, 141.0f, 119.0f, 104.0f, 92.0f, 215.0f, 162.0f, 125.0f, 111.0f, 108.0f, 104.0f, 104.0f, 98.0f, 95.0f, 80.0f},
													{0.0f, 0.0f, 141.0f, 119.0f, 104.0f, 92.0f, 215.0f, 162.0f, 125.0f, 111.0f, 108.0f, 104.0f, 104.0f, 98.0f, 95.0f, 80.0f}};

//takes index from delayValueCorrection to calculate weight for weighted average and returns the correct delay value (don't call directly)
static float delayValCor(uint8_t slidePosInd, float slidePos){

	if (slidePosInd == 0) return 0.0f;

    float fraction = ((slidePos - slidePositions[slidePosInd-1]) / (slidePositions[slidePosInd] - slidePositions[slidePosInd-1]));

    //determines weighted average of the appropriate two delayCor[][] values based on slide position and harmonic
    return ((delayCor[slidePosInd-1][((uint8_t) intHarmonic) - 1] * (1 - fraction)) + (delayCor[slidePosInd][((uint8_t) intHarmonic) - 1] * fraction));
}

//calculates first index in slidePositions[] that is past the current slide position and passes into delayValCor to return the correct delay value
static float delayValueCorrection(float slidePos){
    uint8_t i = 0;
    for(i; i < 8; i++){
        if(slidePos < slidePositions[i]){
            return delayValCor(i, slidePos);
        }
    }
    return 0.0f;
}

static int additionalDelay(float Tfreq)
{
	int period_in_samples = (SAMPLE_RATE / Tfreq);
	return (period_in_samples - (SYSTEM_DELAY % period_in_samples));
}


float audioTickSynth(float audioIn)
{
/*
	sample = 0.0f;


	calculatePeaks();

	tRamp_setDest(&finalFreqRamp, intPeak);

	float pedal = tRamp_tick(&adc[ADCPedal]);

	knobValueToUse = tRamp_tick(&adc[ADCKnob]);
	float theCrushParam = tRamp_tick(&adc[ADCJoyX]);

	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRamp_setDest(&adc[ADCBreath], breath);

	rampedBreath = tRamp_tick(&adc[ADCBreath]);


	tSawtooth_setFreq(&osc, tRamp_tick(&finalFreqRamp));

	sample = tSawtooth_tick(&osc);

	//sample *= pedal;

	sample *= rampedBreath;

	sample *= 2.0f;
	sample = LEAF_shaper(sample, 1.6f);

	tCrusher_setQuality(&crush,theCrushParam);

	tCrusher_setSamplingRatio(&crush,theCrushParam);
	sample = tCrusher_tick(&crush, sample);
*/
	//sample *= 0.1;


	//sample = audioIn;
	//sample *= LEAF_clip(0.0f,(tRamp_tick(&adc[ADCKnob]) - .1f) ,1.0f);
	//tCycle_setFreq(&sine, 440.0f);
	//tCycle_setFreq(&sine, tRamp_tick(&adc[ADCSlide]) + 50.0f);
	testVal = ((tRamp_tick(&adc[ADCKnob]) * 2.0f) - .1f);
	//sample = tCycle_tick(&sine) * LEAF_clip(0.0f,(testVal - .1f) ,1.0f) ;

	sample = audioIn * testVal;
	return sample;
}






float knob, Q;
float audioTickFeedback(float audioIn)
{
/*
	float pedal = tRamp_tick(&adc[ADCPedal]);
	pedal = LEAF_clip(0.0f, pedal - 0.05f, 1.0f);

	tSawtooth_setFreq(&osc, 200);
	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRamp_setDest(&adc[ADCBreath], breath);

	rampedBreath = tRamp_tick(&adc[ADCBreath]);
	sample = tSawtooth_tick(&osc) * rampedBreath;
*/
	/*
	sample = 0.0f;

	calculatePeaks();

	tRampSetDest(finalFreqRamp, intPeak);

	knob = tRampTick(adc[ADCKnob]);

	Q = OOPS_clip(0.5f, (knob - 0.1) * 300.0f, 300.0);

	tRampSetDest(qRamp, Q);

	tSVFSetQ(filter1, tRampTick(qRamp));


	breath = adcVals[ADCBreath];
	breath = breath * INV_TWO_TO_16;
	breath = breath - breath_baseline;
	breath = breath * breath_mult;
	breath *= amp_mult;

	if (breath < 0.0f)					breath = 0.0f;
	else if (breath > 1.0f)		  breath = 1.0f;

	tRampSetDest(adc[ADCBreath], breath);

	rampedBreath = tRampTick(adc[ADCBreath]);

	tSVFSetFreq(filter1, tRampTick(finalFreqRamp));

	sample = tSVFTick(filter1, audioIn);

	//sample = tCompressorTick(compressor, sample);

	// Delay correction
	newTestDelay = (float) additionalDelay(intPeak) + (pedal * 256.0f);// + delayValueCorrection(slideLengthM);
	tRampSetDest(correctionRamp, newTestDelay);

	tDelayLSetDelay(correctionDelay, tRampTick(correctionRamp));

	sample = tDelayLTick(correctionDelay, sample);


	sample *= rampedBreath;

	//sample *= pedal;

	//sample = OOPS_clip(-1.0f, sample * 20.0f, 1.0f);
	//sample *= 0.1f;
*/
	//sample = tCycle_tick(&sine) * 0.5f * pedal;
	sample = audioIn;

	return sample;

}


void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	;
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
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(0);
}
