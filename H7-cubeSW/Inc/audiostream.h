/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/waveplayer.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for waveplayer.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */   
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIOSTREAM_H
#define __AUDIOSTREAM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "leaf.h"

#define AUDIO_FRAME_SIZE     64
#define HALF_BUFFER_SIZE      AUDIO_FRAME_SIZE * 2 //number of samples per half of the "double-buffer" (twice the audio frame size because there are interleaved samples for both left and right channels)
#define AUDIO_BUFFER_SIZE     AUDIO_FRAME_SIZE * 4 //number of samples in the whole data structure (four times the audio frame size because of stereo and also double-buffering/ping-ponging)


typedef enum FTMode
{
	FTFeedback = 0,
	FTSynthesisOne,
	FTModeNil,
	FTModeCount = FTModeNil
} FTMode;

typedef enum KnobMode
{
	SlideTune = 0,
	MasterTune = 1,
	OctaveTune = 2,
	DelayTune = 3,
	KnobModeNil = 4,
	KnobModeCount = 5
} KnobMode;

typedef enum HarmonicMode
{
	CaraMode = 0,
	RajeevMode,
	JennyMode,
	HarmonicModeNil,
	HarmonicModeCount = HarmonicModeNil
} HarmonicMode;

extern HarmonicMode hMode;

extern int octave;
extern float position;
extern float firstPositionValue;
extern uint16_t knobValue;
extern float knobValueToUse;
extern uint16_t slideValue;
extern float testVal;
extern FTMode ftMode;
extern KnobMode kMode;
extern tRamp adc[6];

extern uint16_t Distance;

extern float intHarmonic;
extern float floatHarmonic;

extern float fundamental;
extern float customFundamental;

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;

extern float fundamental_hz;
extern float fundamental_cm;
extern float fundamental_m;
extern float inv_fundamental_m;
extern float cutoff_offset;
extern float intPeak;
extern float floatPeak;
extern float testDelay;
extern float slide_tune;
extern float slideLengthM;
extern float intPeak;
extern float rampedBreath;
extern float slideLengthPreRamp;
extern float slideLengthPostRamp;
extern float Q;
extern float dist;
extern int footSwitch1;
extern int footSwitch2;

extern float valPerM;
extern float mPerVal;

#define SLIDE_BITS 16

#define SAMPLE_RATE 48000.0f
#define INV_SAMPLE_RATE 1.f/SAMPLE_RATE 
#define SAMPLE_RATE_MS (SAMPLE_RATE / 1000.f)
#define INV_SR_MS 1.f/SAMPLE_RATE_MS
#define SAMPLE_RATE_DIV_PARAMS SAMPLE_RATE / 3
#define SAMPLE_RATE_DIV_PARAMS_MS (SAMPLE_RATE_DIV_PARAMS / 1000.f)
#define INV_SR_DIV_PARAMS_MS 1.f/SAMPLE_RATE_DIV_PARAMS_MS


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, uint16_t* myADCArray);

void audioFrame(uint16_t buffer_offset);

void DMA1_TransferCpltCallback(DMA_HandleTypeDef *hdma);
void DMA1_HalfTransferCpltCallback(DMA_HandleTypeDef *hdma);
#endif /* __AUDIOSTREAM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
