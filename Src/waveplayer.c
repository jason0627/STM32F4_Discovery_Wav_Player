/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @brief   I2S Audio player program. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "waveplayer.h"
#include "stm32f4_discovery_audio.h"
#include "fatfs.h"
#include "usb_host.h"
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define AUDIO_BUFFER_SIZE             4096

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;

/* Audio Play Start variable. 
   Defined as external in main.c*/
__IO uint32_t AudioPlayStart = 0;

/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;

/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 60;


/* Variable used to indicate audio mode (play, record or stop). */
/* Defined in main.c */
extern __IO uint32_t CmdIndex;

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

char* MusicList[5] = {"001.wav", "002.wav", "003.wav", "004.wav", "005.wav"};
static uint8_t MusicIndex = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* @brief  Volume control function by Blue Button of STM32F4-Discovery Board. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == UserButton_Pin)
  {
    if( Volume > 70 ) Volume = 60;
    else Volume += 2;
    BSP_AUDIO_OUT_SetVolume(Volume);
  }
}

/**
  * @brief  Starts Wave player.
  * @param  None
  * @retval None
  */
void WavePlayerStart(void)
{
  UINT bytesread = 0;
  char path[] = "0:/";
  char* wavefilename = NULL;
  WAVE_FormatTypeDef waveformat;

   /* Get the read out protection status */
   if(f_opendir(&Directory, path) == FR_OK)
   {
        wavefilename = MusicList[MusicIndex];

	/* Open the Wave file to be played */
	if(f_open(&FileRead, wavefilename , FA_READ) == FR_OK)
	{
	  /* Read sizeof(WaveFormat) from the selected file */
	  f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);

	  /* Set WaveDataLenght to the Speech Wave length */
	  WaveDataLength = waveformat.FileSize;

	  /* Play the Wave */
	  WavePlayBack(waveformat.SampleRate);
	}
   }
}

/**
  * @brief  Plays Wave from a mass storage.
  * @param  AudioFreq: Audio Sampling Frequency
  * @retval None
*/
void WavePlayBack(uint32_t AudioFreq)
{ 
  UINT bytesread = 0;
  
  /* Initialize Wave player (Codec, DMA, I2C) */
  if(WavePlayerInit(AudioFreq) != 0)
  {
    Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = WaveDataLength - bytesread;
  
//  BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
  /* Start playing Wave */
  BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);

  /* Check if the device is connected.*/
  while((AudioRemSize != 0))
  { 
      bytesread = 0;
      
      if(buffer_offset == BUFFER_OFFSET_HALF)
      {
        f_read(&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE/2, (void *)&bytesread);
        buffer_offset = BUFFER_OFFSET_NONE;
      }
      
      if(buffer_offset == BUFFER_OFFSET_FULL)
      {
        f_read(&FileRead, &Audio_Buffer[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/2, (void *)&bytesread);
        buffer_offset = BUFFER_OFFSET_NONE;
      } 
      if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2))
      {
        AudioRemSize -= bytesread;
      }
      else
      {
        AudioRemSize = 0;
//        BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
        WavePlayerStop();
//        BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
        /* Close file */
        f_close(&FileRead);
        if(MusicIndex++ >= 4) MusicIndex = 0;
        else MusicIndex++;
        break;
      }
  }
}

/**
  * @brief  Pauses or Resumes a played Wave.
  * @param  state: Player state: Pause, Resume or Idle
  * @retval None
  */
  // ==> This function was deleted by Jason (2018-07-26)
  //     Please make your own function if you want to use PAUSE/RESUME.

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}
 
/**
* @brief  Initializes the Wave player.
* @param  AudioFreq: Audio sampling frequency
* @retval None
*/
int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* MEMS Accelerometer configure to manage PAUSE, RESUME operations */
//  BSP_ACCELERO_Click_ITConfig();

  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  return(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq));  
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
  
  /* Could also generate a system reset to recover from the error */
  /* .... */
}



/**
  * @brief Wave player.
  * @param  None
  * @retval None
  */


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
