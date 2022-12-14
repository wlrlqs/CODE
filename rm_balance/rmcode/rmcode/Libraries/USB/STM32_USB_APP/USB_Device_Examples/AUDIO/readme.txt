/**
  @page USBD_AUDIO  USB Device Audio device example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief   Description of the USB Device Audio device example
  ******************************************************************************
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
  * limitations under the License
  *   
  ******************************************************************************
   @endverbatim

   
@par Example Description 

The Audio device example allows device to communicate with host (PC) as USB Speaker
using isochronous pipe for audio data transfer along with some control commands (i.e.
Mute). 
It follows the "Universal Serial Bus Device Class Definition for Audio Devices
Release 1.0 March 18, 1998" defined by the USB Implementers Forum for reprogramming
an application through USB-FS-Device. 
Following this specification, it is possible to manage only Full Speed USB mode 
(High Speed is not supported). 

This class is natively supported by most Operating Systems (no need for specific
driver setup).
  
This example uses the I2S interface to stream audio data from USB Host to the audio
codec implemented on the evaluation board. Then audio stream is output to the 
Headphone or on-board Speaker depending on the user selection:
  - On STM322xG-EVAL, STM324xG-EVAL or STM324x9I-EVAL, the Headphone is selected as output by default. 
    When pushing Key button the output is switched to Speaker (or to Headphone if the
    current output is the Speaker, No speaker is availabe on STM324x9I-EVAL board).
  - On STM3210C-EVAL, the default state is Automatic detection; when the Headphone
    is plugged on it is used as output, and when it is unplugged, output automatically
    switches to Speaker. When pushing Key button, the automatic detection is disabled
    and only the output set by the Key command is configured (Headphone or Speaker).
	For this board, it possible to use one of the two quartz below:
    - 14.7456MHz which provides best audio quality
    - Standard 25MHz which provides lesser quality. When this quartz is used:
	   - If audio frequency is set to 16Khz (define "USBD_AUDIO_FREQ" in usbd_conf.h) 
	    then the quality on the headphone is better than the quality on the speaker.
	   - If audio frequency is set to 48KHz then the quality on the speaker is better
	   but quality on headphone is degraded.
	   This is due to limited range of input frequencies on the CS43L22 codec combined
	   with lack of precision when 25MHz quartz is used to generate audio frequency.

The device supports one audio frequency (the host driver manages the sampling rate
conversion from original audio file sampling rate to the sampling rate supported
by the device). It is possible to configure this audio frequency by modifying the
usbd_conf.h file (define USBD_AUDIO_FREQ). It is advised to set high frequencies
to guarantee a high audio quality.
It is also possible to modify the default volume through define DEFAULT_VOLUME in file
usbd_conf.h.

@note The Key push button allows to switch (on the fly) between Headphone and Speaker
      outputs.

@note The audio frequencies leading to non integer number of data (44.1KHz, 22.05KHz, 
      11.025KHz...) will not allow an optimum audio quality since one data will be lost
      every two/more frames.

@note This example doesn't provide Synchronization mechanisms allowing to overcome the
      difference between USB clock domain and STM32 clock domain. For more details about 
	  the complete solution, please contact your local ST sales representative.

	  
This example works
- in full speed (FS) when the STM322xG-EVAL, the STM324xG-EVAL or the STM324x9I-EVAL board and the
  USB OTG FS peripheral are used, or when using the STM3210C-EVAL board.
    

@par Hardware and Software environment 

   - This example runs on STM32F107xx Connectivity line, STM32F207xx ,STM32F429xx and STM32F407xx devices.
  
   - This example has been tested with STM3210C-EVAL RevB (STM32F107xx devices), 
     STM322xG-EVAL RevB (STM32F207xx), STM324xG-EVAL RevB (STM32F407xx) and STM324x9I-EVAL RevB (STM32F429xx)    

  - STM3210C-EVAL Set-up 
    - Use CN2 connector to connect the board to a PC host

  - STM322xG-EVAL Set-up
    - Use CN8 connector to connect the board to a PC host when using USB OTG FS peripheral
		 
  - STM324xG-EVAL Set-up
    - Use CN8 connector to connect the board to a PC host when using USB OTG FS peripheral 
  - STM324x9I-EVAL Set-up
    - Use CN14 connector to connect the board to a PC host when using USB OTG FS peripheral 
        Please ensure that jumper JP16 is not fitted.

@par How to use it ?

In order to make the program work, you must do the following:
  - Open your preferred toolchain
  - In the workspace toolbar select the project config:
  
    - STM322xG-EVAL_USBD-FS: to configure the project for STM32F207xx devices and use USB OTG FS peripheral

    - STM324xG-EVAL_USBD-FS: to configure the project for STM32F407xx devices and use USB OTG FS peripheral

    - STM324x9i-EVAL_USBD-FS:to configure the project for STM32F429xx devices and use USB OTG FS peripheral

    - STM3210C-EVAL_USBD-FS_25MHz: to configure the project for STM32F107xx devices with 25 MHz external quartz
    - STM3210C-EVAL_USBD-FS_14745600Hz: to configure the project for STM32F107xx devices with 14.7456 MHz external quartz    

@note Known Limitations
      This example retarget the C library printf() function to the EVAL board?s LCD
      screen (C library I/O redirected to LCD) to display some Library and user debug
      messages. TrueSTUDIO Lite version does not support I/O redirection, and instead
      have do-nothing stubs compiled into the C runtime library. 
      As consequence, when using this toolchain no debug messages will be displayed
      on the LCD (only some control messages in green will be displayed in bottom of
      the LCD). To use printf() with TrueSTUDIO Professional version, just include the
      TrueSTUDIO Minimal System calls file "syscalls.c" provided within the toolchain.
      It contains additional code to support printf() redirection.
    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
