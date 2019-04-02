/*! \file  SPI_DDS_AD9958.h
    \brief Header for SPI DDS (Direct Digital Synthesis) interface.
    \author Anthony Marshall
    
    AD9958 Dual DDS Implemented.
*/

#ifndef _SPI_DDS_AD9958_H
#define _SPI_DDS_AD9958_H

#include <stdint.h>
#include <stdbool.h>

/*! typedef enum E_DDS_PWR
    \brief DDS Power modes.
*/
typedef enum
{
  E_DDS_PWR_POWERED, /*!< Fully powered */
  E_DDS_PWR_PARTIAL, /*!< Fast recovery   - Powers down digital logic and DAC digital logic */
  E_DDS_PWR_DOWN     /*!< Full power down - Powers down all functions including DAC and PLL */
} E_DDS_PWR;

/*! typedef enum E_DDS_CHANNEL
    \brief DDS Channel.
*/
typedef enum
{
  E_DDS_CHANNEL_BOTH, /*!< Select both channels */
  E_DDS_CHANNEL_0,    /*!< Select channel 0 only */
  E_DDS_CHANNEL_1     /*!< Select channel 1 only */
} E_DDS_CHANNEL;

// Pin defines - Change to suit
#define AD9958_PWR_DWN_CTL_PORT     GPIOA
#define AD9958_PWR_DWN_CTL_PIN      GPIO_PIN_1
#define AD9958_MASTER_RESET_PORT    GPIOA
#define AD9958_MASTER_RESET_PIN     GPIO_PIN_1
#define AD9958_IO_UPDATE_PORT       GPIOA
#define AD9958_IO_UPDATE_PIN        GPIO_PIN_1
#define AD9958_SDIO_3_PORT          GPIOA
#define AD9958_SDIO_3_PIN           GPIO_PIN_1
#define AD9958_CS_GPIO_PORT         GPIOA
#define AD9958_CS_PIN               GPIO_PIN_1

void AD9958_Initialise_SPI(void);
void AD9958_Initialise_GPIO(void);
void AD9958_TestFunctions(void);

/*! void AD9958_SetFrequency(E_DDS_CHANNEL channel, uint32_t frequency)
    \brief Set the frequency output on a given channel of the DDS.
    
    \param channel Which channel to output on.
    \param frequency Desired output frequency in Hz.
*/
void AD9958_SetFrequency(E_DDS_CHANNEL channel, uint32_t frequency);

/*! void AD9958_SetAmplitude(E_DDS_CHANNEL channel, uint32_t amplitude)
    \brief Set the amplitude scale factor on a given channel of the DDS.
    
    \param channel Which channel to output on.
    \param amplitude Desired amplitude scale factor (0-1023).
*/
void AD9958_SetAmplitude(E_DDS_CHANNEL channel, uint32_t amplitude);

/*! void DDS_Set_Phase_Offset(E_DDS_CHANNEL channel, float phase)
    \brief Set the phase offset on a given channel of the DDS.
    
    \param channel Which channel to output on.
    \param phase Desired phase offset.
*/
void AD9958_SetPhaseOffset(E_DDS_CHANNEL channel, float phase);

/*! void DDS_Set_Power_State(E_DDS_PWR state)
    \brief Set the power state of the DDS.
    
    \param state Desired power state.
*/
void AD9958_SetPowerState(E_DDS_PWR state);


#endif
