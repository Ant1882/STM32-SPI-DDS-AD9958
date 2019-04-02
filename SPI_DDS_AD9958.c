/*! \file SPI_DDS_AD9958.c
    \brief Source for SPI DDS (Direct Digital Synthesis) interface - STM32 Based
    \author Anthony Marshall.
    
    AD9958 Dual DDS Implemented.
*/

#include "SPI_DDS_AD9958.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"

#define REF_CLK         25000000.0      //!< Xtal used
#define TUNEWORD_BITS   4294967296.0    //!< 2^32
#define PHASE_OFF_BITS  16384.0         //!< 2^14
#define MAX_DEGREES     360.0           //!< For phase calculation

// Registers not currently being masked (no state kept in code)
// Register addresses, writing to (MSB is 0 for a write)
#define CSR     0x00            //!< Channel select register            1 Byte
#define FR1     0x01            //!< Function register 1                3 Bytes
#define CFTW0   0x04            //!< Channel Frequency Tuning Word      4 Bytes
#define CPOW    0x05            //!< Channel Phase Offset Word          2 Bytes
#define ACR     0x06            //!< Amplitude Control Register         3 Bytes

// Register bit patterns
#define CH0_ENABLE      0x70    //!< Only ch0 registers are written to
#define CH1_ENABLE      0xB0    //!< Only ch1 registers are written to
#define BOTH_ENABLE     0xF0    //!< Both channel registers are written to
#define MAN_AMP         0x10    //!< Enable manual amplitude control
#define PWR_DWN_FULL    0x40    //!< Select full power down mode
#define PWR_DWN_PART    0x00    //!< Select partial power down mode
#define SYNC_DISABLE    0x20    //!< Disable the SYNC_CLK signal


// Internal functions
static uint8_t AD9958_GetChannelBits(E_DDS_CHANNEL channel);

static void AD9958_IO_Update_Toggle();
static void AD9958_Reset(void);
static void AD9958_Disable_SYNC_CLK(void);

static void AD9958_SetFrequency_SPI(uint8_t channel, uint32_t frequency);
static void AD9958_SetAmplitude_SPI(uint8_t channel, uint32_t amplitude);
static void AD9958_SetPhaseOffset_SPI(uint8_t channel, float phase);
static void AD9958_SetPowerDown_partial_SPI();
static void AD9958_SetPowerDown_full_SPI();

// Public Functions --------------------------------------------------

// Setup DDS control lines - Replace with your own setup, default pin states given
void AD9958_Initialise_GPIO(void)
{
  /*
  dds_pwr_dwn_ctl  = GPIO_Setup (AD9958_PWR_DWN_CTL_PORT,  AD9958_PWR_DWN_CTL_PIN,  MUX_ALT_1, E_GPIO_OUT, E_GPIO_HIGH);
  dds_master_reset = GPIO_Setup (AD9958_MASTER_RESET_PORT, AD9958_MASTER_RESET_PIN, MUX_ALT_1, E_GPIO_OUT, E_GPIO_HIGH);
  dds_io_update    = GPIO_Setup (AD9958_IO_UPDATE_PORT,    AD9958_IO_UPDATE_PIN,    MUX_ALT_1, E_GPIO_OUT, E_GPIO_HIGH);
  dds_sdio_3       = GPIO_Setup (AD9958_SDIO_3_PORT,       AD9958_SDIO_3_PIN,       MUX_ALT_1, E_GPIO_OUT, E_GPIO_HIGH);
  */
}

void AD9958_TestFunctions(void)
{
  AD9958_Initialise_SPI();
  AD9958_SetAmplitude(E_DDS_CHANNEL_0, 512);
  AD9958_SetAmplitude(E_DDS_CHANNEL_1, 512);
  AD9958_SetAmplitude(E_DDS_CHANNEL_BOTH, 512);

  AD9958_SetFrequency(E_DDS_CHANNEL_0, 60000);
  AD9958_SetFrequency(E_DDS_CHANNEL_1, 60000);
  AD9958_SetFrequency(E_DDS_CHANNEL_BOTH, 60000);

  AD9958_SetPhaseOffset(E_DDS_CHANNEL_0, 90.00f);
  AD9958_SetPhaseOffset(E_DDS_CHANNEL_1, 90.00f);
  AD9958_SetPhaseOffset(E_DDS_CHANNEL_BOTH, 90.00f);

  AD9958_SetPowerState(E_DDS_PWR_DOWN);
  AD9958_SetPowerState(E_DDS_PWR_PARTIAL);
  AD9958_SetPowerState(E_DDS_PWR_POWERED);
}

// Select the output frequency of desired channel/s
void AD9958_SetFrequency(E_DDS_CHANNEL channel, uint32_t frequency)
{
  uint8_t Channel = 0;
  
  // Select which register bit pattern we need
  Channel = AD9958_GetChannelBits(channel);
  
  AD9958_SetFrequency_SPI(Channel, frequency);
}

// Set the amplitude scale factor (max 1023)
void AD9958_SetAmplitude(E_DDS_CHANNEL channel, uint32_t amplitude)
{
  uint8_t Channel = 0;
  
  // Select which register bit pattern we need
  Channel = AD9958_GetChannelBits(channel);
  
  AD9958_SetAmplitude_SPI(Channel, amplitude);
}

// Offset the desired channel from the other
void AD9958_SetPhaseOffset(E_DDS_CHANNEL channel, float phase)
{
  uint8_t Channel = 0;
  
  // Select which register bit pattern we need
  Channel = AD9958_GetChannelBits(channel);
  
  AD9958_SetPhaseOffset_SPI(Channel, phase);
}

// DDS Power control
void AD9958_SetPowerState(E_DDS_PWR state)
{
  switch(state)
  {
    case E_DDS_PWR_POWERED:
    {
      // Set PWR-DWN-CTL pin LOW to disable power-down control
      HAL_GPIO_WritePin(AD9958_PWR_DWN_CTL_PORT, AD9958_PWR_DWN_CTL_PIN, GPIO_PIN_RESET);
    } 
    break;
    
    case E_DDS_PWR_PARTIAL:
    {
      AD9958_SetPowerDown_partial_SPI();
      
      // Set PWR-DWN-CTL pin HIGH to enable the selected power-down mode
      HAL_GPIO_WritePin(AD9958_PWR_DWN_CTL_PORT, AD9958_PWR_DWN_CTL_PIN, GPIO_PIN_SET);
    } 
    break;
    
    case E_DDS_PWR_DOWN:
    {
      AD9958_SetPowerDown_full_SPI();
      
      // Set PWR-DWN-CTL pin HIGH to enable the selected power-down mode
      HAL_GPIO_WritePin(AD9958_PWR_DWN_CTL_PORT, AD9958_PWR_DWN_CTL_PIN, GPIO_PIN_SET);
    } 
    break;
  
    default:
      break; 
  }
}

// Private Functions --------------------------------------------------

/*! static uint8_t AD9958_GetChannelBits(E_DDS_CHANNEL channel)
    \brief Convert enum to bit pattern
    
    \param channel Desired DDS channel.
    \return Register bit pattern for a given channel.
*/
static uint8_t AD9958_GetChannelBits(E_DDS_CHANNEL channel)
{
  uint8_t Channel = 0;
   
  switch(channel)
  {
    case E_DDS_CHANNEL_BOTH:
    {
      Channel = BOTH_ENABLE;
    }
    break;
    case E_DDS_CHANNEL_0:
    {
      Channel = CH0_ENABLE;
    }
    break;
    case E_DDS_CHANNEL_1:
    {
      Channel = CH1_ENABLE;
    }
    break;
    default:
      break;
  }
  
  return Channel;
}

/*! static uint32_t AD9958_calcPOW(f32 degrees)
    \brief Calculates the Phase Offset Word. 
    
    \param degrees (0-360).
    \return Phase Offset Word.
*/
static uint32_t AD9958_calcPOW(float degrees)
{
  uint32_t pow = 0;
  
  pow = (uint32_t)( degrees * (PHASE_OFF_BITS / MAX_DEGREES) );
  
  return pow;
}

/*! static uint32_t AD9958_calcFTW(uint32_t frequency)
    \brief Calculates the Frequency Tuning Word. 
    
    \param frequency Desired frequency.
    \return Frequency Tuning Word.
*/
static uint32_t AD9958_calcFTW(uint32_t frequency)
{
  uint32_t setPoint = 0;
  
  setPoint = (uint32_t)( frequency / (REF_CLK / TUNEWORD_BITS) );
                    
  return setPoint;
}

/*! static void AD9958_IO_Update_Toggle()
    \brief Toggles I/O Update pin on DDS to force register changes. 
    
    \warning Minimum pulse width of > 1 SYNC_CLK period (~160ns) required.
*/
static void AD9958_IO_Update_Toggle(void)
{
  HAL_GPIO_WritePin(AD9958_IO_UPDATE_PORT, AD9958_IO_UPDATE_PIN, GPIO_PIN_SET);
  // Minimum pulse width needs to be > 1 SYNC_CLK period (~160ns)
  // NEED TO MAKE SURE THIS IS THE CASE FOR YOUR PLATFORM.
  HAL_GPIO_WritePin(AD9958_IO_UPDATE_PORT, AD9958_IO_UPDATE_PIN, GPIO_PIN_RESET);
}

/*! static void AD9958_Reset()
    \brief Reset the DDS. 
    
    \warning Minimum pulse width of > 1 SYNC_CLK period (~160ns) required.
*/
static void AD9958_Reset(void)
{
  // Toggle MASTER_RESET
  HAL_GPIO_WritePin(AD9958_MASTER_RESET_PORT, AD9958_MASTER_RESET_PIN, GPIO_PIN_SET);
  // Minimum pulse width needs to be > 1 SYNC_CLK period (~160ns)
  // NEED TO MAKE SURE THIS IS THE CASE FOR YOUR PLATFORM.
  HAL_GPIO_WritePin(AD9958_MASTER_RESET_PORT, AD9958_MASTER_RESET_PIN, GPIO_PIN_RESET);
  
  // Set PWR-DWN-CTL pin LOW to disable power-down control
  HAL_GPIO_WritePin(AD9958_PWR_DWN_CTL_PORT, AD9958_PWR_DWN_CTL_PIN, GPIO_PIN_RESET);
  
  // sync_io low (To enable SPI reception)
  HAL_GPIO_WritePin(AD9958_SDIO_3_PORT, AD9958_SDIO_3_PIN, GPIO_PIN_RESET);
}

// SPI Functions --------------------------------------------------

/*! static void AD9958_Dev_Select( void )
    \brief Assert chip select for the DDS. 
*/
static void AD9958_Dev_Select( void )
{ 
  HAL_GPIO_WritePin(AD9958_CS_GPIO_PORT, AD9958_CS_PIN, GPIO_PIN_RESET);
}

/*! static void AD9958_Dev_Deselect( void )
    \brief Deassert chip select for the DDS. 
*/
static void AD9958_Dev_Deselect( void )
{
  HAL_GPIO_WritePin(AD9958_CS_GPIO_PORT, AD9958_CS_PIN, GPIO_PIN_SET);
}

void AD9958_Initialise_SPI(void)
{
  // Create control block for SPI and enable bus
  /*SPI_DDS_dev = SPI_setup (SPI_DDS_AD9958,
                           SPI_DDS_AD9958_BAUD,
                           SPI_master,
                           SPI_inactive_low,
                           SPI_ck_leading,
                           SPI_8Bit,
                           SPI_DDS_AD9958_CS_PORT,
                           SPI_DDS_AD9958_CS_PIN);
  */
  
  // Reset & power up the device
  AD9958_Reset();
  
  // Toggle external power down pin
  // This resets the digital logic in the DDS to ensure SPI comms is possible
  AD9958_SetPowerState(E_DDS_PWR_DOWN);
  AD9958_SetPowerState(E_DDS_PWR_POWERED);
  
  // Not using multiple DDS devices so not needed
  AD9958_Disable_SYNC_CLK();
}

// Test disabling SYNC_CLK
static void AD9958_Disable_SYNC_CLK(void)
{
  uint8_t regBuff;
  uint8_t dataBuff;
  
  // Select channel
  //--------------------------------------
  AD9958_Dev_Select();

  // Send instruction byte for Function Register 1 (FR1)
  regBuff = FR1;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data bytes
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = SYNC_DISABLE;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);

  AD9958_Dev_Deselect();
  // ---------------------------------------
   
   // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}

static void AD9958_SetFrequency_SPI(uint8_t channel, uint32_t frequency)
{
  uint8_t regBuff;
  uint8_t dataBuff;
  
  static uint32_t FTW = 0;
  
  static uint8_t ftwBytes[4];
 
  // Get the Frequency Tuning Word
  FTW = AD9958_calcFTW(frequency);
  
  ftwBytes[0] = FTW >> 24;
  ftwBytes[1] = FTW >> 16;
  ftwBytes[2] = FTW >> 8;
  ftwBytes[3] = FTW >> 0;
   
  // Select channel
  //--------------------------------------
  AD9958_Dev_Select();
    
  // Send instruction byte for Channel Select Register (CSR)
  regBuff = CSR;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data byte
  dataBuff = channel;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  
  AD9958_Dev_Deselect();
// ---------------------------------------
//  This section is normally commented out, left in for test purposes    
//  if(channel == BOTH_ENABLE)
//  {
//    // Set the phase to be 0 for each channel - To keep in sync
//    //--------------------------------------
//    AD9958_Dev_Select();
//    
//    // Send instruction byte for Channel Phase Offset Word Register (CPOW)
//    regBuff = CPOW;
//    HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
//    // Send data bytes to select phase offset to 0
//    dataBuff = 0x00;
//    HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
//    HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
//    
//    AD9958_Dev_Deselect();
//   //--------------------------------------    
//  }
  
  // Request the frequency change
  //--------------------------------------
  AD9958_Dev_Select();
  // Send instruction byte for Channel Frequency Tuning Word Register (CFTW0)
  regBuff = CFTW0;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data bytes of FTW
  HAL_SPI_Transmit(&hspi2, &ftwBytes[0], 1, 100); // MSB
  HAL_SPI_Transmit(&hspi2, &ftwBytes[1], 1, 100);
  HAL_SPI_Transmit(&hspi2, &ftwBytes[2], 1, 100);
  HAL_SPI_Transmit(&hspi2, &ftwBytes[3], 1, 100); // LSB
  
  AD9958_Dev_Deselect();
  // ---------------------------------------
  
  // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}

static void AD9958_SetAmplitude_SPI(uint8_t channel, uint32_t amplitude)
{
  uint8_t regBuff;
  uint8_t dataBuff;
  
  static uint8_t ampBytes[2];
  
  ampBytes[0] = amplitude >> 8;
  ampBytes[1] = amplitude >> 0;
  
  // Select channel
  //--------------------------------------
  AD9958_Dev_Select();
  
  // Send instruction byte for Channel Select Register (CSR)
  regBuff = CSR;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data byte
  dataBuff = channel;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
   
  AD9958_Dev_Deselect();
  //--------------------------------------
  
  // Enable manual Amplitude control (needed... see pg 28 AD9958 datasheet)
  // Manual mode allows the user to directly control the output amplitude by manually writing to the amplitude scale factor value in the ACR (Register 0x06). 
  // Manual mode is enabled by setting ACR[12] = 1 and ACR[11] = 0.

  //--------------------------------------
  AD9958_Dev_Select();
  // Send instruction byte for Amplitude Control Register (ACR)
  regBuff = ACR;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data bytes to select manual amplitude control
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = MAN_AMP;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  
  AD9958_Dev_Deselect();
  //--------------------------------------
  
  // Request the Amplitude change  
  //--------------------------------------
  AD9958_Dev_Select();
  // Send instruction byte for Amplitude Control Register (ACR)
  regBuff = ACR;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data bytes to select amplitude
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = ampBytes[0] | MAN_AMP;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = ampBytes[1];
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
     
  AD9958_Dev_Deselect();
  //--------------------------------------
  
  // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}

static void AD9958_SetPhaseOffset_SPI(uint8_t channel, float phase)
{
  uint8_t regBuff;
  uint8_t dataBuff;
  
  static uint32_t POW = 0;
  static uint8_t powBytes[2];
    
  // Get the Phase Offset Word
  POW = AD9958_calcPOW(phase);
  
  powBytes[0] = POW >> 8;
  powBytes[1] = POW >> 0;
  
  // Select Channel
  //--------------------------------------
  AD9958_Dev_Select();
  
  // Send instruction byte for Channel Select Register (CSR)
  regBuff = CSR;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data byte
  dataBuff = channel;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
   
  AD9958_Dev_Deselect();
  //--------------------------------------
  
  // Request phase offset change
  //--------------------------------------
  AD9958_Dev_Select();
  
  // Send instruction byte for Channel Phase Offset Word Register (CPOW)
  regBuff = CPOW;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Send data bytes to select phase offset
  HAL_SPI_Transmit(&hspi2, &powBytes[0], 1, 100);
  HAL_SPI_Transmit(&hspi2, &powBytes[1], 1, 100);
  
  AD9958_Dev_Deselect();
  //-------------------------------------- 
    
   // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}

static void AD9958_SetPowerDown_partial_SPI(void)
{  
  uint8_t regBuff;
  uint8_t dataBuff;
  
  // Select partial power down mode
  //--------------------------------------
  AD9958_Dev_Select();
  // Send instruction byte for Function Register 1 (FR1)
  regBuff = FR1;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Clear bit [6] of FR1 to select Fast recovery power down mode
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = PWR_DWN_PART | SYNC_DISABLE;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  
  AD9958_Dev_Deselect();
  //--------------------------------------
    
  // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}

static void AD9958_SetPowerDown_full_SPI(void)
{
  uint8_t regBuff;
  uint8_t dataBuff;
  
  // Select full power down mode
  //--------------------------------------
  AD9958_Dev_Select();
  // Send instruction byte for Function Register 1 (FR1)
  regBuff = FR1;
  HAL_SPI_Transmit(&hspi2, &regBuff, 1, 100);
  // Clear bit [6] of FR1 to select full power down mode
  dataBuff = 0x00;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  dataBuff = PWR_DWN_FULL | SYNC_DISABLE;
  HAL_SPI_Transmit(&hspi2, &dataBuff, 1, 100);
  
  AD9958_Dev_Deselect();
  //--------------------------------------
    
  // Transfer data from buffers to active registers in the DDS
  AD9958_IO_Update_Toggle();
}
