/*****************************************************************************
******************************* RoboClawFS.h ******************************
*****************************************************************************/

/*
   Interface for the IonMC Robo Claw Motor Controllers using the
   Packet Serial Communication Mode.

   This driver is intended to work on any platform where the USB port can be
   accessed via standard file system calls [open(), read(), write()].
   Initial testing was done on Windows 8.1 with IonMC's driver installed which
   makes the link to the RoboClaw show up as one of the COMx ports.

   The Boost endian library is used to automatically deal with the RoboClaw
   using big endian integers and whatever the host processor happens to use.
   Little endian on Intel and ARM.

   One of the features is that I've made the order of the function parameters
   more consistent and regular in terms of how (at least I think of them).
   For example, I think speed then acceleration instead of acceleration and then
   speed.  The PID constant functions aren't consistent in the order they're called
   out so that has been made consistent using PID as the natural order.

   Author: Tim Craig (Druai Robotics) 2017
*/

#if !defined(__ROBOCLAWFS_H__)
#define __ROBOCLAWFS_H__

/*****************************************************************************
******************************  I N C L U D E  *******************************
*****************************************************************************/

#include <cstdint>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <boost/endian/conversion.hpp>

/*****************************************************************************
**************************** class DRoboClawFS ****************************
*****************************************************************************/
// Uncomment the following line to debug the RoboClaw command responses
//#define DEBUG_RESPONSE

// Comment out the following line to prevent string name functions from being
// implemented.  Save space on systems with limited memory.
#define IMPLEMENT_STRING_FUNTIONS

class DRoboClawFS
   {
   public :
      using CRC16 = uint16_t;

      // Direction values used by several commands
      enum EDirection : uint8_t { eFwd, eRev };

      // Get a readable string for the direction
      static std::string GetDirectionString(EDirection eDirection);

      // Current Unit Status Bit Flags
      // Returned by Read Status (Command 90)
      enum class EStatus : uint16_t
         {
         eNormal = 0x0000,
         eM1OverCurrentWarning = 0x0001,
         eM2OverCurrentWarning = 0x0002,
         eEStop = 0x0004,
         eTempError = 0x0008,
         eTemp2Error = 0x0010,
         eMainBatteryHighError = 0x0020,
         eLogicBatteryHighError = 0x0040,
         eLogicBatteryLowError = 0x0080,
         eM1DriveFault = 0x0100,
         eM2DriveFault = 0x0200,
         eMainBatteryHighWarning = 0x0400,
         eMainBatteryLowWarning = 0x0800,
         eTempWarning = 0x1000,
         eTemp2Warning = 0x2000,
         eM1Home = 0x4000,
         eM2Home = 0x8000,
         };

      // Return a readable string for the status
      static std::string GetStatusString(EStatus eStatus);

      std::string GetStatusString() const
         {
         EStatus eStatus;
         std::string strStatus;
         bool bRet = ReadStatus(eStatus);
         if (bRet)
            {
            strStatus = GetStatusString(eStatus);
            } // end if

         return(strStatus);
         }

      // Commands 98 & 99
      enum EStdConfigSettings
         {
         eRCMode = 0x0000,
         eAnalog = 0x0001,
         eSimpleSerial = 0x0002,
         ePacketSerial = 0x0003,
         eBatteryModeOff = 0x0000,
         eBatteryModeAuto = 0x0004,
         eBatteryMode2Cell = 0x0008,
         eBatteryMode3Cell = 0x000C,
         eBatteryMode4Cell = 0x0010,
         eBatteryMode5Cell = 0x0014,
         eBatteryMode6Cell = 0x0018,
         eBatteryMode7Cell = 0x001C,
         eMixing = 0x0020,
         eExponential = 0x0040,
         eMCU = 0x0080,
         eBaudRate2400 = 0x0000,
         eBaudRate9600 = 0x0002,
         eBaudRate19200 = 0x0004,
         eBaudRate38400 = 0x0006,
         eBaudRate57600 = 0x000,
         eBaudRate115200 = 0x00A0,
         eBaudRate230400 = 0x00C0,
         eBaudRate460800 = 0x00E0,
         eFlipSwitch = 0x0100,
         ePackAddr0x80 = 0x0000,
         ePackAddr0x81 = 0x0100,
         ePackAddr0x82 = 0x0200,
         ePackAddr0x83 = 0x0300,
         ePackAddr0x84 = 0x0400,
         ePackAddr0x85 = 0x0500,
         ePackAddr0x86 = 0x0600,
         ePackAddr0x87 = 0x0700,
         eSlaveMode = 0x0800,
         eRelayMode = 0x1000,
         eSwapEncoders = 0x2000,
         eSwapButtons = 0x4000,
         eMultiUnitMode = 0x8000,
         };

      // CTRL1 and CTRL2 output pins (Commands 100 & 101)
      enum ECTRLMode : uint8_t
         {
         eDisable, eUser, eVoltageClamp, eBrake
         };

      // PWM Mode (Commands 148 & 149)
      enum EPWMMode : uint8_t { eLockedAntiphase, eSignMagnitude };

      // Return a readable string for the PWM Mode
      static std::string GetPWMModeString(EPWMMode ePWMMode);

      // Used with Commands 91, 92, & 93.  Strongly types the Encoder Modes
      class EncoderMode
         {
         public:
            enum EType { eQuadrature = 0, eAbsolute = 1 };

            bool m_bRC_AnalogEnabed;
            EType m_eType;

            EncoderMode(bool bRC_AEnabled = false, EType eType = eQuadrature)
                  : m_bRC_AnalogEnabed(bRC_AEnabled), m_eType(eType)
               {
               return;
               }

            ~EncoderMode() = default;

            void Decode(uint8_t uMode)
               {
               m_bRC_AnalogEnabed = ((uMode & eEnableRC_AnalogEncoder) != 0);
               m_eType = ((uMode & eType) != 0) ? eAbsolute : eQuadrature;
               return;
               }

            uint8_t Encode() const
               {
               uint8_t uMode = m_bRC_AnalogEnabed ? eEnableRC_AnalogEncoder : 0;
               uMode |= m_eType;
               return (uMode);
               }

         protected:
            enum EEncoderModeBitsMask : uint8_t
               {
               eEnableRC_AnalogEncoder = 0x80,
               eType = 0x01, // Quadrature(0)/Absolute(1)
               };
         };

      enum class ES3Mode : uint8_t
         {
         eDefault = 0, eEStopLatching = 1, eEStop = 2, eVoltageClamp = 3,
         };

      // Get a readable string for the S3 Mode
      static std::string GetS3ModeString(ES3Mode eMode);

      enum class ES4Mode : uint8_t
         {
         eDisabled = 0, eEStopLatching = 1, eEStop = 2, eVoltageClamp = 3, eM1Home = 4,
         };

      // Get a readable string for the S4 Mode
      static std::string GetS4ModeString(ES4Mode eMode);

      enum class ES5Mode : uint8_t
         {
         eDisabled = 0, eEStopLatching = 1, eEStop = 2, eVoltageClamp = 3, eM2Home = 4,
         };

      // Get a readable string for the S5 Mode
      static std::string GetS5ModeString(ES5Mode eMode);

      // Encapsulate the encoder status
      class EncoderStatus
         {
         public:
            bool m_bUnderflow;
            bool m_bOverflow;
            EDirection m_eDirection;

            EncoderStatus()
               {
               m_bUnderflow = false;
               m_bOverflow = false;
               m_eDirection = eFwd;
               return;
               }

            void DecodeByte(uint8_t uStatus)
               {
               m_bUnderflow = ((uStatus & eEncoderUnderflow) != 0);
               m_bOverflow = ((uStatus & eEncoderOverflow) != 0);
               m_eDirection = ((uStatus & eEncoderDirection)) ? eRev : eFwd;
               return;
               }
         };

      enum EEncoderStatusMask : uint8_t
         {
         eEncoderUnderflow = 0x01,
         eEncoderDirection = 0x02,
         eEncoderOverflow = 0x04
         };

      DRoboClawFS(const std::string strTTYDevice = "/dev/roboclaw", uint8_t uAddr = 0x80,
            uint32_t uReadTimeout = 100);
      DRoboClawFS(const DRoboClawFS& src) = delete;
      DRoboClawFS(const DRoboClawFS&& src) = delete;

      virtual ~DRoboClawFS();

      DRoboClawFS& operator=(const DRoboClawFS& rhs) = delete;
      DRoboClawFS& operator=(const DRoboClawFS&& rhs) = delete;

      // RoboClaw address handling

      void SetAddr(uint8_t nAddr)
         {
         m_nAddr = nAddr;
         return;
         }

      uint8_t GetAddr() const
         {
         return (m_nAddr);
         }

      // Device handling

      void SetDevice(std::string strTTYDevice)
         {
         m_strTTYDevice = strTTYDevice;
         return;
         }

      std::string GetDevice() const
         {
         return (m_strTTYDevice);
         }

      // Read timeout
      uint32_t GetReadTimeout() const
         {
         return (m_uReadTimeout);
         }

      void SetReadTimeout(uint32_t uTimeout)
         {
         m_uReadTimeout = uTimeout;
         return;
         }

      // Port handling

      bool OpenPort();
      void ClosePort()
         {
         if (m_nPort != -1)
            {
            close(m_nPort);
            m_nPort = -1;
            } // end if

         return;
         }

      bool IsPortOpen() const
         {
         return (m_nPort >= 0);
         }

      // Flush the receive buffer
      void FlushBuffer()
         {
         uint8_t uByte;
         while (read(m_nPort, &uByte, 1) == 1);

         return;
         }

      /***********************************************************************
      ************************** RoboClaw Commands ***************************
      ***********************************************************************/

      /***********************************************************************
      ********************* Compatibility Mode Commands **********************
      ***********************************************************************/

      //**  Cmd 0  (0 <= nSpeed <= 127)
      // Verified TTC
      bool M1Forward(uint8_t nSpeed)
         {
         return (SendCompatibilityCmd(ECommandCode::eM1Forward, nSpeed));
         }

      //**  Cmd 1  (0 <= nSpeed <= 127)
      bool M1Backward(uint8_t nSpeed)
         {
         return (SendCompatibilityCmd(ECommandCode::eM1Backward, nSpeed));
         }

      //** Cmd 2 Sets the minimum allowable main battery voltage
      // 0 <= nV <= 120 (6V-30V)  nV = 5.0 * (Volts - 6.0)
      //!!  Deprecated.  Used Command 57
      bool SetMinMainVoltage(uint8_t nVolts) const
         {
         return (SendCompatibilityCmd(ECommandCode::eSetMinMainVolt, nVolts));
         }

      // Convenience function taking actual voltage as a float (6V-30V)
      bool SetMinMainVoltage(float fVolts) const
         {
         return (SendCompatibilityCmd(ECommandCode::eSetMaxMainVolt,
            static_cast<uint8_t>((fVolts - 6.0f) * 5.0f)));
         }

      //** Cmd 3 Sets the maximum allowable main battery voltage
      // 0 <= nV <= 154 (0V-30V)  nV = Volts * 5.12
      //!! Deprecated.  Used Command 57
      bool SetMaxMainVoltage(uint8_t nV) const
         {
         return (SendCompatibilityCmd(ECommandCode::eSetMaxMainVolt, nV));
         }

      // Convenience function taking actual voltage as a float (0V-30V)
      bool SetMaxMainVoltage(float fVolts) const
         {
         return (SetMaxMainVoltage(static_cast<uint8_t>(fVolts * 5.12f)));
         }

      //** Cmd 4  (0 <= nSpeed <= 127)
      bool M2Forward(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eM2Forward, nSpeed));
         }

      //** Cmd 5  (0 <= nSpeed <= 127)
      bool M2Backward(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eM2Backward, nSpeed));
         }

      //** Cmd 6  (0 [Full Reverse] <= nSpeed <= 127 [Full Forward])
      bool M1FDrive7Bit(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eM1Drive7Bit, nSpeed));
         }

      //** Cmd 7  (0 [Full Reverse] <= nSpeed <= 127 [Full Forward])
      bool M2FDrive7Bit(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eM2Drive7Bit, nSpeed));
         }

      /***********************************************************************
      ****************** Mixed Mode Compatibility Commands *******************
      ***********************************************************************/

      /* The following commands are mix mode compatibility commands used to
       * control speed and turn using differential steering. Before a command
       * is executed valid drive and turn data is required.  You only need to
       * send both data packets once. After receiving both valid drive and turn
       * data RoboClaw will begin to operate the motors. At this point you only
       * need to update turn or drive data as needed.
       */

      //** Cmd 8 (0 <= nSpeed <= 127)
      bool DriveFwd(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedDriveFwd, nSpeed));
         }

      //** Cmd 9  (0 <= nSpeed <= 127)
      bool DriveBck(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedDriveBck, nSpeed));
         }

      //** Cmd 10  (0 <= nSpeed <= 127)
      bool TurnRight(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedTurnRight, nSpeed));
         }

      //** Cmd 11  (0 <= nSpeed <= 127)
      bool TurnLeft(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedTurnLeft, nSpeed));
         }

      //**Cmd 12  (0 [Full Backward] <= nSpeed <= 127 [Full Forward])
      bool DriveMixed7Bit(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedDrive7Bit, nSpeed));
         }

      //**Cmd 13  (0 [Full Backward] <= nSpeed <= 127 [Full Forward])
      bool TurnMixed7Bit(uint8_t nSpeed) const
         {
         return (SendCompatibilityCmd(ECommandCode::eMixedTurn7Bit, nSpeed));
         }

      /***********************************************************************
      *************************** Encoder Commands ***************************
      ***********************************************************************/

      //** Cmd 16 Read Encoder 1
      bool ReadEncoder1(uint32_t& uCount, EncoderStatus& Status) const
         {
         return (ReadEncoder(ECommandCode::eReadEncoder1, uCount, Status));
         }

      //** Cmd 17 Read Encoder 2
      bool ReadEncoder2(uint32_t& uCount, EncoderStatus& Status)
         {
         return (ReadEncoder(ECommandCode::eReadEncoder2, uCount, Status));
         }

      //** Cmd 16 Read Encoder 1
      bool ReadEncoder1(int32_t& nCount, EncoderStatus& Status) const
         {
         return (ReadEncoder(ECommandCode::eReadEncoder1, nCount, Status));
         }

      //** Cmd 17 Read Encoder 2
      bool ReadEncoder2(int32_t& nCount, EncoderStatus& Status) const
         {
         return (ReadEncoder(ECommandCode::eReadEncoder2, nCount, Status));
         }

      //** Cmd 18 Read Encoder 1 Speed
      // Value is in counts/second.
      //Manual says filtered so I assume that means a running average
      // The direction parameter seemed to imply the speed is unsigned.  In
      // practice it is NOT.  Need to treat it as signed.  Then why direction?
      // Verified TTC
      bool ReadEncoder1Speed(int32_t& nSpeed, EDirection& eDirection) const
         {
         return (ReadEncoderSpeed(ECommandCode::eReadEncoder1Speed,
            nSpeed, eDirection));
         }

      // Since Speed is signed, no real need for direction
      bool ReadEncoder1Speed(int32_t& nSpeed) const
         {
         EDirection eDirection;
         return (ReadEncoder1Speed(nSpeed, eDirection));
         }

      //** Cmd 19 Read Encoder 2 Speed
      // Value is in counts/second.
      //Manual says filtered so I assume that means a running average
      bool ReadEncoder2Speed(int32_t& nSpeed, EDirection& eDirection) const
         {
         return (ReadEncoderSpeed(ECommandCode::eReadEncoder2Speed,
            nSpeed, eDirection));
         }

      // Since Speed is signed, no real need for direction
      bool ReadEncoder2Speed(int32_t& nSpeed) const
         {
         EDirection eDirection;
         return (ReadEncoder2Speed(nSpeed, eDirection));
         }

      //** Cmd 20 Reset the Encoder Counters
      // Verified TTC
      bool ResetEncoders() const
         {
         return (SendHeaderCRC0xFF(ECommandCode::eResetEncoders));
         }

      //** Cmd 22 Set Encoder 1 Value
      // Verified TTC
      bool SetEncoder1(uint32_t uCounts) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetEncoder1, uCounts));
         }

      bool SetEncoder1(int32_t nCounts) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetEncoder1, nCounts));
         }

      //** Cmd 23 Set Encoder 2 Value
      // Verified TTC
      bool SetEncoder2(uint32_t uCounts) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetEncoder2, uCounts));
         }

      bool SetEncoder2(int32_t nCounts) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetEncoder2, nCounts));
         }

      //** Cmd 30 Read Encoder 1 Raw Speed
      // Encoder counts per second based on the last 1/300th second
      // The direction parameter seemed to imply the speed is unsigned.  In
      // practice it is NOT.  Need to treat it as signed.  Then why direction?
      // Verified TTC
      bool ReadEncoder1RawSpeed(int32_t& nSpeed, EDirection& eDirection) const
         {
         return (ReadEncoderRawSpeed(ECommandCode::eReadEncoder1RawSpeed, nSpeed, eDirection));
         }

      // Since Speed is signed, no real need for direction
      bool ReadEncoder1RawSpeed(int32_t& nSpeed) const
         {
         EDirection eDirection;
         return (ReadEncoder1RawSpeed(nSpeed, eDirection));
         }

      //** Cmd 31 Read Encoder 2 Raw Speed
      // Encoder counts per second based on the last 1/300th second
      // The direction parameter seemed to imply the speed is unsigned.  In
      // practice it is NOT.  Need to treat it as signed.  Then why direction?
      bool ReadEncoder2RawSpeed(int32_t& nSpeed, EDirection& eDirection) const
         {
         return (ReadEncoderRawSpeed(ECommandCode::eReadEncoder2RawSpeed, nSpeed, eDirection));
         }

      // Since Speed is signed, no real need for direction
      bool ReadEncoder2RawSpeed(int32_t& nSpeed) const
         {
         EDirection eDirection;
         return (ReadEncoder2RawSpeed(nSpeed, eDirection));
         }

      //** Cmd 78 Read Both Encoders
      // Verified TTC
      bool ReadEncodersCounts(uint32_t& uCounts1, uint32_t& uCounts2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadEncodersCounts, uCounts1, uCounts2));
         }

      // Read Both Encoders with counts as signed values
      // Good for controlling position where it's centered around zero.
      bool ReadEncodersCounts(int32_t& nCounts1, int32_t& nCounts2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadEncodersCounts, nCounts1, nCounts2));
         }

      //** Cmd 79 Read Both Encoder Speeds (Counts/Second based on last 1/300th second)
      bool ReadEncodersSpeeds(int32_t& nSpeed1, int32_t& nSpeed2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadEncodersSpeeds, nSpeed1, nSpeed2));
         }

      /***********************************************************************
      ************************ Advanced Packet Serial ************************
      ***********************************************************************/

      //** Cmd 21 Read the firmware version (Max 48 characters)
      // Verified TTC
      bool ReadFirmwareVersion(std::string& strVersion) const;

      //** Cmd 24 Read the main battery voltage in tenths of a volt
      // Verified TTC
      bool ReadMainBatteryVoltage(uint16_t& uVolts) const
         {
         return (Read1IntegralTypeCRC(ECommandCode::eReadMainBatteryVoltage, uVolts));
         }

      // Get the main battery voltage in volts
      bool ReadMainBatteryVoltage(float& fVolts) const
         {
         uint16_t uVolts;
         bool bRet = ReadMainBatteryVoltage(uVolts);
         if (bRet)
            {
            fVolts = uVolts * 0.1f;
            } // end if

         return (bRet);
         }

      //** Cmd 25 Read the logic battery voltage in tenths of a volt
      // Verified TTC
      bool ReadLogicBatteryVoltage(uint16_t& uVolts) const
         {
         return (Read1IntegralTypeCRC(ECommandCode::eReadLogicLogicBatteryVoltage, uVolts));
         }

      // Get the logic battery voltage in volts
      bool ReadLogicBatteryVoltage(float& fVolts) const
         {
         uint16_t uVolts;
         bool bRet = ReadLogicBatteryVoltage(uVolts);
         if (bRet)
            {
            fVolts = uVolts * 0.1f;
            } // end if

         return (bRet);
         }

      //** Cmd 26 Set the Minimum Logic Battery Voltage
      // Valid range 0-140 in 0.2V increments. [0=6V, 140 = 34V]
      //  Deprecated!!  Use Cmd 58
      bool SetMinLogicBatteryVoltage(uint8_t uVolts) const
         {
         return (SendHeaderByteCRC0xFF(ECommandCode::eSetMinLogicVolt, uVolts));
         }

      // Set Minimum Logic Batter Voltage in volts (6.0V-34.0V)
      bool SetMinLogicBatteryVoltage(float fVolts) const
         {
         return (SetMinLogicBatteryVoltage(static_cast<uint8_t>((fVolts - 6.0f) * 5.0f)));
         }

      //** Cmd 27 Set the Maximum Logic Battery Voltage
      // Valid range 30-175. Value = 5.12 * Voltage.
      //  Deprecated!!  Use Cmd 58
      bool SetMaxLogicBatteryVoltage(uint8_t uVolts) const
         {
         return (SendHeaderByteCRC0xFF(ECommandCode::eSetMinLogicVolt, uVolts));
         }

      // Set Maximum Logic Batter Voltage in volts (6.0V-34.0V)
      bool SetMaxLogicBatteryVoltage(float fVolts) const
         {
         return (SetMaxLogicBatteryVoltage(static_cast<uint8_t>(fVolts * 5.12f)));
         }

      //** Cmd 48 Read Both Motor PWM Values.  Values +-32,767.
      // Percent duty cycle divide by 327.67
      // Verified TTC
      bool ReadMotorPWMValues(int16_t& nPWM1, int16_t& nPWM2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadMotorPWMs, nPWM1, nPWM2));
         }

      // Read Motor PWM Values in percent
      bool ReadMotorPWMValues(float& fPWM1, float& fPWM2) const
         {
         int16_t nPWM1, nPWM2;
         bool bRet = ReadMotorPWMValues(nPWM1, nPWM2);
         if (bRet)
            {
            fPWM1 = nPWM1 / 327.67f;
            fPWM2 = nPWM2 / 327.67f;
            } // end if

         return (bRet);
         }

      //** Cmd 49 Read Both Motor Currents
      // Values are in 10ma increments (divide by 100 get get amps)
      // Verified TTC
      bool ReadMotorCurrents(uint16_t& uCur1, uint16_t& uCur2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadMotorCurrents, uCur1, uCur2));
         }

      // Read the motor currents in Amps.
      bool ReadMotorCurrents(float& fCur1, float& fCur2) const
         {
         uint16_t uCur1, uCur2;
         bool bRet = ReadMotorCurrents(uCur1, uCur2);
         if (bRet)
            {
            fCur1 = uCur1 / 100.0f;
            fCur2 = uCur2 / 100.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 57 Set Main Battery Voltages
      // Set the Min and Max Main Battery Voltages in 10ths of a volt
      // Verified TTC
      bool SetMainBatteryVoltageMinMax(uint16_t uMin, uint16_t uMax) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eSetMainBatteryVoltages, uMin, uMax));
         }

      // Set the Main Batter Voltages in Volts
      bool SetMainBatteryVoltageMinMax(float fMin, float fMax) const
         {
         return (SetMainBatteryVoltageMinMax(static_cast<uint16_t>(fMin * 10.0f),
               static_cast<uint16_t>(fMax * 10.0f)));
         }

      //** Cmd 58 Set Logic Battery Voltages
      // Set the Min and Max Logic Battery Voltages in 10ths of a volt
      // Verified TTC
      bool SetLogicBatteryVoltageMinMax(uint16_t uMin, uint16_t uMax) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eSetLogicBatteryVoltages, uMin, uMax));
         }

      // Set the Logic Batter Voltages in Volts
      bool SetLogicBatteryVoltageMinMax(float fMin, float fMax) const
         {
         return (SetLogicBatteryVoltageMinMax(static_cast<uint16_t>(fMin * 10.0f),
               static_cast<uint16_t>(fMax * 10.0f)));
         }

      //** Cmd 59 Read Main Battery Voltage Settings
      // Read the Min and Max Main Battery Voltage Settings in 10th of a volt
      // Verified TTC
      bool ReadMainBatteryVoltageMinMax(uint16_t& uMin, uint16_t& uMax) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadMainBatteryVoltageSettings, uMin, uMax));
         }

      // Read the Min and Max Main Battery Voltage Settings in Volts
      bool ReadMainBatteryVoltageMinMax(float& fMin, float& fMax) const
         {
         uint16_t uMin, uMax;
         bool bRet = ReadMainBatteryVoltageMinMax(uMin, uMax);
         if (bRet)
            {
            fMin = uMin / 10.0f;
            fMax = uMax / 10.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 60 Read Logic Battery Voltage Settings
      // Read the Min and Max Main Battery Voltage Settings in 10th of a volt
      // Verified TTC
      bool ReadLogicBatteryVoltageMinMax(uint16_t& uMin, uint16_t& uMax) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadLogicBatterVoltageSettings, uMin, uMax));
         }

      // Read the Min and Max Logic Battery Voltage Settings in Volts
      bool ReadLogicBatteryVoltageMinMax(float& fMin, float& fMax) const
         {
         uint16_t uMin, uMax;
         bool bRet = ReadLogicBatteryVoltageMinMax(uMin, uMax);
         if (bRet)
            {
            fMin = uMin / 10.0f;
            fMax = uMax / 10.0f;
            } // end if

         return (bRet);
         }

      // Cmd 68 Set M1 Default Duty Acceleration
      // Set default Acceleration for M1 when using duty cycle commands
      // (Cmds 32, 33, & 34) or Standard Serial, RC and Analog PWM modes.
      // Verified TTC
      bool SetM1DefaultDutyAccel(uint32_t uAccel) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetM1DefaultDutyCycleAccel, uAccel));
         }

      // Cmd 69 Set M2 Default Duty Acceleration
      // Set default Acceleration for M2 when using duty cycle commands
      // (Cmds 32, 33, & 34) or Standard Serial, RC and Analog PWM modes.
      // Verified TTC
      bool SetM2DefaultDutyAccel(uint32_t uAccel) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetM2DefaultDutyCycleAccel, uAccel));
         }

      //** Cmd 74 Set S3, S4, S5 Modes
      bool SetS3_S4_S5Modes(ES3Mode eS3, ES4Mode eS4, ES5Mode eS5) const
         {
         return (Send3IntegralTypes0xFF(ECommandCode::eSetS3_S4_S5Modes,
            static_cast<uint8_t>(eS3), static_cast<uint8_t>(eS4),
            static_cast<uint8_t>(eS5)));
         }

      //** Cmd 75 Get S3, S4, S5 Modes
      bool GetS3_S4_S5Modes(ES3Mode& eS3, ES4Mode& eS4, ES5Mode& eS5) const
         {
         uint8_t uS3, uS4, uS5;
         bool bRet = Read3IntegralTypesCRC(ECommandCode::eSetS3_S4_S5Modes, uS3, uS4, uS5);
         if (bRet)
            {
            eS3 = static_cast<ES3Mode>(uS3);
            eS4 = static_cast<ES4Mode>(uS4);
            eS5 = static_cast<ES5Mode>(uS5);
            } // end if

         return (bRet);
         }

      //** Cmd 80 Set Factory Defaults
      bool RestoreFactoryDefaults() const
         {
         return (SendSimpleCommand(ECommandCode::eRestoreDefaults));
         }

      //** Cmd 81 Read Default Duty Acceleration Settings Both Motors
      // Verified TTC
      bool ReadDefaultDutyAccels(uint32_t& uAccel1, uint32_t& uAccel2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadDefaultDutyCycleAccels, uAccel1, uAccel2));
         }

      //** Cmd 82 Read Temperature (10ths of a degree (C or F?)
      // Verified TTC
      bool ReadTemperature(uint16_t& uTemp) const
         {
         return (Read1IntegralTypeCRC(ECommandCode::eReadTemperature, uTemp));
         }

      // Read Temperature in Degrees
      bool ReadTemperature(float& fTemp) const
         {
         uint16_t uTemp;
         bool bRet = ReadTemperature(uTemp);
         if (bRet)
            {
            fTemp = uTemp / 10.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 83 Read Temperature 2 (10ths of a degree (C or F?)
      // Available on supported units only
      // Verified TTC (Not supported on my test unit.  Returned 0)
      bool ReadTemperature2(uint16_t& uTemp) const
         {
         return (Read1IntegralTypeCRC(ECommandCode::eReadTemperature2, uTemp));
         }

      // Read Temperature 2 in Degrees
      bool ReadTemperature2(float& fTemp) const
         {
         uint16_t uTemp;
         bool bRet = ReadTemperature2(uTemp);
         if (bRet)
            {
            fTemp = uTemp / 10.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 90 Read Status
      // Verified TTC
      bool ReadStatus(EStatus& eStatus) const
         {
         uint16_t uStatus;
         bool bRet = Read1IntegralTypeCRC(ECommandCode::eReadStatus, uStatus);
         if (bRet)
            {
            eStatus = static_cast<EStatus>(uStatus);
            } // end if

         return (bRet);
         }

      //** Cmd 91 Read Encoder Mode for both encoders
      // Verified TTC
      bool ReadEncoderMode(EncoderMode& Mode1, EncoderMode& Mode2) const
         {
         uint8_t uMode1, uMode2;
         bool bRet = Read2IntegralTypesCRC(ECommandCode::eReadEncoderModes, uMode1, uMode2);
         if (bRet)
            {
            Mode1.Decode(uMode1);
            Mode2.Decode(uMode2);
            } // end if

         return (bRet);
         }

      //** Cmd 92 Set Motor Encoder 1 Mode
      // Verified TTC
      bool SetEncoder1Mode(EncoderMode Mode) const
         {
         return (SendCompatibilityCmd(ECommandCode::eSetM1EncoderMode, Mode.Encode()));
         }

      //** Cmd 93 Set Motor Encoder 2 Mode
      // Verified TTC
      bool SetEncoder2Mode(EncoderMode Mode) const
         {
         return (SendCompatibilityCmd(ECommandCode::eSetM2EncoderMode, Mode.Encode()));
         }

      //** Cmd 94 Write Settings to EEPROM
      // Preserve settings through power cycles
      bool WriteSettingsToEEPROM() const
         {
         return (SendSimpleCommand(ECommandCode::eWriteSettingsToEEPROM));
         }

      //** Cmd 95 Read Settings from EEPROM
      // Documentation seems off on this one ????
      // I'm going to guess it's a simple command, not what's in the manual (TTC 040117)
      bool ReadSetttingsFromEEPROM() const
         {
         return (SendSimpleCommand(ECommandCode::eReadSettingsFromEEPROM));
         }

      //** Cmd 98 Set Standard Config Settings
      // Manual is more than fuzzy on how to use the mask bits ????
      bool SetStandardConfigSettings(uint16_t uConfig) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetStdConfigSettings, uConfig));
         }

      //** Cmd 99 Read Standard Config Setting
      // ????
      bool ReadStandardConfigSettings(uint16_t& uConfig) const
         {
         return (Read1IntegralTypeCRC(ECommandCode::eReadStdConfigSettings, uConfig));
         }

      //** Cmd 100 SetCtrlMode
      // Set the CTRL Modes for models equipped with the pins
      // Manual if screwed up again in this section ????
      bool SetCTRLModes(ECTRLMode eMode1, ECTRLMode eMode2) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eSetCTRLModes,
            static_cast<uint8_t>(eMode1), static_cast<uint8_t>(eMode2)));
         }

      //** Cmd 101 Read CTRL Modes
      // Read the Modes for the CTRL pins if available
      bool ReadCTRLModes(ECTRLMode& eMode1, ECTRLMode& eMode2) const
         {
         uint8_t uMode1, uMode2;
         bool bRet = Read2IntegralTypesCRC(ECommandCode::eReadCTRLModes, uMode1, uMode2);
         if (bRet)
            {
            eMode1 = static_cast<ECTRLMode>(uMode1);
            eMode2 = static_cast<ECTRLMode>(uMode2);
            } // end if
         return (bRet);
         }

      //** Cmd 102 Set CTRL 1
      bool SetCTRL1(uint16_t uValue) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetCTRL1, uValue));
         }

      //** Cmd 103 Set CTRL 2
      bool SetCTRL2(uint16_t uValue) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetCTRL2, uValue));
         }

      //** Cmd 104 Read CTRL Values
      bool ReadCTRLs(uint16_t& uCTRL1, uint16_t& uCTRL2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadCTRLs, uCTRL1, uCTRL2));
         }

      //** Cmd 133 Set M1 Max Current Limit
      // Current value is in 10ma units.
      // Manual shows 4 zeros for the second parameter.  The Min Current is ignored and aways zero.
      // Verified TTC
      bool SetM1MaxCurrentLimit(uint32_t uMaxCurrent, uint32_t uMinCurrent = 0) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eSetM1MaxCurrentLimit,  uMaxCurrent, uMinCurrent));
         }

      // Set M1 Max Current Limit in Amps
      bool SetM1MaxCurrentLimit(float fMaxCurrent, float fMinCurrent = 0.0f) const
         {
         return (SetM1MaxCurrentLimit(static_cast<uint32_t>(fMaxCurrent * 100.0f),
            static_cast<uint32_t>(fMinCurrent * 100.0f)));
         }

      //** Cmd 134 Set M2 Max Current Limit
      // Current value is in 10ma units.
      // Manual shows 4 zeros for the second parameter.  The Min Current is ignored and aways zero.
      // Verified TTC
      bool SetM2MaxCurrentLimit(uint32_t uMaxCurrent, uint32_t uMinCurrent = 0) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eSetM2MaxCurrentLimit, uMaxCurrent, uMinCurrent));
         }

      // Set M2 Max Current Limit in Amps
      bool SetM2MaxCurrentLimit(float fMaxCurrent, float fMinCurrent = 0.0f) const
         {
         return (SetM2MaxCurrentLimit(static_cast<uint32_t>(fMaxCurrent * 100.0f),
            static_cast<uint32_t>(fMinCurrent * 100.0f)));
         }

      //** Cmd 135 Read M1 Max Current Limit
      // Current value is in 10ma units
      // Min Current is always zero
      // Verified TTC
      bool ReadM1MaxCurrentLimit(uint32_t& uMaxCurrent, uint32_t& uMinCurrent) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadM1MaxCurrentLimit, uMaxCurrent, uMinCurrent));
         }

      // Convenience to prevent handling an unused value
      bool ReadM1MaxCurrentLimit(uint32_t& uMaxCurrent) const
         {
         uint32_t uMinCurrent;
         return (Read2IntegralTypesCRC(ECommandCode::eReadM1MaxCurrentLimit, uMaxCurrent, uMinCurrent));
         }

      // Read M1 Max Current Limit in Amps
      bool ReadM1MaxCurrentLimit(float& fMaxCurrent, float& fMinCurrent) const
         {
         uint32_t uMaxCurrent, uMinCurrent;
         bool bRet = ReadM1MaxCurrentLimit(uMaxCurrent, uMinCurrent);
         if (bRet)
            {
            fMaxCurrent = uMaxCurrent / 100.0f;
            fMinCurrent = uMinCurrent / 100.0f;
            } // end if

         return (bRet);
         }

      // Read M1 Max Current Limit in Amps
      bool ReadM1MaxCurrentLimit(float& fMaxCurrent) const
         {
         uint32_t uMaxCurrent, uMinCurrent;
         bool bRet = ReadM1MaxCurrentLimit(uMaxCurrent, uMinCurrent);
         if (bRet)
            {
            fMaxCurrent = uMaxCurrent / 100.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 136 Read M2 Max Current Limit
      // Current value is in 10ma units
      // Min Current is always zero
      // Verified TTC
      bool ReadM2MaxCurrentLimit(uint32_t& uMaxCurrent, uint32_t& uMinCurrent) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadM2MaxCurrentLimit, uMaxCurrent, uMinCurrent));
         }

      // Convenience to prevent handling an unused value
      bool ReadM2MaxCurrentLimit(uint32_t& uMaxCurrent) const
         {
         uint32_t uMinCurrent;
         return (Read2IntegralTypesCRC(ECommandCode::eReadM2MaxCurrentLimit, uMaxCurrent, uMinCurrent));
         }

      // Read M2 Max Current Limit in Amps
      bool ReadM2MaxCurrentLimit(float& fMaxCurrent, float& fMinCurrent) const
         {
         uint32_t uMaxCurrent, uMinCurrent;
         bool bRet = ReadM2MaxCurrentLimit(uMaxCurrent, uMinCurrent);
         if (bRet)
            {
            fMaxCurrent = uMaxCurrent / 100.0f;
            fMinCurrent = uMinCurrent / 100.0f;
            } // end if

         return (bRet);
         }

      // Read M2 Max Current Limit in Amps
      bool ReadM2MaxCurrentLimit(float& fMaxCurrent) const
         {
         uint32_t uMaxCurrent, uMinCurrent;
         bool bRet = ReadM2MaxCurrentLimit(uMaxCurrent, uMinCurrent);
         if (bRet)
            {
            fMaxCurrent = uMaxCurrent / 100.0f;
            } // end if

         return (bRet);
         }

      //** Cmd 148 Set PWM Mode
      // Locked Antiphase = 0, Sign Magnitude = 1
      // Verified TTC
      bool SetPWMMode(EPWMMode ePWMMode) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eSetPWMMode, static_cast<uint8_t>(ePWMMode)));
         }

      //** Cmd 149 Read PWM Mode
      // Verified TTC
      bool ReadPWMMode(EPWMMode& ePWMMode) const
         {
         uint8_t uPWMMode;
         bool bRet = Read1IntegralTypeCRC(ECommandCode::eReadPWMMode, uPWMMode);
         if (bRet)
            {
            ePWMMode = static_cast<EPWMMode>(uPWMMode);
            } // end if

         return (bRet);
         }

      /***********************************************************************
      ************************ Advanced Motor Control ************************
      ***********************************************************************/

      //** Cmd 28 Set Velocity PID Constants M1
      // uMaxSpeed is the motor speed in QPPS at 100% power
      // Verified TTC
      bool SetM1VelocityPIDConstants(uint32_t uP = 0x0010000, uint32_t uI = 0x00008000,
            uint32_t uD = 0x00004000, uint32_t uMaxSpeed = 44000) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eSetM1VelocityPID, uD, uP, uI, uMaxSpeed));
         }

      //** Cmd 29 Set Velocity PID Constants M2
      // uMaxSpeed is the motor speed in QPPS at 100% power
      // Verified TTC
      bool SetM2VelocityPIDConstants(uint32_t uP = 0x0010000, uint32_t uI = 0x00008000,
            uint32_t uD = 0x00004000, uint32_t uMaxSpeed = 44000) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eSetM2VelocityPID, uD, uP, uI, uMaxSpeed));
         }

      //** Cmd 32 Drive M1 Signed Duty Cycle (No encorders required)
      // Value is -32,767 to +32,767 (-100% to +100%)
      // Verified TTC
      bool DriveM1SignedDutyCycle(int16_t nDutyCycle) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eDriveM1SignedDutyCycle, nDutyCycle));
         }

      // Drive M1 Signed Duty Cycle in Percent (-100% to 100%)
      bool DriveM1SignedDutyCycle(float fDutyCycle) const
         {
         return (DriveM1SignedDutyCycle(static_cast<int16_t>(fDutyCycle * 327.67)));
         }

      //** Cmd 33 Drive M2 Signed Duty Cycle (No encorders required)
      // Value is -32,767 to +32,767 (-100% to +100%)
      bool DriveM2SignedDutyCycle(int16_t nDutyCycle) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eDriveM2SignedDutyCycle, nDutyCycle));
         }

      // Drive M2 Signed Duty Cycle in Percent (-100% to 100%)
      bool DriveM2SignedDutyCycle(float fDutyCycle) const
         {
         return (DriveM2SignedDutyCycle(static_cast<int16_t>(fDutyCycle * 327.67)));
         }

      //** Cmd 34 Drive M1 & M2 Signed Duty Cycle (No encorders required)
      // Value is -32,767 to +32,767 (-100% to +100%)
      bool DriveBothSignedDutyCycle(int16_t nDutyCycle1, int16_t nDutyCycle2) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveBothSignedDutyCycle, nDutyCycle1, nDutyCycle2));
         }

      // Drive M1 & M2 Signed Duty Cycle in Percent (-100% to 100%)
      bool DriveBothSignedDutyCycle(float fDutyCycle1, float fDutyCycle2) const
         {
         return (DriveBothSignedDutyCycle(static_cast<int16_t>(fDutyCycle1 * 327.67),
            static_cast<int16_t>(fDutyCycle2 * 327.67)));
         }

      //** Cmd 35 Drive M1 Signed Speed
      // Speed is in quadrature pulses/second
      bool DriveM1SignedSpeed(int32_t nSpeed) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eDriveM1SignedSpeed, nSpeed));
         }

      //** Cmd 36 Drive M2 Signed Speed
      // Speed is in quadrature pulses/second
      bool DriveM2SignedSpeed(int32_t nSpeed) const
         {
         return (Send1IntegralType0xFF(ECommandCode::eDriveM2SignedSpeed, nSpeed));
         }

      //** Cmd 37 Drive M1 & M2 Signed Speed
      // Speed is in quadrature pulses/second
      bool DriveBothSignedSpeed(int32_t nSpeed1, int32_t nSpeed2) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeed, nSpeed1, nSpeed2));
         }

      //** Cmd 38 Drive M1 Signed Speed & Acceleration
      // Sign on speed indicates the direction, acceleration is not signed.
      bool DriveM1SignedSpeedAccel(int32_t nSpeed, uint32_t uAccel) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveM1SignedSpeedAccel, uAccel, nSpeed));
         }

      //** Cmd 39 Drive M2 Signed Speed & Acceleration
      // Sign on speed indicates the direction, acceleration is not signed.
      bool DriveM2SignedSpeedAccel(int32_t nSpeed, uint32_t uAccel) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveM2SignedSpeedAccel, uAccel, nSpeed));
         }

      //** Cmd 40 Drive M2 Signed Speed & Acceleration
      // Sign on speed indicates the direction, acceleration is not signed.
      // Only 1 Accel value for both motors
      bool DriveBothSignedSpeedAccel(int32_t nSpeed1, int32_t nSpeed2, uint32_t uAccel) const
         {
         return (Send3IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeedAccel,
            uAccel, nSpeed1, nSpeed2));
         }

      //** Cmd 41 Buffered Drive M1 With Signed Speed and Distance
      // Sign on speed indicates direction, distance is unsigned
      bool DriveM1SignedSpeedDistance(int32_t nSpeed, uint32_t uDistance,
            bool bImmediate = true)
         {
         return (Send3IntegralTypes0xFF(ECommandCode::eDriveM1SignedSpeedDistance,
            nSpeed, uDistance, BufferCommand(bImmediate)));
         }

      //** Cmd 42 Buffered Drive M2 With Signed Speed and Distance
      // Sign on speed indicates direction, distance is unsigned
      bool DriveM2SignedSpeedDistance(int32_t nSpeed, uint32_t uDistance,
            bool bImmediate = true) const
         {
         return (Send3IntegralTypes0xFF(ECommandCode::eDriveM2SignedSpeedDistance,
            nSpeed, uDistance, BufferCommand(bImmediate)));
         }

      //** Cmd 43 Buffered Drive M1 & M2 With Signed Speed and Distance
      // Sign on speed indicates direction, distance is unsigned
      bool DriveBothSignedSpeedDistance(int32_t nSpeed1, uint32_t uDistance1,
            int32_t nSpeed2, uint32_t uDistance2, bool bImmediate = true) const
         {
         return (Send5IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeedDistance,
            nSpeed1, uDistance1, nSpeed2, uDistance2, BufferCommand(bImmediate)));
         }

      //** Cmd 44 Buffered Drive M1 With Signed Speed, Accel, & Distance
      bool DriveM1SignedSpeedAccelDistance(int32_t nSpeed, uint32_t uAccel,
            uint32_t uDistance, bool bImmediate = true) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eDriveM1SignedSpeedAccelDistance,
            uAccel, nSpeed, uDistance, BufferCommand(bImmediate)));
         }

      //** Cmd 45 Buffered Drive M2 With Signed Speed, Accel, & Distance
      bool DriveM2SignedSpeedAccelDistance(int32_t nSpeed, uint32_t uAccel,
            uint32_t uDistance, bool bImmediate = true) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eDriveM2SignedSpeedAccelDistance,
            uAccel, nSpeed, uDistance, BufferCommand(bImmediate)));
         }

      //** Cmd 46 Buffered Drive M1 & M2 With Signed Speed, Accel, & Distance
      // Only one Accel value is used for both motors.
      bool DriveBothSignedSpeedAccelDistance(
            int32_t nSpeed1, uint32_t uAccel, uint32_t uDistance1,
            int32_t nSpeed2, uint32_t uDistance2, bool bImmediate = true) const
         {
         return (Send6IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeedAccelDistance,
            uAccel, nSpeed1, uDistance1, nSpeed2, uDistance2, BufferCommand(bImmediate)));
         }

      //** Cmd 47 Read Buffer Length
      // Get the buffer lengths for both motors for the buffered commands (41 - 46)
      // Max buffer length is 64 commands.  Return of 0x80 indicates the buffer is empty.
      // Return of 0 indicates the last command is EXECUTING.  (Screwy)
      bool ReadBufferLength(uint8_t& uLen1, uint8_t& uLen2) const
         {
         return (Read2IntegralTypesCRC(ECommandCode::eReadBufferLength, uLen1, uLen2));
         }

      //** Cmd 50 Drive M1/M2 With Signed Speed And Individual Acceleration
      bool DriveBothSignedSpeedIndividualAccel(int32_t nSpeed1, uint32_t uAccel1,
            int32_t nSpeed2, uint32_t uAccel2) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeedIndividualAccel,
            uAccel1, nSpeed1, uAccel2, nSpeed2));
         }

      //** Cmd 51 Buffered Drive M1/M2 With Signed Speed, Individual Accel And Distance
      bool DriveBothSignedSpeedIndividualAccelDist(int32_t nSpeed1, uint32_t uAccel1, uint32_t uDist1,
            int32_t nSpeed2, uint32_t uAccel2, uint32_t uDist2, bool bImmediate = true) const
         {
         return (Send7IntegralTypes0xFF(ECommandCode::eDriveBothSignedSpeedIndividualAccelDist,
            uAccel1, nSpeed1, uDist1, uAccel2, nSpeed2, uDist2, BufferCommand(bImmediate)));
         }

      //** Cmd 52 Drive M1 With Signed Duty And Acceleration
      // The duty value is signed and the range is -32768 to +32767(eg. +-100% duty).
      // The accel value range is 0 to 65535(eg maximum acceleration rate is -100% to 100% in 100ms).
      // The manual says accel max is 655359.  Typo????
      bool DriveM1SignedDutyAccel(int16_t nDuty, uint16_t uAccel) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveM1SignedDutyAccel, nDuty, uAccel));
         }

      // Duty in percent (-100% to 100%), Accel in percent (0% to 100%)
      bool DriveM1SignedDutyAccel(float fDuty, float fAccel) const
         {
         return (DriveM1SignedDutyAccel(static_cast<int16_t>(fDuty * 32767),
            static_cast<uint16_t>(fAccel * 65535)));
         }

      //** Cmd 53 Drive M2 With Signed Duty And Acceleration
      // The duty value is signed and the range is -32768 to +32767(eg. +-100% duty).
      // The accel value range is 0 to 65535(eg maximum acceleration rate is -100% to 100% in 100ms).
      // The manual says accel max is 655359.  Typo????
      bool DriveM2SignedDutyAccel(int16_t nDuty, uint16_t uAccel) const
         {
         return (Send2IntegralTypes0xFF(ECommandCode::eDriveM2SignedDutyAccel, nDuty, uAccel));
         }

      // Duty in percent (-100% to 100%), Accel in percent (0% to 100%)
      bool DriveM2SignedDutyAccel(float fDuty, float fAccel) const
         {
         return (DriveM2SignedDutyAccel(static_cast<int16_t>(fDuty * 32767),
            static_cast<uint16_t>(fAccel * 65535)));
         }

      //** Cmd 54 Drive M1/M2 With Signed Duty And Acceleration
      // The duty value is signed and the range is -32768 to +32767(eg. +-100% duty).
      // The accel value range is 0 to 65535(eg maximum acceleration rate is -100% to 100% in 100ms).
      // The manual says accel max is 655359.  Typo????
      bool DriveBothSignedDutyAccel(int16_t nDuty1, uint16_t uAccel1, int16_t nDuty2, uint16_t uAccel2) const
         {
         return (Send4IntegralTypes0xFF(ECommandCode::eDriveBothSignedDutyAccel,
            nDuty1, uAccel1, nDuty2, uAccel2));
         }

      // Duty in percent (-100% to 100%), Accel in percent (0% to 100%)
      bool DriveBothSignedDutyAccel(float fDuty1, float fAccel1, float fDuty2, float fAccel2) const
         {
         return (DriveBothSignedDutyAccel(static_cast<int16_t>(fDuty1 * 32767),
            static_cast<uint16_t>(fAccel1 * 65535), static_cast<int16_t>(fDuty2 * 32767),
            static_cast<uint16_t>(fAccel2 * 65535)));
         }

      //** Cmd 55 Read Motor 1 Velocity PID and MaxSpeed Constants
      // uMaxSpeed is the motor speed in QPPS at 100% power
      // Verified TTC
      bool ReadM1VelocityPIDConstants(uint32_t& uP, uint32_t& uI, uint32_t& uD, uint32_t& uMaxSpeed) const
         {
         return (Read4IntegralTypesCRC(ECommandCode::eReadM1VelocityPID,
            uP, uI, uD, uMaxSpeed));
         }

      //** Cmd 56 Read Motor 2 Velocity PID and MaxSpeed Constants
      // uMaxSpeed is the motor speed in QPPS at 100% power
      // Verified TTC
      bool ReadM2VelocityPIDConstants(uint32_t& uP, uint32_t& uI, uint32_t& uD, uint32_t& uMaxSpeed) const
         {
         return (Read4IntegralTypesCRC(ECommandCode::eReadM2VelocityPID,
            uP, uI, uD, uMaxSpeed));
         }

      //** Cmd 61 Set Motor 1 Position PID Constants
      // Verified TTC
      bool SetM1PositionPIDConstants(uint32_t uP, uint32_t uI, uint32_t uD, uint32_t uMaxIWindup,
            uint32_t uDeadzone, int32_t nMinPos, int32_t nMaxPos) const
         {
         return (Send7IntegralTypes0xFF(ECommandCode::eSetM1PositionPID, uD, uP, uI, uMaxIWindup,
            uDeadzone, nMinPos, nMaxPos));
         }

      //** Cmd 62 Set Motor 2 Position PID Constants
      // Verified TTC
      bool SetM2PositionPIDConstants(uint32_t uP, uint32_t uI, uint32_t uD, uint32_t uMaxIWindup,
            uint32_t uDeadzone, int32_t nMinPos, int32_t nMaxPos) const
         {
         return (Send7IntegralTypes0xFF(ECommandCode::eSetM2PositionPID, uD, uP, uI, uMaxIWindup,
            uDeadzone, nMinPos, nMaxPos));
         }

      //** Cmd 63 Read Motor 1 Position PID Constants
      // Verified TTC
      bool ReadM1PositionPIDConstants(uint32_t& uP, uint32_t& uI, uint32_t& uD, uint32_t& uMaxIWindup,
            uint32_t& uDeadzone, int32_t& nMinPos, int32_t& nMaxPos) const
         {
         return (Read7IntegralTypesCRC(ECommandCode::eReadM1PositionPID, uP, uI, uD,
            uMaxIWindup, uDeadzone, nMinPos, nMaxPos));
         }

      //** Cmd 64 Read Motor 2 Position PID Constants
      // Verified TTCs
      bool ReadM2PositionPIDConstants(uint32_t& uP, uint32_t& uI, uint32_t& uD, uint32_t& uMaxIWindup,
            uint32_t& uDeadzone, int32_t& nMinPos, int32_t& nMaxPos) const
         {
         return (Read7IntegralTypesCRC(ECommandCode::eReadM2PositionPID, uP, uI, uD,
            uMaxIWindup, uDeadzone, nMinPos, nMaxPos));
         }

      //** Cmd 65 Buffered Drive M1 with signed Speed, Accel, Deccel and Position
      bool DriveM1SignedSpeedAccelDecelPos(int32_t nSpeed, uint32_t uAccel, uint32_t uDecel,
            int32_t nPos, bool bImmediate = true) const
         {
         return (Send5IntegralTypes0xFF(ECommandCode::eDriveM1SpeedAccelDeccelPos,
            uAccel, nSpeed, uDecel, nPos, BufferCommand(bImmediate)));
         }

      //** Cmd 66 Buffered Drive M2 with signed Speed, Accel, Deccel and Position
      bool DriveM2SignedSpeedAccelDecelPos(int32_t nSpeed, uint32_t uAccel, uint32_t uDecel,
            int32_t nPos, bool bImmediate = true) const
         {
         return (Send5IntegralTypes0xFF(ECommandCode::eDriveM2SpeedAccelDeccelPos,
            uAccel, nSpeed, uDecel, nPos, BufferCommand(bImmediate)));
         }

      //** Cmd 67 Buffered Drive M1/M2 with signed Speed, Accel, Deccel and Position
      bool DriveBothSignedSpeedAccelDecelPos(int32_t nSpeed1, uint32_t uAccel1, uint32_t uDecel1,
            int32_t nPos1, int32_t nSpeed2, uint32_t uAccel2, uint32_t uDecel2,
                                             int32_t nPos2, bool bImmediate = true) const
         {
         return (Send9IntegralTypes0xFF(ECommandCode::eDriveBothSpeedAccelDeccelPos,
            uAccel1, nSpeed1, uDecel1, nPos1, uAccel2, nSpeed2, uDecel2, nPos2,
            BufferCommand(bImmediate)));
         }

   protected :
      // Named values for all the command codes
      enum class ECommandCode : uint8_t
         {
         // Compatibility Mode
         eM1Forward = 0,
         eM1Backward = 1,
         eSetMinMainVolt = 2,
         eSetMaxMainVolt = 3,
         eM2Forward = 4,
         eM2Backward = 5,
         eM1Drive7Bit = 6,
         eM2Drive7Bit = 7,
         // Mixed Mode
         eMixedDriveFwd = 8,
         eMixedDriveBck = 9,
         eMixedTurnRight = 10,
         eMixedTurnLeft = 11,
         eMixedDrive7Bit = 12,
         eMixedTurn7Bit = 13,

         // Encoder Commands
         eReadEncoder1 = 16,
         eReadEncoder2 = 17,
         eReadEncoder1Speed = 18,
         eReadEncoder2Speed = 19,
         eResetEncoders = 20,
         eSetEncoder1 = 22,
         eSetEncoder2 = 23,
         eReadEncoder1RawSpeed = 30,
         eReadEncoder2RawSpeed = 31,

         // Advanced Packet Serial
         eReadVersion = 21,
         eReadMainBatteryVoltage = 24,
         eReadLogicLogicBatteryVoltage = 25,
         eSetMinLogicVolt = 26,
         eSetMaxLogicVolt = 27,

         // Advanced Motor Control
         eSetM1VelocityPID = 28,
         eSetM2VelocityPID = 29,
         eDriveM1SignedDutyCycle = 32,
         eDriveM2SignedDutyCycle = 33,
         eDriveBothSignedDutyCycle = 34,
         eDriveM1SignedSpeed = 35,
         eDriveM2SignedSpeed = 36,
         eDriveBothSignedSpeed = 37,
         eDriveM1SignedSpeedAccel = 38,
         eDriveM2SignedSpeedAccel = 39,
         eDriveBothSignedSpeedAccel = 40,
         eDriveM1SignedSpeedDistance = 41,
         eDriveM2SignedSpeedDistance = 42,
         eDriveBothSignedSpeedDistance = 43,
         eDriveM1SignedSpeedAccelDistance = 44,
         eDriveM2SignedSpeedAccelDistance = 45,
         eDriveBothSignedSpeedAccelDistance = 46,
         eReadBufferLength = 47,
         eDriveBothSignedSpeedIndividualAccel = 50,
         eDriveBothSignedSpeedIndividualAccelDist = 51,
         eDriveM1SignedDutyAccel = 52,
         eDriveM2SignedDutyAccel = 53,
         eDriveBothSignedDutyAccel = 54,
         eReadM1VelocityPID = 55,
         eReadM2VelocityPID = 56,
         eSetM1PositionPID = 61,
         eSetM2PositionPID = 62,
         eReadM1PositionPID = 63,
         eReadM2PositionPID = 64,
         eDriveM1SpeedAccelDeccelPos = 65,
         eDriveM2SpeedAccelDeccelPos = 66,
         eDriveBothSpeedAccelDeccelPos = 67,
         eSetM1DefaultDutyCycleAccel = 68,
         eSetM2DefaultDutyCycleAccel = 69,

         eReadMotorPWMs = 48,
         eReadMotorCurrents = 49,
         eSetMainBatteryVoltages = 57,
         eSetLogicBatteryVoltages = 58,
         eReadMainBatteryVoltageSettings = 59,
         eReadLogicBatterVoltageSettings = 60,
         eSetS3_S4_S5Modes = 74,
         eReadS3_S4_S5Modes = 75,
         eSetDeadBandRC_Analog = 76,
         eReadDeadBandRC_Analog = 77,
         eReadEncodersCounts = 78,
         eReadEncodersSpeeds = 79,
         eRestoreDefaults = 80,
         eReadDefaultDutyCycleAccels = 81,
         eReadTemperature = 82,
         eReadTemperature2 = 83,
         eReadStatus = 90,
         eReadEncoderModes = 91,
         eSetM1EncoderMode = 92,
         eSetM2EncoderMode = 93,
         eWriteSettingsToEEPROM = 94,
         eReadSettingsFromEEPROM = 95,
         eSetStdConfigSettings = 98,
         eReadStdConfigSettings = 99,
         eSetCTRLModes = 100,
         eReadCTRLModes = 101,
         eSetCTRL1 = 102,
         eSetCTRL2 = 103,
         eReadCTRLs = 104,
         eSetM1MaxCurrentLimit = 133,
         eSetM2MaxCurrentLimit = 134,
         eReadM1MaxCurrentLimit = 135,
         eReadM2MaxCurrentLimit = 136,
         eSetPWMMode = 148,
         eReadPWMMode = 149,
         };

      // Device we're talking to, serial or USB
      std::string m_strTTYDevice;

      // RoboClaw address 0x80-0x87
      uint8_t m_nAddr;

      // Timeout for Read Operations (milliseconds)
      uint32_t m_uReadTimeout;

      // File handle
      int m_nPort;

      // Ensure consistency handling buffered commands
      // I prefer interms of indicating "immediate" excution
      // which is in line with their flag setting
      uint8_t BufferCommand(bool bImmediate) const
         {
         return (bImmediate ? 1 : 0);
         }

      // Calculate the CRC
      CRC16 CalcCRC16(int nBytes, uint8_t Packet[]) const
         {
         CRC16 CRC = 0;
         return (UpdateCRC16(CRC, nBytes, Packet));
         }

      // Update the CRC with additional data
      CRC16 UpdateCRC16(CRC16 CRC, int nBytes, uint8_t Packet[]) const;

      // Command send methods
      bool SendCmdBlockCRC(ECommandCode eCmd, int nBytes, uint8_t* pData) const;

      // Send command and calculate CRC
      bool SendHeader(ECommandCode eCmd, CRC16* pCRC = nullptr) const;

      // Send command and CRC  (Command has no data)
      bool SendHeaderCRC(ECommandCode eCmd, CRC16* pHeaderCRC = nullptr) const;

      // Send Command, CRC, (no data), and Receive 0xFF
      bool SendHeaderCRC0xFF(ECommandCode eCmd) const;

      // Send Command, one data byte, and CRC.
      // Return Header CRC if requested to check command response
      bool SendHeaderByteCRC(ECommandCode eCmd, uint8_t uData, CRC16* pHeaderCRC = nullptr) const;
      bool SendHeaderBlockCRC(ECommandCode eCmd, size_t nSize, uint8_t* pBlock,
            CRC16* pHeaderCRC = nullptr) const;

      // Send a simple command (no data and no CRC) and receive the Ack 0xFF
      bool SendSimpleCommand(ECommandCode eCmd) const
         {
         return (SendHeader(eCmd) && Receive0xFF());
         }

      // Send Command, one byte data, CRC, and receive ACK 0xFF
      bool SendHeaderByteCRC0xFF(ECommandCode eCmd, uint8_t uData) const;

      bool SendBlock(size_t nSize, uint8_t* pBlock, uint8_t& CRC) const;

      bool SendCRC(CRC16 CRC) const
         {
         return (SendIntegralType(CRC));
         }

      // Send a compativility mode command and receive the ack
      bool SendCompatibilityCmd(ECommandCode eCmd, uint8_t nData) const;

      // Read one byte and compare to 0xFF
      bool Receive0xFF() const
         {
         uint8_t uByte;
         return (ReadBlock(&uByte, 1) && (uByte == 0xFF));
         }

      // RoboClaw uses integers in Big Endian format.
      // Functions to deal with sending and receiving them in native format.
      template <typename T>
      bool SendIntegralType(T nInteger, CRC16* pCRC = nullptr) const
         {
         T nTransmit = boost::endian::native_to_big(nInteger);
         if (pCRC != nullptr)
            {
            *pCRC = UpdateCRC16(*pCRC, sizeof(T), reinterpret_cast<uint8_t*>(&nTransmit));
            } // end if

         return (write(m_nPort, &nTransmit, sizeof(T)) == sizeof(T));

         }

      template <typename T>
      bool ReadIntegralType(T& nInteger, CRC16* pCRC = nullptr) const
         {
         T nReceive = 0;
         bool bRet = ReadBlock(reinterpret_cast<uint8_t*>(&nReceive), sizeof(T), pCRC);
         if (bRet)
            {
            nInteger = boost::endian::big_to_native(nReceive);
            } // end if

         return (bRet);

         }

      /*************
      *  Since RoboClaw commands are fairly consistent in form but differ in number
      *  and types of parameters (all integral types) it makes sense to template them
      *  so once the template is correct, all commands of that form will be correct.
      **************/

      // Send a command with one integral parameter, receive 0xFF in response
      template <typename T>
      bool Send1IntegralType0xFF(ECommandCode eCmd, T V) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if ((bRet = SendIntegralType(V, &CRC)))
            {
            if ((bRet = SendCRC(CRC)))
               {
               bRet = Receive0xFF();
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command that receives one integral type and checks the CRC
      template <typename T>
      bool Read1IntegralTypeCRC(ECommandCode eCmd, T& V) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = ReadIntegralType(V, &CRC)))
               {
               bRet = CheckCRC(CRC);
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with two integral parameters, receive 0xFF in response
      template <typename T1, typename T2>
      bool Send2IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendCRC(CRC)))
                     {
                     bRet = Receive0xFF();
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command that receives two integral types and checks the CRC
      template <typename T1, typename T2>
      bool Read2IntegralTypesCRC(ECommandCode eCmd, T1& V1, T2& V2) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = ReadIntegralType(V1, &CRC)))
               {
               if ((bRet = ReadIntegralType(V2, &CRC)))
                  {
                  bRet = CheckCRC(CRC);
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with three integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3>
      bool Send3IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendCRC(CRC)))
                        {
                        bRet = Receive0xFF();
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command that receives three integral types and checks the CRC
      template <typename T1, typename T2, typename T3>
      bool Read3IntegralTypesCRC(ECommandCode eCmd, T1& V1, T2& V2, T3& V3) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = ReadIntegralType(V1, &CRC)))
               {
               if ((bRet = ReadIntegralType(V2, &CRC)))
                  {
                  if ((bRet = ReadIntegralType(V3, &CRC)))
                     {
                     bRet = CheckCRC(CRC);
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with four integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3, typename T4>
      bool Send4IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3, T4 V4) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendIntegralType(V4, &CRC)))
                        {
                        if ((bRet = SendCRC(CRC)))
                           {
                           bRet = Receive0xFF();
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command that receives four integral types and checks the CRC
      template <typename T1, typename T2, typename T3, typename T4>
      bool Read4IntegralTypesCRC(ECommandCode eCmd, T1& V1, T2& V2, T3& V3, T4& V4) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = ReadIntegralType(V1, &CRC)))
               {
               if ((bRet = ReadIntegralType(V2, &CRC)))
                  {
                  if ((bRet = ReadIntegralType(V3, &CRC)))
                     {
                     if ((bRet = ReadIntegralType(V4, &CRC)))
                        {
                        bRet = CheckCRC(CRC);
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with five integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3, typename T4, typename T5>
      bool Send5IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3, T4 V4, T5 V5) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendIntegralType(V4, &CRC)))
                        {
                        if ((bRet = SendIntegralType(V5, &CRC)))
                           {
                           if ((bRet = SendCRC(CRC)))
                              {
                              bRet = Receive0xFF();
                              } // end if
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with six integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
      bool Send6IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3, T4 V4, T5 V5, T6 V6) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendIntegralType(V4, &CRC)))
                        {
                        if ((bRet = SendIntegralType(V5, &CRC)))
                           {
                           if ((bRet = SendIntegralType(V6, &CRC)))
                              {
                              if ((bRet = SendCRC(CRC)))
                                 {
                                 bRet = Receive0xFF();
                                 } // end if
                              } // end if
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with seven integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
      bool Send7IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3, T4 V4, T5 V5, T6 V6, T7 V7) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendIntegralType(V4, &CRC)))
                        {
                        if ((bRet = SendIntegralType(V5, &CRC)))
                           {
                           if ((bRet = SendIntegralType(V6, &CRC)))
                              {
                              if ((bRet = SendIntegralType(V7, &CRC)))
                                 {
                                 if ((bRet = SendCRC(CRC)))
                                    {
                                    bRet = Receive0xFF();
                                    } // end if
                                 } // end if
                              } // end if
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command that receives seven integral types and checks the CRC
      template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
      bool Read7IntegralTypesCRC(ECommandCode eCmd, T1& V1, T2& V2, T3& V3, T4& V4, T5& V5, T6& V6, T7& V7) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = ReadIntegralType(V1, &CRC)))
               {
               if ((bRet = ReadIntegralType(V2, &CRC)))
                  {
                  if ((bRet = ReadIntegralType(V3, &CRC)))
                     {
                     if ((bRet = ReadIntegralType(V4, &CRC)))
                        {
                        if ((bRet = ReadIntegralType(V5, &CRC)))
                           {
                           if ((bRet = ReadIntegralType(V6, &CRC)))
                              {
                              if ((bRet = ReadIntegralType(V7, &CRC)))
                                 {
                                 bRet = CheckCRC(CRC);
                                 } // end if
                              } // end if
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a command with nine integral parameters, receive 0xFF in response
      template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6,
            typename T7, typename T8, typename T9>
      bool Send9IntegralTypes0xFF(ECommandCode eCmd, T1 V1, T2 V2, T3 V3, T4 V4, T5 V5, T6 V6,
            T7 V7, T8 V8, T9 V9) const
         {
         CRC16 CRC;
         bool bRet = SendHeader(eCmd, &CRC);
         if (bRet)
            {
            if ((bRet = SendIntegralType(V1, &CRC)))
               {
               if ((bRet = SendIntegralType(V2, &CRC)))
                  {
                  if ((bRet = SendIntegralType(V3, &CRC)))
                     {
                     if ((bRet = SendIntegralType(V4, &CRC)))
                        {
                        if ((bRet = SendIntegralType(V5, &CRC)))
                           {
                           if ((bRet = SendIntegralType(V6, &CRC)))
                              {
                              if ((bRet = SendIntegralType(V7, &CRC)))
                                 {
                                 if ((bRet = SendIntegralType(V8, &CRC)))
                                    {
                                    if ((bRet = SendIntegralType(V9, &CRC)))
                                       {
                                       if ((bRet = SendCRC(CRC)))
                                          {
                                          bRet = Receive0xFF();
                                          } // end if
                                       } // end if
                                    } // end if
                                 } // end if
                              } // end if
                           } // end if
                        } // end if
                     } // end if
                  } // end if
               } // end if
            } // end if

         return (bRet);

         }

      // Send a block of data and update the CRC if requested
      bool SendBlock(size_t nSize, uint8_t* pBlock, CRC16* pCRC) const;

      bool SendBool(bool bBool, CRC16* pCRC) const
         {
         uint8_t nValue = bBool ? 1 : 0;
         return (SendIntegralType(nValue, pCRC));
         }

      // Read a null terminated string into the buffer.
      // The default is to strip out nonprintable characters
      bool ReadString(std::string& strString, CRC16& CalcCRC, bool bStripCTRL = true) const;

      // Read a specific size block of data into the buffer
      bool ReadBlock(uint8_t* pBuf, size_t nBytes, CRC16* pCalcCRC = nullptr) const;

      // Read a single character from the RoboClaw with timeout
      bool ReadByte(uint8_t& uChar) const;

      // Read the CRC sent from the RoboClaw.  Value is in NATIVE format!
      bool ReadCRC16(CRC16& CRC) const
         {
         return (ReadIntegralType(CRC));
         }

      // Read the CRC from the RoboClaw and validate against the calculated CRC
      bool CheckCRC(CRC16 CalcCRC) const
         {
         CRC16 RecdCRC;
         bool bRet = ReadCRC16(RecdCRC);
         if (bRet)
            {
            bRet = (RecdCRC == CalcCRC);
            } // end if

         return (bRet);
         }

      // Common ReadEncoder code via Commands 16 & 17
      bool ReadEncoder(ECommandCode eCmd, uint32_t& uCount, EncoderStatus& Status) const;
      bool ReadEncoder(ECommandCode eCmd, int32_t& uCount, EncoderStatus& Status) const;

      // Common code for reading encoder speed via Commands 18 & 19
      bool ReadEncoderSpeed(ECommandCode eCmd, int32_t& nSpeed, EDirection& eDirection) const;

      // Common code for reading raw encoder speed via Commands 30 & 31
      bool ReadEncoderRawSpeed(ECommandCode eCmd, int32_t& nSpeed, EDirection& eDirection) const;

   private :
   };  // End of class DRoboClawFS

#endif // __ROBOCLAWFS_H__
