/*****************************************************************************
******************************** RoboClaw.cpp ********************************
*****************************************************************************/

/*****************************************************************************
******************************  I N C L U D E  *******************************
*****************************************************************************/

#include "RoboClawFS.h"

#include <math.h>
#include <boost/timer/timer.hpp>

/*****************************************************************************
********************* Class DRoboClawFS Implementation ********************
*****************************************************************************/

/*****************************************************************************
*
*  DRoboClawFS::DRoboClawFS
*
*****************************************************************************/

DRoboClawFS::DRoboClawFS(
      const std::string strTTYDevice /* = "/dev/roboclaw" */,
      uint8_t uAddr /* = 0x80 */, uint32_t uTimeout /* = 100 */)
      : m_strTTYDevice(strTTYDevice), m_nAddr(uAddr),
      m_uReadTimeout(uTimeout), m_nPort(-1)
   {

   return;

   } // End of function DRoboClawFS::DRoboClawFS

/******************************************************************************
*
***  DRoboClawFS::~DRoboClawFS
*
******************************************************************************/

DRoboClawFS::~DRoboClawFS()
   {
   ClosePort();

   return;

   } // end of method DRoboClawFS::DRoboClawFS

/******************************************************************************
*
***  DRoboClawFS::OpenPort
*
******************************************************************************/

bool DRoboClawFS::OpenPort()
   {
   #ifndef O_NOCTTY
   #define O_NOCTTY	00000400	/* not fcntl */
   #endif

   #ifndef O_NONBLOCK
   #define O_NONBLOCK	00004000
   #endif

   if (m_nPort != -1)
      {
      ClosePort();
      } // end if

   m_nPort = open(m_strTTYDevice.c_str(), O_RDWR | O_BINARY | O_NONBLOCK | O_NOCTTY);
   if (m_nPort < 0)
      {
      m_nPort = -1;

      std::cerr << "Cannot open device " << m_strTTYDevice << std::endl;
      } // end if

   return (m_nPort != -1);

   } // end of method DRoboClawFS::OpenPort

/******************************************************************************
*
***  DRoboClawFS::UpdateCRC16
*
*  Update the passed CRC by including the nBypes in Buffer.
*
******************************************************************************/

DRoboClawFS::CRC16 DRoboClawFS::UpdateCRC16(CRC16 CRC, int nBytes,
      uint8_t Buffer[]) const
   {
   for (int nByte = 0 ; nByte < nBytes ; nByte++)
      {
      CRC ^= (static_cast<CRC16>(Buffer[nByte]) << 8);
      for (unsigned char nBit = 0 ; nBit < 8 ; nBit++)
         {
         if (CRC & 0x8000)
            {
            CRC = (CRC << 1) ^ 0x1021;
            } // end if
         else
            {
            CRC = CRC << 1;
            } // end else
         } // end for
      } // end for

   return (CRC);

   } // end of method DRoboClawFS::UpdateCRC16

/******************************************************************************
*
***  DRoboClawFS::SendHeader
*
* Send the Address and Command.  Calculate the CRC if requested.
*
******************************************************************************/

bool DRoboClawFS::SendHeader(ECommandCode eCmd, CRC16* pCRC /* = nullptr */) const
   {
   uint8_t Header[2] = { m_nAddr, static_cast<uint8_t>(eCmd) };
   if (pCRC != nullptr)
      {
      *pCRC = CalcCRC16(2, Header);
      } // end if

   return (write(m_nPort, Header, 2) == 2);

   } // end of method DRoboClawFS::SendHeader

/******************************************************************************
*
***  DRoboClawFS::SendHeaderCRC
*
* Send the Address and Command and CRC.  A simple command with no data.
*
******************************************************************************/

bool DRoboClawFS::SendHeaderCRC(ECommandCode eCmd,
      CRC16* pHeaderCRC /* = nullptr */) const
   {
   CRC16 CRC;
   bool bRet = SendHeader(eCmd, &CRC);
   if (bRet)
      {
      if (pHeaderCRC != nullptr)
         {
         *pHeaderCRC = CRC;
         } // end if

      bRet = SendCRC(CRC);
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::SendHeaderCRC

/******************************************************************************
*
***  DRoboClawFS::SendHeaderBlockCRC
*
*  Send the command with a block of data and the CRC.  Return the HeaderCRC
*  in case it's needed to form the CRC for received data.
*
******************************************************************************/

bool DRoboClawFS::SendHeaderBlockCRC(ECommandCode eCmd, size_t nSize,
      uint8_t* pBlock, CRC16* pHeaderCRC /* = nullptr */) const
   {
   CRC16 CRC;
   bool bRet = SendHeader(eCmd, &CRC);

   if (bRet)
      {
      if (pHeaderCRC != nullptr)
         {
         *pHeaderCRC = CRC;
         } // end if

      bRet = SendBlock(nSize, pBlock, &CRC);
      if (bRet)
         {
         bRet = SendCRC(CRC);
         } // end if
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::SendHeaderBlockCRC

/******************************************************************************
*
***  DRoboClawFS::SendHeaderByteCRC
*
******************************************************************************/

bool DRoboClawFS::SendHeaderByteCRC(ECommandCode eCmd, uint8_t uData,
      CRC16* pHeaderCRC /* = nullptr */) const
   {
   bool bRet = SendHeaderBlockCRC(eCmd, 1, &uData, pHeaderCRC);

   return (bRet);

   } // end of method DRoboClawFS::SendHeaderByteCRC

/******************************************************************************\
*
***  DRoboClawFS::SendHeaderCRC0xFF
*
* Send the Address and Command and CRC.  A simple command with no data.
* Receive the ACK of 0xFF in return from RoboClaw.
*
******************************************************************************/

bool DRoboClawFS::SendHeaderCRC0xFF(ECommandCode eCmd) const
   {
   bool bRet = SendHeaderCRC(eCmd);
   if (bRet)
      {
      bRet = Receive0xFF();
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::SendHeaderCRC0xFF

/******************************************************************************
*
***  DRoboClawFS::SendHeaderByteCRC0xFF
*
******************************************************************************/

bool DRoboClawFS::SendHeaderByteCRC0xFF(ECommandCode eCmd, uint8_t uByte) const
   {
   bool bRet = SendHeaderByteCRC(eCmd, uByte);
   if (bRet)
      {
      bRet = Receive0xFF();
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::SendHeaderByteCRC0xFF

/*****************************************************************************
*
*  DRoboClawFS::SendBlock
*
*  Send a block of data to the RoboClaw.  Accumulate the CRC.  This is used
*  to handle block data transfers to the RoboClaw and should follow the
*  SendCmd() call.
*
*****************************************************************************/

bool DRoboClawFS::SendBlock(size_t nSize, uint8_t* pBlock, CRC16* pCRC) const
   {
   if (pCRC != nullptr)
      {
      *pCRC = UpdateCRC16(*pCRC, nSize, pBlock);
      } // end if

   size_t nWritten = write(m_nPort, pBlock, nSize);

   return (nWritten == nSize);

   } // End of function DRoboClawFS::SendBlock

/******************************************************************************
*
***  DRoboClawFS::SendCompatibilityCmd
*
******************************************************************************/

bool DRoboClawFS::SendCompatibilityCmd(ECommandCode eCmd, uint8_t nData) const
   {

   return (Send1IntegralType0xFF(eCmd, nData));

   } // end of method DRoboClawFS::SendCompatibilityCmd

/******************************************************************************
*
***  DRoboClawFS::ReadEncoder
*
* Common code to read a specified encoder via Commands 16 & 17.
*
******************************************************************************/

bool DRoboClawFS::ReadEncoder(ECommandCode eCmd, uint32_t& uCount, EncoderStatus& Status) const
   {
   uint8_t uStatus;
   bool bRet = Read2IntegralTypesCRC(eCmd, uCount, uStatus);
   if (bRet)
      {
      Status.DecodeByte(uStatus);
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::ReadEncoder

bool DRoboClawFS::ReadEncoder(ECommandCode eCmd, int32_t& nCount, EncoderStatus& Status) const
      {
      uint8_t uStatus;
      bool bRet = Read2IntegralTypesCRC(eCmd, nCount, uStatus);
      if (bRet)
         {
         Status.DecodeByte(uStatus);
         } // end if

      return (bRet);

   } // end of method DRoboClawFS::ReadEncoder

/******************************************************************************
*
***  DRoboClawFS::SetEncoder
*
* Common code to set an Encoder via Cmds 22 & 23
*
******************************************************************************/

bool DRoboClawFS::SetEncoder(ECommandCode eCmd, uint32_t uCounts) const
   {

   return (Send1IntegralType0xFF(eCmd, uCounts));

   } // end of method DRoboClawFS::SetEncoder

bool DRoboClawFS::SetEncoder(ECommandCode eCmd, int32_t nCounts) const
   {

   return (Send1IntegralType0xFF(eCmd, nCounts));

   } // end of method DRoboClawFS::SetEncoder

/******************************************************************************
*
***  DRoboClawFS::ReadEncoderSpeed
*
******************************************************************************/

bool DRoboClawFS::ReadEncoderSpeed(ECommandCode eCmd, int32_t& nSpeed,
      EDirection& eDirection) const
   {
   uint8_t uDirection;
   bool bRet = Read2IntegralTypesCRC(eCmd, nSpeed, uDirection);
   if (bRet)
      {
      eDirection = (uDirection == 0) ? eFwd : eRev;
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::ReadEncoderSpeed

/******************************************************************************
*
***  DRoboClawFS::ReadEncoderRawSpeed
*
*  Common code for reading the raw encoder speed via Commands 30 & 31.
*  Counts recorded in the last 1/300th of a second.  [Documentation says
*  counts/second but sounds like should be raw counts]
*
******************************************************************************/

bool DRoboClawFS::ReadEncoderRawSpeed(ECommandCode eCmd, int32_t& nSpeed,
      EDirection& eDirection) const
   {
   uint8_t uDirection;
   bool bRet = Read2IntegralTypesCRC(eCmd, nSpeed, uDirection);
   if (bRet)
      {
      eDirection = (uDirection == 0) ? eFwd : eRev;
      } // end if

   return (bRet);

   } // end of method DRoboClawFS::ReadEncoderRawSpeed

/*****************************************************************************
*
*  DRoboClawFS::ReadFirmwareVersion
*
*****************************************************************************/

bool DRoboClawFS::ReadFirmwareVersion(std::string& strVersion) const
   {
   CRC16 uCalcCRC;
   bool bRet = SendHeader(ECommandCode::eReadVersion, &uCalcCRC);
   if (bRet)
      {
      bRet = ReadString(strVersion, uCalcCRC);
      } // end if
   else
      {
      strVersion = "Read Version Failed!";
      } // end else

      return (bRet);

   } // End of function DRoboClawFS::ReadFirmwareVersion

/*****************************************************************************
*
*  DRoboClawFS::ReadString
*
*  Read a NULL terminated string into the buffer and check the CRC.
*
*****************************************************************************/

bool DRoboClawFS::ReadString(std::string& strString, CRC16& CalcCRC,
      bool bStripCTRL /* = true */) const
   {
   bool bRet = true;
   uint8_t Buf[60];
   uint8_t* pBuf = Buf;
   bool bRead = true;
   uint8_t uByte;

   do
      {
      if ((bRet = ReadByte(uByte)))
         {
         // The received byte is part of the string
         CalcCRC = UpdateCRC16(CalcCRC, 1, &uByte);

         // Classify the character
         if ((uByte >= 0x20) || (uByte == 0) || !bStripCTRL)
            {
            // Include the character in the string
            *pBuf = uByte;
            pBuf++;
            } // end if

         // Null character terminates string, so check CRC
         bRead = (uByte != 0);
         } // end if
      } while (bRead && bRet);

   bRet = CheckCRC(CalcCRC);

   if (bRet)
      {
      strString = reinterpret_cast<char*>(Buf);
      } // end if
   else
      {
      strString = "Read Version Failed!!!";
      } // end else

   return (bRet);

   } // End of function DRoboClawFS::ReadString

/*****************************************************************************
*
*  DRoboClawFS::ReadBlock
*
*  Read nBytes of data into the buffer and update the CRC in requested.
*
*****************************************************************************/

bool DRoboClawFS::ReadBlock(uint8_t* pBuf, size_t nBytes, CRC16* pCalcCRC /* = nullptr */) const
   {
   bool bRet = true;

   for (size_t nByte = 0 ; bRet && (nByte < nBytes) ; nByte++)
      {
      uint8_t uByte;
      bRet = ReadByte(uByte);
      if (bRet)
         {
         pBuf[nByte] = uByte;
         } // end if
      } // end for

   if (pCalcCRC != nullptr)
      {
      *pCalcCRC = UpdateCRC16(*pCalcCRC, nBytes, pBuf);
      } // end if

   return (bRet);

   } // End of function DRoboClawFS::ReadBlock

/*****************************************************************************
*
*  DRoboClawFS::ReadByte
*
*****************************************************************************/

bool DRoboClawFS::ReadByte(uint8_t& uByte) const
   {
   bool bRet = false;

   bool bRead = true;
   boost::timer::cpu_timer Timer;
   boost::timer::nanosecond_type Timeout(m_uReadTimeout * 1000000LL);
   while (bRead)
      {
      if (read(m_nPort, &uByte, 1) == 1)
         {
         // Got a character so bail
         bRead = false;
         bRet = true;

#if defined(DEBUG_RESPONSE)
         if ((uByte >= ' ') && (uByte <= 127))
            {
            std::cout << "Got byte " << static_cast<char>(uByte) << std::endl;
            } // end if
         else
            {
            std::cout << "Got byte " << static_cast<uint32_t>(uByte) << std::endl;
            } // end else
#endif
         } // end if
      else
         {
         // Check the elapsed time against the timeout
         std::cout << "No byte available" << std::endl;
         const boost::timer::cpu_times Elapsed(Timer.elapsed());
         if (boost::timer::nanosecond_type(Elapsed.user + Elapsed.system) > Timeout)
            {
            // Timed out
            bRet = false;
            bRead = false;
            std::cout << "Timed out!!!" << std::endl;
            } // end if
         } // end else
      } // end while

   return (bRet);

   } // End of function DRoboClawFS::ReadByte

/*****************************************************************************
*** Advanced Motor Control
*****************************************************************************/

/******************************************************************************
*
***  DRoboClawFS::SeMotorVelocityPIDConstants
*
* Cmd 28 & 29 Set Velocity PID constants
*
******************************************************************************/

bool DRoboClawFS::SetVelocityPIDConstants(ECommandCode eCmd, uint32_t uP,
      uint32_t uI, uint32_t uD, uint32_t uMaxSpeed) const
   {

   return (Send4IntegralTypes0xFF(eCmd, uD, uP, uI, uMaxSpeed));

   } // end of method DRoboClawFS::SetMotorVelocityPIDConstants

/******************************************************************************
*
***  DRoboClawFS::DriveMotorSignedDutyCycle
*
* Cmd 32 & 33 Drive Motor Signed Duty Cycle (No encorders required)
* Value is -32,767 to +32,767 (-100% to +100%)
*
******************************************************************************/

bool DRoboClawFS::DriveSignedDutyCycle(ECommandCode eCmd, int16_t nDutyCycle) const
   {

   return (Send1IntegralType0xFF(eCmd, nDutyCycle));

   } // end of method DRoboClawFS::DriveMotorSignedDutyCycle

/******************************************************************************
*
***  DRoboClawFS::DriveSignedSpeed
*
* Cmd 35 & 36 Drive Motor Signed Speed
* Speed is in quadrature pulses/second
*
******************************************************************************/

bool DRoboClawFS::DriveSignedSpeed(ECommandCode eCmd, int32_t nSpeed) const
   {

   return (Send1IntegralType0xFF(eCmd, nSpeed));

   } // end of method DRoboClawFS::DriveSignedSpeed

/******************************************************************************
*
***  DRoboClawFS::DriveSignedSpeedAccel
*
* Cmd 38 & 39 Drive Motor Signed Speed & Acceleration
* Sign on speed indicates the direction, acceleration is not signed.
*
******************************************************************************/

bool DRoboClawFS::DriveSignedSpeedAccel(ECommandCode eCmd, int32_t nSpeed, uint32_t uAccel) const
   {

   return (Send2IntegralTypes0xFF(eCmd, uAccel, nSpeed));

   } // end of method DRoboClawFS::DriveSignedSpeedAccel

/******************************************************************************
*
***  DRoboClawFS::DriveSignedSpeedDistance
*
* Cmd 41 & 42 Buffered Drive Motor With Signed Speed and Distance
* Sign on speed indicates direction, distance is unsigned
*
******************************************************************************/

bool DRoboClawFS::DriveSignedSpeedDistance(ECommandCode eCmd, int32_t nSpeed,
      uint32_t uDistance, bool bImmediate /* = true */) const
   {

   return (Send3IntegralTypes0xFF(eCmd, nSpeed, uDistance, BufferCommand(bImmediate)));

   } // end of method DRoboClawFS::DriveSignedSpeedDistance

/******************************************************************************
*
***  DRoboClawFS::DriveSignedSpeedAccelDistance
*
* Cmd 44 & 45 Buffered Drive Motor With Signed Speed, Accel, & Distance
*
******************************************************************************/

bool DRoboClawFS::DriveSignedSpeedAccelDistance(ECommandCode eCmd, int32_t nSpeed,
      uint32_t uAccel, uint32_t uDistance, bool bImmediate /* = true */) const
   {

   return (Send4IntegralTypes0xFF(eCmd, uAccel, nSpeed, uDistance, BufferCommand(bImmediate)));

   } // end of method DRoboClawFS::DriveSignedSpeedAccelDistance

/******************************************************************************
*
***  DRoboClawFS::DriveSignedDutyAccel
*
* Cmd 52 & 52 Drive Motor With Signed Duty And Acceleration
* The duty value is signed and the range is -32768 to +32767(eg. +-100% duty).
* The accel value range is 0 to 65535(eg maximum acceleration rate is -100% to 100% in 100ms).
* The manual says accel max is 655359.  Typo????
*
******************************************************************************/

bool DRoboClawFS::DriveSignedDutyAccel(ECommandCode eCmd, int16_t nDuty, uint16_t uAccel) const
   {

   return (Send2IntegralTypes0xFF(eCmd, nDuty, uAccel));

   } // end of method DRoboClawFS::DriveSignedDutyAccel

/******************************************************************************
*
***  DRoboClawFS::ReadVelocityPIDConstants
*
* Cmd 55 & 56 Read Motor Velocity PID and MaxSpeed Constants
* uMaxSpeed is the motor speed in QPPS at 100% power
*
******************************************************************************/

bool DRoboClawFS::ReadVelocityPIDConstants(ECommandCode eCmd, uint32_t& uP, uint32_t& uI,
      uint32_t& uD, uint32_t& uMaxSpeed) const
   {

   return (Read4IntegralTypesCRC(eCmd, uP, uI, uD, uMaxSpeed));

   } // end of method DRoboClawFS::ReadVelocityPIDConstants

/******************************************************************************
*
***  DRoboClawFS::SetPositionPIDConstants
*
* Cmd 61 & 62 Set Motor Position PID Constants
******************************************************************************/

bool DRoboClawFS::SetPositionPIDConstants(ECommandCode eCmd, uint32_t uP, uint32_t uI,
      uint32_t uD, uint32_t uMaxIWindup, uint32_t uDeadzone, int32_t nMinPos,
      int32_t nMaxPos) const
   {

   return (Send7IntegralTypes0xFF(eCmd, uD, uP, uI, uMaxIWindup, uDeadzone,
         nMinPos, nMaxPos));

   } // end of method DRoboClawFS::SetPositionPIDConstants

/******************************************************************************
*
***  DRoboClawFS::ReadPositionPIDConstants
*
* Cmd 63 & 64 Read Motor Position PID Constants
*
******************************************************************************/

bool DRoboClawFS::ReadPositionPIDConstants(ECommandCode eCmd, uint32_t& uP, uint32_t& uI,
      uint32_t& uD, uint32_t& uMaxIWindup, uint32_t& uDeadzone, int32_t& nMinPos,
      int32_t& nMaxPos) const
   {

    return (Read7IntegralTypesCRC(eCmd, uP, uI, uD,
       uMaxIWindup, uDeadzone, nMinPos, nMaxPos));

   } // end of method DRoboClawFS::ReadPositionPIDConstants

/******************************************************************************
*
***  DRoboClawFS::DriveSignedSpeedAccelDecelPos
*
* Cmd 65 & 66 Buffered Drive M1 with signed Speed, Accel, Deccel and Position
*
******************************************************************************/

bool DRoboClawFS::DriveSignedSpeedAccelDecelPos(ECommandCode eCmd, int32_t nSpeed,
      uint32_t uAccel, uint32_t uDecel, int32_t nPos, bool bImmediate /* = true */) const
   {

   return (Send5IntegralTypes0xFF(eCmd, uAccel, nSpeed, uDecel, nPos,
      BufferCommand(bImmediate)));

   } // end of method DRoboClawFS::DriveSignedSpeedAccelDecelPos

/******************************************************************************
*
***  DRoboClawFS::SetMaxCurrentLimit
*
* Cmd 133 & 134 Set Motor Max Current Limit
* Current value is in 10ma units.
*
******************************************************************************/

bool DRoboClawFS::SetMaxCurrentLimit(ECommandCode eCmd, uint32_t uMaxCurrent, uint32_t uMinCurrent /* = 0*/) const
   {

   return (Send2IntegralTypes0xFF(eCmd,  uMaxCurrent, uMinCurrent));

   } // end of method DRoboClawFS::SetMaxCurrentLimit

/******************************************************************************
*
***  DRoboClawFS::ReadMaxCurrentLimit
*
* Cmd 135 & 136 Read Motor Max Current Limit
* Current value is in 10ma units
* Min Current is always zero
*
******************************************************************************/

bool DRoboClawFS::ReadMaxCurrentLimit(ECommandCode eCmd, uint32_t& uMaxCurrent, uint32_t& uMinCurrent) const
   {

   return (Read2IntegralTypesCRC(eCmd, uMaxCurrent, uMinCurrent));

   } // end of method DRoboClawFS::ReadMaxCurrentLimit

#if defined(IMPLEMENT_STRING_FUNTIONS)

/******************************************************************************
*
***  DRoboClawFS::GetStatusString
*
******************************************************************************/

std::string DRoboClawFS::GetStatusString(EStatus eStatus)
   {
   std::string strStatus = "Unknown";

   switch (eStatus)
      {
      case EStatus::eNormal:
         strStatus = "Normal";
         break;

      case EStatus::eM1OverCurrentWarning:
         strStatus = "M1 Over Current Warning";
         break;

      case EStatus::eM2OverCurrentWarning:
         strStatus = "M2 Over Current Warning";
         break;

      case EStatus::eEStop:
         strStatus = "EStop";
         break;

      case EStatus::eTempError:
         strStatus = "Temperature Error";
         break;

      case EStatus::eTemp2Error:
         strStatus = "Temperature2 Error";
         break;

      case EStatus::eMainBatteryHighError:
         strStatus = "Main Battery High Error";
         break;

      case EStatus::eLogicBatteryHighError:
         strStatus = "Logic Battery High Error";
         break;

      case EStatus::eLogicBatteryLowError:
         strStatus = "Logic Battery Low Error";
         break;

      case EStatus::eM1DriveFault:
         strStatus = "M1 Drive Fault";
         break;

      case EStatus::eM2DriveFault:
         strStatus = "M2 Drive Fault";
         break;

      case EStatus::eMainBatteryHighWarning:
         strStatus = "Main Battery High Warning";
         break;

      case EStatus::eMainBatteryLowWarning:
         strStatus = "Main Battery Low Warning";
         break;

      case EStatus::eTempWarning:
         strStatus = "Temperature Warning";
         break;

      case EStatus::eTemp2Warning:
         strStatus = "Temperature2 Warning";
         break;

      case EStatus::eM1Home:
         strStatus = "M1 Home";
         break;

      case EStatus::eM2Home:
         strStatus = "M2 Home";
         break;

      default:
         break;
      } // end switch

   return (strStatus);

   } // end of method DRoboClawFS::GetStatusString

/******************************************************************************
*
***  DRoboClawFS::GetPWMModeString
*
******************************************************************************/

std::string DRoboClawFS::GetPWMModeString(EPWMMode ePWMMode)
   {

   return ((ePWMMode == eLockedAntiphase) ? "Locked Antiphase" : "Sign Magnitude");

   } // end of method DRoboClawFS::GetPWMModeString

/******************************************************************************
*
***  DRoboClawFS::GetDirectionString
*
******************************************************************************/

std::string DRoboClawFS::GetDirectionString(EDirection eDirection)
   {

   return ((eDirection == eFwd) ? "Forward" : "Reverse");

   } // end of method DRoboClawFS::GetDirectionString

/******************************************************************************
*
***  DRoboClawFS::GetS3ModeString
*
******************************************************************************/

std::string DRoboClawFS::GetS3ModeString(ES3Mode eMode)
   {
   static std::string Names[] = { "Disabled", "EStop Latching", "EStop",
      "Voltage Clamp" };

   return (Names[static_cast<uint8_t>(eMode)]);

   } // end of method DRoboClawFS::GetS3ModeString

/******************************************************************************
*
***  DRoboClawFS::GetS4ModeString
*
******************************************************************************/

std::string DRoboClawFS::GetS4ModeString(ES4Mode eMode)
   {
   static std::string Names[] = { "Disabled", "EStop Latching", "EStop",
      "Voltage Clamp", "Motor 1 Home" };

   return (Names[static_cast<uint8_t>(eMode)]);

   } // end of method DRoboClawFS::GetS4ModeString

/******************************************************************************
*
***  DRoboClawFS::GetS5ModeString
*
******************************************************************************/

std::string DRoboClawFS::GetS5ModeString(ES5Mode eMode)
   {
   static std::string Names[] = { "Disabled", "EStop Latching", "EStop",
      "Voltage Clamp", "Motor 2 Home" };

   return (Names[static_cast<uint8_t>(eMode)]);

   } // end of method DRoboClawFS::GetS5ModeString

#endif
