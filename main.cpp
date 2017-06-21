#include <iostream>
#include <thread>
#include <chrono>

#include "PanTilt.h"

using namespace std;

void Function(string strName)
   {
   cout << strName << endl;
   return;
   }

void OutputFailure(string strMsg)
   {
   cout << strMsg << " Failed!!!" << endl;
   return;
   }

std::string BoolString(bool bBool)
   {
   return (bBool ? "True" : "False");
   }

void PrintEncoderStatus(int nEncoder, uint32_t uCount,
      DRoboClawFS::EncoderStatus eStatus)
   {
   cout << "Encoder " << nEncoder << " Count: " << uCount
         << " Underflow: " << BoolString(eStatus.m_bUnderflow)
         << " Overflow: " << BoolString(eStatus.m_bOverflow)
         << " Direction: " <<  DRoboClawFS::GetDirectionString(eStatus.m_eDirection)
         << endl;

   return;

   }

void PrintMotorSpeed(int nMotor, int32_t nSpeed, DRoboClawFS::EDirection eDirection)
   {
   cout << "Motor " << nMotor << " Speed: " << nSpeed << " "
         << DRoboClawFS::GetDirectionString(eDirection) << endl;

   return;

   }

#if 0
void Test%%Name%%(DRoboClawFS& RC)
   {
   Function("%%Name%%");
   if (RC.%%Name%%())
      {
      cout << "" <<  << endl;
      } // end if
   else
      {
      OutputFailure("%%Name%%");
      } // end else

   return;

   }
#endif

// Cmd 21
void TestFirmwareVersion(DRoboClawFS& RC)
   {
   Function("Read Firmware Version");
   string strVersion;
   if (RC.ReadFirmwareVersion(strVersion))
      {
      cout << "Firmware Version: " << strVersion << endl;
      } // end if
   else
      {
      OutputFailure("ReadFirmwareVersion");
      } // end else

   return;

   }

// Cmd 24
void TestReadMainBatteryVoltage(DRoboClawFS& RC)
   {
   Function("ReadMainBatteryVoltage");
   float fVoltage;
   if (RC.ReadMainBatteryVoltage(fVoltage))
      {
      cout << "Main Battery Voltage: " << fVoltage << endl;
      } // end if
   else
      {
      OutputFailure("ReadMainBatteryVoltage");
      } // end else

   return;

   }

// Cmd 25
void TestReadLogicBatteryVoltage(DRoboClawFS& RC)
   {
   Function("ReadLogicBatteryVoltage");
   float fVoltage;
   if (RC.ReadLogicBatteryVoltage(fVoltage))
      {
      cout << "Logic Battery Voltage: " << fVoltage << endl;
      } // end if
   else
      {
      OutputFailure("ReadLogicBatteryVoltage");
      } // end else

   return;

   }

// Cmd 48
void TestReadMotorPWMValues(DRoboClawFS& RC)
   {
   Function("ReadMotorPWMValues");
   float fPWM1, fPWM2;
   if (RC.ReadMotorPWMValues(fPWM1, fPWM2))
      {
      cout << "Motor PWM Values: " << fPWM1 << ",  " << fPWM2<< endl;
      } // end if
   else
      {
      OutputFailure("ReadMotorPWMValues");
      } // end else

   return;

   }

// Cmd 49
void TestReadMotorCurrents(DRoboClawFS& RC)
   {
   Function("ReadMotorPWMValues");
   float fCurrent1, fCurrent2;
   if (RC.ReadMotorCurrents(fCurrent1, fCurrent2))
      {
      cout << "Motor Current Values: " << fCurrent1 << ",  " << fCurrent2<< endl;
      } // end if
   else
      {
      OutputFailure("ReadMotorPWMValues");
      } // end else

   return;

   }

// Cmds 57 & 59
void TestMainBatteryVoltageMinMax(DRoboClawFS& RC)
   {
   float fMin = 10.0f;
   float fMax = 20.0f;
   Function("SetMainBatteryVoltageMinMax");
   if (!RC.SetMainBatteryVoltageMinMax(fMin, fMax))
      {
      OutputFailure("SetMainBatteryVoltageMinMax");
      } // end if

   Function("ReadMainBatteryVoltageMinMax");
   if (RC.ReadMainBatteryVoltageMinMax(fMin, fMax))
      {
      cout << "Main Battery Voltage Min Max: " << fMin << ", " << fMax << endl;
      } // end if
   else
      {
      OutputFailure("Read Main Battery Voltage Min Max");
      } // end else

   return;

   }

// Cmds 58 & 60
void TestLogicBatteryVoltageMinMax(DRoboClawFS& RC)
   {
   float fMin = 10.0f;
   float fMax = 20.0f;
   Function("SetLogicBatteryVoltageMinMax");
   if (!RC.SetLogicBatteryVoltageMinMax(fMin, fMax))
      {
      OutputFailure("SetLogicBatteryVoltageMinMax");
      } // end if

   Function("ReadLogicBatteryVoltageMinMax");
   if (RC.ReadLogicBatteryVoltageMinMax(fMin, fMax))
      {
      cout << "Logic Battery Voltage Min Max: " << fMin << ", " << fMax << endl;
      } // end if
   else
      {
      OutputFailure("Read Logic Battery Voltage Min Max");
      } // end else

   return;

   }

// Cmds 68, 69, 81
void TestDefaultDutyAccel(DRoboClawFS& RC)
   {
   Function("SetM1DefaultDutyAccel");
   uint32_t uAccel1 = 500;
   uint32_t uAccel2 = 600;
   if (!RC.SetM1DefaultDutyAccel(uAccel1))
      {
      OutputFailure("Set M1 DefaultDutyAccel");
      } // end if

   Function("SetM2DefaultDutyAccel");
   if (!RC.SetM2DefaultDutyAccel(uAccel2))
      {
      OutputFailure("Set M2 DefaultDutyAccel");
      } // end if

   Function("ReadDefaultDutyAccels");
   if (RC.ReadDefaultDutyAccels(uAccel1, uAccel2))
      {
      cout << "Default Duty Accels: " << uAccel1 << ", " << uAccel2 << endl;
      } // end if
   else
      {
      OutputFailure("Read Duty Accels");
      } // end else

   return;

   }

// Cmds 16, 17, 20, 22, 23, & 78
void TestEncoders(DRoboClawFS& RC)
   {
   uint32_t uCount1, uCount2;

   Function("SetEncoder1");
   if (RC.SetEncoder1(uCount1 = 100))
      {
      cout << "Set Encoder 1: " << uCount1 << endl;
      } // end if
   else
      {
      OutputFailure("SetEncoder1");
      } // end else

   Function("SetEncoder2");
   if (RC.SetEncoder2(uCount2 = 200))
      {
      cout << "Set Encoder 2: " << uCount2 << endl;
      } // end if
   else
      {
      OutputFailure("SetEncoder2");
      } // end else

   uCount1 = 0;
   uCount2 = 0;

   DRoboClawFS::EncoderStatus eStatus1, eStatus2;

   Function("ReadEncoder1");
   if (RC.ReadEncoder1(uCount1, eStatus1))
      {
      PrintEncoderStatus(1, uCount1, eStatus1);
      } // end if
   else
      {
      OutputFailure("ReadEncoder1");
      } // end else

   Function("ReadEncoder2");
   if (RC.ReadEncoder2(uCount2, eStatus2))
      {
      PrintEncoderStatus(2, uCount2, eStatus2);
      } // end if
   else
      {
      OutputFailure("ReadEncoder2");
      } // end else

   uCount1 = 0;
   uCount2 = 0;

   Function("ReadEncodersCounts");
   if (RC.ReadEncodersCounts(uCount1, uCount2))
      {
      cout << "Encoder Counts: " << uCount1 << ", " << uCount2  << endl;
      } // end if
   else
      {
      OutputFailure("ReadEncodersCounts");
      } // end else

   Function("ResetEncoders");
   if (!RC.ResetEncoders())
      {
      OutputFailure("ResetEncoders");
      } // end if

   Function("ReadEncodersCounts");
   if (RC.ReadEncodersCounts(uCount1, uCount2))
      {
      cout << "Encoder Counts: " << uCount1 << ", " << uCount2  << endl;
      } // end if
   else
      {
      OutputFailure("ReadEncodersCounts");
      } // end else

   return;

   }

// Cmd 79
void TestReadEncodersSpeeds(DRoboClawFS& RC)
   {
   Function("ReadEncodersSpeeds");
   int32_t nSpeed1, nSpeed2;
   if (RC.ReadEncodersSpeeds(nSpeed1, nSpeed2))
      {
      cout << "Encoder Speeds: " << nSpeed1 << ", " << nSpeed2 << endl;
      } // end if
   else
      {
      OutputFailure("ReadEncodersSpeeds");
      } // end else

   return;

   }

// Cmds 82 & 83
void TestReadTemperatures(DRoboClawFS& RC)
   {
   Function("ReadTemperature");
   float fTemp;
   if (RC.ReadTemperature(fTemp))
      {
      cout << "Temperature: " << fTemp << endl;
      } // end if
   else
      {
      OutputFailure("ReadTemperature");
      } // end else

   Function("ReadTemperature2");
   if (RC.ReadTemperature2(fTemp))
      {
      cout << "Temperature2: " << fTemp << endl;
      } // end if
   else
      {
      OutputFailure("ReadTemperature2");
      } //

   return;

   }

// Cmd 90
void TestReadStatus(DRoboClawFS& RC)
   {
   Function("ReadStatus");
   DRoboClawFS::EStatus eStatus;
   if (RC.ReadStatus(eStatus))
      {
      cout << "RoboClaw Status: " << static_cast<uint16_t>(eStatus) << " " << RC.GetStatusString(eStatus) << endl;
      } // end if
   else
      {
      OutputFailure("ReadStatus");
      } // end else

   return;

   }

// Cmds 91, 92, & 93
void TestEncoderMode(DRoboClawFS& RC)
   {
   DRoboClawFS::EncoderMode eMode1(true, DRoboClawFS::EncoderMode::eAbsolute);
   DRoboClawFS::EncoderMode eMode2(true, DRoboClawFS::EncoderMode::eAbsolute);

   Function("SetEncoderMode1");
   if (!RC.SetEncoder1Mode(eMode1))
      {
      OutputFailure("SetEncoder1Mode");
      } // end if

   Function("SetEncoder2Mode");
   if (!RC.SetEncoder2Mode(eMode1))
      {
      OutputFailure("SetEncoder2Mode");
      } // end if

   Function("ReadEncoderMode");
   if (RC.ReadEncoderMode(eMode1, eMode2))
      {
      std::string strType = (eMode1.m_eType == DRoboClawFS::EncoderMode::eQuadrature) ? "Quadrature" : "Absolute";
      cout << "Encoder1 Mode: " << eMode1.m_bRC_AnalogEnabed << ", " << strType << endl;
      strType = (eMode2.m_eType == DRoboClawFS::EncoderMode::eQuadrature) ? "Quadrature" : "Absolute";
      cout << "Encoder2 Mode: " << eMode2.m_bRC_AnalogEnabed << ", " << strType << endl;
      } // end if
   else
      {
      OutputFailure("ReadEncoderMode");
      } // end else

   return;

   }

// Cmds 133, 134, 135, & 136
void TestMaxCurrentLimits(DRoboClawFS& RC)
   {
   Function("SetM1MaxCurrentLimit");
   if (!RC.SetM1MaxCurrentLimit(7.0f, 3.0f))
      {
      OutputFailure("SetM1MaxCurrentLimit");
      } // end if

   Function("SetM2MaxCurrentLimit");
   if (!RC.SetM2MaxCurrentLimit(6.0f, 2.0f))
      {
      OutputFailure("SetM2MaxCurrentLimit");
      } // end if

   float fMin, fMax;
   Function("ReadM1MaxCurrentLimit");
   if (RC.ReadM1MaxCurrentLimit(fMax, fMin))
      {
      cout << "M1 Max: " << fMax  << ", Min: " << fMin << endl;
      } // end if
   else
      {
      OutputFailure("ReadM1MaxCurrentLimit");
      } // end else

   Function("ReadM2MaxCurrentLimit");
   if (RC.ReadM2MaxCurrentLimit(fMax, fMin))
      {
      cout << "M2 Max: " << fMax  << ", Min: " << fMin << endl;
      } // end if
   else
      {
      OutputFailure("ReadM2MaxCurrentLimit");
      } // end else

   return;

   }

// Cmds 148 & 149
void TestPWMMode(DRoboClawFS& RC)
   {
   Function("SetPWMMode");
   if (!RC.SetPWMMode(DRoboClawFS::EPWMMode::eSignMagnitude))
      {
      OutputFailure("SetPWMMode");
      } // end if

   Function("ReadPWMMode");
   DRoboClawFS::EPWMMode eMode;
   if (RC.ReadPWMMode(eMode))
      {
      cout << "PWM Mode: " << RC.GetPWMModeString(eMode) << endl;
      } // end if
   else
      {
      OutputFailure("ReadPWMMode");
      } // end else

   return;

   }

// Cmds 28, 29, 55, & 56
void TestVelocityPIDSettings(DRoboClawFS& RC)
   {
   uint32_t uQPPS, uP, uD, uI;

   if (RC.ReadM1VelocityPIDConstants(uP, uI, uD, uQPPS))
      {
      cout << "Default M1 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM1VelocityPID");
      } // end else

   if (RC.ReadM2VelocityPIDConstants(uP, uI, uD, uQPPS))
      {
      cout << "Default M2 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM2VelocityPID");
      } // end else

   if (RC.SetM1VelocityPIDConstants(uP = 1, uI = 2, uD = 3, uQPPS = 4))
      {
      cout << "Set M1 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("SetM1VelocityPID");
      } // end else

   if (RC.SetM2VelocityPIDConstants(uP = 5, uI = 6, uD = 7, uQPPS = 8))
      {
      cout << "Set M2 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("SetM2VelocityPID");
      } // end else

   if (RC.ReadM1VelocityPIDConstants(uP, uI, uD, uQPPS))
      {
      cout << "M1 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM1VelocityPID");
      } // end else

   if (RC.ReadM2VelocityPIDConstants(uP, uI, uD, uQPPS))
      {
      cout << "M2 Velocity PID Constants (PID QPPS): "
         << uP << ", " << uI << ", " << uD << ", " << uQPPS << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM2VelocityPID");
      } // end else

   return;

   }

// Cmds 61, 62, 63, &  64
void TestPositionPIDSettings(DRoboClawFS& RC)
   {
   uint32_t uP, uD, uI, uIWindup, uDeadzone;
   int32_t nMinPos, nMaxPos;

   if (RC.ReadM1PositionPIDConstants(uP, uI, uD, uIWindup, uDeadzone, nMinPos, nMaxPos))
      {
      cout << "Default M1 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM1PositionPID");
      } // end else

   if (RC.ReadM2PositionPIDConstants(uP, uI, uD, uIWindup, uDeadzone, nMinPos, nMaxPos))
      {
      cout << "Default M2 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM2PositionPID");
      } // end else

   if (RC.SetM1PositionPIDConstants(uP = 1, uI = 2, uD = 3, uIWindup = 4, uDeadzone = 5, nMinPos = 6, nMaxPos = 7))
      {
      cout << "Set M1 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("SetM1PositionPID");
      } // end else

   if (RC.SetM2PositionPIDConstants(uP = 8, uI = 9, uD = 10, uIWindup = 11, uDeadzone = 12, nMinPos = 13, nMaxPos = 14))
      {
      cout << "Set M2 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("SetM2PositionPID");
      } // end else

   if (RC.ReadM1PositionPIDConstants(uP, uI, uD, uIWindup, uDeadzone, nMinPos, nMaxPos))
      {
      cout << "M1 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM1PositionPID");
      } // end else

   if (RC.ReadM2PositionPIDConstants(uP, uI, uD, uIWindup, uDeadzone, nMinPos, nMaxPos))
      {
      cout << "M2 Position PID Constants (PID IWindup Deadzone MinPos MaxPos): "
         << uP << ", " << uI << ", " << uD << ", " << uIWindup << ", " << uDeadzone  << ", "
         << nMinPos << ", " << nMaxPos << ", " << endl;
      } // end if
   else
      {
      OutputFailure("ReadM2PositionPID");
      } // end else

   return;

   }

void TestMotorDirection(DRoboClawFS& RC)
   {
   RC.ResetEncoders();

   RC.DriveM1SignedDutyCycle(100.0f);

   for (int i = 0 ; i < 100 ; i++)
      {
      uint32_t uCount;
      DRoboClawFS::EncoderStatus eStatus;
      int32_t nSpeed;
      DRoboClawFS::EDirection eDirection;

      if (RC.ReadEncoder1(uCount, eStatus) && RC.ReadEncoder1RawSpeed(nSpeed, eDirection))
         {
         PrintEncoderStatus(1, uCount, eStatus);
         PrintMotorSpeed(1, nSpeed, eDirection);
         } // end if
      else
         {
         cout << "Encoder Error" << endl;
         } // end else

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } // end for

   RC.M1Forward(0);

   return;

   }


int main(int argc, char *argv[])
   {
   cout << "Starting RoboClawTest" << endl;

   DPanTilt RC(DPanTilt::eTilt1Pan2, 1856, 1216);

   if (argc > 1)
      {
      RC.SetDevice(argv[1]);
      } // end if
   else
      {
      RC.SetDevice("COM3");
      } // end else

   if (RC.OpenPort())
      {
      cout << "Port Open" << endl;

      TestMotorDirection(RC);
      } // end if
   else
      {
       cout << "Port Open Failed!!!" << endl;
      } // end else

   cout << "Exiting RoboClawTest" << endl;

   return (0);

   }
