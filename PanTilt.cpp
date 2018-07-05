/******************************************************************************
*** PanTilt.cpp
******************************************************************************/

/*****************************************************************************
 *******************************  I N C L U D E  ******************************
 *****************************************************************************/

#include "PanTilt.h"

/*****************************************************************************
 ***  class DPanTilt
 ****************************************************************************/

/******************************************************************************
*
***  DPanTilt::DPanTilt
*
******************************************************************************/

DPanTilt::DPanTilt(EMotorConfig eMotorConfig, int32_t nPanCntsPerRev, int32_t nTiltCntsPerRev,
      const std::string strTTYDevice /* = /dev/roboclaw */,
      uint8_t uAddr /* = 0x80 */, uint32_t uReadTimeout /* = 100 */)
      : DRoboClawFS(strTTYDevice, uAddr, uReadTimeout)
   {
   ConfigMotors(eMotorConfig);

   SetPanCountsPerRev(nPanCntsPerRev);
   SetTiltCountsPerRev(nTiltCntsPerRev);

   return;

   } // end of method DPanTilt::DPanTilt

/******************************************************************************
*
***  DPanTilt::ConfigMotors
*
* Configure which motor runs the pan and tilt axes.
*
* There may be cases where the user doesn't have a choice as to how the axes
* are wired to the RoboClaw.  This allows the motor configuration to be set at
* runtime with no need to change the code and recompile.
*
******************************************************************************/

void DPanTilt::ConfigMotors(EMotorConfig eMotorConfig)
   {
   switch (eMotorConfig)
      {
      case ePan1Tilt2:
         m_nPanMotor = M1;
         m_nTiltMotor =M2;
         break;

      case eTilt1Pan2:
         m_nPanMotor = M2;
         m_nTiltMotor = M1;
         break;

      default:
         break;
      } // end switch

   return;

   } // end of method DPanTilt::ConfigMotors

/******************************************************************************
*
***  DPanTilt::MoveToCnt
*
* Drive both axes to the specified position using specified motion parameters.
*
******************************************************************************/

bool DPanTilt::MoveToCnt(int32_t nPan, int32_t nPanSpeed, uint32_t uPanAccel, uint32_t uPanDecel,
      int32_t nTilt, int32_t nTiltSpeed, uint32_t uTiltAccel, uint32_t uTiltDecel,
      bool bImmediate /* = true */) const
   {
   bool bRet = false;

   nPan = boost::algorithm::clamp(nPan, m_Parms[m_nPanMotor].m_nMin, m_Parms[m_nPanMotor].m_nMax);
   nTilt = boost::algorithm::clamp(nTilt, m_Parms[m_nTiltMotor].m_nMin, m_Parms[m_nTiltMotor].m_nMax);

   if (m_nPanMotor == 0)
      {
      bRet = DriveBothSignedSpeedAccelDecelPos(nPanSpeed, uPanAccel, uPanDecel, nPan,
            nTiltSpeed, uTiltAccel, uTiltDecel, nTilt, bImmediate);
      } // end if
   else
      {
      bRet = DriveBothSignedSpeedAccelDecelPos(nTiltSpeed, uTiltAccel, uTiltDecel,
            nTilt, nPanSpeed, uPanAccel, uPanDecel, nPan, bImmediate);
      } // end else

   return (bRet);

   } // end of method DPanTilt::MoveToCnt

/******************************************************************************
*
***  DPanTilt::MoveToCnt
*
* Drive both axes to the specified position trying to get a straight line in
* angle space so that both axes arrive at their final positions at the same time.
* The specified speed is along the path so speeds need to be calculated for
* each axis.  Default acceleration and deceleartion values are used but
* (currently) are not used in the timing calculation.
*
* Even though a count in pan or tilt may represent different angualar distances,
* once we're working in counts and speeds are in counts/second, that all comes
* out in the wash.  The only variation will come from the accel and decel vales.
* Startup and stopping will have more of an effect on the overall trajectory
* for small moves than long ones.
*
******************************************************************************/

bool DPanTilt::MoveToCnt(int32_t nPan, int32_t nTilt, int32_t nSpeed,
      bool bImmediate /* = true */) const
   {
   bool bRet = true;

   nPan = boost::algorithm::clamp(nPan, m_Parms[m_nPanMotor].m_nMin, m_Parms[m_nPanMotor].m_nMax);
   nTilt = boost::algorithm::clamp(nTilt, m_Parms[m_nTiltMotor].m_nMin, m_Parms[m_nTiltMotor].m_nMax);

   double dDiag = std::sqrt((nPan * nPan) + (nTilt * nTilt));
   if (dDiag != 0.0)
      {
      // dDiag will be zero ONLY if BOTH nPan and nTile are zero
      int32_t nPanSpeed = static_cast<int32_t>((nPan / dDiag) * nSpeed);
      int32_t nTiltSpeed = static_cast<int32_t>((nTilt / dDiag) * nSpeed);

      bRet = MoveToCnt(nPan, nPanSpeed, nTilt, nTiltSpeed, bImmediate);
      } // end if

   return (bRet);

   } // end of method DPanTilt::MoveToCnt

