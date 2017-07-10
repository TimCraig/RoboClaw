/*****************************************************************************
*** DiffDrive.cpp
*****************************************************************************/

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include "DiffDrive.h"

/*****************************************************************************
 ***  class DDiffDrive
 ****************************************************************************/

/******************************************************************************
*
***  DDiffDrive::DDiffDrive
*
******************************************************************************/

DDiffDrive::DDiffDrive(EMotorConfig eMotorConfig, double dWheelDiameter, int32_t nCntsPerRev,
      double dWheelBase, const std::string strTTYDevice /* = /dev/roboclaw */,
      uint8_t uAddr /* = 0x80 */, uint32_t uReadTimeout /* = 100 */)
      : DRoboClawFS(strTTYDevice, uAddr, uReadTimeout)
   {
   m_dWheelBase = dWheelBase;
   m_dWheelDiameter = dWheelDiameter;
   m_nCntsPerRev = nCntsPerRev;
   m_dWheelCircumference = dPi * m_dWheelDiameter;
   m_dCountLength = m_dWheelCircumference / nCntsPerRev;

   m_nDefaultSpeed = 0;
   m_uDefaultAccel = 0;
   m_uDefaultDecel = 0;

   ConfigMotors(eMotorConfig);

   return;

   } // end of method DDiffDrive::DDiffDrive

/******************************************************************************
*
***  DDiffDrive::ConfigMotors
*
* Configure which motor runs the Left and Right axes.
*
* There may be cases where the user doesn't have a choice as to how the axes
* are wired to the RoboClaw.  This allows the motor configuration to be set at
* runtime with no need to change the code and recompile.
*
******************************************************************************/

void DDiffDrive::ConfigMotors(EMotorConfig eMotorConfig)
   {
   switch (eMotorConfig)
      {
      case eLeft1Right2:
         m_nLeftMotor = M1;
         m_nRightMotor = M2;
         break;

      case eRight1Left2:
         m_nLeftMotor = M2;
         m_nRightMotor = M1;
         break;

      default:
         break;
      } // end switch

   return;

   } // end of method DDiffDrive::ConfigMotors

/******************************************************************************
*
***  DDiffDrive::SpinRPS
*
* Spin in place with an angular velocity given in Radians/Second.
* Following ROS conventions (x axis forward, z axis up), positive turns to the
* left viewed from above.
*
******************************************************************************/

bool DDiffDrive::SpinRPS(double dOmega) const
   {
   int32_t nSpeed = ((m_dWheelBase / 2) / m_dCountLength) * dOmega;

   // Wheel speeds are equal in magnitude and opposite in sign
   int32_t nWheelSpeed[2];
   nWheelSpeed[m_nLeftMotor] = -nSpeed;
   nWheelSpeed[m_nRightMotor] = nSpeed;

   return (Base::DriveBothSignedSpeedAccel(nWheelSpeed[M1], nWheelSpeed[M2], m_uDefaultAccel));

   } // end of method DDiffDrive::SpinRPS

/******************************************************************************
*
***  DDiffDrive::TwistRPS
*
* For a Differential Drive Robot the valid parts of a Twist message are
* linear velocity (along the x-axis) and angular velocity (around the z-axis).
* The angular velicity is basically a Spin that is added to the linear velocity
* resulting in a path that's a circular arc (for nonzero angular velocity).
*
* This version specifies Linear Speed in EncoderCounts/Second and Angular
* Velocity in Radians/Second.
*
* The Adjust Acceleration flag is used to compensate for the inside wheel moving
* slower than the outside wheel.  Hence, it needs to accelrate more slowly to
* follow the circular arc.
*
******************************************************************************/

bool DDiffDrive::TwistRPS(int32_t nLinearSpeed, double dOmega, bool bAdjustAccel /* = true */)
   {
   int32_t nAngularSpeed = ((m_dWheelBase / 2) / m_dCountLength) * dOmega;

   // Combine the two components of wheel speed
   m_MotionParms[m_nLeftMotor].m_nSpeed = nLinearSpeed - nAngularSpeed;
   m_MotionParms[m_nRightMotor].m_nSpeed = nLinearSpeed + nAngularSpeed;

   m_MotionParms[m_nLeftMotor].m_uAccel = m_uDefaultAccel;
   m_MotionParms[m_nRightMotor].m_uAccel = m_uDefaultAccel;

   if (bAdjustAccel)
      {
      int32_t nSpeed[2];
      nSpeed[m_nLeftMotor] = std::abs(m_MotionParms[m_nLeftMotor].m_nSpeed);
      nSpeed[m_nRightMotor] = std::abs(m_MotionParms[m_nRightMotor].m_nSpeed);

      if ((nSpeed[m_nLeftMotor] > nSpeed[m_nRightMotor]) && (nSpeed[m_nLeftMotor] != 0))
         {
         m_MotionParms[m_nRightMotor].m_uAccel =
               (nSpeed[m_nRightMotor] * m_uDefaultAccel) / nSpeed[m_nLeftMotor];
         } // end if
      else if ((nSpeed[m_nRightMotor] > nSpeed[m_nLeftMotor]) && (nSpeed[m_nRightMotor] != 0))
         {
         m_MotionParms[m_nLeftMotor].m_uAccel =
               (nSpeed[m_nLeftMotor] * m_uDefaultAccel) / nSpeed[m_nRightMotor];
         }  // end else if
      } // end if

   return (Base::DriveBothSignedSpeedIndividualAccel(m_MotionParms[M1].m_nSpeed, m_MotionParms[M1].m_uAccel,
         m_MotionParms[M2].m_nSpeed, m_MotionParms[M2].m_uAccel));

   } // end of method DDiffDrive::TwistRPS
