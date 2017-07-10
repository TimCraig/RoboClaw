/*****************************************************************************
*** DiffDrive.h
*****************************************************************************/

/*
This class extends the DRoboClaw interface to provide common functionality for
controlling two motors in a differential drive setup (specifically for a robot).

Since RoboClaw funcions are called specifically for a motor axis rather than
passing the motor number, this code is written to be configured at runtime
configuring which motor controls the left and right motors.  Sometimes this
configuration may be beyond the of the software developer or may need to be
reconfigured in the field.

While this is a generic appliction, some of the functions involving Twist are
implemented with robots using ROS in mind.  (Robot Operating System www.ROS.org)

Length specifications are left generic but in ROS the unit of Length is in meters.
All length variables are to be given in the SAME units.

Author: Tim Craig (Druai Robotics) 2017

*/

#if !defined(DIFFDRIVE_H)
#define DIFFDRIVE_H

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include "RoboClawFS.h"

/*****************************************************************************
*
***  class DDiffDrive
*
*****************************************************************************/

class DDiffDrive : public DRoboClawFS
   {
   public:
      using Base = DRoboClawFS;

      // How to configure which motor controls which axis
      enum EMotorConfig { eLeft1Right2, eRight1Left2 };

      // Generic order for array parameters
      enum { eLeft, eRight };

      DDiffDrive(EMotorConfig eMotorConfig, double dWheelDiameter, int32_t nCntsPerRev,
           double dWheelBase,
           const std::string strTTYDevice = "/dev/roboclaw", uint8_t uAddr = 0x80,
           uint32_t uReadTimeout = 100);

      DDiffDrive(const DDiffDrive&& src) = delete;

      virtual ~DDiffDrive() = default;

      DDiffDrive& operator=(const DDiffDrive& rhs) = delete;
      DDiffDrive& operator=(const DDiffDrive&& rhs) = delete;

      /*****************************************************************************
      *
      ***  class MotorMotionParms
      *
      * Class for grouping motor motion parameters to make it feasible to
      * programatically specify the motor parameters for the RoboClaw's complex
      * motion commands.
      *
      *****************************************************************************/

      class MotorMotionParms
         {
         public:
            MotorMotionParms() = default;
            MotorMotionParms(const MotorMotionParms& src) = delete;
            MotorMotionParms(const MotorMotionParms&& src) = delete;
            ~MotorMotionParms() = default;
            MotorMotionParms& operator=(const MotorMotionParms& rhs) = delete;
            MotorMotionParms& operator=(const MotorMotionParms&& rhs) = delete;

            int32_t m_nSpeed;
            uint32_t m_uAccel;
            uint32_t m_uDecel;
            uint32_t m_uDistance;
         }; // end of class MotorMotionParms

      /*****************************************************************************
      *** Motor Parameters
      *****************************************************************************/

      void ConfigMotors(EMotorConfig eMotorConfig);

      void GetMotorConfig(int& nLeftMotor, int& nRightMotor) const
         {
         nLeftMotor = GetLeftMotor();
         nRightMotor = GetRightMotor();
         return;
         }

      int GetLeftMotor() const
         {
         return (m_nLeftMotor + 1);
         }

      int GetRightMotor() const
         {
         return (m_nRightMotor + 1);
         }

      MotorMotionParms& GetLeftMotorMotionParams()
         {
         return (m_MotionParms[m_nLeftMotor]);
         }

      MotorMotionParms& GetRightMotorMotionParams()
         {
         return (m_MotionParms[m_nRightMotor]);
         }

      /*****************************************************************************
      *** Encoder handling functions
      *****************************************************************************/

      bool ReadPositionCnts(int32_t& nLeftCounts, int32_t& nRightCounts) const
         {
         bool bRet = false;
         if (m_nLeftMotor == 0)
            {
            bRet = ReadEncodersCounts(nLeftCounts, nRightCounts);
            } // end if
         else
            {
            bRet = ReadEncodersCounts(nRightCounts, nLeftCounts);
            } // end else
         return (bRet);
         }

      bool ReadLeftEncoderCnts(int32_t& nCounts, EncoderStatus& Status) const
         {
         return (ReadMotorEncoder(m_nLeftMotor, nCounts, Status));
         }

      bool ReadRightEncoderCnts(int32_t& nCounts, EncoderStatus& Status) const
         {
         return (ReadMotorEncoder(m_nRightMotor, nCounts, Status));
         }

      // Convenience functions for setting encoder counts by name
      bool SetLeftEncoderCounts(int32_t nCounts) const
         {
         return (SetMotorEncoder(m_nLeftMotor, nCounts));
         }

      bool SetRightEncoderCounts(int32_t nCounts) const
         {
         return (SetMotorEncoder(m_nRightMotor, nCounts));
         }

      bool ResetLeftEncoder() const
         {
         return (SetLeftEncoderCounts(0));
         }

      bool ResetRightEncoder() const
         {
         return (SetRightEncoderCounts(0));
         }

      bool ReadEncoders(int32_t& nLeftCounts, int32_t& nRightCounts) const
         {
         int32_t nCounts[2];
         bool bRet = Base::ReadEncodersCounts(nCounts[0], nCounts[1]);
         if (bRet)
            {
            nLeftCounts = nCounts[m_nLeftMotor];
            nRightCounts = nCounts[m_nRightMotor];
            } // end if
         return (bRet);
         }

      // Slightly more efficient version of ReadEncoders returning an array
      bool ReadEncoders(int32_t nCounts[2]) const
         {
         return (Base::ReadEncodersCounts(nCounts[0], nCounts[1]));
         }

      /*****************************************************************************
      *** Simple Motion Commands
      *****************************************************************************/

      // Speed is in Counts/Second signed for direction
      bool DriveStraight(int32_t nSpeed) const
         {
         return (Base::DriveBothSignedSpeedAccel(nSpeed, nSpeed, m_uDefaultAccel));
         }

      // Speed is in Length/Second signed for direction
      bool DriveStraight(double dSpeed) const
         {
         return (DriveStraight(LengthToCnts(dSpeed)));
         }

      // Spin with angular velocity in Radians/Sec
      // Turning left is positive
      bool SpinRPS(double dOmega) const;

      // Spin with angular velocity in Degrees/Sec
      // Turning left is positive
      bool SpinDPS(double dOmega) const
         {
         return (SpinRPS(DegToRad(dOmega)));
         }

      // Speed is in Counts/Second signed for direction
      // Acceleration is unsigned
      bool DriveStraightAccel(int32_t nSpeed, uint32_t uAccel) const
         {
         return (Base::DriveBothSignedSpeedAccel(nSpeed, nSpeed, uAccel));
         }

      // Speed is in Length/Second signed for direction
      // Acceleration is unsigned
      bool DriveStraightAccel(double dSpeed, uint32_t uAccel) const
         {
         int32_t nSpeed = LengthToCnts(dSpeed);
         return (DriveStraightAccel(nSpeed, uAccel));
         }

      /*****************************************************************************
      *** Twist Commands (ROS primary motion command)
      *****************************************************************************/

      // The only parts of a Twist command that are valid for a Differential Drive Robot are:
      // x-axis (forward linear speed) and the z-axis rotational velocity.
      // Linear Speed is in EncoderCounts/Second.
      bool TwistRPS(int32_t nLinearSpeed, double dOmega, bool bAdjustAccel = true);

      // Linear Speed is in Length/Second (Meters/Second in ROS) and Angular Speed is in Radians/Second
      // This is the default units in ROS
      bool TwistRPS(double dLinearSpeed, double dOmega, bool bAdjustAccel = true) const
         {
         return (TwistRPS(LengthToCnts(dLinearSpeed), dOmega, bAdjustAccel));
         }

      // Twist with Angular Velocity in Degrees/Second
      bool TwistDPS(int32_t nLinearSpeed, double dOmega, bool bAdjustAccel = true) const
         {
         return (TwistRPS(nLinearSpeed, DegToRad(dOmega), bAdjustAccel));
         }

      // Twist with Linear Velocity in Length/Second and Angular Velocity in Degrees/Second
      bool TwistDPS(double dLinearSpeed, double dOmega, bool bAdjustAccel = true) const
         {
         return (TwistRPS(LengthToCnts(dLinearSpeed), DegToRad(dOmega), bAdjustAccel));
         }

      /*****************************************************************************
      *** Turns with Radius Specified
      *****************************************************************************/

      /*****************************************************************************
      *** Wheel and Distance Parameters
      *****************************************************************************/

      int32_t GetCountsPerRev() const
         {
         return (m_nCntsPerRev);
         }

      double GetWheelDiameter() const
         {
         return (m_dWheelDiameter);
         }

      double GetWheelCircumference() const
         {
         return (m_dWheelCircumference);
         }

      double GetCountLength() const
         {
         return (m_dCountLength);
         }

      double CntsToLength(int32_t nCnts) const
         {
         return (nCnts * m_dCountLength);
         }

      int32_t LengthToCnts(double dLength) const
         {
         return (static_cast<int32_t>(dLength / m_dCountLength));
         }

   protected:
      // Have a persistent set of parameters since some may not change frequently
      MotorMotionParms m_MotionParms[2];

      // Motor configuration
      int m_nLeftMotor;
      int m_nRightMotor;

      // Encoder counts per full revolution
      int32_t m_nCntsPerRev;

      // Wheel parameters
      double m_dWheelDiameter;
      double m_dWheelCircumference;
      double m_dCountLength;
      double m_dWheelBase;

      // Default speed, acceleration, and decelration values
      int32_t m_nDefaultSpeed;
      uint32_t m_uDefaultAccel;
      uint32_t m_uDefaultDecel;

      // Handy math constants
      static constexpr double dPi = 3.14159265358979323846;
      static constexpr double dDegPerRad = 180.0 / dPi;

      // Angle conversion functions
      static constexpr double DegToRad(double dDegrees)
         {
         return (dDegrees / dDegPerRad);
         }

      static constexpr double RadToDeg(double dRadians)
         {
         return (dRadians * dDegPerRad);
         }

   private:

   }; // end of class DDiffDrive


#endif // DIFFDRIVE_H
