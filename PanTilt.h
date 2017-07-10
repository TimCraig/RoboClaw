/******************************************************************************
*** PanTilt.h
******************************************************************************/

/*
This class extends the DRoboClaw interface to provide common functionality for
controlling a pan and tilt camera platform.

Since RoboClaw funcions are called specifically for a motor axis rather than
passing the motor number, this code is written to be configured at runtime
configuring which motor controls the pan and tilt axes.  Sometimes this
configuration may be beyond the of the software developer or may need to be
reconfigured in the field.

Author: Tim Craig (Druai Robotics) 2017

*/

#if !defined(PANTILT_H)
#define PANTILT_H

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include "RoboClawFS.h"
#include <boost/algorithm/clamp.hpp>

/*****************************************************************************
*
***  class DPanTilt
*
* This class implements pan and tilt control for a motorized camera platform
* using the Ion Motion Control RoboClaw.  It builds on the basic motor control
* functionality of the DRoboClawFS class.
*
*****************************************************************************/

class DPanTilt : public DRoboClawFS
   {
   public:
      // How to configure which motor controls which axis
      enum EMotorConfig { ePan1Tilt2, eTilt1Pan2 };

      // Generic order for array parameters
      enum { ePan, eTilt };

      using Position = int32_t[2];
      using PositionDeg = double[2];
      using PositionRad = double[2];

      DPanTilt(EMotorConfig eMotorConfig, int32_t nPanCntsPerRev, int32_t nTiltCntsPerRev,
            const std::string strTTYDevice = "/dev/roboclaw", uint8_t uAddr = 0x80,
            uint32_t uReadTimeout = 100);

      DPanTilt(const DPanTilt& src) = delete;
      DPanTilt(const DPanTilt&& src) = delete;

      virtual ~DPanTilt() = default;

      DPanTilt& operator=(const DPanTilt& rhs) = delete;
      DPanTilt& operator=(const DPanTilt&& rhs) = delete;

      // Motor configuration
      void ConfigMotors(EMotorConfig eMotorConfig);

      void GetMotorConfig(int& nPanMotor, int& nTiltMotor) const
         {
         nPanMotor = GetPanMotor();
         nTiltMotor = GetTiltMotor();
         return;
         }

      int GetPanMotor() const
         {
         return (m_nPanMotor + 1);
         }

      int GetTiltMotor() const
         {
         return (m_nTiltMotor + 1);
         }

      /*****************************************************************************
      *** Individual axis moves with default speed, acceleration, and deceleration
      *****************************************************************************/

      bool PanToDeg(double dDegrees, bool bImmediate = true) const
         {
         return (PanToCnt(PanDegToCnts(dDegrees), bImmediate));
         }

      bool PanToRad(double dRadians, bool bImmediate = true) const
         {
         return (PanToCnt(PanRadToCnts(dRadians), bImmediate));
         }

      bool PanToCnt(int32_t nCount, bool bImmediate = true) const
         {
         return (PanToCnt(nCount, m_Parms[m_nPanMotor].m_nDefaultSpeed,
               m_Parms[m_nPanMotor].m_uDefaultAccel, m_Parms[m_nPanMotor].m_uDefaultDecel, bImmediate));
         }

      bool TiltToDeg(double dDegrees, bool bImmediate = true) const
         {
         return (TiltToCnt(TiltDegToCnts(dDegrees), bImmediate));
         }

      bool TiltToRad(double dRadians, bool bImmediate = true) const
         {
         return (TiltToCnt(TiltRadToCnts(dRadians), bImmediate));
         }

      bool TiltToCnt(int32_t nCount, bool bImmediate = true) const
         {
         return (TiltToCnt(nCount, m_Parms[m_nTiltMotor].m_nDefaultSpeed,
               m_Parms[m_nTiltMotor].m_uDefaultAccel, m_Parms[m_nTiltMotor].m_uDefaultDecel, bImmediate));
         }

      /*****************************************************************************
      *** Move both axes with the default speed, acceleration, and deceleration
      *****************************************************************************/

      bool MoveToDeg(double dPan, double dTilt, bool bImmediate = true) const
         {
         return (MoveToCnt(PanDegToCnts(dPan), TiltDegToCnts(dTilt), bImmediate));
         }

      bool MoveToRad(double dPan, double dTilt, bool bImmediate = true) const
         {
         return (MoveToCnt(PanRadToCnts(dPan), TiltRadToCnts(dTilt), bImmediate));
         }

      bool MoveToCnt(int32_t nPan, int32_t nTilt, bool bImmediate = true) const
         {
         return (MoveToCnt(nPan, m_Parms[m_nPanMotor].m_nDefaultSpeed, m_Parms[m_nPanMotor].m_uDefaultAccel,
               m_Parms[m_nPanMotor].m_uDefaultDecel, nTilt, m_Parms[m_nTiltMotor].m_nDefaultSpeed,
               m_Parms[m_nTiltMotor].m_uDefaultAccel, m_Parms[m_nTiltMotor].m_uDefaultDecel, bImmediate));
         }

      /*****************************************************************************
      *** Move both axes with the specified speed but default acceleration, and deceleration
      *****************************************************************************/

      bool MoveToDeg(double dPan, int32_t nPanSpeed, double dTilt, int32_t nTiltSpeed, bool bImmediate = true) const
         {
         return (MoveToCnt(PanDegToCnts(dPan), nPanSpeed, TiltDegToCnts(dTilt), nTiltSpeed, bImmediate));
         }

      bool MoveToRad(double dPan, int32_t nPanSpeed, double dTilt, int32_t nTiltSpeed, bool bImmediate = true) const
         {
         return (MoveToCnt(PanRadToCnts(dPan), nPanSpeed, TiltRadToCnts(dTilt), nTiltSpeed, bImmediate));
         }

      bool MoveToCnt(int32_t nPan, int32_t nPanSpeed, int32_t nTilt, int32_t nTiltSpeed, bool bImmediate = true) const
         {
         return (MoveToCnt(nPan, nPanSpeed, m_Parms[m_nPanMotor].m_uDefaultAccel,
               m_Parms[m_nPanMotor].m_uDefaultDecel, nTilt, nTiltSpeed,
               m_Parms[m_nTiltMotor].m_uDefaultAccel, m_Parms[m_nTiltMotor].m_uDefaultDecel, bImmediate));
         }

      /*****************************************************************************
      *** Move individual axes with specified speed, acceleration, and deceleration
      *****************************************************************************/

      bool PanToDeg(double dDegrees, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         return (PanToCnt(PanDegToCnts(dDegrees), nSpeed, uAccel, uDecel, bImmediate));
         }

      bool PanToRad(double dRadians, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         return (PanToCnt(PanRadToCnts(dRadians), nSpeed, uAccel, uDecel, bImmediate));
         }

      bool PanToCnt(int32_t nCount, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         nCount = boost::algorithm::clamp(nCount, m_Parms[m_nPanMotor].m_nMin, m_Parms[m_nPanMotor].m_nMax);
         return (DriveMotorSignedSpeedAccelDecelPos(m_nPanMotor, nSpeed, uAccel, uDecel, nCount, bImmediate));
         }

      bool TiltToDeg(double dDegrees, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         return (TiltToCnt(TiltDegToCnts(dDegrees), nSpeed, uAccel, uDecel, bImmediate));
         }

      bool TiltToRad(double dRadians, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         return (TiltToCnt(TiltRadToCnts(dRadians), nSpeed, uAccel, uDecel, bImmediate));
         }

      bool TiltToCnt(int32_t nCount, int32_t nSpeed, uint32_t uAccel, uint32_t uDecel, bool bImmediate = true) const
         {
         nCount = boost::algorithm::clamp(nCount, m_Parms[m_nTiltMotor].m_nMin, m_Parms[m_nTiltMotor].m_nMax);
         return (DriveMotorSignedSpeedAccelDecelPos(m_nTiltMotor, nSpeed, uAccel, uDecel, nCount, bImmediate));
         }

      /*****************************************************************************
      *** Move both axes with specified speed, acceleration, and deceleration
      *****************************************************************************/

      bool MoveToDeg(double dPan, int32_t nPanSpeed, uint32_t uPanAccel, uint32_t uPanDecel,
                     double dTilt, int32_t nTiltSpeed, uint32_t uTiltAccel, uint32_t uTiltDecel,
                     bool bImmediate = true) const
         {
         return (MoveToCnt(PanDegToCnts(dPan), nPanSpeed, uPanAccel, uPanDecel,
            TiltDegToCnts(dTilt), nTiltSpeed, uTiltAccel, uTiltDecel, bImmediate));
         }

      bool MoveToRad(double dPan, int32_t nPanSpeed, uint32_t uPanAccel, uint32_t uPanDecel,
                     double dTilt, int32_t nTiltSpeed, uint32_t uTiltAccel, uint32_t uTiltDecel,
                     bool bImmediate = true) const
         {
         return (MoveToCnt(PanRadToCnts(dPan), nPanSpeed, uPanAccel, uPanDecel,
            TiltRadToCnts(dTilt), nTiltSpeed, uTiltAccel, uTiltDecel, bImmediate));
         }

      bool MoveToCnt(int32_t nPan, int32_t nPanSpeed, uint32_t uPanAccel, uint32_t uPanDecel,
            int32_t nTilt, int32_t nTiltSpeed, uint32_t uTiltAccel, uint32_t uTiltDecel,
            bool bImmediate = true) const;

      /*****************************************************************************
      *** Smooth motion functions
      *****************************************************************************/

      bool MoveToCnt(int32_t nPan, int32_t nTilt, int32_t nSpeed, bool bImmediate = true) const;

      /*****************************************************************************
      *** Encoder handling functions
      *****************************************************************************/

      // Read both encoders for position
      bool ReadPositionDeg(double& dPan, double& dTilt) const
         {
         int32_t nPan, nTilt;
         bool bRet = ReadPositionCnts(nPan, nTilt);
         if (bRet)
            {
            dPan = PanCntsToDeg(nPan);
            dTilt = TiltCntsToDeg(nTilt);
            } // end if

         return (bRet);
         }

      bool ReadPositionRad(double& dPan, double& dTilt) const
         {
         int32_t nPan, nTilt;
         bool bRet = ReadPositionCnts(nPan, nTilt);
         if (bRet)
            {
            dPan = PanCntsToRad(nPan);
            dTilt = TiltCntsToRad(nTilt);
            } // end if

         return (bRet);
         }

      bool ReadPositionCnts(int32_t& nPanCounts, int32_t& nTiltCounts) const
         {
         bool bRet = false;
         if (m_nPanMotor == 0)
            {
            bRet = ReadEncodersCounts(nPanCounts, nTiltCounts);
            } // end if
         else
            {
            bRet = ReadEncodersCounts(nTiltCounts, nPanCounts);
            } // end else
         return (bRet);
         }

      bool ReadPanEncoderDeg(double& dDegrees, EncoderStatus& Status) const
         {
         int32_t nCounts;
         bool bRet = ReadPanEncoderCnts(nCounts, Status);
         if (bRet)
            {
            dDegrees = PanCntsToDeg(nCounts);
            } // end if
         return (bRet);
         }

      bool ReadPanEncoderRad(double& dRadians, EncoderStatus& Status) const
         {
         int32_t nCounts;
         bool bRet = ReadPanEncoderCnts(nCounts, Status);
         if (bRet)
            {
            dRadians = PanCntsToRad(nCounts);
            } // end if
         return (bRet);
         }

      bool ReadPanEncoderCnts(int32_t& nCounts, EncoderStatus& Status) const
         {
         return (ReadMotorEncoder(m_nPanMotor, nCounts, Status));
         }

      bool ReadTiltEncoderDeg(double& dDegrees, EncoderStatus& Status) const
         {
         int32_t nCounts;
         bool bRet = ReadTiltEncoderCnts(nCounts, Status);
         if (bRet)
            {
            dDegrees = TiltCntsToDeg(nCounts);
            } // end if
         return (bRet);
         }

      bool ReadTiltEncoderRad(double& dRadians, EncoderStatus& Status) const
         {
         int32_t nCounts;
         bool bRet = ReadTiltEncoderCnts(nCounts, Status);
         if (bRet)
            {
            dRadians = TiltCntsToRad(nCounts);
            } // end if
         return (bRet);
         }

      bool ReadTiltEncoderCnts(int32_t& nCounts, EncoderStatus& Status) const
         {
         return (ReadMotorEncoder(m_nTiltMotor, nCounts, Status));
         }

      // Convenience functions for setting encoder counts by name
      bool SetPanEncoderCounts(int32_t nCounts) const
         {
         return (SetMotorEncoder(m_nPanMotor, nCounts));
         }

      bool SetTiltEncoderCounts(int32_t nCounts) const
         {
         return (SetMotorEncoder(m_nTiltMotor, nCounts));
         }

      bool ResetPanEncoder() const
         {
         return (SetPanEncoderCounts(0));
         }

      bool ResetTiltEncoder() const
         {
         return (SetTiltEncoderCounts(0));
         }

      // Handle the encoder constants for the configuration
      void SetPanCountsPerRev(int32_t nCounts)
         {
         m_Parms[m_nPanMotor].m_nCntsPerRev = nCounts;
         m_Parms[m_nPanMotor].m_dDegPerCnt = nCounts / 360.0;
         m_Parms[m_nPanMotor].m_dRadPerCnt = nCounts / (2 * dPi);
         return;
         }

      int32_t GetPanCountsPerRev() const
         {
         return (m_Parms[m_nPanMotor].m_nCntsPerRev);
         }

      void SetTiltCountsPerRev(int32_t nCounts)
         {
         m_Parms[m_nTiltMotor].m_nCntsPerRev = nCounts;
         m_Parms[m_nTiltMotor].m_dDegPerCnt = nCounts / 360.0;
         m_Parms[m_nTiltMotor].m_dRadPerCnt = nCounts / (2 * dPi);
         return;
         }

      int32_t GetTiltCountsPerRev() const
         {
         return (m_Parms[m_nTiltMotor].m_nCntsPerRev);
         }

      void SetPanLimits(int32_t nMinPan, int32_t nMaxPan)
         {
         m_Parms[m_nPanMotor].m_nMin = nMinPan;
         m_Parms[m_nPanMotor].m_nMax = nMaxPan;
         return;
         }

      void GetPanLimits(int32_t& nMinPan, int32_t& nMaxPan) const
         {
         nMinPan = m_Parms[m_nPanMotor].m_nMin;
         nMaxPan = m_Parms[m_nPanMotor].m_nMax;
         return;
         }

      void SetTiltLimits(int32_t nMinTilt, int32_t nMaxTilt)
         {
         m_Parms[m_nTiltMotor].m_nMin = nMinTilt;
         m_Parms[m_nTiltMotor].m_nMax = nMaxTilt;
         return;
         }

      void GetTiltLimits(int32_t& nMinTilt, int32_t& nMaxTilt) const
         {
         nMinTilt = m_Parms[m_nTiltMotor].m_nMin;
         nMaxTilt = m_Parms[m_nTiltMotor].m_nMax;
         return;
         }

      // Handle the default speed, accel, and decel values
      void SetDefaultPanSpeed(int32_t nSpeed)
         {
         m_Parms[m_nPanMotor].m_nDefaultSpeed = nSpeed;
         return;
         }

      int32_t GetDefaultPanSpeed() const
         {
         return (m_Parms[m_nPanMotor].m_nDefaultSpeed);
         }

      void SetDefaultPanAccel(uint32_t uAccel)
         {
         m_Parms[m_nPanMotor].m_uDefaultAccel = uAccel;
         return;
         }

      uint32_t GetDefaultPanAccel() const
         {
         return (m_Parms[m_nPanMotor].m_uDefaultAccel);
         }

      void SetDefaultPanDecel(uint32_t uDecel)
         {
         m_Parms[m_nPanMotor].m_uDefaultDecel = uDecel;
         return;
         }

      uint32_t GetDefaultPanDecel() const
         {
         return (m_Parms[m_nPanMotor].m_uDefaultDecel);
         }

      void SetDefaultTiltSpeed(int32_t nSpeed)
         {
         m_Parms[m_nTiltMotor].m_nDefaultSpeed = nSpeed;
         return;
         }

      int32_t GetDefaultTiltSpeed() const
         {
         return (m_Parms[m_nTiltMotor].m_nDefaultSpeed);
         }

      void SetDefaultTiltAccel(uint32_t uAccel)
         {
         m_Parms[m_nTiltMotor].m_uDefaultAccel = uAccel;
         return;
         }

      uint32_t GetDefaultTiltAccel() const
         {
         return (m_Parms[m_nTiltMotor].m_uDefaultAccel);
         }

      void SetDefaultTiltDecel(uint32_t uDecel)
         {
         m_Parms[m_nTiltMotor].m_uDefaultDecel = uDecel;
         return;
         }

      uint32_t GetDefaultTiltDecel() const
         {
         return (m_Parms[m_nTiltMotor].m_uDefaultDecel);
         }

      // Functions for converting angles to encoder counts
      int32_t PanDegToCnts(double dDegrees) const
         {
         return (static_cast<int32_t>(dDegrees / m_Parms[m_nPanMotor].m_dDegPerCnt));
         }

      int32_t TiltDegToCnts(double dDegrees) const
         {
         return (static_cast<int32_t>(dDegrees / m_Parms[m_nTiltMotor].m_dDegPerCnt));
         }

      int32_t PanRadToCnts(double dRadians) const
         {
         return (static_cast<int32_t>(dRadians / m_Parms[m_nPanMotor].m_dRadPerCnt));
         }

      int32_t TiltRadToCnts(double dRadians) const
         {
         return (static_cast<int32_t>(dRadians / m_Parms[m_nTiltMotor].m_dRadPerCnt));
         }

      // Functions to convert encoder counts to angular values
      double PanCntsToDeg(int32_t nCounts) const
         {
         return (nCounts * m_Parms[m_nPanMotor].m_dDegPerCnt);
         }

      double TiltCntsToDeg(int32_t nCounts) const
         {
         return (nCounts * m_Parms[m_nTiltMotor].m_dDegPerCnt);
         }

      double PanCntsToRad(int32_t nCounts) const
         {
         return (nCounts * m_Parms[m_nPanMotor].m_dRadPerCnt);
         }

      double TiltCntsToRad(int32_t nCounts) const
         {
         return (nCounts * m_Parms[m_nTiltMotor].m_dRadPerCnt);
         }


   protected:

      /*****************************************************************************
      *
      ***  class AxisParms
      *
      * Class for grouping axis values
      *
      *****************************************************************************/

      class AxisParms
         {
         public:
            AxisParms() = default;
            AxisParms(const AxisParms& src) = delete;
            AxisParms(const AxisParms&& src) = delete;
            ~AxisParms() = default;
            AxisParms& operator=(const AxisParms& rhs) = delete;
            AxisParms& operator=(const AxisParms&& rhs) = delete;

            // Encoder counts per full revolution
            int32_t m_nCntsPerRev;

            // Degrees per encoder count
            double m_dDegPerCnt;

            // Radians per encoder count
            double m_dRadPerCnt;

            // Angle limits in encoder counts
            int32_t m_nMin;
            int32_t m_nMax;

            // Default speed, acceleration, and decelration values
            int32_t m_nDefaultSpeed = 0;
            uint32_t m_uDefaultAccel = 0;
            uint32_t m_uDefaultDecel = 0;
         }; // end of class AxisParms

      AxisParms m_Parms[2];

      // Motor configuration
      int m_nPanMotor;
      int m_nTiltMotor;

      // Handy math constants
      static constexpr double dPi = 3.14159265358979323846;
      static constexpr double dDegPerRad = 180.0 / dPi;

      // Angle conversion functions
      static double DegToRad(double dDegrees)
         {
         return (dDegrees / dDegPerRad);
         }

      static double RadToDeg(double dRadians)
         {
         return (dRadians * dDegPerRad);
         }

   private:

   }; // end of class DPanTilt

#endif // PANTILT_H
