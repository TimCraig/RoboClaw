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

Author: Tim Craig (Druai Robotics) 2017

*/

#if !defined(DIFFDRIVE_H)
#define DIFFDRIVE_H

/*****************************************************************************
*
***  class DDiffDrive
*
*****************************************************************************/

class DDiffDrive : public DRoboClawFS
   {
   public:
      // How to configure which motor controls which axis
      enum EMotorConfig { eLeft1Right2, eRight1Left2 };

      // Generic order for array parameters
      enum { eLeft, eRight };

      DDiffDrive(EMotorConfig eMotorConfig, int32_t nLeftCntsPerRev, int32_t nRightCntsPerRev,
           const std::string strTTYDevice = "/dev/roboclaw", uint8_t uAddr = 0x80,
           uint32_t uReadTimeout = 100);

      DDiffDrive(const DDiffDrive&& src) = delete;

      virtual ~DDiffDrive() = default;

      DDiffDrive& operator=(const DDiffDrive& rhs) = delete;
      DDiffDrive& operator=(const DDiffDrive&& rhs) = delete;

      // Motor configuration
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

   protected:
      int32_t m_nLeftMotor;
      int32_t m_nRightMotor;

   private:

   }; // end of class DDiffDrive


#endif // DIFFDRIVE_H
