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
* Usually nLeftCntsPerRev and nRightCntsPerRev will be equal.  However, we
* allow different values for the unusual cases where the wheels are actually
* different or if the user is both able and fussy enough to measure subtle
* differences in manufacturing.
*
******************************************************************************/

DDiffDrive::DDiffDrive(EMotorConfig eMotorConfig, int32_t nLeftCntsPerRev,
      int32_t nRightCntsPerRev, const std::string strTTYDevice /* = /dev/roboclaw */,
      uint8_t uAddr /* = 0x80 */, uint32_t uReadTimeout /* = 100 */)
      : DRoboClawFS(strTTYDevice, uAddr, uReadTimeout))
   {

   return;

   } // end of method DDiffDrive::DDiffDrive
