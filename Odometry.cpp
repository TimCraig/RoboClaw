/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include "Odometry.h"

/*****************************************************************************
 ***  class DOdometry
 ****************************************************************************/

/******************************************************************************
*
***  DOdometry::UpdatePose
*
* Update the Pose given the distance traveled by the Left and Right Weels
*
******************************************************************************/

void DOdometry::UpdatePose(double dLeft, double dRight)
   {
   // Distance the center travels is just the average of the two wheels
   double dLength = (dLeft + dRight) / 2.0;

   // The delta distance determines the turn radius and heading angle change
   // Left turns are positive (Right > Left)
   double dDeltaLen = dRight - dLeft;

   // In practice, the minimum delta is the distance of one encoder tick
   if (std::abs(dDeltaLen) > 1.0e-6)
      {
      // Turn is a circular arc
      double dAlpha = dDeltaLen / m_dTrack;
      double dRadiusLen = m_dTrack * (dLength / dDeltaLen);

      // Find the turn center.  The radius vector is perpendicular to the heading. Assume
      // the vector is for a positive turn and the sign of dRadius will do the side correctly.
      double dRX, dRY;
      m_Pose.GetHeadingVector(dRY, dRX);
      dRY = -dRY;

      // Starting radius vector
      double dStartRadiusX = dRX * dRadiusLen;
      double dStartRadiusY = dRY * dRadiusLen;

      // Current position minus the turn radius vector is the center
      double dXCenter = m_Pose.GetX() - dStartRadiusX;
      double dYCenter = m_Pose.GetY() - dStartRadiusY;

      // Rotate the radius vector through the turn angle Alpha
      double dCosAlpha = std::cos(dAlpha);
      double dSinAlpha = std::sin(dAlpha);

      // Apply rotation matrix
      double dEndRadiusX = (dStartRadiusX * dCosAlpha) + (-dStartRadiusY * dSinAlpha);
      double dEndRadiusY = (dStartRadiusX * dSinAlpha) + (dStartRadiusY * dCosAlpha);

      // Compute the new end position
      double dXEnd = dXCenter + dEndRadiusX;
      double dYEnd = dYCenter + dEndRadiusY;

      double dThetaEnd = m_Pose.GetTheta() + dAlpha;

      // Keep the heading between 0 and 2Pi
      double d2Pi = 2.0 * M_PI;
      if (dThetaEnd >= d2Pi)
         {
         dThetaEnd -= d2Pi;
         } // end if
      else if (dThetaEnd < 0.0)
         {
         dThetaEnd += d2Pi;
         }  // end else if

      m_Pose.Reset(dXEnd, dYEnd, dThetaEnd);
      } // end if
   else
      {
      // Consider motion a straight line and simplify
      // ie Avoid numerical instability, etc.
      // Heading unit vector
      double dHeadingX, dHeadingY;
      m_Pose.GetHeadingVector(dHeadingX, dHeadingY);

      // New end position is old plus the distance times the heading unit vector
      double dXEnd = m_Pose.GetX() + (dLength * dHeadingX);
      double dYEnd = m_Pose.GetY() + (dLength * dHeadingY);

      m_Pose.SetX(dXEnd);
      m_Pose.SetY(dYEnd);
      } // end else

   return;

   } // end of method DOdometry::UpdatePose
