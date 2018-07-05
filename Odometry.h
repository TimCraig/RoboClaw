/******************************************************************************
*** Odometry.h
******************************************************************************/

/*
This class provides a generic odometry mode based on wheel movement for a
differential drive robot.  A differential drive robot travels in circular arcs
with the special case of a straight line being an arc of infinite radius.  The
equations are exact for constant velocity and over small movements acceleration
should be negligible.

Author: Tim Craig (Druai Robotics) 2018

*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#pragma once

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <cmath>
#include <math.h>

/*****************************************************************************
*
***  class DPose
*
* General pose for at robot operating on a 2D plane.
*
*****************************************************************************/

class DPose
   {
   public:
      DPose(double dX = 0.0, double dY = 0.0, double dTheta = 0.0)
            : m_dX{dX}, m_dY{dY}
         {
         SetTheta(dTheta);

         return;
         }

      DPose(const DPose& src) = default;
      DPose(DPose&& src) = default;

      ~DPose() = default;

      DPose& operator=(const DPose& rhs) = default;
      DPose& operator=(DPose&& rhs) = default;

      void Reset(double dX, double dY, double dTheta)
         {
         m_dX = dX;
         m_dY = dY;
         SetTheta(dTheta);

         return;
         }

      void SetX(double dX)
         {
         m_dX = dX;
         return;
         }

      double GetX() const
         {
         return(m_dX);
         }

      void SetY(double dY)
         {
         m_dY = dY;
         return;
         }

      double GetY() const
         {
         return (m_dY);
         }

      double GetTheta() const
         {
         return (m_dTheta);
         }

      void GetHeadingVector(double& dHeadingX, double& dHeadingY) const
         {
         dHeadingX = m_dHeadingX;
         dHeadingY = m_dHeadingY;

         return;
         }

      void SetTheta(double dTheta)
         {
         m_dTheta = dTheta;
         m_dHeadingX = std::cos(m_dTheta);
         m_dHeadingY = std::sin(m_dTheta);

         return;
         }

      void Update(double dDeltaX, double dDeltaY)
         {
         m_dX += dDeltaX;
         m_dY += dDeltaY;
         \
         return;
         }

      void Update(double dDeltaX, double dDeltaY, double dDeltaTheta)
         {
         m_dX += dDeltaX;
         m_dY += dDeltaY;
         SetTheta(m_dTheta + dDeltaTheta);
         \
         return;
         }

   protected:
      // Position in world units
      double m_dX;
      double m_dY;

      // Heading angle in radians
      double m_dTheta;

      // Heading unit vector (sin and cos of Theta)
      double m_dHeadingY;
      double m_dHeadingX;

   private:

   }; // end of class DPose

/*****************************************************************************
*
***  class DOdometry
*
* Provide updating pose for a differential drive robot using wheel rotation
* measurements.  Motion of the robot is always along a circular arc (an arc of
* infinite radius is also known as a straight line which is treated as a separate
* case).  This assumes motion at constant wheel speeds so the equations are exact
* neglecting noise and encoder quantization errors.  Wheel accelerataions are
* not accounted for and assumed to be negligible over short sampling times.
*
*****************************************************************************/

class DOdometry
   {
   public:
      DOdometry(double dX = 0.0, double dY = 0.0, double dTheta = 0.0, double dTrack = 1.0)
            : m_Pose{dX, dY, dTheta}, m_dTrack{dTrack}
         {
         return;
         }

      DOdometry(const DPose& Pose, double dTrack = 1.0)
            : m_Pose{Pose}, m_dTrack{dTrack}
         {
         return;
         }

      DOdometry(const DOdometry& src) = default;
      DOdometry(DOdometry&& src) = default;

      ~DOdometry() = default;

      DOdometry& operator=(const DOdometry& rhs) = default;
      DOdometry& operator=(DOdometry&& rhs) = default;

      const DPose& GetPose() const
         {
         return (m_Pose);
         }

      double GetTrack() const
         {
         return (m_dTrack);
         }

      // Primary function.  Update the pose given wheel travel.
      void UpdatePose(double dLeft, double dRight);

   protected:
      // Pose of the robot X, Y, and Heading (Rotation relative to the world system)
      DPose m_Pose;

      // Distance between wheel centers.  Same units as robot positions.
      const double m_dTrack;

   private:

   }; // end of class DOdometry

#endif // ODOMETRY_H
