/*
 c  OrbiterAttOpts.java
 c
 c  Copyright (C) 2011 Kurt Motekew
 c
 c  This library is free software; you can redistribute it and/or
 c  modify it under the terms of the GNU Lesser General Public
 c  License as published by the Free Software Foundation; either
 c  version 2.1 of the License, or (at your option) any later version.
 c
 c  This library is distributed in the hope that it will be useful,
 c  but WITHOUT ANY WARRANTY; without even the implied warranty of
 c  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 c  Lesser General Public License for more details.
 c
 c  You should have received a copy of the GNU Lesser General Public
 c  License along with this library; if not, write to the Free Software
 c  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 c  02110-1301 USA
 */

package com.motekew.orbiter;

/**
 * This enum contains the various attitude control options available to
 * the Orbiter simulation.
 * <P>
 * The Stability Control option uses only the kinematic portion of the
 * control law to damp all angular velocities to zero.
 * <P>
 * The RPY option orients relative to the orbit RPY frame.  In this
 * reference frame, the z-axis points towards nadir while the x-axis
 * is in the same direction as the positive velocity vector (in the
 * same plane as that formed by the z-axis and the inertial velocity
 * vector)
 * <P>
 * The Reference Point option uses a reference point attached to the
 * central body to orient the Orbiter.  The z-axis points towards the
 * reference point while the y-axis is in the plane formed by the line
 * from the orbiter to the point, and the line from nadir to the point.
 *
 * @author   Kurt Motekew
 * @since    20111113
 */
public enum OrbiterAttOpts {
  CNT_OFF,     // Disable all attitude control
  CNT_STAB,    // Stability Control
  CNT_I,       // Relative to inertal reference frame
  CNT_RPY,     // RPY, orbit local level
  CNT_RPNT     // Reference Point
}
