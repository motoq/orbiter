/*
 c  RefPntAttOpts.java
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
 * This enum contains the various attitude control options available to the
 * Orbiter simulation that make use of a reference point.
 * <P>
 * It is best to look to the code implemented by these options to see what
 * each means (<code>com.motekew.trmtm.AttitudeRefPoint</code>).
 *
 * @author    Kurt Motekew
 * @since     20111124
 */
public enum RefPntAttOpts {
  RPATT_YAXIS,
  RPATT_ZEROYAW
}
