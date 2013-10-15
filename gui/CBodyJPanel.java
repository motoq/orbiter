/*
 c  CBodyJPanel.java
 c
 c  Copyright (C) 2012 Kurt Motekew
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

package com.motekew.orbiter.gui;

import java.util.ArrayList;
import java.awt.*;
import javax.swing.*;

import com.motekew.servm.*;
import com.motekew.ui.DecimalMinMaxJTF;

/*
 * Creates a JPanel allowing modifications to the central body
 * and a single reference point associated with it.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class CBodyJPanel extends JPanel {
  private static final double MAX_VALUE = IORanges.MAX_VALUE;

    // Central body related parameters
  DecimalMinMaxJTF omega = new DecimalMinMaxJTF(11, -MAX_VALUE,
                                                     MAX_VALUE);
  DecimalMinMaxJTF rpntx = new DecimalMinMaxJTF(11, -MAX_VALUE,
                                                     MAX_VALUE);
  DecimalMinMaxJTF rpnty = new DecimalMinMaxJTF(11, -MAX_VALUE,
                                                     MAX_VALUE);
  DecimalMinMaxJTF rpntz = new DecimalMinMaxJTF(11, -MAX_VALUE,
                                                     MAX_VALUE);

  /**
   * @param   fieldsList   A list used by the parent to check for fields
   *                       with errors.
   */
  public CBodyJPanel(ArrayList<IEditError> fieldsList) {
    setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
      // Currently, can only customize central body angular velocity.
    JPanel cbJP = new JPanel();
    JPanel cBodyJP = new JPanel();
    cBodyJP.setLayout(new BoxLayout(cBodyJP, BoxLayout.Y_AXIS));
    cBodyJP.setBorder(BorderFactory.createTitledBorder(
                      BorderFactory.createEtchedBorder(), "CB Properties")
                     );
    JPanel omegaPanel = new JPanel();
    omegaPanel.setLayout(new BorderLayout());
    omegaPanel.add(new JLabel("Central Body Angular Velocity (rad/TU):  "),
                                                         BorderLayout.WEST);
    omegaPanel.add(omega, BorderLayout.EAST);
    fieldsList.add(omega);
    omega.setErrorLabel("CB Angular Velocity");
    cBodyJP.add(omegaPanel);
    cbJP.add(cBodyJP);

      // Add a single reference point.
    JPanel rpJP = new JPanel();
    JPanel refPntJP = new JPanel();
    refPntJP.setLayout(new BoxLayout(refPntJP, BoxLayout.Y_AXIS));
    refPntJP.setBorder(BorderFactory.createTitledBorder(
                       BorderFactory.createEtchedBorder(), "Reference Point")
                      );
      // X-axis position
    JPanel xPosJPanel = new JPanel();
    xPosJPanel.setLayout(new BorderLayout());
    xPosJPanel.add(new JLabel("X-Axis Coordinate (DU):  "), BorderLayout.WEST);
    xPosJPanel.add(rpntx, BorderLayout.EAST);
    fieldsList.add(rpntx);
    rpntx.setErrorLabel("Reference Point X-Axis Coordinate");
      // Y-axis position
    JPanel yPosJPanel = new JPanel();
    yPosJPanel.setLayout(new BorderLayout());
    yPosJPanel.add(new JLabel("Y-Axis Coordinate (DU):  "), BorderLayout.WEST);
    yPosJPanel.add(rpnty, BorderLayout.EAST);
    fieldsList.add(rpnty);
    rpnty.setErrorLabel("Reference Point Y-Axis Coordinate");
      // Z-axis position
    JPanel zPosJPanel = new JPanel();
    zPosJPanel.setLayout(new BorderLayout());
    zPosJPanel.add(new JLabel("Z-Axis Coordinate (DU):  "), BorderLayout.WEST);
    zPosJPanel.add(rpntz, BorderLayout.EAST);
    fieldsList.add(rpntz);
    rpntz.setErrorLabel("Reference Point Z-Axis Coordinate");

    refPntJP.add(xPosJPanel);
    refPntJP.add(yPosJPanel);
    refPntJP.add(zPosJPanel);
    rpJP.add(refPntJP);
    add(cbJP);
    add(rpJP);
  }
}
