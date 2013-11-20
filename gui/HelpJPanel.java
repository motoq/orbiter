/*
 c  HelpJPanel.java
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

import javax.swing.*;

/**
 * Creates a JPanel with text describing what effect keys
 * have.  Also mention availability of context sensitive
 * help.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class HelpJPanel extends JPanel {
  public HelpJPanel() {
    JTextArea keyDefsJTA = new JTextArea(
      "More info:  http://vehsim.sourceforge.net/" +
      "\nMouse over fields and/or labels for bubble help." +
      "\nRight click fields for input limits.\n" +
      " E:\tPitch Down\n" +
      " D:\tPitch Up\n" +
      " F:\tRoll Right Wing Down\n" +
      " S:\tRoll Left Wing Down\n" +
      " G:\tYaw Right\n" +
      " A:\tYaw Left\n" +
      " UP:\tIncrease Thrust\n" +
      " DOWN:\tDecrease Thrust (minimum value of zero) \n" +
      " SPACE:\tShow/Hide Model State Viewer windows\n" +
      " Q:\tZero User Input Thrusts & Torques\n" +
      " R:\tZero All Rotation Rates\n" +
      " T:\tZero All Translational Velocity"
    );
    //keyDefsJTA.setFont(new Font("Courier", keyDefsJTA.getFont().getStyle(),
    //                                       keyDefsJTA.getFont().getSize()));
    keyDefsJTA.setEditable(false);
    add(keyDefsJTA);
  }
}
