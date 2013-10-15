/*
 c  OrbitOrbiterJPanel.java
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
 * Create window for orbit parameters and orbiter properties.
 * Multiple package protected objects are available for use
 * outside of this class.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class OrbitOrbiterJPanel extends JPanel {
  private static final double MAX_VALUE = IORanges.MAX_VALUE;
  
    // Semi-major axis
  DecimalMinMaxJTF a_oe = new DecimalMinMaxJTF(11, 1.0, MAX_VALUE);
    // Eccentricity
  DecimalMinMaxJTF e_oe = new DecimalMinMaxJTF(11, 0.0, 0.9);
    // Inclination
  DecimalMinMaxJTF i_oe = new DecimalMinMaxJTF(11, -180.0, 180.0);
    // Right Ascension of the Ascending Node
  DecimalMinMaxJTF o_oe = new DecimalMinMaxJTF(11, -360.0, 360.0);
   // Argument of periapsis
  DecimalMinMaxJTF w_oe = new DecimalMinMaxJTF(11, -360.0, 360.0);
    // True Anomaly
  DecimalMinMaxJTF v_oe = new DecimalMinMaxJTF(11, -360.0, 360.0);
    // Mass and distribution
  DecimalMinMaxJTF mass = new DecimalMinMaxJTF(8, 0.0000001, MAX_VALUE);
  DecimalMinMaxJTF ixx = new DecimalMinMaxJTF(6, 0.0000001, MAX_VALUE);
  DecimalMinMaxJTF iyy = new DecimalMinMaxJTF(6, 0.0000001, MAX_VALUE);
  DecimalMinMaxJTF izz = new DecimalMinMaxJTF(6, 0.0000001, MAX_VALUE);
  DecimalMinMaxJTF ixy = new DecimalMinMaxJTF(6, -MAX_VALUE, MAX_VALUE);
  DecimalMinMaxJTF ixz = new DecimalMinMaxJTF(6, -MAX_VALUE, MAX_VALUE);
  DecimalMinMaxJTF iyz = new DecimalMinMaxJTF(6, -MAX_VALUE, MAX_VALUE);

  /**
   * @param   fieldsList   A list used by the parent to check for fields
   *                       with errors.
   */
  public OrbitOrbiterJPanel(ArrayList<IEditError> fieldsList) {
      // Orbit parameters
    JPanel oeJP = new JPanel();
    oeJP.setLayout(new BoxLayout(oeJP, BoxLayout.Y_AXIS));
    oeJP.setBorder(BorderFactory.createTitledBorder(
                   BorderFactory.createEtchedBorder(), "Orbital Elements")
                  );
      // Semi-major axis
    JPanel aJP = new JPanel();
    aJP.setLayout(new BorderLayout());
    a_oe.setHorizontalAlignment(JTextField.RIGHT);
    JLabel albl = new JLabel("a (DU):  ");
    aJP.add(albl, BorderLayout.WEST);
    aJP.add(a_oe, BorderLayout.EAST);
    fieldsList.add(a_oe);
    a_oe.setErrorLabel("a");
    a_oe.setToolTipText("Semi-major Axis");
      // Eccentricity
    JPanel eJP = new JPanel();
    eJP.setLayout(new BorderLayout());
    e_oe.setHorizontalAlignment(JTextField.RIGHT);
    e_oe.setFormat("0.########");
    JLabel elbl = new JLabel("e:  ");
    eJP.add(elbl, BorderLayout.WEST);
    eJP.add(e_oe, BorderLayout.EAST);
    fieldsList.add(e_oe);
    e_oe.setErrorLabel("e");
    e_oe.setToolTipText("Eccentricity");
      // Inclination
    JPanel iJP = new JPanel();
    iJP.setLayout(new BorderLayout());
    i_oe.setHorizontalAlignment(JTextField.RIGHT);
    i_oe.setFormat("0.########");
    JLabel ilbl = new JLabel("i (deg):  ");
    iJP.add(ilbl, BorderLayout.WEST);
    iJP.add(i_oe, BorderLayout.EAST);
    fieldsList.add(i_oe);
    i_oe.setErrorLabel("i");
    i_oe.setToolTipText("Inclination");
      // Right ascenscion of the ascending node
    JPanel oJP = new JPanel();
    oJP.setLayout(new BorderLayout());
    o_oe.setHorizontalAlignment(JTextField.RIGHT);
    o_oe.setFormat("0.########");
    JLabel olbl = new JLabel("o (deg):  ");
    oJP.add(olbl, BorderLayout.WEST);
    oJP.add(o_oe, BorderLayout.EAST);
    fieldsList.add(o_oe);
    o_oe.setErrorLabel("o");
    o_oe.setToolTipText("Right Ascension of the Ascending Node");
      // Argument of periapsis
    JPanel wJP = new JPanel();
    wJP.setLayout(new BorderLayout());
    w_oe.setHorizontalAlignment(JTextField.RIGHT);
    w_oe.setFormat("0.########");
    JLabel wlbl = new JLabel("w (deg):  ");
    wJP.add(wlbl, BorderLayout.WEST);
    wJP.add(w_oe, BorderLayout.EAST);
    fieldsList.add(w_oe);
    w_oe.setErrorLabel("w");
    w_oe.setToolTipText("Argument of periapsis");
      // True anomaly
    JPanel vJP = new JPanel();
    vJP.setLayout(new BorderLayout());
    v_oe.setHorizontalAlignment(JTextField.RIGHT);
    v_oe.setFormat("0.########");
    JLabel vlbl = new JLabel("v (deg):  ");
    vJP.add(vlbl, BorderLayout.WEST);
    vJP.add(v_oe, BorderLayout.EAST);
    fieldsList.add(v_oe);
    v_oe.setErrorLabel("v");
    v_oe.setToolTipText("True Anomaly");

    oeJP.add(aJP);
    oeJP.add(eJP);
    oeJP.add(iJP);
    oeJP.add(oJP);
    oeJP.add(wJP);
    oeJP.add(vJP);

      // Mass properties
    JPanel massDistJP = new JPanel();
    massDistJP.setLayout(new BoxLayout(massDistJP, BoxLayout.Y_AXIS));
    massDistJP.setBorder(BorderFactory.createTitledBorder(
                         BorderFactory.createEtchedBorder(), "Mass Properties")
                        );
      // Orbiter Mass
    JPanel massJP = new JPanel();
    mass.setFormat("0.00");
    mass.setHorizontalAlignment(JTextField.RIGHT);
    JLabel mlbl = new JLabel("Mass:  ");
    massJP.add(mlbl);
    massJP.add(mass);
    fieldsList.add(mass);
    mass.setErrorLabel("Mass");
      // Mass distribution - X
    JPanel ixxJP = new JPanel();
    ixxJP.setLayout(new BorderLayout());
    ixx.setHorizontalAlignment(JTextField.RIGHT);
    ixx.setFormat("0.0000");
    JLabel ixxlbl = new JLabel("Ixx:  ");
    ixxJP.add(ixxlbl, BorderLayout.WEST);
    ixxJP.add(ixx, BorderLayout.EAST);
    fieldsList.add(ixx);
    ixx.setErrorLabel("Ixx");
      // Mass distribution - Y
    JPanel iyyJP = new JPanel();
    iyyJP.setLayout(new BorderLayout());
    iyy.setHorizontalAlignment(JTextField.RIGHT);
    iyy.setFormat("0.0000");
    JLabel iyylbl = new JLabel("Iyy:  ");
    iyyJP.add(iyylbl, BorderLayout.WEST);
    iyyJP.add(iyy, BorderLayout.EAST);
    fieldsList.add(iyy);
    iyy.setErrorLabel("Iyy");
      // Mass distribution - Z
    JPanel izzJP = new JPanel();
    izzJP.setLayout(new BorderLayout());
    izz.setHorizontalAlignment(JTextField.RIGHT);
    izz.setFormat("0.0000");
    JLabel izzlbl = new JLabel("Izz:  ");
    izzJP.add(izzlbl, BorderLayout.WEST);
    izzJP.add(izz, BorderLayout.EAST);
    fieldsList.add(izz);
    izz.setErrorLabel("Izz");
      // Asymmetry
    JPanel ixyJP = new JPanel();
    ixyJP.setLayout(new BorderLayout());
    ixy.setHorizontalAlignment(JTextField.RIGHT);
    ixy.setFormat("0.0000");
    JLabel ixylbl = new JLabel("Ixy:  ");
    ixyJP.add(ixylbl, BorderLayout.WEST);
    ixyJP.add(ixy, BorderLayout.EAST);
    fieldsList.add(ixy);
    ixy.setErrorLabel("Ixy");
      //
    JPanel ixzJP = new JPanel();
    ixzJP.setLayout(new BorderLayout());
    ixz.setHorizontalAlignment(JTextField.RIGHT);
    ixz.setFormat("0.0000");
    JLabel ixzlbl = new JLabel("Ixz:  ");
    ixzJP.add(ixzlbl, BorderLayout.WEST);
    ixzJP.add(ixz, BorderLayout.EAST);
    fieldsList.add(ixz);
    ixz.setErrorLabel("Ixz");
      //
    JPanel iyzJP = new JPanel();
    iyzJP.setLayout(new BorderLayout());
    iyz.setHorizontalAlignment(JTextField.RIGHT);
    iyz.setFormat("0.0000");
    JLabel iyzlbl = new JLabel("Iyz:  ");
    iyzJP.add(iyzlbl, BorderLayout.WEST);
    iyzJP.add(iyz, BorderLayout.EAST);
    fieldsList.add(iyz);
    iyz.setErrorLabel("Iyz");

    massDistJP.add(massJP);
    massDistJP.add(ixxJP);
    massDistJP.add(iyyJP);
    massDistJP.add(izzJP);
    massDistJP.add(ixyJP);
    massDistJP.add(ixzJP);
    massDistJP.add(iyzJP);

    add(oeJP);
    add(massDistJP);
  }
}
