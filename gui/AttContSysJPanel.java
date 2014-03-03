/*
 c  AttContSysJPanel.java
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
import javax.swing.table.*;

import com.motekew.vse.math.*;
import com.motekew.vse.servm.*;
import com.motekew.vse.ui.*;

/**
 * Creates the user interface for the Attitude Control System Settings.
 * The desired attitude relative to a reference attitude (inertial,
 * local level, etc.) can be entered.  The deviation from the desired
 * is listed, along with the control system torques in use to correct
 * for deviations.  An option to compute and display attitude control
 * inputs without applying them exists.  Attitude control tuning
 * parameters, to be used with the Orbiter mass properties, are
 * also available.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class AttContSysJPanel extends JPanel {
  JCheckBox ignoreAttTqsCB;
  String ignoreTqAC = "IgnoreACT";
  JCheckBox enableADS_CB;
  JCheckBox useADS_CB;
  String enableADS = "EnableADS";
  String useADS = "UseADS";
    // Desired, deviation, correcting torque, for heading, elevation, bank
  DecimalMinMaxJTF heading = new DecimalMinMaxJTF(11, -180.0, 180.0);
  ValueViewJP yaw = new ValueViewJP("Yaw:  ", 0.0);
  ValueViewJP uz = new ValueViewJP("Uz:  ", 0.000);
  DecimalMinMaxJTF elevation = new DecimalMinMaxJTF(11, -90.0, 90.0);
  ValueViewJP pitch = new ValueViewJP("Pitch:  ", 0.0);
  ValueViewJP uy = new ValueViewJP("Uy:  ", 0.000);
  DecimalMinMaxJTF bank = new DecimalMinMaxJTF(11, -180.0, 180.0);
  ValueViewJP roll = new ValueViewJP("Roll:  ", 0.0);
  ValueViewJP ux = new ValueViewJP("Ux:  ", 0.000);
    // Damping and Stiffness matrices/fields
  Matrix kvM = new Matrix3X3();        // Default matrices all zeros...
  Matrix kpM = new Matrix3X3();        // so
  Tuple axesV = new Tuple3D();         // no action by control system
  MatrixJP kv = new MatrixJP(3, 3, 3);
  MatrixJP kp = new MatrixJP(3, 3, 3);
  MatrixJP axes = new MatrixJP(3, 1, 3);

  /**
   * @param   parent       The object accepting actions from this JPanel.
   * @param   fieldsList   A list used by the parent to check for fields
   *                       with errors.
   */
  public AttContSysJPanel(OrbiterInputsFrame parent,
                          ArrayList<IEditError> fieldsList) {
    setBorder(BorderFactory.createTitledBorder(
        BorderFactory.createEtchedBorder(), "Attitude Control")
              );
    setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

    /*
     * Desired attitude, deviation (in Euler Angles), and corrective torques
     */
    JPanel eulerPanel = new JPanel();    // FlowLayout
      //
    JPanel headingJPanel = new JPanel();
    headingJPanel.setLayout(new BorderLayout());
    headingJPanel.add(new JLabel("Heading (deg):  "), BorderLayout.WEST);
    headingJPanel.add(heading, BorderLayout.EAST);
    heading.setFormat("0.########");
    fieldsList.add(heading);
    heading.setErrorLabel("Heading");
    heading.setToolTipText("Desired Heading");
      //
    JPanel elevationJPanel = new JPanel();
    elevationJPanel.setLayout(new BorderLayout());
    elevationJPanel.add(new JLabel("Elevation (deg):  "), BorderLayout.WEST);
    elevationJPanel.add(elevation, BorderLayout.EAST);
    elevation.setFormat("0.########");
    fieldsList.add(elevation);
    elevation.setErrorLabel("Elevation");
    elevation.setToolTipText("Desired Elevation");
      //
    JPanel bankJPanel = new JPanel();
    bankJPanel.setLayout(new BorderLayout());
    bankJPanel.add(new JLabel("Bank (deg):  "), BorderLayout.WEST);
    bankJPanel.add(bank, BorderLayout.EAST);
    bank.setFormat("0.########");
    fieldsList.add(bank);
    bank.setErrorLabel("Bank");
    bank.setToolTipText("Desired Bank");
      //
    JPanel desiredEulerJP = new JPanel();
    desiredEulerJP.setLayout(new BoxLayout(desiredEulerJP, BoxLayout.Y_AXIS));
    desiredEulerJP.add(headingJPanel);
    desiredEulerJP.add(elevationJPanel);
    desiredEulerJP.add(bankJPanel);
      // Deviations
    JPanel deltaEulerJP = new JPanel();
    deltaEulerJP.setLayout(new BoxLayout(deltaEulerJP, BoxLayout.Y_AXIS));
    deltaEulerJP.add(yaw);
    deltaEulerJP.add(pitch);
    deltaEulerJP.add(roll);
    yaw.setToolTipText("Deviation from Desired Heading");
    pitch.setToolTipText("Deviation from Desired Elevation");
    roll.setToolTipText("Deviation from Desired Bank");
      // Attitude Control Corrective Torques
    JPanel attTorquesJP = new JPanel();
    attTorquesJP.setLayout(new BoxLayout(attTorquesJP, BoxLayout.Y_AXIS));
    attTorquesJP.add(uz);
    attTorquesJP.add(uy);
    attTorquesJP.add(ux);
    uz.setToolTipText("Control System Corrective Torque:  Yaw");
    uy.setToolTipText("Control System Corrective Torque:  Pitch");
    ux.setToolTipText("Control System Corrective Torque:  Roll");
      //
    eulerPanel.add(desiredEulerJP);
    eulerPanel.add(deltaEulerJP);
    eulerPanel.add(attTorquesJP);

      // Control Option
    JPanel ignoreAttTqsJP = new JPanel();
    ignoreAttTqsCB = new JCheckBox("Don't Apply Attitude Correction");
    ignoreAttTqsCB.setActionCommand(ignoreTqAC);
    ignoreAttTqsCB.addActionListener(parent);
    ignoreAttTqsJP.add(ignoreAttTqsCB);
    
      // Gain Matrices
    JPanel gainPanel = new JPanel();
    gainPanel.setLayout(new BoxLayout(gainPanel, BoxLayout.X_AXIS));
      //
    JPanel kvPanel = new JPanel();
    kvPanel.add(new JLabel(" Damping:  "));
    kvPanel.add(kv);
    kv.setFormat("0.0");
    kv.setErrorLabel("Damping");
    fieldsList.add(kv);
    gainPanel.add(kvPanel);
      //
    JPanel kpPanel = new JPanel();
    kpPanel.add(new JLabel(" Stiffness:  "));
    kpPanel.add(kp);
    kp.setFormat("0.0");
    kp.setErrorLabel("Stiffness");
    fieldsList.add(kp);
    gainPanel.add(kpPanel);
      //
    JPanel aPanel = new JPanel();
    aPanel.add(new JLabel("  Axes:  "));
    aPanel.add(axes);
    axes.setFormat("0.0");
    axes.setErrorLabel("Axes");
    fieldsList.add(axes);
    gainPanel.add(aPanel);

    JPanel adsFiltOutJP = new JPanel();
      // Use an inner JPanel to keep table from
      // stretching the entire width of the window
    JPanel adsFiltOutBodyJP = new JPanel();
    adsFiltOutBodyJP.setLayout(new BoxLayout(adsFiltOutBodyJP,
                                             BoxLayout.Y_AXIS));
    adsFiltOutBodyJP.setBorder(BorderFactory.createTitledBorder(
           BorderFactory.createEtchedBorder(), "Filtered Attitude Solution")
                              );
      // Stack filter enabling and use options
    JPanel foPanel = new JPanel();
    foPanel.setLayout(new BoxLayout(foPanel, BoxLayout.Y_AXIS));
    enableADS_CB = new JCheckBox("Enable ADS");
    enableADS_CB.setActionCommand(enableADS);
    enableADS_CB.addActionListener(parent);
    useADS_CB = new JCheckBox("Use ADS");
    useADS_CB.setActionCommand(useADS);
    useADS_CB.addActionListener(parent);
    foPanel.add(enableADS_CB);
    foPanel.add(useADS_CB);
    adsFiltOutJP.add(foPanel);
      //
    TableModel fadsTableModel = new OutTableModel();
    JTable adsTable = new JTable(fadsTableModel);
    adsFiltOutBodyJP.add(adsTable.getTableHeader());
    adsFiltOutBodyJP.add(adsTable);
    adsFiltOutJP.add(adsFiltOutBodyJP);

    add(eulerPanel);
    add(ignoreAttTqsJP);
    add(gainPanel);
    add(adsFiltOutJP);
  }

  /**
   * Points internal gain matrices to those used by the control system.
   * Then updates the UI versions.
   *
   * @param   kvIn    Damping matrix (kinetic part of control law)
   * @param   kpIn    Stiffness (potential)
   * @param   axesIn  For Stiffness
   */
  void setKvKpA(Matrix3X3 kvIn, Matrix3X3 kpIn, Tuple3D axesIn) {
    kvM = kvIn;
    kpM = kpIn;
    axesV = axesIn;
    kv.set(kvM);
    kp.set(kpM);
    axes.set(axesV);
  }

  /*
   * TableModel for outputting true vs. measured attitude.
   *
   */
  private class OutTableModel extends AbstractTableModel {
    private String[] columnNames = {" ", "Heading \u00B0",
                                         "Elevation \u00B0",
                                         "Bank \u00B0"};
    private Object[][] data = {
      {"Estimate", new Double(0), new Double(0), new Double(0)},
      {"\u0394", new Double(0), new Double(0), new Double(0)},
      {"3\u03C3", new Double(0), new Double(0), new Double(0)}
    };

    public int getColumnCount() {
      return columnNames.length;
    }

    public int getRowCount() {
      return data.length;
    }

    public String getColumnName(int col) {
      return columnNames[col];
    }

    public Object getValueAt(int row, int col) {
      return data[row][col];
    }

    public void setValueAt(Object value, int row, int col) {
      data[row][col] = value;
      fireTableCellUpdated(row, col);
    }
  }
}
