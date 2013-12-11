/*
 c  ConeTrackerTableModel.java
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

import javax.swing.table.*;
import javax.swing.JFrame;
import javax.swing.JOptionPane;

import com.motekew.vse.enums.EulerA;
import com.motekew.vse.math.Angles;
import com.motekew.vse.math.NumberUtil;
import com.motekew.vse.math.Quaternion;
import com.motekew.vse.sensm.SimpleConeTrackerCfg;
import com.motekew.vse.trmtm.EulerAngles;

/**
 * A Table model initialized with default SimpleConeTracker settings
 * and a method to return new values entered by the user.
 *
 * @author  Kurt Motekew
 * @since   2012, 2013
 */
public class ConeTrackerTableModel extends AbstractTableModel {
  private final int NTRACKERS;
  private final int NCOL = 6;

    // Table columns
  private String[] columnNames = {"X-Rot (deg)", "Y-Rot (deg)", "Z-Rot (deg)",
                                  "Cone Width", "1-\u03C3 (deg)", "Max Meas"};

    // Default orientation, 10 deg conewidth, 0.001 deg 1-sig random error
  private Object[][] data = null;

  /**
   * @param   cfgs   Default Cone tracker Settings.  One tracker
   *                 per array entry.
   */
  public ConeTrackerTableModel(SimpleConeTrackerCfg[] cfgs) {
    NTRACKERS = cfgs.length;

    EulerAngles ea = new EulerAngles();
    data = new Object[NTRACKERS][NCOL];
    for (int ii=0; ii<NTRACKERS; ii++) {
      ea.fromQuatFrameRot(cfgs[ii].bodyToSensorAtt());
      data[ii][0] = new Double(NumberUtil.truncate(ea.getDeg(EulerA.BANK), 1));
      data[ii][1] = new Double(NumberUtil.truncate(ea.getDeg(EulerA.ELEV), 1));
      data[ii][2] = new Double(NumberUtil.truncate(ea.getDeg(EulerA.HEAD), 1));
      data[ii][3] = new Double(NumberUtil.truncate(
                                  Math.toDegrees(cfgs[ii].fullConeWidth()), 1));
      data[ii][4] = new Double(NumberUtil.truncate(
                                 Math.toDegrees(cfgs[ii].oneSigmaRandom()), 4));
      data[ii][5] = new Integer(cfgs[ii].maxMeasurementSets());
    }
  }

  /**
   * Creates an array of cone tracker settings with the most recently
   * entered values entered into the table.
   *
   * @return    A new array of cone tracker settings with the values most
   *            recently entered into the table.  If a parsing error has
   *            occurred, a null is returned.
   */
  public SimpleConeTrackerCfg[] getConeTrackerSettings() {
    SimpleConeTrackerCfg[] trackers = new SimpleConeTrackerCfg[NTRACKERS];

      // Parse tracker parameters from table and update
      // system for each along with its associated tracker.
    EulerAngles ea = new EulerAngles();
    Quaternion orientation = new Quaternion();
    for (int ii=0; ii<NTRACKERS; ii++) {
      Object objrx = getValueAt(ii, 1);
      Object objry = getValueAt(ii, 2);
      Object objrz = getValueAt(ii, 3);
      Object objcone = getValueAt(ii, 4);
      Object objsigma = getValueAt(ii, 5);
      Object objnm = getValueAt(ii, 6);
      try {
        // Parse degree inputs and convert to radians
      ea.put(EulerA.BANK, Math.toRadians(Double.parseDouble(objrx.toString())));
      ea.put(EulerA.ELEV, Math.toRadians(Double.parseDouble(objry.toString())));
      ea.put(EulerA.HEAD, Math.toRadians(Double.parseDouble(objrz.toString())));
      double dvalcone = Math.toRadians(Double.parseDouble(objcone.toString()));
      double dvalsigma = Math.toRadians(Double.parseDouble(objsigma.toString()));
      int nm = Integer.parseInt(objnm.toString());
        // Do something really basic for error checking
      if (dvalcone > Angles.PIO2  ||  dvalcone < -Angles.PIO2  ||
          dvalsigma < 0.0         ||  dvalsigma > dvalcone/10.0) {
        throw new NumberFormatException("Bad cone or sigma value");
      }
        // Set orientation
      ea.toQuatFrameRot(orientation);
      trackers[ii] = new SimpleConeTrackerCfg(nm, dvalcone, dvalsigma,
                                                          orientation);
      } catch(NumberFormatException nfe) {
        trackers = null;
        JOptionPane.showMessageDialog(new JFrame(), "Bad Star Tracker Inputs",
                                      "Table Input", JOptionPane.ERROR_MESSAGE);
      }
    }
    return trackers;
  }

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

 /**
  * Only allow edits to data columns
  */
  public boolean isCellEditable(int row, int col) {
    if (col < 1) {
      return false;
    } else {
      return true;
    }
  }

  public void setValueAt(Object value, int row, int col) {
    data[row][col] = value;
    fireTableCellUpdated(row, col);
  }
}
