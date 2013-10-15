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

import com.motekew.enums.EulerA;
import com.motekew.math.Angles;
import com.motekew.math.Quaternion;
import com.motekew.trmtm.EulerAngles;
import com.motekew.sensm.SimpleConeTracker;

/**
 * A TableModel hard coded to handle 3 SimpleConeTracker models.
 * The table structure is defined, and upon request, a pointer to
 * the stored list of SimpleConeTrackers is returned with the most
 * recently entered values.
 */
public class ConeTrackerTableModel extends AbstractTableModel {
  private static final int NTRACKERS = 3;
  
    // Table columns
  private String[] columnNames = {"Tracker",
                                  "X-Rot (deg)", "Y-Rot (deg)", "Z-Rot (deg)",
                                  "Cone Width", "1-\u03C3 (deg)", "Max Meas"};

    // Default orientation, 10 deg conewidth, 0.001 deg 1-sig random error
  private Object[][] data = {
              // Point along +y
            {"First",  new Double(-90), new Double(0),  new Double(0),
                       new Double(10), new Double(.001), new Integer(5)},
              // Point along +x
            {"Second", new Double(0),   new Double(90), new Double(0),
                       new Double(10), new Double(.001), new Integer(5)},
              // Point along -z
            {"Third",  new Double(180), new Double(0),  new Double(0),
                       new Double(10), new Double(.001), new Integer(5)}
  };

  /**
   * Updates the array of trackers with most recently entered values from
   * the corresponding table and returns a pointer.
   *
   * @return    Array of trackers with updated internal values based on table
   *            inputs.  If a parsing error has occurred, a null is returned.
   */
  public SimpleConeTracker[] getConeTrackers() {
    SimpleConeTracker[] trackers = new SimpleConeTracker[NTRACKERS];

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
      trackers[ii] = new SimpleConeTracker(nm);
      ea.toQuatFrameRot(orientation);
      trackers[ii].setOrientation(orientation);
        // Set conewidth and measurement uncertainty (convert sigma to sines)
      trackers[ii].setConeWidth(dvalcone);
      trackers[ii].setRandomError(Math.sin(dvalsigma));
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
