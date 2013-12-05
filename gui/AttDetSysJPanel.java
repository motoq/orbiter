/*
 c  AttDetSysJPanel.java
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
import javax.swing.table.*;

import com.motekew.vse.sensm.IPointingPlatform;
import com.motekew.vse.ui.StatusJP;

/**
 * Create an area to add reference points in inertial space to
 * be used by the attitude determination system.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class AttDetSysJPanel extends JPanel {
  private JButton estAttBtn = new JButton("Estimate Attitude");
  JCheckBox recAttCB;
  String estAttAC = "estAtt";
  String recAttAC = "RecordATT";
  ConeTrackerTableModel adsTableModel; 
  StatusJP attTime = new StatusJP(0.0, "Time (TU):  ");
  TableModel adsOutTableModel;

  /**
   * @param   parent   The object accepting actions from this JPanel
   * @param   re       Reference radius
   */
  public AttDetSysJPanel(OrbiterInputsFrame parent, IPointingPlatform platform) {
    setBorder(BorderFactory.createTitledBorder(
              BorderFactory.createEtchedBorder(), "Attitude Determination")
              );
    setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

      // ADS reference points - a table
    JPanel adsrpJP = new JPanel();
    JPanel adsrpBodyJP = new JPanel();
    adsrpBodyJP.setLayout(new BoxLayout(adsrpBodyJP, BoxLayout.Y_AXIS));
    adsrpBodyJP.setBorder(BorderFactory.createTitledBorder(
         BorderFactory.createEtchedBorder(),
                                   "Star Tracker Body to Sensor Orientation")
                          );

    adsTableModel = new ConeTrackerTableModel(platform);
    JTable adsTable = new JTable((TableModel) adsTableModel);
    //fieldsList.add(omega);
    //omega.setErrorLabel("CB Angular Velocity");
    adsrpBodyJP.add(adsTable.getTableHeader());
    adsrpBodyJP.add(adsTable);
    adsrpJP.add(adsrpBodyJP);

      // ADS outputs
    JPanel adsOutJP = new JPanel();
    JPanel adsOutBodyJP = new JPanel();
    adsOutBodyJP.setLayout(new BoxLayout(adsOutBodyJP, BoxLayout.Y_AXIS));
    adsOutBodyJP.setBorder(BorderFactory.createTitledBorder(
         BorderFactory.createEtchedBorder(), "Attitude Output")
                          );
    adsOutTableModel = new OutTableModel();
    JTable outTable = new JTable(adsOutTableModel);
    adsOutBodyJP.add(attTime);
    adsOutBodyJP.add(outTable.getTableHeader());
    adsOutBodyJP.add(outTable);
    adsOutJP.add(adsOutBodyJP);
    
      // Panel to turn ADS on/off
    JPanel adsOnOff = new JPanel();
    estAttBtn.setActionCommand(estAttAC);
    adsOnOff.add(estAttBtn);
    estAttBtn.addActionListener(parent);
    estAttBtn.setToolTipText("Prints Estimate of Attitude");
    recAttCB = new JCheckBox("Record Attitude Estimates");
    recAttCB.setActionCommand(recAttAC);
    adsOnOff.add(recAttCB);
    recAttCB.addActionListener(parent);

    add(adsrpJP);
    add(adsOutJP);
    add(adsOnOff);
  }

    /*
     * TableModel for outputting true vs. measured attitude.
     *
     */
  private class OutTableModel extends AbstractTableModel {
    private String[] columnNames = {" ",
                                    "Heading \u00B0",
                                    "Elevation \u00B0",
                                    "Bank \u00B0",
                                    "Iterations"};
    private Object[][] data = {
      {"True",
        new Double(0), new Double(0), new Double(0), "-"},
      {"TRIAD \u0394",
        new Double(0), new Double(0), new Double(0), "-"},
      {"QWLS \u0394",
        new Double(0), new Double(0), new Double(0), new Integer(0)},
      {"QWLS 3\u03C3",
        new Double(0), new Double(0), new Double(0), new Integer(0)},
      {"DQWLS \u0394",
        new Double(0), new Double(0), new Double(0), new Integer(0)}
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
