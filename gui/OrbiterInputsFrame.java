/*
 c  OrbiterInputsFrame.java
 c
 c  Copyright (C) 2011, 2013 Kurt Motekew
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

import java.io.*;
import java.util.ArrayList;
import java.text.DecimalFormat;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;


import com.motekew.vse.c0ntm.*;
import com.motekew.vse.enums.*;
import com.motekew.vse.envrm.Gravity;
import com.motekew.vse.math.*;
import com.motekew.vse.sensm.*;
import com.motekew.vse.servm.*;
import com.motekew.vse.trmtm.*;
import com.motekew.vse.ui.*;

import com.motekew.orbiter.OrbiterSys;
import com.motekew.orbiter.OrbiterAttOpts;
import com.motekew.orbiter.RefPntAttOpts;

/**
 * Main window for Orbiter simulation user inputs.  The simulation
 * time can be started and stopped.  Orbital, mass, and attitude
 * control settings/inputs can be made.  See the classes for each
 * panel added to the JTabbedPane for more info.
 * <P>
 * Not neat, but writing GUIs is boring....
 *
 * @author   Kurt Motekew
 * @since    20111011
 * @since    20120929   Made less messy with external classes for tabs
 */
public class OrbiterInputsFrame extends JFrame implements IHandleObserver,
                                                           ActionListener {

    // Utility to track fields with errors
  private ArrayList<IEditError> fieldsList = new ArrayList<IEditError>(); 
  private String lastErrorLabel = "";

    // Stuff that needs to be persistent
  private final int ORITERHANDLE;
  private double gm = 1.0;
  private OrbiterSys oSys = null;
  private RotatingBodySys cbSys = null;
  private ReferencePointSys rpSys = null;
  private IStartStop runStop;
  private StatusJP status = new StatusJP(0.0, "Time (TU):  ");
  private JButton runBtn = new JButton(">");
  private JButton pauseBtn = new JButton("||");

    // Runtime Attitude Control Options
  private JRadioButton offButton = new JRadioButton("Off", true);
  private JRadioButton scButton = new JRadioButton("Stability Control", false);
  private JRadioButton atiButton = new JRadioButton("Inertial (rest-to-rest)",
                                                                        false);
  private JRadioButton rpyButton = new JRadioButton("Orbit RPY (dynamic)",
                                                                        false);
  private JRadioButton rpButton = new JRadioButton("Reference Point (dynamic)",
                                                                        false);

    // For Att Cont: desired attitude, perturbation from desired,
    // and control commanded to correct
  private EulerAngles desiredAttitude = new EulerAngles();
  private EulerAngles deltaAtt = new EulerAngles();
  private Tuple3D attitudeControlU = new Tuple3D();

    // Externally defined JPanels with package protected data for inputs
    // and settings
  private OrbitOrbiterJPanel orbitPanel;
  private CBodyJPanel cbPanel;
  private AttDetSysJPanel adsPanel;
  private AttContSysJPanel attitudePanel;
  private ArrayList<AttDetData> attDetDataList = new ArrayList<AttDetData>();
  private File attDetDataDir = null;

  /**
   * Create the primary frame with a time and time controls panel at the top,
   * a tabbed pane panel in the middle containing with many inputs and
   * controls, and a panel at the bottom allowing selection of attitude
   * control settings.
   * 
   * @param   gin    Central body gravity model
   * @param   cb     Central body system.
   * @param   os     The orbiter system being monitored and controlled.
   * @param   rps    Reference point system.
   * @param   rs     The start() & stop() methods of this object are
   *                 called when the "play" and "pause" buttons are
   *                 clicked from the GUI.
   */
  public OrbiterInputsFrame(Gravity gin, RotatingBodySys cb, OrbiterSys os,
                                         ReferencePointSys rps, IStartStop rs) {
      // Overall init
    gm = gin.getGravParam();
    cbSys = cb;
    oSys = os;
    rpSys = rps;
    ORITERHANDLE = oSys.hashCode();
    oSys.registerObserver(this, ORITERHANDLE);
    runStop = rs;
    setTitle("Orbiter Configuration");

      // Default layout manager is BorderLayout.
    Container cp = getContentPane();

    /*
     * Top panel displays time and controls start/stop
     */
    JPanel timeControlJP = new JPanel();
    timeControlJP.setBorder(BorderFactory.createEtchedBorder());
    timeControlJP.setLayout(new FlowLayout());
    timeControlJP.add(status);
      // Start simulation time progression
    runBtn.setActionCommand("run");
    timeControlJP.add(runBtn);
    runBtn.addActionListener(this);
    runBtn.setToolTipText("Run Simulation From Current Time & State");
      // Hault simulation time progression
    pauseBtn.setActionCommand("pause");
    timeControlJP.add(pauseBtn);
    pauseBtn.addActionListener(this);
    pauseBtn.setToolTipText("Pause Simulation to Modify Inputs");
    pauseBtn.setEnabled(false);
      // Quit application
    JButton exitBtn = new JButton("Exit");
    exitBtn.addActionListener(new ExitApplicationAction());
    timeControlJP.add(exitBtn);
    exitBtn.addActionListener(this);
    exitBtn.setToolTipText("Exit Entire Application");

    /*
     * Bottom panel enables/disables attitude control.
     */
    JPanel acOnOffJP = new JPanel();
      // Off button, stability control, and inertial options 
    offButton.setActionCommand("atcOff");
    offButton.addActionListener(this);
    offButton.setToolTipText("Disable all Attitude Control");
    scButton.setActionCommand("atcSC");
    scButton.addActionListener(this);
    scButton.setToolTipText("Damp Angular Velocity");
    atiButton.setActionCommand("atcATI");
    atiButton.addActionListener(this);
    atiButton.setToolTipText("Orient Relative to Inertial X-Y-Z Axes");
      // Sub panel for dynamic attitude control options (moving target frame)
    rpyButton.setActionCommand("atcATRPY");
    rpyButton.addActionListener(this);
    rpyButton.setToolTipText("Orient Relative to Nadir & Orbit Plane");
    rpButton.setActionCommand("atcATRP");
    rpButton.addActionListener(this);
    rpButton.setToolTipText(
             "Orient Relative to Reference Point (Right Click for Options)");
        // Popup for target constrained attitude options
      final JPopupMenu rpOptionMenu = new JPopupMenu("Orientation Method");
      ButtonGroup rpOptGroup = new ButtonGroup();
      JMenuItem yaxis_cnt = new JRadioButtonMenuItem(
                                      "Y-Axis Constrained", true);
      yaxis_cnt.addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent event) {
          oSys.setRefPointAttOpt(RefPntAttOpts.RPATT_YAXIS);
        }
      });
      rpOptGroup.add(yaxis_cnt);
      rpOptionMenu.add(yaxis_cnt);
      JMenuItem zyaw_cnt = new JRadioButtonMenuItem("Zero Yaw", false);
      zyaw_cnt.addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent event) {
          oSys.setRefPointAttOpt(RefPntAttOpts.RPATT_ZEROYAW);
        }
      });
      rpOptGroup.add(zyaw_cnt);
      rpOptionMenu.add(zyaw_cnt);
      rpButton.addMouseListener(new MouseAdapter() {
        public void mousePressed(MouseEvent evt) {
          if (evt.isPopupTrigger()) {
            rpOptionMenu.show(evt.getComponent(), evt.getX(), evt.getY());
          }
        }
        public void mouseReleased(MouseEvent evt) {
          if (evt.isPopupTrigger()) {              
            rpOptionMenu.show(evt.getComponent(), evt.getX(), evt.getY());
          }
        }
      });

    JPanel acDynJP = new JPanel();
    acDynJP.setLayout(new BoxLayout(acDynJP, BoxLayout.Y_AXIS));
    acDynJP.add(rpyButton);
    acDynJP.add(rpButton);

    ButtonGroup acGroup = new ButtonGroup();
    acGroup.add(offButton);
    acGroup.add(scButton);
    acGroup.add(atiButton);
    acGroup.add(rpyButton);
    acGroup.add(rpButton);

    acOnOffJP.add(offButton);
    acOnOffJP.add(scButton);
    acOnOffJP.add(atiButton);
    acOnOffJP.add(acDynJP);
    acOnOffJP.setBorder(BorderFactory.createTitledBorder(
              BorderFactory.createEtchedBorder(), "Attitude Control Options")
                       );

      // Middle panel has multiple tabbed panes
    JTabbedPane tabbedPane = new JTabbedPane();
    orbitPanel = new OrbitOrbiterJPanel(fieldsList);
    tabbedPane.addTab("Orbiter", orbitPanel);
    attitudePanel = new AttContSysJPanel(this, fieldsList);
    tabbedPane.addTab("ACS", attitudePanel);
    adsPanel = new AttDetSysJPanel(this);
    tabbedPane.addTab("ADS", adsPanel);
    cbPanel = new CBodyJPanel(fieldsList);
    tabbedPane.addTab("Central Body", cbPanel);
    JPanel keyDefsPanel = new HelpJPanel();
    tabbedPane.addTab("Help", keyDefsPanel);
    JPanel creditsPanel = new CreditsJPanel();
    tabbedPane.addTab("Credits", creditsPanel);

      // Add time control, tabbed pane, and attitude control panel
    cp.add(timeControlJP, BorderLayout.NORTH);
    cp.add(tabbedPane, BorderLayout.CENTER);
    cp.add(acOnOffJP, BorderLayout.SOUTH);

      // Update values to be displayed - get from model
    getModelSettings();

    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    pack();
    setResizable(true);
    setVisible(true);
  }

  /**
   * Points internal gain matrices to those used by the control system.
   *
   * @param   kvIn    Damping matrix (kinetic part of control law)
   * @param   kpIn    Stiffness (potential)
   * @param   axesIn  For Stiffness
   */
  public void setKvKpA(Matrix3X3 kvIn, Matrix3X3 kpIn, Tuple3D axesIn) {
    attitudePanel.setKvKpA(kvIn, kpIn, axesIn);
  }

  /**
   * Called when the Orbiter System has been updated
   * for outputs.  Only values that change with time should
   * be updated.  For example, right now, mass and mass
   * distribution is constant.
   * 
   * @param   handle     Handle of object being observed.
   */
  @Override
  public void touch(int observableHandle) {
    if (observableHandle == ORITERHANDLE) {
        // Display time
      setDisplayTime(oSys.getT());
        // Update orbital elements outputs
      orbitPanel.a_oe.set(oSys.getY(KepEuler.A));
      orbitPanel.e_oe.set(oSys.getY(KepEuler.E));
      orbitPanel.i_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.I));
      orbitPanel.o_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.O));
      orbitPanel.w_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.W));
      orbitPanel.v_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.V));
        // Always display current attitude torques.
      oSys.getAttU(attitudeControlU);
      attitudePanel.uz.refresh(attitudeControlU.get(Basis3D.K));
      attitudePanel.uy.refresh(attitudeControlU.get(Basis3D.J));
      attitudePanel.ux.refresh(attitudeControlU.get(Basis3D.I));
        // Display attitude and desired attitude, if applicable
      OrbiterAttOpts currentAttContOpt = oSys.getAttitudeControlOption();
      if (currentAttContOpt == OrbiterAttOpts.CNT_I   ||
          currentAttContOpt == OrbiterAttOpts.CNT_RPY ||
          currentAttContOpt == OrbiterAttOpts.CNT_RPNT) {
        oSys.getAttitudeDeviations(deltaAtt);
        attitudePanel.yaw.refresh(Angles.DEG_PER_RAD*deltaAtt.get(EulerA.HEAD));
        attitudePanel.pitch.refresh(Angles.DEG_PER_RAD*deltaAtt.get(EulerA.ELEV));
        attitudePanel.roll.refresh(Angles.DEG_PER_RAD*deltaAtt.get(EulerA.BANK));
      }
    }
  }

  /**
   * @param   t   New time to display.
   */
  public void setDisplayTime(double t) {
    status.refreshTime(t);
  }

  /**
   * Catches the "run", "pause", and attitude control button actions.
   * A couple others that don't belong are here too....
   */
  public void actionPerformed(ActionEvent e) {
    if ("pause".equals(e.getActionCommand())) {
      runStop.stop();
      pauseBtn.setEnabled(false);
      runBtn.setEnabled(true);
      setEditables(true);
    } else if ("run".equals(e.getActionCommand())) {
      setEditables(false);
      if (inputsValid()) {
        applyInputs();
        runStop.start();
        runBtn.setEnabled(false);
        pauseBtn.setEnabled(true);
      } else {
        JOptionPane.showMessageDialog(this, "Input Error:  " + lastErrorLabel);
        setEditables(true);
      }
    } else if ("atcOff".equals(e.getActionCommand())) {
      oSys.setAttitudeControlOption(OrbiterAttOpts.CNT_OFF);
      zeroRPY();
    } else if ("atcSC".equals(e.getActionCommand())) {
      oSys.setAttitudeControlOption(OrbiterAttOpts.CNT_STAB);
      zeroRPY();
    } else if ("atcATI".equals(e.getActionCommand())) {
      oSys.setAttitudeControlOption(OrbiterAttOpts.CNT_I);
    } else if ("atcATRPY".equals(e.getActionCommand())) {
      oSys.setAttitudeControlOption(OrbiterAttOpts.CNT_RPY);
    } else if ("atcATRP".equals(e.getActionCommand())) {
      oSys.setAttitudeControlOption(OrbiterAttOpts.CNT_RPNT);
    } else if (attitudePanel.ignoreTqAC.equals(e.getActionCommand())) {
      oSys.disableAttCntTorques(attitudePanel.ignoreAttTqsCB.isSelected());
    } else if (adsPanel.estAttAC.equals(e.getActionCommand())) {
      attitudeDetermination();
    } else if (adsPanel.recAttAC.equals(e.getActionCommand())) {
      attitudeDetRec();
    }
  }

  /**
   * Loops through the list of registered editable objects
   * and sets them to editable depending on the passed in state.
   * 
   * @param   state    If true, then set to editable.  False, then
   *                   not editable.
   */
  private void setEditables(boolean state) {
    IEditError ee = null;
    int num = fieldsList.size();
    for (int ii=0; ii<num; ii++) {
      ee = fieldsList.get(ii);
      ee.setEditable(state);
    }
  }

  /*
   * Returns true if none of the registered IEditError objects
   * are reporting an error.
   */
  private boolean inputsValid() {
    IEditError ee = null;
    int num = fieldsList.size();
    for (int ii=0; ii<num; ii++) {
      ee = fieldsList.get(ii);
      if (ee.getErrorFlag()) {
        lastErrorLabel = ee.getErrorLabel();
        return false;
      }
    }
    return true;
  }

  /*
   * Updates oSys values with inputs from this frame.
   * Should only be called after calling inputsValid() to verify
   * everything is kosher.  Should match stuff in getModelSettings().
   */
  private void applyInputs() {
    cbSys.setAngularVelocity(cbPanel.omega.get());
    KeplerianOE kep = new KeplerianOE();
    kep.setGM(gm);
    kep.put(Keplerian.A, orbitPanel.a_oe.get());
    kep.put(Keplerian.E, orbitPanel.e_oe.get());
    kep.put(Keplerian.I, Angles.RAD_PER_DEG*orbitPanel.i_oe.get());
    kep.put(Keplerian.O, Angles.RAD_PER_DEG*orbitPanel.o_oe.get());
    kep.put(Keplerian.W, Angles.RAD_PER_DEG*orbitPanel.w_oe.get());
    kep.put(Keplerian.V, Angles.RAD_PER_DEG*orbitPanel.v_oe.get());
    oSys.setOE(kep);
    oSys.setMass(orbitPanel.mass.get());
    oSys.putJ(Basis3D.I, Basis3D.I, orbitPanel.ixx.get());
    oSys.putJ(Basis3D.J, Basis3D.J, orbitPanel.iyy.get());
    oSys.putJ(Basis3D.K, Basis3D.K, orbitPanel.izz.get());
    oSys.putJ(Basis3D.I, Basis3D.J, orbitPanel.ixy.get());
    oSys.putJ(Basis3D.I, Basis3D.K, orbitPanel.ixz.get());
    oSys.putJ(Basis3D.J, Basis3D.K, orbitPanel.iyz.get());
    desiredAttitude.putDeg(EulerA.HEAD, attitudePanel.heading.get());
    desiredAttitude.putDeg(EulerA.ELEV, attitudePanel.elevation.get());
    desiredAttitude.putDeg(EulerA.BANK, attitudePanel.bank.get());
    oSys.setDesiredAttitude(desiredAttitude);
    attitudePanel.kv.get(attitudePanel.kvM);
    attitudePanel.kp.get(attitudePanel.kpM);
    attitudePanel.axes.get(attitudePanel.axesV);
    Tuple3D rpPos = new Tuple3D(cbPanel.rpntx.get(),
                                cbPanel.rpnty.get(),
                                cbPanel.rpntz.get());
    rpSys.setRelativePosition(rpPos);
  }

  /*
   * Sets input field values based on the OrbiterSys settings.
   * Meant to be called at initialization.  Should match stuff in
   * applyInputs().
   */
  private void getModelSettings() {
    zeroRPY();
    cbPanel.omega.set(cbSys.getAngularVelocity());
    orbitPanel.a_oe.set(oSys.getY(KepEuler.A));
    orbitPanel.e_oe.set(oSys.getY(KepEuler.E));
    orbitPanel.i_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.I));
    orbitPanel.o_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.O));
    orbitPanel.w_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.W));
    orbitPanel.v_oe.set(Angles.DEG_PER_RAD*oSys.getY(KepEuler.V));
    orbitPanel.mass.set(oSys.getMass());
    orbitPanel.ixx.set(oSys.getJ(Basis3D.I, Basis3D.I));
    orbitPanel.iyy.set(oSys.getJ(Basis3D.J, Basis3D.J));
    orbitPanel.izz.set(oSys.getJ(Basis3D.K, Basis3D.K));
    orbitPanel.ixy.set(oSys.getJ(Basis3D.I, Basis3D.J));
    orbitPanel.ixz.set(oSys.getJ(Basis3D.I, Basis3D.K));
    orbitPanel.iyz.set(oSys.getJ(Basis3D.J, Basis3D.K));
    oSys.getDesiredAttitude(desiredAttitude);
    attitudePanel.heading.set(desiredAttitude.getDeg(EulerA.HEAD));
    attitudePanel.elevation.set(desiredAttitude.getDeg(EulerA.ELEV));
    attitudePanel.bank.set(desiredAttitude.getDeg(EulerA.BANK));
    attitudePanel.kv.set(attitudePanel.kvM);
    attitudePanel.kp.set(attitudePanel.kpM);
    attitudePanel.axes.set(attitudePanel.axesV);
    Tuple3D rpPos = new Tuple3D();
    rpSys.getRelativePostion(rpPos);
    cbPanel.rpntx.set(rpPos.get(Basis3D.I));
    cbPanel.rpnty.set(rpPos.get(Basis3D.J));
    cbPanel.rpntz.set(rpPos.get(Basis3D.K));
  }

  /*
   * Sets output deviations from desired attitude to zero.
   */
  private void zeroRPY() {
    attitudePanel.yaw.refresh(0.0);
    attitudePanel.pitch.refresh(0.0);
    attitudePanel.roll.refresh(0.0);
  }

  /*
   * Performs *batch* attitude determination given simulated star tracker
   * measurements and the primary attitude determination method
   * du jour.
   */
  private void attitudeDetermination() {
    DecimalFormat df = new DecimalFormat("0.0000");
    SimpleConeTracker[] trackers = adsPanel.adsTableModel.getConeTrackers();

    Tuple3D position = new Tuple3D();
    Quaternion attitude = new Quaternion();
    double sysTime = oSys.getT();
    oSys.getPosition(sysTime, position);
    oSys.getAttitude(sysTime, attitude);

    if (trackers != null) {
        // Call upon each tracker to make measurements
      for (int ii=0; ii<trackers.length; ii++) {
        trackers[ii].measure(position, attitude);
      }

        // Set output time
      adsPanel.attTime.refreshTime(sysTime);
        // Create attitude determination objects
        // TRIAD makes a good reference method.
      AttitudeDetTRIAD triadADS = new AttitudeDetTRIAD();
      AttitudeDetQuat ads = new AttitudeDetQuat();
      //AttitudeDetQuatC ads = new AttitudeDetQuatC();
      AttitudeDetDQuat ads2 = new AttitudeDetDQuat();
      Quaternion triadAtt = new Quaternion();
      Quaternion estAtt = new Quaternion();
        // For outputs
      EulerAngles truthRPY = new EulerAngles();
      truthRPY.fromQuatFrameRot(attitude);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(truthRPY.getDeg(EulerA.HEAD)), 0, 1);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(truthRPY.getDeg(EulerA.ELEV)), 0, 2);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(truthRPY.getDeg(EulerA.BANK)), 0, 3);

        // First TRIAD - estimate and output
      EulerAngles estRPY = new EulerAngles();
      int nitr = triadADS.estimateAtt(trackers);
      triadAtt.set(triadADS);
      estRPY.fromQuatFrameRot(triadAtt);
      EulerAngles deltaRPY = new EulerAngles();
      deltaRPY.minus(truthRPY, estRPY);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.HEAD)), 1, 1);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.ELEV)), 1, 2);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.BANK)), 1, 3);
      if (nitr >= 0) {
        adsPanel.adsOutTableModel.setValueAt("-", 1, 4);
      } else {
        adsPanel.adsOutTableModel.setValueAt("X", 1, 4);
      }

        // WLS directly solving for quaternion - estimate and output
      nitr = ads.estimateAtt(trackers);
      estAtt.set(ads);
      estRPY.fromQuatFrameRot(estAtt);
      deltaRPY.minus(truthRPY, estRPY);
      double droll = deltaRPY.getDeg(EulerA.BANK);
      double dpitch = deltaRPY.getDeg(EulerA.ELEV);
      double dyaw = deltaRPY.getDeg(EulerA.HEAD);
      adsPanel.adsOutTableModel.setValueAt(df.format(dyaw), 2, 1);
      adsPanel.adsOutTableModel.setValueAt(df.format(dpitch), 2, 2);
      adsPanel.adsOutTableModel.setValueAt(df.format(droll), 2, 3);
      if (nitr >= 0) {
        adsPanel.adsOutTableModel.setValueAt(new Integer(nitr), 2, 4);
        Matrix qCov = ads.covariance();
        Matrix eCov = new Matrix(estRPY.length());
        DEulerDQuat dedq = new DEulerDQuat();
        dedq.partials(estAtt);
        eCov.transform(dedq, qCov);
          // Three Sigma
        double sigR = 3.0*Math.toDegrees(Math.sqrt(eCov.get(1, 1)));
        double sigP = 3.0*Math.toDegrees(Math.sqrt(eCov.get(2, 2)));
        double sigY = 3.0*Math.toDegrees(Math.sqrt(eCov.get(3, 3)));
        adsPanel.adsOutTableModel.setValueAt(df.format(sigY), 3, 1);
        adsPanel.adsOutTableModel.setValueAt(df.format(sigP), 3, 2);
        adsPanel.adsOutTableModel.setValueAt(df.format(sigR), 3, 3);
        boolean rok = (sigR-Math.abs(droll) >= 0.0) ? true : false;
        boolean pok = (sigP-Math.abs(dpitch) >= 0.0) ? true : false;
        boolean yok = (sigY-Math.abs(dyaw) >= 0.0) ? true : false;
        if (rok && pok && yok) {
          adsPanel.adsOutTableModel.setValueAt("\u2713", 3, 4);
        } else {
          adsPanel.adsOutTableModel.setValueAt("X", 3, 4);
        }
        if (adsPanel.recAttCB.isSelected()) {
          AttDetData ad = new AttDetData();
          ad.time = sysTime;
          ad.truthAtt.set(attitude);
          ad.triadAtt.set(triadAtt);
          ad.estAtt.set(estAtt);
          ad.attCov.set(qCov);
          ad.deltaEuler.set(deltaRPY);
          ad.eulerCov.set(eCov);
          attDetDataList.add(ad);
        }
      } else {
        adsPanel.adsOutTableModel.setValueAt("X", 2, 4);
        adsPanel.adsOutTableModel.setValueAt("X", 3, 1);
        adsPanel.adsOutTableModel.setValueAt("X", 3, 2);
        adsPanel.adsOutTableModel.setValueAt("X", 3, 3);
        adsPanel.adsOutTableModel.setValueAt("X", 3, 4);
      }

        // WLS solving for quaternion error - estimate and output
      nitr = ads2.estimateAtt(trackers);
      estAtt.set(ads2);
      estRPY.fromQuatFrameRot(estAtt);
      deltaRPY.minus(truthRPY, estRPY);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.HEAD)), 4, 1);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.ELEV)), 4, 2);
      adsPanel.adsOutTableModel.setValueAt(
                          df.format(deltaRPY.getDeg(EulerA.BANK)), 4, 3);
      if (nitr >= 0) {
        adsPanel.adsOutTableModel.setValueAt(new Integer(nitr), 4, 4);
      } else {
        adsPanel.adsOutTableModel.setValueAt("X", 4, 4);
      }
    }
  }

  /*
   * Writes attitude residual information to a file.  When the
   * same file is selected for a save, it is overwritten.  This
   * function really exists just for developer experiments and
   * validation efforts....
   */
  private void attitudeDetRec() {
    if (adsPanel.recAttCB.isSelected()) {
        // When selected, start with a fresh list
      attDetDataList.clear();
    } else {
        // Write ArrayList to tape when record function is turned off.
      JFileChooser saveFileChooser = new JFileChooser();
      if (attDetDataDir != null) {
        saveFileChooser.setCurrentDirectory(attDetDataDir);
      }
      int returnVal = saveFileChooser.showOpenDialog(this);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
          // Grab current directory for next save request.
        attDetDataDir = saveFileChooser.getCurrentDirectory();
        File attDetFile = saveFileChooser.getSelectedFile();
        double deltaSD, deltaID, deltaJD, deltaKD;   // Deterministic (TRIAD)
        double deltaS, deltaI, deltaJ, deltaK;       // Estimated
        double sigQ0, sigQI, sigQJ, sigQK;           // Standard Deviation
        try {
          FileWriter fw = new FileWriter(attDetFile);
          BufferedWriter bw = new BufferedWriter(fw);
          PrintWriter pw = new PrintWriter(bw);
            // Print Header First
          pw.printf("\nTime");
          pw.printf("\t\t\tdq0D dq0 3sig q0");
          pw.printf("\t\t\tdqiD dqi 3sig qi");
          pw.printf("\t\t\tdqjD dqj 3sig qj");
          pw.printf("\t\t\tdqkD dqk 3sig qk");
          pw.printf("\t\tdroll 3sig roll (rad)");
          pw.printf("\t\tdpitch 3sig pitch (rad)");
          pw.printf("\t\tdyaw 3sig yaw (rad)");
          pw.printf("\t\tdAngle 3sig Angle (rad)");
            // Loop over recored attitude data - delta then sigma
          DAngDQuat dadq = new DAngDQuat();
          Matrix angCov = new Matrix(1);
          Tuple3D eigAxis = new Tuple3D();
          double ang, dang;
          for (AttDetData ad : attDetDataList) {
            ad.truthAtt.standardize();
            ad.triadAtt.standardize();
            ad.estAtt.standardize();
              // Differences from deterministic method
            deltaSD = ad.truthAtt.get(Q.Q0) - ad.triadAtt.get(Q.Q0);
            deltaID = ad.truthAtt.get(Q.QI) - ad.triadAtt.get(Q.QI);
            deltaJD = ad.truthAtt.get(Q.QJ) - ad.triadAtt.get(Q.QJ);
            deltaKD = ad.truthAtt.get(Q.QK) - ad.triadAtt.get(Q.QK);
              // Differences from estimation method
            deltaS = ad.truthAtt.get(Q.Q0) - ad.estAtt.get(Q.Q0);
            deltaI = ad.truthAtt.get(Q.QI) - ad.estAtt.get(Q.QI);
            deltaJ = ad.truthAtt.get(Q.QJ) - ad.estAtt.get(Q.QJ);
            deltaK = ad.truthAtt.get(Q.QK) - ad.estAtt.get(Q.QK);
              // Bump up to 3-sigma for containment checks
            sigQ0 = 3.0*Math.sqrt(ad.attCov.get(1,1));
            sigQI = 3.0*Math.sqrt(ad.attCov.get(2,2));
            sigQJ = 3.0*Math.sqrt(ad.attCov.get(3,3));
            sigQK = 3.0*Math.sqrt(ad.attCov.get(4,4));
            pw.printf("\n%f\t", ad.time);
            pw.printf("%e %e %e\t%e %e %e\t%e %e %e\t%e %e %e",
                      deltaSD, deltaS, sigQ0, deltaID, deltaI, sigQI, 
                      deltaJD, deltaJ, sigQJ, deltaKD, deltaK, sigQK);
            pw.printf("\t%e %e\t%e %e\t%e %e", 
                      ad.deltaEuler.get(EulerA.BANK),
                      3.0*Math.sqrt(ad.eulerCov.get(1,1)), 
                      ad.deltaEuler.get(EulerA.ELEV), 
                      3.0*Math.sqrt(ad.eulerCov.get(2,2)), 
                      ad.deltaEuler.get(EulerA.HEAD),
                      3.0*Math.sqrt(ad.eulerCov.get(3,3)));
            ang = ad.estAtt.axisAngle(eigAxis);
            dang = ad.truthAtt.axisAngle(eigAxis) - ang;
            dadq.partials(ad.estAtt);
            angCov.transform(dadq, ad.attCov);
            pw.printf("\t%e %e", Math.abs(dang), 3.0*Math.sqrt(angCov.get(1,1)));
          }
          bw.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
  }

  /*
   * Holds residual and covariance information related to the
   * attitude determination function.
   */
  private class AttDetData {
    double time = 0.0;
    Quaternion truthAtt = new Quaternion();    // From EOM
    Quaternion triadAtt = new Quaternion();    // TRIAD
    Quaternion estAtt = new Quaternion();      // Estimated
    Matrix attCov = new Matrix(4);
    EulerAngles deltaEuler = new EulerAngles();
    Matrix eulerCov = new Matrix(3);
  }
}
