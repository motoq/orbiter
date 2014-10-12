/*
 c  OrbiterStartupFrame.java
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
import java.awt.event.*;
import javax.swing.*;

import com.motekew.vse.envrm.Gravity;
import com.motekew.vse.envrm.WGS84EGM96Ref;
import com.motekew.vse.math.Matrix;
import com.motekew.vse.math.SphericalHarmonicCoeff;
import com.motekew.vse.servm.*;
import com.motekew.vse.ui.*;

/**
 * Startup window for the Orbiter simulation.  The user can set the central
 * body properties (gravitational parameter and coefficients, scaling radius,
 * and angular velocity).  In addition, integration step size options are available.
 * <P>
 * Not all that pretty, just a boring GUI.  Not much error checking done - user beware.
 * 
 * @author   Kurt Motekew
 * @since    20120128
 */
public class OrbiterStartupFrame extends JFrame implements ActionListener {

  private static final double MAX_VALUE = IORanges.MAX_VALUE;
    // Number of gravitational coefficients
  private static final int NGC = 5;

    // Integration step options
  private static final int[] fineVals = { 25, 2, 9 };
  private static final int[] mediumVals = { 50, 2, 4 };
  private static final int[] coarseVals = { 1000, 1, 0 };
  
  private int[] integVals = fineVals;

    // Used to keep track of input fields with errors before launch
  private ArrayList<ErrorReportable> errorList =
                                     new ArrayList<ErrorReportable>();

  private boolean sleeping = true;
  private JCheckBox useEarthValuesCB = new JCheckBox("Use Earth Model");
  private JCheckBox useCanonicalCB = new JCheckBox("Canonical Earth Model");
  private JRadioButton fineIntegBtn = new JRadioButton("Fine", true);
  private JRadioButton mediumIntegBtn = new JRadioButton("Medium", false);
  private JRadioButton coarseIntegBtn = new JRadioButton("Coarse", false);
  private Matrix clM;
  private Matrix slM;

    // Gravitational parameter field
  private DecimalMinMaxJTF mu = new DecimalMinMaxJTF(11, 1e-30, MAX_VALUE);
    // Scaling radius field
  private DecimalMinMaxJTF re = new DecimalMinMaxJTF(11, 1e-30, MAX_VALUE);
    // Angular velocity of central body about spin axis
  private DecimalMinMaxJTF omega = new DecimalMinMaxJTF(11, -MAX_VALUE, MAX_VALUE);
    // Solid model scale factor
  private DecimalMinMaxJTF sms = new DecimalMinMaxJTF(4, 0.01, 10.0);
    // Gravitational coefficients
  private JCheckBox normalizedJCB = new JCheckBox("Normalized");
  private MatrixJP clmJP = new MatrixJP(NGC, NGC, 6);
  private MatrixJP slmJP = new MatrixJP(NGC, NGC, 6);

  /**
   * Creates the window, but doesn't display anything.
   */
  public OrbiterStartupFrame() {
      // Default Gravitational Coefficients - Kidney Bean
    double[][] clm = { { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.5,   -0.2,    0.0,    0.0,    0.0},
                       { 0.3,    0.0,    0.0,    0.0,    0.0},
                       { 0.2,    0.0,    0.0,    0.0,    0.0} };
    double[][] slm = { { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.0,    0.0,    0.0,    0.0,    0.0},
                       { 0.0,    0.0,    0.0,    0.0,    0.0} };
    clM = new Matrix(clm);
    slM = new Matrix(slm);
    clmJP.set(clM);
    slmJP.set(slM);
    int dim = clM.numRows();
    for (int ll=1; ll<=dim; ll++) {
      slmJP.setEnabled(false, ll, 1);
      for (int mm=1; mm<=dim; mm++) {
        if (mm > ll) {
          clmJP.setEnabled(false, ll, mm);
          slmJP.setEnabled(false, ll, mm);
        }
      }
    }
    clmJP.setEnabled(false, 1, 1);
    clmJP.setEnabled(false, 2, 1);
    clmJP.setEnabled(false, 2, 2);
    slmJP.setEnabled(false, 2, 2);
    mu.set(1.0);
    re.set(1.0);
    omega.set(0.0);
    sms.set(1.0);

    setTitle("Central Body Gravitational Properties");
    Container cp = getContentPane();

      // Use default FlowLayout
    JPanel runJP = new JPanel();
    runJP.setBorder(BorderFactory.createEtchedBorder());
    JButton runBtn = new JButton("Start Simulation");
    runBtn.setActionCommand("startSim");
    runBtn.addActionListener(this);
      //
    useEarthValuesCB.setActionCommand("useEarthValues");
    useEarthValuesCB.addActionListener(this);
    useCanonicalCB.setActionCommand("useCanonical");
    useCanonicalCB.addActionListener(this);
    useCanonicalCB.setEnabled(false);
    JPanel evOptJP = new JPanel();
    evOptJP.setLayout(new BoxLayout(evOptJP, BoxLayout.Y_AXIS));
    evOptJP.add(useEarthValuesCB);
    evOptJP.add(useCanonicalCB);

      //
    fineIntegBtn.setActionCommand("fineInteg");
    mediumIntegBtn.setActionCommand("mediumInteg");
    coarseIntegBtn.setActionCommand("coarseInteg");
    fineIntegBtn.addActionListener(this);
    mediumIntegBtn.addActionListener(this);
    coarseIntegBtn.addActionListener(this);
    fineIntegBtn.setToolTipText(
                 "0.025 TU Integration Step and 50 ms J3D Update Rate");
    mediumIntegBtn.setToolTipText(
                   "0.05 TU Integration Step and 100 ms J3D Update Rate");
    coarseIntegBtn.setToolTipText(
                   "1.0 TU Integration Step and 1000 ms J3D Update Rate");
    JPanel integOptJP = new JPanel();
    integOptJP.setLayout(new BoxLayout(integOptJP, BoxLayout.Y_AXIS));
    integOptJP.add(fineIntegBtn);
    integOptJP.add(mediumIntegBtn);
    integOptJP.add(coarseIntegBtn);
    ButtonGroup integGroup = new ButtonGroup();
    integGroup.add(fineIntegBtn);
    integGroup.add(mediumIntegBtn);
    integGroup.add(coarseIntegBtn);
      //
    runJP.add(evOptJP);
    runJP.add(runBtn);
    runJP.add(integOptJP);

    JPanel scalingJP = new JPanel();
    scalingJP.setBorder(BorderFactory.createTitledBorder(
                        BorderFactory.createEtchedBorder(), "Primary Terms")
                       );
    JPanel mureJP = new JPanel();
    mureJP.setLayout(new BoxLayout(mureJP, BoxLayout.Y_AXIS));
      //
    JPanel muJP = new JPanel();
    muJP.setLayout(new BorderLayout());
    mu.setHorizontalAlignment(JTextField.RIGHT);
    muJP.add(new JLabel("Gravitational Parameter (DU\u00B3/TU\u00B2):  "),
                        BorderLayout.WEST);
    muJP.add(mu, BorderLayout.EAST);
    errorList.add(mu);
    mu.setFormat("0.#########E0");
    mu.setErrorLabel("Gravitational Parameter");
      //
    JPanel reJP = new JPanel();
    reJP.setLayout(new BorderLayout());
    re.setHorizontalAlignment(JTextField.RIGHT);
    reJP.add(new JLabel("Gravitational Scaling Radius (DU):  "),
                        BorderLayout.WEST);
    reJP.add(re, BorderLayout.EAST);
    errorList.add(re);
    re.setFormat("0.########E0");
    re.setErrorLabel("Scaling Radius");
      //
    JPanel omegaJP = new JPanel();
    omegaJP.setLayout(new BorderLayout());
    omega.setHorizontalAlignment(JTextField.RIGHT);
    omegaJP.add(new JLabel("Central Body Rotation Rate (rad/TU):  "),
                           BorderLayout.WEST);
    omegaJP.add(omega, BorderLayout.EAST);
    errorList.add(omega);
    omega.setFormat("0.########E0");
    omega.setErrorLabel("CB Rotation Rate");
      //
    JPanel smsJP = new JPanel();
    smsJP.setLayout(new BorderLayout());
    sms.setHorizontalAlignment(JTextField.RIGHT);
    smsJP.add(new JLabel("Solid Model Scale Factor:  "),
                         BorderLayout.WEST);
    smsJP.add(sms, BorderLayout.EAST);
    errorList.add(sms);
    sms.setFormat("0.00");
    sms.setErrorLabel("Model Scale Factor");
      //
    mureJP.add(muJP);
    mureJP.add(reJP);
    mureJP.add(omegaJP);
    mureJP.add(smsJP);
    scalingJP.add(mureJP);

      // Gravitational coefficients
    JPanel gStuffJP = new JPanel();
    gStuffJP.setLayout(new BorderLayout());
    gStuffJP.setBorder(BorderFactory.createTitledBorder(
                       BorderFactory.createEtchedBorder(),
                                     "Gravitational Coefficients")
                      );
    normalizedJCB.setToolTipText(
                  "Select if Inputting Kaula Normalized Coefficients");
    JPanel gCoeffJP = new JPanel();
    gCoeffJP.setLayout(new BoxLayout(gCoeffJP, BoxLayout.X_AXIS));
      //
    JPanel clJP = new JPanel();
    clJP.add(new JLabel(" Clm:  "));
    clJP.add(clmJP);
    clmJP.setFormat("0.0##E0");
    clmJP.setErrorLabel("Clm");
    errorList.add(clmJP);
    clJP.add(clmJP);
      //
    JPanel slJP = new JPanel();
    slJP.add(new JLabel(" Slm:  "));
    slJP.add(slmJP);
    slmJP.setFormat("0.0##E0");
    slmJP.setErrorLabel("Slm");
    errorList.add(slmJP);
    slJP.add(slmJP);
      //
    gCoeffJP.add(clJP);
    gCoeffJP.add(slJP);
    gStuffJP.add(normalizedJCB, BorderLayout.NORTH);
    gStuffJP.add(gCoeffJP, BorderLayout.SOUTH);

    cp.add(scalingJP, BorderLayout.NORTH);
    cp.add(runJP, BorderLayout.SOUTH);
    cp.add(gStuffJP, BorderLayout.CENTER);

      // If the user closes the window instead of hitting run, shut down.
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    
    pack();
    setResizable(false);
  }

  /**
   * Waits for the user to accept/set central body settings and continue with
   * the simulation.
   * 
   * @return    Gravitational parameters, including scaling parameters (scaling
   *            radius and gravitational parameter).
   */
  public Gravity getGravity() {
    setVisible(true);
    try { 
      while (sleeping) {
        Thread.sleep(500);
      }
    } catch(InterruptedException ie) {
      System.err.println("Orbiter Startup Sleep Interrupted");
    }

    // Run command was hit, now grab current inputs, create Gravity object,
    // and return.

      // Central body shape
    clM = clmJP.get();
    slM = slmJP.get();
      // Resize to minimum nonzero dimension
    int[] clmMN = clM.trimToMN();
    int[] slmMN = slM.trimToMN();
    int maxDim = (clmMN[0] > clmMN[1]) ? clmMN[0] : clmMN[1];
    maxDim = (slmMN[0] > maxDim) ? slmMN[0] : maxDim;
    maxDim = (slmMN[1] > maxDim) ? slmMN[1] : maxDim;
    clM = clM.trimToMN(maxDim, maxDim);
    slM = slM.trimToMN(maxDim,  maxDim);
      // Now init gravity model
    double[][] clm = clM.values();
    double[][] slm = slM.values();
    boolean normalized = normalizedJCB.isSelected();
    SphericalHarmonicCoeff shc = new SphericalHarmonicCoeff(normalized, clm, slm);
    Gravity geo = new Gravity(mu.get(), re.get(), shc);

    return geo;
  }

  /**
   * Indicates if the use of earth gravitational parameters is selected.
   * This does not garantee the earth values are set as the user can still
   * change them after selecting this option.  However, if checked, the
   * orbiter model will use a sphere with an earth texture to represent the
   * central body.
   * 
   * @return    If true, then the earth model is selected.
   */
  public boolean earthModelSelected() {
    return useEarthValuesCB.isSelected();
  }

  /**
   * @return    Angular velocity of central body, in radians
   */
  public double getOmega() {
    return omega.get();
  }

  /**
   * @return    Scale factor to be used on solid models
   */
  public double getModelScale() {
    return sms.get();
  }

  /**
   * @return    A three element array of integers composed of the integration
   *            step size (ms), the number of integration steps per J3D refresh,
   *            And the number of integration steps to skip between outputs.
   */
  public int[] getTimeSteps() {
    return integVals;
  }

  public void actionPerformed(ActionEvent e) {
    if ("startSim".equals(e.getActionCommand())) {
      int errorIndex = ErrorReportUtil.getErrorLabelIndex(errorList);
      if (errorIndex < 0) {
        sleeping = false;
      } else {
        ErrorReportable er = errorList.get(errorIndex);
        JOptionPane.showMessageDialog(this, "Input Error:  " +
                                            er.getErrorLabel());
      }
    } else if ("useEarthValues".equals(e.getActionCommand())) {
      if (useEarthValuesCB.isSelected()) {
        WGS84EGM96Ref cbRef = new WGS84EGM96Ref();
        double m_p_er = cbRef.metersPerDU();
        double s_p_tu = cbRef.secondsPerTU();
          // Coefficients
        clM = cbRef.unnormalizedGravityCosCoeff(NGC-1, NGC-1);
        slM = cbRef.unnormalizedGravitySinCoeff(NGC-1, NGC-1);
          // Semimajor and grav param - convert to SI
        double a_er = cbRef.gravitationalReferenceRadius();
        double gm_erpmin = cbRef.gravitationalParameter();
        double gm_mpsec = gm_erpmin*m_p_er*m_p_er*m_p_er/
                                   (s_p_tu*s_p_tu);
        mu.set(gm_mpsec);                              // m^3/s^2
        re.set(a_er*m_p_er);                           // m
        omega.set(cbRef.angularVelocity()/s_p_tu);     // rad/s  

        clmJP.set(clM);
        slmJP.set(slM);        

        useCanonicalCB.setEnabled(true);
        normalizedJCB.setSelected(false);
      } else {
        useCanonicalCB.setEnabled(false);
        useCanonicalCB.setSelected(false);
      }
    } else if (useCanonicalCB.isSelected()  &&
               "useCanonical".equals(e.getActionCommand())) {
      mu.set(1.0);
      re.set(1.0);
      omega.set(0.0588335998015);
    } else if ("fineInteg".equals(e.getActionCommand())) {
      integVals = fineVals;
    } else if ("mediumInteg".equals(e.getActionCommand())) {
      integVals = mediumVals;
    } else if ("coarseInteg".equals(e.getActionCommand())) {
      integVals = coarseVals;
    }
  }
}
