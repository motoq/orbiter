/*
 c  Orbiter.java
 c
 c  Copyright (C) 2011, 2012 Kurt Motekew
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

import java.net.URL;
import javax.media.j3d.*;
import javax.vecmath.Color3f;

import com.motekew.vse.c0ntm.AttitudeControlDCM;
import com.motekew.vse.sensm.SimpleConeTracker;
import com.motekew.vse.cycxm.ModelStepper;
import com.motekew.vse.enums.Basis3D;
import com.motekew.vse.enums.XdX6DQ;
import com.motekew.vse.enums.EulerA;
import com.motekew.vse.envrm.*;
import com.motekew.vse.j3d.*;
import com.motekew.vse.j3d.meshmodels.SparkySpacecraftMat;
import com.motekew.vse.math.Matrix3X3;
import com.motekew.vse.math.Tuple3D;
import com.motekew.vse.math.Quaternion;
import com.motekew.vse.servm.ISimModel;
import com.motekew.vse.trmtm.EulerAngles;
import com.motekew.vse.trmtm.KeplerianOE;
import com.motekew.vse.trmtm.ReferencePointSys;
import com.motekew.vse.trmtm.RotatingBodySys;

import com.motekew.orbiter.gui.OrbiterStartupFrame;
import com.motekew.orbiter.gui.OrbiterInputsFrame;

/**
 * Orbiter model.
 * <P>
 * This class is the starting point for a simple six degree
 * of freedom spacecraft simulation with a single thruster acting
 * along the longitudinal axis propelling the spacecraft forward.
 * This means, to stop, the spacecraft must be flipped around, and
 * thrust must be applied in a direction opposing the motion.  Turns
 * require bursts of thrust, combined with maneuvers to direct the thrust
 * appropriately.
 * <P>
 * The spacecraft is subject to a single outside environmental factor,
 * a gravitational body at the center of the inertial (computational)
 * reference frame.
 * <P>
 * Torques can be applied about each axis to change the attitude
 * of the spacecraft.  Note that the choice of an inertia matrix
 * with non-zero off diagonal elements will result in a coupling
 * effect between axes even when torques are applied about a
 * single axis, and the spacecraft is starting off with no rotational
 * motion.  Also note that gyroscopic effects result in a highly
 * coupled system.  If the spacecraft is rotating about a single
 * axis, then a torque applied about another axis will result in
 * a rotation about another axis.  Also, coupling results in an
 * acceleration about a third axis due to angular rates about
 * two other axes.  Controlling the spacecraft attitude can be
 * very difficult when the attitude control system is not active.
 * <P>
 * An attitude control system has been implemented.  See
 * <code>OrbiterSys</code>.
 * <P>
 * Preliminary attitude determination has been implemented.
 * <P>
 *  Note that the ModelTimeBehavior modelTimer, which controls the
 *  Java3D refresh rate, drives the simulation clock.
 *
 * @author  Kurt Motekew
 * @since   20081225
 */
public class Orbiter implements ISimModel {

  public static final int NUMMODELS = 3;
  public static final int N_STAR_TRACKERS = 3;

  /**
   * Launches the simulation - satisfies ISimModel Interface
   */
  @Override
  public void launch() {
      // Visualization environment configuration - no grid, just axes
    SimV3Dcfg cfg = new SimV3Dcfg();
    cfg.setPlotGrid(false);

      // Get initial inputs that don't change during the simulation.
      // No further execution until returns from getGravity()
    OrbiterStartupFrame startFrame = new OrbiterStartupFrame();
    Gravity geo = startFrame.getGravity();
    startFrame.dispose();
    int[] timeSteps = startFrame.getTimeSteps();
    cfg.setDT(timeSteps[0], timeSteps[1], timeSteps[2]);
    double duScale = 1.0/geo.getRefRadius();
    cfg.setDU(duScale);
    boolean useEarthModel = startFrame.earthModelSelected();

      // vehicle solid model scale
    double solidModelScale = startFrame.getModelScale();

      // This Behavior is a timer that tells the Java3D display
      // to update.  This, in turn, results in a request to update
      // each of the dyamic models (IVisualModel) that are registered.
      // This Behavior also tracks the total Simulation Time.
    ModelTimeBehavior modelTimer = new ModelTimeBehavior(cfg.getJDT());
    modelTimer.stop();
    modelTimer.setSchedulingBounds(cfg.getBoundingSphere());
    //modelTimer.setTimeScaleFactor(3.0);

      // Integration step size control object.  Initialize with default
      // integration step size and Java3D refresh rate (converted from
      // milliseconds to seconds.
    ModelStepper mStepper = new ModelStepper();
    mStepper.setDt(cfg.getDT()); 
    mStepper.setODt( ((double) cfg.getJDT()) / 1000.0);

      // Central body - creates shape used in display based on gravitational
      // model
    RotatingBodySys centralBodySys = new RotatingBodySys();
    centralBodySys.setT(modelTimer.getSimulationTime());
    centralBodySys.setSpinAxis(0.0, 
                 new Tuple3D(  new double[] {0.0, 0.0, 5.0}  ));
    centralBodySys.setAngularVelocity(startFrame.getOmega());
      // Tie together the gravity model and the dynamic model
    RotatingCentralBody rcb = new RotatingCentralBody(centralBodySys, geo);

      // Reference point used for stuff
    ReferencePointSys rps = new ReferencePointSys(0.0, 0.0, 0.0);
    rps.setT(modelTimer.getSimulationTime());
    rps.setPointOfOrigin(centralBodySys);

      // Create vehicle model and add gravity influence.
    OrbiterSys vehicleSys = new OrbiterSys();
      // Set mass and principal axes inertial moments
    vehicleSys.setMass(1.0);
    vehicleSys.putJ(Basis3D.I, Basis3D.I, 1.92);
    vehicleSys.putJ(Basis3D.J, Basis3D.J, 9.38);
    vehicleSys.putJ(Basis3D.K, Basis3D.K, 10.7);
      // Only XZ plane of symmetry if non-zero
    vehicleSys.putJ(Basis3D.I, Basis3D.K, 0.169);

      // Initialize orbiter star trackers
    SimpleConeTracker[] trackers = new SimpleConeTracker[N_STAR_TRACKERS];
      // Body to tracker:       Roll    Pitch   Yaw
    double[][] tkr_atts = { {  135.0,  -45.0,   0.0 },
                            { -135.0,  -45.0,   0.0 },
                            {    0.0,    0.0,   0.0 }};
    EulerAngles ea = new EulerAngles();
    Quaternion q_b2s = new Quaternion();
    int    maxMeas   = 10;                         // Max 10 meas/tracker
    double dvalcone  = Math.toRadians(10.0);       // Full Conewidth
    double dvalsigma = Math.toRadians(0.001);      // 1-sigma meas uncertainty
    for (int ii=0; ii<N_STAR_TRACKERS; ii++) {
      trackers[ii] = new SimpleConeTracker(vehicleSys, maxMeas);
      trackers[ii].setConeWidth(dvalcone);
      trackers[ii].setRandomError(dvalsigma);
      ea.put(EulerA.BANK, Math.toRadians(tkr_atts[ii][0]));
      ea.put(EulerA.ELEV, Math.toRadians(tkr_atts[ii][1]));
      ea.put(EulerA.HEAD, Math.toRadians(tkr_atts[ii][2]));
      ea.toQuatFrameRot(q_b2s);
      trackers[ii].setOrientation(q_b2s);
    }
    vehicleSys.setStarTrackers(trackers);

      // Initialize orbiter attitude control system - leave it OFF
    double[][] kv_array = { { 2.0,  0.0,  0.0 },
                            { 0.0,  9.0,  0.0 },
                            { 0.0,  0.0, 10.0 }};
    double[][] kp_array = { { 1.0,   0.0,  0.0 },
                            { 0.0,   2.0,  0.0 },
                            { 0.0,   0.0,  3.0 }};
    Matrix3X3 kv = new Matrix3X3(kv_array);
    Matrix3X3 kp = new Matrix3X3(kp_array);
    Tuple3D axes = new Tuple3D(1.0, 2.0, 3.0);
    AttitudeControlDCM attCnt = new AttitudeControlDCM(kv, kp, axes);
    vehicleSys.setAttitudeControl(attCnt);

    vehicleSys.setT(modelTimer.getSimulationTime());
    vehicleSys.enableGravity(rcb);
    vehicleSys.setReferencePoint(rps);
    
      // Initialize state (position, velocity, attitude)                                       
    for (XdX6DQ x : XdX6DQ.values()) {
      vehicleSys.setX(x, 0.0);
    }
      // attitude: bank = elevation = heading = 0 --> q0 = 1, q_vec = (0,0,0)
    vehicleSys.setX(XdX6DQ.Q0, 1.0);
    
      // Keplerian element set with true anomaly as the fast variable
      // Note that call to aeGivenPeriApo sets semi-major and eccentricity
      // (orbit size and shape) after this initialization.
    KeplerianOE koe = new KeplerianOE(geo.getGravParam(), (1.0/duScale),
                                                           0.0,
                                          Math.toRadians(60.0),
                                          Math.toRadians(30.0),
                                          Math.toRadians(40.0),
                                                          0.0);
      // Set semi-major axis and eccentricity with periapsis and apoapsis
    koe.aeGivenPeriApo((5.0/duScale), (7.5/duScale));
      // Convert to Cartesian for state vector
    Tuple3D pos0 = new Tuple3D();
    Tuple3D vel0 = new Tuple3D();
    koe.getRV(pos0, vel0);
    vehicleSys.setX(XdX6DQ.X, pos0.get(Basis3D.I));
    vehicleSys.setX(XdX6DQ.Y, pos0.get(Basis3D.J));
    vehicleSys.setX(XdX6DQ.Z, pos0.get(Basis3D.K));
    vehicleSys.setX(XdX6DQ.DX, vel0.get(Basis3D.I));
    vehicleSys.setX(XdX6DQ.DY, vel0.get(Basis3D.J));
    vehicleSys.setX(XdX6DQ.DZ, vel0.get(Basis3D.K));
      // Done with init - compute outputs to init them
      // so they are valid at epoch for external objects
      // dependent on them.
    vehicleSys.computeOutputs();

    /*
     * --- Create Visualization components and link to math models. ---
     * 
     * An IVisualModel will need to be part of a BranchGroup to
     * be displayed, but a BranchGroup does not necessarily need
     * to include an IVisualModel (static in nature, or not
     * tied to the Java3D refresh rate).
     */
   
      // Array of visualization models - they handle updating
      // of the models.
    IVisualModel visModels[] = new IVisualModel[NUMMODELS];
    
      // Central body solid model
    Color3f emissive = new Color3f(0.0f, 0.0f, 0.0f);
    Color3f amb_diff = new Color3f(0.8f, 0.3f, 0.3f);
    Color3f specular = new Color3f(0.9f, 0.9f, 0.9f);

      // The TG only needs the central body shape, so just pass the geo object.
    TransformGroup centralBodyTG = null;
    Appearance app = new Appearance();
    if (useEarthModel) {
      URL matURL = Orbiter.class.getResource("earth_lights_lrg.jpg");
      Color3f earth_amb_diff = new Color3f(0.0f, 0.9f, 0.0f);
      centralBodyTG = new TexturedSphereTG(1.0f, 30, earth_amb_diff, matURL);
    }else {
      Material mat = new Material(amb_diff, emissive, amb_diff,
                                                    specular, 125.0f);
      mat.setLightingEnable(true);
      app.setMaterial(mat); 
        // Scale potential function results by r/GM because we want the shape
        // to be relative to a unit length, not true potential values in
        // units of GM/r (that doesn't directly map as a distance on the grid).
      centralBodyTG = new SphericalTG(64, geo, app,
                                     (geo.getRefRadius()/geo.getGravParam()));
    }
    centralBodyTG.addChild(new XyzRgbTG(2.5f));
    visModels[0] = new RotatingBodyVisModel(cfg, centralBodyTG, centralBodySys);

      // Reference point
    URL rpmatURL = Orbiter.class.getResource("foil_gold_256.jpg");
    Color3f rp_amb_diff = new Color3f(0.8f, 0.8f, 0.0f);
    TransformGroup rpTG = new TexturedSphereTG((float) (0.1*solidModelScale),
                                                  16, rp_amb_diff, rpmatURL);
    
    visModels[1] = new ReferencePointVisModel(cfg, rpTG, rps);

      // Orbiter
    TransformGroup modelTG = new SparkySpacecraftMat(solidModelScale);
    visModels[2] = new OrbiterVisModel(cfg, mStepper, modelTG, vehicleSys);

    /*
     * Add models to time loop.
     */
    modelTimer.setModels(visModels);

      // Create and add BranchGroups to the list
    BranchGroup[] bgs = new BranchGroup[NUMMODELS+1];
    bgs[0] = new TimeBGModel(modelTimer);
    bgs[1] = new SimpleBGModel(centralBodyTG, visModels[0], null,
                                                      cfg.getBoundingSphere());
    bgs[2] = new SimpleBGModel(rpTG, visModels[1], null,
                                                      cfg.getBoundingSphere());
    bgs[3] = new SimpleBGModel(modelTG, visModels[2], null,
                                                      cfg.getBoundingSphere());

    OrbiterInputsFrame oif = new OrbiterInputsFrame(geo, centralBodySys,
                                                    vehicleSys, rps, modelTimer);
    oif.setKvKpA(kv, kp, axes);
      // Fire up simulation
    new SimV3D(cfg, bgs);
  }
}
