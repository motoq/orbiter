/*
 c  OrbiterVisModel.java
 c
 c  Copyright (C) 2011 Kurt Motekew
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

import java.awt.event.*;
import javax.media.j3d.*;
import javax.vecmath.Vector3d;
import javax.vecmath.Quat4d;

import com.motekew.enums.*;
import com.motekew.j3d.*;
import com.motekew.ui.*;
import com.motekew.cycxm.ModelStepper;
import com.motekew.trmtm.*;

/**
 * This class holds both a <code>ISysEqns</code> object and a
 * <code>TransformGroup</code> object.  Since it implements the
 * <code>IVisualModel</code> interface, it guarantees the existence
 * of an update method.
 * <P>
 * This class forms the link between the graphic model display and the
 * algorithms describing the motion of the system being modeled.
 * It makes the jump between dynamics and graphics.
 *
 * @author  Kurt Motekew
 * @since   20081225
 */
public class OrbiterVisModel implements IVisualModel {
    // Java3D config settings
  private SimV3Dcfg cfg;
  private double duScale = 1.0;
  
    // Integration step size control
  private ModelStepper mStepper;
  
    // Deltas for control inputs
  private final double FORCEDELTA  = 0.1;
  private final double TORQUEDELTA = 0.2;

    // Dynamic model
  private OrbiterSys sys;

    // Indexes just to make the code cleaner
  private final XdX6DQ X  = XdX6DQ.X;    // Three position values
  private final XdX6DQ Y  = XdX6DQ.Y;
  private final XdX6DQ Z  = XdX6DQ.Z;
  private final XdX6DQ DX = XdX6DQ.DX;   // Three velocity values
  private final XdX6DQ DY = XdX6DQ.DY;
  private final XdX6DQ DZ = XdX6DQ.DZ;
  private final XdX6DQ Q0 = XdX6DQ.Q0;   // Four attitude quaternion values
  private final XdX6DQ QI = XdX6DQ.QI;
  private final XdX6DQ QJ = XdX6DQ.QJ;
  private final XdX6DQ QK = XdX6DQ.QK;
  private final FTxyz  FX = FTxyz.FX;    // One force - always along body x-axis
  private final FTxyz  TX = FTxyz.TX;    // One torque about each axis
  private final FTxyz  TY = FTxyz.TY;
  private final FTxyz  TZ = FTxyz.TZ;
  
    // Transform group passed in to be adjusted.  This is where position
    // and attitude updates are made.  This transform group contains the
    // solid model being simulated.
  private TransformGroup tg;
  
    // Cache - Java3D objects used to update the TransformGroup.  These
    //         objects are filled making use of the state vector.
  private class TransformCache {
    Transform3D t3d = new Transform3D();  // position and orientation
    Vector3d trans  = new Vector3d();     // Java3D position vector
    Quat4d attitudeQ = new Quat4d();      // Java3D inertial to body
  }
  private TransformCache tfC = new TransformCache();

  /*
   * Used to store state, output, and control values locally for display in
   * a graphic output window.
   */
  private State6DQ      state   = new State6DQ();
  private StateKepEuler output  = new StateKepEuler();
  private ControlFTxyz  control = new ControlFTxyz();
    // Used to display state vector, derivative, and outputs.
  private StateControlOutputViewerFrame scovf;
  private int scovf_skips = 0;

  /**
   * Initialize the link with the <code>TransformGroup</code>
   * used to manipulate the model and the <code>ISysEqns</code>
   * representing the model.
   *
   * @param  cfgin  Java3D environment configuration object.
   * @param  msin   Controls integration step size relative to
   *                output/Java3d refresh rate.
   * @param  tgIn   The TransformGroup to be controlled by
   *                this object.
   * @param  sysIn  The system of equations being visualized.
   */
  public OrbiterVisModel(SimV3Dcfg cfgin, ModelStepper msin,
                         TransformGroup tgIn, OrbiterSys sysIn) {
    cfg = cfgin;
    duScale = cfg.getDU();
    mStepper = msin;
    tg = tgIn;
    sys = sysIn;
    
      // Initialize Visual model state
    tfC.trans.set(sys.getX(X), sys.getX(Y), sys.getX(Z));
    tfC.trans.scale(duScale);
    tfC.attitudeQ.set(sys.getX(QI), sys.getX(QJ), sys.getX(QK), sys.getX(Q0));
      // And update visualization environment settings
    tfC.t3d.set(tfC.attitudeQ, tfC.trans, 1.0);
    tg.setTransform(tfC.t3d);

      // Create display outputs
    scovf = new StateControlOutputViewerFrame("Orbiter", 0.0,
                                              state, control, output);
    updateOutputs();  
  }

  /**
   * Returns the time corresponding to the model's current state.
   * 
   * @return    Model simulation time.
   */
  @Override
  public double getModelTime() {
    return sys.getT();
  }

  /**
   * Requests an update be made.  This request is made externally
   * by the graphic environment.
   *
   * @param delta  A double representing the elapsed time in seconds
   *               since the last update of the Java3D environment.
   *               The state of the model should be updated based on
   *               this time.  This is essentially the output step
   *               size at which to produce a model state vector.
   */
  @Override
  public void update(double delta) {
      // Update system state to new time and get state.  New time will
      // be sys.getT() + delta.
    mStepper.setODt(delta);
    mStepper.stepper(sys);
    
      // Check to see if if time to print outputs
    if (scovf_skips == cfg.getTskip()) {
      scovf_skips = 0;
      updateOutputs();
    } else {
      scovf_skips++;
    }
    
      // update position
    tfC.trans.set(sys.getX(X), sys.getX(Y), sys.getX(Z));
    tfC.trans.scale(duScale);
    
      // Now attitude.
    tfC.attitudeQ.set(sys.getX(QI), sys.getX(QJ), sys.getX(QK), sys.getX(Q0));

      // update rotation and translation
    tfC.t3d.set(tfC.attitudeQ, tfC.trans, 1.0);
    tg.setTransform(tfC.t3d);
  }

  /**
   * Updates the model based on user keyboard input (primarily
   * used to affect model control values).
   *
   * @param  eventKey  A KeyEvent containing the key codes
   *                   representing the key that was pressed.
   */
  @Override
  public void processKeyEvent(KeyEvent eventKey) {
      // force and torque values acting through/about aircraft cg
      // in the body reference frame.
    double force  = 0.0;
    double torque = 0.0;

    int    keyCode = eventKey.getKeyCode();

    switch(keyCode) {
        /* *******  Thrust Control  ******* */
      /*  Add one unit of force acting along +X body (forward) */
      case KeyEvent.VK_UP:
        force = sys.getU(FX) + FORCEDELTA;
        sys.setU(FX, force);
        break;
      /*  Add one unit of force acting along -X (backward) */
      /*  Zero force if less than zero along X body axis (no reverse thrust) */
      case KeyEvent.VK_DOWN:
        force = sys.getU(FX) - FORCEDELTA;
        if (force < 0.0) {                       // zero force
          sys.setU(FX, 0.0);
        } else {                                 // otherwise, apply decrease
          sys.setU(FX, force);
        }
        break;
        /* *******  Attitude  ******* */
      case KeyEvent.VK_S:
        torque = sys.getU(TX);
        torque -= TORQUEDELTA;
        sys.setU(TX, torque);
        break;
      case KeyEvent.VK_F:
        torque = sys.getU(TX);
        torque += TORQUEDELTA;
        sys.setU(TX, torque);
        break;
      case KeyEvent.VK_E:
        torque = sys.getU(TY);
        torque += TORQUEDELTA;
        sys.setU(TY, torque);
        break;
      case KeyEvent.VK_D:
        torque = sys.getU(TY);
        torque -= TORQUEDELTA;
        sys.setU(TY, torque);
        break;
      case KeyEvent.VK_A:
        torque = sys.getU(TZ);
        torque += TORQUEDELTA;
        sys.setU(TZ, torque);
        break;
      case KeyEvent.VK_G:
        torque = sys.getU(TZ);
        torque -= TORQUEDELTA;
        sys.setU(TZ, torque);
        break;
      /* zero rotation rates */
      case KeyEvent.VK_R:
        sys.setX(XdX6DQ.P, 0.0);
        sys.setX(XdX6DQ.Q, 0.0);
        sys.setX(XdX6DQ.R, 0.0);
        discontinuity();
        break;
      /* zero translation rates */
      case KeyEvent.VK_T:
        sys.setX(DX, 0.0);
        sys.setX(DY, 0.0);
        sys.setX(DZ, 0.0);
        discontinuity();
        break;
      /* zero forces and torques */
      case KeyEvent.VK_Q:
        sys.setU(FX, 0.0);
        sys.setU(TX, 0.0);
        sys.setU(TY, 0.0);
        sys.setU(TZ, 0.0);
        break;
       /* *******  Display Info  ******* */
      case KeyEvent.VK_SPACE:
        if (scovf.isVisible()) {
          scovf.setVisible(false);
        } else {
          scovf.pack();
          scovf.setVisible(true);
        }
        break;
      default:
        break;
    }
  }

  /*
   * Updates the local arrays that store state, control, and
   * output vector values.  Used for text output window.
   */
  private void updateOutputs() {
    if (scovf.isVisible()) {
      for (XdX6DQ x : XdX6DQ.values()) {
        state.put(x, sys.getX(x));
      }
      for (FTxyz u : FTxyz.values()) {
        control.put(u, sys.getU(u));
      }
      for (KepEuler y : KepEuler.values()) {
        output.put(y, sys.getY(y));
      }
      scovf.refresh(sys.getT(), state, control, output);
    }
      // Tell external interfaces to pull state if interested.
    sys.notifyObservers();
  }
  
  /*
   * Called whenever the state is abruptly changed external to
   * propagation.  For example, manually setting velocity values
   * to zero.
   */
  private void discontinuity() {
  }
}
