/*
 c  OrbiterSys.java
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

import com.motekew.vse.c0ntm.AttitudeControlDCM;
import com.motekew.vse.enums.*;
import com.motekew.vse.envrm.*;
import com.motekew.vse.math.Matrix3X3;
import com.motekew.vse.math.Quaternion;
import com.motekew.vse.math.Tuple3D;
import com.motekew.vse.servm.HandleObserverNotificationUtil;
import com.motekew.vse.servm.IHandleObservable;
import com.motekew.vse.servm.IHandleObserver;
import com.motekew.vse.trmtm.*;

/**
 * This is a model of a simple spacecraft subject to a single environmental
 * influence (the gravitational attraction of a single mass) and 4 control
 * inputs.  One of the controls adjusts thrust along the spacecraft's
 * longitudinal axis, while the three remaining controls impart torques
 * about each axis.  The method by which these forces/torques are created
 * is currently not being modeled.
 * <P>
 * This class also computes outputs that are derived from the state vector.
 * These outputs are not directly solved for by the EOM via integration but
 * may be representative of the model's state and are useful in a control
 * system.  For this simulation, Keplerian orbital elements are computed
 * if a central body has been initialized.  Euler angles are also computed.
 * See <code>OrbiterSys.computeOutputs</code> for more details.
 * <P>
 * A control system with four modes has been implemented.  The first, labeled
 * "Stability Control", eases manual changes in the spacecraft attitude by
 * automatically dampening out angular velocity.  The second is a rest-to-rest
 * mode that will set the spacecraft to a desired attitude relative to
 * "inertial" space.  The third is a dynamic system that attempts to maintain
 * a desired orbit relative to an orbit RPY reference frame.  The control
 * law is completely reactive at this point, so the actual attitude will not
 * exactly match the desired one (upgrades on the way...).  A forth one makes
 * use of a reference point in defining the reference frame from which attitude
 * is measured (see code for more info on this one).
 * <P>
 * The body reference frame has the positive X-axis along the vehicle
 * longitudinal axis (forward = +X).  The body Z-axis is up relative
 * to the vehicle (opposite to what is generally used for aircraft
 * simulations) with the Y-axis finishing the right handed system to
 * the left of the vehicle.  All torques and rotations are positive in
 * a right hand sense (nose down, right wing down, yaw left, are all
 * positive).
 * 
 * @author  Kurt Motekew
 * @since   20081225
 */
public class OrbiterSys extends Simple6DOFSys implements IHandleObservable {

    //Controls (force and torque components) acting on the system.
  private ControlFTxyz ft = new ControlFTxyz();
  private Tuple3D attCntTorque = new Tuple3D();
  private AttitudeControlDCM attCntLaw = null;
  private OrbiterAttOpts attOpt = OrbiterAttOpts.CNT_OFF;
  private RefPntAttOpts rpAttOpt = RefPntAttOpts.RPATT_YAXIS;
  private boolean noAttCntTorques = false;
    // Orbital element and Euler angle outputs
  private StateKepEuler koe_ea = new StateKepEuler();
    // Desired attitude relative to inertial, RPY, or Ref Point, depending
    // on enabled control system.
  private EulerAngles desiredAttitude = new EulerAngles();
    // Latest Intermediate to Body attitude transformation
    // Identity if desiredAttitude values are all zero
  private Quaternion qT2B = new Quaternion(1.0, 0.0, 0.0, 0.0);
    // Converter - Computes Inertial to Reference Point DCM
  private AttitudeRefPoint refPntAttUtil = new AttitudeRefPoint();
    // Central body creating gravity potential
  private ICentralBody cb = null;
    // Reference point for attitude control option
  private IPosition refPnt = null;

    // Used to keep track of external interfaces interested in knowing
    // when to pull current state from this object.  Stimulated at the
    // same rate as the above outputs.
  private HandleObserverNotificationUtil observers = 
                                         new HandleObserverNotificationUtil();

  /** 
   * Initialize this system - activate the control and output Tuples.
   */
  public OrbiterSys() {
    super();
    super.enableControls(ft);
    super.enableOutputs(koe_ea);
  }
  
  /**
   * Initialize gravity.
   * 
   * @param   geo_in  Gravitational model to use, not copied.
   */
  public void enableGravity(ICentralBody cb_in) {
    cb = cb_in;
  }

  /**
   * Disable gravity.
   */
  public void disableGravity() {
    cb = null;
  }

  /**
   * Set reference point used for one of the attitude control options.
   * 
   * @param   rp      Object to point to for reference point information
   *                  (not copied).
   */
  public void setReferencePoint(IPosition rp) {
    refPnt = rp;
  }
  
  public void setRefPointAttOpt(RefPntAttOpts opt) {
    rpAttOpt = opt;
  }

  /**
   * @param   oscOE   New osculating orbital elements to replace
   *                  of this models pos/vel state.  Units must
   *                  be compatible with those of the central body
   *                  gravitational and scaling parameters as they
   *                  are in the central body model.  Nothing happens
   *                  if the central body has not been initialized.
   */
  public void setOE(KeplerianOE oscOE) {
    if (cb != null) {
      Tuple3D pos = new Tuple3D();
      Tuple3D vel = new Tuple3D();
      oscOE.getRV(pos, vel);
      setX(XdX6DQ.X, pos.get(Basis3D.I));
      setX(XdX6DQ.Y, pos.get(Basis3D.J));
      setX(XdX6DQ.Z, pos.get(Basis3D.K));
      setX(XdX6DQ.DX, vel.get(Basis3D.I));
      setX(XdX6DQ.DY, vel.get(Basis3D.J));
      setX(XdX6DQ.DZ, vel.get(Basis3D.K));
    }
  }

  /**
   * Sets the internal attitude control class.
   *
   * @param    at     Attitude control object to use.  Will
   *                  add a torque component about appropriate
   *                  axes to the user input controls.
   *                  Compatibility of the control system with
   *                  user inputs is up to the designer.  The
   *                  pointer is set - values are not copied.
   */
  public void setAttitudeControl(AttitudeControlDCM at) {
    attCntLaw = at;
  }

  /**
   * If the attitude control law has been enabled, this method sets the
   * option to be used.  If an option is selected requiring the central
   * body, then this option will only be activated if the central body
   * has been initialized.
   * 
   * @param   ao    The attitude control option to enable.
   */
  public void setAttitudeControlOption(OrbiterAttOpts ao) {
    attOpt = OrbiterAttOpts.CNT_OFF;
    if (attCntLaw != null) {
      if (ao == OrbiterAttOpts.CNT_RPY  ||  ao == OrbiterAttOpts.CNT_RPNT) {
        if (cb != null) {
          attOpt = ao;
        }
      } else {
        attOpt = ao;
      }
    }
  }

  /**
   * Indicates which attitude control option is enabled.
   * 
   * @return        Active attitude control option.
   */
  public OrbiterAttOpts getAttitudeControlOption() {
    return attOpt;
  }

  /**
   * When enabled, attitude control options will still be processed,
   * with corrective torques and deviations from the desired attitude
   * generated.  However, the actual corrective torques will not be
   * applied.  This is useful for seeing how the attitude would deviate
   * from the desired if the orbiter were allowed to "tumble" in space
   * after establishing some form of attitude control.
   *  
   * @param   tf   If true, don't apply attitude control torques
   */
  public void disableAttCntTorques(boolean tf) {
    noAttCntTorques = tf;
  }

  /*
   * Set the desired attitude to be used by the attitude control
   * system.
   *
   * @param   ea   Set desired attitude to these values (copied)
   */
  public void setDesiredAttitude(EulerAngles ea) {
    desiredAttitude.set(ea);
  }

  /*
   * Get the desired attitude to be used by the attitude control
   * system.
   *
   * @param   ea   Copy current desired attitude into this object.
   * @param        Pointer to ea.
   */
  public EulerAngles getDesiredAttitude(EulerAngles ea) {
    ea.set(desiredAttitude);
    return ea;
  }

  /**
   * @param   ea    Yaw, pitch, and roll from desired attitude
   *                relative to control system intermediate frame,
   *                in radians.
   * @return        pointer to ea.
   */
  public EulerAngles getAttitudeDeviations(EulerAngles ea) {
    switch (attOpt) {
      case CNT_I:
      case CNT_RPY:
      case CNT_RPNT:
        ea.fromQuatFrameRot(qT2B);
          // compute delta
        ea.put(EulerA.HEAD, ea.get(EulerA.HEAD) -
               desiredAttitude.get(EulerA.HEAD));
        ea.put(EulerA.ELEV, ea.get(EulerA.ELEV) -
               desiredAttitude.get(EulerA.ELEV));
        ea.put(EulerA.BANK, ea.get(EulerA.BANK) -
               desiredAttitude.get(EulerA.BANK));
        break;
      default:
        ea.zero();
        break;
    }
    return ea;
  }

  /**
   * Compute gravitational acceleration acting on body given position
   * relative to central body.  First updates bForce and bTorque using
   * accessor methods of superclass.  See superclass description for more info.
   * 
   * @param   t         Input:  Time for which to compute influences.
   * @param   x         The state of the system at the entered time.
   * @param   bForce    Output:  Forces acting through vehicle CG
   *                    in body reference frame.
   * @param   bTorque   Output:  Torques acting through vehicle CG
   *                    in body reference frame.
   * @param   iForce    N/A
   * @param  gravityI   Output:  Gravitational acceleration.
   */
  private GravityCache gvC = new GravityCache();
  private AttitudeCache atC = new AttitudeCache();
  @Override
  protected void finishModel(double t, State6DQ x,
                             Tuple3D bForce, Tuple3D bTorque,
                             Tuple3D iForce, Tuple3D gravityI)  {
    
      // Body forces from controls
    bForce.put(Basis3D.I, ft.get(FTxyz.FX));
    bForce.put(Basis3D.J, ft.get(FTxyz.FY));
    bForce.put(Basis3D.K, ft.get(FTxyz.FZ));

      // Compute attitude control component if enabled.  If not, zeros
      // attitude control torques.
    switch (attOpt) {
      case CNT_STAB:
          // Get velocity and find torque to counteract it
        atC.omega.set(x.get(XdX6DQ.P), x.get(XdX6DQ.Q), x.get(XdX6DQ.R));
        attCntLaw.getU(atC.omega, attCntTorque);
        qT2B.identity();
          // Set desired attitude to current since there is no desired att...
        break;
      case CNT_I:
        getAttitude(t, qT2B);
        desiredAttitude.toQuatFrameRot(atC.qd);
          // inertial is intermediate frame in this case
        atC.omega.set(x.get(XdX6DQ.P), x.get(XdX6DQ.Q), x.get(XdX6DQ.R));
        attCntLaw.getU(qT2B, atC.qd, atC.omega, attCntTorque);
        break;
      case CNT_RPY:
        getPosition(t, atC.posI);
        getVelocity(t, atC.velI);
        getAttitude(t, atC.q);
        atC.omega.set(x.get(XdX6DQ.P), x.get(XdX6DQ.Q), x.get(XdX6DQ.R));
        desiredAttitude.toDCM(atC.dcmIntermediate2Body);
        AttitudeRPY.posVel2DCM(atC.posI, atC.velI, atC.dcmI2Intermediate);
        atC.dcmd.mult(atC.dcmIntermediate2Body, atC.dcmI2Intermediate);
        attCntLaw.getU(atC.q, atC.dcmd, atC.omega, attCntTorque);
          // Now compute Intermediate to Body (T2B)- reuse space
        atC.dcmI2Intermediate.transpose();    // Intermediate2I (T2I)
        atC.qd.set(atC.dcmI2Intermediate);
        qT2B.mult(atC.qd, atC.q);
        break;
      case CNT_RPNT:
        boolean good = false;
        if (refPnt != null) {
          refPnt.getPosition(t, atC.rpPosI);
          atC.bs.minus(atC.posI, atC.rpPosI);
          atC.bs.minus(atC.posI);
          if (atC.bs.mag() > 100.0*Double.MIN_VALUE) {
            good = true;
          }
        }
        if (good) {
          getPosition(t, atC.posI);
          getAttitude(t, atC.q);
          if (rpAttOpt == RefPntAttOpts.RPATT_ZEROYAW) {  // Zero yaw targeting
              // Get RPY frame first
            getVelocity(t, atC.velI);
            AttitudeRPY.posVel2DCM(atC.posI, atC.velI, atC.dcmI2Intermediate); 
            Quaternion q_i2rpy = new Quaternion();
            q_i2rpy.set(atC.dcmI2Intermediate);
              // Now the transition from RPY to targeted
            atC.bs.minus(atC.rpPosI, atC.posI);
            atC.posI.mult(-1.0);
            Tuple3D nvec = new Tuple3D();
            nvec.cross(atC.posI, atC.bs);
            double alpha = Math.atan2(nvec.mag(), atC.posI.dot(atC.bs));
            nvec.unitize();
            Quaternion q_rpy2t = new Quaternion();
            q_rpy2t.set(alpha, nvec);
              // Bring the two together
            Quaternion q_i2t = new Quaternion();
            q_i2t.mult(q_i2rpy, q_rpy2t);
            Quaternion q_t2b = new Quaternion();
              // Apply Euler angles
            desiredAttitude.toQuatFrameRot(q_t2b);
            atC.qd.mult(q_i2t, q_t2b);
              // And apply control law
            atC.omega.set(x.get(XdX6DQ.P), x.get(XdX6DQ.Q), x.get(XdX6DQ.R));
            attCntLaw.getU(atC.q, atC.qd, atC.omega, attCntTorque);
          } else {
            refPntAttUtil.posPnt2DCM(atC.posI,
                                     atC.rpPosI, atC.dcmI2Intermediate);
            desiredAttitude.toDCM(atC.dcmIntermediate2Body);
            atC.dcmd.mult(atC.dcmIntermediate2Body, atC.dcmI2Intermediate);
              // Call control law
            atC.omega.set(x.get(XdX6DQ.P), x.get(XdX6DQ.Q), x.get(XdX6DQ.R));
            attCntLaw.getU(atC.q, atC.dcmd, atC.omega, attCntTorque);
          }
            // Now compute Intermediate to Body (T2B)- reuse space
          atC.dcmI2Intermediate.transpose();    // Intermediate2I (T2I)
          atC.qd.set(atC.dcmI2Intermediate);
          qT2B.mult(atC.qd, atC.q);
        } else {
          attCntTorque.zero();
          qT2B.identity();
        }
        break;
      default:
        attCntTorque.zero();
        qT2B.identity();
        break;
    }

    if (noAttCntTorques) {
        // Total Body Torques:  user inputs only
      bTorque.put(Basis3D.I, ft.get(FTxyz.TX));
      bTorque.put(Basis3D.J, ft.get(FTxyz.TY));
      bTorque.put(Basis3D.K, ft.get(FTxyz.TZ));
    } else {
        // Total Body Torques:  user inputs + attitude control
      bTorque.put(Basis3D.I, ft.get(FTxyz.TX) + attCntTorque.get(Basis3D.I));
      bTorque.put(Basis3D.J, ft.get(FTxyz.TY) + attCntTorque.get(Basis3D.J));
      bTorque.put(Basis3D.K, ft.get(FTxyz.TZ) + attCntTorque.get(Basis3D.K));
    }

    /*
     * Update gravity vector - gravitational acceleration.
     */
    if (cb != null) {
      gravityI.put(Basis3D.I, x.get(XdX6DQ.X));
      gravityI.put(Basis3D.J, x.get(XdX6DQ.Y));
      gravityI.put(Basis3D.K, x.get(XdX6DQ.Z));
        // Do subtractions here to get position relative to
        // CB.  Then get attitude to determine CB relative
        // position.
      x.getPosition(t, gvC.r);               // model pos in I
      cb.getPosition(t, gvC.r_cb);           // Central body pos in I
      gvC.r.minus(gvC.r_cb);                 // model relative to CB in I
      cb.getAttitude(t, gvC.cb_i2b);         //Attitude of central body
      gvC.r_cb.fRot(gvC.cb_i2b, gvC.r);      // Now the pos of model in CB
      cb.gravt(gvC.r_cb, gvC.gravityCB);
      gravityI.vRot(gvC.cb_i2b, gvC.gravityCB);
    } else {
      gravityI.zero();
    }
  }

  // finishModel() Cache - gravity related
  private class GravityCache {
    Tuple3D r = new Tuple3D();
    Tuple3D r_cb = new Tuple3D();
    Tuple3D gravityCB = new Tuple3D();
    Quaternion cb_i2b = new Quaternion();
  }
  // finishModel() Cache - attitude control related
  private class AttitudeCache {
    Tuple3D posI = new Tuple3D();                        // Current Position
    Tuple3D velI = new Tuple3D();                        // Current Velocity
    Quaternion q = new Quaternion();                     // Current Attitude
    Quaternion qd = new Quaternion();                    // Desired Attitude
    Matrix3X3 dcmI2Intermediate = new Matrix3X3();       // I to tmp
    Matrix3X3 dcmIntermediate2Body = new Matrix3X3();    // tmp to body
    Matrix3X3 dcmd = new Matrix3X3();                    // Desired Attitude
    Tuple3D omega = new Tuple3D();                       // Angular velocity
    Tuple3D rpPosI = new Tuple3D();                      // Ref Pnt I Position
    Tuple3D bs     = new Tuple3D();                      // posI to rpPosI
    AttitudeCache() {
      dcmd.identity();
    }
  }

  /**
   * If a central body has been initialized, this method
   * computes Euler angles relative to the Roll, Pitch, Yaw
   * reference frame.  See <code>EulerAngles.i2rpy()</code>.
   * These is essentially the attitude of the object in an
   * instantaneous local level (relative to a sphere) reference
   * frame, relative to a central body.  These Euler angles are
   * computed no matter which control system is in place (for
   * example, if the reference point based attitude control method
   * is in place, then raw outputs in this method are still computed
   * as Euler angles relative to the RPY reference frame).  This
   * method also uses orbital elements to compute the RPY reference
   * frame attitude, where as the control system creates a DCM by
   * forming a basis with inertial position and velocity vectors.
   * The different methods are used for now primarily because of
   * a desire to com.motekew.vse.test the two for consistency.
   * <P>
   * If gravity has not been enabled, then bank, elevation,
   * and heading are computed relative to the inertial (computational)
   * reference frame.
   * <P>
   * Orbital elements are computed from the current position and velocity.
   * If the central body hasn't been initialized, then the orbital elements
   * are set to zero and the semi-major axis term is set to the position
   * vector magnitude.
   */
  @Override
  protected void computeOutputs() {
    double time = this.getT();

    if (cb != null) {
        // Find Roll, Pitch, Yaw reference frame given inertial
        // position and velocity.  Need orbital elements anyway,
        // so derive local level based on this instead of forming
        // basis with position & velocity vectors.
      this.getPosition(time, qeC.posI);
      this.getVelocity(time, qeC.velI);
      qeC.koe.set(cb.getGravParam(), cb.getRefRadius(), qeC.posI, qeC.velI);
      AttitudeRPY.orbitalElem2Quat(qeC.koe, qeC.rpy2i); // Returns I to RPY.
        // Invert to get RPY to inertial
      qeC.rpy2i.conj();             // Invert to get RPY to inertial

        // Now get the transformation from inertial to body
      this.getAttitude(time, qeC.i2b);

        // The transformation from the RPY reference frame is then:
      qeC.rpy2b.mult(qeC.rpy2i, qeC.i2b);
    
        // Convert RPY relative attitude to roll, pitch, and yaw
      qeC.eaAtt.fromQuatFrameRot(qeC.rpy2b);
    } else {
      qeC.koe.zero();
      this.getPosition(time, qeC.posI);
      qeC.koe.put(Keplerian.A, qeC.posI.mag());
      qeC.eaAtt.fromQuatFrameRot(getX(XdX6DQ.Q0), getX(XdX6DQ.QI),
                                 getX(XdX6DQ.QJ), getX(XdX6DQ.QK));
    }
    koe_ea.set(qeC.koe, qeC.eaAtt);
  }

  // computeOutputs Cache - quaternion 2 Euler
  private class QuatEulerCache {
    Tuple3D posI = new Tuple3D();
    Tuple3D velI = new Tuple3D();
    Quaternion i2b = new Quaternion();
    Quaternion rpy2b = new Quaternion();
    EulerAngles eaAtt = new EulerAngles();
    KeplerianOE koe = new KeplerianOE();
    Quaternion rpy2i = new Quaternion();
  }
  private QuatEulerCache qeC = new QuatEulerCache();

  /**
   * @param   attU   Torques imparted by the attitude control
   *                 system (output).
   * @return         Pointer to attU
   */
   public Tuple3D getAttU(Tuple3D attU) {
     attU.set(attCntTorque);
     return attU;
   }

  /**
   * Returns the control value acting on the StateControl6DQ.  The
   * forces act in the x, y, and z axes directions.  The torques  
   * act in a right hand sense about the x, y, and z axes.  Torque
   * values do not include those imparted by the attitude control
   * system.
   *
   * @param  ndx   A FTxyz enum representing which force/torque to return.
   *
   * @return       A double forcing function value
   */
  public double getU(FTxyz ndx) {
    return ft.get(ndx);
  }

  /**
   * Sets the control value acting on the StateControl6DQ.  The
   * forces act in the x, y, and z axes directions.  The torques
   * act in a right hand sense about the x, y, and z axes.  Does
   * not affect torques inparted by the attitude control system.
   *
   * @param  ndx    A FTxyz enum representing which force to return.
   * @param  ftVal  A double to be used as the new forcing fucntion
   *                value.
   */
  public void setU(FTxyz ndx, double ftVal) {
    ft.put(ndx, ftVal);
  }

  /**
   *  Gets outputs that are a function of the state/EOM, but not directly
   *  solved for.  See <code>OrbiterSys.computeOutputs</code> for the
   *  details.
   *  
   *  @param   i      index to the output value.
   *
   *  @return         Output double value of the requested output value
   */
  public double getY(KepEuler i) {
    return koe_ea.get(i);
  }

  /**
   * Add objects interested in keeping track of the state associated
   * with this object.
   * 
   * @param   observer   An object observing this object.
   * @param   handle     Handle by which this object is to be known
   *                     by the <code>IHandleObserver</code> object.
   */
  @Override
  public void registerObserver(IHandleObserver observer, int handle) {
    observers.addObserverObserved(observer, handle);
  }

  /**
   * Tell external interfaces to pull state if interested.
   */
  @Override
  public void notifyObservers() {
    observers.notifyObservers();
  }

}
