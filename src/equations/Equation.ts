import { JacobianElement } from '../math/JacobianElement';
import { Vec3 } from '../math/Vec3';
import { Body } from '../objects/Body';
import { Shape } from '../shapes/Shape';

/**
 * Equation base class
 * @class Equation
 * @constructor
 * @author schteppe
 * @param {Body} bi
 * @param {Body} bj
 * @param {Number} minForce Minimum (read: negative max) force to be applied by the constraint.
 * @param {Number} maxForce Maximum (read: positive max) force to be applied by the constraint.
 */
export class Equation {
  static EquationId = 0;
  id = 0;
  minForce: number;
  maxForce: number;

  bi: Body;
  bj: Body;

  si: Shape; // Used by Narrowphase only
  sj: Shape; // Used by Narrowphase only

  a: number;
  b: number;
  eps: number;
  jacobianElementA: JacobianElement;
  jacobianElementB: JacobianElement;
  enabled: boolean;
  multiplier: number;

  constructor(bi: Body, bj: Body, minForce?: number, maxForce?: number) {
    this.id = Equation.EquationId++;

    /**
     * @property {number} minForce
     */
    this.minForce = minForce || -1e6;

    /**
     * @property {number} maxForce
     */
    this.maxForce = maxForce || 1e6;

    /**
     * @property bi
     * @type {Body}
     */
    this.bi = bi;

    /**
     * @property bj
     * @type {Body}
     */
    this.bj = bj;

    /**
     * SPOOK parameter
     * @property {number} a
     */
    this.a = 0.0;

    /**
     * SPOOK parameter
     * @property {number} b
     */
    this.b = 0.0;

    /**
     * SPOOK parameter
     * @property {number} eps
     */
    this.eps = 0.0;

    /**
     * @property {JacobianElement} jacobianElementA
     */
    this.jacobianElementA = new JacobianElement();

    /**
     * @property {JacobianElement} jacobianElementB
     */
    this.jacobianElementB = new JacobianElement();

    /**
     * @property {boolean} enabled
     * @default true
     */
    this.enabled = true;

    /**
     * A number, proportional to the force added to the bodies.
     * @property {number} multiplier
     * @readonly
     */
    this.multiplier = 0;

    // Set typical spook params
    this.setSpookParams(1e7, 4, 1 / 60);
  }

  /**
   * Recalculates a,b,eps.
   * @method setSpookParams
   */
  setSpookParams(stiffness: number, relaxation: number, timeStep: number) {
    const d = relaxation,
      k = stiffness,
      h = timeStep;
    this.a = 4.0 / (h * (1 + 4 * d));
    this.b = (4.0 * d) / (1 + 4 * d);
    this.eps = 4.0 / (h * h * k * (1 + 4 * d));
  }

  /**
   * Computes the RHS of the SPOOK equation
   * @method computeB
   * @return {Number}
   */
  computeB(a: number, b?: number, h?: number): number {
    const GW = this.computeGW(),
      Gq = this.computeGq(),
      GiMf = this.computeGiMf();
    return - Gq * a - GW * b - GiMf * h;
  }

  /**
   * Computes G*q, where q are the generalized body coordinates
   * @method computeGq
   * @return {Number}
   */
  computeGq() {
    const GA = this.jacobianElementA,
      GB = this.jacobianElementB,
      bi = this.bi,
      bj = this.bj,
      xi = bi.position,
      xj = bj.position;
    return GA.spatial.dot(xi) + GB.spatial.dot(xj);
  }

  private zero = new Vec3();

  /**
   * Computes G*W, where W are the body velocities
   * @method computeGW
   * @return {Number}
   */
  computeGW() {
    const GA = this.jacobianElementA,
      GB = this.jacobianElementB,
      bi = this.bi,
      bj = this.bj,
      vi = bi.velocity,
      vj = bj.velocity,
      wi = bi.angularVelocity,
      wj = bj.angularVelocity;
    return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
  }


  /**
   * Computes G*Wlambda, where W are the body velocities
   * @method computeGWlambda
   * @return {Number}
   */
  computeGWlambda() {
    const GA = this.jacobianElementA,
      GB = this.jacobianElementB,
      bi = this.bi,
      bj = this.bj,
      vi = bi.vlambda,
      vj = bj.vlambda,
      wi = bi.wlambda,
      wj = bj.wlambda;
    return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
  }

  private iMfi = new Vec3();
  private iMfj = new Vec3();
  private invIi_vmult_taui = new Vec3();
  private invIj_vmult_tauj = new Vec3();
  /**
   * Computes G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies.
   * @method computeGiMf
   * @return {Number}
   */
  computeGiMf() {
    const GA = this.jacobianElementA,
      GB = this.jacobianElementB,
      bi = this.bi,
      bj = this.bj,
      fi = bi.force,
      ti = bi.torque,
      fj = bj.force,
      tj = bj.torque,
      invMassi = bi.invMassSolve,
      invMassj = bj.invMassSolve;

    fi.scale(invMassi, this.iMfi);
    fj.scale(invMassj, this.iMfj);

    bi.invInertiaWorldSolve.vmult(ti, this.invIi_vmult_taui);
    bj.invInertiaWorldSolve.vmult(tj, this.invIj_vmult_tauj);

    return GA.multiplyVectors(this.iMfi, this.invIi_vmult_taui) + GB.multiplyVectors(this.iMfj, this.invIj_vmult_tauj);
  }

  private tmp = new Vec3();
  /**
   * Computes G*inv(M)*G'
   * @method computeGiMGt
   * @return {Number}
   */
  computeGiMGt(): number {
    const GA = this.jacobianElementA,
      GB = this.jacobianElementB,
      bi = this.bi,
      bj = this.bj,
      invMassi = bi.invMassSolve,
      invMassj = bj.invMassSolve,
      invIi = bi.invInertiaWorldSolve,
      invIj = bj.invInertiaWorldSolve;
    let result = invMassi + invMassj;

    invIi.vmult(GA.rotational, this.tmp);
    result += this.tmp.dot(GA.rotational);

    invIj.vmult(GB.rotational, this.tmp);
    result += this.tmp.dot(GB.rotational);

    return result;
  }

  private addToWlambda_temp = new Vec3();
  // private addToWlambda_Gi = new Vec3();
  // private addToWlambda_Gj = new Vec3();
  // private addToWlambda_ri = new Vec3();
  // private addToWlambda_rj = new Vec3();
  // private addToWlambda_Mdiag = new Vec3();

  /**
   * Add constraint velocity to the bodies.
   * @method addToWlambda
   * @param {Number} deltalambda
   */
  addToWlambda(deltalambda: number) {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const temp = this.addToWlambda_temp;

    // Add to linear velocity
    // v_lambda += inv(M) * delta_lamba * G
    bi.vlambda.addScaledVector(bi.invMassSolve * deltalambda, GA.spatial, bi.vlambda);
    bj.vlambda.addScaledVector(bj.invMassSolve * deltalambda, GB.spatial, bj.vlambda);

    // Add to angular velocity
    bi.invInertiaWorldSolve.vmult(GA.rotational, temp);
    bi.wlambda.addScaledVector(deltalambda, temp, bi.wlambda);

    bj.invInertiaWorldSolve.vmult(GB.rotational, temp);
    bj.wlambda.addScaledVector(deltalambda, temp, bj.wlambda);
  }

  /**
   * Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
   * @method computeInvC
   * @param  {Number} eps
   * @return {Number}
   */
  computeC() {
    return this.computeGiMGt() + this.eps;
  }
}
