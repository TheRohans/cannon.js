import { JacobianElement } from '../math/JacobianElement';
import { Vec3 } from '../math/Vec3';
var Equation = (function () {
    function Equation(bi, bj, minForce, maxForce) {
        this.id = 0;
        this.zero = new Vec3();
        this.iMfi = new Vec3();
        this.iMfj = new Vec3();
        this.invIi_vmult_taui = new Vec3();
        this.invIj_vmult_tauj = new Vec3();
        this.tmp = new Vec3();
        this.addToWlambda_temp = new Vec3();
        this.id = Equation.EquationId++;
        this.minForce = minForce || -1e6;
        this.maxForce = maxForce || 1e6;
        this.bi = bi;
        this.bj = bj;
        this.a = 0.0;
        this.b = 0.0;
        this.eps = 0.0;
        this.jacobianElementA = new JacobianElement();
        this.jacobianElementB = new JacobianElement();
        this.enabled = true;
        this.multiplier = 0;
        this.setSpookParams(1e7, 4, 1 / 60);
    }
    Equation.prototype.setSpookParams = function (stiffness, relaxation, timeStep) {
        var d = relaxation, k = stiffness, h = timeStep;
        this.a = 4.0 / (h * (1 + 4 * d));
        this.b = (4.0 * d) / (1 + 4 * d);
        this.eps = 4.0 / (h * h * k * (1 + 4 * d));
    };
    Equation.prototype.computeB = function (a, b, h) {
        var GW = this.computeGW(), Gq = this.computeGq(), GiMf = this.computeGiMf();
        return -Gq * a - GW * b - GiMf * h;
    };
    Equation.prototype.computeGq = function () {
        var GA = this.jacobianElementA, GB = this.jacobianElementB, bi = this.bi, bj = this.bj, xi = bi.position, xj = bj.position;
        return GA.spatial.dot(xi) + GB.spatial.dot(xj);
    };
    Equation.prototype.computeGW = function () {
        var GA = this.jacobianElementA, GB = this.jacobianElementB, bi = this.bi, bj = this.bj, vi = bi.velocity, vj = bj.velocity, wi = bi.angularVelocity, wj = bj.angularVelocity;
        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    };
    Equation.prototype.computeGWlambda = function () {
        var GA = this.jacobianElementA, GB = this.jacobianElementB, bi = this.bi, bj = this.bj, vi = bi.vlambda, vj = bj.vlambda, wi = bi.wlambda, wj = bj.wlambda;
        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    };
    Equation.prototype.computeGiMf = function () {
        var GA = this.jacobianElementA, GB = this.jacobianElementB, bi = this.bi, bj = this.bj, fi = bi.force, ti = bi.torque, fj = bj.force, tj = bj.torque, invMassi = bi.invMassSolve, invMassj = bj.invMassSolve;
        fi.scale(invMassi, this.iMfi);
        fj.scale(invMassj, this.iMfj);
        bi.invInertiaWorldSolve.vmult(ti, this.invIi_vmult_taui);
        bj.invInertiaWorldSolve.vmult(tj, this.invIj_vmult_tauj);
        return GA.multiplyVectors(this.iMfi, this.invIi_vmult_taui) + GB.multiplyVectors(this.iMfj, this.invIj_vmult_tauj);
    };
    Equation.prototype.computeGiMGt = function () {
        var GA = this.jacobianElementA, GB = this.jacobianElementB, bi = this.bi, bj = this.bj, invMassi = bi.invMassSolve, invMassj = bj.invMassSolve, invIi = bi.invInertiaWorldSolve, invIj = bj.invInertiaWorldSolve;
        var result = invMassi + invMassj;
        invIi.vmult(GA.rotational, this.tmp);
        result += this.tmp.dot(GA.rotational);
        invIj.vmult(GB.rotational, this.tmp);
        result += this.tmp.dot(GB.rotational);
        return result;
    };
    Equation.prototype.addToWlambda = function (deltalambda) {
        var GA = this.jacobianElementA;
        var GB = this.jacobianElementB;
        var bi = this.bi;
        var bj = this.bj;
        var temp = this.addToWlambda_temp;
        bi.vlambda.addScaledVector(bi.invMassSolve * deltalambda, GA.spatial, bi.vlambda);
        bj.vlambda.addScaledVector(bj.invMassSolve * deltalambda, GB.spatial, bj.vlambda);
        bi.invInertiaWorldSolve.vmult(GA.rotational, temp);
        bi.wlambda.addScaledVector(deltalambda, temp, bi.wlambda);
        bj.invInertiaWorldSolve.vmult(GB.rotational, temp);
        bj.wlambda.addScaledVector(deltalambda, temp, bj.wlambda);
    };
    Equation.prototype.computeC = function () {
        return this.computeGiMGt() + this.eps;
    };
    Equation.EquationId = 0;
    return Equation;
}());
export { Equation };
