var __extends = (this && this.__extends) || (function () {
    var extendStatics = function (d, b) {
        extendStatics = Object.setPrototypeOf ||
            ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
            function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
        return extendStatics(d, b);
    };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
import { Solver } from './Solver';
var GSSolver = (function (_super) {
    __extends(GSSolver, _super);
    function GSSolver(iterations, tolerance) {
        if (iterations === void 0) { iterations = 10; }
        if (tolerance === void 0) { tolerance = 1e-7; }
        var _this = _super.call(this) || this;
        _this.GSSolver_solve_lambda = [];
        _this.GSSolver_solve_invCs = [];
        _this.GSSolver_solve_Bs = [];
        _this.iterations = iterations;
        _this.tolerance = tolerance;
        return _this;
    }
    GSSolver.prototype.solve = function (dt, world) {
        var maxIter = this.iterations, tolSquared = this.tolerance * this.tolerance, equations = this.equations, Neq = equations.length, bodies = world.bodies, Nbodies = bodies.length, h = dt;
        var iter = 0, B, invC, deltalambda, deltalambdaTot, GWlambda, lambdaj;
        if (Neq !== 0) {
            for (var ii = 0; ii !== Nbodies; ii++) {
                bodies[ii].updateSolveMassProperties();
            }
        }
        var invCs = this.GSSolver_solve_invCs, Bs = this.GSSolver_solve_Bs, lambda = this.GSSolver_solve_lambda;
        invCs.length = Neq;
        Bs.length = Neq;
        lambda.length = Neq;
        for (var i = 0; i !== Neq; i++) {
            var c = equations[i];
            lambda[i] = 0.0;
            Bs[i] = c.computeB(h);
            var invs = 1.0 / c.computeC();
            invCs[i] = (isNaN(invs) || !isFinite(invs)) ? 1 : invs;
        }
        if (Neq !== 0) {
            for (var i = 0; i !== Nbodies; i++) {
                var b = bodies[i], vlambda = b.vlambda, wlambda = b.wlambda;
                vlambda.set(0, 0, 0);
                wlambda.set(0, 0, 0);
            }
            for (iter = 0; iter !== maxIter; iter++) {
                deltalambdaTot = 0.0;
                for (var j = 0; j !== Neq; j++) {
                    var c = equations[j];
                    B = Bs[j];
                    invC = invCs[j];
                    lambdaj = lambda[j];
                    GWlambda = c.computeGWlambda();
                    deltalambda = invC * (B - GWlambda - c.eps * lambdaj);
                    if (lambdaj + deltalambda < c.minForce) {
                        deltalambda = c.minForce - lambdaj;
                    }
                    else if (lambdaj + deltalambda > c.maxForce) {
                        deltalambda = c.maxForce - lambdaj;
                    }
                    lambda[j] += deltalambda;
                    deltalambdaTot += deltalambda > 0.0 ? deltalambda : -deltalambda;
                    c.addToWlambda(deltalambda);
                }
                if (deltalambdaTot * deltalambdaTot < tolSquared) {
                    break;
                }
            }
            for (var i = 0; i !== Nbodies; i++) {
                var b = bodies[i], v = b.velocity, w = b.angularVelocity;
                b.vlambda.vmul(b.linearFactor, b.vlambda);
                v.vadd(b.vlambda, v);
                b.wlambda.vmul(b.angularFactor, b.wlambda);
                w.vadd(b.wlambda, w);
            }
            var l = equations.length;
            var invDt = 1 / h;
            invDt = (isNaN(invDt) || !isFinite(invDt)) ? 1 : invDt;
            while (l--) {
                equations[l].multiplier = lambda[l] * invDt;
            }
        }
        return iter;
    };
    return GSSolver;
}(Solver));
export { GSSolver };
