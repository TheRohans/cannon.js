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
import { ContactEquation } from '../equations/ContactEquation';
import { Constraint } from './Constraint';
import { Vec3 } from '../math/Vec3';
var PointToPointConstraint = (function (_super) {
    __extends(PointToPointConstraint, _super);
    function PointToPointConstraint(bodyA, pivotA, bodyB, pivotB, maxForce) {
        if (maxForce === void 0) { maxForce = 1e6; }
        var _this = _super.call(this, bodyA, bodyB) || this;
        _this.pivotA = pivotA ? pivotA.clone() : new Vec3();
        _this.pivotB = pivotB ? pivotB.clone() : new Vec3();
        var x = _this.equationX = new ContactEquation(bodyA, bodyB);
        var y = _this.equationY = new ContactEquation(bodyA, bodyB);
        var z = _this.equationZ = new ContactEquation(bodyA, bodyB);
        _this.equations.push(x, y, z);
        x.minForce = y.minForce = z.minForce = -maxForce;
        x.maxForce = y.maxForce = z.maxForce = maxForce;
        x.ni.set(1, 0, 0);
        y.ni.set(0, 1, 0);
        z.ni.set(0, 0, 1);
        return _this;
    }
    PointToPointConstraint.prototype.update = function () {
        var bodyA = this.bodyA;
        var bodyB = this.bodyB;
        var x = this.equationX;
        var y = this.equationY;
        var z = this.equationZ;
        bodyA.quaternion.vmult(this.pivotA, x.ri);
        bodyB.quaternion.vmult(this.pivotB, x.rj);
        y.ri.copy(x.ri);
        y.rj.copy(x.rj);
        z.ri.copy(x.ri);
        z.rj.copy(x.rj);
    };
    return PointToPointConstraint;
}(Constraint));
export { PointToPointConstraint };
