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
import { Constraint } from './Constraint';
import { ContactEquation } from '../equations/ContactEquation';
var DistanceConstraint = (function (_super) {
    __extends(DistanceConstraint, _super);
    function DistanceConstraint(bodyA, bodyB, distance, maxForce) {
        if (maxForce === void 0) { maxForce = 1e6; }
        var _this = _super.call(this, bodyA, bodyB) || this;
        if (distance == undefined) {
            distance = bodyA.position.distanceTo(bodyB.position);
        }
        _this.distance = distance;
        var eq = _this.distanceEquation = new ContactEquation(bodyA, bodyB);
        _this.equations.push(eq);
        eq.minForce = -maxForce;
        eq.maxForce = maxForce;
        return _this;
    }
    DistanceConstraint.prototype.update = function () {
        var bodyA = this.bodyA;
        var bodyB = this.bodyB;
        var eq = this.distanceEquation;
        var halfDist = this.distance * 0.5;
        var normal = eq.ni;
        bodyB.position.vsub(bodyA.position, normal);
        normal.normalize();
        normal.mult(halfDist, eq.ri);
        normal.mult(-halfDist, eq.rj);
    };
    return DistanceConstraint;
}(Constraint));
export { DistanceConstraint };
