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
import { Equation } from './Equation';
import { Vec3 } from '../math/Vec3';
var RotationalMotorEquation = (function (_super) {
    __extends(RotationalMotorEquation, _super);
    function RotationalMotorEquation(bodyA, bodyB, maxForce) {
        if (maxForce === void 0) { maxForce = 1e6; }
        var _this = _super.call(this, bodyA, bodyB, maxForce) || this;
        _this.axisA = new Vec3();
        _this.axisB = new Vec3();
        _this.targetVelocity = 0;
        _this.maxForce = maxForce;
        _this.minForce = -maxForce;
        return _this;
    }
    RotationalMotorEquation.prototype.computeB = function (h) {
        var a = this.a, b = this.b, axisA = this.axisA, axisB = this.axisB, GA = this.jacobianElementA, GB = this.jacobianElementB;
        GA.rotational.copy(axisA);
        axisB.negate(GB.rotational);
        var GW = this.computeGW() - this.targetVelocity, GiMf = this.computeGiMf();
        var B = -GW * b - h * GiMf;
        return B;
    };
    return RotationalMotorEquation;
}(Equation));
export { RotationalMotorEquation };
