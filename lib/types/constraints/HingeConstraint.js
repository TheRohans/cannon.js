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
import { PointToPointConstraint } from './PointToPointConstraint';
import { Vec3 } from '../math/Vec3';
import { RotationalEquation } from '../equations/RotationalEquation';
import { RotationalMotorEquation } from '../equations/RotationalMotorEquation';
var HingeConstraint = (function (_super) {
    __extends(HingeConstraint, _super);
    function HingeConstraint(bodyA, bodyB, options) {
        if (options === void 0) { options = {}; }
        var _this = _super.call(this, bodyA, (options.pivotA ? options.pivotA.clone() : new Vec3()), bodyB, (options.pivotB ? options.pivotB.clone() : new Vec3()), options.maxForce || 1e6) || this;
        _this.HingeConstraint_update_tmpVec1 = new Vec3();
        _this.HingeConstraint_update_tmpVec2 = new Vec3();
        var maxForce = options.maxForce || 1e6;
        _this.axisA = options.axisA ? options.axisA.clone() : new Vec3(1, 0, 0);
        _this.axisA.normalize();
        _this.axisB = options.axisB ? options.axisB.clone() : new Vec3(1, 0, 0);
        _this.axisB.normalize();
        _this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, options);
        _this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, options);
        _this.motorEquation = new RotationalMotorEquation(bodyA, bodyB, maxForce);
        _this.motorEquation.enabled = false;
        _this.equations.push(_this.rotationalEquation1, _this.rotationalEquation2, _this.motorEquation);
        return _this;
    }
    HingeConstraint.prototype.enableMotor = function () {
        this.motorEquation.enabled = true;
    };
    HingeConstraint.prototype.disableMotor = function () {
        this.motorEquation.enabled = false;
    };
    HingeConstraint.prototype.setMotorSpeed = function (speed) {
        this.motorEquation.targetVelocity = speed;
    };
    HingeConstraint.prototype.setMotorMaxForce = function (maxForce) {
        this.motorEquation.maxForce = maxForce;
        this.motorEquation.minForce = -maxForce;
    };
    HingeConstraint.prototype.update = function () {
        var bodyA = this.bodyA, bodyB = this.bodyB, motor = this.motorEquation, r1 = this.rotationalEquation1, r2 = this.rotationalEquation2, worldAxisA = this.HingeConstraint_update_tmpVec1, worldAxisB = this.HingeConstraint_update_tmpVec2;
        var axisA = this.axisA;
        var axisB = this.axisB;
        _super.prototype.update.call(this);
        bodyA.quaternion.vmult(axisA, worldAxisA);
        bodyB.quaternion.vmult(axisB, worldAxisB);
        worldAxisA.tangents(r1.axisA, r2.axisA);
        r1.axisB.copy(worldAxisB);
        r2.axisB.copy(worldAxisB);
        if (this.motorEquation.enabled) {
            bodyA.quaternion.vmult(this.axisA, motor.axisA);
            bodyB.quaternion.vmult(this.axisB, motor.axisB);
        }
    };
    return HingeConstraint;
}(PointToPointConstraint));
export { HingeConstraint };
