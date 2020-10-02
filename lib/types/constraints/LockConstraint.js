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
import { Vec3, Vec3Consts } from '../math/Vec3';
import { RotationalEquation } from '../equations/RotationalEquation';
var LockConstraint = (function (_super) {
    __extends(LockConstraint, _super);
    function LockConstraint(bodyA, bodyB, options) {
        if (options === void 0) { options = {}; }
        var _this = _super.call(this, bodyA, new Vec3(), bodyB, new Vec3(), (options.maxForce) ? options.maxForce : 1e6) || this;
        _this.LockConstraint_update_tmpVec1 = new Vec3();
        _this.LockConstraint_update_tmpVec2 = new Vec3();
        var halfWay = new Vec3();
        bodyA.position.vadd(bodyB.position, halfWay);
        halfWay.scale(0.5, halfWay);
        bodyB.pointToLocalFrame(halfWay, _this.pivotB);
        bodyA.pointToLocalFrame(halfWay, _this.pivotA);
        _this.xA = bodyA.vectorToLocalFrame(Vec3Consts.UNIT_X);
        _this.xB = bodyB.vectorToLocalFrame(Vec3Consts.UNIT_X);
        _this.yA = bodyA.vectorToLocalFrame(Vec3Consts.UNIT_Y);
        _this.yB = bodyB.vectorToLocalFrame(Vec3Consts.UNIT_Y);
        _this.zA = bodyA.vectorToLocalFrame(Vec3Consts.UNIT_Z);
        _this.zB = bodyB.vectorToLocalFrame(Vec3Consts.UNIT_Z);
        var r1 = _this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, options);
        var r2 = _this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, options);
        var r3 = _this.rotationalEquation3 = new RotationalEquation(bodyA, bodyB, options);
        _this.equations.push(r1, r2, r3);
        return _this;
    }
    LockConstraint.prototype.update = function () {
        var bodyA = this.bodyA, bodyB = this.bodyB, r1 = this.rotationalEquation1, r2 = this.rotationalEquation2, r3 = this.rotationalEquation3, worldAxisA = this.LockConstraint_update_tmpVec1, worldAxisB = this.LockConstraint_update_tmpVec2;
        _super.prototype.update.call(this);
        bodyA.vectorToWorldFrame(this.xA, r1.axisA);
        bodyB.vectorToWorldFrame(this.yB, r1.axisB);
        bodyA.vectorToWorldFrame(this.yA, r2.axisA);
        bodyB.vectorToWorldFrame(this.zB, r2.axisB);
        bodyA.vectorToWorldFrame(this.zA, r3.axisA);
        bodyB.vectorToWorldFrame(this.xB, r3.axisB);
    };
    return LockConstraint;
}(PointToPointConstraint));
export { LockConstraint };
