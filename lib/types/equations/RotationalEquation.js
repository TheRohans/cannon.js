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
var RotationalEquation = (function (_super) {
    __extends(RotationalEquation, _super);
    function RotationalEquation(bodyA, bodyB, options) {
        if (options === void 0) { options = {}; }
        var _this = _super.call(this, bodyA, bodyB, -((options.maxForce) ? options.maxForce : 1e6), ((options.maxForce) ? options.maxForce : 1e6)) || this;
        _this.tmpVec1 = new Vec3();
        _this.tmpVec2 = new Vec3();
        _this.axisA = options.axisA ? options.axisA.clone() : new Vec3(1, 0, 0);
        _this.axisB = options.axisB ? options.axisB.clone() : new Vec3(0, 1, 0);
        _this.maxAngle = Math.PI / 2;
        return _this;
    }
    RotationalEquation.prototype.computeB = function (h) {
        var a = this.a, b = this.b, ni = this.axisA, nj = this.axisB, nixnj = this.tmpVec1, njxni = this.tmpVec2, GA = this.jacobianElementA, GB = this.jacobianElementB;
        ni.cross(nj, nixnj);
        nj.cross(ni, njxni);
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);
        var g = Math.cos(this.maxAngle) - ni.dot(nj), GW = this.computeGW(), GiMf = this.computeGiMf();
        var B = -g * a - GW * b - h * GiMf;
        return B;
    };
    return RotationalEquation;
}(Equation));
export { RotationalEquation };
