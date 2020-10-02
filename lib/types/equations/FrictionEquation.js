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
import { Vec3 } from '../math/Vec3';
import { ContactEquation } from './ContactEquation';
var FrictionEquation = (function (_super) {
    __extends(FrictionEquation, _super);
    function FrictionEquation(bodyA, bodyB, slipForce) {
        var _this = _super.call(this, bodyA, bodyB, slipForce) || this;
        _this.FrictionEquation_computeB_temp1 = new Vec3();
        _this.FrictionEquation_computeB_temp2 = new Vec3();
        _this.ri = new Vec3();
        _this.rj = new Vec3();
        _this.t = new Vec3();
        return _this;
    }
    FrictionEquation.prototype.computeB = function (h) {
        var a = this.a, b = this.b, bi = this.bi, bj = this.bj, ri = this.ri, rj = this.rj, rixt = this.FrictionEquation_computeB_temp1, rjxt = this.FrictionEquation_computeB_temp2, t = this.t;
        ri.cross(t, rixt);
        rj.cross(t, rjxt);
        var GA = this.jacobianElementA, GB = this.jacobianElementB;
        t.negate(GA.spatial);
        rixt.negate(GA.rotational);
        GB.spatial.copy(t);
        GB.rotational.copy(rjxt);
        var GW = this.computeGW();
        var GiMf = this.computeGiMf();
        var B = -GW * b - h * GiMf;
        return B;
    };
    return FrictionEquation;
}(ContactEquation));
export { FrictionEquation };
