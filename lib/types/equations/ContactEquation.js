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
var ContactEquation = (function (_super) {
    __extends(ContactEquation, _super);
    function ContactEquation(bodyA, bodyB, maxForce) {
        if (maxForce === void 0) { maxForce = 1e6; }
        var _this = _super.call(this, bodyA, bodyB, 0, maxForce) || this;
        _this.ContactEquation_computeB_temp1 = new Vec3();
        _this.ContactEquation_computeB_temp2 = new Vec3();
        _this.ContactEquation_computeB_temp3 = new Vec3();
        _this.ContactEquation_getImpactVelocityAlongNormal_vi = new Vec3();
        _this.ContactEquation_getImpactVelocityAlongNormal_vj = new Vec3();
        _this.ContactEquation_getImpactVelocityAlongNormal_xi = new Vec3();
        _this.ContactEquation_getImpactVelocityAlongNormal_xj = new Vec3();
        _this.ContactEquation_getImpactVelocityAlongNormal_relVel = new Vec3();
        _this.restitution = 0.0;
        _this.ri = new Vec3();
        _this.rj = new Vec3();
        _this.ni = new Vec3();
        return _this;
    }
    ContactEquation.prototype.computeB = function (h) {
        var a = this.a, b = this.b, bi = this.bi, bj = this.bj, ri = this.ri, rj = this.rj, rixn = this.ContactEquation_computeB_temp1, rjxn = this.ContactEquation_computeB_temp2, vi = bi.velocity, wi = bi.angularVelocity, fi = bi.force, taui = bi.torque, vj = bj.velocity, wj = bj.angularVelocity, fj = bj.force, tauj = bj.torque, penetrationVec = this.ContactEquation_computeB_temp3, GA = this.jacobianElementA, GB = this.jacobianElementB, n = this.ni;
        ri.cross(n, rixn);
        rj.cross(n, rjxn);
        n.negate(GA.spatial);
        rixn.negate(GA.rotational);
        GB.spatial.copy(n);
        GB.rotational.copy(rjxn);
        penetrationVec.copy(bj.position);
        penetrationVec.vadd(rj, penetrationVec);
        penetrationVec.vsub(bi.position, penetrationVec);
        penetrationVec.vsub(ri, penetrationVec);
        var g = n.dot(penetrationVec);
        var ePlusOne = this.restitution + 1;
        var GW = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
        var GiMf = this.computeGiMf();
        var B = -g * a - GW * b - h * GiMf;
        return B;
    };
    ContactEquation.prototype.getImpactVelocityAlongNormal = function () {
        var vi = this.ContactEquation_getImpactVelocityAlongNormal_vi;
        var vj = this.ContactEquation_getImpactVelocityAlongNormal_vj;
        var xi = this.ContactEquation_getImpactVelocityAlongNormal_xi;
        var xj = this.ContactEquation_getImpactVelocityAlongNormal_xj;
        var relVel = this.ContactEquation_getImpactVelocityAlongNormal_relVel;
        this.bi.position.vadd(this.ri, xi);
        this.bj.position.vadd(this.rj, xj);
        this.bi.getVelocityAtWorldPoint(xi, vi);
        this.bj.getVelocityAtWorldPoint(xj, vj);
        vi.vsub(vj, relVel);
        return this.ni.dot(relVel);
    };
    return ContactEquation;
}(Equation));
export { ContactEquation };
