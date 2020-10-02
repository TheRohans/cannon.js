import { Vec3 } from '../math/Vec3';
var Spring = (function () {
    function Spring(bodyA, bodyB, options) {
        if (options === void 0) { options = {}; }
        this.applyForce_r = new Vec3();
        this.applyForce_r_unit = new Vec3();
        this.applyForce_u = new Vec3();
        this.applyForce_f = new Vec3();
        this.applyForce_worldAnchorA = new Vec3();
        this.applyForce_worldAnchorB = new Vec3();
        this.applyForce_ri = new Vec3();
        this.applyForce_rj = new Vec3();
        this.applyForce_ri_x_f = new Vec3();
        this.applyForce_rj_x_f = new Vec3();
        this.applyForce_tmp = new Vec3();
        options = options || {};
        this.restLength = (options.restLength != undefined) ? options.restLength : 1;
        this.stiffness = options.stiffness || 100;
        this.damping = options.damping || 1;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.localAnchorA = new Vec3();
        this.localAnchorB = new Vec3();
        if (options.localAnchorA) {
            this.localAnchorA.copy(options.localAnchorA);
        }
        if (options.localAnchorB) {
            this.localAnchorB.copy(options.localAnchorB);
        }
        if (options.worldAnchorA) {
            this.setWorldAnchorA(options.worldAnchorA);
        }
        if (options.worldAnchorB) {
            this.setWorldAnchorB(options.worldAnchorB);
        }
    }
    Spring.prototype.setWorldAnchorA = function (worldAnchorA) {
        this.bodyA.pointToLocalFrame(worldAnchorA, this.localAnchorA);
    };
    Spring.prototype.setWorldAnchorB = function (worldAnchorB) {
        this.bodyB.pointToLocalFrame(worldAnchorB, this.localAnchorB);
    };
    Spring.prototype.getWorldAnchorA = function (result) {
        this.bodyA.pointToWorldFrame(this.localAnchorA, result);
    };
    Spring.prototype.getWorldAnchorB = function (result) {
        this.bodyB.pointToWorldFrame(this.localAnchorB, result);
    };
    Spring.prototype.applyForce = function () {
        var k = this.stiffness, d = this.damping, l = this.restLength, bodyA = this.bodyA, bodyB = this.bodyB, r = this.applyForce_r, r_unit = this.applyForce_r_unit, u = this.applyForce_u, f = this.applyForce_f, tmp = this.applyForce_tmp;
        var worldAnchorA = this.applyForce_worldAnchorA, worldAnchorB = this.applyForce_worldAnchorB, ri = this.applyForce_ri, rj = this.applyForce_rj, ri_x_f = this.applyForce_ri_x_f, rj_x_f = this.applyForce_rj_x_f;
        this.getWorldAnchorA(worldAnchorA);
        this.getWorldAnchorB(worldAnchorB);
        worldAnchorA.vsub(bodyA.position, ri);
        worldAnchorB.vsub(bodyB.position, rj);
        worldAnchorB.vsub(worldAnchorA, r);
        var rlen = r.norm();
        r_unit.copy(r);
        r_unit.normalize();
        bodyB.velocity.vsub(bodyA.velocity, u);
        bodyB.angularVelocity.cross(rj, tmp);
        u.vadd(tmp, u);
        bodyA.angularVelocity.cross(ri, tmp);
        u.vsub(tmp, u);
        r_unit.mult(-k * (rlen - l) - d * u.dot(r_unit), f);
        bodyA.force.vsub(f, bodyA.force);
        bodyB.force.vadd(f, bodyB.force);
        ri.cross(f, ri_x_f);
        rj.cross(f, rj_x_f);
        bodyA.torque.vsub(ri_x_f, bodyA.torque);
        bodyB.torque.vadd(rj_x_f, bodyB.torque);
    };
    return Spring;
}());
export { Spring };
