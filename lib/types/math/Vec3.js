import { Mat3 } from './Mat3';
var Vec3 = (function () {
    function Vec3(x, y, z) {
        this.x = x || 0.0;
        this.y = y || 0.0;
        this.z = z || 0.0;
    }
    Vec3.prototype.cross = function (v, target) {
        var vx = v.x, vy = v.y, vz = v.z, x = this.x, y = this.y, z = this.z;
        target = target || new Vec3();
        target.x = (y * vz) - (z * vy);
        target.y = (z * vx) - (x * vz);
        target.z = (x * vy) - (y * vx);
        return target;
    };
    Vec3.prototype.set = function (x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    };
    Vec3.prototype.setZero = function () {
        this.x = this.y = this.z = 0;
    };
    Vec3.prototype.vadd = function (v, target) {
        if (target) {
            target.x = v.x + this.x;
            target.y = v.y + this.y;
            target.z = v.z + this.z;
            return target;
        }
        else {
            return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z);
        }
    };
    Vec3.prototype.vsub = function (v, target) {
        if (target) {
            target.x = this.x - v.x;
            target.y = this.y - v.y;
            target.z = this.z - v.z;
            return target;
        }
        else {
            return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z);
        }
    };
    Vec3.prototype.crossmat = function () {
        return new Mat3([0, -this.z, this.y,
            this.z, 0, -this.x,
            -this.y, this.x, 0]);
    };
    Vec3.prototype.normalize = function () {
        var x = this.x, y = this.y, z = this.z;
        var n = Math.sqrt(x * x + y * y + z * z);
        if (n > 0.0) {
            var invN = 1 / n;
            this.x *= invN;
            this.y *= invN;
            this.z *= invN;
        }
        else {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        }
        return n;
    };
    Vec3.prototype.unit = function (target) {
        target = target || new Vec3();
        var x = this.x, y = this.y, z = this.z;
        var ninv = Math.sqrt(x * x + y * y + z * z);
        if (ninv > 0.0) {
            ninv = 1.0 / ninv;
            target.x = x * ninv;
            target.y = y * ninv;
            target.z = z * ninv;
        }
        else {
            target.x = 1;
            target.y = 0;
            target.z = 0;
        }
        return target;
    };
    Vec3.prototype.norm = function () {
        var x = this.x, y = this.y, z = this.z;
        return Math.sqrt(x * x + y * y + z * z);
    };
    Vec3.prototype.length = function () {
        return this.norm();
    };
    Vec3.prototype.norm2 = function () {
        return this.dot(this);
    };
    Vec3.prototype.lengthSquared = function () {
        return this.norm2();
    };
    Vec3.prototype.distanceTo = function (p) {
        var x = this.x, y = this.y, z = this.z;
        var px = p.x, py = p.y, pz = p.z;
        return Math.sqrt((px - x) * (px - x) +
            (py - y) * (py - y) +
            (pz - z) * (pz - z));
    };
    Vec3.prototype.distanceSquared = function (p) {
        var x = this.x, y = this.y, z = this.z;
        var px = p.x, py = p.y, pz = p.z;
        return (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z);
    };
    Vec3.prototype.mult = function (scalar, target) {
        target = target || new Vec3();
        var x = this.x, y = this.y, z = this.z;
        target.x = scalar * x;
        target.y = scalar * y;
        target.z = scalar * z;
        return target;
    };
    Vec3.prototype.vmul = function (vector, target) {
        target = target || new Vec3();
        target.x = vector.x * this.x;
        target.y = vector.y * this.y;
        target.z = vector.z * this.z;
        return target;
    };
    Vec3.prototype.scale = function (scalar, target) {
        return this.mult(scalar, target);
    };
    Vec3.prototype.addScaledVector = function (scalar, vector, target) {
        target = target || new Vec3();
        target.x = this.x + scalar * vector.x;
        target.y = this.y + scalar * vector.y;
        target.z = this.z + scalar * vector.z;
        return target;
    };
    Vec3.prototype.dot = function (v) {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    };
    Vec3.prototype.isZero = function () {
        return this.x === 0 && this.y === 0 && this.z === 0;
    };
    Vec3.prototype.negate = function (target) {
        target = target || new Vec3();
        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        return target;
    };
    Vec3.prototype.tangents = function (t1, t2) {
        var Vec3_tangents_n = new Vec3();
        var Vec3_tangents_randVec = new Vec3();
        var norm = this.norm();
        if (norm > 0.0) {
            var n = Vec3_tangents_n;
            var inorm = 1 / norm;
            n.set(this.x * inorm, this.y * inorm, this.z * inorm);
            var randVec = Vec3_tangents_randVec;
            if (Math.abs(n.x) < 0.9) {
                randVec.set(1, 0, 0);
                n.cross(randVec, t1);
            }
            else {
                randVec.set(0, 1, 0);
                n.cross(randVec, t1);
            }
            n.cross(t1, t2);
        }
        else {
            t1.set(1, 0, 0);
            t2.set(0, 1, 0);
        }
    };
    Vec3.prototype.toString = function () {
        return this.x + ',' + this.y + ',' + this.z;
    };
    Vec3.prototype.toArray = function () {
        return [this.x, this.y, this.z];
    };
    Vec3.prototype.copy = function (source) {
        this.x = source.x;
        this.y = source.y;
        this.z = source.z;
        return this;
    };
    Vec3.prototype.lerp = function (v, t, target) {
        var x = this.x, y = this.y, z = this.z;
        target.x = x + (v.x - x) * t;
        target.y = y + (v.y - y) * t;
        target.z = z + (v.z - z) * t;
    };
    Vec3.prototype.almostEquals = function (v, precision) {
        if (precision === void 0) { precision = 1e-6; }
        if (Math.abs(this.x - v.x) > precision ||
            Math.abs(this.y - v.y) > precision ||
            Math.abs(this.z - v.z) > precision) {
            return false;
        }
        return true;
    };
    Vec3.prototype.almostZero = function (precision) {
        if (precision === void 0) { precision = 0; }
        if (precision === undefined) {
            precision = 1e-6;
        }
        if (Math.abs(this.x) > precision ||
            Math.abs(this.y) > precision ||
            Math.abs(this.z) > precision) {
            return false;
        }
        return true;
    };
    Vec3.prototype.isAntiparallelTo = function (v, precision) {
        if (precision === void 0) { precision = 0; }
        var antip_neg = new Vec3();
        this.negate(antip_neg);
        return antip_neg.almostEquals(v, precision);
    };
    Vec3.prototype.clone = function () {
        return new Vec3(this.x, this.y, this.z);
    };
    return Vec3;
}());
export { Vec3 };
var Vec3Consts = (function () {
    function Vec3Consts() {
    }
    Vec3Consts.ZERO = new Vec3(0, 0, 0);
    Vec3Consts.UNIT_X = new Vec3(1, 0, 0);
    Vec3Consts.UNIT_Y = new Vec3(0, 1, 0);
    Vec3Consts.UNIT_Z = new Vec3(0, 0, 1);
    return Vec3Consts;
}());
export { Vec3Consts };
