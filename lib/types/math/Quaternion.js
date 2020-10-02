import { Vec3 } from './Vec3';
var Quaternion = (function () {
    function Quaternion(x, y, z, w) {
        this.sfv_t1 = new Vec3();
        this.sfv_t2 = new Vec3();
        this.x = x !== undefined ? x : 0;
        this.y = y !== undefined ? y : 0;
        this.z = z !== undefined ? z : 0;
        this.w = w !== undefined ? w : 1;
    }
    Quaternion.prototype.set = function (x, y, z, w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    };
    Quaternion.prototype.toString = function () {
        return this.x + ',' + this.y + ',' + this.z + ',' + this.w;
    };
    Quaternion.prototype.toArray = function () {
        return [this.x, this.y, this.z, this.w];
    };
    Quaternion.prototype.setFromAxisAngle = function (axis, angle) {
        var s = Math.sin(angle * 0.5);
        this.x = axis.x * s;
        this.y = axis.y * s;
        this.z = axis.z * s;
        this.w = Math.cos(angle * 0.5);
        return this;
    };
    Quaternion.prototype.toAxisAngle = function (targetAxis) {
        targetAxis = targetAxis || new Vec3();
        this.normalize();
        var angle = 2 * Math.acos(this.w);
        var s = Math.sqrt(1 - this.w * this.w);
        if (s < 0.001) {
            targetAxis.x = this.x;
            targetAxis.y = this.y;
            targetAxis.z = this.z;
        }
        else {
            targetAxis.x = this.x / s;
            targetAxis.y = this.y / s;
            targetAxis.z = this.z / s;
        }
        return [targetAxis, angle];
    };
    Quaternion.prototype.setFromVectors = function (u, v) {
        if (u.isAntiparallelTo(v)) {
            var t1 = this.sfv_t1;
            var t2 = this.sfv_t2;
            u.tangents(t1, t2);
            this.setFromAxisAngle(t1, Math.PI);
        }
        else {
            var a = u.cross(v);
            this.x = a.x;
            this.y = a.y;
            this.z = a.z;
            this.w = Math.sqrt(Math.pow(u.norm(), 2) * Math.pow(v.norm(), 2)) + u.dot(v);
            this.normalize();
        }
        return this;
    };
    Quaternion.prototype.mult = function (q, target) {
        target = target || new Quaternion();
        var ax = this.x, ay = this.y, az = this.z, aw = this.w, bx = q.x, by = q.y, bz = q.z, bw = q.w;
        target.x = ax * bw + aw * bx + ay * bz - az * by;
        target.y = ay * bw + aw * by + az * bx - ax * bz;
        target.z = az * bw + aw * bz + ax * by - ay * bx;
        target.w = aw * bw - ax * bx - ay * by - az * bz;
        return target;
    };
    Quaternion.prototype.inverse = function (target) {
        var x = this.x, y = this.y, z = this.z, w = this.w;
        target = target || new Quaternion();
        this.conjugate(target);
        var inorm2 = 1 / (x * x + y * y + z * z + w * w);
        target.x *= inorm2;
        target.y *= inorm2;
        target.z *= inorm2;
        target.w *= inorm2;
        return target;
    };
    Quaternion.prototype.conjugate = function (target) {
        target = target || new Quaternion();
        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        target.w = this.w;
        return target;
    };
    Quaternion.prototype.normalize = function () {
        var l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
        if (l === 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        }
        else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
        return this;
    };
    Quaternion.prototype.normalizeFast = function () {
        var f = (3.0 - (this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w)) / 2.0;
        if (f === 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        }
        else {
            this.x *= f;
            this.y *= f;
            this.z *= f;
            this.w *= f;
        }
        return this;
    };
    Quaternion.prototype.vmult = function (v, target) {
        target = target || new Vec3();
        var x = v.x, y = v.y, z = v.z;
        var qx = this.x, qy = this.y, qz = this.z, qw = this.w;
        var ix = qw * x + qy * z - qz * y, iy = qw * y + qz * x - qx * z, iz = qw * z + qx * y - qy * x, iw = -qx * x - qy * y - qz * z;
        target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
        return target;
    };
    Quaternion.prototype.copy = function (source) {
        this.x = source.x;
        this.y = source.y;
        this.z = source.z;
        this.w = source.w;
        return this;
    };
    Quaternion.prototype.toEuler = function (target, order) {
        if (order === void 0) { order = 'YZX'; }
        var heading, attitude, bank;
        var x = this.x, y = this.y, z = this.z, w = this.w;
        switch (order) {
            case 'YZX':
                var test_1 = x * y + z * w;
                if (test_1 > 0.499) {
                    heading = 2 * Math.atan2(x, w);
                    attitude = Math.PI / 2;
                    bank = 0;
                }
                if (test_1 < -0.499) {
                    heading = -2 * Math.atan2(x, w);
                    attitude = -Math.PI / 2;
                    bank = 0;
                }
                if (isNaN(heading)) {
                    var sqx = x * x;
                    var sqy = y * y;
                    var sqz = z * z;
                    heading = Math.atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
                    attitude = Math.asin(2 * test_1);
                    bank = Math.atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz);
                }
                break;
            default:
                throw new Error('Euler order ' + order + ' not supported yet.');
        }
        target.y = heading;
        target.z = attitude;
        target.x = bank;
    };
    Quaternion.prototype.setFromEuler = function (x, y, z, order) {
        if (order === void 0) { order = 'XYZ'; }
        var c1 = Math.cos(x / 2);
        var c2 = Math.cos(y / 2);
        var c3 = Math.cos(z / 2);
        var s1 = Math.sin(x / 2);
        var s2 = Math.sin(y / 2);
        var s3 = Math.sin(z / 2);
        if (order === 'XYZ') {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === 'YXZ') {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        else if (order === 'ZXY') {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === 'ZYX') {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        else if (order === 'YZX') {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === 'XZY') {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        return this;
    };
    Quaternion.prototype.clone = function () {
        return new Quaternion(this.x, this.y, this.z, this.w);
    };
    Quaternion.prototype.slerp = function (toQuat, t, target) {
        target = target || new Quaternion();
        var ax = this.x, ay = this.y, az = this.z, aw = this.w;
        var bx = toQuat.x, by = toQuat.y, bz = toQuat.z, bw = toQuat.w;
        var omega, cosom, sinom, scale0, scale1;
        cosom = ax * bx + ay * by + az * bz + aw * bw;
        if (cosom < 0.0) {
            cosom = -cosom;
            bx = -bx;
            by = -by;
            bz = -bz;
            bw = -bw;
        }
        if ((1.0 - cosom) > 0.000001) {
            omega = Math.acos(cosom);
            sinom = Math.sin(omega);
            scale0 = Math.sin((1.0 - t) * omega) / sinom;
            scale1 = Math.sin(t * omega) / sinom;
        }
        else {
            scale0 = 1.0 - t;
            scale1 = t;
        }
        target.x = scale0 * ax + scale1 * bx;
        target.y = scale0 * ay + scale1 * by;
        target.z = scale0 * az + scale1 * bz;
        target.w = scale0 * aw + scale1 * bw;
        return target;
    };
    Quaternion.prototype.integrate = function (angularVelocity, dt, angularFactor, target) {
        target = target || new Quaternion();
        var ax = angularVelocity.x * angularFactor.x, ay = angularVelocity.y * angularFactor.y, az = angularVelocity.z * angularFactor.z, bx = this.x, by = this.y, bz = this.z, bw = this.w;
        var half_dt = dt * 0.5;
        target.x += half_dt * (ax * bw + ay * bz - az * by);
        target.y += half_dt * (ay * bw + az * bx - ax * bz);
        target.z += half_dt * (az * bw + ax * by - ay * bx);
        target.w += half_dt * (-ax * bx - ay * by - az * bz);
        return target;
    };
    return Quaternion;
}());
export { Quaternion };
