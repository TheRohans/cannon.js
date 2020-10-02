import { Vec3 } from '../math/Vec3';
var AABB = (function () {
    function AABB(options) {
        if (options === void 0) { options = {}; }
        this.tmp = new Vec3();
        this.transformIntoFrame_corners = [
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3()
        ];
        this.lowerBound = new Vec3();
        if (options.lowerBound) {
            this.lowerBound.copy(options.lowerBound);
        }
        this.upperBound = new Vec3();
        if (options.upperBound) {
            this.upperBound.copy(options.upperBound);
        }
    }
    AABB.prototype.setFromPoints = function (points, position, quaternion, skinSize) {
        var l = this.lowerBound, u = this.upperBound, q = quaternion;
        l.copy(points[0]);
        if (q) {
            q.vmult(l, l);
        }
        u.copy(l);
        for (var i = 1; i < points.length; i++) {
            var p = points[i];
            if (q) {
                q.vmult(p, this.tmp);
                p = this.tmp;
            }
            if (p.x > u.x) {
                u.x = p.x;
            }
            if (p.x < l.x) {
                l.x = p.x;
            }
            if (p.y > u.y) {
                u.y = p.y;
            }
            if (p.y < l.y) {
                l.y = p.y;
            }
            if (p.z > u.z) {
                u.z = p.z;
            }
            if (p.z < l.z) {
                l.z = p.z;
            }
        }
        if (position) {
            position.vadd(l, l);
            position.vadd(u, u);
        }
        if (skinSize) {
            l.x -= skinSize;
            l.y -= skinSize;
            l.z -= skinSize;
            u.x += skinSize;
            u.y += skinSize;
            u.z += skinSize;
        }
        return this;
    };
    AABB.prototype.copy = function (aabb) {
        this.lowerBound.copy(aabb.lowerBound);
        this.upperBound.copy(aabb.upperBound);
        return this;
    };
    AABB.prototype.clone = function () {
        return new AABB().copy(this);
    };
    AABB.prototype.extend = function (aabb) {
        this.lowerBound.x = Math.min(this.lowerBound.x, aabb.lowerBound.x);
        this.upperBound.x = Math.max(this.upperBound.x, aabb.upperBound.x);
        this.lowerBound.y = Math.min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.y = Math.max(this.upperBound.y, aabb.upperBound.y);
        this.lowerBound.z = Math.min(this.lowerBound.z, aabb.lowerBound.z);
        this.upperBound.z = Math.max(this.upperBound.z, aabb.upperBound.z);
    };
    AABB.prototype.overlaps = function (aabb) {
        var l1 = this.lowerBound, u1 = this.upperBound, l2 = aabb.lowerBound, u2 = aabb.upperBound;
        var overlapsX = ((l2.x <= u1.x && u1.x <= u2.x) || (l1.x <= u2.x && u2.x <= u1.x));
        var overlapsY = ((l2.y <= u1.y && u1.y <= u2.y) || (l1.y <= u2.y && u2.y <= u1.y));
        var overlapsZ = ((l2.z <= u1.z && u1.z <= u2.z) || (l1.z <= u2.z && u2.z <= u1.z));
        return overlapsX && overlapsY && overlapsZ;
    };
    AABB.prototype.volume = function () {
        var l = this.lowerBound, u = this.upperBound;
        return (u.x - l.x) * (u.y - l.y) * (u.z - l.z);
    };
    AABB.prototype.contains = function (aabb) {
        var l1 = this.lowerBound, u1 = this.upperBound, l2 = aabb.lowerBound, u2 = aabb.upperBound;
        return ((l1.x <= l2.x && u1.x >= u2.x) &&
            (l1.y <= l2.y && u1.y >= u2.y) &&
            (l1.z <= l2.z && u1.z >= u2.z));
    };
    AABB.prototype.getCorners = function (a, b, c, d, e, f, g, h) {
        var l = this.lowerBound, u = this.upperBound;
        a.copy(l);
        b.set(u.x, l.y, l.z);
        c.set(u.x, u.y, l.z);
        d.set(l.x, u.y, u.z);
        e.set(u.x, l.y, l.z);
        f.set(l.x, u.y, l.z);
        g.set(l.x, l.y, u.z);
        h.copy(u);
    };
    AABB.prototype.toLocalFrame = function (frame, target) {
        var corners = this.transformIntoFrame_corners;
        var a = corners[0];
        var b = corners[1];
        var c = corners[2];
        var d = corners[3];
        var e = corners[4];
        var f = corners[5];
        var g = corners[6];
        var h = corners[7];
        this.getCorners(a, b, c, d, e, f, g, h);
        for (var i = 0; i !== 8; i++) {
            var corner = corners[i];
            frame.pointToLocal(corner, corner);
        }
        return target.setFromPoints(corners);
    };
    AABB.prototype.toWorldFrame = function (frame, target) {
        var corners = this.transformIntoFrame_corners;
        var a = corners[0];
        var b = corners[1];
        var c = corners[2];
        var d = corners[3];
        var e = corners[4];
        var f = corners[5];
        var g = corners[6];
        var h = corners[7];
        this.getCorners(a, b, c, d, e, f, g, h);
        for (var i = 0; i !== 8; i++) {
            var corner = corners[i];
            frame.pointToWorld(corner, corner);
        }
        return target.setFromPoints(corners);
    };
    AABB.prototype.overlapsRay = function (ray) {
        var dirFracX = 1 / ray._direction.x;
        var dirFracY = 1 / ray._direction.y;
        var dirFracZ = 1 / ray._direction.z;
        var t1 = (this.lowerBound.x - ray.from.x) * dirFracX;
        var t2 = (this.upperBound.x - ray.from.x) * dirFracX;
        var t3 = (this.lowerBound.y - ray.from.y) * dirFracY;
        var t4 = (this.upperBound.y - ray.from.y) * dirFracY;
        var t5 = (this.lowerBound.z - ray.from.z) * dirFracZ;
        var t6 = (this.upperBound.z - ray.from.z) * dirFracZ;
        var tmin = Math.max(Math.max(Math.min(t1, t2), Math.min(t3, t4)), Math.min(t5, t6));
        var tmax = Math.min(Math.min(Math.max(t1, t2), Math.max(t3, t4)), Math.max(t5, t6));
        if (tmax < 0) {
            return false;
        }
        if (tmin > tmax) {
            return false;
        }
        return true;
    };
    AABB.prototype.halfExtents = function () {
        if (!this.upperBound || !this.lowerBound) {
            return new Vec3();
        }
        return this.upperBound.vsub(this.lowerBound).mult(.5);
    };
    return AABB;
}());
export { AABB };
