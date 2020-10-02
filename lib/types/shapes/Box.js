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
import { Shape } from './Shape';
import { Vec3 } from '../math/Vec3';
import { ConvexPolyhedron } from './ConvexPolyhedron';
var Box = (function (_super) {
    __extends(Box, _super);
    function Box(halfExtents) {
        var _this = _super.call(this, { type: Shape.types.BOX }) || this;
        _this.worldCornerTempPos = new Vec3();
        _this.worldCornerTempNeg = new Vec3();
        _this.worldCornersTemp = [
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3(),
            new Vec3()
        ];
        _this.halfExtents = halfExtents;
        _this.convexPolyhedronRepresentation = null;
        _this.updateConvexPolyhedronRepresentation();
        _this.updateBoundingSphereRadius();
        return _this;
    }
    Box.prototype.updateConvexPolyhedronRepresentation = function () {
        var sx = this.halfExtents.x;
        var sy = this.halfExtents.y;
        var sz = this.halfExtents.z;
        var V = Vec3;
        var vertices = [
            new V(-sx, -sy, -sz),
            new V(sx, -sy, -sz),
            new V(sx, sy, -sz),
            new V(-sx, sy, -sz),
            new V(-sx, -sy, sz),
            new V(sx, -sy, sz),
            new V(sx, sy, sz),
            new V(-sx, sy, sz)
        ];
        var indices = [
            [3, 2, 1, 0],
            [4, 5, 6, 7],
            [5, 4, 0, 1],
            [2, 3, 7, 6],
            [0, 4, 7, 3],
            [1, 2, 6, 5],
        ];
        var h = new ConvexPolyhedron(vertices, indices);
        this.convexPolyhedronRepresentation = h;
        h.material = this.material;
    };
    Box.prototype.calculateLocalInertia = function (mass, target) {
        target = target || new Vec3();
        Box.calculateInertia(this.halfExtents, mass, target);
        return target;
    };
    Box.calculateInertia = function (halfExtents, mass, target) {
        var e = halfExtents;
        target.x = 1.0 / 12.0 * mass * (2 * e.y * 2 * e.y + 2 * e.z * 2 * e.z);
        target.y = 1.0 / 12.0 * mass * (2 * e.x * 2 * e.x + 2 * e.z * 2 * e.z);
        target.z = 1.0 / 12.0 * mass * (2 * e.y * 2 * e.y + 2 * e.x * 2 * e.x);
    };
    Box.prototype.getSideNormals = function (sixTargetVectors, quat) {
        var sides = sixTargetVectors;
        var ex = this.halfExtents;
        sides[0].set(ex.x, 0, 0);
        sides[1].set(0, ex.y, 0);
        sides[2].set(0, 0, ex.z);
        sides[3].set(-ex.x, 0, 0);
        sides[4].set(0, -ex.y, 0);
        sides[5].set(0, 0, -ex.z);
        if (quat !== undefined) {
            for (var i = 0; i !== sides.length; i++) {
                quat.vmult(sides[i], sides[i]);
            }
        }
        return sides;
    };
    Box.prototype.volume = function () {
        return 8.0 * this.halfExtents.x * this.halfExtents.y * this.halfExtents.z;
    };
    Box.prototype.updateBoundingSphereRadius = function () {
        this.boundingSphereRadius = this.halfExtents.norm();
    };
    Box.prototype.forEachWorldCorner = function (pos, quat, callback) {
        var e = this.halfExtents;
        var corners = [[e.x, e.y, e.z],
            [-e.x, e.y, e.z],
            [-e.x, -e.y, e.z],
            [-e.x, -e.y, -e.z],
            [e.x, -e.y, -e.z],
            [e.x, e.y, -e.z],
            [-e.x, e.y, -e.z],
            [e.x, -e.y, e.z]];
        for (var i = 0; i < corners.length; i++) {
            this.worldCornerTempPos.set(corners[i][0], corners[i][1], corners[i][2]);
            quat.vmult(this.worldCornerTempPos, this.worldCornerTempPos);
            pos.vadd(this.worldCornerTempPos, this.worldCornerTempPos);
            callback(this.worldCornerTempPos.x, this.worldCornerTempPos.y, this.worldCornerTempPos.z);
        }
    };
    Box.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        var e = this.halfExtents;
        this.worldCornersTemp[0].set(e.x, e.y, e.z);
        this.worldCornersTemp[1].set(-e.x, e.y, e.z);
        this.worldCornersTemp[2].set(-e.x, -e.y, e.z);
        this.worldCornersTemp[3].set(-e.x, -e.y, -e.z);
        this.worldCornersTemp[4].set(e.x, -e.y, -e.z);
        this.worldCornersTemp[5].set(e.x, e.y, -e.z);
        this.worldCornersTemp[6].set(-e.x, e.y, -e.z);
        this.worldCornersTemp[7].set(e.x, -e.y, e.z);
        var wc = this.worldCornersTemp[0];
        quat.vmult(wc, wc);
        pos.vadd(wc, wc);
        max.copy(wc);
        min.copy(wc);
        for (var i = 1; i < 8; i++) {
            var wc2 = this.worldCornersTemp[i];
            quat.vmult(wc2, wc2);
            pos.vadd(wc2, wc2);
            var x = wc2.x;
            var y = wc2.y;
            var z = wc2.z;
            if (x > max.x) {
                max.x = x;
            }
            if (y > max.y) {
                max.y = y;
            }
            if (z > max.z) {
                max.z = z;
            }
            if (x < min.x) {
                min.x = x;
            }
            if (y < min.y) {
                min.y = y;
            }
            if (z < min.z) {
                min.z = z;
            }
        }
    };
    return Box;
}(Shape));
export { Box };
