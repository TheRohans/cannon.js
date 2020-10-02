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
var Plane = (function (_super) {
    __extends(Plane, _super);
    function Plane() {
        var _this = _super.call(this, { type: Shape.types.PLANE }) || this;
        _this.tempNormal = new Vec3();
        _this.worldNormal = new Vec3();
        _this.worldNormalNeedsUpdate = true;
        _this.boundingSphereRadius = Number.MAX_VALUE;
        return _this;
    }
    Plane.prototype.computeWorldNormal = function (quat) {
        var n = this.worldNormal;
        n.set(0, 0, 1);
        quat.vmult(n, n);
        this.worldNormalNeedsUpdate = false;
    };
    Plane.prototype.calculateLocalInertia = function (mass, target) {
        target = target || new Vec3();
        return target;
    };
    Plane.prototype.volume = function () {
        return Number.MAX_VALUE;
    };
    Plane.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        this.tempNormal.set(0, 0, 1);
        quat.vmult(this.tempNormal, this.tempNormal);
        var maxVal = Number.MAX_VALUE;
        min.set(-maxVal, -maxVal, -maxVal);
        max.set(maxVal, maxVal, maxVal);
        if (this.tempNormal.x === 1) {
            max.x = pos.x;
        }
        if (this.tempNormal.y === 1) {
            max.y = pos.y;
        }
        if (this.tempNormal.z === 1) {
            max.z = pos.z;
        }
        if (this.tempNormal.x === -1) {
            min.x = pos.x;
        }
        if (this.tempNormal.y === -1) {
            min.y = pos.y;
        }
        if (this.tempNormal.z === -1) {
            min.z = pos.z;
        }
    };
    Plane.prototype.updateBoundingSphereRadius = function () {
        this.boundingSphereRadius = Number.MAX_VALUE;
    };
    return Plane;
}(Shape));
export { Plane };
