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
var Sphere = (function (_super) {
    __extends(Sphere, _super);
    function Sphere(radius) {
        var _this = _super.call(this, { type: Shape.types.SPHERE }) || this;
        _this.radius = radius !== undefined ? radius : 1.0;
        if (_this.radius < 0) {
            throw new Error('The sphere radius cannot be negative.');
        }
        _this.updateBoundingSphereRadius();
        return _this;
    }
    Sphere.prototype.calculateLocalInertia = function (mass, target) {
        target = target || new Vec3();
        var I = 2.0 * mass * this.radius * this.radius / 5.0;
        target.x = I;
        target.y = I;
        target.z = I;
        return target;
    };
    Sphere.prototype.volume = function () {
        return 4.0 * Math.PI * this.radius / 3.0;
    };
    Sphere.prototype.updateBoundingSphereRadius = function () {
        this.boundingSphereRadius = this.radius;
    };
    Sphere.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        var r = this.radius;
        min.x = pos.x - r;
        max.x = pos.x + r;
        min.y = pos.y - r;
        max.y = pos.y + r;
        min.z = pos.z - r;
        max.z = pos.z + r;
    };
    return Sphere;
}(Shape));
export { Sphere };
