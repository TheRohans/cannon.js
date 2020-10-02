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
var Particle = (function (_super) {
    __extends(Particle, _super);
    function Particle() {
        return _super.call(this, { type: Shape.types.PARTICLE }) || this;
    }
    Particle.prototype.calculateLocalInertia = function (mass, target) {
        target = target || new Vec3();
        target.set(0, 0, 0);
        return target;
    };
    Particle.prototype.volume = function () {
        return 0;
    };
    Particle.prototype.updateBoundingSphereRadius = function () {
        this.boundingSphereRadius = 0;
    };
    Particle.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        min.copy(pos);
        max.copy(pos);
    };
    return Particle;
}(Shape));
export { Particle };
