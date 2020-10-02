import { Vec3 } from './Vec3';
var JacobianElement = (function () {
    function JacobianElement() {
        this.spatial = new Vec3();
        this.rotational = new Vec3();
    }
    JacobianElement.prototype.multiplyElement = function (element) {
        return element.spatial.dot(this.spatial) + element.rotational.dot(this.rotational);
    };
    JacobianElement.prototype.multiplyVectors = function (spatial, rotational) {
        return spatial.dot(this.spatial) + rotational.dot(this.rotational);
    };
    return JacobianElement;
}());
export { JacobianElement };
