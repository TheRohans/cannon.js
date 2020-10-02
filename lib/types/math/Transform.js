import { Vec3 } from './Vec3';
import { Quaternion } from './Quaternion';
var Transform = (function () {
    function Transform(options) {
        if (options === void 0) { options = {}; }
        this.position = new Vec3();
        if (options.position) {
            this.position.copy(options.position);
        }
        this.quaternion = new Quaternion();
        if (options.quaternion) {
            this.quaternion.copy(options.quaternion);
        }
    }
    Transform.pointToLocalFrame = function (position, quaternion, worldPoint, result) {
        var tmpQuat = Transform._plfTempQuat;
        result = result || new Vec3();
        worldPoint.vsub(position, result);
        quaternion.conjugate(tmpQuat);
        tmpQuat.vmult(result, result);
        return result;
    };
    Transform.pointToWorldFrame = function (position, quaternion, localPoint, result) {
        result = result || new Vec3();
        quaternion.vmult(localPoint, result);
        result.vadd(position, result);
        return result;
    };
    Transform.vectorToWorldFrame = function (quaternion, localVector, result) {
        result = result || new Vec3();
        quaternion.vmult(localVector, result);
        return result;
    };
    Transform.vectorToLocalFrame = function (position, quaternion, worldVector, result) {
        result = result || new Vec3();
        quaternion.w *= -1;
        quaternion.vmult(worldVector, result);
        quaternion.w *= -1;
        return result;
    };
    Transform.prototype.pointToLocal = function (worldPoint, result) {
        return Transform.pointToLocalFrame(this.position, this.quaternion, worldPoint, result);
    };
    Transform.prototype.pointToWorld = function (localPoint, result) {
        return Transform.pointToWorldFrame(this.position, this.quaternion, localPoint, result);
    };
    Transform._plfTempQuat = new Quaternion();
    return Transform;
}());
export { Transform };
