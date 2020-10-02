import { Vec3 } from '../math/Vec3';
var RaycastResult = (function () {
    function RaycastResult() {
        this.rayFromWorld = new Vec3();
        this.rayToWorld = new Vec3();
        this.hitNormalWorld = new Vec3();
        this.hitPointWorld = new Vec3();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this._shouldStop = false;
    }
    RaycastResult.prototype.reset = function () {
        this.rayFromWorld.setZero();
        this.rayToWorld.setZero();
        this.hitNormalWorld.setZero();
        this.hitPointWorld.setZero();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this._shouldStop = false;
    };
    RaycastResult.prototype.abort = function () {
        this._shouldStop = true;
    };
    RaycastResult.prototype.set = function (rayFromWorld, rayToWorld, hitNormalWorld, hitPointWorld, shape, body, distance) {
        this.rayFromWorld.copy(rayFromWorld);
        this.rayToWorld.copy(rayToWorld);
        this.hitNormalWorld.copy(hitNormalWorld);
        this.hitPointWorld.copy(hitPointWorld);
        this.shape = shape;
        this.body = body;
        this.distance = distance;
    };
    return RaycastResult;
}());
export { RaycastResult };
