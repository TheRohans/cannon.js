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
import { EventTarget } from '../utils/EventTarget';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { Mat3 } from '../math/Mat3';
import { AABB } from '../collision/AABB';
import { Box } from '../shapes/Box';
var BodyOptions = (function () {
    function BodyOptions() {
    }
    return BodyOptions;
}());
export { BodyOptions };
var Body = (function (_super) {
    __extends(Body, _super);
    function Body(options) {
        var _this = _super.call(this) || this;
        _this.tmpVec = new Vec3();
        _this.tmpQuat = new Quaternion();
        _this.computeAABB_shapeAABB = new AABB();
        _this.uiw_m1 = new Mat3();
        _this.uiw_m2 = new Mat3();
        _this.uiw_m3 = new Mat3();
        _this.Body_applyForce_r = new Vec3();
        _this.Body_applyForce_rotForce = new Vec3();
        _this.Body_applyLocalForce_worldForce = new Vec3();
        _this.Body_applyLocalForce_relativePointWorld = new Vec3();
        _this.Body_applyImpulse_r = new Vec3();
        _this.Body_applyImpulse_velo = new Vec3();
        _this.Body_applyImpulse_rotVelo = new Vec3();
        _this.Body_applyLocalImpulse_worldImpulse = new Vec3();
        _this.Body_applyLocalImpulse_relativePoint = new Vec3();
        _this.Body_updateMassProperties_halfExtents = new Vec3();
        _this.invI_tau_dt = new Vec3();
        _this.w = new Quaternion();
        _this.wq = new Quaternion();
        options = options || {};
        _this.goid = '';
        _this.id = Body.idCounter++;
        _this.world = null;
        _this.preStep = null;
        _this.postStep = null;
        _this.vlambda = new Vec3();
        _this.collisionFilterGroup = typeof (options.collisionFilterGroup) === 'number' ? options.collisionFilterGroup : 1;
        _this.collisionFilterMask = typeof (options.collisionFilterMask) === 'number' ? options.collisionFilterMask : -1;
        _this.collisionResponse = true;
        _this.position = new Vec3();
        _this.previousPosition = new Vec3();
        _this.interpolatedPosition = new Vec3();
        _this.initPosition = new Vec3();
        if (options.position) {
            _this.position.copy(options.position);
            _this.previousPosition.copy(options.position);
            _this.interpolatedPosition.copy(options.position);
            _this.initPosition.copy(options.position);
        }
        _this.velocity = new Vec3();
        if (options.velocity) {
            _this.velocity.copy(options.velocity);
        }
        _this.initVelocity = new Vec3();
        _this.force = new Vec3();
        var mass = options.mass ? options.mass : 0;
        _this.mass = mass;
        _this.invMass = mass > 0 ? 1.0 / mass : 0;
        _this.material = options.material || null;
        _this.linearDamping = typeof (options.linearDamping) === 'number' ? options.linearDamping : 0.01;
        _this.type = (mass <= 0.0 ? Body.STATIC : Body.DYNAMIC);
        if (typeof (options.type) === typeof (Body.STATIC)) {
            _this.type = options.type;
        }
        _this.allowSleep = (options.allowSleep !== 'undefined') ? options.allowSleep : true;
        _this.sleepState = Body.AWAKE;
        _this.sleepSpeedLimit = options.sleepSpeedLimit ? options.sleepSpeedLimit : 0.1;
        _this.sleepTimeLimit = options.sleepTimeLimit ? options.sleepTimeLimit : 1;
        _this.timeLastSleepy = 0;
        _this._wakeUpAfterNarrowphase = false;
        _this.torque = new Vec3();
        _this.quaternion = new Quaternion();
        _this.initQuaternion = new Quaternion();
        _this.previousQuaternion = new Quaternion();
        _this.interpolatedQuaternion = new Quaternion();
        if (options.quaternion) {
            _this.quaternion.copy(options.quaternion);
            _this.initQuaternion.copy(options.quaternion);
            _this.previousQuaternion.copy(options.quaternion);
            _this.interpolatedQuaternion.copy(options.quaternion);
        }
        _this.angularVelocity = new Vec3();
        if (options.angularVelocity) {
            _this.angularVelocity.copy(options.angularVelocity);
        }
        _this.initAngularVelocity = new Vec3();
        _this.shapes = [];
        _this.shapeOffsets = [];
        _this.shapeOrientations = [];
        _this.inertia = new Vec3();
        _this.invInertia = new Vec3();
        _this.invInertiaWorld = new Mat3();
        _this.invMassSolve = 0;
        _this.invInertiaSolve = new Vec3();
        _this.invInertiaWorldSolve = new Mat3();
        _this.fixedRotation = (options.fixedRotation !== undefined) ? options.fixedRotation : false;
        _this.angularDamping = (options.angularDamping !== undefined) ? options.angularDamping : 0.01;
        _this.linearFactor = new Vec3(1, 1, 1);
        if (options.linearFactor) {
            _this.linearFactor.copy(options.linearFactor);
        }
        _this.angularFactor = new Vec3(1, 1, 1);
        if (options.angularFactor) {
            _this.angularFactor.copy(options.angularFactor);
        }
        _this.aabb = new AABB();
        _this.aabbNeedsUpdate = true;
        _this.boundingRadius = 0;
        _this.wlambda = new Vec3();
        if (options.shape) {
            _this.addShape(options.shape);
        }
        _this.updateMassProperties();
        return _this;
    }
    Body.prototype.wakeUp = function () {
        var s = this.sleepState;
        this.sleepState = Body.AWAKE;
        this._wakeUpAfterNarrowphase = false;
        if (s === Body.SLEEPING) {
            this.dispatchEvent(Body.wakeupEvent);
        }
    };
    Body.prototype.sleep = function () {
        this.sleepState = Body.SLEEPING;
        this.velocity.set(0, 0, 0);
        this.angularVelocity.set(0, 0, 0);
        this._wakeUpAfterNarrowphase = false;
    };
    Body.prototype.sleepTick = function (time) {
        if (this.allowSleep) {
            var sleepState = this.sleepState;
            var speedSquared = this.velocity.norm2() + this.angularVelocity.norm2();
            var speedLimitSquared = Math.pow(this.sleepSpeedLimit, 2);
            if (sleepState === Body.AWAKE && speedSquared < speedLimitSquared) {
                this.sleepState = Body.SLEEPY;
                this.timeLastSleepy = time;
                this.dispatchEvent(Body.sleepyEvent);
            }
            else if (sleepState === Body.SLEEPY && speedSquared > speedLimitSquared) {
                this.wakeUp();
            }
            else if (sleepState === Body.SLEEPY && (time - this.timeLastSleepy) > this.sleepTimeLimit) {
                this.sleep();
                this.dispatchEvent(Body.sleepEvent);
            }
        }
    };
    Body.prototype.updateSolveMassProperties = function () {
        if (this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC) {
            this.invMassSolve = 0;
            this.invInertiaSolve.setZero();
            this.invInertiaWorldSolve.setZero();
        }
        else {
            this.invMassSolve = this.invMass;
            this.invInertiaSolve.copy(this.invInertia);
            this.invInertiaWorldSolve.copy(this.invInertiaWorld);
        }
    };
    Body.prototype.pointToLocalFrame = function (worldPoint, result) {
        result = result || new Vec3();
        worldPoint.vsub(this.position, result);
        this.quaternion.conjugate().vmult(result, result);
        return result;
    };
    Body.prototype.vectorToLocalFrame = function (worldVector, result) {
        result = result || new Vec3();
        this.quaternion.conjugate().vmult(worldVector, result);
        return result;
    };
    Body.prototype.pointToWorldFrame = function (localPoint, result) {
        result = result || new Vec3();
        this.quaternion.vmult(localPoint, result);
        result.vadd(this.position, result);
        return result;
    };
    Body.prototype.vectorToWorldFrame = function (localVector, result) {
        result = result || new Vec3();
        this.quaternion.vmult(localVector, result);
        return result;
    };
    Body.prototype.addShape = function (shape, offset, orientation) {
        this.shapes.push(shape);
        this.shapeOffsets.push(offset || new Vec3());
        this.shapeOrientations.push(orientation || new Quaternion());
        this.recalculateShapes();
        shape.body = this;
        return this;
    };
    Body.prototype.recalculateShapes = function () {
        this.updateMassProperties();
        this.updateBoundingRadius();
        this.aabbNeedsUpdate = true;
    };
    Body.prototype.updateBoundingRadius = function () {
        var shapes = this.shapes, shapeOffsets = this.shapeOffsets, N = shapes.length;
        var radius = 0;
        for (var i = 0; i !== N; i++) {
            var shape = shapes[i];
            shape.updateBoundingSphereRadius();
            var offset = shapeOffsets[i].norm(), r = shape.boundingSphereRadius;
            if (offset + r > radius) {
                radius = offset + r;
            }
        }
        this.boundingRadius = radius;
    };
    Body.prototype.computeAABB = function () {
        var shapes = this.shapes, shapeOffsets = this.shapeOffsets, shapeOrientations = this.shapeOrientations, N = shapes.length, offset = this.tmpVec, orientation = this.tmpQuat, bodyQuat = this.quaternion, aabb = this.aabb, shapeAABB = this.computeAABB_shapeAABB;
        var i = N;
        while (i--) {
            var shape = shapes[i];
            bodyQuat.vmult(shapeOffsets[i], offset);
            offset.vadd(this.position, offset);
            shapeOrientations[i].mult(bodyQuat, orientation);
            shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);
            if (i === 0) {
                aabb.copy(shapeAABB);
            }
            else {
                aabb.extend(shapeAABB);
            }
        }
        this.aabbNeedsUpdate = false;
    };
    Body.prototype.updateInertiaWorld = function (force) {
        if (force === void 0) { force = false; }
        var I = this.invInertia;
        if (I.x === I.y && I.y === I.z && !force) {
        }
        else {
            var m1 = this.uiw_m1;
            var m2 = this.uiw_m2;
            var m3 = this.uiw_m3;
            m1.setRotationFromQuaternion(this.quaternion);
            m1.transpose(m2);
            m1.scale(I, m1);
            m1.mmult(m2, this.invInertiaWorld);
        }
    };
    Body.prototype.applyForce = function (force, relativePoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        var rotForce = this.Body_applyForce_rotForce;
        relativePoint.cross(force, rotForce);
        this.force.vadd(force, this.force);
        this.torque.vadd(rotForce, this.torque);
    };
    Body.prototype.applyLocalForce = function (localForce, localPoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        var worldForce = this.Body_applyLocalForce_worldForce;
        var relativePointWorld = this.Body_applyLocalForce_relativePointWorld;
        this.vectorToWorldFrame(localForce, worldForce);
        this.vectorToWorldFrame(localPoint, relativePointWorld);
        this.applyForce(worldForce, relativePointWorld);
    };
    Body.prototype.applyImpulse = function (impulse, relativePoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        var r = relativePoint;
        var velo = this.Body_applyImpulse_velo;
        velo.copy(impulse);
        velo.mult(this.invMass, velo);
        this.velocity.vadd(velo, this.velocity);
        var rotVelo = this.Body_applyImpulse_rotVelo;
        r.cross(impulse, rotVelo);
        this.invInertiaWorld.vmult(rotVelo, rotVelo);
        this.angularVelocity.vadd(rotVelo, this.angularVelocity);
    };
    Body.prototype.applyLocalImpulse = function (localImpulse, localPoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        var worldImpulse = this.Body_applyLocalImpulse_worldImpulse;
        var relativePointWorld = this.Body_applyLocalImpulse_relativePoint;
        this.vectorToWorldFrame(localImpulse, worldImpulse);
        this.vectorToWorldFrame(localPoint, relativePointWorld);
        this.applyImpulse(worldImpulse, relativePointWorld);
    };
    Body.prototype.updateMassProperties = function () {
        var halfExtents = this.Body_updateMassProperties_halfExtents;
        this.invMass = this.mass > 0 ? 1.0 / this.mass : 0;
        var I = this.inertia;
        var fixed = this.fixedRotation;
        this.computeAABB();
        halfExtents.set((this.aabb.upperBound.x - this.aabb.lowerBound.x) / 2, (this.aabb.upperBound.y - this.aabb.lowerBound.y) / 2, (this.aabb.upperBound.z - this.aabb.lowerBound.z) / 2);
        Box.calculateInertia(halfExtents, this.mass, I);
        this.invInertia.set(I.x > 0 && !fixed ? 1.0 / I.x : 0, I.y > 0 && !fixed ? 1.0 / I.y : 0, I.z > 0 && !fixed ? 1.0 / I.z : 0);
        this.updateInertiaWorld(true);
    };
    Body.prototype.getVelocityAtWorldPoint = function (worldPoint, result) {
        var r = new Vec3();
        worldPoint.vsub(this.position, r);
        this.angularVelocity.cross(r, result);
        this.velocity.vadd(result, result);
        return result;
    };
    Body.prototype.integrate = function (dt, quatNormalize, quatNormalizeFast) {
        this.previousPosition.copy(this.position);
        this.previousQuaternion.copy(this.quaternion);
        if (!(this.type === Body.DYNAMIC || this.type === Body.KINEMATIC) || this.sleepState === Body.SLEEPING) {
            return;
        }
        var velo = this.velocity, angularVelo = this.angularVelocity, pos = this.position, force = this.force, torque = this.torque, quat = this.quaternion, invMass = this.invMass, invInertia = this.invInertiaWorld, linearFactor = this.linearFactor;
        var iMdt = invMass * dt;
        velo.x += force.x * iMdt * linearFactor.x;
        velo.y += force.y * iMdt * linearFactor.y;
        velo.z += force.z * iMdt * linearFactor.z;
        var e = invInertia.elements;
        var angularFactor = this.angularFactor;
        var tx = torque.x * angularFactor.x;
        var ty = torque.y * angularFactor.y;
        var tz = torque.z * angularFactor.z;
        angularVelo.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
        angularVelo.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
        angularVelo.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);
        pos.x += velo.x * dt;
        pos.y += velo.y * dt;
        pos.z += velo.z * dt;
        quat.integrate(this.angularVelocity, dt, this.angularFactor, quat);
        if (quatNormalize) {
            if (quatNormalizeFast) {
                quat.normalizeFast();
            }
            else {
                quat.normalize();
            }
        }
        this.aabbNeedsUpdate = true;
        this.updateInertiaWorld();
    };
    Body.idCounter = 0;
    Body.COLLIDE_EVENT_NAME = 'collide';
    Body.DYNAMIC = 1;
    Body.STATIC = 2;
    Body.KINEMATIC = 4;
    Body.AWAKE = 0;
    Body.SLEEPY = 1;
    Body.SLEEPING = 2;
    Body.wakeupEvent = {
        type: 'wakeup'
    };
    Body.sleepyEvent = {
        type: 'sleepy'
    };
    Body.sleepEvent = {
        type: 'sleep'
    };
    return Body;
}(EventTarget));
export { Body };
