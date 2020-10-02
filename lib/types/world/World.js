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
import { Material } from '../material/Material';
import { ContactMaterial } from '../material/ContactMaterial';
import { Ray } from '../collision/Ray';
import { TupleDictionary } from '../utils/TupleDictionary';
import { Body } from '../objects/Body';
import { RaycastResult } from '../collision/RaycastResult';
import { GSSolver } from '../solver/GSSolver';
import { Narrowphase } from './Narrowphase';
import { OverlapKeeper } from '../collision/OverlapKeeper';
import { ArrayCollisionMatrix } from '../collision/ArrayCollisionMatrix';
import { NaiveBroadphase } from '../collision/NaiveBroadphase';
var World = (function (_super) {
    __extends(World, _super);
    function World(options) {
        var _this = _super.call(this) || this;
        _this.gravity = new Vec3();
        _this.tmpRay = new Ray();
        _this.removeBody = function (body) { return _this.remove(body); };
        _this.World_step_preStepEvent = { type: 'preStep' };
        _this.World_step_collideEvent = { type: Body.COLLIDE_EVENT_NAME, body: null, contact: null };
        _this.World_step_oldContacts = [];
        _this.World_step_frictionEquationPool = [];
        _this.World_step_p1 = [];
        _this.World_step_p2 = [];
        _this.World_step_postStepEvent = { type: 'postStep' };
        _this.emitContactEvents = function () {
            var additions = [];
            var removals = [];
            var beginContactEvent = {
                type: 'beginContact',
                bodyA: null,
                bodyB: null
            };
            var endContactEvent = {
                type: 'endContact',
                bodyA: null,
                bodyB: null
            };
            var beginShapeContactEvent = {
                type: 'beginShapeContact',
                bodyA: null,
                bodyB: null,
                shapeA: null,
                shapeB: null
            };
            var endShapeContactEvent = {
                type: 'endShapeContact',
                bodyA: null,
                bodyB: null,
                shapeA: null,
                shapeB: null
            };
            return (function () {
                var hasBeginContact = _this.hasAnyEventListener('beginContact');
                var hasEndContact = _this.hasAnyEventListener('endContact');
                if (hasBeginContact || hasEndContact) {
                    _this.bodyOverlapKeeper.getDiff(additions, removals);
                }
                if (hasBeginContact) {
                    for (var i = 0, l = additions.length; i < l; i += 2) {
                        beginContactEvent.bodyA = _this.getBodyById(additions[i]);
                        beginContactEvent.bodyB = _this.getBodyById(additions[i + 1]);
                        if (beginContactEvent.bodyA && beginContactEvent.bodyB) {
                            _this.dispatchEvent(beginContactEvent);
                        }
                        beginContactEvent.bodyA = beginContactEvent.bodyB = undefined;
                    }
                }
                if (hasEndContact) {
                    for (var i = 0, l = removals.length; i < l; i += 2) {
                        endContactEvent.bodyA = _this.getBodyById(removals[i]);
                        endContactEvent.bodyB = _this.getBodyById(removals[i + 1]);
                        if (endContactEvent.bodyA && endContactEvent.bodyB) {
                            _this.dispatchEvent(endContactEvent);
                        }
                        endContactEvent.bodyA = endContactEvent.bodyB = undefined;
                    }
                }
                additions.length = removals.length = 0;
                var hasBeginShapeContact = _this.hasAnyEventListener('beginShapeContact');
                var hasEndShapeContact = _this.hasAnyEventListener('endShapeContact');
                if (hasBeginShapeContact || hasEndShapeContact) {
                    _this.shapeOverlapKeeper.getDiff(additions, removals);
                }
                if (hasBeginShapeContact) {
                    for (var i = 0, l = additions.length; i < l; i += 2) {
                        var shapeA = _this.getShapeById(additions[i]);
                        var shapeB = _this.getShapeById(additions[i + 1]);
                        beginShapeContactEvent.shapeA = shapeA;
                        beginShapeContactEvent.shapeB = shapeB;
                        beginShapeContactEvent.bodyA = shapeA.body;
                        beginShapeContactEvent.bodyB = shapeB.body;
                        _this.dispatchEvent(beginShapeContactEvent);
                        beginShapeContactEvent.shapeA = beginShapeContactEvent.shapeB = undefined;
                        beginShapeContactEvent.bodyA = beginShapeContactEvent.bodyB = undefined;
                    }
                }
                if (hasEndShapeContact) {
                    for (var i = 0, l = removals.length; i < l; i += 2) {
                        var shapeA = _this.getShapeById(removals[i]);
                        var shapeB = _this.getShapeById(removals[i + 1]);
                        endShapeContactEvent.shapeA = shapeA;
                        endShapeContactEvent.shapeB = shapeB;
                        endShapeContactEvent.bodyA = shapeA.body;
                        endShapeContactEvent.bodyB = shapeB.body;
                        _this.dispatchEvent(endShapeContactEvent);
                        endShapeContactEvent.shapeA = endShapeContactEvent.shapeB = undefined;
                        endShapeContactEvent.bodyA = endShapeContactEvent.bodyB = undefined;
                    }
                }
            })();
        };
        options = options || {};
        _this.dt = -1;
        _this.allowSleep = options.allowSleep;
        _this.contacts = [];
        _this.frictionEquations = [];
        _this.quatNormalizeSkip = options.quatNormalizeSkip !== undefined ? options.quatNormalizeSkip : 0;
        _this.quatNormalizeFast = options.quatNormalizeFast !== undefined ? options.quatNormalizeFast : false;
        _this.time = 0.0;
        _this.stepnumber = 0;
        _this.default_dt = 1 / 60;
        _this.nextId = 0;
        _this.gravity = options.gravity ? _this.gravity.copy(options.gravity) : new Vec3();
        _this.broadphase = options.broadphase ? options.broadphase : new NaiveBroadphase();
        _this.bodies = [];
        _this.solver = options.solver ? options.solver : new GSSolver();
        _this.constraints = [];
        _this.narrowphase = new Narrowphase(_this);
        _this.collisionMatrix = new ArrayCollisionMatrix();
        _this.collisionMatrixPrevious = new ArrayCollisionMatrix();
        _this.bodyOverlapKeeper = new OverlapKeeper();
        _this.shapeOverlapKeeper = new OverlapKeeper();
        _this.materials = [];
        _this.contactmaterials = [];
        _this.contactMaterialTable = new TupleDictionary();
        _this.defaultMaterial = new Material({ name: 'default' });
        _this.defaultContactMaterial = new ContactMaterial(_this.defaultMaterial, _this.defaultMaterial, { friction: 1, restitution: 0.0 });
        _this.doProfiling = false;
        _this.profile = {
            solve: 0,
            makeContactConstraints: 0,
            broadphase: 0,
            integrate: 0,
            narrowphase: 0,
        };
        _this.accumulator = 0;
        _this.subsystems = [];
        _this.addBodyEvent = {
            type: 'addBody',
            body: null
        };
        _this.removeBodyEvent = {
            type: 'removeBody',
            body: null
        };
        _this.idToBodyMap = {};
        _this.broadphase.setWorld(_this);
        return _this;
    }
    World.prototype.getContactMaterial = function (m1, m2) {
        return this.contactMaterialTable.get(m1.id, m2.id);
    };
    World.prototype.numObjects = function () {
        return this.bodies.length;
    };
    World.prototype.collisionMatrixTick = function () {
        var temp = this.collisionMatrixPrevious;
        this.collisionMatrixPrevious = this.collisionMatrix;
        this.collisionMatrix = temp;
        this.collisionMatrix.reset();
        this.bodyOverlapKeeper.tick();
        this.shapeOverlapKeeper.tick();
    };
    World.prototype.addBody = function (body) {
        if (this.bodies.indexOf(body) !== -1) {
            return;
        }
        body.index = this.bodies.length;
        this.bodies.push(body);
        body.world = this;
        body.initPosition.copy(body.position);
        body.initVelocity.copy(body.velocity);
        body.timeLastSleepy = this.time;
        if (body instanceof Body) {
            body.initAngularVelocity.copy(body.angularVelocity);
            body.initQuaternion.copy(body.quaternion);
        }
        this.collisionMatrix.setNumObjects(this.bodies.length);
        this.addBodyEvent.body = body;
        this.idToBodyMap[body.id] = body;
        this.dispatchEvent(this.addBodyEvent);
    };
    World.prototype.addConstraint = function (c) {
        this.constraints.push(c);
    };
    World.prototype.removeConstraint = function (c) {
        var idx = this.constraints.indexOf(c);
        if (idx !== -1) {
            this.constraints.splice(idx, 1);
        }
    };
    World.prototype.rayTest = function (from, to, result) {
        if (result instanceof RaycastResult) {
            this.raycastClosest(from, to, {
                skipBackfaces: true
            }, result);
        }
        else {
            this.raycastAll(from, to, {
                skipBackfaces: true
            }, result);
        }
    };
    World.prototype.raycastAll = function (from, to, options, callback) {
        options.mode = Ray.ALL;
        options.from = from;
        options.to = to;
        options.callback = callback;
        return this.tmpRay.intersectWorld(this, options);
    };
    World.prototype.raycastAny = function (from, to, options, result) {
        options.mode = Ray.ANY;
        options.from = from;
        options.to = to;
        options.result = result;
        return this.tmpRay.intersectWorld(this, options);
    };
    World.prototype.raycastClosest = function (from, to, options, result) {
        options.mode = Ray.CLOSEST;
        options.from = from;
        options.to = to;
        options.result = result;
        return this.tmpRay.intersectWorld(this, options);
    };
    World.prototype.remove = function (body) {
        body.world = null;
        var n = this.bodies.length - 1, bodies = this.bodies, idx = bodies.indexOf(body);
        if (idx !== -1) {
            bodies.splice(idx, 1);
            for (var i = 0; i !== bodies.length; i++) {
                bodies[i].index = i;
            }
            this.collisionMatrix.setNumObjects(n);
            this.removeBodyEvent.body = body;
            delete this.idToBodyMap[body.id];
            this.dispatchEvent(this.removeBodyEvent);
        }
    };
    World.prototype.getBodyById = function (id) {
        return this.idToBodyMap[id];
    };
    World.prototype.getShapeById = function (id) {
        var bodies = this.bodies;
        for (var i = 0, bl = bodies.length; i < bl; i++) {
            var shapes = bodies[i].shapes;
            for (var j = 0, sl = shapes.length; j < sl; j++) {
                var shape = shapes[j];
                if (shape.id === id) {
                    return shape;
                }
            }
        }
        return undefined;
    };
    World.prototype.addMaterial = function (m) {
        this.materials.push(m);
    };
    World.prototype.addContactMaterial = function (cmat) {
        this.contactmaterials.push(cmat);
        this.contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
    };
    World.prototype.step = function (dt, timeSinceLastCalled, maxSubSteps) {
        maxSubSteps = maxSubSteps || 10;
        timeSinceLastCalled = timeSinceLastCalled || -1;
        if (timeSinceLastCalled === -1) {
            this.internalStep(dt);
            this.time += dt;
        }
        else {
            this.accumulator += timeSinceLastCalled;
            var substeps = 0;
            while (this.accumulator >= dt && substeps < maxSubSteps) {
                this.internalStep(dt);
                this.accumulator -= dt;
                substeps++;
            }
            this.accumulator %= dt;
            var t = this.accumulator / dt;
            var blen = this.bodies.length;
            for (var j = 0; j !== blen; j++) {
                var b = this.bodies[j];
                b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
                b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
                b.previousQuaternion.normalize();
            }
            this.time += timeSinceLastCalled;
        }
    };
    World.prototype.internalStep = function (dt) {
        this.dt = dt;
        var contacts = this.contacts, p1 = this.World_step_p1, p2 = this.World_step_p2, N = this.numObjects(), bodies = this.bodies, solver = this.solver, gravity = this.gravity, doProfiling = this.doProfiling, profile = this.profile, DYNAMIC = Body.DYNAMIC, constraints = this.constraints, frictionEquationPool = this.World_step_frictionEquationPool, gx = gravity.x, gy = gravity.y, gz = gravity.z;
        var i = 0;
        var profilingStart;
        if (doProfiling) {
            profilingStart = performance.now();
        }
        for (i = 0; i !== N; i++) {
            var bi = bodies[i];
            if (bi.type === DYNAMIC) {
                var f = bi.force, m = bi.mass;
                f.x += m * gx;
                f.y += m * gy;
                f.z += m * gz;
            }
        }
        var Nsubsystems;
        for (i = 0, Nsubsystems = this.subsystems.length; i !== Nsubsystems; i++) {
            this.subsystems[i].update();
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        p1.length = 0;
        p2.length = 0;
        this.broadphase.collisionPairs(this, p1, p2);
        if (doProfiling) {
            profile.broadphase = performance.now() - profilingStart;
        }
        var Nconstraints = constraints.length;
        for (i = 0; i !== Nconstraints; i++) {
            var c = constraints[i];
            if (!c.collideConnected) {
                for (var j = p1.length - 1; j >= 0; j -= 1) {
                    if ((c.bodyA === p1[j] && c.bodyB === p2[j]) ||
                        (c.bodyB === p1[j] && c.bodyA === p2[j])) {
                        p1.splice(j, 1);
                        p2.splice(j, 1);
                    }
                }
            }
        }
        this.collisionMatrixTick();
        if (doProfiling) {
            profilingStart = performance.now();
        }
        var oldcontacts = this.World_step_oldContacts;
        var NoldContacts = contacts.length;
        for (i = 0; i !== NoldContacts; i++) {
            oldcontacts.push(contacts[i]);
        }
        contacts.length = 0;
        var NoldFrictionEquations = this.frictionEquations.length;
        for (i = 0; i !== NoldFrictionEquations; i++) {
            frictionEquationPool.push(this.frictionEquations[i]);
        }
        this.frictionEquations.length = 0;
        this.narrowphase.getContacts(p1, p2, this, contacts, oldcontacts, this.frictionEquations, frictionEquationPool);
        if (doProfiling) {
            profile.narrowphase = performance.now() - profilingStart;
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        for (i = 0; i < this.frictionEquations.length; i++) {
            solver.addEquation(this.frictionEquations[i]);
        }
        var ncontacts = contacts.length;
        for (var k = 0; k !== ncontacts; k++) {
            var c = contacts[k];
            var bi = c.bi, bj = c.bj, si = c.si, sj = c.sj;
            var cm = void 0;
            if (bi.material && bj.material) {
                cm = this.getContactMaterial(bi.material, bj.material) || this.defaultContactMaterial;
            }
            else {
                cm = this.defaultContactMaterial;
            }
            var mu = cm.friction;
            if (bi.material && bj.material) {
                if (bi.material.friction >= 0 && bj.material.friction >= 0) {
                    mu = bi.material.friction * bj.material.friction;
                }
                if (bi.material.restitution >= 0 && bj.material.restitution >= 0) {
                    c.restitution = bi.material.restitution * bj.material.restitution;
                }
            }
            solver.addEquation(c);
            if (bi.allowSleep &&
                bi.type === Body.DYNAMIC &&
                bi.sleepState === Body.SLEEPING &&
                bj.sleepState === Body.AWAKE &&
                bj.type !== Body.STATIC) {
                var speedSquaredB = bj.velocity.norm2() + bj.angularVelocity.norm2();
                var speedLimitSquaredB = Math.pow(bj.sleepSpeedLimit, 2);
                if (speedSquaredB >= speedLimitSquaredB * 2) {
                    bi._wakeUpAfterNarrowphase = true;
                }
            }
            if (bj.allowSleep &&
                bj.type === Body.DYNAMIC &&
                bj.sleepState === Body.SLEEPING &&
                bi.sleepState === Body.AWAKE &&
                bi.type !== Body.STATIC) {
                var speedSquaredA = bi.velocity.norm2() + bi.angularVelocity.norm2();
                var speedLimitSquaredA = Math.pow(bi.sleepSpeedLimit, 2);
                if (speedSquaredA >= speedLimitSquaredA * 2) {
                    bj._wakeUpAfterNarrowphase = true;
                }
            }
            this.collisionMatrix.set(bi, bj, true);
            if (!this.collisionMatrixPrevious.get(bi, bj)) {
                this.World_step_collideEvent.body = bj;
                this.World_step_collideEvent.contact = c;
                bi.dispatchEvent(this.World_step_collideEvent);
                this.World_step_collideEvent.body = bi;
                bj.dispatchEvent(this.World_step_collideEvent);
            }
            this.bodyOverlapKeeper.set(bi.id, bj.id);
            this.shapeOverlapKeeper.set(si.id, sj.id);
        }
        this.emitContactEvents();
        if (doProfiling) {
            profile.makeContactConstraints = performance.now() - profilingStart;
            profilingStart = performance.now();
        }
        for (i = 0; i !== N; i++) {
            var bi = bodies[i];
            if (bi._wakeUpAfterNarrowphase) {
                bi.wakeUp();
                bi._wakeUpAfterNarrowphase = false;
            }
        }
        var Nconstraints2 = constraints.length;
        for (i = 0; i !== Nconstraints2; i++) {
            var c = constraints[i];
            c.update();
            for (var j = 0, Neq = c.equations.length; j !== Neq; j++) {
                var eq = c.equations[j];
                solver.addEquation(eq);
            }
        }
        solver.solve(dt, this);
        if (doProfiling) {
            profile.solve = performance.now() - profilingStart;
        }
        solver.removeAllEquations();
        var pow = Math.pow;
        for (i = 0; i !== N; i++) {
            var bi = bodies[i];
            if (bi.type & DYNAMIC) {
                var ld = pow(1.0 - bi.linearDamping, dt);
                var v = bi.velocity;
                v.mult(ld, v);
                var av = bi.angularVelocity;
                if (av) {
                    var ad = pow(1.0 - bi.angularDamping, dt);
                    av.mult(ad, av);
                }
            }
        }
        this.dispatchEvent(this.World_step_preStepEvent);
        for (i = 0; i !== N; i++) {
            var bi = bodies[i];
            if (bi.preStep) {
                bi.preStep.call(bi);
            }
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        var stepnumber = this.stepnumber;
        var quatNormalize = stepnumber % (this.quatNormalizeSkip + 1) === 0;
        var quatNormalizeFast = this.quatNormalizeFast;
        for (i = 0; i !== N; i++) {
            bodies[i].integrate(dt, quatNormalize, quatNormalizeFast);
        }
        this.clearForces();
        this.broadphase.dirty = true;
        if (doProfiling) {
            profile.integrate = performance.now() - profilingStart;
        }
        this.time += dt;
        this.stepnumber += 1;
        this.dispatchEvent(this.World_step_postStepEvent);
        for (i = 0; i !== N; i++) {
            var bi = bodies[i];
            var postStep = bi.postStep;
            if (postStep) {
                postStep.call(bi);
            }
        }
        if (this.allowSleep) {
            for (i = 0; i !== N; i++) {
                bodies[i].sleepTick(this.time);
            }
        }
    };
    World.prototype.clearForces = function () {
        var bodies = this.bodies;
        var N = bodies.length;
        for (var i = 0; i !== N; i++) {
            var b = bodies[i], force = b.force, tau = b.torque;
            b.force.set(0, 0, 0);
            b.torque.set(0, 0, 0);
        }
    };
    return World;
}(EventTarget));
export { World };
