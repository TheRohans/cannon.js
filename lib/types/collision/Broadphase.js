import { Body } from '../objects/Body';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
var Broadphase = (function () {
    function Broadphase(world) {
        this.Broadphase_collisionPairs_r = new Vec3();
        this.Broadphase_collisionPairs_normal = new Vec3();
        this.Broadphase_collisionPairs_quat = new Quaternion();
        this.Broadphase_collisionPairs_relpos = new Vec3();
        this.Broadphase_makePairsUnique_p1 = [];
        this.Broadphase_makePairsUnique_p2 = [];
        this.bsc_dist = new Vec3();
        this.world = null;
        this.useBoundingBoxes = false;
        this.dirty = true;
    }
    Broadphase.prototype.collisionPairs = function (world, p1, p2) {
        throw new Error('collisionPairs not implemented for this BroadPhase class!');
    };
    Broadphase.prototype.needBroadphaseCollision = function (bodyA, bodyB) {
        if ((bodyA.collisionFilterGroup & bodyB.collisionFilterMask) === 0 || (bodyB.collisionFilterGroup & bodyA.collisionFilterMask) === 0) {
            return false;
        }
        if (((bodyA.type & Body.STATIC) !== 0 || bodyA.sleepState === Body.SLEEPING) &&
            ((bodyB.type & Body.STATIC) !== 0 || bodyB.sleepState === Body.SLEEPING)) {
            return false;
        }
        return true;
    };
    Broadphase.prototype.intersectionTest = function (bodyA, bodyB, pairs1, pairs2) {
        if (this.useBoundingBoxes) {
            this.doBoundingBoxBroadphase(bodyA, bodyB, pairs1, pairs2);
        }
        else {
            this.doBoundingSphereBroadphase(bodyA, bodyB, pairs1, pairs2);
        }
    };
    Broadphase.prototype.doBoundingSphereBroadphase = function (bodyA, bodyB, pairs1, pairs2) {
        var r = this.Broadphase_collisionPairs_r;
        bodyB.position.vsub(bodyA.position, r);
        var boundingRadiusSum2 = Math.pow(bodyA.boundingRadius + bodyB.boundingRadius, 2);
        var norm2 = r.norm2();
        if (norm2 < boundingRadiusSum2) {
            pairs1.push(bodyA);
            pairs2.push(bodyB);
        }
    };
    Broadphase.prototype.doBoundingBoxBroadphase = function (bodyA, bodyB, pairs1, pairs2) {
        if (bodyA.aabbNeedsUpdate) {
            bodyA.computeAABB();
        }
        if (bodyB.aabbNeedsUpdate) {
            bodyB.computeAABB();
        }
        if (bodyA.aabb.overlaps(bodyB.aabb)) {
            pairs1.push(bodyA);
            pairs2.push(bodyB);
        }
    };
    Broadphase.prototype.makePairsUnique = function (pairs1, pairs2) {
        var t = this.Broadphase_makePairsUnique_temp, p1 = this.Broadphase_makePairsUnique_p1, p2 = this.Broadphase_makePairsUnique_p2, N = pairs1.length;
        for (var i = 0; i !== N; i++) {
            p1[i] = pairs1[i];
            p2[i] = pairs2[i];
        }
        pairs1.length = 0;
        pairs2.length = 0;
        for (var i = 0; i !== N; i++) {
            var id1 = p1[i].id, id2 = p2[i].id;
            var key = id1 < id2 ? id1 + ',' + id2 : id2 + ',' + id1;
            t[key] = i;
            t.keys.push(key);
        }
        for (var i = 0; i !== t.keys.length; i++) {
            var key = t.keys.pop(), pairIndex = t[key];
            pairs1.push(p1[pairIndex]);
            pairs2.push(p2[pairIndex]);
            delete t[key];
        }
    };
    Broadphase.prototype.setWorld = function (world) { };
    Broadphase.boundingSphereCheck = function (bodyA, bodyB) {
        var dist = new Vec3();
        bodyA.position.vsub(bodyB.position, dist);
        var sa = bodyA.shapes[0];
        var sb = bodyB.shapes[0];
        return Math.pow(sa.boundingSphereRadius + sb.boundingSphereRadius, 2) > dist.norm2();
    };
    Broadphase.prototype.aabbQuery = function (world, aabb, result) {
        console.warn('.aabbQuery is not implemented in this Broadphase subclass.');
        return [];
    };
    return Broadphase;
}());
export { Broadphase };
