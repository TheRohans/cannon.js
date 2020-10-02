import { Vec3Pool } from '../utils/Vec3Pool';
import { Shape } from '../shapes/Shape';
import { Body } from '../objects/Body';
import { ContactEquation } from '../equations/ContactEquation';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { FrictionEquation } from '../equations/FrictionEquation';
var Narrowphase = (function () {
    function Narrowphase(world) {
        this.averageNormal = new Vec3();
        this.averageContactPointA = new Vec3();
        this.averageContactPointB = new Vec3();
        this.tmpVec1 = new Vec3();
        this.tmpVec2 = new Vec3();
        this.tmpQuat1 = new Quaternion();
        this.tmpQuat2 = new Quaternion();
        this.point_on_plane_to_sphere = new Vec3();
        this.plane_to_sphere_ortho = new Vec3();
        this.pointInPolygon_edge = new Vec3();
        this.pointInPolygon_edge_x_normal = new Vec3();
        this.pointInPolygon_vtp = new Vec3();
        this.box_to_sphere = new Vec3();
        this.sphereBox_ns = new Vec3();
        this.sphereBox_ns1 = new Vec3();
        this.sphereBox_ns2 = new Vec3();
        this.sphereBox_sides = [new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3()];
        this.sphereBox_sphere_to_corner = new Vec3();
        this.sphereBox_side_ns = new Vec3();
        this.sphereBox_side_ns1 = new Vec3();
        this.sphereBox_side_ns2 = new Vec3();
        this.convex_to_sphere = new Vec3();
        this.sphereConvex_edge = new Vec3();
        this.sphereConvex_edgeUnit = new Vec3();
        this.sphereConvex_sphereToCorner = new Vec3();
        this.sphereConvex_worldCorner = new Vec3();
        this.sphereConvex_worldNormal = new Vec3();
        this.sphereConvex_worldPoint = new Vec3();
        this.sphereConvex_worldSpherePointClosestToPlane = new Vec3();
        this.sphereConvex_penetrationVec = new Vec3();
        this.sphereConvex_sphereToWorldPoint = new Vec3();
        this.planeConvex_v = new Vec3();
        this.planeConvex_normal = new Vec3();
        this.planeConvex_relpos = new Vec3();
        this.planeConvex_projected = new Vec3();
        this.convexConvex_sepAxis = new Vec3();
        this.convexConvex_q = new Vec3();
        this.particlePlane_normal = new Vec3();
        this.particlePlane_relpos = new Vec3();
        this.particlePlane_projected = new Vec3();
        this.particleSphere_normal = new Vec3();
        this.cqj = new Quaternion();
        this.convexParticle_local = new Vec3();
        this.convexParticle_normal = new Vec3();
        this.convexParticle_penetratedFaceNormal = new Vec3();
        this.convexParticle_vertexToParticle = new Vec3();
        this.convexParticle_worldPenetrationVec = new Vec3();
        this.contactPointPool = [];
        this.frictionEquationPool = [];
        this.result = [];
        this.frictionResult = [];
        this.v3pool = new Vec3Pool();
        this.world = world;
        this.currentContactMaterial = world.defaultContactMaterial;
        this.enableFrictionReduction = false;
    }
    Narrowphase.prototype.createContactEquation = function (bi, bj, si, sj, overrideShapeA, overrideShapeB) {
        var c;
        if (this.contactPointPool.length) {
            c = this.contactPointPool.pop();
            c.bi = bi;
            c.bj = bj;
        }
        else {
            c = new ContactEquation(bi, bj);
        }
        c.enabled = bi.collisionResponse && bj.collisionResponse && si.collisionResponse && sj.collisionResponse;
        var cm = this.currentContactMaterial;
        c.restitution = cm.restitution;
        c.setSpookParams(cm.contactEquationStiffness, cm.contactEquationRelaxation, this.world.dt);
        var matA = si.material || bi.material;
        var matB = sj.material || bj.material;
        if (matA && matB && matA.restitution >= 0 && matB.restitution >= 0) {
            c.restitution = matA.restitution * matB.restitution;
        }
        c.si = overrideShapeA || si;
        c.sj = overrideShapeB || sj;
        return c;
    };
    Narrowphase.prototype.createFrictionEquationsFromContact = function (contactEquation, outArray) {
        var bodyA = contactEquation.bi;
        var bodyB = contactEquation.bj;
        var shapeA = contactEquation.si;
        var shapeB = contactEquation.sj;
        var world = this.world;
        var cm = this.currentContactMaterial;
        var friction = cm.friction;
        var matA = shapeA.material || bodyA.material;
        var matB = shapeB.material || bodyB.material;
        if (matA && matB && matA.friction >= 0 && matB.friction >= 0) {
            friction = matA.friction * matB.friction;
        }
        if (friction > 0) {
            var mug = friction * world.gravity.length();
            var reducedMass = (bodyA.invMass + bodyB.invMass);
            if (reducedMass > 0) {
                reducedMass = 1 / reducedMass;
            }
            var pool = this.frictionEquationPool;
            var c1 = pool.length ? pool.pop() : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
            var c2 = pool.length ? pool.pop() : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
            c1.bi = c2.bi = bodyA;
            c1.bj = c2.bj = bodyB;
            c1.minForce = c2.minForce = -mug * reducedMass;
            c1.maxForce = c2.maxForce = mug * reducedMass;
            c1.ri.copy(contactEquation.ri);
            c1.rj.copy(contactEquation.rj);
            c2.ri.copy(contactEquation.ri);
            c2.rj.copy(contactEquation.rj);
            contactEquation.ni.tangents(c1.t, c2.t);
            c1.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);
            c2.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);
            c1.enabled = c2.enabled = contactEquation.enabled;
            outArray.push(c1, c2);
            return true;
        }
        return false;
    };
    Narrowphase.prototype.createFrictionFromAverage = function (numContacts) {
        var c = this.result[this.result.length - 1];
        if (!this.createFrictionEquationsFromContact(c, this.frictionResult) || numContacts === 1) {
            return;
        }
        var f1 = this.frictionResult[this.frictionResult.length - 2];
        var f2 = this.frictionResult[this.frictionResult.length - 1];
        this.averageNormal.setZero();
        this.averageContactPointA.setZero();
        this.averageContactPointB.setZero();
        var bodyA = c.bi;
        var bodyB = c.bj;
        for (var i = 0; i !== numContacts; i++) {
            c = this.result[this.result.length - 1 - i];
            if (c.bi !== bodyA) {
                this.averageNormal.vadd(c.ni, this.averageNormal);
                this.averageContactPointA.vadd(c.ri, this.averageContactPointA);
                this.averageContactPointB.vadd(c.rj, this.averageContactPointB);
            }
            else {
                this.averageNormal.vsub(c.ni, this.averageNormal);
                this.averageContactPointA.vadd(c.rj, this.averageContactPointA);
                this.averageContactPointB.vadd(c.ri, this.averageContactPointB);
            }
        }
        var invNumContacts = 1 / numContacts;
        this.averageContactPointA.scale(invNumContacts, f1.ri);
        this.averageContactPointB.scale(invNumContacts, f1.rj);
        f2.ri.copy(f1.ri);
        f2.rj.copy(f1.rj);
        this.averageNormal.normalize();
        this.averageNormal.tangents(f1.t, f2.t);
    };
    Narrowphase.prototype.getContacts = function (p1, p2, world, result, oldcontacts, frictionResult, frictionPool) {
        this.contactPointPool = oldcontacts;
        this.frictionEquationPool = frictionPool;
        this.result = result;
        this.frictionResult = frictionResult;
        var qi = this.tmpQuat1;
        var qj = this.tmpQuat2;
        var xi = this.tmpVec1;
        var xj = this.tmpVec2;
        var N = p1.length;
        var k = N;
        while (k--) {
            var bi = p1[k], bj = p2[k];
            var bodyContactMaterial = null;
            if (bi.material && bj.material) {
                bodyContactMaterial = world.getContactMaterial(bi.material, bj.material) || null;
            }
            var justTest = !!(((bi.type & Body.KINEMATIC) && (bj.type & Body.STATIC))
                ||
                    ((bi.type & Body.STATIC) && (bj.type & Body.KINEMATIC))
                ||
                    ((bi.type & Body.KINEMATIC) && (bj.type & Body.KINEMATIC)));
            var i = bi.shapes.length;
            while (i--) {
                bi.quaternion.mult(bi.shapeOrientations[i], qi);
                bi.quaternion.vmult(bi.shapeOffsets[i], xi);
                xi.vadd(bi.position, xi);
                var si = bi.shapes[i];
                var j = bj.shapes.length;
                while (j--) {
                    bj.quaternion.mult(bj.shapeOrientations[j], qj);
                    bj.quaternion.vmult(bj.shapeOffsets[j], xj);
                    xj.vadd(bj.position, xj);
                    var sj = bj.shapes[j];
                    if (!((si.collisionFilterMask & sj.collisionFilterGroup)
                        && (sj.collisionFilterMask & si.collisionFilterGroup))) {
                        continue;
                    }
                    if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
                        continue;
                    }
                    var shapeContactMaterial = null;
                    if (si.material && sj.material) {
                        shapeContactMaterial = world.getContactMaterial(si.material, sj.material) || null;
                    }
                    this.currentContactMaterial = shapeContactMaterial
                        || bodyContactMaterial
                        || world.defaultContactMaterial;
                    var aFirst = (si.type < sj.type);
                    var retval = false;
                    switch (si.type | sj.type) {
                        case (Shape.types.CONVEXPOLYHEDRON | Shape.types.CONVEXPOLYHEDRON):
                            retval = (aFirst)
                                ? this.convexConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.convexConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.CONVEXPOLYHEDRON | Shape.types.PARTICLE):
                            retval = (aFirst)
                                ? this.convexParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.convexParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.BOX | Shape.types.CONVEXPOLYHEDRON):
                            retval = (aFirst)
                                ? this.boxConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.boxConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.SPHERE | Shape.types.CONVEXPOLYHEDRON):
                            retval = (aFirst)
                                ? this.sphereConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.sphereConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.BOX | Shape.types.BOX):
                            retval = (aFirst)
                                ? this.boxBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.boxBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.BOX | Shape.types.PARTICLE):
                            retval = (aFirst)
                                ? this.boxParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.boxParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.SPHERE | Shape.types.BOX):
                            retval = (aFirst)
                                ? this.sphereBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.sphereBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.SPHERE | Shape.types.SPHERE):
                            retval = (aFirst)
                                ? this.sphereSphere(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.sphereSphere(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.SPHERE | Shape.types.PLANE):
                            retval = (aFirst)
                                ? this.spherePlane(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.spherePlane(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.SPHERE | Shape.types.PARTICLE):
                            retval = (aFirst)
                                ? this.sphereParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.sphereParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.PLANE | Shape.types.BOX):
                            retval = (aFirst)
                                ? this.planeBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.planeBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.PLANE | Shape.types.CONVEXPOLYHEDRON):
                            retval = (aFirst)
                                ? this.planeConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.planeConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                        case (Shape.types.PLANE | Shape.types.PARTICLE):
                            retval = (aFirst)
                                ? this.planeParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                                : this.planeParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
                            break;
                    }
                    if (retval && justTest) {
                        world.shapeOverlapKeeper.set(si.id, sj.id);
                        world.bodyOverlapKeeper.set(bi.id, bj.id);
                    }
                }
            }
        }
    };
    Narrowphase.prototype.boxBox = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        sj.convexPolyhedronRepresentation.material = sj.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
        return this.convexConvex(si.convexPolyhedronRepresentation, sj.convexPolyhedronRepresentation, xi, xj, qi, qj, bi, bj, si, sj, justTest);
    };
    Narrowphase.prototype.boxConvex = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        return this.convexConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
    };
    Narrowphase.prototype.boxParticle = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        return this.convexParticle(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
    };
    Narrowphase.prototype.sphereSphere = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var ssi = si;
        var ssj = sj;
        if (justTest) {
            return xi.distanceSquared(xj) < Math.pow(ssi.radius + ssj.radius, 2);
        }
        var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        xj.vsub(xi, r.ni);
        r.ni.normalize();
        r.ri.copy(r.ni);
        r.rj.copy(r.ni);
        r.ri.mult(ssi.radius, r.ri);
        r.rj.mult(-ssj.radius, r.rj);
        r.ri.vadd(xi, r.ri);
        r.ri.vsub(bi.position, r.ri);
        r.rj.vadd(xj, r.rj);
        r.rj.vsub(bj.position, r.rj);
        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
        return false;
    };
    Narrowphase.prototype.spherePlane = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ni.set(0, 0, 1);
        qj.vmult(r.ni, r.ni);
        r.ni.negate(r.ni);
        r.ni.normalize();
        var ssi = si;
        var psj = sj;
        r.ni.mult(ssi.radius, r.ri);
        xi.vsub(xj, this.point_on_plane_to_sphere);
        r.ni.mult(r.ni.dot(this.point_on_plane_to_sphere), this.plane_to_sphere_ortho);
        this.point_on_plane_to_sphere.vsub(this.plane_to_sphere_ortho, r.rj);
        if (-this.point_on_plane_to_sphere.dot(r.ni) <= ssi.radius) {
            if (justTest) {
                return true;
            }
            var ri = r.ri;
            var rj = r.rj;
            ri.vadd(xi, ri);
            ri.vsub(bi.position, ri);
            rj.vadd(xj, rj);
            rj.vsub(bj.position, rj);
            this.result.push(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
        return false;
    };
    Narrowphase.prototype.pointInPolygon = function (verts, normal, p) {
        var positiveResult = null;
        var N = verts.length;
        for (var i = 0; i !== N; i++) {
            var v = verts[i];
            var edge = this.pointInPolygon_edge;
            verts[(i + 1) % (N)].vsub(v, edge);
            var edge_x_normal = this.pointInPolygon_edge_x_normal;
            edge.cross(normal, edge_x_normal);
            var vertex_to_p = this.pointInPolygon_vtp;
            p.vsub(v, vertex_to_p);
            var r = edge_x_normal.dot(vertex_to_p);
            if (positiveResult === null || (r > 0 && positiveResult === true) || (r <= 0 && positiveResult === false)) {
                if (positiveResult === null) {
                    positiveResult = r > 0;
                }
                continue;
            }
            else {
                return false;
            }
        }
        return true;
    };
    Narrowphase.prototype.sphereBox = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var v3pool = this.v3pool;
        var ssi = si;
        var bsj = sj;
        var sides = this.sphereBox_sides;
        xi.vsub(xj, this.box_to_sphere);
        bsj.getSideNormals(sides, qj);
        var R = ssi.radius;
        var penetrating_sides = [];
        var found = false;
        var side_ns = this.sphereBox_side_ns;
        var side_ns1 = this.sphereBox_side_ns1;
        var side_ns2 = this.sphereBox_side_ns2;
        var side_h = null;
        var side_penetrations = 0;
        var side_dot1 = 0;
        var side_dot2 = 0;
        var side_distance = null;
        for (var idx = 0, nsides = sides.length; idx !== nsides && found === false; idx++) {
            var ns = this.sphereBox_ns;
            ns.copy(sides[idx]);
            var h = ns.norm();
            ns.normalize();
            var dot = this.box_to_sphere.dot(ns);
            if (dot < h + R && dot > 0) {
                var ns1 = this.sphereBox_ns1;
                var ns2 = this.sphereBox_ns2;
                ns1.copy(sides[(idx + 1) % 3]);
                ns2.copy(sides[(idx + 2) % 3]);
                var h1 = ns1.norm();
                var h2 = ns2.norm();
                ns1.normalize();
                ns2.normalize();
                var dot1 = this.box_to_sphere.dot(ns1);
                var dot2 = this.box_to_sphere.dot(ns2);
                if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
                    var dist1 = Math.abs(dot - h - R);
                    if (side_distance === null || dist1 < side_distance) {
                        side_distance = dist1;
                        side_dot1 = dot1;
                        side_dot2 = dot2;
                        side_h = h;
                        side_ns.copy(ns);
                        side_ns1.copy(ns1);
                        side_ns2.copy(ns2);
                        side_penetrations++;
                        if (justTest) {
                            return true;
                        }
                    }
                }
            }
        }
        if (side_penetrations) {
            found = true;
            var r1 = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            side_ns.mult(-R, r1.ri);
            r1.ni.copy(side_ns);
            r1.ni.negate(r1.ni);
            side_ns.mult(side_h, side_ns);
            side_ns1.mult(side_dot1, side_ns1);
            side_ns.vadd(side_ns1, side_ns);
            side_ns2.mult(side_dot2, side_ns2);
            side_ns.vadd(side_ns2, r1.rj);
            r1.ri.vadd(xi, r1.ri);
            r1.ri.vsub(bi.position, r1.ri);
            r1.rj.vadd(xj, r1.rj);
            r1.rj.vsub(bj.position, r1.rj);
            this.result.push(r1);
            this.createFrictionEquationsFromContact(r1, this.frictionResult);
        }
        var rj = v3pool.get();
        var sphere_to_corner = this.sphereBox_sphere_to_corner;
        for (var j = 0; j !== 2 && !found; j++) {
            for (var k = 0; k !== 2 && !found; k++) {
                for (var l = 0; l !== 2 && !found; l++) {
                    rj.set(0, 0, 0);
                    if (j) {
                        rj.vadd(sides[0], rj);
                    }
                    else {
                        rj.vsub(sides[0], rj);
                    }
                    if (k) {
                        rj.vadd(sides[1], rj);
                    }
                    else {
                        rj.vsub(sides[1], rj);
                    }
                    if (l) {
                        rj.vadd(sides[2], rj);
                    }
                    else {
                        rj.vsub(sides[2], rj);
                    }
                    xj.vadd(rj, sphere_to_corner);
                    sphere_to_corner.vsub(xi, sphere_to_corner);
                    if (sphere_to_corner.norm2() < R * R) {
                        if (justTest) {
                            return true;
                        }
                        found = true;
                        var r1 = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                        r1.ri.copy(sphere_to_corner);
                        r1.ri.normalize();
                        r1.ni.copy(r1.ri);
                        r1.ri.mult(R, r1.ri);
                        r1.rj.copy(rj);
                        r1.ri.vadd(xi, r1.ri);
                        r1.ri.vsub(bi.position, r1.ri);
                        r1.rj.vadd(xj, r1.rj);
                        r1.rj.vsub(bj.position, r1.rj);
                        this.result.push(r1);
                        this.createFrictionEquationsFromContact(r1, this.frictionResult);
                    }
                }
            }
        }
        v3pool.release(rj);
        rj = null;
        var edgeTangent = v3pool.get();
        var edgeCenter = v3pool.get();
        var r = v3pool.get();
        var orthogonal = v3pool.get();
        var dist = v3pool.get();
        var Nsides = sides.length;
        for (var j = 0; j !== Nsides && !found; j++) {
            for (var k = 0; k !== Nsides && !found; k++) {
                if (j % 3 !== k % 3) {
                    sides[k].cross(sides[j], edgeTangent);
                    edgeTangent.normalize();
                    sides[j].vadd(sides[k], edgeCenter);
                    r.copy(xi);
                    r.vsub(edgeCenter, r);
                    r.vsub(xj, r);
                    var orthonorm = r.dot(edgeTangent);
                    edgeTangent.mult(orthonorm, orthogonal);
                    var l = 0;
                    while (l === j % 3 || l === k % 3) {
                        l++;
                    }
                    dist.copy(xi);
                    dist.vsub(orthogonal, dist);
                    dist.vsub(edgeCenter, dist);
                    dist.vsub(xj, dist);
                    var tdist = Math.abs(orthonorm);
                    var ndist = dist.norm();
                    if (tdist < sides[l].norm() && ndist < R) {
                        if (justTest) {
                            return true;
                        }
                        found = true;
                        var res = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                        edgeCenter.vadd(orthogonal, res.rj);
                        res.rj.copy(res.rj);
                        dist.negate(res.ni);
                        res.ni.normalize();
                        res.ri.copy(res.rj);
                        res.ri.vadd(xj, res.ri);
                        res.ri.vsub(xi, res.ri);
                        res.ri.normalize();
                        res.ri.mult(R, res.ri);
                        res.ri.vadd(xi, res.ri);
                        res.ri.vsub(bi.position, res.ri);
                        res.rj.vadd(xj, res.rj);
                        res.rj.vsub(bj.position, res.rj);
                        this.result.push(res);
                        this.createFrictionEquationsFromContact(res, this.frictionResult);
                    }
                }
            }
        }
        v3pool.release(edgeTangent, edgeCenter, r, orthogonal, dist);
        return false;
    };
    Narrowphase.prototype.sphereConvex = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var v3pool = this.v3pool;
        var ssi = si;
        var csj = sj;
        xi.vsub(xj, this.convex_to_sphere);
        var normals = csj.faceNormals;
        var faces = csj.faces;
        var verts = csj.vertices;
        var R = ssi.radius;
        var i = verts.length;
        while (i--) {
            var v = verts[i];
            var worldCorner = this.sphereConvex_worldCorner;
            qj.vmult(v, worldCorner);
            xj.vadd(worldCorner, worldCorner);
            var sphere_to_corner = this.sphereConvex_sphereToCorner;
            worldCorner.vsub(xi, sphere_to_corner);
            if (sphere_to_corner.norm2() < R * R) {
                if (justTest) {
                    return true;
                }
                var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                r.ri.copy(sphere_to_corner);
                r.ri.normalize();
                r.ni.copy(r.ri);
                r.ri.mult(R, r.ri);
                worldCorner.vsub(xj, r.rj);
                r.ri.vadd(xi, r.ri);
                r.ri.vsub(bi.position, r.ri);
                r.rj.vadd(xj, r.rj);
                r.rj.vsub(bj.position, r.rj);
                this.result.push(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
                return false;
            }
        }
        var found = false;
        var nfaces = faces.length;
        i = nfaces;
        while (i-- && found === false) {
            var normal = normals[i];
            var face = faces[i];
            var worldNormal = this.sphereConvex_worldNormal;
            qj.vmult(normal, worldNormal);
            var worldPoint = this.sphereConvex_worldPoint;
            qj.vmult(verts[face[0]], worldPoint);
            worldPoint.vadd(xj, worldPoint);
            var worldSpherePointClosestToPlane = this.sphereConvex_worldSpherePointClosestToPlane;
            worldNormal.mult(-R, worldSpherePointClosestToPlane);
            xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);
            var penetrationVec = this.sphereConvex_penetrationVec;
            worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);
            var penetration = penetrationVec.dot(worldNormal);
            var worldPointToSphere = this.sphereConvex_sphereToWorldPoint;
            xi.vsub(worldPoint, worldPointToSphere);
            if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
                var faceVerts = [];
                var Nverts = face.length;
                var j = Nverts;
                while (j--) {
                    var worldVertex = v3pool.get();
                    qj.vmult(verts[face[j]], worldVertex);
                    xj.vadd(worldVertex, worldVertex);
                    faceVerts.push(worldVertex);
                }
                if (this.pointInPolygon(faceVerts, worldNormal, xi)) {
                    if (justTest) {
                        return true;
                    }
                    found = true;
                    var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                    worldNormal.mult(-R, r.ri);
                    worldNormal.negate(r.ni);
                    var penetrationVec2 = v3pool.get();
                    worldNormal.mult(-penetration, penetrationVec2);
                    var penetrationSpherePoint = v3pool.get();
                    worldNormal.mult(-R, penetrationSpherePoint);
                    xi.vsub(xj, r.rj);
                    r.rj.vadd(penetrationSpherePoint, r.rj);
                    r.rj.vadd(penetrationVec2, r.rj);
                    r.rj.vadd(xj, r.rj);
                    r.rj.vsub(bj.position, r.rj);
                    r.ri.vadd(xi, r.ri);
                    r.ri.vsub(bi.position, r.ri);
                    v3pool.release(penetrationVec2);
                    v3pool.release(penetrationSpherePoint);
                    this.result.push(r);
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                    j = faceVerts.length;
                    while (j--) {
                        v3pool.release(faceVerts[j]);
                    }
                    return false;
                }
                else {
                    j = face.length;
                    while (j--) {
                        var v1 = v3pool.get();
                        var v2 = v3pool.get();
                        qj.vmult(verts[face[(j + 1) % face.length]], v1);
                        qj.vmult(verts[face[(j + 2) % face.length]], v2);
                        xj.vadd(v1, v1);
                        xj.vadd(v2, v2);
                        var edge = this.sphereConvex_edge;
                        v2.vsub(v1, edge);
                        var edgeUnit = this.sphereConvex_edgeUnit;
                        edge.unit(edgeUnit);
                        var p = v3pool.get();
                        var v1_to_xi = v3pool.get();
                        xi.vsub(v1, v1_to_xi);
                        var dot = v1_to_xi.dot(edgeUnit);
                        edgeUnit.mult(dot, p);
                        p.vadd(v1, p);
                        var xi_to_p = v3pool.get();
                        p.vsub(xi, xi_to_p);
                        if (dot > 0 && dot * dot < edge.norm2() && xi_to_p.norm2() < R * R) {
                            if (justTest) {
                                return true;
                            }
                            var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                            p.vsub(xj, r.rj);
                            p.vsub(xi, r.ni);
                            r.ni.normalize();
                            r.ni.mult(R, r.ri);
                            r.rj.vadd(xj, r.rj);
                            r.rj.vsub(bj.position, r.rj);
                            r.ri.vadd(xi, r.ri);
                            r.ri.vsub(bi.position, r.ri);
                            this.result.push(r);
                            this.createFrictionEquationsFromContact(r, this.frictionResult);
                            var jj = faceVerts.length;
                            while (jj--) {
                                v3pool.release(faceVerts[jj]);
                            }
                            v3pool.release(v1);
                            v3pool.release(v2);
                            v3pool.release(p);
                            v3pool.release(xi_to_p);
                            v3pool.release(v1_to_xi);
                            return false;
                        }
                        v3pool.release(v1);
                        v3pool.release(v2);
                        v3pool.release(p);
                        v3pool.release(xi_to_p);
                        v3pool.release(v1_to_xi);
                    }
                }
                j = faceVerts.length;
                while (j--) {
                    v3pool.release(faceVerts[j]);
                }
            }
        }
        return false;
    };
    Narrowphase.prototype.planeBox = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var bsj = sj;
        bsj.convexPolyhedronRepresentation.material = sj.material;
        bsj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
        bsj.convexPolyhedronRepresentation.id = sj.id;
        return this.planeConvex(si, bsj.convexPolyhedronRepresentation, xi, xj, qi, qj, bi, bj, si, sj, justTest);
    };
    Narrowphase.prototype.planeConvex = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var planeShape = si;
        var convexShape = sj;
        var planePosition = xi;
        var convexPosition = xj;
        var planeQuat = qi;
        var convexQuat = qj;
        var planeBody = bi;
        var convexBody = bj;
        var worldVertex = this.planeConvex_v, worldNormal = this.planeConvex_normal;
        worldNormal.set(0, 0, 1);
        planeQuat.vmult(worldNormal, worldNormal);
        var numContacts = 0;
        var relpos = this.planeConvex_relpos;
        for (var i = 0; i !== convexShape.vertices.length; i++) {
            worldVertex.copy(convexShape.vertices[i]);
            convexQuat.vmult(worldVertex, worldVertex);
            convexPosition.vadd(worldVertex, worldVertex);
            worldVertex.vsub(planePosition, relpos);
            var dot = worldNormal.dot(relpos);
            if (dot <= 0.0) {
                if (justTest) {
                    return true;
                }
                var r = this.createContactEquation(planeBody, convexBody, planeShape, convexShape, rsi, rsj);
                var projected = this.planeConvex_projected;
                worldNormal.mult(worldNormal.dot(relpos), projected);
                worldVertex.vsub(projected, projected);
                projected.vsub(planePosition, r.ri);
                r.ni.copy(worldNormal);
                worldVertex.vsub(convexPosition, r.rj);
                r.ri.vadd(planePosition, r.ri);
                r.ri.vsub(planeBody.position, r.ri);
                r.rj.vadd(convexPosition, r.rj);
                r.rj.vsub(convexBody.position, r.rj);
                this.result.push(r);
                numContacts++;
                if (!this.enableFrictionReduction) {
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                }
            }
        }
        if (this.enableFrictionReduction && numContacts) {
            this.createFrictionFromAverage(numContacts);
        }
        return false;
    };
    Narrowphase.prototype.convexConvex = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest, faceListA, faceListB) {
        var sepAxis = this.convexConvex_sepAxis;
        if (xi.distanceTo(xj) > (si.boundingSphereRadius + sj.boundingSphereRadius)) {
            return false;
        }
        var cpsi = si;
        var numContacts = 0;
        if (cpsi.findSeparatingAxis(sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)) {
            var res = [];
            var q = this.convexConvex_q;
            cpsi.clipAgainstHull(xi, qi, sj, xj, qj, sepAxis, -100, 100, res);
            var j = res.length;
            while (j--) {
                if (justTest) {
                    return true;
                }
                var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj), ri = r.ri, rj = r.rj;
                sepAxis.negate(r.ni);
                res[j].normal.negate(q);
                q.mult(res[j].depth, q);
                res[j].point.vadd(q, ri);
                rj.copy(res[j].point);
                ri.vsub(xi, ri);
                rj.vsub(xj, rj);
                ri.vadd(xi, ri);
                ri.vsub(bi.position, ri);
                rj.vadd(xj, rj);
                rj.vsub(bj.position, rj);
                this.result.push(r);
                numContacts++;
                if (!this.enableFrictionReduction) {
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                }
            }
            if (this.enableFrictionReduction && numContacts) {
                this.createFrictionFromAverage(numContacts);
            }
        }
        return numContacts > 0;
    };
    Narrowphase.prototype.planeParticle = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var normal = this.particlePlane_normal;
        normal.set(0, 0, 1);
        bj.quaternion.vmult(normal, normal);
        var relpos = this.particlePlane_relpos;
        xi.vsub(bj.position, relpos);
        var dot = normal.dot(relpos);
        if (dot <= 0.0) {
            if (justTest) {
                return true;
            }
            var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ni.copy(normal);
            r.ni.negate(r.ni);
            r.ri.set(0, 0, 0);
            var projected = this.particlePlane_projected;
            normal.mult(normal.dot(xi), projected);
            xi.vsub(projected, projected);
            r.rj.copy(projected);
            this.result.push(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
        return false;
    };
    Narrowphase.prototype.sphereParticle = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var psi = si;
        var ssj = sj;
        var normal = this.particleSphere_normal;
        normal.set(0, 0, 1);
        xi.vsub(xj, normal);
        var lengthSquared = normal.norm2();
        if (lengthSquared <= ssj.radius * ssj.radius) {
            if (justTest) {
                return true;
            }
            var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            normal.normalize();
            r.rj.copy(normal);
            r.rj.mult(ssj.radius, r.rj);
            r.ni.copy(normal);
            r.ni.negate(r.ni);
            r.ri.set(0, 0, 0);
            this.result.push(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
        return false;
    };
    Narrowphase.prototype.convexParticle = function (si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
        var penetratedFaceIndex = -1;
        var penetratedFaceNormal = this.convexParticle_penetratedFaceNormal;
        var worldPenetrationVec = this.convexParticle_worldPenetrationVec;
        var minPenetration = null;
        var numDetectedFaces = 0;
        var csj = sj;
        var local = this.convexParticle_local;
        local.copy(xi);
        local.vsub(xj, local);
        qj.conjugate(this.cqj);
        this.cqj.vmult(local, local);
        if (csj.pointIsInside(local)) {
            if (csj.worldVerticesNeedsUpdate) {
                csj.computeWorldVertices(xj, qj);
            }
            if (csj.worldFaceNormalsNeedsUpdate) {
                csj.computeWorldFaceNormals(qj);
            }
            for (var i = 0, nfaces = csj.faces.length; i !== nfaces; i++) {
                var verts = [csj.worldVertices[csj.faces[i][0]]];
                var normal = csj.worldFaceNormals[i];
                xi.vsub(verts[0], this.convexParticle_vertexToParticle);
                var penetration = -normal.dot(this.convexParticle_vertexToParticle);
                if (minPenetration === null || Math.abs(penetration) < Math.abs(minPenetration)) {
                    if (justTest) {
                        return true;
                    }
                    minPenetration = penetration;
                    penetratedFaceIndex = i;
                    penetratedFaceNormal.copy(normal);
                    numDetectedFaces++;
                }
            }
            if (penetratedFaceIndex !== -1) {
                var r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                penetratedFaceNormal.mult(minPenetration, worldPenetrationVec);
                worldPenetrationVec.vadd(xi, worldPenetrationVec);
                worldPenetrationVec.vsub(xj, worldPenetrationVec);
                r.rj.copy(worldPenetrationVec);
                penetratedFaceNormal.negate(r.ni);
                r.ri.set(0, 0, 0);
                var ri = r.ri, rj = r.rj;
                ri.vadd(xi, ri);
                ri.vsub(bi.position, ri);
                rj.vadd(xj, rj);
                rj.vsub(bj.position, rj);
                this.result.push(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
            }
            else {
                console.warn('Point found inside convex, but did not find penetrating face!');
            }
        }
        return false;
    };
    return Narrowphase;
}());
export { Narrowphase };
