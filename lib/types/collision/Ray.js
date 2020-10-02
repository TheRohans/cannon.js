import { Vec3 } from '../math/Vec3';
import { RaycastResult } from './RaycastResult';
import { Quaternion } from '../math/Quaternion';
import { Shape } from '../shapes/Shape';
import { AABB } from './AABB';
var Ray = (function () {
    function Ray(from, to) {
        this.tmpAABB = new AABB();
        this.tmpArray = [];
        this.v1 = new Vec3();
        this.v2 = new Vec3();
        this.intersectBody_xi = new Vec3();
        this.intersectBody_qi = new Quaternion();
        this.vector = new Vec3();
        this.normal = new Vec3();
        this.intersectPoint = new Vec3();
        this.a = new Vec3();
        this.b = new Vec3();
        this.c = new Vec3();
        this.d = new Vec3();
        this.tmpRaycastResult = new RaycastResult();
        this.Ray_intersectSphere_intersectionPoint = new Vec3();
        this.Ray_intersectSphere_normal = new Vec3();
        this.intersectConvex_normal = new Vec3();
        this.intersectConvex_minDistNormal = new Vec3();
        this.intersectConvex_minDistIntersect = new Vec3();
        this.intersectConvex_vector = new Vec3();
        this.v0 = new Vec3();
        this.intersect = new Vec3();
        this.from = from ? from.clone() : new Vec3();
        this.to = to ? to.clone() : new Vec3();
        this._direction = new Vec3();
        this.precision = 0.0001;
        this.checkCollisionResponse = true;
        this.skipBackfaces = false;
        this.collisionFilterMask = -1;
        this.collisionFilterGroup = -1;
        this.mode = Ray.ANY;
        this.result = new RaycastResult();
        this.hasHit = false;
        this.callback = function (result) { };
    }
    Ray.prototype.intersectWorld = function (world, options) {
        this.mode = options.mode || Ray.ANY;
        this.result = options.result || new RaycastResult();
        this.skipBackfaces = !!options.skipBackfaces;
        this.collisionFilterMask = typeof (options.collisionFilterMask) !== 'undefined' ? options.collisionFilterMask : -1;
        this.collisionFilterGroup = typeof (options.collisionFilterGroup) !== 'undefined' ? options.collisionFilterGroup : -1;
        if (options.from) {
            this.from.copy(options.from);
        }
        if (options.to) {
            this.to.copy(options.to);
        }
        this.callback = options.callback || (function () { });
        this.hasHit = false;
        this.result.reset();
        this.updateDirection();
        this.getAABB(this.tmpAABB);
        this.tmpArray.length = 0;
        world.broadphase.aabbQuery(world, this.tmpAABB, this.tmpArray);
        this.intersectBodies(this.tmpArray);
        return this.hasHit;
    };
    Ray.pointInTriangle = function (p, a, b, c) {
        var v0 = new Vec3();
        var v1 = new Vec3();
        var v2 = new Vec3();
        c.vsub(a, v0);
        b.vsub(a, v1);
        p.vsub(a, v2);
        var dot00 = v0.dot(v0);
        var dot01 = v0.dot(v1);
        var dot02 = v0.dot(v2);
        var dot11 = v1.dot(v1);
        var dot12 = v1.dot(v2);
        var u, v;
        return ((u = dot11 * dot02 - dot01 * dot12) >= 0) &&
            ((v = dot00 * dot12 - dot01 * dot02) >= 0) &&
            (u + v < (dot00 * dot11 - dot01 * dot01));
    };
    Ray.prototype.intersectBody = function (body, result) {
        if (result) {
            this.result = result;
            this.updateDirection();
        }
        var checkCollisionResponse = this.checkCollisionResponse;
        if (checkCollisionResponse && !body.collisionResponse) {
            return;
        }
        if ((this.collisionFilterGroup & body.collisionFilterMask) === 0 || (body.collisionFilterGroup & this.collisionFilterMask) === 0) {
            return;
        }
        var xi = this.intersectBody_xi;
        var qi = this.intersectBody_qi;
        for (var i = 0, N = body.shapes.length; i < N; i++) {
            var shape = body.shapes[i];
            if (checkCollisionResponse && !shape.collisionResponse) {
                continue;
            }
            body.quaternion.mult(body.shapeOrientations[i], qi);
            body.quaternion.vmult(body.shapeOffsets[i], xi);
            xi.vadd(body.position, xi);
            this.intersectShape(shape, qi, xi, body);
            if (this.result._shouldStop) {
                break;
            }
        }
    };
    Ray.prototype.intersectBodies = function (bodies, result) {
        if (result) {
            this.result = result;
            this.updateDirection();
        }
        for (var i = 0, l = bodies.length; !this.result._shouldStop && i < l; i++) {
            this.intersectBody(bodies[i]);
        }
    };
    Ray.prototype.updateDirection = function () {
        this.to.vsub(this.from, this._direction);
        this._direction.normalize();
    };
    Ray.prototype.intersectShape = function (shape, quat, position, body) {
        var from = this.from;
        var distance = this.distanceFromIntersection(from, this._direction, position);
        if (distance > shape.boundingSphereRadius) {
            return;
        }
        switch (shape.type) {
            case Shape.types.CONVEXPOLYHEDRON:
                this.intersectConvex(shape, quat, position, body, shape);
                break;
            case Shape.types.BOX:
                this.intersectBox(shape, quat, position, body, shape);
                break;
            case Shape.types.SPHERE:
                this.intersectSphere(shape, quat, position, body, shape);
                break;
            case Shape.types.PLANE:
                this.intersectPlane(shape, quat, position, body, shape);
                break;
        }
    };
    Ray.prototype.intersectBox = function (shape, quat, position, body, reportedShape) {
        return this.intersectConvex(shape.convexPolyhedronRepresentation, quat, position, body, reportedShape);
    };
    Ray.prototype.intersectPlane = function (shape, quat, position, body, reportedShape) {
        var from = this.from;
        var to = this.to;
        var direction = this._direction;
        var worldNormal = new Vec3(0, 0, 1);
        quat.vmult(worldNormal, worldNormal);
        var len = new Vec3();
        from.vsub(position, len);
        var planeToFrom = len.dot(worldNormal);
        to.vsub(position, len);
        var planeToTo = len.dot(worldNormal);
        if (planeToFrom * planeToTo > 0) {
            return;
        }
        if (from.distanceTo(to) < planeToFrom) {
            return;
        }
        var n_dot_dir = worldNormal.dot(direction);
        if (Math.abs(n_dot_dir) < this.precision) {
            return;
        }
        var planePointToFrom = new Vec3();
        var dir_scaled_with_t = new Vec3();
        var hitPointWorld = new Vec3();
        from.vsub(position, planePointToFrom);
        var t = -worldNormal.dot(planePointToFrom) / n_dot_dir;
        direction.scale(t, dir_scaled_with_t);
        from.vadd(dir_scaled_with_t, hitPointWorld);
        this.reportIntersection(worldNormal, hitPointWorld, reportedShape, body, -1);
    };
    Ray.prototype.getAABB = function (result) {
        var to = this.to;
        var from = this.from;
        result.lowerBound.x = Math.min(to.x, from.x);
        result.lowerBound.y = Math.min(to.y, from.y);
        result.lowerBound.z = Math.min(to.z, from.z);
        result.upperBound.x = Math.max(to.x, from.x);
        result.upperBound.y = Math.max(to.y, from.y);
        result.upperBound.z = Math.max(to.z, from.z);
    };
    Ray.prototype.intersectSphere = function (shape, quat, position, body, reportedShape) {
        var from = this.from;
        var to = this.to;
        var r = shape.radius;
        var a = Math.pow(to.x - from.x, 2) + Math.pow(to.y - from.y, 2) + Math.pow(to.z - from.z, 2);
        var b = 2 * ((to.x - from.x) * (from.x - position.x)
            + (to.y - from.y) * (from.y - position.y) + (to.z - from.z) * (from.z - position.z));
        var c = Math.pow(from.x - position.x, 2) + Math.pow(from.y - position.y, 2) + Math.pow(from.z - position.z, 2) - Math.pow(r, 2);
        var delta = Math.pow(b, 2) - 4 * a * c;
        var intersectionPoint = this.Ray_intersectSphere_intersectionPoint;
        var normal = this.Ray_intersectSphere_normal;
        if (delta < 0) {
            return;
        }
        else if (delta === 0) {
            from.lerp(to, delta, intersectionPoint);
            intersectionPoint.vsub(position, normal);
            normal.normalize();
            this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
        }
        else {
            var d1 = (-b - Math.sqrt(delta)) / (2 * a);
            var d2 = (-b + Math.sqrt(delta)) / (2 * a);
            if (d1 >= 0 && d1 <= 1) {
                from.lerp(to, d1, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
            }
            if (this.result._shouldStop) {
                return;
            }
            if (d2 >= 0 && d2 <= 1) {
                from.lerp(to, d2, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
            }
        }
    };
    Ray.prototype.intersectConvex = function (shape, quat, position, body, reportedShape, options) {
        var minDistNormal = this.intersectConvex_minDistNormal;
        var normal = this.intersectConvex_normal;
        var vector = this.intersectConvex_vector;
        var minDistIntersect = this.intersectConvex_minDistIntersect;
        var faceList = (options && options.faceList) || null;
        var faces = shape.faces;
        var vertices = shape.vertices;
        var normals = shape.faceNormals;
        var direction = this._direction;
        var from = this.from;
        var to = this.to;
        var fromToDistance = from.distanceTo(to);
        var minDist = -1;
        var Nfaces = faceList ? faceList.length : faces.length;
        var result = this.result;
        for (var j = 0; !result._shouldStop && j < Nfaces; j++) {
            var fi = faceList ? faceList[j] : j;
            var face = faces[fi];
            var faceNormal = normals[fi];
            var q = quat;
            var x = position;
            vector.copy(vertices[face[0]]);
            q.vmult(vector, vector);
            vector.vadd(x, vector);
            vector.vsub(from, vector);
            q.vmult(faceNormal, normal);
            var dot = direction.dot(normal);
            if (Math.abs(dot) < this.precision) {
                continue;
            }
            var scalar = normal.dot(vector) / dot;
            if (scalar < 0) {
                continue;
            }
            direction.mult(scalar, this.intersectPoint);
            this.intersectPoint.vadd(from, this.intersectPoint);
            this.a.copy(vertices[face[0]]);
            q.vmult(this.a, this.a);
            x.vadd(this.a, this.a);
            for (var i = 1; !result._shouldStop && i < face.length - 1; i++) {
                this.b.copy(vertices[face[i]]);
                this.c.copy(vertices[face[i + 1]]);
                q.vmult(this.b, this.b);
                q.vmult(this.c, this.c);
                x.vadd(this.b, this.b);
                x.vadd(this.c, this.c);
                var distance = this.intersectPoint.distanceTo(from);
                if (!(Ray.pointInTriangle(this.intersectPoint, this.a, this.b, this.c)
                    || Ray.pointInTriangle(this.intersectPoint, this.b, this.a, this.c))
                    || distance > fromToDistance) {
                    continue;
                }
                this.reportIntersection(normal, this.intersectPoint, reportedShape, body, fi);
            }
        }
    };
    Ray.prototype.reportIntersection = function (normal, hitPointWorld, shape, body, hitFaceIndex) {
        var from = this.from;
        var to = this.to;
        var distance = from.distanceTo(hitPointWorld);
        var result = this.result;
        if (this.skipBackfaces && normal.dot(this._direction) > 0) {
            return;
        }
        result.hitFaceIndex = typeof (hitFaceIndex) !== 'undefined' ? hitFaceIndex : -1;
        switch (this.mode) {
            case Ray.ALL:
                this.hasHit = true;
                result.set(from, to, normal, hitPointWorld, shape, body, distance);
                result.hasHit = true;
                this.callback(result);
                break;
            case Ray.CLOSEST:
                if (distance < result.distance || !result.hasHit) {
                    this.hasHit = true;
                    result.hasHit = true;
                    result.set(from, to, normal, hitPointWorld, shape, body, distance);
                }
                break;
            case Ray.ANY:
                this.hasHit = true;
                result.hasHit = true;
                result.set(from, to, normal, hitPointWorld, shape, body, distance);
                result._shouldStop = true;
                break;
        }
    };
    Ray.prototype.distanceFromIntersection = function (from, direction, position) {
        position.vsub(from, this.v0);
        var dot = this.v0.dot(direction);
        direction.mult(dot, this.intersect);
        this.intersect.vadd(from, this.intersect);
        var distance = position.distanceTo(this.intersect);
        return distance;
    };
    Ray.CLOSEST = 1;
    Ray.ANY = 2;
    Ray.ALL = 4;
    return Ray;
}());
export { Ray };
