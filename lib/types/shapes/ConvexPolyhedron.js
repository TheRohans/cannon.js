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
import { Transform } from '../math/Transform';
var HullResult = (function () {
    function HullResult() {
    }
    return HullResult;
}());
export { HullResult };
var ConvexPolyhedron = (function (_super) {
    __extends(ConvexPolyhedron, _super);
    function ConvexPolyhedron(points, faces, uniqueAxes, faceNormals) {
        var _this = _super.call(this, { type: Shape.types.CONVEXPOLYHEDRON }) || this;
        _this.cah_WorldNormal = new Vec3();
        _this.fsa_faceANormalWS3 = new Vec3();
        _this.fsa_Worldnormal1 = new Vec3();
        _this.fsa_deltaC = new Vec3();
        _this.fsa_worldEdge0 = new Vec3();
        _this.fsa_worldEdge1 = new Vec3();
        _this.fsa_Cross = new Vec3();
        _this.cli_aabbmin = new Vec3();
        _this.cli_aabbmax = new Vec3();
        _this.cfah_edge0 = new Vec3();
        _this.cfah_WorldEdge0 = new Vec3();
        _this.cfah_worldPlaneAnormal1 = new Vec3();
        _this.cfah_planeNormalWS1 = new Vec3();
        _this.cfah_worldA1 = new Vec3();
        _this.cfah_localPlaneNormal = new Vec3();
        _this.cfah_planeNormalWS = new Vec3();
        _this.ConvexPolyhedron_pointIsInside = new Vec3();
        _this.ConvexPolyhedron_vToP = new Vec3();
        _this.ConvexPolyhedron_vToPointInside = new Vec3();
        _this.vertices = points || [];
        _this.worldVertices = [];
        _this.worldVerticesNeedsUpdate = true;
        _this.faces = faces || [];
        _this.faceNormals = faceNormals || [];
        _this.computeNormals();
        _this.worldFaceNormalsNeedsUpdate = true;
        _this.worldFaceNormals = [];
        _this.uniqueEdges = [];
        _this.uniqueAxes = uniqueAxes ? uniqueAxes.slice() : null;
        _this.computeEdges();
        _this.updateBoundingSphereRadius();
        return _this;
    }
    ConvexPolyhedron.prototype.computeEdges = function () {
        var computeEdges_tmpEdge = new Vec3();
        var faces = this.faces;
        var vertices = this.vertices;
        var nv = vertices.length;
        var edges = this.uniqueEdges;
        edges.length = 0;
        var edge = computeEdges_tmpEdge;
        for (var i = 0; i !== faces.length; i++) {
            var face = faces[i];
            var numVertices = face.length;
            for (var j = 0; j !== numVertices; j++) {
                var k = (j + 1) % numVertices;
                vertices[face[j]].vsub(vertices[face[k]], edge);
                edge.normalize();
                var found = false;
                for (var p = 0; p !== edges.length; p++) {
                    if (edges[p].almostEquals(edge) || edges[p].almostEquals(edge)) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    edges.push(edge.clone());
                }
            }
        }
    };
    ConvexPolyhedron.prototype.computeNormals = function () {
        if (!this.faceNormals.length) {
            this.faceNormals = new Array(this.faces.length);
            for (var i = 0; i < this.faces.length; i++) {
                for (var j = 0; j < this.faces[i].length; j++) {
                    if (!this.vertices[this.faces[i][j]]) {
                        throw new Error('Vertex ' + this.faces[i][j] + ' not found!');
                    }
                }
                var n = this.faceNormals[i] || new Vec3();
                this.getFaceNormal(i, n);
                this.faceNormals[i] = n;
            }
        }
    };
    ConvexPolyhedron.computeNormal = function (va, vb, vc, target) {
        var cb = new Vec3();
        var ab = new Vec3();
        vb.vsub(va, ab);
        vc.vsub(vb, cb);
        cb.cross(ab, target);
        if (!target.isZero()) {
            target.normalize();
        }
        target.negate(target);
    };
    ConvexPolyhedron.prototype.getFaceNormal = function (i, target) {
        var tg = target || new Vec3();
        var f = this.faces[i];
        var va = this.vertices[f[0]];
        var vb = this.vertices[f[1]];
        var vc = this.vertices[f[2]];
        ConvexPolyhedron.computeNormal(va, vb, vc, tg);
        return tg;
    };
    ConvexPolyhedron.prototype.findClosestFace = function (hull, quat, separatingNormal, useMin) {
        if (useMin === void 0) { useMin = false; }
        var scratchVec = this.cah_WorldNormal;
        var dmax = -Number.MAX_VALUE;
        var dmin = Number.MAX_VALUE;
        var closestFace = -1;
        var flen = hull.faces.length;
        for (var face = 0; face < flen; face++) {
            scratchVec.copy(hull.faceNormals[face]);
            quat.vmult(scratchVec, scratchVec);
            var d = scratchVec.dot(separatingNormal);
            if (useMin) {
                if (d < dmin) {
                    dmin = d;
                    closestFace = face;
                }
            }
            else {
                if (d > dmax) {
                    dmax = d;
                    closestFace = face;
                }
            }
        }
        return closestFace;
    };
    ConvexPolyhedron.prototype.clipAgainstHull = function (posA, quatA, hullB, posB, quatB, separatingNormal, minDist, maxDist, result) {
        var closestFaceB = this.findClosestFace(hullB, quatB, separatingNormal);
        var worldVertsB1 = [];
        var polyB = hullB.faces[closestFaceB];
        var numVertices = polyB.length;
        for (var e0 = 0; e0 < numVertices; e0++) {
            var b = hullB.vertices[polyB[e0]];
            var worldb = new Vec3();
            worldb.copy(b);
            quatB.vmult(worldb, worldb);
            posB.vadd(worldb, worldb);
            worldVertsB1.push(worldb);
        }
        if (closestFaceB >= 0) {
            this.clipFaceAgainstHull(separatingNormal, posA, quatA, worldVertsB1, minDist, maxDist, result);
        }
    };
    ConvexPolyhedron.prototype.findSeparatingAxis = function (hullB, posA, quatA, posB, quatB, target, faceListA, faceListB) {
        var faceANormalWS3 = this.fsa_faceANormalWS3, Worldnormal1 = this.fsa_Worldnormal1, deltaC = this.fsa_deltaC, worldEdge0 = this.fsa_worldEdge0, worldEdge1 = this.fsa_worldEdge1, Cross = this.fsa_Cross;
        var dmin = Number.MAX_VALUE;
        var hullA = this;
        if (!hullA.uniqueAxes) {
            var numFacesA = faceListA ? faceListA.length : hullA.faces.length;
            var i = numFacesA;
            while (i--) {
                var fi = faceListA ? faceListA[i] : i;
                faceANormalWS3.copy(hullA.faceNormals[fi]);
                quatA.vmult(faceANormalWS3, faceANormalWS3);
                var _a = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB), b = _a[0], d = _a[1];
                if (b === false) {
                    return false;
                }
                if (d < dmin) {
                    dmin = d;
                    target.copy(faceANormalWS3);
                }
            }
        }
        else {
            var i = hullA.uniqueAxes.length;
            while (i--) {
                quatA.vmult(hullA.uniqueAxes[i], faceANormalWS3);
                var _b = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB), b = _b[0], d = _b[1];
                if (b === false) {
                    return false;
                }
                if (d < dmin) {
                    dmin = d;
                    target.copy(faceANormalWS3);
                }
            }
        }
        if (!hullB.uniqueAxes) {
            var numFacesB = faceListB ? faceListB.length : hullB.faces.length;
            for (var i = 0; i < numFacesB; i++) {
                var fi = faceListB ? faceListB[i] : i;
                Worldnormal1.copy(hullB.faceNormals[fi]);
                quatB.vmult(Worldnormal1, Worldnormal1);
                var _c = hullA.testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB), b = _c[0], d = _c[1];
                if (b === false) {
                    return false;
                }
                if (d < dmin) {
                    dmin = d;
                    target.copy(Worldnormal1);
                }
            }
        }
        else {
            var i = hullB.uniqueAxes.length;
            while (i--) {
                quatB.vmult(hullB.uniqueAxes[i], Worldnormal1);
                var _d = hullA.testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB), b = _d[0], d = _d[1];
                if (b === false) {
                    return false;
                }
                if (d < dmin) {
                    dmin = d;
                    target.copy(Worldnormal1);
                }
            }
        }
        var e0 = hullA.uniqueEdges.length;
        while (e0--) {
            quatA.vmult(hullA.uniqueEdges[e0], worldEdge0);
            var e1 = hullB.uniqueEdges.length;
            while (e1--) {
                quatB.vmult(hullB.uniqueEdges[e1], worldEdge1);
                worldEdge0.cross(worldEdge1, Cross);
                if (!Cross.almostZero()) {
                    Cross.normalize();
                    var _e = hullA.testSepAxis(Cross, hullB, posA, quatA, posB, quatB), b = _e[0], dist = _e[1];
                    if (b === false) {
                        return false;
                    }
                    if (dist < dmin) {
                        dmin = dist;
                        target.copy(Cross);
                    }
                }
            }
        }
        posB.vsub(posA, deltaC);
        if ((deltaC.dot(target)) > 0.0) {
            target.negate(target);
        }
        return true;
    };
    ConvexPolyhedron.prototype.testSepAxis = function (axis, hullB, posA, quatA, posB, quatB) {
        var maxminA = [];
        var maxminB = [];
        var hullA = this;
        ConvexPolyhedron.project(hullA, axis, posA, quatA, maxminA);
        ConvexPolyhedron.project(hullB, axis, posB, quatB, maxminB);
        var maxA = maxminA[0];
        var minA = maxminA[1];
        var maxB = maxminB[0];
        var minB = maxminB[1];
        if (maxA < minB || maxB < minA) {
            return [false, 0];
        }
        var d0 = maxA - minB;
        var d1 = maxB - minA;
        var depth = d0 < d1 ? d0 : d1;
        return [true, depth];
    };
    ConvexPolyhedron.prototype.calculateLocalInertia = function (mass, target) {
        var maxminA = [], maxminB = [];
        this.computeLocalAABB(this.cli_aabbmin, this.cli_aabbmax);
        var x = this.cli_aabbmax.x - this.cli_aabbmin.x, y = this.cli_aabbmax.y - this.cli_aabbmin.y, z = this.cli_aabbmax.z - this.cli_aabbmin.z;
        target.x = 1.0 / 12.0 * mass * (2 * y * 2 * y + 2 * z * 2 * z);
        target.y = 1.0 / 12.0 * mass * (2 * x * 2 * x + 2 * z * 2 * z);
        target.z = 1.0 / 12.0 * mass * (2 * y * 2 * y + 2 * x * 2 * x);
    };
    ConvexPolyhedron.prototype.getPlaneConstantOfFace = function (face_i) {
        var f = this.faces[face_i];
        var n = this.faceNormals[face_i];
        var v = this.vertices[f[0]];
        var c = -n.dot(v);
        return c;
    };
    ConvexPolyhedron.prototype.clipFaceAgainstHull = function (separatingNormal, posA, quatA, worldVertsB1, minDist, maxDist, result) {
        var edge0 = this.cfah_edge0, WorldEdge0 = this.cfah_WorldEdge0, worldPlaneAnormal1 = this.cfah_worldPlaneAnormal1, planeNormalWS1 = this.cfah_planeNormalWS1, worldA1 = this.cfah_worldA1, localPlaneNormal = this.cfah_localPlaneNormal, planeNormalWS = this.cfah_planeNormalWS;
        var hullA = this;
        var worldVertsB2 = [];
        var pVtxIn = worldVertsB1;
        var pVtxOut = worldVertsB2;
        var closestFaceA = this.findClosestFace(hullA, quatA, separatingNormal, true);
        if (closestFaceA < 0) {
            return;
        }
        var polyA = hullA.faces[closestFaceA];
        var connectedFaces = [];
        for (var i = 0; i < hullA.faces.length; i++) {
            for (var j = 0; j < hullA.faces[i].length; j++) {
                if (polyA.indexOf(hullA.faces[i][j]) !== -1 &&
                    i !== closestFaceA &&
                    connectedFaces.indexOf(i) === -1) {
                    connectedFaces.push(i);
                }
            }
        }
        var numContacts = pVtxIn.length;
        var numVerticesA = polyA.length;
        var res = [];
        var planeEqWS;
        for (var e0 = 0; e0 < numVerticesA; e0++) {
            var a = hullA.vertices[polyA[e0]];
            var b = hullA.vertices[polyA[(e0 + 1) % numVerticesA]];
            a.vsub(b, edge0);
            WorldEdge0.copy(edge0);
            quatA.vmult(WorldEdge0, WorldEdge0);
            posA.vadd(WorldEdge0, WorldEdge0);
            worldPlaneAnormal1.copy(this.faceNormals[closestFaceA]);
            quatA.vmult(worldPlaneAnormal1, worldPlaneAnormal1);
            posA.vadd(worldPlaneAnormal1, worldPlaneAnormal1);
            WorldEdge0.cross(worldPlaneAnormal1, planeNormalWS1);
            planeNormalWS1.negate(planeNormalWS1);
            worldA1.copy(a);
            quatA.vmult(worldA1, worldA1);
            posA.vadd(worldA1, worldA1);
            var otherFace = connectedFaces[e0];
            if (otherFace != undefined) {
                localPlaneNormal.copy(this.faceNormals[otherFace]);
                var localPlaneEqT = this.getPlaneConstantOfFace(otherFace);
                planeNormalWS.copy(localPlaneNormal);
                quatA.vmult(planeNormalWS, planeNormalWS);
                planeEqWS = localPlaneEqT - planeNormalWS.dot(posA);
            }
            else {
                continue;
            }
            this.clipFaceAgainstPlane(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);
            while (pVtxIn.length) {
                pVtxIn.shift();
            }
            while (pVtxOut.length) {
                pVtxIn.push(pVtxOut.shift());
            }
        }
        localPlaneNormal.copy(this.faceNormals[closestFaceA]);
        var localPlaneEq = this.getPlaneConstantOfFace(closestFaceA);
        planeNormalWS.copy(localPlaneNormal);
        quatA.vmult(planeNormalWS, planeNormalWS);
        planeEqWS = localPlaneEq - planeNormalWS.dot(posA);
        for (var i = 0; i < pVtxIn.length; i++) {
            var depth = planeNormalWS.dot(pVtxIn[i]) + planeEqWS;
            if (depth <= minDist) {
                console.log('clamped: depth=' + depth + ' to minDist=' + (minDist + ''));
                depth = minDist;
            }
            if (depth <= maxDist) {
                var point = pVtxIn[i];
                if (depth <= 0) {
                    var p = {
                        point: point,
                        normal: planeNormalWS,
                        depth: depth,
                    };
                    result.push(p);
                }
            }
        }
    };
    ConvexPolyhedron.prototype.clipFaceAgainstPlane = function (inVertices, outVertices, planeNormal, planeConstant) {
        var n_dot_first, n_dot_last;
        var numVerts = inVertices.length;
        if (numVerts < 2) {
            return outVertices;
        }
        var firstVertex = inVertices[inVertices.length - 1], lastVertex = inVertices[0];
        n_dot_first = planeNormal.dot(firstVertex) + planeConstant;
        for (var vi = 0; vi < numVerts; vi++) {
            lastVertex = inVertices[vi];
            n_dot_last = planeNormal.dot(lastVertex) + planeConstant;
            if (n_dot_first < 0) {
                if (n_dot_last < 0) {
                    var newv = new Vec3();
                    newv.copy(lastVertex);
                    outVertices.push(newv);
                }
                else {
                    var newv = new Vec3();
                    firstVertex.lerp(lastVertex, n_dot_first / (n_dot_first - n_dot_last), newv);
                    outVertices.push(newv);
                }
            }
            else {
                if (n_dot_last < 0) {
                    var newv = new Vec3();
                    firstVertex.lerp(lastVertex, n_dot_first / (n_dot_first - n_dot_last), newv);
                    outVertices.push(newv);
                    outVertices.push(lastVertex);
                }
            }
            firstVertex = lastVertex;
            n_dot_first = n_dot_last;
        }
        return outVertices;
    };
    ConvexPolyhedron.prototype.computeWorldVertices = function (position, quat) {
        var N = this.vertices.length;
        while (this.worldVertices.length < N) {
            this.worldVertices.push(new Vec3());
        }
        var verts = this.vertices, worldVerts = this.worldVertices;
        for (var i = 0; i !== N; i++) {
            quat.vmult(verts[i], worldVerts[i]);
            position.vadd(worldVerts[i], worldVerts[i]);
        }
        this.worldVerticesNeedsUpdate = false;
    };
    ConvexPolyhedron.prototype.computeLocalAABB = function (aabbmin, aabbmax) {
        var computeLocalAABB_worldVert = new Vec3();
        var n = this.vertices.length, vertices = this.vertices, worldVert = computeLocalAABB_worldVert;
        aabbmin.set(Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE);
        aabbmax.set(-Number.MAX_VALUE, -Number.MAX_VALUE, -Number.MAX_VALUE);
        for (var i = 0; i < n; i++) {
            var v = vertices[i];
            if (v.x < aabbmin.x) {
                aabbmin.x = v.x;
            }
            else if (v.x > aabbmax.x) {
                aabbmax.x = v.x;
            }
            if (v.y < aabbmin.y) {
                aabbmin.y = v.y;
            }
            else if (v.y > aabbmax.y) {
                aabbmax.y = v.y;
            }
            if (v.z < aabbmin.z) {
                aabbmin.z = v.z;
            }
            else if (v.z > aabbmax.z) {
                aabbmax.z = v.z;
            }
        }
    };
    ConvexPolyhedron.prototype.computeWorldFaceNormals = function (quat) {
        var N = this.faceNormals.length;
        while (this.worldFaceNormals.length < N) {
            this.worldFaceNormals.push(new Vec3());
        }
        var normals = this.faceNormals, worldNormals = this.worldFaceNormals;
        for (var i = 0; i !== N; i++) {
            quat.vmult(normals[i], worldNormals[i]);
        }
        this.worldFaceNormalsNeedsUpdate = false;
    };
    ConvexPolyhedron.prototype.updateBoundingSphereRadius = function () {
        var max2 = 0;
        var verts = this.vertices;
        for (var i = 0, N = verts.length; i !== N; i++) {
            var norm2 = verts[i].norm2();
            if (norm2 > max2) {
                max2 = norm2;
            }
        }
        this.boundingSphereRadius = Math.sqrt(max2);
    };
    ConvexPolyhedron.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        var tempWorldVertex = new Vec3();
        var n = this.vertices.length, verts = this.vertices;
        var minx, miny, minz, maxx, maxy, maxz;
        for (var i = 0; i < n; i++) {
            tempWorldVertex.copy(verts[i]);
            quat.vmult(tempWorldVertex, tempWorldVertex);
            pos.vadd(tempWorldVertex, tempWorldVertex);
            var v = tempWorldVertex;
            if (v.x < minx || minx === undefined) {
                minx = v.x;
            }
            if (v.x > maxx || maxx === undefined) {
                maxx = v.x;
            }
            if (v.y < miny || miny === undefined) {
                miny = v.y;
            }
            if (v.y > maxy || maxy === undefined) {
                maxy = v.y;
            }
            if (v.z < minz || minz === undefined) {
                minz = v.z;
            }
            if (v.z > maxz || maxz === undefined) {
                maxz = v.z;
            }
        }
        min.set(minx, miny, minz);
        max.set(maxx, maxy, maxz);
    };
    ConvexPolyhedron.prototype.volume = function () {
        return 4.0 * Math.PI * this.boundingSphereRadius / 3.0;
    };
    ConvexPolyhedron.prototype.getAveragePointLocal = function (target) {
        target = target || new Vec3();
        var n = this.vertices.length, verts = this.vertices;
        for (var i = 0; i < n; i++) {
            target.vadd(verts[i], target);
        }
        target.scale(1 / n, target);
        return target;
    };
    ConvexPolyhedron.prototype.transformAllPoints = function (offset, quat) {
        var n = this.vertices.length, verts = this.vertices;
        if (quat) {
            for (var i = 0; i < n; i++) {
                var v = verts[i];
                quat.vmult(v, v);
            }
            for (var i = 0; i < this.faceNormals.length; i++) {
                var v = this.faceNormals[i];
                quat.vmult(v, v);
            }
        }
        if (offset) {
            for (var i = 0; i < n; i++) {
                var v = verts[i];
                v.vadd(offset, v);
            }
        }
    };
    ConvexPolyhedron.prototype.pointIsInside = function (p) {
        var verts = this.vertices;
        var faces = this.faces;
        var normals = this.faceNormals;
        var N = this.faces.length;
        var pointInside = this.ConvexPolyhedron_pointIsInside;
        this.getAveragePointLocal(pointInside);
        var sidesCrossed = 0;
        for (var i = 0; i < N; i++) {
            var norm = normals[i];
            var v = verts[faces[i][0]];
            var vToP = this.ConvexPolyhedron_vToP;
            p.vsub(v, vToP);
            var r1 = norm.dot(vToP);
            var vToPointInside = this.ConvexPolyhedron_vToPointInside;
            pointInside.vsub(v, vToPointInside);
            var r2 = norm.dot(vToPointInside);
            if (Math.sign(r1) !== Math.sign(r2) && r1 !== 0) {
                sidesCrossed++;
            }
            if (sidesCrossed !== 0 && sidesCrossed % 3 === 0) {
                return false;
            }
        }
        return true;
    };
    ConvexPolyhedron.project = function (hull, axis, pos, quat, result) {
        var localAxis = ConvexPolyhedron.project_localAxis, localOrigin = ConvexPolyhedron.project_localOrigin, vs = hull.vertices;
        var max = 0, min = 0;
        localOrigin.setZero();
        Transform.vectorToLocalFrame(pos, quat, axis, localAxis);
        Transform.pointToLocalFrame(pos, quat, localOrigin, localOrigin);
        var add = localOrigin.dot(localAxis);
        min = max = vs[0].dot(localAxis);
        var i = hull.vertices.length;
        while (i--) {
            var val = vs[i].dot(localAxis);
            if (val > max) {
                max = val;
            }
            if (val < min) {
                min = val;
            }
        }
        min -= add;
        max -= add;
        if (min > max) {
            var temp = min;
            min = max;
            max = temp;
        }
        result[0] = max;
        result[1] = min;
    };
    ConvexPolyhedron.project_localAxis = new Vec3();
    ConvexPolyhedron.project_localOrigin = new Vec3();
    return ConvexPolyhedron;
}(Shape));
export { ConvexPolyhedron };
