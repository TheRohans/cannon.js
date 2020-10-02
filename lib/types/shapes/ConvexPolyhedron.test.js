import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { ConvexPolyhedron } from './ConvexPolyhedron';
import { mockBoxHull, mockPolyBox, mockCube } from '../dapao.test';
import { Body } from '../objects/Body';
describe('ConvexPolyhedron', function () {
    it('should calculateWorldAABB', function () {
        var poly = mockPolyBox(1, 1, 1);
        var min = new Vec3();
        var max = new Vec3();
        poly.calculateWorldAABB(new Vec3(1, 0, 0), new Quaternion(0, 0, 0, 1), min, max);
        expect(min.x).toEqual(0);
        expect(max.x).toEqual(2);
        expect(min.y).toEqual(-1);
        expect(max.y).toEqual(1);
    });
    it('should calculate surface normals given a cube (quad)', function () {
        var hullA = mockBoxHull(1);
        hullA.faceNormals.forEach(function (fn) {
            expect(fn.length()).toEqual(1);
        });
    });
    it('should calculate surface normals given a cube (tri)', function () {
        var mc = mockCube();
        var cp = new ConvexPolyhedron(mc[0], mc[2]);
        cp.faceNormals.forEach(function (fn) {
            expect(fn.length()).toEqual(1);
        });
    });
    it('should caclualte surface norms', function () {
        var p1 = new Vec3(5, 4, -12);
        var p2 = new Vec3(5, 12, -12);
        var p3 = new Vec3(5, 4, -4);
        var target = new Vec3();
        ConvexPolyhedron.computeNormal(p1, p2, p3, target);
        expect(target.x).toEqual(1);
        expect(target.y).not.toBeGreaterThan(0);
        expect(target.z).not.toBeGreaterThan(0);
    });
    it('should caclualte surface norms 2', function () {
        var p1 = new Vec3(12, 3, -4);
        var p2 = new Vec3(12, 3, -12);
        var p3 = new Vec3(12, 5, -12);
        var target = new Vec3();
        ConvexPolyhedron.computeNormal(p1, p2, p3, target);
        expect(target.x).toEqual(1);
        expect(target.y).not.toBeGreaterThan(0);
        expect(target.z).not.toBeGreaterThan(0);
    });
    it('should caclualte surface norms 3', function () {
        var p1 = new Vec3(4, 3, -12);
        var p2 = new Vec3(4, 3, -4);
        var p3 = new Vec3(4, 5, -4);
        var target = new Vec3();
        ConvexPolyhedron.computeNormal(p1, p2, p3, target);
        expect(target.x).toEqual(-1);
        expect(target.y).not.toBeGreaterThan(0);
        expect(target.z).not.toBeGreaterThan(0);
    });
    it('should clipFaceAgainstPlane', function () {
        var h = mockBoxHull();
        var inverts = [new Vec3(-0.2, -0.2, -1),
            new Vec3(-0.2, 0.2, -1),
            new Vec3(0.2, 0.2, -1),
            new Vec3(0.2, -0.2, -1)];
        var outverts = [];
        h.clipFaceAgainstPlane(inverts, outverts, new Vec3(0, 0, 1), 0.0);
        expect(outverts.length).toEqual(4);
        inverts = [];
        outverts = [];
        h.clipFaceAgainstPlane(inverts, outverts, new Vec3(0, 0, 1), 2);
        expect(outverts.length).toEqual(0);
        var inverts2 = [new Vec3(-2, -2, 1),
            new Vec3(-2, 2, 1),
            new Vec3(2, 2, -1),
            new Vec3(2, -2, -1)];
        outverts = [];
        h.clipFaceAgainstPlane(inverts2, outverts, new Vec3(0, 0, 1), 0.0);
        expect(outverts.length).toEqual(4);
    });
    it('should clipFaceAgainstHull', function () {
        var hullA = mockBoxHull(0.5);
        var res = [];
        var sepNormal = new Vec3(0, 0, 1);
        var posA = new Vec3(0, 0, 0.45), quatA = new Quaternion();
        var worldVertsB = [new Vec3(-1.0, -1.0, 0),
            new Vec3(-1.0, 1.0, 0),
            new Vec3(1.0, 1.0, 0),
            new Vec3(1.0, -1.0, 0)];
        hullA.clipFaceAgainstHull(sepNormal, posA, quatA, worldVertsB, -100, 100, res);
        res.forEach(function (r) {
            expect(r.depth).toEqual(-0.04999999999999999);
        });
    });
    it('should clipAgainstHull X', function () {
        var hullA = mockBoxHull(1), posA = new Vec3(-0.5, 0, 0), quatA = new Quaternion();
        var hullB = mockBoxHull(1), posB = new Vec3(0.5, 0, 0), quatB = new Quaternion();
        var sepaxis = new Vec3();
        var found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
        var result = [];
        hullA.clipAgainstHull(posA, quatA, hullB, posB, quatB, sepaxis, -100, 100, result);
        result.forEach(function (r) {
            expect(r.depth).toEqual(-1);
        });
    });
    it('should clipAgainstHull Y', function () {
        var hullA = mockBoxHull(1), posA = new Vec3(0, -0.5, 0), quatA = new Quaternion();
        var hullB = mockBoxHull(1), posB = new Vec3(0, 0.5, 0), quatB = new Quaternion();
        var sepaxis = new Vec3();
        var found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
        var result = [];
        hullA.clipAgainstHull(posA, quatA, hullB, posB, quatB, sepaxis, -100, 100, result);
        result.forEach(function (r) {
            expect(r.depth).toEqual(-1);
        });
    });
    it('should clipAgainstHull Y (quad)', function () {
        var mc = mockBoxHull(1);
        var mc2 = mockBoxHull(1);
        var bi = new Body({ mass: 1 });
        bi.addShape(mc);
        bi.position = new Vec3(0, 0.5, 0);
        var bj = new Body({ mass: 1 });
        bi.addShape(mc);
        bj.position = new Vec3(0, -0.5, 0);
        var res = [];
        var dist = 100;
        var sepAxis = new Vec3(0, 1, 0);
        var xi = new Vec3(0, 0.5, 0);
        var xj = new Vec3(0, -0.5, 0);
        var qi = new Quaternion(0, 0, 0, 1);
        var qj = new Quaternion(0, 0, 0, 1);
        mc.clipAgainstHull(xi, qi, mc2, xj, qj, sepAxis, -dist, dist, res);
        expect(res.length).toBeGreaterThan(0);
    });
    it('should clipAgainstHull Y (tri)', function () {
        var mc = mockCube();
        var mc2 = mockCube();
        var si = new ConvexPolyhedron(mc[0], mc[2]);
        var sj = new ConvexPolyhedron(mc2[0], mc2[2]);
        var bi = new Body({ mass: 1 });
        bi.addShape(si);
        bi.position = new Vec3(0, 0.5, 0);
        var bj = new Body({ mass: 1 });
        bj.addShape(sj);
        bj.position = new Vec3(0, -0.5, 0);
        var res = [];
        var dist = 100;
        var sepAxis = new Vec3(0, 1, 0);
        var xi = new Vec3(0, 0.5, 0);
        var xj = new Vec3(0, -0.5, 0);
        var qi = new Quaternion(0, 0, 0, 1);
        var qj = new Quaternion(0, 0, 0, 1);
        si.clipAgainstHull(xi, qi, sj, xj, qj, sepAxis, -dist, dist, res);
        expect(res.length).toBeGreaterThan(0);
    });
    it('should clipAgainstHull Z', function () {
        var hullA = mockBoxHull(1), posA = new Vec3(0, 0, -0.5), quatA = new Quaternion();
        var hullB = mockBoxHull(1), posB = new Vec3(0, 0, 0.5), quatB = new Quaternion();
        var sepaxis = new Vec3();
        var found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
        var result = [];
        hullA.clipAgainstHull(posA, quatA, hullB, posB, quatB, sepaxis, -100, 100, result);
        result.forEach(function (r) {
            expect(r.depth).toEqual(-1);
        });
    });
    it('should testSepAxis', function () {
        var hullA = mockBoxHull(0.5), posA = new Vec3(-0.2, 0, 0), quatA = new Quaternion();
        var hullB = mockBoxHull(), posB = new Vec3(0.2, 0, 0), quatB = new Quaternion();
        var sepAxis = new Vec3(1, 0, 0);
        var _a = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB), found1 = _a[0], depth = _a[1];
        expect(found1).toBeTruthy();
        expect(depth).toEqual(0.6);
        posA.x = -5;
        var _b = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB), found2 = _b[0], dp = _b[1];
        expect(found2).toBeFalsy();
        posA.x = 1;
        quatB.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
        var _c = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB), found3 = _c[0], depth3 = _c[1];
        expect(typeof (found3)).toEqual('boolean');
        expect(typeof (depth3)).toEqual('number');
    });
    it('should findSepAxis', function () {
        var hullA = mockBoxHull(), posA = new Vec3(-0.2, 0, 0), quatA = new Quaternion();
        var hullB = mockBoxHull(), posB = new Vec3(0.2, 0, 0), quatB = new Quaternion();
        var sepaxis = new Vec3();
        var found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
        expect(found).toEqual(true);
        quatB.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
        var found2 = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
        expect(found2).toEqual(true);
    });
    it('should project', function () {
        var convex = mockBoxHull(0.5), pos = new Vec3(0, 0, 0), quat = new Quaternion();
        var axis = new Vec3(1, 0, 0);
        var result = [];
        ConvexPolyhedron.project(convex, axis, pos, quat, result);
        expect(result).toEqual([0.5, -0.5]);
        axis.set(-1, 0, 0);
        ConvexPolyhedron.project(convex, axis, pos, quat, result);
        expect(result).toEqual([0.5, -0.5]);
        axis.set(0, 1, 0);
        ConvexPolyhedron.project(convex, axis, pos, quat, result);
        expect(result).toEqual([0.5, -0.5]);
        pos.set(0, 1, 0);
        axis.set(0, 1, 0);
        ConvexPolyhedron.project(convex, axis, pos, quat, result);
        expect(result).toEqual([1.5, 0.5]);
        quat.setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);
        pos.set(0, 1, 0);
        axis.set(0, 1, 0);
        ConvexPolyhedron.project(convex, axis, pos, quat, result);
        expect(Math.abs(result[0] - 1.5) < 0.01).toBeTruthy();
        expect(Math.abs(result[1] - 0.5) < 0.01).toBeTruthy();
    });
    it('should get the average local point (the middle of the poly)', function () {
        var mc = mockCube();
        var poly = new ConvexPolyhedron(mc[0], mc[2]);
        var actual = poly.getAveragePointLocal();
        expect(actual).toEqual(new Vec3(0, 0, 0));
    });
    it('should get the average local point (the middle of the poly) 2', function () {
        var poly = mockPolyBox(1.5, 6, 0.20);
        var actual = poly.getAveragePointLocal();
        expect(actual).toEqual(new Vec3(0, 0, -6.938893903907228e-18));
    });
    it('should return true for pointInside (quad)', function () {
        var poly = mockBoxHull(1);
        var actual = poly.pointIsInside(new Vec3(0.2, 0.3, 0.2));
        expect(actual).toEqual(true);
    });
    it('should return true for pointInside (tri)', function () {
        var mc = mockCube();
        var poly = new ConvexPolyhedron(mc[0], mc[2]);
        var actual = poly.pointIsInside(new Vec3(0.2, 0.3, 0.2));
        expect(actual).toEqual(true);
    });
    it('should return false for pointInside (quad)', function () {
        var poly = mockBoxHull(1);
        var actual = poly.pointIsInside(new Vec3(1.5, 1.5, 1.5));
        expect(actual).toEqual(false);
    });
    it('should return false for pointInside (tri)', function () {
        var mc = mockCube();
        var poly = new ConvexPolyhedron(mc[0], mc[2]);
        var actual = poly.pointIsInside(new Vec3(1.5, 1.5, 1.5));
        expect(actual).toEqual(false);
    });
    it('should calculateWorldAABB from triangle polyhedron', function () {
        var mc = mockCube();
        var poly = new ConvexPolyhedron(mc[0], mc[2]);
        var min = new Vec3();
        var max = new Vec3();
        poly.calculateWorldAABB(new Vec3(0, 0, 0), new Quaternion(0, 0, 0, 1), min, max);
        expect(min.x).toEqual(-1);
        expect(min.y).toEqual(-1);
        expect(min.z).toEqual(-1);
        expect(max.x).toEqual(1);
        expect(max.y).toEqual(1);
        expect(max.z).toEqual(1);
    });
    it('should compute normals from triangles', function () {
        var input = [
            [new Vec3(1.000000, 1.000000, -1.000000), new Vec3(1.000000, -1.000000, 1.000000), new Vec3(1.000000, -1.000000, -1.000000)],
            [new Vec3(1.000000, 1.000000, 1.000000), new Vec3(-1.000000, -1.000000, 1.000000), new Vec3(1.000000, -1.000000, 1.000000)],
            [new Vec3(1.000000, -1.000000, -1.000000), new Vec3(-1.000000, -1.000000, 1.000000), new Vec3(-1.000000, -1.000000, -1.000000)],
            [new Vec3(1, -1, 1), new Vec3(-1, -1, -1), new Vec3(-1, 1, -1)],
        ];
        var expected = [
            new Vec3(1.0000, 0.0000, 0.0000),
            new Vec3(0.0000, 0.0000, 1.0000),
            new Vec3(0.0000, -1.0000, 0.0000),
            new Vec3(0.7071067811865475, 0, -0.7071067811865475),
        ];
        var scratch = new Vec3();
        input.forEach(function (v, i) {
            ConvexPolyhedron.computeNormal(v[0], v[1], v[2], scratch);
            expect(scratch.almostEquals(expected[i])).toEqual(true);
        });
    });
    it('should calculateWorldAABB always decreasing verts NoUndefined', function () {
        var vertices = [
            new Vec3(4, 4, 4),
            new Vec3(3, 3, 3),
            new Vec3(2, 2, 2),
            new Vec3(1, 1, 1),
            new Vec3(0, 0, 0),
            new Vec3(-1, -1, -1),
            new Vec3(-2, -2, -2),
            new Vec3(-3, -3, -3)
        ];
        var indices = [
            [3, 2, 1, 0],
            [4, 5, 6, 7],
            [5, 4, 0, 1],
            [2, 3, 7, 6],
            [0, 4, 7, 3],
            [1, 2, 6, 5],
        ];
        var poly = new ConvexPolyhedron(vertices, indices);
        var min = new Vec3();
        var max = new Vec3();
        poly.calculateWorldAABB(new Vec3(0, 0, 0), new Quaternion(0, 0, 0, 1), min, max);
        expect(min.x).not.toBeUndefined();
        expect(max.x).not.toBeUndefined();
        expect(min.y).not.toBeUndefined();
        expect(max.y).not.toBeUndefined();
        expect(min.z).not.toBeUndefined();
        expect(max.z).not.toBeUndefined();
    });
    it('should find the closest face', function () {
        var mc = mockCube();
        var cp = new ConvexPolyhedron(mc[0], mc[2]);
        var actual = cp.findClosestFace(cp, new Quaternion().setFromEuler(0, 90, 0, 'XYZ'), new Vec3(0, 1, 0), false);
        expect(actual).toEqual(5);
    });
    it('should find the closest face 2', function () {
        var mc = mockCube();
        var cp = new ConvexPolyhedron(mc[0], mc[2]);
        var actual = cp.findClosestFace(cp, new Quaternion().setFromEuler(0, 90, 0, 'XYZ'), new Vec3(0, -1, 0));
        expect(actual).toEqual(4);
    });
});
