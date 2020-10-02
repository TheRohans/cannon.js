import { Vec3 } from './math/Vec3';
import { Box } from './shapes/Box';
export var mockCube = function () {
    var vertices = [
        new Vec3(-1.000000, -1.000000, 1.000000),
        new Vec3(-1.000000, 1.000000, 1.000000),
        new Vec3(-1.000000, -1.000000, -1.000000),
        new Vec3(-1.000000, 1.000000, -1.000000),
        new Vec3(1.000000, -1.000000, 1.000000),
        new Vec3(1.000000, 1.000000, 1.000000),
        new Vec3(1.000000, -1.000000, -1.000000),
        new Vec3(1.000000, 1.000000, -1.000000),
    ];
    var normals = [
        new Vec3(-1.0000, 0.0000, 0.0000),
        new Vec3(0.0000, 0.0000, -1.0000),
        new Vec3(1.0000, 0.0000, 0.0000),
        new Vec3(0.0000, 0.0000, 1.0000),
        new Vec3(0.0000, -1.0000, 0.0000),
        new Vec3(0.0000, 1.0000, 0.0000),
    ];
    var vn = [
        [2, 1], [3, 1], [1, 1],
        [4, 2], [7, 2], [3, 2],
        [8, 3], [5, 3], [7, 3],
        [6, 4], [1, 4], [5, 4],
        [7, 5], [1, 5], [3, 5],
        [4, 6], [6, 6], [8, 6],
        [2, 1], [4, 1], [3, 1],
        [4, 2], [8, 2], [7, 2],
        [8, 3], [6, 3], [5, 3],
        [6, 4], [2, 4], [1, 4],
        [7, 5], [5, 5], [1, 5],
        [4, 6], [2, 6], [6, 6],
    ];
    var faces = [];
    for (var z = 0; z < vn.length; z += 3) {
        var f = [
            vn[z][0] - 1,
            vn[z + 1][0] - 1,
            vn[z + 2][0] - 1,
        ];
        faces.push(f);
    }
    return [vertices, normals, faces];
};
export var mockBoxHull = function (size) {
    if (size === void 0) { size = 0.5; }
    var box = new Box(new Vec3(size, size, size));
    return box.convexPolyhedronRepresentation;
};
export var mockPolyBox = function (sx, sy, sz) {
    var v = Vec3;
    var box = new Box(new Vec3(sx, sy, sz));
    return box.convexPolyhedronRepresentation;
};
describe('Main', function () {
    it('should just exist', function () {
        expect(true).toEqual(true);
    });
});
