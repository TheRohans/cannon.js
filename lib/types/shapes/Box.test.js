import { Box } from './Box';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
describe('Box', function () {
    it('should forEachWorldCorner', function () {
        var box = new Box(new Vec3(1, 1, 1));
        var pos = new Vec3();
        var quat = new Quaternion();
        quat.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI * 0.25);
        var numCorners = 0;
        var unique = [];
        box.forEachWorldCorner(pos, quat, function (x, y, z) {
            var corner = new Vec3(x, y, z);
            for (var i = 0; i < unique.length; i++) {
                expect(corner.almostEquals(unique[i])).toBeFalsy();
            }
            unique.push(corner);
            numCorners++;
        });
        expect(numCorners).toEqual(8);
    });
    it('should calculateWorldAABB', function () {
        var box = new Box(new Vec3(1, 1, 1));
        var min = new Vec3();
        var max = new Vec3();
        box.calculateWorldAABB(new Vec3(3, 0, 0), new Quaternion(0, 0, 0, 1), min, max);
        expect(min.x).toEqual(2);
        expect(max.x).toEqual(4);
        expect(min.y).toEqual(-1);
        expect(max.y).toEqual(1);
    });
});
