import { Mat3 } from './Mat3';
import { Vec3 } from './Vec3';
import { Quaternion } from './Quaternion';
describe('Mat3', function () {
    it('should creation', function () {
        var m = new Mat3();
        var success = true;
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                success = success && (m.e(r, c) === 0);
            }
        }
        expect(success);
    });
    it('should e', function () {
        var m = new Mat3();
        m.e(1, 2, 5);
        expect(m.e(1, 2)).toEqual(5);
        var success = true;
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                if (r !== 1 || c !== 2) {
                    success = success && (m.e(r, c) === 0);
                }
            }
        }
        expect(success);
    });
    it('should identity', function () {
        var m = new Mat3();
        m.identity();
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                expect(m.e(r, c)).toEqual((r === c) ? 1 : 0);
            }
        }
    });
    it('should vmult', function () {
        var v = new Vec3(2, 3, 7);
        var m = new Mat3();
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                m.e(r, c, (1 + r * 3 + c));
            }
        }
        var t = m.vmult(v);
        expect(t.x === 29 && t.y === 65 && t.z === 101);
    });
    it('should mmult', function () {
        var m1 = new Mat3();
        var m2 = new Mat3();
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                m1.e(r, c, (1 + r * 3 + c));
            }
        }
        m2.e(0, 0, 5);
        m2.e(0, 1, 2);
        m2.e(0, 2, 4);
        m2.e(1, 0, 4);
        m2.e(1, 1, 5);
        m2.e(1, 2, 1);
        m2.e(2, 0, 1);
        m2.e(2, 1, 8);
        m2.e(2, 2, 0);
        var m3 = m1.mmult(m2);
        expect(m3.e(0, 0) === 16
            && m3.e(0, 1) === 36
            && m3.e(0, 2) === 6
            && m3.e(1, 0) === 46
            && m3.e(1, 1) === 81
            && m3.e(1, 2) === 21
            && m3.e(2, 0) === 76
            && m3.e(2, 1) === 126
            && m3.e(2, 2) === 36);
    });
    it('should solve', function () {
        var m = new Mat3();
        var v = new Vec3(2, 3, 7);
        m.e(0, 0, 5);
        m.e(0, 1, 2);
        m.e(0, 2, 4);
        m.e(1, 0, 4);
        m.e(1, 1, 5);
        m.e(1, 2, 1);
        m.e(2, 0, 1);
        m.e(2, 1, 8);
        m.e(2, 2, 0);
        var t = m.solve(v);
        var vv = m.vmult(t);
        expect(vv.almostEquals(v, 0.00001));
        var m1 = new Mat3();
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                m1.e(r, c, (1 + r * 3 + c));
            }
        }
        var error = false;
        try {
            m1.solve(v);
        }
        catch (e) {
            error = true;
        }
        expect(error);
    });
    it('should reverse ', function () {
        var m = new Mat3();
        m.e(0, 0, 5);
        m.e(0, 1, 2);
        m.e(0, 2, 4);
        m.e(1, 0, 4);
        m.e(1, 1, 5);
        m.e(1, 2, 1);
        m.e(2, 0, 1);
        m.e(2, 1, 8);
        m.e(2, 2, 0);
        var m2 = m.reverse();
        var m3 = m2.mmult(m);
        var success = true;
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                success = success && (Math.abs(m3.e(r, c) - (c === r ? 1 : 0)) < 0.00001);
            }
        }
        expect(success);
        var m1 = new Mat3();
        for (var c = 0; c < 3; c++) {
            for (var r = 0; r < 3; r++) {
                m1.e(r, c, (1 + r * 3 + c));
            }
        }
        var error = false;
        try {
            m1.reverse();
        }
        catch (e) {
            error = true;
        }
        expect(error);
    });
    it('should transpose', function () {
        var M = new Mat3([1, 2, 3,
            4, 5, 6,
            7, 8, 9]);
        var Mt = M.transpose();
        expect(Mt.elements).toEqual([1, 4, 7,
            2, 5, 8,
            3, 6, 9]);
    });
    it('should scale', function () {
        var M = new Mat3([1, 1, 1,
            1, 1, 1,
            1, 1, 1]);
        var Mt = M.scale(new Vec3(1, 2, 3));
        expect(Mt.elements).toEqual([1, 2, 3,
            1, 2, 3,
            1, 2, 3]);
    });
    it('should setRotationFromQuaternion', function () {
        var M = new Mat3(), q = new Quaternion(), original = new Vec3(1, 2, 3);
        M.setRotationFromQuaternion(q);
        var v = M.vmult(original);
        expect(v.almostEquals(original));
        q.setFromEuler(0.222, 0.123, 1.234);
        M.setRotationFromQuaternion(q);
        var Mv = M.vmult(original);
        var qv = q.vmult(original);
        expect(Mv.almostEquals(qv));
    });
});
