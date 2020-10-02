import { Vec3 } from './Vec3';
var Mat3 = (function () {
    function Mat3(elements) {
        if (elements) {
            this.elements = elements;
        }
        else {
            this.elements = [0, 0, 0, 0, 0, 0, 0, 0, 0];
        }
    }
    Mat3.prototype.identity = function () {
        var e = this.elements;
        e[0] = 1;
        e[1] = 0;
        e[2] = 0;
        e[3] = 0;
        e[4] = 1;
        e[5] = 0;
        e[6] = 0;
        e[7] = 0;
        e[8] = 1;
    };
    Mat3.prototype.setZero = function () {
        var e = this.elements;
        e[0] = 0;
        e[1] = 0;
        e[2] = 0;
        e[3] = 0;
        e[4] = 0;
        e[5] = 0;
        e[6] = 0;
        e[7] = 0;
        e[8] = 0;
    };
    Mat3.prototype.setTrace = function (vec3) {
        var e = this.elements;
        e[0] = vec3.x;
        e[4] = vec3.y;
        e[8] = vec3.z;
    };
    Mat3.prototype.getTrace = function (target) {
        target = target || new Vec3();
        var e = this.elements;
        target.x = e[0];
        target.y = e[4];
        target.z = e[8];
    };
    Mat3.prototype.vmult = function (v, target) {
        target = target || new Vec3();
        var e = this.elements, x = v.x, y = v.y, z = v.z;
        target.x = e[0] * x + e[1] * y + e[2] * z;
        target.y = e[3] * x + e[4] * y + e[5] * z;
        target.z = e[6] * x + e[7] * y + e[8] * z;
        return target;
    };
    Mat3.prototype.smult = function (s) {
        for (var i = 0; i < this.elements.length; i++) {
            this.elements[i] *= s;
        }
    };
    Mat3.prototype.mmult = function (m, target) {
        var r = target || new Mat3();
        for (var i = 0; i < 3; i++) {
            for (var j = 0; j < 3; j++) {
                var sum = 0.0;
                for (var k = 0; k < 3; k++) {
                    sum += m.elements[i + k * 3] * this.elements[k + j * 3];
                }
                r.elements[i + j * 3] = sum;
            }
        }
        return r;
    };
    Mat3.prototype.scale = function (v, target) {
        target = target || new Mat3();
        var e = this.elements, t = target.elements;
        for (var i = 0; i !== 3; i++) {
            t[3 * i + 0] = v.x * e[3 * i + 0];
            t[3 * i + 1] = v.y * e[3 * i + 1];
            t[3 * i + 2] = v.z * e[3 * i + 2];
        }
        return target;
    };
    Mat3.prototype.solve = function (b, target) {
        target = target || new Vec3();
        var nr = 3;
        var nc = 4;
        var eqns = [];
        for (var i = 0; i < nr * nc; i++) {
            eqns.push(0);
        }
        var j;
        for (var i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = this.elements[i + 3 * j];
            }
        }
        eqns[3 + 4 * 0] = b.x;
        eqns[3 + 4 * 1] = b.y;
        eqns[3 + 4 * 2] = b.z;
        var n = 3, np;
        var k = n;
        var kp = 4;
        var p;
        do {
            var i = k - n;
            if (eqns[i + nc * i] === 0) {
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] !== 0) {
                        np = kp;
                        do {
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np);
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] !== 0) {
                for (j = i + 1; j < k; j++) {
                    var multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        p = kp - np;
                        eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np);
                }
            }
        } while (--n);
        target.z = eqns[2 * nc + 3] / eqns[2 * nc + 2];
        target.y = (eqns[1 * nc + 3] - eqns[1 * nc + 2] * target.z) / eqns[1 * nc + 1];
        target.x = (eqns[0 * nc + 3] - eqns[0 * nc + 2] * target.z - eqns[0 * nc + 1] * target.y) / eqns[0 * nc + 0];
        if (isNaN(target.x) || isNaN(target.y) || isNaN(target.z) || target.x === Infinity || target.y === Infinity || target.z === Infinity) {
            throw new Error('Could not solve equation! Got x=['
                + target.toString() + '], b=[' + b.toString()
                + '], A=[' + this.toString() + ']');
        }
        return target;
    };
    Mat3.prototype.e = function (row, column, value) {
        if (value === undefined) {
            return this.elements[column + 3 * row];
        }
        else {
            return this.elements[column + 3 * row] = value;
        }
    };
    Mat3.prototype.copy = function (source) {
        for (var i = 0; i < source.elements.length; i++) {
            this.elements[i] = source.elements[i];
        }
        return this;
    };
    Mat3.prototype.toString = function () {
        var r = '';
        var sep = ',';
        for (var i = 0; i < 9; i++) {
            r += this.elements[i] + sep;
        }
        return r;
    };
    Mat3.prototype.reverse = function (target) {
        target = target || new Mat3();
        var nr = 3;
        var nc = 6;
        var eqns = [];
        var i = 0;
        for (i = 0; i < nr * nc; i++) {
            eqns.push(0);
        }
        var j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = this.elements[i + 3 * j];
            }
        }
        eqns[3 + 6 * 0] = 1;
        eqns[3 + 6 * 1] = 0;
        eqns[3 + 6 * 2] = 0;
        eqns[4 + 6 * 0] = 0;
        eqns[4 + 6 * 1] = 1;
        eqns[4 + 6 * 2] = 0;
        eqns[5 + 6 * 0] = 0;
        eqns[5 + 6 * 1] = 0;
        eqns[5 + 6 * 2] = 1;
        var n = 3;
        var k = n;
        var np;
        var kp = nc;
        var p;
        do {
            i = k - n;
            if (eqns[i + nc * i] === 0) {
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] !== 0) {
                        np = kp;
                        do {
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np);
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] !== 0) {
                for (j = i + 1; j < k; j++) {
                    var multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        p = kp - np;
                        eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np);
                }
            }
        } while (--n);
        i = 2;
        do {
            j = i - 1;
            do {
                var multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                np = nc;
                do {
                    p = nc - np;
                    eqns[p + nc * j] = eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                } while (--np);
            } while (j--);
        } while (--i);
        i = 2;
        do {
            var multiplier = 1 / eqns[i + nc * i];
            np = nc;
            do {
                p = nc - np;
                eqns[p + nc * i] = eqns[p + nc * i] * multiplier;
            } while (--np);
        } while (i--);
        i = 2;
        do {
            j = 2;
            do {
                p = eqns[nr + j + nc * i];
                if (isNaN(p) || p === Infinity) {
                    throw new Error('Could not reverse! A=[' + this.toString() + ']');
                }
                target.e(i, j, p);
            } while (j--);
        } while (i--);
        return target;
    };
    Mat3.prototype.setRotationFromQuaternion = function (q) {
        var x = q.x, y = q.y, z = q.z, w = q.w, x2 = x + x, y2 = y + y, z2 = z + z, xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2, e = this.elements;
        e[3 * 0 + 0] = 1 - (yy + zz);
        e[3 * 0 + 1] = xy - wz;
        e[3 * 0 + 2] = xz + wy;
        e[3 * 1 + 0] = xy + wz;
        e[3 * 1 + 1] = 1 - (xx + zz);
        e[3 * 1 + 2] = yz - wx;
        e[3 * 2 + 0] = xz - wy;
        e[3 * 2 + 1] = yz + wx;
        e[3 * 2 + 2] = 1 - (xx + yy);
        return this;
    };
    Mat3.prototype.transpose = function (target) {
        target = target || new Mat3();
        var Mt = target.elements, M = this.elements;
        for (var i = 0; i !== 3; i++) {
            for (var j = 0; j !== 3; j++) {
                Mt[3 * i + j] = M[3 * j + i];
            }
        }
        return target;
    };
    return Mat3;
}());
export { Mat3 };
