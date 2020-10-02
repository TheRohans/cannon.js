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
import { Broadphase } from './Broadphase';
import { Vec3 } from '../math/Vec3';
import { Shape } from '../shapes/Shape';
var GridBroadphase = (function (_super) {
    __extends(GridBroadphase, _super);
    function GridBroadphase(aabbMin, aabbMax, nx, ny, nz) {
        if (nx === void 0) { nx = 10; }
        if (ny === void 0) { ny = 10; }
        if (nz === void 0) { nz = 10; }
        var _this = _super.call(this) || this;
        _this.GridBroadphase_collisionPairs_d = new Vec3();
        _this.GridBroadphase_collisionPairs_binPos = new Vec3();
        _this.aabbMin = aabbMin || new Vec3(100, 100, 100);
        _this.aabbMax = aabbMax || new Vec3(-100, -100, -100);
        var nbins = _this.nx * _this.ny * _this.nz;
        if (nbins <= 0) {
            throw new Error('GridBroadphase: Each dimensions n must be >0');
        }
        _this.bins = [];
        _this.binLengths = [];
        _this.bins.length = nbins;
        _this.binLengths.length = nbins;
        for (var i = 0; i < nbins; i++) {
            _this.bins[i] = [];
            _this.binLengths[i] = 0;
        }
        return _this;
    }
    GridBroadphase.prototype.collisionPairs = function (world, pairs1, pairs2) {
        var N = world.numObjects(), bodies = world.bodies;
        var max = this.aabbMax, min = this.aabbMin;
        var nx = this.nx, ny = this.ny, nz = this.nz;
        var xstep = ny * nz;
        var ystep = nz;
        var zstep = 1;
        var xmax = max.x, ymax = max.y, zmax = max.z, xmin = min.x, ymin = min.y, zmin = min.z;
        var xmult = nx / (xmax - xmin), ymult = ny / (ymax - ymin), zmult = nz / (zmax - zmin);
        var binsizeX = (xmax - xmin) / nx, binsizeY = (ymax - ymin) / ny, binsizeZ = (zmax - zmin) / nz;
        var binRadius = Math.sqrt(binsizeX * binsizeX + binsizeY * binsizeY + binsizeZ * binsizeZ) * 0.5;
        var types = Shape.types;
        var SPHERE = types.SPHERE, PLANE = types.PLANE, BOX = types.BOX, COMPOUND = types.COMPOUND, CONVEXPOLYHEDRON = types.CONVEXPOLYHEDRON;
        var bins = this.bins, binLengths = this.binLengths, Nbins = this.bins.length;
        for (var i = 0; i !== Nbins; i++) {
            binLengths[i] = 0;
        }
        var ceil = Math.ceil;
        var addBoxToBins = function (x0, y0, z0, x1, y1, z1, bi) {
            var xoff0 = ((x0 - xmin) * xmult) | 0, yoff0 = ((y0 - ymin) * ymult) | 0, zoff0 = ((z0 - zmin) * zmult) | 0, xoff1 = ceil((x1 - xmin) * xmult), yoff1 = ceil((y1 - ymin) * ymult), zoff1 = ceil((z1 - zmin) * zmult);
            if (xoff0 < 0) {
                xoff0 = 0;
            }
            else if (xoff0 >= nx) {
                xoff0 = nx - 1;
            }
            if (yoff0 < 0) {
                yoff0 = 0;
            }
            else if (yoff0 >= ny) {
                yoff0 = ny - 1;
            }
            if (zoff0 < 0) {
                zoff0 = 0;
            }
            else if (zoff0 >= nz) {
                zoff0 = nz - 1;
            }
            if (xoff1 < 0) {
                xoff1 = 0;
            }
            else if (xoff1 >= nx) {
                xoff1 = nx - 1;
            }
            if (yoff1 < 0) {
                yoff1 = 0;
            }
            else if (yoff1 >= ny) {
                yoff1 = ny - 1;
            }
            if (zoff1 < 0) {
                zoff1 = 0;
            }
            else if (zoff1 >= nz) {
                zoff1 = nz - 1;
            }
            xoff0 *= xstep;
            yoff0 *= ystep;
            zoff0 *= zstep;
            xoff1 *= xstep;
            yoff1 *= ystep;
            zoff1 *= zstep;
            for (var xoff = xoff0; xoff <= xoff1; xoff += xstep) {
                for (var yoff = yoff0; yoff <= yoff1; yoff += ystep) {
                    for (var zoff = zoff0; zoff <= zoff1; zoff += zstep) {
                        var idx = xoff + yoff + zoff;
                        bins[idx][binLengths[idx]++] = bi;
                    }
                }
            }
        };
        for (var i = 0; i !== N; i++) {
            var bi = bodies[i];
            var si = bi.shapes[0];
            switch (si.type) {
                case SPHERE:
                    var ssi = si;
                    var x = bi.position.x, y = bi.position.y, z = bi.position.z;
                    var r = ssi.radius;
                    addBoxToBins(x - r, y - r, z - r, x + r, y + r, z + r, bi);
                    break;
                case PLANE:
                    var psi = si;
                    if (psi.worldNormalNeedsUpdate) {
                        psi.computeWorldNormal(bi.quaternion);
                    }
                    var planeNormal = psi.worldNormal;
                    var xreset = xmin + binsizeX * 0.5 - bi.position.x, yreset = ymin + binsizeY * 0.5 - bi.position.y, zreset = zmin + binsizeZ * 0.5 - bi.position.z;
                    var d = this.GridBroadphase_collisionPairs_d;
                    d.set(xreset, yreset, zreset);
                    for (var xi = 0, xoff = 0; xi !== nx; xi++, xoff += xstep, d.y = yreset, d.x += binsizeX) {
                        for (var yi = 0, yoff = 0; yi !== ny; yi++, yoff += ystep, d.z = zreset, d.y += binsizeY) {
                            for (var zi = 0, zoff = 0; zi !== nz; zi++, zoff += zstep, d.z += binsizeZ) {
                                if (d.dot(planeNormal) < binRadius) {
                                    var idx = xoff + yoff + zoff;
                                    bins[idx][binLengths[idx]++] = bi;
                                }
                            }
                        }
                    }
                    break;
                default:
                    if (bi.aabbNeedsUpdate) {
                        bi.computeAABB();
                    }
                    addBoxToBins(bi.aabb.lowerBound.x, bi.aabb.lowerBound.y, bi.aabb.lowerBound.z, bi.aabb.upperBound.x, bi.aabb.upperBound.y, bi.aabb.upperBound.z, bi);
                    break;
            }
        }
        for (var i = 0; i !== Nbins; i++) {
            var binLength = binLengths[i];
            if (binLength > 1) {
                var bin = bins[i];
                for (var xi = 0; xi !== binLength; xi++) {
                    var bi = bin[xi];
                    for (var yi = 0; yi !== xi; yi++) {
                        var bj = bin[yi];
                        if (this.needBroadphaseCollision(bi, bj)) {
                            this.intersectionTest(bi, bj, pairs1, pairs2);
                        }
                    }
                }
            }
        }
        this.makePairsUnique(pairs1, pairs2);
    };
    return GridBroadphase;
}(Broadphase));
export { GridBroadphase };
