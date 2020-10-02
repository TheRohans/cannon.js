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
var SAPBroadphase = (function (_super) {
    __extends(SAPBroadphase, _super);
    function SAPBroadphase(world) {
        var _this = _super.call(this, world) || this;
        Broadphase.apply(_this);
        _this.axisList = [];
        _this.axisIndex = 0;
        var axisList = _this.axisList;
        _this._addBodyHandler = function (e) {
            axisList.push(e.body);
        };
        _this._removeBodyHandler = function (e) {
            var idx = axisList.indexOf(e.body);
            if (idx !== -1) {
                axisList.splice(idx, 1);
            }
        };
        if (world) {
            _this.setWorld(world);
        }
        return _this;
    }
    SAPBroadphase.prototype.setWorld = function (world) {
        this.axisList.length = 0;
        for (var i = 0; i < world.bodies.length; i++) {
            this.axisList.push(world.bodies[i]);
        }
        world.removeEventListener('addBody', this._addBodyHandler);
        world.removeEventListener('removeBody', this._removeBodyHandler);
        world.addEventListener('addBody', this._addBodyHandler);
        world.addEventListener('removeBody', this._removeBodyHandler);
        this.world = world;
        this.dirty = true;
    };
    SAPBroadphase.prototype.insertionSortX = function (a) {
        var i, j, l;
        for (i = 1, l = a.length; i < l; i++) {
            var v = a[i];
            for (j = i - 1; j >= 0; j--) {
                if (a[j].aabb.lowerBound.x <= v.aabb.lowerBound.x) {
                    break;
                }
                a[j + 1] = a[j];
            }
            a[j + 1] = v;
        }
        return a;
    };
    SAPBroadphase.prototype.insertionSortY = function (a) {
        var i, j, l;
        for (i = 1, l = a.length; i < l; i++) {
            var v = a[i];
            for (j = i - 1; j >= 0; j--) {
                if (a[j].aabb.lowerBound.y <= v.aabb.lowerBound.y) {
                    break;
                }
                a[j + 1] = a[j];
            }
            a[j + 1] = v;
        }
        return a;
    };
    SAPBroadphase.prototype.insertionSortZ = function (a) {
        var i, j, l;
        for (i = 1, l = a.length; i < l; i++) {
            var v = a[i];
            for (j = i - 1; j >= 0; j--) {
                if (a[j].aabb.lowerBound.z <= v.aabb.lowerBound.z) {
                    break;
                }
                a[j + 1] = a[j];
            }
            a[j + 1] = v;
        }
        return a;
    };
    SAPBroadphase.prototype.collisionPairs = function (world, p1, p2) {
        var bodies = this.axisList, N = bodies.length, axisIndex = this.axisIndex;
        var i = N;
        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }
        while (i--) {
            var bi = bodies[i];
            var j = i;
            while (j-- && i > 0) {
                var bj = bodies[j];
                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }
                if (!this.checkBounds(bi, bj, axisIndex)) {
                    break;
                }
                this.intersectionTest(bi, bj, p1, p2);
            }
        }
    };
    SAPBroadphase.prototype.sortList = function () {
        var axisList = this.axisList;
        var axisIndex = this.axisIndex;
        var N = axisList.length;
        for (var i = 0; i !== N; i++) {
            var bi = axisList[i];
            if (bi.aabbNeedsUpdate) {
                bi.computeAABB();
            }
        }
        if (axisIndex === 0) {
            this.insertionSortX(axisList);
        }
        else if (axisIndex === 1) {
            this.insertionSortY(axisList);
        }
        else if (axisIndex === 2) {
            this.insertionSortZ(axisList);
        }
    };
    SAPBroadphase.prototype.checkBounds = function (bi, bj, axisIndex) {
        var biPos;
        var bjPos;
        if (axisIndex === 0) {
            biPos = bi.position.x;
            bjPos = bj.position.x;
        }
        else if (axisIndex === 1) {
            biPos = bi.position.y;
            bjPos = bj.position.y;
        }
        else if (axisIndex === 2) {
            biPos = bi.position.z;
            bjPos = bj.position.z;
        }
        var ri = bi.boundingRadius, rj = bj.boundingRadius, boundA2 = biPos + ri, boundB1 = bjPos - rj;
        return boundB1 < boundA2;
    };
    SAPBroadphase.prototype.autoDetectAxis = function () {
        var sumX = 0, sumX2 = 0, sumY = 0, sumY2 = 0, sumZ = 0, sumZ2 = 0;
        var bodies = this.axisList, N = bodies.length, invN = 1 / N;
        for (var i = 0; i !== N; i++) {
            var b = bodies[i];
            var centerX = b.position.x;
            sumX += centerX;
            sumX2 += centerX * centerX;
            var centerY = b.position.y;
            sumY += centerY;
            sumY2 += centerY * centerY;
            var centerZ = b.position.z;
            sumZ += centerZ;
            sumZ2 += centerZ * centerZ;
        }
        var varianceX = sumX2 - sumX * sumX * invN, varianceY = sumY2 - sumY * sumY * invN, varianceZ = sumZ2 - sumZ * sumZ * invN;
        if (varianceX > varianceY) {
            if (varianceX > varianceZ) {
                this.axisIndex = 0;
            }
            else {
                this.axisIndex = 2;
            }
        }
        else if (varianceY > varianceZ) {
            this.axisIndex = 1;
        }
        else {
            this.axisIndex = 2;
        }
    };
    SAPBroadphase.prototype.aabbQuery = function (world, aabb, result) {
        result = result || [];
        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }
        var axisIndex = this.axisIndex;
        var axis = 'x';
        if (axisIndex === 1) {
            axis = 'y';
        }
        if (axisIndex === 2) {
            axis = 'z';
        }
        var axisList = this.axisList;
        var lower = aabb.lowerBound[axis];
        var upper = aabb.upperBound[axis];
        for (var i = 0; i < axisList.length; i++) {
            var b = axisList[i];
            if (b.aabbNeedsUpdate) {
                b.computeAABB();
            }
            if (b.aabb.overlaps(aabb)) {
                result.push(b);
            }
        }
        return result;
    };
    return SAPBroadphase;
}(Broadphase));
export { SAPBroadphase };
