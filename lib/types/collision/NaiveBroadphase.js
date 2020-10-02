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
import { Broadphase } from '../collision/Broadphase';
var NaiveBroadphase = (function (_super) {
    __extends(NaiveBroadphase, _super);
    function NaiveBroadphase() {
        return _super.call(this) || this;
    }
    NaiveBroadphase.prototype.collisionPairs = function (world, pairs1, pairs2) {
        var bodies = world.bodies, n = bodies.length;
        var i, j, bi, bj;
        for (i = 0; i !== n; i++) {
            for (j = 0; j !== i; j++) {
                bi = bodies[i];
                bj = bodies[j];
                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }
                this.intersectionTest(bi, bj, pairs1, pairs2);
            }
        }
    };
    NaiveBroadphase.prototype.aabbQuery = function (world, aabb, result) {
        result = result || [];
        for (var i = 0; i < world.bodies.length; i++) {
            var b = world.bodies[i];
            if (b.aabbNeedsUpdate) {
                b.computeAABB();
            }
            if (b.aabb.overlaps(aabb)) {
                result.push(b);
            }
        }
        return result;
    };
    return NaiveBroadphase;
}(Broadphase));
export { NaiveBroadphase };
