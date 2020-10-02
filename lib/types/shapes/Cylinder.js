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
import { ConvexPolyhedron } from './ConvexPolyhedron';
import { Vec3 } from '../math/Vec3';
var Cylinder = (function (_super) {
    __extends(Cylinder, _super);
    function Cylinder(radiusTop, radiusBottom, height, numSegments) {
        var _this = this;
        var N = numSegments;
        var verts = [];
        var axes = [];
        var faces = [];
        var bottomface = [];
        var topface = [];
        var cos = Math.cos;
        var sin = Math.sin;
        verts.push(new Vec3(radiusBottom * cos(0), radiusBottom * sin(0), -height * 0.5));
        bottomface.push(0);
        verts.push(new Vec3(radiusTop * cos(0), radiusTop * sin(0), height * 0.5));
        topface.push(1);
        for (var i = 0; i < N; i++) {
            var theta = 2 * Math.PI / N * (i + 1);
            var thetaN = 2 * Math.PI / N * (i + 0.5);
            if (i < N - 1) {
                verts.push(new Vec3(radiusBottom * cos(theta), radiusBottom * sin(theta), -height * 0.5));
                bottomface.push(2 * i + 2);
                verts.push(new Vec3(radiusTop * cos(theta), radiusTop * sin(theta), height * 0.5));
                topface.push(2 * i + 3);
                faces.push([2 * i + 2, 2 * i + 3, 2 * i + 1, 2 * i]);
            }
            else {
                faces.push([0, 1, 2 * i + 1, 2 * i]);
            }
            if (N % 2 === 1 || i < N / 2) {
                axes.push(new Vec3(cos(thetaN), sin(thetaN), 0));
            }
        }
        faces.push(topface);
        axes.push(new Vec3(0, 0, 1));
        var temp = [];
        for (var i = 0; i < bottomface.length; i++) {
            temp.push(bottomface[bottomface.length - i - 1]);
        }
        faces.push(temp);
        _this = _super.call(this, verts, faces, axes) || this;
        return _this;
    }
    return Cylinder;
}(ConvexPolyhedron));
export { Cylinder };
