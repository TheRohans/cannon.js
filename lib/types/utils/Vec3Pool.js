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
import { Vec3 } from '../math/Vec3';
import { Pool } from './Pool';
var Vec3Pool = (function (_super) {
    __extends(Vec3Pool, _super);
    function Vec3Pool() {
        var _this = _super.call(this) || this;
        _this.type = Vec3;
        return _this;
    }
    Vec3Pool.prototype.constructObject = function () {
        return new Vec3();
    };
    return Vec3Pool;
}(Pool));
export { Vec3Pool };
