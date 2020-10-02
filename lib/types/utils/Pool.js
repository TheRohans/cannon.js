var Pool = (function () {
    function Pool() {
        this.objects = [];
        this.type = Object;
    }
    Pool.prototype.release = function () {
        var obj = [];
        for (var _i = 0; _i < arguments.length; _i++) {
            obj[_i] = arguments[_i];
        }
        var Nargs = obj.length;
        for (var i = 0; i !== Nargs; i++) {
            this.objects.push(obj[i]);
        }
        return this;
    };
    Pool.prototype.get = function () {
        if (this.objects.length === 0) {
            return this.constructObject();
        }
        else {
            return this.objects.pop();
        }
    };
    Pool.prototype.constructObject = function () {
        throw new Error('constructObject() not implemented in this Pool subclass yet!');
    };
    Pool.prototype.resize = function (size) {
        var objects = this.objects;
        while (objects.length > size) {
            objects.pop();
        }
        while (objects.length < size) {
            objects.push(this.constructObject());
        }
        return this;
    };
    return Pool;
}());
export { Pool };
