var ArrayCollisionMatrix = (function () {
    function ArrayCollisionMatrix() {
        this.matrix = [];
    }
    ArrayCollisionMatrix.prototype.get = function (ii, ji) {
        var i = ii.index;
        var j = ji.index;
        if (j > i) {
            var temp = j;
            j = i;
            i = temp;
        }
        return this.matrix[(i * (i + 1) >> 1) + j - 1];
    };
    ArrayCollisionMatrix.prototype.set = function (ii, ji, value) {
        var i = ii.index;
        var j = ji.index;
        if (j > i) {
            var temp = j;
            j = i;
            i = temp;
        }
        this.matrix[(i * (i + 1) >> 1) + j - 1] = value ? 1 : 0;
    };
    ArrayCollisionMatrix.prototype.reset = function () {
        for (var i = 0, l = this.matrix.length; i !== l; i++) {
            this.matrix[i] = 0;
        }
    };
    ArrayCollisionMatrix.prototype.setNumObjects = function (n) {
        this.matrix.length = n * (n - 1) >> 1;
    };
    return ArrayCollisionMatrix;
}());
export { ArrayCollisionMatrix };
