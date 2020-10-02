var OverlapKeeper = (function () {
    function OverlapKeeper() {
        this.current = [];
        this.previous = [];
    }
    OverlapKeeper.prototype.getKey = function (i, j) {
        if (j < i) {
            var temp = j;
            j = i;
            i = temp;
        }
        return (i << 16) | j;
    };
    OverlapKeeper.prototype.set = function (i, jj) {
        var key = this.getKey(i, jj);
        var current = this.current;
        var index = 0;
        while (key > current[index]) {
            index++;
        }
        if (key === current[index]) {
            return;
        }
        for (var j = current.length - 1; j >= index; j--) {
            current[j + 1] = current[j];
        }
        current[index] = key;
    };
    OverlapKeeper.prototype.tick = function () {
        var tmp = this.current;
        this.current = this.previous;
        this.previous = tmp;
        this.current.length = 0;
    };
    OverlapKeeper.prototype.getDiff = function (additions, removals) {
        var a = this.current;
        var b = this.previous;
        var al = a.length;
        var bl = b.length;
        var j = 0;
        var found = false;
        for (var i = 0; i < al; i++) {
            var keyA = a[i];
            while (keyA > b[j]) {
                j++;
            }
            found = keyA === b[j];
            if (!found) {
                unpackAndPush(additions, keyA);
            }
        }
        j = 0;
        found = false;
        for (var i = 0; i < bl; i++) {
            var keyB = b[i];
            while (keyB > a[j]) {
                j++;
            }
            found = a[j] === keyB;
            if (!found) {
                unpackAndPush(removals, keyB);
            }
        }
    };
    return OverlapKeeper;
}());
export { OverlapKeeper };
function unpackAndPush(arr, key) {
    arr.push((key & 0xFFFF0000) >> 16, key & 0x0000FFFF);
}
