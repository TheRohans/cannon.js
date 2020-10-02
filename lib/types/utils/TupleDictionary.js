var TupleDictionary = (function () {
    function TupleDictionary() {
        this.data = { keys: [] };
    }
    TupleDictionary.prototype.get = function (i, j) {
        if (i > j) {
            var temp = j;
            j = i;
            i = temp;
        }
        return this.data[i + '-' + j];
    };
    TupleDictionary.prototype.set = function (i, j, value) {
        if (i > j) {
            var temp = j;
            j = i;
            i = temp;
        }
        var key = i + '-' + j;
        if (!this.get(i, j)) {
            this.data.keys.push(key);
        }
        this.data[key] = value;
    };
    TupleDictionary.prototype.reset = function () {
        var data = this.data, keys = data.keys;
        while (keys.length > 0) {
            var key = keys.pop();
            delete data[key];
        }
    };
    return TupleDictionary;
}());
export { TupleDictionary };
