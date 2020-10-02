var EventTarget = (function () {
    function EventTarget() {
    }
    EventTarget.prototype.addEventListener = function (type, listener) {
        if (this._listeners === undefined) {
            this._listeners = {};
        }
        var listeners = this._listeners;
        if (listeners[type] === undefined) {
            listeners[type] = [];
        }
        if (listeners[type].indexOf(listener) === -1) {
            listeners[type].push(listener);
        }
        return this;
    };
    EventTarget.prototype.hasEventListener = function (type, listener) {
        if (this._listeners === undefined) {
            return false;
        }
        var listeners = this._listeners;
        if (listeners[type] !== undefined && listeners[type].indexOf(listener) !== -1) {
            return true;
        }
        return false;
    };
    EventTarget.prototype.hasAnyEventListener = function (type) {
        if (this._listeners === undefined) {
            return false;
        }
        var listeners = this._listeners;
        return (listeners[type] !== undefined);
    };
    EventTarget.prototype.removeEventListener = function (type, listener) {
        if (this._listeners === undefined) {
            return this;
        }
        var listeners = this._listeners;
        if (listeners[type] === undefined) {
            return this;
        }
        var index = listeners[type].indexOf(listener);
        if (index !== -1) {
            listeners[type].splice(index, 1);
        }
        return this;
    };
    EventTarget.prototype.dispatchEvent = function (event) {
        if (this._listeners === undefined) {
            return this;
        }
        var listeners = this._listeners;
        var listenerArray = listeners[event.type];
        if (listenerArray !== undefined) {
            event.target = this;
            for (var i = 0, l = listenerArray.length; i < l; i++) {
                listenerArray[i].call(this, event);
            }
        }
        return this;
    };
    return EventTarget;
}());
export { EventTarget };
