var Constraint = (function () {
    function Constraint(bodyA, bodyB, options) {
        if (options === void 0) { options = {}; }
        options = Object.assign({
            collideConnected: true,
            wakeUpBodies: true,
        }, options);
        this.equations = [];
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.id = Constraint.idCounter++;
        this.collideConnected = options.collideConnected;
        if (options.wakeUpBodies) {
            if (bodyA) {
                bodyA.wakeUp();
            }
            if (bodyB) {
                bodyB.wakeUp();
            }
        }
    }
    Constraint.prototype.update = function () {
        throw new Error('method update() not implmemented in this Constraint subclass!');
    };
    Constraint.prototype.enable = function () {
        var eqs = this.equations;
        for (var i = 0; i < eqs.length; i++) {
            eqs[i].enabled = true;
        }
    };
    Constraint.prototype.disable = function () {
        var eqs = this.equations;
        for (var i = 0; i < eqs.length; i++) {
            eqs[i].enabled = false;
        }
    };
    Constraint.idCounter = 0;
    return Constraint;
}());
export { Constraint };
