var Solver = (function () {
    function Solver() {
        this.equations = [];
    }
    Solver.prototype.solve = function (dt, world) {
        return 0;
    };
    Solver.prototype.addEquation = function (eq) {
        if (eq.enabled) {
            this.equations.push(eq);
        }
    };
    Solver.prototype.removeEquation = function (eq) {
        var eqs = this.equations;
        var i = eqs.indexOf(eq);
        if (i !== -1) {
            eqs.splice(i, 1);
        }
    };
    Solver.prototype.removeAllEquations = function () {
        this.equations.length = 0;
    };
    return Solver;
}());
export { Solver };
