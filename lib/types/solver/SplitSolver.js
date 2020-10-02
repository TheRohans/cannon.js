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
import { Solver } from './Solver';
import { Body } from '../objects/Body';
var SplitSolver = (function (_super) {
    __extends(SplitSolver, _super);
    function SplitSolver(subsolver) {
        var _this = _super.call(this) || this;
        _this.SplitSolver_solve_nodes = [];
        _this.SplitSolver_solve_nodePool = [];
        _this.SplitSolver_solve_eqs = [];
        _this.SplitSolver_solve_dummyWorld = { bodies: [] };
        _this.iterations = 10;
        _this.tolerance = 1e-7;
        _this.subsolver = subsolver;
        _this.nodes = [];
        _this.nodePool = [];
        while (_this.nodePool.length < 128) {
            _this.nodePool.push(_this.createNode());
        }
        return _this;
    }
    SplitSolver.prototype.createNode = function () {
        return { body: null, children: [], eqs: [], visited: false };
    };
    SplitSolver.prototype.solve = function (dt, world) {
        var nodes = this.SplitSolver_solve_nodes, nodePool = this.nodePool, bodies = world.bodies, equations = this.equations, Neq = equations.length, Nbodies = bodies.length, subsolver = this.subsolver;
        while (nodePool.length < Nbodies) {
            nodePool.push(this.createNode());
        }
        nodes.length = Nbodies;
        for (var i = 0; i < Nbodies; i++) {
            nodes[i] = nodePool[i];
        }
        for (var i = 0; i !== Nbodies; i++) {
            var node = nodes[i];
            node.body = bodies[i];
            node.children.length = 0;
            node.eqs.length = 0;
            node.visited = false;
        }
        for (var k = 0; k !== Neq; k++) {
            var eq = equations[k], ii = bodies.indexOf(eq.bi), j = bodies.indexOf(eq.bj), ni = nodes[ii], nj = nodes[j];
            ni.children.push(nj);
            ni.eqs.push(eq);
            nj.children.push(ni);
            nj.eqs.push(eq);
        }
        var child, n = 0, eqs = this.SplitSolver_solve_eqs;
        subsolver.tolerance = this.tolerance;
        subsolver.iterations = this.iterations;
        var dummyWorld = this.SplitSolver_solve_dummyWorld;
        while ((child = getUnvisitedNode(nodes))) {
            eqs.length = 0;
            dummyWorld.bodies.length = 0;
            bfs(child, visitFunc, dummyWorld.bodies, eqs);
            var Neqs = eqs.length;
            eqs = eqs.sort(sortById);
            for (var i = 0; i !== Neqs; i++) {
                subsolver.addEquation(eqs[i]);
            }
            var iter = subsolver.solve(dt, dummyWorld);
            subsolver.removeAllEquations();
            n++;
        }
        return n;
    };
    return SplitSolver;
}(Solver));
export { SplitSolver };
function getUnvisitedNode(nodes) {
    var Nnodes = nodes.length;
    for (var i = 0; i !== Nnodes; i++) {
        var node = nodes[i];
        if (!node.visited && !(node.body.type & Body.STATIC)) {
            return node;
        }
    }
    return undefined;
}
var queue = [];
function bfs(root, visitFunc1, bds, eqs) {
    queue.push(root);
    root.visited = true;
    visitFunc1(root, bds, eqs);
    while (queue.length) {
        var node = queue.pop();
        var child = void 0;
        while ((child = getUnvisitedNode(node.children))) {
            child.visited = true;
            visitFunc1(child, bds, eqs);
            queue.push(child);
        }
    }
}
function visitFunc(node, bds, eqs) {
    bds.push(node.body);
    var Neqs = node.eqs.length;
    for (var i = 0; i !== Neqs; i++) {
        var eq = node.eqs[i];
        if (eqs.indexOf(eq) === -1) {
            eqs.push(eq);
        }
    }
}
function sortById(a, b) {
    return b.id - a.id;
}
