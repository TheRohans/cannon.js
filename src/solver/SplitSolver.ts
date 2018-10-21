import { Solver } from './Solver';
import { Body } from '../objects/Body';
import { World } from '../world/World';
import { Equation } from '../equations/Equation';

export class SplitNode {
  body: Body;
  children: Body[];
  eqs: Equation[];
  visited: boolean;
}

/**
 * Splits the equations into islands and solves them independently. Can improve performance.
 * @class SplitSolver
 * @constructor
 * @extends Solver
 * @param {Solver} subsolver
 */
export class SplitSolver extends Solver {
  iterations: number;
  tolerance: number;
  subsolver: SplitSolver;
  nodes: any[];
  nodePool: any[];

  constructor(subsolver: SplitSolver) {
    super();
    this.iterations = 10;
    this.tolerance = 1e-7;
    this.subsolver = subsolver;
    this.nodes = [];
    this.nodePool = [];

    // Create needed nodes, reuse if possible
    while (this.nodePool.length < 128) {
      this.nodePool.push(this.createNode());
    }
  }

  // Returns the number of subsystems
  private SplitSolver_solve_nodes: any[] = []; // All allocated node objects
  private SplitSolver_solve_nodePool: SplitNode[] = []; // All allocated node objects
  private SplitSolver_solve_eqs: Equation[] = [];   // Temp array
  // private SplitSolver_solve_bds = Body[];   // Temp array
  private SplitSolver_solve_dummyWorld: World = <World>{ bodies: [] }; // Temp object

  createNode() {
    return <SplitNode>{ body: null, children: [], eqs: [], visited: false };
  }

  /**
   * Solve the subsystems
   * @method solve
   * @param  {Number} dt
   * @param  {World} world
   */
  solve(dt: number, world: World) {
    const nodes = this.SplitSolver_solve_nodes,
      nodePool = this.nodePool,
      bodies = world.bodies,
      equations = this.equations,
      Neq = equations.length,
      Nbodies = bodies.length,
      subsolver = this.subsolver;

    // Create needed nodes, reuse if possible
    while (nodePool.length < Nbodies) {
      nodePool.push(this.createNode());
    }
    nodes.length = Nbodies;
    for (let i = 0; i < Nbodies; i++) {
      nodes[i] = nodePool[i];
    }

    // Reset node values
    for (let i = 0; i !== Nbodies; i++) {
      const node = nodes[i];
      node.body = bodies[i];
      node.children.length = 0;
      node.eqs.length = 0;
      node.visited = false;
    }
    for (let k = 0; k !== Neq; k++) {
      const eq = equations[k],
        ii = bodies.indexOf(eq.bi),
        j = bodies.indexOf(eq.bj),
        ni = nodes[ii],
        nj = nodes[j];
      ni.children.push(nj);
      ni.eqs.push(eq);
      nj.children.push(ni);
      nj.eqs.push(eq);
    }

    let child, n = 0, eqs = this.SplitSolver_solve_eqs;

    subsolver.tolerance = this.tolerance;
    subsolver.iterations = this.iterations;

    const dummyWorld = this.SplitSolver_solve_dummyWorld;
    while ((child = getUnvisitedNode(nodes))) {
      eqs.length = 0;
      dummyWorld.bodies.length = 0;
      bfs(child, visitFunc, dummyWorld.bodies, eqs);

      const Neqs = eqs.length;

      eqs = eqs.sort(sortById);

      for (let i = 0; i !== Neqs; i++) {
        subsolver.addEquation(eqs[i]);
      }

      const iter = subsolver.solve(dt, dummyWorld);
      subsolver.removeAllEquations();
      n++;
    }

    return n;
  }
}

function getUnvisitedNode(nodes: SplitNode[]): SplitNode {
  const Nnodes = nodes.length;
  for (let i = 0; i !== Nnodes; i++) {
    const node = nodes[i];
    if (!node.visited && !(node.body.type & Body.STATIC)) {
      return node;
    }
  }
  return undefined;
}

const queue: any[] = [];
function bfs(root: SplitNode, visitFunc1: Function, bds: Body[], eqs: Equation[]) {
  queue.push(root);
  root.visited = true;
  visitFunc1(root, bds, eqs);
  while (queue.length) {
    const node = queue.pop();
    // Loop over unvisited child nodes
    let child;
    while ((child = getUnvisitedNode(node.children))) {
      child.visited = true;
      visitFunc1(child, bds, eqs);
      queue.push(child);
    }
  }
}

function visitFunc(node: SplitNode, bds: Body[], eqs: Equation[]) {
  bds.push(node.body);
  const Neqs = node.eqs.length;
  for (let i = 0; i !== Neqs; i++) {
    const eq = node.eqs[i];
    if (eqs.indexOf(eq) === -1) {
      eqs.push(eq);
    }
  }
}

function sortById(a: { id: number }, b: { id: number }) {
  return b.id - a.id;
}
