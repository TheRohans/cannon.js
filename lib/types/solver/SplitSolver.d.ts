import { Solver } from './Solver';
import { Body } from '../objects/Body';
import { World } from '../world/World';
import { Equation } from '../equations/Equation';
export declare class SplitNode {
    body: Body;
    children: Body[];
    eqs: Equation[];
    visited: boolean;
}
export declare class SplitSolver extends Solver {
    iterations: number;
    tolerance: number;
    subsolver: SplitSolver;
    nodes: any[];
    nodePool: any[];
    constructor(subsolver: SplitSolver);
    private SplitSolver_solve_nodes;
    private SplitSolver_solve_nodePool;
    private SplitSolver_solve_eqs;
    private SplitSolver_solve_dummyWorld;
    createNode(): SplitNode;
    solve(dt: number, world: World): number;
}
