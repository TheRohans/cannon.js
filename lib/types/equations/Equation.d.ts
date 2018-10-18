import { JacobianElement } from '../math/JacobianElement';
import { Body } from '../objects/Body';
import { Shape } from '../shapes/Shape';
export declare class Equation {
    static EquationId: number;
    id: number;
    minForce: number;
    maxForce: number;
    bi: Body;
    bj: Body;
    si: Shape;
    sj: Shape;
    a: number;
    b: number;
    eps: number;
    jacobianElementA: JacobianElement;
    jacobianElementB: JacobianElement;
    enabled: boolean;
    multiplier: number;
    constructor(bi: Body, bj: Body, minForce?: number, maxForce?: number);
    setSpookParams(stiffness: number, relaxation: number, timeStep: number): void;
    computeB(a: number, b?: number, h?: number): number;
    computeGq(): number;
    private zero;
    computeGW(): number;
    computeGWlambda(): number;
    private iMfi;
    private iMfj;
    private invIi_vmult_taui;
    private invIj_vmult_tauj;
    computeGiMf(): number;
    private tmp;
    computeGiMGt(): number;
    private addToWlambda_temp;
    addToWlambda(deltalambda: number): void;
    computeC(): number;
}
