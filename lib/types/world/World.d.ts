import { EventTarget } from '../utils/EventTarget';
import { Vec3 } from '../math/Vec3';
import { Material } from '../material/Material';
import { ContactMaterial } from '../material/ContactMaterial';
import { TupleDictionary } from '../utils/TupleDictionary';
import { Body } from '../objects/Body';
import { RaycastResult } from '../collision/RaycastResult';
import { Shape } from '../shapes/Shape';
import { Constraint } from '../constraints/Constraint';
import { Broadphase } from '../collision/Broadphase';
import { Solver } from '../solver/Solver';
import { Narrowphase } from './Narrowphase';
import { FrictionEquation } from '../equations/FrictionEquation';
import { OverlapKeeper } from '../collision/OverlapKeeper';
import { ArrayCollisionMatrix } from '../collision/ArrayCollisionMatrix';
export declare class ContactEvent {
    type: string;
    bodyA: Body;
    bodyB: Body;
}
export declare class ShapeContactEvent {
    type: string;
    bodyA: Body;
    bodyB: Body;
    shapeA: Shape;
    shapeB: Shape;
}
export declare class CollideEvent {
    type: string;
    body: Body;
    contact: any;
}
export declare class World extends EventTarget {
    dt: number;
    allowSleep: boolean;
    contacts: any[];
    frictionEquations: FrictionEquation[];
    quatNormalizeSkip: number;
    quatNormalizeFast: false;
    time: number;
    stepnumber: number;
    default_dt: number;
    nextId: number;
    gravity: Vec3;
    broadphase: Broadphase;
    bodies: Body[];
    solver: Solver;
    constraints: Constraint[];
    narrowphase: Narrowphase;
    collisionMatrix: ArrayCollisionMatrix;
    collisionMatrixPrevious: ArrayCollisionMatrix;
    bodyOverlapKeeper: OverlapKeeper;
    shapeOverlapKeeper: OverlapKeeper;
    materials: Material[];
    contactmaterials: ContactMaterial[];
    defaultMaterial: Material;
    contactMaterialTable: TupleDictionary;
    defaultContactMaterial: ContactMaterial;
    doProfiling: boolean;
    profile: {
        solve: number;
        makeContactConstraints: number;
        broadphase: number;
        integrate: number;
        narrowphase: number;
    };
    accumulator: number;
    subsystems: any[];
    addBodyEvent: {
        type: string;
        body: Body;
    };
    removeBodyEvent: {
        type: string;
        body: Body;
    };
    idToBodyMap: any;
    constructor(options?: any);
    private tmpRay;
    getContactMaterial(m1: Material, m2: Material): ContactMaterial;
    numObjects(): number;
    collisionMatrixTick(): void;
    addBody(body: Body): void;
    addConstraint(c: Constraint): void;
    removeConstraint(c: Constraint): void;
    rayTest(from: Vec3, to: Vec3, result: RaycastResult): void;
    raycastAll(from: Vec3, to: Vec3, options: any, callback: Function): boolean;
    raycastAny(from: Vec3, to: Vec3, options: any, result: RaycastResult): boolean;
    raycastClosest(from: Vec3, to: Vec3, options: any, result: RaycastResult): boolean;
    remove(body: Body): void;
    removeBody: (body: Body) => void;
    getBodyById(id: number): Body;
    getShapeById(id: number): Shape;
    addMaterial(m: Material): void;
    addContactMaterial(cmat: ContactMaterial): void;
    step(dt: number, timeSinceLastCalled?: number, maxSubSteps?: number): void;
    private World_step_preStepEvent;
    private World_step_collideEvent;
    private World_step_oldContacts;
    private World_step_frictionEquationPool;
    private World_step_p1;
    private World_step_p2;
    private World_step_postStepEvent;
    internalStep(dt: number): void;
    clearForces(): void;
    emitContactEvents: () => () => void;
}
