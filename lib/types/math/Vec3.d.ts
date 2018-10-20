import { Mat3 } from './Mat3';
export declare class Vec3 {
    x: number;
    y: number;
    z: number;
    constructor(x?: number, y?: number, z?: number);
    cross(v: Vec3, target?: Vec3): Vec3;
    set(x: number, y: number, z: number): this;
    setZero(): void;
    vadd(v: Vec3, target?: Vec3): Vec3;
    vsub(v: Vec3, target?: Vec3): Vec3;
    crossmat(): Mat3;
    normalize(): number;
    unit(target: Vec3): Vec3;
    norm(): number;
    length(): number;
    norm2(): number;
    distanceTo(p: Vec3): number;
    distanceSquared(p: Vec3): number;
    mult(scalar: number, target?: Vec3): Vec3;
    vmul(vector: Vec3, target: Vec3): Vec3;
    scale(scalar: number, target?: Vec3): Vec3;
    addScaledVector(scalar: number, vector: Vec3, target: Vec3): Vec3;
    dot(v: Vec3): number;
    isZero(): boolean;
    negate(target?: Vec3): Vec3;
    tangents(t1: Vec3, t2: Vec3): void;
    toString(): string;
    toArray(): number[];
    copy(source: Vec3): Vec3;
    lerp(v: Vec3, t: number, target: Vec3): void;
    almostEquals(v: Vec3, precision?: number): boolean;
    almostZero(precision?: number): boolean;
    isAntiparallelTo(v: Vec3, precision?: number): boolean;
    clone(): Vec3;
}
export declare class Vec3Consts {
    static ZERO: Vec3;
    static UNIT_X: Vec3;
    static UNIT_Y: Vec3;
    static UNIT_Z: Vec3;
}
