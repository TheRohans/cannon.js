import { Vec3 } from './Vec3';
export declare class Quaternion {
    x: number;
    y: number;
    z: number;
    w: number;
    constructor(x?: number, y?: number, z?: number, w?: number);
    set(x: number, y: number, z: number, w: number): this;
    toString(): string;
    toArray(): number[];
    setFromAxisAngle(axis: Vec3, angle: number): this;
    toAxisAngle(targetAxis: Vec3): (number | Vec3)[];
    private sfv_t1;
    private sfv_t2;
    setFromVectors(u: Vec3, v: Vec3): this;
    mult(q: Quaternion, target: Quaternion): Quaternion;
    inverse(target: Quaternion): Quaternion;
    conjugate(target?: Quaternion): Quaternion;
    normalize(): Quaternion;
    normalizeFast(): Quaternion;
    vmult(v: Vec3, target?: Vec3): Vec3;
    copy(source: Quaternion): Quaternion;
    toEuler(target: Vec3, order?: string): void;
    setFromEuler(x: number, y: number, z: number, order?: string): this;
    clone(): Quaternion;
    slerp(toQuat: Quaternion, t: number, target: Quaternion): Quaternion;
    integrate(angularVelocity: Vec3, dt: number, angularFactor: Vec3, target: Quaternion): Quaternion;
}
