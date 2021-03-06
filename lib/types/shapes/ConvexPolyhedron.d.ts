import { Shape } from './Shape';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
export declare class HullResult {
    point: Vec3;
    normal: Vec3;
    depth: number;
}
export declare class ConvexPolyhedron extends Shape {
    vertices: Vec3[];
    worldVertices: Vec3[];
    worldVerticesNeedsUpdate: boolean;
    faces: number[][];
    faceNormals: Vec3[];
    worldFaceNormalsNeedsUpdate: boolean;
    worldFaceNormals: Vec3[];
    uniqueEdges: Vec3[];
    uniqueAxes: Vec3[];
    constructor(points: Vec3[], faces: number[][], uniqueAxes?: Vec3[], faceNormals?: Vec3[]);
    computeEdges(): void;
    computeNormals(): void;
    static computeNormal(va: Vec3, vb: Vec3, vc: Vec3, target?: Vec3): void;
    getFaceNormal(i: number, target?: Vec3): Vec3;
    findClosestFace(hull: ConvexPolyhedron, quat: Quaternion, separatingNormal: Vec3, useMin?: boolean): number;
    private cah_WorldNormal;
    clipAgainstHull(posA: Vec3, quatA: Quaternion, hullB: ConvexPolyhedron, posB: Vec3, quatB: Quaternion, separatingNormal: Vec3, minDist: number, maxDist: number, result: HullResult[]): void;
    private fsa_faceANormalWS3;
    private fsa_Worldnormal1;
    private fsa_deltaC;
    private fsa_worldEdge0;
    private fsa_worldEdge1;
    private fsa_Cross;
    findSeparatingAxis(hullB: ConvexPolyhedron, posA: Vec3, quatA: Quaternion, posB: Vec3, quatB: Quaternion, target: Vec3, faceListA?: any[], faceListB?: any[]): boolean;
    testSepAxis(axis: Vec3, hullB: ConvexPolyhedron, posA: Vec3, quatA: Quaternion, posB: Vec3, quatB: Quaternion): [boolean, number];
    cli_aabbmin: Vec3;
    cli_aabbmax: Vec3;
    calculateLocalInertia(mass: number, target: Vec3): void;
    getPlaneConstantOfFace(face_i: number): number;
    private cfah_edge0;
    private cfah_WorldEdge0;
    private cfah_worldPlaneAnormal1;
    private cfah_planeNormalWS1;
    private cfah_worldA1;
    private cfah_localPlaneNormal;
    private cfah_planeNormalWS;
    clipFaceAgainstHull(separatingNormal: Vec3, posA: Vec3, quatA: Quaternion, worldVertsB1: Vec3[], minDist: number, maxDist: number, result: HullResult[]): void;
    clipFaceAgainstPlane(inVertices: Vec3[], outVertices: Vec3[], planeNormal: Vec3, planeConstant: number): Vec3[];
    computeWorldVertices(position: Vec3, quat: Quaternion): void;
    computeLocalAABB(aabbmin: Vec3, aabbmax: Vec3): void;
    computeWorldFaceNormals(quat: Quaternion): void;
    updateBoundingSphereRadius(): void;
    calculateWorldAABB(pos: Vec3, quat: Quaternion, min: Vec3, max: Vec3): void;
    volume(): number;
    getAveragePointLocal(target?: Vec3): Vec3;
    transformAllPoints(offset: Vec3, quat: Quaternion): void;
    private ConvexPolyhedron_pointIsInside;
    private ConvexPolyhedron_vToP;
    private ConvexPolyhedron_vToPointInside;
    pointIsInside(p: Vec3): boolean;
    static project_localAxis: Vec3;
    static project_localOrigin: Vec3;
    static project(hull: ConvexPolyhedron, axis: Vec3, pos: Vec3, quat: Quaternion, result: number[]): void;
}
