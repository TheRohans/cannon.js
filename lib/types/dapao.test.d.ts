import { Vec3 } from './math/Vec3';
export declare const mockCube: () => [Vec3[], Vec3[], number[][]];
export declare const mockBoxHull: (size?: number) => import("./dapao").ConvexPolyhedron;
export declare const mockPolyBox: (sx: number, sy: number, sz: number) => import("./dapao").ConvexPolyhedron;
