import { Vec3 } from './Vec3';
import { Quaternion } from './Quaternion';
export declare class Transform {
    position: Vec3;
    quaternion: Quaternion;
    constructor(options?: any);
    static _plfTempQuat: Quaternion;
    static pointToLocalFrame(position: Vec3, quaternion: Quaternion, worldPoint: Vec3, result: Vec3): Vec3;
    static pointToWorldFrame(position: Vec3, quaternion: Quaternion, localPoint: Vec3, result: Vec3): Vec3;
    static vectorToWorldFrame(quaternion: Quaternion, localVector: Vec3, result: Vec3): Vec3;
    static vectorToLocalFrame(position: Vec3, quaternion: Quaternion, worldVector: Vec3, result: Vec3): Vec3;
    pointToLocal(worldPoint: Vec3, result: Vec3): Vec3;
    pointToWorld(localPoint: Vec3, result: Vec3): Vec3;
}
