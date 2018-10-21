export interface MaterialOptions {
    name?: string;
    friction?: number;
    restitution?: number;
}
export declare class Material {
    static idCounter: number;
    name: string;
    id: number;
    friction: number;
    restitution: number;
    constructor(options?: MaterialOptions);
}
