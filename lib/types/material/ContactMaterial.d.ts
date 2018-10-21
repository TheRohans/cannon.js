import { Material } from './Material';
export interface ContactMaterialOptions {
    friction?: number;
    restitution?: number;
    contactEquationStiffness?: number;
    contactEquationRelaxation?: number;
    frictionEquationStiffness?: number;
    frictionEquationRelaxation?: number;
}
export declare class ContactMaterial {
    static idCounter: number;
    id: number;
    materials: [Material, Material];
    friction: number;
    restitution: number;
    contactEquationStiffness: number;
    contactEquationRelaxation: number;
    frictionEquationStiffness: number;
    frictionEquationRelaxation: number;
    constructor(m1?: Material, m2?: Material, options?: ContactMaterialOptions);
}
