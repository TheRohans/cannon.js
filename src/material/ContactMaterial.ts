import { Material } from './Material';

export interface ContactMaterialOptions {
  friction?: number;
  restitution?: number;
  contactEquationStiffness?: number;
  contactEquationRelaxation?: number;
  frictionEquationStiffness?: number;
  frictionEquationRelaxation?: number;
}

/**
 * Defines what happens when two materials meet.
 * @class ContactMaterial
 * @constructor
 * @param {Material} m1
 * @param {Material} m2
 * @param {object} [options]
 * @param {Number} [options.friction=0.3]
 * @param {Number} [options.restitution=0.3]
 * @param {number} [options.contactEquationStiffness=1e7]
 * @param {number} [options.contactEquationRelaxation=3]
 * @param {number} [options.frictionEquationStiffness=1e7]
 * @param {Number} [options.frictionEquationRelaxation=3]
 */
export class ContactMaterial {
  static idCounter = 0;

  id: number;
  materials: [Material, Material];
  friction: number;
  restitution: number;

  contactEquationStiffness: number;
  contactEquationRelaxation: number;
  frictionEquationStiffness: number;
  frictionEquationRelaxation: number;

  constructor(m1?: Material, m2?: Material,
    options: ContactMaterialOptions = <ContactMaterialOptions>{}) {

    const ops = Object.assign({
      friction: 0.3,
      restitution: 0.3,
      contactEquationStiffness: 1e7,
      contactEquationRelaxation: 3,
      frictionEquationStiffness: 1e7,
      frictionEquationRelaxation: 3
    }, options);

    /**
     * Identifier of this material
     * @property {Number} id
     */
    this.id = ContactMaterial.idCounter++;

    /**
     * Participating materials
     * @property {Array} materials
     * @todo  Should be .materialA and .materialB instead
     */
    this.materials = [m1 || new Material(), m2 || new Material()];

    /**
     * Friction coefficient
     * @property {Number} friction
     */
    this.friction = ops.friction;

    /**
     * Restitution coefficient
     * @property {Number} restitution
     */
    this.restitution = ops.restitution;

    /**
     * Stiffness of the produced contact equations
     * @property {Number} contactEquationStiffness
     */
    this.contactEquationStiffness = ops.contactEquationStiffness;

    /**
     * Relaxation time of the produced contact equations
     * @property {Number} contactEquationRelaxation
     */
    this.contactEquationRelaxation = ops.contactEquationRelaxation;

    /**
     * Stiffness of the produced friction equations
     * @property {Number} frictionEquationStiffness
     */
    this.frictionEquationStiffness = ops.frictionEquationStiffness;

    /**
     * Relaxation time of the produced friction equations
     * @property {Number} frictionEquationRelaxation
     */
    this.frictionEquationRelaxation = ops.frictionEquationRelaxation;
  }
}

