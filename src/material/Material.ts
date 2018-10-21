export interface MaterialOptions {
  name?: string;
  friction?: number;
  restitution?: number;
}

/**
 * Defines a physics material.
 * @class Material
 * @constructor
 * @param {object} [options]
 * @author schteppe
 */
export class Material {
  static idCounter = 0;

  name: string;
  id: number;
  friction: number;
  restitution: number;

  constructor(options: MaterialOptions = <MaterialOptions>{}) {
    options = Object.assign({
      name: 'default',
      friction: 0.3,
      restitution: 0.3
    }, options);

    /**
     * @property name
     * @type {String}
     */
    this.name = options.name;

    /**
     * material id.
     * @property id
     * @type {number}
     */
    this.id = Material.idCounter++;

    /**
     * Friction for this material. If non-negative, it will be used instead of
     * the friction given by ContactMaterials. If there's no matching ContactMaterial,
     * the value from .defaultContactMaterial in the World will be used.
     * @property {number} friction
     */
    this.friction = options.friction;

    /**
     * Restitution for this material. If non-negative, it will be used instead of
     * the restitution given by ContactMaterials. If there's no matching ContactMaterial,
     * the value from .defaultContactMaterial in the World will be used.
     * @property {number} restitution
     */
    this.restitution = options.restitution;
  }
}
