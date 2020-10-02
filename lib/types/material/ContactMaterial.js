import { Material } from './Material';
var ContactMaterial = (function () {
    function ContactMaterial(m1, m2, options) {
        if (options === void 0) { options = {}; }
        var ops = Object.assign({
            friction: 0.3,
            restitution: 0.3,
            contactEquationStiffness: 1e7,
            contactEquationRelaxation: 3,
            frictionEquationStiffness: 1e7,
            frictionEquationRelaxation: 3
        }, options);
        this.id = ContactMaterial.idCounter++;
        this.materials = [m1 || new Material(), m2 || new Material()];
        this.friction = ops.friction;
        this.restitution = ops.restitution;
        this.contactEquationStiffness = ops.contactEquationStiffness;
        this.contactEquationRelaxation = ops.contactEquationRelaxation;
        this.frictionEquationStiffness = ops.frictionEquationStiffness;
        this.frictionEquationRelaxation = ops.frictionEquationRelaxation;
    }
    ContactMaterial.idCounter = 0;
    return ContactMaterial;
}());
export { ContactMaterial };
