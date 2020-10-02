import { ContactMaterial } from './ContactMaterial';
import { Material } from './Material';
describe('Contact Material', function () {
    it('should construct default', function () {
        var cm = new ContactMaterial();
        expect(cm).not.toBeUndefined();
        expect(cm.friction).toEqual(0.3);
        expect(cm.restitution).toEqual(0.3);
        expect(cm.contactEquationStiffness).toEqual(1e7);
        expect(cm.contactEquationRelaxation).toEqual(3);
        expect(cm.frictionEquationStiffness).toEqual(1e7);
        expect(cm.frictionEquationRelaxation).toEqual(3);
        expect(cm.materials).toBeDefined();
        expect(cm.materials[0]).toBeDefined();
        expect(cm.materials[1]).toBeDefined();
    });
    it('should override defaults', function () {
        var m1 = new Material();
        var m2 = new Material();
        var cm = new ContactMaterial(m1, m2, {
            friction: 1,
            restitution: 0
        });
        expect(cm).not.toBeUndefined();
        expect(cm.friction).toEqual(1);
        expect(cm.restitution).toEqual(0);
        expect(cm.materials[0]).toBe(m1);
        expect(cm.materials[1]).toBe(m2);
    });
});
