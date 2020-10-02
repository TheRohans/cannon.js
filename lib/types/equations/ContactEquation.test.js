import { Body } from '../objects/Body';
import { ContactEquation } from './ContactEquation';
import { Vec3 } from '../math/Vec3';
describe('ContactEquation', function () {
    it('should construct', function () {
        var bodyA = new Body();
        var bodyB = new Body();
        var c = new ContactEquation(bodyA, bodyB);
        expect(c).not.toBeUndefined();
    });
    it('should getImpactVelocityAlongNormal', function () {
        var bodyA = new Body({
            position: new Vec3(1, 0, 0),
            velocity: new Vec3(-10, 0, 0)
        });
        var bodyB = new Body({
            position: new Vec3(-1, 0, 0),
            velocity: new Vec3(1, 0, 0)
        });
        var contact = new ContactEquation(bodyA, bodyB);
        contact.ni.set(1, 0, 0);
        contact.ri.set(-1, 0, 0);
        contact.rj.set(1, 0, 0);
        var v = contact.getImpactVelocityAlongNormal();
        expect(v).toEqual(-11);
    });
});
