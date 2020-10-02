import { Body } from '../objects/Body';
import { Constraint } from './Constraint';
import { Equation } from '../equations/Equation';
describe('Constraint', function () {
    it('should construct', function () {
        var bodyA = new Body();
        var bodyB = new Body();
        var c = new Constraint(bodyA, bodyB);
        expect(c).not.toBeUndefined();
    });
    it('should enable', function () {
        var bodyA = new Body();
        var bodyB = new Body();
        var c = new Constraint(bodyA, bodyB);
        var eq = new Equation(bodyA, bodyB);
        c.equations.push(eq);
        c.enable();
        expect(eq.enabled).toBeTruthy();
        c.disable();
        expect(eq.enabled).toBeFalsy();
    });
});
