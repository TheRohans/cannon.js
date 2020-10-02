import { Vec3 } from '../math/Vec3';
import { Body } from '../objects/Body';
import { HingeConstraint } from './HingeConstraint';
describe('HingeConstraint', function () {
    it('should construct', function () {
        var bodyA = new Body({ mass: 1, position: new Vec3(1, 0, 0) });
        var bodyB = new Body({ mass: 1, position: new Vec3(-1, 0, 0) });
        var c = new HingeConstraint(bodyA, bodyB, { maxForce: 123 });
        expect(c.equations.length).toEqual(6);
        expect(c.equations[0].maxForce).toEqual(123);
        expect(c.equations[1].maxForce).toEqual(123);
        expect(c.equations[2].maxForce).toEqual(123);
        expect(c.equations[3].maxForce).toEqual(123);
        expect(c.equations[4].maxForce).toEqual(123);
        expect(c.equations[5].maxForce).toEqual(123);
        expect(c.equations[0].minForce).toEqual(-123);
        expect(c.equations[1].minForce).toEqual(-123);
        expect(c.equations[2].minForce).toEqual(-123);
        expect(c.equations[3].minForce).toEqual(-123);
        expect(c.equations[4].minForce).toEqual(-123);
        expect(c.equations[5].minForce).toEqual(-123);
    });
    it('should update', function () {
        var bodyA = new Body({ mass: 1, position: new Vec3(1, 0, 0) });
        var bodyB = new Body({ mass: 1, position: new Vec3(-1, 0, 0) });
        var c = new HingeConstraint(bodyA, bodyB, { maxForce: 123 });
        expect(function () { return c.update(); }).not.toThrow();
    });
    it('should enableDisableMotor', function () {
        var bodyA = new Body({ mass: 1, position: new Vec3(1, 0, 0) });
        var bodyB = new Body({ mass: 1, position: new Vec3(-1, 0, 0) });
        var c = new HingeConstraint(bodyA, bodyB);
        c.enableMotor();
        expect(c.motorEquation.enabled);
        c.disableMotor();
        expect(c.motorEquation.enabled).toEqual(false);
    });
    it('should setMotorSpeed', function () {
        var bodyA = new Body({ mass: 1, position: new Vec3(1, 0, 0) });
        var bodyB = new Body({ mass: 1, position: new Vec3(-1, 0, 0) });
        var c = new HingeConstraint(bodyA, bodyB);
        c.setMotorSpeed(5);
        expect(c.motorEquation.targetVelocity).toEqual(5);
    });
    it('should setMotorMaxForce', function () {
        var bodyA = new Body({ mass: 1, position: new Vec3(1, 0, 0) });
        var bodyB = new Body({ mass: 1, position: new Vec3(-1, 0, 0) });
        var c = new HingeConstraint(bodyA, bodyB);
        c.setMotorMaxForce(100);
        expect(c.motorEquation.maxForce).toEqual(100);
    });
});
