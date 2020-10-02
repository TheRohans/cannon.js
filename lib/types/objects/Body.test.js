import { Body } from '../objects/Body';
import { Vec3 } from '../math/Vec3';
import { Box } from '../shapes/Box';
import { Quaternion } from '../math/Quaternion';
import { Sphere } from '../shapes/Sphere';
describe('Body', function () {
    describe('computeAABB', function () {
        it('should box ', function () {
            var body = new Body({ mass: 1 });
            body.addShape(new Box(new Vec3(1, 1, 1)));
            body.computeAABB();
            expect(body.aabb.lowerBound.x).toEqual(-1);
            expect(body.aabb.lowerBound.y).toEqual(-1);
            expect(body.aabb.lowerBound.z).toEqual(-1);
            expect(body.aabb.upperBound.x).toEqual(1);
            expect(body.aabb.upperBound.y).toEqual(1);
            expect(body.aabb.upperBound.z).toEqual(1);
            body.position.x = 1;
            body.computeAABB();
            expect(body.aabb.lowerBound.x).toEqual(0);
            expect(body.aabb.upperBound.x).toEqual(2);
        });
        it('should boxOffset ', function () {
            var quaternion = new Quaternion();
            quaternion.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
            var body = new Body({ mass: 1 });
            body.addShape(new Box(new Vec3(1, 1, 1)), new Vec3(1, 1, 1));
            body.computeAABB();
            expect(body.aabb.lowerBound.x).toEqual(0);
            expect(body.aabb.lowerBound.y).toEqual(0);
            expect(body.aabb.lowerBound.z).toEqual(0);
            expect(body.aabb.upperBound.x).toEqual(2);
            expect(body.aabb.upperBound.y).toEqual(2);
            expect(body.aabb.upperBound.z).toEqual(2);
            body.position.x = 1;
            body.computeAABB();
            expect(body.aabb.lowerBound.x).toEqual(1);
            expect(body.aabb.upperBound.x).toEqual(3);
        });
    });
    it('should updateInertiaWorld', function () {
        var body = new Body({ mass: 1 });
        body.addShape(new Box(new Vec3(1, 1, 1)));
        body.quaternion.setFromEuler(Math.PI / 2, 0, 0);
        body.updateInertiaWorld();
    });
    it('should pointToLocalFrame', function () {
        var body = new Body({ mass: 1 });
        body.addShape(new Sphere(1));
        body.position.set(1, 2, 2);
        var localPoint = body.pointToLocalFrame(new Vec3(1, 2, 3));
        expect(localPoint.almostEquals(new Vec3(0, 0, 1))).toBeTruthy();
    });
    it('should pointToWorldFrame', function () {
        var body = new Body({ mass: 1 });
        body.addShape(new Sphere(1));
        body.position.set(1, 2, 2);
        var worldPoint = body.pointToWorldFrame(new Vec3(1, 0, 0));
        expect(worldPoint.almostEquals(new Vec3(2, 2, 2))).toBeTruthy();
    });
    it('should addShape', function () {
        var sphereShape = new Sphere(1);
        var bodyA = new Body({
            mass: 1,
            shape: sphereShape
        });
        var bodyB = new Body({
            mass: 1
        });
        bodyB.addShape(sphereShape);
        expect(bodyA.shapes).toEqual(bodyB.shapes);
        expect(bodyA.inertia).toEqual(bodyB.inertia);
    });
    it('should applyForce', function () {
        var sphereShape = new Sphere(1);
        var body = new Body({
            mass: 1,
            shape: sphereShape
        });
        var worldPoint = new Vec3(1, 0, 0);
        var forceVector = new Vec3(0, 1, 0);
        body.applyForce(forceVector, worldPoint);
        expect(body.force).toEqual(forceVector);
        expect(body.torque).toEqual(new Vec3(0, 0, 1));
    });
    it('should applyLocalForce', function () {
        var sphereShape = new Sphere(1);
        var body = new Body({
            mass: 1,
            shape: sphereShape
        });
        body.quaternion.setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);
        var localPoint = new Vec3(1, 0, 0);
        var localForceVector = new Vec3(0, 1, 0);
        body.applyLocalForce(localForceVector, localPoint);
        expect(body.force.almostEquals(new Vec3(0, 0, 1))).toBeTruthy();
    });
    it('should applyImpulse', function () {
        var sphereShape = new Sphere(1);
        var body = new Body({
            mass: 1,
            shape: sphereShape
        });
        var f = 1000;
        var dt = 1 / 60;
        var worldPoint = new Vec3(0, 0, 0);
        var impulse = new Vec3(f * dt, 0, 0);
        body.applyImpulse(impulse, worldPoint);
        expect(body.velocity.almostEquals(new Vec3(f * dt, 0, 0))).toBeTruthy();
    });
    it('should applyLocalImpulse', function () {
        var sphereShape = new Sphere(1);
        var body = new Body({
            mass: 1,
            shape: sphereShape
        });
        body.quaternion.setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);
        var f = 1000;
        var dt = 1 / 60;
        var localPoint = new Vec3(1, 0, 0);
        var localImpulseVector = new Vec3(0, f * dt, 0);
        body.applyLocalImpulse(localImpulseVector, localPoint);
        expect(body.velocity.almostEquals(new Vec3(0, 0, f * dt))).toBeTruthy();
    });
});
