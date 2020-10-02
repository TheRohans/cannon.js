import { World } from './World';
import { Body } from '../objects/Body';
import { Vec3 } from '../math/Vec3';
import { RaycastResult } from '../collision/RaycastResult';
import { Box } from '../shapes/Box';
import { Sphere } from '../shapes/Sphere';
import { NaiveBroadphase } from '../collision/NaiveBroadphase';
import { ArrayCollisionMatrix } from '../collision/ArrayCollisionMatrix';
describe('World', function () {
    it('should create a world', function () {
        var world = new World();
        expect(world).not.toBeUndefined();
    });
    it('should clearForces', function () {
        var world = new World();
        var body = new Body();
        world.addBody(body);
        body.force.set(1, 2, 3);
        body.torque.set(4, 5, 6);
        world.clearForces();
        expect(body.force.almostEquals(new Vec3(0, 0, 0))).toBeTruthy();
        expect(body.torque.almostEquals(new Vec3(0, 0, 0))).toBeTruthy();
    });
    it('should rayTestBox', function () {
        var world = new World();
        var body = new Body();
        body.addShape(new Box(new Vec3(1, 1, 1)));
        world.addBody(body);
        var from = new Vec3(-10, 0, 0);
        var to = new Vec3(10, 0, 0);
        var result = new RaycastResult();
        world.rayTest(from, to, result);
        expect(result.hasHit).toBeTruthy();
    });
    it('should rayTestSphere', function () {
        var world = new World();
        var body = new Body();
        body.addShape(new Sphere(1));
        world.addBody(body);
        var from = new Vec3(-10, 0, 0);
        var to = new Vec3(10, 0, 0);
        var result = new RaycastResult();
        world.rayTest(from, to, result);
        expect(result.hasHit).toBeTruthy();
    });
    describe('raycastClosest', function () {
        it('should single', function () {
            var world = new World();
            var body = new Body({
                shape: new Sphere(1)
            });
            world.addBody(body);
            var from = new Vec3(-10, 0, 0);
            var to = new Vec3(10, 0, 0);
            var result = new RaycastResult();
            world.raycastClosest(from, to, {}, result);
            expect(result.hasHit).toBeTruthy();
            expect(result.body).toEqual(body);
            expect(result.shape).toEqual(body.shapes[0]);
        });
        it('should order', function () {
            var world = new World();
            var bodyA = new Body({ shape: new Sphere(1), position: new Vec3(-1, 0, 0) });
            var bodyB = new Body({ shape: new Sphere(1), position: new Vec3(1, 0, 0) });
            world.addBody(bodyA);
            world.addBody(bodyB);
            var from = new Vec3(-10, 0, 0);
            var to = new Vec3(10, 0, 0);
            var result = new RaycastResult();
            world.raycastClosest(from, to, {}, result);
            expect(result.hasHit).toBeTruthy();
            expect(result.body).toEqual(bodyA);
            expect(result.shape).toEqual(bodyA.shapes[0]);
            from.set(10, 0, 0);
            to.set(-10, 0, 0);
            result = new RaycastResult();
            world.raycastClosest(from, to, {}, result);
            expect(result.hasHit).toBeTruthy();
            expect(result.body).toEqual(bodyB);
            expect(result.shape).toEqual(bodyB.shapes[0]);
        });
    });
    describe('raycastAll', function () {
        it('should simple', function () {
            var world = new World();
            var body = new Body({ shape: new Sphere(1) });
            world.addBody(body);
            var from = new Vec3(-10, 0, 0);
            var to = new Vec3(10, 0, 0);
            var hasHit;
            var numResults = 0;
            var resultBody;
            var resultShape;
            var returnVal = world.raycastAll(from, to, {}, function (result) {
                hasHit = result.hasHit;
                resultShape = result.shape;
                resultBody = result.body;
                numResults++;
            });
            expect(returnVal).toBeTruthy();
            expect(hasHit).toBeTruthy();
            expect(resultBody).toEqual(body);
            expect(numResults).toEqual(2);
            expect(resultShape).toEqual(resultBody.shapes[0]);
        });
        it('should twoSpheres', function () {
            var world = new World();
            var body = new Body({ shape: new Sphere(1) });
            world.addBody(body);
            var body2 = new Body({ shape: new Sphere(1) });
            world.addBody(body2);
            var from = new Vec3(-10, 0, 0);
            var to = new Vec3(10, 0, 0);
            var hasHit = false;
            var numResults = 0;
            var resultBody;
            var resultShape;
            world.raycastAll(from, to, {}, function (result) {
                hasHit = result.hasHit;
                resultShape = result.shape;
                resultBody = result.body;
                numResults++;
            });
            expect(hasHit).toBeTruthy();
            expect(numResults).toEqual(4);
        });
        it('should skipBackFaces', function () {
            var world = new World();
            var body = new Body({ shape: new Sphere(1) });
            world.addBody(body);
            var hasHit = false;
            var numResults = 0;
            var resultBody;
            var resultShape;
            world.raycastAll(new Vec3(-10, 0, 0), new Vec3(10, 0, 0), { skipBackfaces: true }, function (result) {
                hasHit = result.hasHit;
                resultShape = result.shape;
                resultBody = result.body;
                numResults++;
            });
            expect(hasHit).toBeTruthy();
            expect(numResults).toEqual(1);
        });
        it('should collisionFilters', function () {
            var world = new World();
            var body = new Body({
                shape: new Sphere(1)
            });
            world.addBody(body);
            body.collisionFilterGroup = 2;
            body.collisionFilterMask = 2;
            var numResults = 0;
            world.raycastAll(new Vec3(-10, 0, 0), new Vec3(10, 0, 0), {
                collisionFilterGroup: 2,
                collisionFilterMask: 2
            }, function (result) {
                numResults++;
            });
            expect(numResults).toEqual(2);
            numResults = 0;
            world.raycastAll(new Vec3(-10, 0, 0), new Vec3(10, 0, 0), {
                collisionFilterGroup: 1,
                collisionFilterMask: 1
            }, function (result) {
                numResults++;
            });
            expect(numResults).toEqual(0);
        });
    });
    it('should raycastAny', function () {
        var world = new World();
        world.addBody(new Body({ shape: new Sphere(1) }));
        var from = new Vec3(-10, 0, 0);
        var to = new Vec3(10, 0, 0);
        var result = new RaycastResult();
        world.raycastAny(from, to, {}, result);
        expect(result.hasHit).toBeTruthy();
    });
    it('should collisionMatrix', function () {
        var test_configs = [
            {
                positions: [
                    [0, 0, 0],
                    [2, 0, 0],
                    [0, 4, 0],
                    [2, 4, 0],
                    [0, 8, 0],
                    [2, 8, 0]
                ],
                colliding: {
                    '0-1': true,
                    '2-3': true,
                    '4-5': true
                }
            },
            {
                positions: [
                    [0, 0, 0],
                    [0, 4, 0],
                    [0, 8, 0],
                    [2, 0, 0],
                    [2, 4, 0],
                    [2, 8, 0]
                ],
                colliding: {
                    '0-3': true,
                    '1-4': true,
                    '2-5': true
                }
            },
            {
                positions: [
                    [0, 0, 0],
                    [0, 1, 0],
                    [0, 10, 0],
                    [0, 20, 0],
                    [0, 30, 0],
                    [0, 40, 0],
                    [0, 50, 0],
                    [0, 51, 0]
                ],
                colliding: {
                    '0-1': true,
                    '6-7': true
                }
            }
        ];
        for (var config_idx = 0; config_idx < test_configs.length; config_idx++) {
            var test_config = test_configs[config_idx];
            var world = new World();
            world.broadphase = new NaiveBroadphase();
            world.collisionMatrix = new ArrayCollisionMatrix();
            world.collisionMatrixPrevious = new ArrayCollisionMatrix();
            for (var position_idx = 0; position_idx < test_config.positions.length; position_idx++) {
                var body = new Body({ mass: 1 });
                body.addShape(new Sphere(1.1));
                body.position.set.apply(body.position, test_config.positions[position_idx]);
                world.addBody(body);
            }
            for (var step_idx = 0; step_idx < 2; step_idx++) {
                world.step(0.1);
                var is_first_step = (step_idx === 0);
                for (var coll_i = 0; coll_i < world.bodies.length; coll_i++) {
                    for (var coll_j = coll_i + 1; coll_j < world.bodies.length; coll_j++) {
                        var is_colliding_pair = test_config.colliding[coll_i + '-' + coll_j] === true;
                        var expected = is_colliding_pair;
                        var is_colliding = is_first_step ?
                            !!world.collisionMatrix.get(world.bodies[coll_i], world.bodies[coll_j]) :
                            !!world.collisionMatrixPrevious.get(world.bodies[coll_i], world.bodies[coll_j]);
                        expect(is_colliding).toEqual(expected);
                    }
                }
            }
        }
    });
});
