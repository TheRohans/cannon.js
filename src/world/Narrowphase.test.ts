import { World } from './World';
import { Narrowphase } from './Narrowphase';
import { Sphere } from '../shapes/Sphere';
import { Body } from '../objects/Body';
import { ContactMaterial } from '../material/ContactMaterial';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { HullResult, ConvexPolyhedron } from '../shapes/ConvexPolyhedron';
import { mockCube, mockBoxHull } from '../main.test';
import { ContactEquation } from 'equations/ContactEquation';


describe('Narrowphase', () => {
  let cm: ContactMaterial;

  // const createHeightfield = () => {
  //   const matrix = [];
  //   const size = 20;
  //   for (let i = 0; i < size; i++) {
  //     matrix.push([]);
  //     for (let j = 0; j < size; j++) {
  //       matrix[i].push(0);
  //     }
  //   }
  //   const hfShape = new Heightfield(matrix, {
  //     elementSize: 1,
  //   });
  //   return hfShape;
  // };
  beforeEach( () => {
    cm = new ContactMaterial();
    cm.friction = 0;
    cm.restitution = 0;
  });

  it('should sphereSphere', () => {
    const world = new World();
    const cg = new Narrowphase(world);
    const result: ContactEquation[] = [];
    const sphereShape = new Sphere(1);

    const bodyA = new Body({ mass: 1 });
    bodyA.addShape(sphereShape);
    const bodyB = new Body({ mass: 1 });
    bodyB.addShape(sphereShape);

    cg.currentContactMaterial = cm;
    cg.result = result;
    cg.sphereSphere(
      sphereShape,
      sphereShape,
      new Vec3(0.5, 0, 0),
      new Vec3(-0.5, 0, 0),
      new Quaternion(),
      new Quaternion(),
      bodyA,
      bodyB
    );

    expect(result.length).toEqual(1);
  });

  it('should return false if convexConvex does not touch (tri)', () => {
    const mc = mockCube();

    const world = new World();
    world.defaultContactMaterial = cm;
    const np = new Narrowphase(world);

    const s1 = new ConvexPolyhedron(mc[0], mc[2]);
    const s2 = new ConvexPolyhedron(mc[0], mc[2]);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0, 1.5, 0);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(0, -1.5, 0);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(false);
    expect(np.result.length).toEqual(0);
  });

  it('should return true if convexConvex does touch X (quad)', () => {
    const s1 = mockBoxHull(1);
    const s2 = mockBoxHull(1);

    const world = new World();
    const np = new Narrowphase(world);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0.5, 0, 0);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(-0.5, 0, 0);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(4);
  });

  it('should return true if convexConvex does touch X (tri)', () => {
    const mc = mockCube();
    const mc2 = mockCube();

    const world = new World();
    const np = new Narrowphase(world);

    const s1 = new ConvexPolyhedron(mc[0], mc[2]);
    const s2 = new ConvexPolyhedron(mc2[0], mc2[2]);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0.5, 0, 0);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(-0.5, 0, 0);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(3);
  });

  it('should return true if convexConvex does touch Y (quad)', () => {
    const s1 = mockBoxHull(1);
    const s2 = mockBoxHull(1);

    const world = new World();
    const np = new Narrowphase(world);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0, 0.5, 0);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(0, -0.5, 0);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(4);
  });

  it('should return true if convexConvex does touch Y (tri)', () => {
    const mc = mockCube();
    const mc2 = mockCube();

    const world = new World();
    const np = new Narrowphase(world);

    const s1 = new ConvexPolyhedron(mc[0], mc[2]);
    const s2 = new ConvexPolyhedron(mc2[0], mc2[2]);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0, 0.5, 0);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(0, -0.5, 0);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(3);
  });

  it('should return true if convexConvex does touch Z (quad)', () => {
    const s1 = mockBoxHull(1);
    const s2 = mockBoxHull(1);

    const world = new World();
    const np = new Narrowphase(world);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0, 0, 0.5);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(0, 0, -0.5);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(4);
  });

  it('should return true if convexConvex does touch Z (tri)', () => {
    const mc = mockCube();
    const mc2 = mockCube();

    const world = new World();
    const np = new Narrowphase(world);

    const s1 = new ConvexPolyhedron(mc[0], mc[2]);
    const s2 = new ConvexPolyhedron(mc2[0], mc2[2]);

    const b1 = new Body({ mass: 1 });
    b1.addShape(s1);
    b1.position = new Vec3(0, 0, 0.5);

    const b2 = new Body({ mass: 1 });
    b2.addShape(s2);
    b2.position = new Vec3(0, 0, -0.5);

    const actual = np.convexConvex(
      s1, s2,
      b1.position, b2.position,
      new Quaternion(), new Quaternion(),
      b1, b2,
      undefined, undefined,
      false
    );

    expect(actual).toEqual(true);
    expect(np.result.length).toEqual(3);
  });

  // it('should sphereHeightfield', () => {
  //   const world = new World();
  //   const cg = new Narrowphase(world);
  //   const result: HullResult[] = [];
  //   const hfShape = createHeightfield();
  //   const sphereShape = new Sphere(0.1);
  //   cg.currentContactMaterial = new ContactMaterial();
  //   cg.result = result;
  //   cg.sphereHeightfield(
  //     sphereShape,
  //     hfShape,
  //     new Vec3(0.25, 0.25, 0.05), // hit the first triangle in the field
  //     new Vec3(0, 0, 0),
  //     new Quaternion(),
  //     new Quaternion(),
  //     new Body(1, sphereShape),
  //     new Body(1, hfShape)
  //   );

  //   expect(result.length).toEqual(1);
  // });

});
