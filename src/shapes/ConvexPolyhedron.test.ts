import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { HullResult, ConvexPolyhedron } from './ConvexPolyhedron';
import { mockBoxHull, mockPolyBox } from '../main.test';

describe('ConvexPolyhedron', () => {

  it('should calculateWorldAABB', () => {
      const poly = mockPolyBox(1, 1, 1);
      const min = new Vec3();
      const max = new Vec3();
      poly.calculateWorldAABB(new Vec3(1, 0, 0), // Translate 2 x in world
                              new Quaternion(0, 0, 0, 1),
                              min,
                              max);
      expect(min.x).toEqual(0);
      expect(max.x).toEqual(2);
      expect(min.y).toEqual(-1);
      expect(max.y).toEqual( 1);
  });

  it('should clipFaceAgainstPlane', () => {
      const h = mockBoxHull();

      // Four points 1 unit below the plane z=0 - we assume to get back 4
      let inverts = [  new Vec3(-0.2, -0.2, -1),
                       new Vec3(-0.2,  0.2, -1),
                       new Vec3( 0.2,  0.2, -1),
                       new Vec3( 0.2, -0.2, -1)];
      let outverts: Vec3[] = [];
      h.clipFaceAgainstPlane(inverts, outverts, new Vec3(0, 0, 1), 0.0);
      expect(outverts.length).toEqual(4);
      inverts = [];
      outverts = [];

      // Lower the plane to z=-2, we assume no points back
      h.clipFaceAgainstPlane(inverts, outverts, new Vec3(0, 0, 1), 2);
      expect(outverts.length).toEqual(0);

      // two points below, two over. We get four points back, though 2 of them are clipped to
      // the back of the  plane
      const inverts2 = [new Vec3(-2, -2,  1),
                      new Vec3(-2,  2,  1),
                      new Vec3( 2,  2, -1),
                      new Vec3( 2, -2, -1)];
      outverts = [];
      h.clipFaceAgainstPlane(inverts2, outverts, new Vec3(0, 0, 1), 0.0);
      expect(outverts.length).toEqual(4);
  });

  it('should clipFaceAgainstHull', () => {
      // Create box
      const hullA = mockBoxHull(0.5);
      const res: HullResult[] = [];
      const sepNormal = new Vec3(0, 0, 1);

      // Move the box 0.45 units up - only 0.05 units of the box will be below plane z=0
      const posA = new Vec3(0, 0, 0.45),
          quatA = new Quaternion();

      // All points from B is in the plane z=0
      const worldVertsB = [ new Vec3(-1.0, -1.0, 0),
                          new Vec3(-1.0, 1.0, 0),
                          new Vec3( 1.0, 1.0, 0),
                          new Vec3( 1.0, -1.0, 0)];

      // We will now clip a face in hullA that is closest to the sepNormal
      // against the points in worldVertsB.
      // We can expect to get back the 4 corners of the box hullA penetrated 0.05 units
      // into the plane worldVertsB we constructed
      hullA.clipFaceAgainstHull(sepNormal, posA, quatA, worldVertsB, -100, 100, res);
      // console.error(res);
      res.forEach( r => {
        expect(r.depth).toEqual(-0.04999999999999999);
      });
  });

  it('should clipAgainstHull', () => {
    const hullA = mockBoxHull(0.6),
        posA = new Vec3(-0.5, 0, 0),
        quatA = new Quaternion();

    const hullB = mockBoxHull(0.5),
        posB = new Vec3(0.5, 0, 0),
        quatB = new Quaternion();

    const sepaxis = new Vec3();
    const found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
    const result: HullResult[] = [];
    quatB.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);

    hullA.clipAgainstHull(posA, quatA, hullB, posB, quatB, sepaxis, -100, 100, result);
    // console.log("result:",result);
    // console.log("done....");
    result.forEach( r => {
      expect(r.depth).toEqual(-0.30710678118654733);
    });
  });

  it('should testSepAxis', () => {
      const hullA = mockBoxHull(0.5),
      posA = new Vec3(-0.2, 0, 0),
      quatA = new Quaternion();

      const hullB = mockBoxHull(),
      posB = new Vec3(0.2, 0, 0),
      quatB = new Quaternion();

      const sepAxis = new Vec3(1, 0, 0);
      const [found1, depth] = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
      // ,"didnt find sep axis depth");
      expect(found1).toBeTruthy();
      expect(depth).toEqual(0.6);

      // Move away
      posA.x = -5;
      const [found2, dp] = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
      // "found separating axis though there are none");
      expect(found2).toBeFalsy();

      // Inclined 45 degrees, what happens then?
      posA.x = 1;
      quatB.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
      const [found3, depth3] = hullA.testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
      // ,"Did not fetch");
      expect(typeof(found3)).toEqual('boolean');
      expect(typeof(depth3)).toEqual('number');
  });

  it('should findSepAxis', () => {
      const hullA = mockBoxHull(),
          posA = new Vec3(-0.2, 0, 0),
          quatA = new Quaternion();

      const hullB = mockBoxHull(),
          posB = new Vec3(0.2, 0, 0),
          quatB = new Quaternion();

      const sepaxis = new Vec3();
      const found = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
      // console.log("SepAxis found:",found,", the axis:",sepaxis.toString());
      expect(found).toEqual(true);

      quatB.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
      const found2 = hullA.findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
      // console.log("SepAxis found:",found2,", the axis:",sepaxis.toString());
      expect(found2).toEqual(true);
  });

  it('should project', () => {
      const convex = mockBoxHull(0.5),
          pos = new Vec3(0, 0, 0),
          quat = new Quaternion();

      const axis = new Vec3(1, 0, 0);
      const result: number[] = [];

      ConvexPolyhedron.project(convex, axis, pos, quat, result);
      expect(result).toEqual([0.5, -0.5]);

      axis.set(-1, 0, 0);
      ConvexPolyhedron.project(convex, axis, pos, quat, result);
      expect(result).toEqual([0.5, -0.5]);

      axis.set(0, 1, 0);
      ConvexPolyhedron.project(convex, axis, pos, quat, result);
      expect(result).toEqual([0.5, -0.5]);

      pos.set(0, 1, 0);
      axis.set(0, 1, 0);
      ConvexPolyhedron.project(convex, axis, pos, quat, result);
      expect(result).toEqual([1.5, 0.5]);

      // Test to rotate
      quat.setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);
      pos.set(0, 1, 0);
      axis.set(0, 1, 0);
      ConvexPolyhedron.project(convex, axis, pos, quat, result);
      expect(Math.abs(result[0] - 1.5) < 0.01).toBeTruthy();
      expect(Math.abs(result[1] - 0.5) < 0.01).toBeTruthy();
  });

  it('should calculateWorldAABB always decreasing verts NoUndefined', () => {
    const vertices = [
        new Vec3( 4, 4, 4),
        new Vec3( 3, 3, 3),
        new Vec3( 2, 2, 2),
        new Vec3( 1, 1, 1),
        new Vec3( 0, 0, 0),
        new Vec3(-1, -1, -1),
        new Vec3(-2, -2, -2),
        new Vec3(-3, -3, -3)
    ];

    const indices = [
        [3, 2, 1, 0],
        [4, 5, 6, 7],
        [5, 4, 0, 1],
        [2, 3, 7, 6],
        [0, 4, 7, 3],
        [1, 2, 6, 5],
    ];
    const poly = new ConvexPolyhedron(vertices, indices);
    const min = new Vec3();
    const max = new Vec3();
    poly.calculateWorldAABB(new Vec3(0, 0, 0), new Quaternion(0, 0, 0, 1), min, max);
    expect(min.x).not.toBeUndefined();
    expect(max.x).not.toBeUndefined();
    expect(min.y).not.toBeUndefined();
    expect(max.y).not.toBeUndefined();
    expect(min.z).not.toBeUndefined();
    expect(max.z).not.toBeUndefined();
  });
});

