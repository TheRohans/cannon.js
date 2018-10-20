import { Vec3 } from './math/Vec3';
import { Box } from './shapes/Box';

// Create a companion cube based on triangles. The basis of this is the
// default cube exported from blender in the OBJ format. The other mocks
// are based around quads
export const mockCube = (): [Vec3[], Vec3[], number[][]] => {
  const vertices = [
    new Vec3(-1.000000, -1.000000, 1.000000),
    new Vec3(-1.000000, 1.000000, 1.000000),
    new Vec3(-1.000000, -1.000000, -1.000000),
    new Vec3(-1.000000, 1.000000, -1.000000),
    new Vec3(1.000000, -1.000000, 1.000000),
    new Vec3(1.000000, 1.000000, 1.000000),
    new Vec3(1.000000, -1.000000, -1.000000),
    new Vec3(1.000000, 1.000000, -1.000000),
  ];
  const normals = [
    new Vec3(-1.0000, 0.0000, 0.0000),
    new Vec3(0.0000, 0.0000, -1.0000),
    new Vec3(1.0000, 0.0000, 0.0000),
    new Vec3(0.0000, 0.0000, 1.0000),
    new Vec3(0.0000, -1.0000, 0.0000),
    new Vec3(0.0000, 1.0000, 0.0000),
  ];
  const vn = [
    [2, 1], [3, 1], [1, 1],
    [4, 2], [7, 2], [3, 2],
    [8, 3], [5, 3], [7, 3],
    [6, 4], [1, 4], [5, 4],
    [7, 5], [1, 5], [3, 5],
    [4, 6], [6, 6], [8, 6],
    [2, 1], [4, 1], [3, 1],
    [4, 2], [8, 2], [7, 2],
    [8, 3], [6, 3], [5, 3],
    [6, 4], [2, 4], [1, 4],
    [7, 5], [5, 5], [1, 5],
    [4, 6], [2, 6], [6, 6],
  ];

  const faces: number[][] = [];

  for (let z = 0; z < vn.length; z += 3) {
    const f = [
      vn[z][0] - 1,
      vn[z + 1][0] - 1,
      vn[z + 2][0] - 1,
    ];
    faces.push(f);
  }
  return [vertices, normals, faces];
};

export const mockBoxHull = (size: number = 0.5) => {
  const box = new Box(new Vec3(size, size, size));
  return box.convexPolyhedronRepresentation;
};

export const mockPolyBox = (sx: number, sy: number, sz: number) => {
  const v = Vec3;
  const box = new Box(new Vec3(sx, sy, sz));
  return box.convexPolyhedronRepresentation;
};
