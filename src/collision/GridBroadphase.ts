import { Broadphase } from './Broadphase';
import { Vec3 } from '../math/Vec3';
import { World } from '../world/World';
import { Body } from '../objects/Body';
import { Shape } from '../shapes/Shape';
import { Plane } from '../shapes/Plane';
import { Sphere } from '../shapes/Sphere';

/**
 * Axis aligned uniform grid broadphase.
 * @class GridBroadphase
 * @constructor
 * @extends Broadphase
 * @todo Needs support for more than just planes and spheres.
 * @param {Vec3} aabbMin
 * @param {Vec3} aabbMax
 * @param {Number} nx Number of boxes along x
 * @param {Number} ny Number of boxes along y
 * @param {Number} nz Number of boxes along z
 */
export class GridBroadphase extends Broadphase {
  aabbMin: Vec3;
  aabbMax: Vec3;
  nx: number;
  ny: number;
  nz: number;

  bins: Body[][];
  binLengths: number[];

  constructor(aabbMin?: Vec3, aabbMax?: Vec3, nx = 10, ny = 10, nz = 10) {
    super();
    this.aabbMin = aabbMin || new Vec3(100, 100, 100);
    this.aabbMax = aabbMax || new Vec3(-100, -100, -100);

    const nbins = this.nx * this.ny * this.nz;
    if (nbins <= 0) {
      throw new Error('GridBroadphase: Each dimensions n must be >0');
    }

    this.bins = [];
    this.binLengths = []; // Rather than continually resizing arrays (thrashing the memory),
    // just record length and allow them to grow
    this.bins.length = nbins;
    this.binLengths.length = nbins;

    for (let i = 0; i < nbins; i++) {
      this.bins[i] = [];
      this.binLengths[i] = 0;
    }
  }

  private GridBroadphase_collisionPairs_d = new Vec3();
  private GridBroadphase_collisionPairs_binPos = new Vec3();
  /**
   * Get all the collision pairs in the physics world
   * @method collisionPairs
   * @param {World} world
   * @param {Array} pairs1
   * @param {Array} pairs2
   */
  collisionPairs(world: World, pairs1: Body[], pairs2: Body[]) {
    const N = world.numObjects(),
      bodies = world.bodies;

    const max = this.aabbMax,
      min = this.aabbMin;
    const nx = this.nx,
      ny = this.ny,
      nz = this.nz;

    const xstep = ny * nz;
    const ystep = nz;
    const zstep = 1;

    const xmax = max.x,
      ymax = max.y,
      zmax = max.z,
      xmin = min.x,
      ymin = min.y,
      zmin = min.z;

    const xmult = nx / (xmax - xmin),
      ymult = ny / (ymax - ymin),
      zmult = nz / (zmax - zmin);

    const binsizeX = (xmax - xmin) / nx,
      binsizeY = (ymax - ymin) / ny,
      binsizeZ = (zmax - zmin) / nz;

    const binRadius = Math.sqrt(binsizeX * binsizeX + binsizeY * binsizeY + binsizeZ * binsizeZ) * 0.5;

    const types = Shape.types;
    const SPHERE = types.SPHERE,
      PLANE = types.PLANE,
      BOX = types.BOX,
      COMPOUND = types.COMPOUND,
      CONVEXPOLYHEDRON = types.CONVEXPOLYHEDRON;

    const bins = this.bins,
      binLengths = this.binLengths,
      Nbins = this.bins.length;

    // Reset bins
    for (let i = 0; i !== Nbins; i++) {
      binLengths[i] = 0;
    }

    const ceil = Math.ceil;
    // const min = Math.min;
    // const max = Math.max;

    // Does this need to be created every time?
    const addBoxToBins = (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, bi: Body) => {
      let xoff0 = ((x0 - xmin) * xmult) | 0,
        yoff0 = ((y0 - ymin) * ymult) | 0,
        zoff0 = ((z0 - zmin) * zmult) | 0,
        xoff1 = ceil((x1 - xmin) * xmult),
        yoff1 = ceil((y1 - ymin) * ymult),
        zoff1 = ceil((z1 - zmin) * zmult);

      if (xoff0 < 0) { xoff0 = 0; } else if (xoff0 >= nx) { xoff0 = nx - 1; }
      if (yoff0 < 0) { yoff0 = 0; } else if (yoff0 >= ny) { yoff0 = ny - 1; }
      if (zoff0 < 0) { zoff0 = 0; } else if (zoff0 >= nz) { zoff0 = nz - 1; }
      if (xoff1 < 0) { xoff1 = 0; } else if (xoff1 >= nx) { xoff1 = nx - 1; }
      if (yoff1 < 0) { yoff1 = 0; } else if (yoff1 >= ny) { yoff1 = ny - 1; }
      if (zoff1 < 0) { zoff1 = 0; } else if (zoff1 >= nz) { zoff1 = nz - 1; }

      xoff0 *= xstep;
      yoff0 *= ystep;
      zoff0 *= zstep;
      xoff1 *= xstep;
      yoff1 *= ystep;
      zoff1 *= zstep;

      for (let xoff = xoff0; xoff <= xoff1; xoff += xstep) {
        for (let yoff = yoff0; yoff <= yoff1; yoff += ystep) {
          for (let zoff = zoff0; zoff <= zoff1; zoff += zstep) {
            const idx = xoff + yoff + zoff;
            bins[idx][binLengths[idx]++] = bi;
          }
        }
      }
    };

    // Put all bodies into the bins
    for (let i = 0; i !== N; i++) {
      const bi = bodies[i];
      const si = bi.shapes[0];

      switch (si.type) {
        case SPHERE:
          const ssi = (<Sphere>si);
          // Put in bin
          // check if overlap with other bins
          const x = bi.position.x,
            y = bi.position.y,
            z = bi.position.z;
          const r = ssi.radius;

          addBoxToBins(x - r, y - r, z - r, x + r, y + r, z + r, bi);
          break;

        case PLANE:
          const psi = (<Plane>si);
          if (psi.worldNormalNeedsUpdate) {
            psi.computeWorldNormal(bi.quaternion);
          }
          const planeNormal = psi.worldNormal;

          // Relative position from origin of plane object to the first bin
          // Incremented as we iterate through the bins
          const xreset = xmin + binsizeX * 0.5 - bi.position.x,
            yreset = ymin + binsizeY * 0.5 - bi.position.y,
            zreset = zmin + binsizeZ * 0.5 - bi.position.z;

          const d = this.GridBroadphase_collisionPairs_d;
          d.set(xreset, yreset, zreset);

          for (let xi = 0, xoff = 0; xi !== nx; xi++ , xoff += xstep, d.y = yreset, d.x += binsizeX) {
            for (let yi = 0, yoff = 0; yi !== ny; yi++ , yoff += ystep, d.z = zreset, d.y += binsizeY) {
              for (let zi = 0, zoff = 0; zi !== nz; zi++ , zoff += zstep, d.z += binsizeZ) {
                if (d.dot(planeNormal) < binRadius) {
                  const idx = xoff + yoff + zoff;
                  bins[idx][binLengths[idx]++] = bi;
                }
              }
            }
          }
          break;

        default:
          if (bi.aabbNeedsUpdate) {
            bi.computeAABB();
          }

          addBoxToBins(
            bi.aabb.lowerBound.x,
            bi.aabb.lowerBound.y,
            bi.aabb.lowerBound.z,
            bi.aabb.upperBound.x,
            bi.aabb.upperBound.y,
            bi.aabb.upperBound.z,
            bi);
          break;
      }
    }

    // Check each bin
    for (let i = 0; i !== Nbins; i++) {
      const binLength = binLengths[i];
      // Skip bins with no potential collisions
      if (binLength > 1) {
        const bin = bins[i];

        // Do N^2 broadphase inside
        for (let xi = 0; xi !== binLength; xi++) {
          const bi = bin[xi];
          for (let yi = 0; yi !== xi; yi++) {
            const bj = bin[yi];
            if (this.needBroadphaseCollision(bi, bj)) {
              this.intersectionTest(bi, bj, pairs1, pairs2);
            }
          }
        }
      }
    }

    // for (var zi = 0, zoff = 0; zi < nz; zi++ , zoff += zstep) {
    //   for (var yi = 0, yoff = 0; yi < ny; yi++ , yoff += ystep) {
    //     var row = '';
    //     for (var xi = 0, xoff = 0; xi < nx; xi++ , xoff += xstep) {
    //       var idx = xoff + yoff + zoff;
    //       row += ' ' + binLengths[idx];
    //     }
    //   }
    // }

    this.makePairsUnique(pairs1, pairs2);
  }

}
