import { World } from './World';
import { Vec3Pool } from '../utils/Vec3Pool';
import { ContactMaterial } from '../material/ContactMaterial';
import { Shape } from '../shapes/Shape';
import { Body } from '../objects/Body';
import { ContactEquation } from '../equations/ContactEquation';
import { Vec3 } from '../math/Vec3';
import { Quaternion } from '../math/Quaternion';
import { ConvexPolyhedron, HullResult } from '../shapes/ConvexPolyhedron';
import { Box } from '../shapes/Box';
import { Sphere } from '../shapes/Sphere';
import { Plane } from '../shapes/Plane';
import { Particle } from '../shapes/Particle';
import { FrictionEquation } from '../equations/FrictionEquation';

/**
 * Helper class for the World. Generates ContactEquations.
 * @class Narrowphase
 * @constructor
 * @todo Sphere-ConvexPolyhedron contacts
 * @todo Contact reduction
 * @todo  should move methods to prototype
 */
export class Narrowphase {
  contactPointPool: any[];
  frictionEquationPool: FrictionEquation[];
  result: ContactEquation[];
  frictionResult: FrictionEquation[];

  v3pool: Vec3Pool;
  world: World;
  currentContactMaterial: ContactMaterial;
  enableFrictionReduction: boolean;

  constructor(world: World) {
    /**
     * Internal storage of pooled contact points.
     * @property {Array} contactPointPool
     */
    this.contactPointPool = [];

    this.frictionEquationPool = [];

    this.result = [];
    this.frictionResult = [];

    /**
     * Pooled vectors.
     * @property {Vec3Pool} v3pool
     */
    this.v3pool = new Vec3Pool();

    this.world = world;
    this.currentContactMaterial = world.defaultContactMaterial; // null;

    /**
     * @property {Boolean} enableFrictionReduction
     */
    this.enableFrictionReduction = false;
  }

  /**
   * Make a contact object, by using the internal pool or creating a new one.
   * @method createContactEquation
   * @param {Body} bi
   * @param {Body} bj
   * @param {Shape} si
   * @param {Shape} sj
   * @param {Shape} overrideShapeA
   * @param {Shape} overrideShapeB
   * @return {ContactEquation}
   */
  createContactEquation(bi: Body, bj: Body, si: Shape, sj: Shape, overrideShapeA: Shape, overrideShapeB: Shape): ContactEquation {
    let c;
    if (this.contactPointPool.length) {
      c = this.contactPointPool.pop();
      c.bi = bi;
      c.bj = bj;
    } else {
      c = new ContactEquation(bi, bj);
    }

    c.enabled = bi.collisionResponse && bj.collisionResponse && si.collisionResponse && sj.collisionResponse;

    const cm = this.currentContactMaterial;

    c.restitution = cm.restitution;

    c.setSpookParams(
      cm.contactEquationStiffness,
      cm.contactEquationRelaxation,
      this.world.dt
    );

    const matA = si.material || bi.material;
    const matB = sj.material || bj.material;
    if (matA && matB && matA.restitution >= 0 && matB.restitution >= 0) {
      c.restitution = matA.restitution * matB.restitution;
    }

    c.si = overrideShapeA || si;
    c.sj = overrideShapeB || sj;

    return c;
  }

  createFrictionEquationsFromContact(contactEquation: ContactEquation, outArray: FrictionEquation[]): boolean {
    const bodyA = contactEquation.bi;
    const bodyB = contactEquation.bj;
    const shapeA = contactEquation.si;
    const shapeB = contactEquation.sj;

    const world = this.world;
    const cm = this.currentContactMaterial;

    // If friction or restitution were specified in the material, use them
    let friction = cm.friction;
    const matA = shapeA.material || bodyA.material;
    const matB = shapeB.material || bodyB.material;
    if (matA && matB && matA.friction >= 0 && matB.friction >= 0) {
      friction = matA.friction * matB.friction;
    }

    if (friction > 0) {
      // Create 2 tangent equations
      const mug = friction * world.gravity.length();
      let reducedMass = (bodyA.invMass + bodyB.invMass);
      if (reducedMass > 0) {
        reducedMass = 1 / reducedMass;
      }
      const pool = this.frictionEquationPool;
      const c1 = pool.length ? pool.pop() : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
      const c2 = pool.length ? pool.pop() : new FrictionEquation(bodyA, bodyB, mug * reducedMass);

      c1.bi = c2.bi = bodyA;
      c1.bj = c2.bj = bodyB;
      c1.minForce = c2.minForce = -mug * reducedMass;
      c1.maxForce = c2.maxForce = mug * reducedMass;

      // Copy over the relative vectors
      c1.ri.copy(contactEquation.ri);
      c1.rj.copy(contactEquation.rj);
      c2.ri.copy(contactEquation.ri);
      c2.rj.copy(contactEquation.rj);

      // Construct tangents
      contactEquation.ni.tangents(c1.t, c2.t);

      // Set spook params
      c1.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);
      c2.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);

      c1.enabled = c2.enabled = contactEquation.enabled;

      outArray.push(c1, c2);

      return true;
    }

    return false;
  }

  private averageNormal = new Vec3();
  private averageContactPointA = new Vec3();
  private averageContactPointB = new Vec3();

  // Take the average N latest contact point on the plane.
  createFrictionFromAverage(numContacts: number) {
    // The last contactEquation
    let c = this.result[this.result.length - 1];

    // Create the result: two "average" friction equations
    if (!this.createFrictionEquationsFromContact(c, this.frictionResult) || numContacts === 1) {
      return;
    }

    const f1 = this.frictionResult[this.frictionResult.length - 2];
    const f2 = this.frictionResult[this.frictionResult.length - 1];

    this.averageNormal.setZero();
    this.averageContactPointA.setZero();
    this.averageContactPointB.setZero();

    const bodyA = c.bi;
    const bodyB = c.bj;
    for (let i = 0; i !== numContacts; i++) {
      c = this.result[this.result.length - 1 - i];
      if (c.bi !== bodyA) {
        this.averageNormal.vadd(c.ni, this.averageNormal);
        this.averageContactPointA.vadd(c.ri, this.averageContactPointA);
        this.averageContactPointB.vadd(c.rj, this.averageContactPointB);
      } else {
        this.averageNormal.vsub(c.ni, this.averageNormal);
        this.averageContactPointA.vadd(c.rj, this.averageContactPointA);
        this.averageContactPointB.vadd(c.ri, this.averageContactPointB);
      }
    }

    const invNumContacts = 1 / numContacts;
    this.averageContactPointA.scale(invNumContacts, f1.ri);
    this.averageContactPointB.scale(invNumContacts, f1.rj);
    f2.ri.copy(f1.ri); // Should be the same
    f2.rj.copy(f1.rj);
    this.averageNormal.normalize();
    this.averageNormal.tangents(f1.t, f2.t);
    // return eq;
  }

  private tmpVec1 = new Vec3();
  private tmpVec2 = new Vec3();
  private tmpQuat1 = new Quaternion();
  private tmpQuat2 = new Quaternion();
  /**
   * Generate all contacts between a list of body pairs
   * @method getContacts
   * @param {array} p1 Array of body indices
   * @param {array} p2 Array of body indices
   * @param {World} world
   * @param {array} result Array to store generated contacts
   * @param {array} oldcontacts Optional. Array of reusable contact objects
   */
  getContacts(p1: Body[], p2: Body[], world: World, result: any[], oldcontacts: any[], frictionResult: any, frictionPool: any) {
    // Save old contact objects
    this.contactPointPool = oldcontacts;
    this.frictionEquationPool = frictionPool;
    this.result = result;
    this.frictionResult = frictionResult;

    const qi = this.tmpQuat1;
    const qj = this.tmpQuat2;
    const xi = this.tmpVec1;
    const xj = this.tmpVec2;

    const N = p1.length;
    let k = N;
    while (k--) {
      // Get current collision bodies
      const bi = p1[k],
        bj = p2[k];

      // Get contact material
      let bodyContactMaterial = null;
      if (bi.material && bj.material) {
        bodyContactMaterial = world.getContactMaterial(bi.material, bj.material) || null;
      }

      // JustTest is used to know if we're just reporting a collision
      // e.g. triggers, etc
      const justTest = !!(
        ((bi.type & Body.KINEMATIC) && (bj.type & Body.STATIC))
        ||
        ((bi.type & Body.STATIC) && (bj.type & Body.KINEMATIC))
        ||
        ((bi.type & Body.KINEMATIC) && (bj.type & Body.KINEMATIC))
      );

      // bodies can have many shapes attached
      let i = bi.shapes.length;
      while (i--) {
        bi.quaternion.mult(bi.shapeOrientations[i], qi);
        bi.quaternion.vmult(bi.shapeOffsets[i], xi);
        xi.vadd(bi.position, xi);
        const si = bi.shapes[i];

        let j = bj.shapes.length;
        while (j--) {
          // Compute world transform of shapes
          bj.quaternion.mult(bj.shapeOrientations[j], qj);
          bj.quaternion.vmult(bj.shapeOffsets[j], xj);
          xj.vadd(bj.position, xj);
          const sj = bj.shapes[j];

          // If the shapes are in some sort of group (layer) and not in the same group skip
          if (!((si.collisionFilterMask & sj.collisionFilterGroup)
            && (sj.collisionFilterMask & si.collisionFilterGroup))) {
            continue;
          }

          if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
            continue;
          }

          // Get collision material
          let shapeContactMaterial = null;
          if (si.material && sj.material) {
            shapeContactMaterial = world.getContactMaterial(si.material, sj.material) || null;
          }

          this.currentContactMaterial = shapeContactMaterial
              || bodyContactMaterial
              || world.defaultContactMaterial;

          // Get contacts
          // let resolver = this[si.type | sj.type];
          // if (resolver) {
          //   let retval = false;
          //   if (si.type < sj.type) {
          //     retval = resolver.call(this, si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
          //   } else {
          //     retval = resolver.call(this, sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
          //   }

          //   if (retval && justTest) {
          //     // Register overlap
          //     world.shapeOverlapKeeper.set(si.id, sj.id);
          //     world.bodyOverlapKeeper.set(bi.id, bj.id);
          //   }
          // }

          const aFirst = (si.type < sj.type);
          let retval = false;
          switch (si.type | sj.type) {
            case (Shape.types.CONVEXPOLYHEDRON | Shape.types.CONVEXPOLYHEDRON):
              retval = (aFirst)
                ? this.convexConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.convexConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.CONVEXPOLYHEDRON | Shape.types.PARTICLE):
              retval = (aFirst)
                ? this.convexParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.convexParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.BOX | Shape.types.CONVEXPOLYHEDRON):
              retval = (aFirst)
                ? this.boxConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.boxConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.SPHERE | Shape.types.CONVEXPOLYHEDRON):
              retval = (aFirst)
                ? this.sphereConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.sphereConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.BOX | Shape.types.BOX):
              retval = (aFirst)
                ? this.boxBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.boxBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.BOX | Shape.types.PARTICLE):
              retval = (aFirst)
                ? this.boxParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.boxParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.SPHERE | Shape.types.BOX):
              retval = (aFirst)
                ? this.sphereBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.sphereBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.SPHERE | Shape.types.SPHERE):
              retval = (aFirst)
                ? this.sphereSphere(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.sphereSphere(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.SPHERE | Shape.types.PLANE):
              retval = (aFirst)
                ? this.spherePlane(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.spherePlane(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;

            case (Shape.types.SPHERE | Shape.types.PARTICLE):
              retval = (aFirst)
                ? this.sphereParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.sphereParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.PLANE | Shape.types.BOX):
              retval = (aFirst)
                ? this.planeBox(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.planeBox(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.PLANE | Shape.types.CONVEXPOLYHEDRON):
              retval = (aFirst)
                ? this.planeConvex(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.planeConvex(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
            case (Shape.types.PLANE | Shape.types.PARTICLE):
              retval = (aFirst)
                ? this.planeParticle(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest)
                : this.planeParticle(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
              break;
          }

          if (retval && justTest) {
            // Register overlap
            world.shapeOverlapKeeper.set(si.id, sj.id);
            world.bodyOverlapKeeper.set(bi.id, bj.id);
          }
        }
      }
    }
  }

  boxBox(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean): boolean {

    (<Box>si).convexPolyhedronRepresentation.material = si.material;
    (<Box>sj).convexPolyhedronRepresentation.material = sj.material;
    (<Box>si).convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    (<Box>sj).convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    return this.convexConvex(
      (<Box>si).convexPolyhedronRepresentation, (<Box>sj).convexPolyhedronRepresentation,
      xi, xj,
      qi, qj,
      bi, bj,
      si, sj,
      justTest);
  }

  boxConvex(si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean) {
    (<Box>si).convexPolyhedronRepresentation.material = si.material;
    (<Box>si).convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexConvex((<Box>si).convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  boxParticle(si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean) {
    (<Box>si).convexPolyhedronRepresentation.material = si.material;
    (<Box>si).convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexParticle((<Box>si).convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  /**
   * @method sphereSphere
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  sphereSphere(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi?: Shape, rsj?: Shape,
    justTest?: boolean): boolean {

    const ssi = (<Sphere>si);
    const ssj = (<Sphere>sj);

    if (justTest) {
      return xi.distanceSquared(xj) < Math.pow(ssi.radius + ssj.radius, 2);
    }

    // We will have only one contact in this case
    const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);

    // Contact normal
    xj.vsub(xi, r.ni);
    r.ni.normalize();

    // Contact point locations
    r.ri.copy(r.ni);
    r.rj.copy(r.ni);
    r.ri.mult(ssi.radius, r.ri);
    r.rj.mult(-ssj.radius, r.rj);

    r.ri.vadd(xi, r.ri);
    r.ri.vsub(bi.position, r.ri);

    r.rj.vadd(xj, r.rj);
    r.rj.vsub(bj.position, r.rj);

    this.result.push(r);

    this.createFrictionEquationsFromContact(r, this.frictionResult);

    return false;
  }

  // private planeTrimesh_normal = new Vec3();
  // private planeTrimesh_relpos = new Vec3();
  // private planeTrimesh_projected = new Vec3();
  // /**
  //  * @method planeTrimesh
  //  * @param  {Shape}      si
  //  * @param  {Shape}      sj
  //  * @param  {Vec3}       xi
  //  * @param  {Vec3}       xj
  //  * @param  {Quaternion} qi
  //  * @param  {Quaternion} qj
  //  * @param  {Body}       bi
  //  * @param  {Body}       bj
  //  */
  // planeTrimesh(
  //   planeShape,
  //   trimeshShape,
  //   planePos,
  //   trimeshPos,
  //   planeQuat,
  //   trimeshQuat,
  //   planeBody,
  //   trimeshBody,
  //   rsi,
  //   rsj,
  //   justTest
  // ) {
  //   // Make contacts!
  //   let v = new Vec3();

  //   let normal = planeTrimesh_normal;
  //   normal.set(0, 0, 1);
  //   planeQuat.vmult(normal, normal); // Turn normal according to plane

  //   for (let i = 0; i < trimeshShape.vertices.length / 3; i++) {

  //     // Get world vertex from trimesh
  //     trimeshShape.getVertex(i, v);

  //     // Safe up
  //     let v2 = new Vec3();
  //     v2.copy(v);
  //     Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

  //     // Check plane side
  //     let relpos = planeTrimesh_relpos;
  //     v.vsub(planePos, relpos);
  //     let dot = normal.dot(relpos);

  //     if (dot <= 0.0) {
  //       if (justTest) {
  //         return true;
  //       }

  //       let r = this.createContactEquation(planeBody, trimeshBody, planeShape, trimeshShape, rsi, rsj);

  //       r.ni.copy(normal); // Contact normal is the plane normal

  //       // Get vertex position projected on plane
  //       let projected = planeTrimesh_projected;
  //       normal.scale(relpos.dot(normal), projected);
  //       v.vsub(projected, projected);

  //       // ri is the projected world position minus plane position
  //       r.ri.copy(projected);
  //       r.ri.vsub(planeBody.position, r.ri);

  //       r.rj.copy(v);
  //       r.rj.vsub(trimeshBody.position, r.rj);

  //       // Store result
  //       this.result.push(r);
  //       this.createFrictionEquationsFromContact(r, this.frictionResult);
  //     }
  //   }
  // };

  // private sphereTrimesh_normal = new Vec3();
  // private sphereTrimesh_relpos = new Vec3();
  // private sphereTrimesh_projected = new Vec3();
  // private sphereTrimesh_v = new Vec3();
  // private sphereTrimesh_v2 = new Vec3();
  // private sphereTrimesh_edgeVertexA = new Vec3();
  // private sphereTrimesh_edgeVertexB = new Vec3();
  // private sphereTrimesh_edgeVector = new Vec3();
  // private sphereTrimesh_edgeVectorUnit = new Vec3();
  // private sphereTrimesh_localSpherePos = new Vec3();
  // private sphereTrimesh_tmp = new Vec3();
  // private sphereTrimesh_va = new Vec3();
  // private sphereTrimesh_vb = new Vec3();
  // private sphereTrimesh_vc = new Vec3();
  // private sphereTrimesh_localSphereAABB = new AABB();
  // private sphereTrimesh_triangles = [];
  // /**
  //  * @method sphereTrimesh
  //  * @param  {Shape}      sphereShape
  //  * @param  {Shape}      trimeshShape
  //  * @param  {Vec3}       spherePos
  //  * @param  {Vec3}       trimeshPos
  //  * @param  {Quaternion} sphereQuat
  //  * @param  {Quaternion} trimeshQuat
  //  * @param  {Body}       sphereBody
  //  * @param  {Body}       trimeshBody
  //  */
  // sphereTrimesh(
  //   sphereShape,
  //   trimeshShape,
  //   spherePos,
  //   trimeshPos,
  //   sphereQuat,
  //   trimeshQuat,
  //   sphereBody,
  //   trimeshBody,
  //   rsi,
  //   rsj,
  //   justTest
  // ) {

  //   let edgeVertexA = sphereTrimesh_edgeVertexA;
  //   let edgeVertexB = sphereTrimesh_edgeVertexB;
  //   let edgeVector = sphereTrimesh_edgeVector;
  //   let edgeVectorUnit = sphereTrimesh_edgeVectorUnit;
  //   let localSpherePos = sphereTrimesh_localSpherePos;
  //   let tmp = sphereTrimesh_tmp;
  //   let localSphereAABB = sphereTrimesh_localSphereAABB;
  //   let v2 = sphereTrimesh_v2;
  //   let relpos = sphereTrimesh_relpos;
  //   let triangles = sphereTrimesh_triangles;

  //   // Convert sphere position to local in the trimesh
  //   Transform.pointToLocalFrame(trimeshPos, trimeshQuat, spherePos, localSpherePos);

  //   // Get the aabb of the sphere locally in the trimesh
  //   let sphereRadius = sphereShape.radius;
  //   localSphereAABB.lowerBound.set(
  //     localSpherePos.x - sphereRadius,
  //     localSpherePos.y - sphereRadius,
  //     localSpherePos.z - sphereRadius
  //   );
  //   localSphereAABB.upperBound.set(
  //     localSpherePos.x + sphereRadius,
  //     localSpherePos.y + sphereRadius,
  //     localSpherePos.z + sphereRadius
  //   );

  //   trimeshShape.getTrianglesInAABB(localSphereAABB, triangles);
  //   //for (let i = 0; i < trimeshShape.indices.length / 3; i++) triangles.push(i); // All

  //   // Vertices
  //   let v = sphereTrimesh_v;
  //   let radiusSquared = sphereShape.radius * sphereShape.radius;
  //   for (let i = 0; i < triangles.length; i++) {
  //     for (let j = 0; j < 3; j++) {

  //       trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + j], v);

  //       // Check vertex overlap in sphere
  //       v.vsub(localSpherePos, relpos);

  //       if (relpos.norm2() <= radiusSquared) {

  //         // Safe up
  //         v2.copy(v);
  //         Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

  //         v.vsub(spherePos, relpos);

  //         if (justTest) {
  //           return true;
  //         }

  //         let r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);
  //         r.ni.copy(relpos);
  //         r.ni.normalize();

  //         // ri is the vector from sphere center to the sphere surface
  //         r.ri.copy(r.ni);
  //         r.ri.scale(sphereShape.radius, r.ri);
  //         r.ri.vadd(spherePos, r.ri);
  //         r.ri.vsub(sphereBody.position, r.ri);

  //         r.rj.copy(v);
  //         r.rj.vsub(trimeshBody.position, r.rj);

  //         // Store result
  //         this.result.push(r);
  //         this.createFrictionEquationsFromContact(r, this.frictionResult);
  //       }
  //     }
  //   }

  //   // Check all edges
  //   for (let i = 0; i < triangles.length; i++) {
  //     for (let j = 0; j < 3; j++) {

  //       trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + j], edgeVertexA);
  //       trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + ((j + 1) % 3)], edgeVertexB);
  //       edgeVertexB.vsub(edgeVertexA, edgeVector);

  //       // Project sphere position to the edge
  //       localSpherePos.vsub(edgeVertexB, tmp);
  //       let positionAlongEdgeB = tmp.dot(edgeVector);

  //       localSpherePos.vsub(edgeVertexA, tmp);
  //       let positionAlongEdgeA = tmp.dot(edgeVector);

  //       if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {

  //         // Now check the orthogonal distance from edge to sphere center
  //         localSpherePos.vsub(edgeVertexA, tmp);

  //         edgeVectorUnit.copy(edgeVector);
  //         edgeVectorUnit.normalize();
  //         positionAlongEdgeA = tmp.dot(edgeVectorUnit);

  //         edgeVectorUnit.scale(positionAlongEdgeA, tmp);
  //         tmp.vadd(edgeVertexA, tmp);

  //         // tmp is now the sphere center position projected to the edge, defined locally in the trimesh frame
  //         let dist = tmp.distanceTo(localSpherePos);
  //         if (dist < sphereShape.radius) {

  //           if (justTest) {
  //             return true;
  //           }

  //           let r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

  //           tmp.vsub(localSpherePos, r.ni);
  //           r.ni.normalize();
  //           r.ni.scale(sphereShape.radius, r.ri);

  //           Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
  //           tmp.vsub(trimeshBody.position, r.rj);

  //           Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
  //           Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

  //           this.result.push(r);
  //           this.createFrictionEquationsFromContact(r, this.frictionResult);
  //         }
  //       }
  //     }
  //   }

  //   // Triangle faces
  //   let va = sphereTrimesh_va;
  //   let vb = sphereTrimesh_vb;
  //   let vc = sphereTrimesh_vc;
  //   let normal = sphereTrimesh_normal;
  //   for (let i = 0, N = triangles.length; i !== N; i++) {
  //     trimeshShape.getTriangleVertices(triangles[i], va, vb, vc);
  //     trimeshShape.getNormal(triangles[i], normal);
  //     localSpherePos.vsub(va, tmp);
  //     let dist = tmp.dot(normal);
  //     normal.scale(dist, tmp);
  //     localSpherePos.vsub(tmp, tmp);

  //     // tmp is now the sphere position projected to the triangle plane
  //     dist = tmp.distanceTo(localSpherePos);
  //     if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < sphereShape.radius) {
  //       if (justTest) {
  //         return true;
  //       }
  //       let r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

  //       tmp.vsub(localSpherePos, r.ni);
  //       r.ni.normalize();
  //       r.ni.scale(sphereShape.radius, r.ri);

  //       Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
  //       tmp.vsub(trimeshBody.position, r.rj);

  //       Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
  //       Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

  //       this.result.push(r);
  //       this.createFrictionEquationsFromContact(r, this.frictionResult);
  //     }
  //   }

  //   triangles.length = 0;
  // };

  private point_on_plane_to_sphere = new Vec3();
  private plane_to_sphere_ortho = new Vec3();
  /**
   * @method spherePlane
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  spherePlane(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean): boolean {

    // We will have one contact in this case
    const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);

    // Contact normal
    r.ni.set(0, 0, 1);
    qj.vmult(r.ni, r.ni);
    r.ni.negate(r.ni); // body i is the sphere, flip normal
    r.ni.normalize(); // Needed?

    const ssi = (<Sphere>si);
    const psj = (<Plane>sj);

    // Vector from sphere center to contact point
    r.ni.mult(ssi.radius, r.ri);

    // Project down sphere on plane
    xi.vsub(xj, this.point_on_plane_to_sphere);
    r.ni.mult(r.ni.dot(this.point_on_plane_to_sphere), this.plane_to_sphere_ortho);
    this.point_on_plane_to_sphere.vsub(this.plane_to_sphere_ortho, r.rj); // The sphere position projected to plane

    if (-this.point_on_plane_to_sphere.dot(r.ni) <= ssi.radius) {

      if (justTest) {
        return true;
      }

      // Make it relative to the body
      const ri = r.ri;
      const rj = r.rj;
      ri.vadd(xi, ri);
      ri.vsub(bi.position, ri);
      rj.vadd(xj, rj);
      rj.vsub(bj.position, rj);

      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }

    return false;
  }

  // See http://bulletphysics.com/Bullet/BulletFull/SphereTriangleDetector_8cpp_source.html
  private pointInPolygon_edge = new Vec3();
  private pointInPolygon_edge_x_normal = new Vec3();
  private pointInPolygon_vtp = new Vec3();
  pointInPolygon(verts: Vec3[], normal: Vec3, p: Vec3) {
    let positiveResult = null;
    const N = verts.length;
    for (let i = 0; i !== N; i++) {
      const v = verts[i];

      // Get edge to the next vertex
      const edge = this.pointInPolygon_edge;
      verts[(i + 1) % (N)].vsub(v, edge);

      // Get cross product between polygon normal and the edge
      const edge_x_normal = this.pointInPolygon_edge_x_normal;
      // let edge_x_normal = new Vec3();
      edge.cross(normal, edge_x_normal);

      // Get vector between point and current vertex
      const vertex_to_p = this.pointInPolygon_vtp;
      p.vsub(v, vertex_to_p);

      // This dot product determines which side of the edge the point is
      const r = edge_x_normal.dot(vertex_to_p);

      // If all such dot products have same sign, we are inside the polygon.
      if (positiveResult === null || (r > 0 && positiveResult === true) || (r <= 0 && positiveResult === false)) {
        if (positiveResult === null) {
          positiveResult = r > 0;
        }
        continue;
      } else {
        return false; // Encountered some other sign. Exit.
      }
    }

    // If we got here, all dot products were of the same sign.
    return true;
  }

  private box_to_sphere = new Vec3();
  private sphereBox_ns = new Vec3();
  private sphereBox_ns1 = new Vec3();
  private sphereBox_ns2 = new Vec3();
  private sphereBox_sides = [new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3()];
  private sphereBox_sphere_to_corner = new Vec3();
  private sphereBox_side_ns = new Vec3();
  private sphereBox_side_ns1 = new Vec3();
  private sphereBox_side_ns2 = new Vec3();
  /**
   * @method sphereBox
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  sphereBox(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean): boolean {
    const v3pool = this.v3pool;

    const ssi = <Sphere>si;
    const bsj = <Box>sj;

    // we refer to the box as body j
    const sides = this.sphereBox_sides;
    xi.vsub(xj, this.box_to_sphere);
    bsj.getSideNormals(sides, qj);
    const R = ssi.radius;
    const penetrating_sides = [];

    // Check side (plane) intersections
    let found = false;

    // Store the resulting side penetration info
    const side_ns = this.sphereBox_side_ns;
    const side_ns1 = this.sphereBox_side_ns1;
    const side_ns2 = this.sphereBox_side_ns2;
    let side_h = null;
    let side_penetrations = 0;
    let side_dot1 = 0;
    let side_dot2 = 0;
    let side_distance = null;
    for (let idx = 0, nsides = sides.length; idx !== nsides && found === false; idx++) {
      // Get the plane side normal (ns)
      const ns = this.sphereBox_ns;
      ns.copy(sides[idx]);

      const h = ns.norm();
      ns.normalize();

      // The normal/distance dot product tells which side of the plane we are
      const dot = this.box_to_sphere.dot(ns);

      if (dot < h + R && dot > 0) {
        // Intersects plane. Now check the other two dimensions
        const ns1 = this.sphereBox_ns1;
        const ns2 = this.sphereBox_ns2;
        ns1.copy(sides[(idx + 1) % 3]);
        ns2.copy(sides[(idx + 2) % 3]);
        const h1 = ns1.norm();
        const h2 = ns2.norm();
        ns1.normalize();
        ns2.normalize();
        const dot1 = this.box_to_sphere.dot(ns1);
        const dot2 = this.box_to_sphere.dot(ns2);
        if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
          const dist1 = Math.abs(dot - h - R);
          if (side_distance === null || dist1 < side_distance) {
            side_distance = dist1;
            side_dot1 = dot1;
            side_dot2 = dot2;
            side_h = h;
            side_ns.copy(ns);
            side_ns1.copy(ns1);
            side_ns2.copy(ns2);
            side_penetrations++;

            if (justTest) {
              return true;
            }
          }
        }
      }
    }
    if (side_penetrations) {
      found = true;
      const r1 = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      side_ns.mult(-R, r1.ri); // Sphere r
      r1.ni.copy(side_ns);
      r1.ni.negate(r1.ni); // Normal should be out of sphere
      side_ns.mult(side_h, side_ns);
      side_ns1.mult(side_dot1, side_ns1);
      side_ns.vadd(side_ns1, side_ns);
      side_ns2.mult(side_dot2, side_ns2);
      side_ns.vadd(side_ns2, r1.rj);

      // Make relative to bodies
      r1.ri.vadd(xi, r1.ri);
      r1.ri.vsub(bi.position, r1.ri);
      r1.rj.vadd(xj, r1.rj);
      r1.rj.vsub(bj.position, r1.rj);

      this.result.push(r1);
      this.createFrictionEquationsFromContact(r1, this.frictionResult);
    }

    // Check corners
    let rj = v3pool.get();
    const sphere_to_corner = this.sphereBox_sphere_to_corner;
    for (let j = 0; j !== 2 && !found; j++) {
      for (let k = 0; k !== 2 && !found; k++) {
        for (let l = 0; l !== 2 && !found; l++) {
          rj.set(0, 0, 0);
          if (j) {
            rj.vadd(sides[0], rj);
          } else {
            rj.vsub(sides[0], rj);
          }
          if (k) {
            rj.vadd(sides[1], rj);
          } else {
            rj.vsub(sides[1], rj);
          }
          if (l) {
            rj.vadd(sides[2], rj);
          } else {
            rj.vsub(sides[2], rj);
          }

          // World position of corner
          xj.vadd(rj, sphere_to_corner);
          sphere_to_corner.vsub(xi, sphere_to_corner);

          if (sphere_to_corner.norm2() < R * R) {
            if (justTest) {
              return true;
            }
            found = true;
            const r1 = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            r1.ri.copy(sphere_to_corner);
            r1.ri.normalize();
            r1.ni.copy(r1.ri);
            r1.ri.mult(R, r1.ri);
            r1.rj.copy(rj);

            // Make relative to bodies
            r1.ri.vadd(xi, r1.ri);
            r1.ri.vsub(bi.position, r1.ri);
            r1.rj.vadd(xj, r1.rj);
            r1.rj.vsub(bj.position, r1.rj);

            this.result.push(r1);
            this.createFrictionEquationsFromContact(r1, this.frictionResult);
          }
        }
      }
    }
    v3pool.release(rj);
    rj = null;

    // Check edges
    const edgeTangent = v3pool.get();
    const edgeCenter = v3pool.get();
    const r = v3pool.get(); // r = edge center to sphere center
    const orthogonal = v3pool.get();
    const dist = <Vec3>v3pool.get();
    const Nsides = sides.length;
    for (let j = 0; j !== Nsides && !found; j++) {
      for (let k = 0; k !== Nsides && !found; k++) {
        if (j % 3 !== k % 3) {
          // Get edge tangent
          sides[k].cross(sides[j], edgeTangent);
          edgeTangent.normalize();
          sides[j].vadd(sides[k], edgeCenter);
          r.copy(xi);
          r.vsub(edgeCenter, r);
          r.vsub(xj, r);
          const orthonorm = r.dot(edgeTangent); // distance from edge center to sphere center in the tangent direction
          edgeTangent.mult(orthonorm, orthogonal); // Vector from edge center to sphere center in the tangent direction

          // Find the third side orthogonal to this one
          let l = 0;
          while (l === j % 3 || l === k % 3) {
            l++;
          }

          // vec from edge center to sphere projected to the plane orthogonal to the edge tangent
          dist.copy(xi);
          dist.vsub(orthogonal, dist);
          dist.vsub(edgeCenter, dist);
          dist.vsub(xj, dist);

          // Distances in tangent direction and distance in the plane orthogonal to it
          const tdist = Math.abs(orthonorm);
          const ndist = dist.norm();

          if (tdist < sides[l].norm() && ndist < R) {
            if (justTest) {
              return true;
            }
            found = true;
            const res = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            edgeCenter.vadd(orthogonal, res.rj); // box rj
            res.rj.copy(res.rj);
            dist.negate(res.ni);
            res.ni.normalize();

            res.ri.copy(res.rj);
            res.ri.vadd(xj, res.ri);
            res.ri.vsub(xi, res.ri);
            res.ri.normalize();
            res.ri.mult(R, res.ri);

            // Make relative to bodies
            res.ri.vadd(xi, res.ri);
            res.ri.vsub(bi.position, res.ri);
            res.rj.vadd(xj, res.rj);
            res.rj.vsub(bj.position, res.rj);

            this.result.push(res);
            this.createFrictionEquationsFromContact(res, this.frictionResult);
          }
        }
      }
    }
    v3pool.release(edgeTangent, edgeCenter, r, orthogonal, dist);

    return false;
  }

  private convex_to_sphere = new Vec3();
  private sphereConvex_edge = new Vec3();
  private sphereConvex_edgeUnit = new Vec3();
  private sphereConvex_sphereToCorner = new Vec3();
  private sphereConvex_worldCorner = new Vec3();
  private sphereConvex_worldNormal = new Vec3();
  private sphereConvex_worldPoint = new Vec3();
  private sphereConvex_worldSpherePointClosestToPlane = new Vec3();
  private sphereConvex_penetrationVec = new Vec3();
  private sphereConvex_sphereToWorldPoint = new Vec3();
  /**
   * @method sphereConvex
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  sphereConvex(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean): boolean {
    const v3pool = this.v3pool;

    const ssi = <Sphere>si;
    const csj = <ConvexPolyhedron>sj;

    xi.vsub(xj, this.convex_to_sphere);
    const normals = csj.faceNormals;
    const faces = csj.faces;
    const verts = csj.vertices;
    const R = ssi.radius;
    // const penetrating_sides = [];

    // if (this.convex_to_sphere.norm2() > si.boundingSphereRadius + sj.boundingSphereRadius) {
    //   return false;
    // }

    // Check corners
    let i = verts.length;
    while (i--) {
      const v = verts[i];

      // World position of corner
      const worldCorner = this.sphereConvex_worldCorner;
      qj.vmult(v, worldCorner);
      xj.vadd(worldCorner, worldCorner);
      const sphere_to_corner = this.sphereConvex_sphereToCorner;
      worldCorner.vsub(xi, sphere_to_corner);
      if (sphere_to_corner.norm2() < R * R) {
        if (justTest) {
          return true;
        }
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ri.copy(sphere_to_corner);
        r.ri.normalize();
        r.ni.copy(r.ri);
        r.ri.mult(R, r.ri);
        worldCorner.vsub(xj, r.rj);

        // Should be relative to the body.
        r.ri.vadd(xi, r.ri);
        r.ri.vsub(bi.position, r.ri);

        // Should be relative to the body.
        r.rj.vadd(xj, r.rj);
        r.rj.vsub(bj.position, r.rj);

        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
        return false;
      }
    }

    // Check side (plane) intersections
    let found = false;
    const nfaces = faces.length;
    i = nfaces;
    while (i-- && found === false) {
      const normal = normals[i];
      const face = faces[i];

      // Get world-transformed normal of the face
      const worldNormal = this.sphereConvex_worldNormal;
      qj.vmult(normal, worldNormal);

      // Get a world vertex from the face
      const worldPoint = this.sphereConvex_worldPoint;
      qj.vmult(verts[face[0]], worldPoint);
      worldPoint.vadd(xj, worldPoint);

      // Get a point on the sphere, closest to the face normal
      const worldSpherePointClosestToPlane = this.sphereConvex_worldSpherePointClosestToPlane;
      worldNormal.mult(-R, worldSpherePointClosestToPlane);
      xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);

      // Vector from a face point to the closest point on the sphere
      const penetrationVec = this.sphereConvex_penetrationVec;
      worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);

      // The penetration. Negative value means overlap.
      const penetration = penetrationVec.dot(worldNormal);

      const worldPointToSphere = this.sphereConvex_sphereToWorldPoint;
      xi.vsub(worldPoint, worldPointToSphere);

      if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
        // Intersects plane. Now check if the sphere is inside the face polygon
        const faceVerts = []; // Face vertices, in world coords
        const Nverts = face.length;
        let j = Nverts;
        while (j--) {
          const worldVertex = v3pool.get();
          qj.vmult(verts[face[j]], worldVertex);
          xj.vadd(worldVertex, worldVertex);
          faceVerts.push(worldVertex);
        }

        if (this.pointInPolygon(faceVerts, worldNormal, xi)) { // Is the sphere center in the face polygon?
          if (justTest) {
            return true;
          }
          found = true;
          const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);

          worldNormal.mult(-R, r.ri); // Contact offset, from sphere center to contact
          worldNormal.negate(r.ni); // Normal pointing out of sphere

          const penetrationVec2 = v3pool.get();
          worldNormal.mult(-penetration, penetrationVec2);
          const penetrationSpherePoint = v3pool.get();
          worldNormal.mult(-R, penetrationSpherePoint);

          xi.vsub(xj, r.rj);
          r.rj.vadd(penetrationSpherePoint, r.rj);
          r.rj.vadd(penetrationVec2, r.rj);

          // Should be relative to the body.
          r.rj.vadd(xj, r.rj);
          r.rj.vsub(bj.position, r.rj);

          // Should be relative to the body.
          r.ri.vadd(xi, r.ri);
          r.ri.vsub(bi.position, r.ri);

          v3pool.release(penetrationVec2);
          v3pool.release(penetrationSpherePoint);

          this.result.push(r);
          this.createFrictionEquationsFromContact(r, this.frictionResult);

          // Release world vertices
          // const Nfaceverts = faceVerts.length;
          j = faceVerts.length; // Nfaceverts;
          while (j--) {
            v3pool.release(faceVerts[j]);
          }

          return false; // We only expect *one* face contact
        } else {
          // Edge?
          j = face.length;
          while (j--) {
            // Get two world transformed vertices
            const v1 = v3pool.get();
            const v2 = v3pool.get();
            qj.vmult(verts[face[(j + 1) % face.length]], v1);
            qj.vmult(verts[face[(j + 2) % face.length]], v2);
            xj.vadd(v1, v1);
            xj.vadd(v2, v2);

            // Construct edge vector
            const edge = this.sphereConvex_edge;
            v2.vsub(v1, edge);

            // Construct the same vector, but normalized
            const edgeUnit = this.sphereConvex_edgeUnit;
            edge.unit(edgeUnit);

            // p is xi projected onto the edge
            const p = v3pool.get();
            const v1_to_xi = v3pool.get();
            xi.vsub(v1, v1_to_xi);
            const dot = v1_to_xi.dot(edgeUnit);
            edgeUnit.mult(dot, p);
            p.vadd(v1, p);

            // Compute a vector from p to the center of the sphere
            const xi_to_p = v3pool.get();
            p.vsub(xi, xi_to_p);

            // Collision if the edge-sphere distance is less than the radius
            // AND if p is in between v1 and v2
            if (dot > 0 && dot * dot < edge.norm2() && xi_to_p.norm2() < R * R) {
              // Collision if the edge-sphere distance is less than the radius
              // Edge contact!
              if (justTest) {
                return true;
              }
              const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
              p.vsub(xj, r.rj);

              p.vsub(xi, r.ni);
              r.ni.normalize();

              r.ni.mult(R, r.ri);

              // Should be relative to the body.
              r.rj.vadd(xj, r.rj);
              r.rj.vsub(bj.position, r.rj);

              // Should be relative to the body.
              r.ri.vadd(xi, r.ri);
              r.ri.vsub(bi.position, r.ri);

              this.result.push(r);
              this.createFrictionEquationsFromContact(r, this.frictionResult);

              // Release world vertices
              let jj = faceVerts.length;
              while (jj--) {
                v3pool.release(faceVerts[jj]);
              }

              v3pool.release(v1);
              v3pool.release(v2);
              v3pool.release(p);
              v3pool.release(xi_to_p);
              v3pool.release(v1_to_xi);

              return false;
            }

            v3pool.release(v1);
            v3pool.release(v2);
            v3pool.release(p);
            v3pool.release(xi_to_p);
            v3pool.release(v1_to_xi);
          }
        }

        // Release world vertices
        j = faceVerts.length;
        while (j--) {
          v3pool.release(faceVerts[j]);
        }
      }
    }
    return false;
  }

  /**
   * @method planeBox
   * @param  {Array}      result
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  planeBox(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean): boolean {

    const bsj = <Box>sj;

    bsj.convexPolyhedronRepresentation.material = sj.material;
    bsj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    bsj.convexPolyhedronRepresentation.id = sj.id;
    return this.planeConvex(si, bsj.convexPolyhedronRepresentation, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  private planeConvex_v = new Vec3();
  private planeConvex_normal = new Vec3();
  private planeConvex_relpos = new Vec3();
  private planeConvex_projected = new Vec3();
  /**
   * @method planeConvex
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  planeConvex(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean
  ): boolean {
    const planeShape = <Plane>si;
    const convexShape = <ConvexPolyhedron>sj;
    const planePosition = xi;
    const convexPosition = xj;
    const planeQuat = qi;
    const convexQuat = qj;
    const planeBody = bi;
    const convexBody = bj;

    // Simply return the points behind the plane.
    const worldVertex = this.planeConvex_v,
      worldNormal = this.planeConvex_normal;
    worldNormal.set(0, 0, 1);
    planeQuat.vmult(worldNormal, worldNormal); // Turn normal according to plane orientation

    let numContacts = 0;
    const relpos = this.planeConvex_relpos;
    for (let i = 0; i !== convexShape.vertices.length; i++) {

      // Get world convex vertex
      worldVertex.copy(convexShape.vertices[i]);
      convexQuat.vmult(worldVertex, worldVertex);
      convexPosition.vadd(worldVertex, worldVertex);
      worldVertex.vsub(planePosition, relpos);

      const dot = worldNormal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        const r = this.createContactEquation(planeBody, convexBody, planeShape, convexShape, rsi, rsj);

        // Get vertex position projected on plane
        const projected = this.planeConvex_projected;
        worldNormal.mult(worldNormal.dot(relpos), projected);
        worldVertex.vsub(projected, projected);
        projected.vsub(planePosition, r.ri); // From plane to vertex projected on plane

        r.ni.copy(worldNormal); // Contact normal is the plane normal out from plane

        // rj is now just the vector from the convex center to the vertex
        worldVertex.vsub(convexPosition, r.rj);

        // Make it relative to the body
        r.ri.vadd(planePosition, r.ri);
        r.ri.vsub(planeBody.position, r.ri);
        r.rj.vadd(convexPosition, r.rj);
        r.rj.vsub(convexBody.position, r.rj);

        this.result.push(r);
        numContacts++;
        if (!this.enableFrictionReduction) {
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
    }

    if (this.enableFrictionReduction && numContacts) {
      this.createFrictionFromAverage(numContacts);
    }

    return false;
  }

  private convexConvex_sepAxis = new Vec3();
  private convexConvex_q = new Vec3();
  /**
   * @method convexConvex
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  convexConvex(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,  // override shape?
    justTest: boolean,
    faceListA?: any[], faceListB?: any[]): boolean {
    const sepAxis = this.convexConvex_sepAxis;

    if (xi.distanceTo(xj) > (si.boundingSphereRadius + sj.boundingSphereRadius)) {
      return false;
    }

    const cpsi = <ConvexPolyhedron>si;
    let numContacts = 0;
    if (cpsi.findSeparatingAxis(<ConvexPolyhedron>sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)) {
      const res: HullResult[] = [];
      const q = this.convexConvex_q;

      cpsi.clipAgainstHull(xi, qi, <ConvexPolyhedron>sj, xj, qj, sepAxis, -100, 100, res);

      let j = res.length;
      while (j--) {
        if (justTest) {
          return true;
        }
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj),
          ri = r.ri,
          rj = r.rj;
        sepAxis.negate(r.ni);
        res[j].normal.negate(q);
        q.mult(res[j].depth, q);
        res[j].point.vadd(q, ri);
        rj.copy(res[j].point);

        // Contact points are in world coordinates. Transform back to relative
        ri.vsub(xi, ri);
        rj.vsub(xj, rj);

        // Make relative to bodies
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);

        this.result.push(r);
        numContacts++;
        if (!this.enableFrictionReduction) {
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
      if (this.enableFrictionReduction && numContacts) {
        this.createFrictionFromAverage(numContacts);
      }
    }
    return numContacts > 0;
  }

  /**
   * @method convexTrimesh
   * @param  {Array}      result
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  // Narrowphase.prototype[Shape.types.CONVEXPOLYHEDRON | Shape.types.TRIMESH] =
  // convexTrimesh(si,sj,xi,xj,qi,qj,bi,bj,rsi,rsj,faceListA,faceListB){
  //     let sepAxis = convexConvex_sepAxis;

  //     if(xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius){
  //         return;
  //     }

  //     // Construct a temp hull for each triangle
  //     let hullB = new ConvexPolyhedron();

  //     hullB.faces = [[0,1,2]];
  //     let va = new Vec3();
  //     let vb = new Vec3();
  //     let vc = new Vec3();
  //     hullB.vertices = [
  //         va,
  //         vb,
  //         vc
  //     ];

  //     for (let i = 0; i < sj.indices.length / 3; i++) {

  //         let triangleNormal = new Vec3();
  //         sj.getNormal(i, triangleNormal);
  //         hullB.faceNormals = [triangleNormal];

  //         sj.getTriangleVertices(i, va, vb, vc);

  //         let d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);
  //         if(!d){
  //             triangleNormal.scale(-1, triangleNormal);
  //             d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);

  //             if(!d){
  //                 continue;
  //             }
  //         }

  //         let res = [];
  //         let q = convexConvex_q;
  //         si.clipAgainstHull(xi,qi,hullB,xj,qj,triangleNormal,-100,100,res);
  //         for(let j = 0; j !== res.length; j++){
  //             let r = this.createContactEquation(bi,bj,si,sj,rsi,rsj),
  //                 ri = r.ri,
  //                 rj = r.rj;
  //             r.ni.copy(triangleNormal);
  //             r.ni.negate(r.ni);
  //             res[j].normal.negate(q);
  //             q.mult(res[j].depth, q);
  //             res[j].point.vadd(q, ri);
  //             rj.copy(res[j].point);

  //             // Contact points are in world coordinates. Transform back to relative
  //             ri.vsub(xi,ri);
  //             rj.vsub(xj,rj);

  //             // Make relative to bodies
  //             ri.vadd(xi, ri);
  //             ri.vsub(bi.position, ri);
  //             rj.vadd(xj, rj);
  //             rj.vsub(bj.position, rj);

  //             result.push(r);
  //         }
  //     }
  // };

  private particlePlane_normal = new Vec3();
  private particlePlane_relpos = new Vec3();
  private particlePlane_projected = new Vec3();
  /**
   * @method particlePlane
   * @param  {Array}      result
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  planeParticle(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean
  ): boolean {
    const normal = this.particlePlane_normal;
    normal.set(0, 0, 1);
    bj.quaternion.vmult(normal, normal); // Turn normal according to plane orientation
    const relpos = this.particlePlane_relpos;
    xi.vsub(bj.position, relpos);
    const dot = normal.dot(relpos);
    if (dot <= 0.0) {

      if (justTest) {
        return true;
      }

      const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      r.ni.copy(normal); // Contact normal is the plane normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle

      // Get particle position projected on plane
      const projected = this.particlePlane_projected;
      normal.mult(normal.dot(xi), projected);
      xi.vsub(projected, projected);
      // projected.vadd(bj.position,projected);

      // rj is now the projected world position minus plane position
      r.rj.copy(projected);
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }

    return false;
  }

  private particleSphere_normal = new Vec3();
  /**
   * @method particleSphere
   * @param  {Array}      result
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  sphereParticle(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean
  ) {
    // TODO: these seem backwards...
    const psi = <Particle>si;
    const ssj = <Sphere>sj;

    // The normal is the unit vector from sphere center to particle center
    const normal = this.particleSphere_normal;
    normal.set(0, 0, 1);
    xi.vsub(xj, normal);
    const lengthSquared = normal.norm2();

    if (lengthSquared <= ssj.radius * ssj.radius) {
      if (justTest) {
        return true;
      }
      const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      normal.normalize();
      r.rj.copy(normal);
      r.rj.mult(ssj.radius, r.rj);
      r.ni.copy(normal); // Contact normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }

    return false;
  }

  // WIP
  private cqj = new Quaternion();
  private convexParticle_local = new Vec3();
  private convexParticle_normal = new Vec3();
  private convexParticle_penetratedFaceNormal = new Vec3();
  private convexParticle_vertexToParticle = new Vec3();
  private convexParticle_worldPenetrationVec = new Vec3();
  /**
   * @method convexParticle
   * @param  {Array}      result
   * @param  {Shape}      si
   * @param  {Shape}      sj
   * @param  {Vec3}       xi
   * @param  {Vec3}       xj
   * @param  {Quaternion} qi
   * @param  {Quaternion} qj
   * @param  {Body}       bi
   * @param  {Body}       bj
   */
  convexParticle(
    si: Shape, sj: Shape,
    xi: Vec3, xj: Vec3,
    qi: Quaternion, qj: Quaternion,
    bi: Body, bj: Body,
    rsi: Shape, rsj: Shape,
    justTest: boolean
  ): boolean {
    let penetratedFaceIndex = -1;
    const penetratedFaceNormal = this.convexParticle_penetratedFaceNormal;
    const worldPenetrationVec = this.convexParticle_worldPenetrationVec;
    let minPenetration = null;
    let numDetectedFaces = 0;

    // TODO: these seem backwards
    // const psi = <Particle>si;
    const csj = <ConvexPolyhedron>sj;

    // Convert particle position xi to local coords in the convex
    const local = this.convexParticle_local;
    local.copy(xi);
    local.vsub(xj, local); // Convert position to relative the convex origin
    qj.conjugate(this.cqj);
    this.cqj.vmult(local, local);

    if (csj.pointIsInside(local)) {

      if (csj.worldVerticesNeedsUpdate) {
        csj.computeWorldVertices(xj, qj);
      }
      if (csj.worldFaceNormalsNeedsUpdate) {
        csj.computeWorldFaceNormals(qj);
      }

      // For each world polygon in the polyhedra
      for (let i = 0, nfaces = csj.faces.length; i !== nfaces; i++) {

        // Construct world face vertices
        const verts = [csj.worldVertices[csj.faces[i][0]]];
        const normal = csj.worldFaceNormals[i];

        // Check how much the particle penetrates the polygon plane.
        xi.vsub(verts[0], this.convexParticle_vertexToParticle);
        const penetration = -normal.dot(this.convexParticle_vertexToParticle);
        if (minPenetration === null || Math.abs(penetration) < Math.abs(minPenetration)) {

          if (justTest) {
            return true;
          }

          minPenetration = penetration;
          penetratedFaceIndex = i;
          penetratedFaceNormal.copy(normal);
          numDetectedFaces++;
        }
      }

      if (penetratedFaceIndex !== -1) {
        // Setup contact
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        penetratedFaceNormal.mult(minPenetration, worldPenetrationVec);

        // rj is the particle position projected to the face
        worldPenetrationVec.vadd(xi, worldPenetrationVec);
        worldPenetrationVec.vsub(xj, worldPenetrationVec);
        r.rj.copy(worldPenetrationVec);
        // let projectedToFace = xi.vsub(xj).vadd(worldPenetrationVec);
        // projectedToFace.copy(r.rj);

        // qj.vmult(r.rj,r.rj);
        penetratedFaceNormal.negate(r.ni); // Contact normal
        r.ri.set(0, 0, 0); // Center of particle

        const ri = r.ri,
          rj = r.rj;

        // Make relative to bodies
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);

        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      } else {
        console.warn('Point found inside convex, but did not find penetrating face!');
      }
    }

    return false;
  }

  // boxHeightfield(
  //   si: Shape, sj: Shape,
  //   xi: Vec3, xj: Vec3,
  //   qi: Quaternion, qj: Quaternion,
  //   bi: Body, bj: Body,
  //   rsi: Shape, rsj: Shape,
  //   justTest: boolean
  // ): boolean {
  //   si.convexPolyhedronRepresentation.material = si.material;
  //   si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
  //   return this.convexHeightfield(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  // }

  // private convexHeightfield_tmp1 = new Vec3();
  // private convexHeightfield_tmp2 = new Vec3();
  // private convexHeightfield_faceList = [0];
  // /**
  //  * @method convexHeightfield
  //  */
  // convexHeightfield(
  //   si: Shape, sj: Shape,
  //   xi: Vec3, xj: Vec3,
  //   qi: Quaternion, qj: Quaternion,
  //   bi: Body, bj: Body,
  //   rsi: Shape, rsj: Shape,
  //   justTest: boolean
  // ) {
  //   const convexShape = <ConvexPolyhedron>si;
  //   const hfShape = <HeightField>sj;
  //   convexPos = xi;
  //   hfPos = xj;
  //   convexQuat = qi;
  //   hfQuat = qj;
  //   convexBody = bi;
  //   hfBody = bj;

  //   let data = hfShape.data,
  //     w = hfShape.elementSize,
  //     radius = convexShape.boundingSphereRadius,
  //     worldPillarOffset = convexHeightfield_tmp2,
  //     faceList = convexHeightfield_faceList;

  //   // Get sphere position to heightfield local!
  //   let localConvexPos = convexHeightfield_tmp1;
  //   Transform.pointToLocalFrame(hfPos, hfQuat, convexPos, localConvexPos);

  //   // Get the index of the data points to test against
  //   let iMinX = Math.floor((localConvexPos.x - radius) / w) - 1,
  //     iMaxX = Math.ceil((localConvexPos.x + radius) / w) + 1,
  //     iMinY = Math.floor((localConvexPos.y - radius) / w) - 1,
  //     iMaxY = Math.ceil((localConvexPos.y + radius) / w) + 1;

  //   // Bail out if we are out of the terrain
  //   if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length) {
  //     return;
  //   }

  //   // Clamp index to edges
  //   if (iMinX < 0) { iMinX = 0; }
  //   if (iMaxX < 0) { iMaxX = 0; }
  //   if (iMinY < 0) { iMinY = 0; }
  //   if (iMaxY < 0) { iMaxY = 0; }
  //   if (iMinX >= data.length) { iMinX = data.length - 1; }
  //   if (iMaxX >= data.length) { iMaxX = data.length - 1; }
  //   if (iMaxY >= data[0].length) { iMaxY = data[0].length - 1; }
  //   if (iMinY >= data[0].length) { iMinY = data[0].length - 1; }

  //   let minMax = [];
  //   hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
  //   let min = minMax[0];
  //   let max = minMax[1];

  //   // Bail out if we're cant touch the bounding height box
  //   if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
  //     return;
  //   }

  //   for (let i = iMinX; i < iMaxX; i++) {
  //     for (let j = iMinY; j < iMaxY; j++) {

  //       let intersecting = false;

  //       // Lower triangle
  //       hfShape.getConvexTrianglePillar(i, j, false);
  //       Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
  //       if (convexPos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius + convexShape.boundingSphereRadius) {
  //         intersecting = this.convexConvex(convexShape, hfShape.pillarConvex, convexPos,
  //            worldPillarOffset, convexQuat, hfQuat, convexBody, hfBody, null, null, justTest, faceList, null);
  //       }

  //       if (justTest && intersecting) {
  //         return true;
  //       }

  //       // Upper triangle
  //       hfShape.getConvexTrianglePillar(i, j, true);
  //       Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
  //       if (convexPos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius + convexShape.boundingSphereRadius) {
  //         intersecting = this.convexConvex(convexShape, hfShape.pillarConvex, convexPos,
  //              worldPillarOffset, convexQuat, hfQuat, convexBody, hfBody, null, null, justTest, faceList, null);
  //       }

  //       if (justTest && intersecting) {
  //         return true;
  //       }
  //     }
  //   }
  // }

  // private sphereHeightfield_tmp1 = new Vec3();
  // private sphereHeightfield_tmp2 = new Vec3();
  // /**
  //  * @method sphereHeightfield
  //  */
  // sphereHeightfield(
  //   sphereShape,
  //   hfShape,
  //   spherePos,
  //   hfPos,
  //   sphereQuat,
  //   hfQuat,
  //   sphereBody,
  //   hfBody,
  //   rsi,
  //   rsj,
  //   justTest
  // ) {
  //   let data = hfShape.data,
  //     radius = sphereShape.radius,
  //     w = hfShape.elementSize,
  //     worldPillarOffset = sphereHeightfield_tmp2;

  //   // Get sphere position to heightfield local!
  //   let localSpherePos = sphereHeightfield_tmp1;
  //   Transform.pointToLocalFrame(hfPos, hfQuat, spherePos, localSpherePos);

  //   // Get the index of the data points to test against
  //   let iMinX = Math.floor((localSpherePos.x - radius) / w) - 1,
  //     iMaxX = Math.ceil((localSpherePos.x + radius) / w) + 1,
  //     iMinY = Math.floor((localSpherePos.y - radius) / w) - 1,
  //     iMaxY = Math.ceil((localSpherePos.y + radius) / w) + 1;

  //   // Bail out if we are out of the terrain
  //   if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMaxY > data[0].length) {
  //     return;
  //   }

  //   // Clamp index to edges
  //   if (iMinX < 0) { iMinX = 0; }
  //   if (iMaxX < 0) { iMaxX = 0; }
  //   if (iMinY < 0) { iMinY = 0; }
  //   if (iMaxY < 0) { iMaxY = 0; }
  //   if (iMinX >= data.length) { iMinX = data.length - 1; }
  //   if (iMaxX >= data.length) { iMaxX = data.length - 1; }
  //   if (iMaxY >= data[0].length) { iMaxY = data[0].length - 1; }
  //   if (iMinY >= data[0].length) { iMinY = data[0].length - 1; }

  //   let minMax = [];
  //   hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
  //   let min = minMax[0];
  //   let max = minMax[1];

  //   // Bail out if we're cant touch the bounding height box
  //   if (localSpherePos.z - radius > max || localSpherePos.z + radius < min) {
  //     return;
  //   }

  //   let result = this.result;
  //   for (let i = iMinX; i < iMaxX; i++) {
  //     for (let j = iMinY; j < iMaxY; j++) {

  //       let numContactsBefore = result.length;

  //       let intersecting = false;

  //       // Lower triangle
  //       hfShape.getConvexTrianglePillar(i, j, false);
  //       Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
  //       if (spherePos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius + sphereShape.boundingSphereRadius) {
  //         intersecting = this.sphereConvex(sphereShape, hfShape.pillarConvex, spherePos,
  //               worldPillarOffset, sphereQuat, hfQuat, sphereBody, hfBody, sphereShape, hfShape, justTest);
  //       }

  //       if (justTest && intersecting) {
  //         return true;
  //       }

  //       // Upper triangle
  //       hfShape.getConvexTrianglePillar(i, j, true);
  //       Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
  //       if (spherePos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius + sphereShape.boundingSphereRadius) {
  //         intersecting = this.sphereConvex(sphereShape, hfShape.pillarConvex, spherePos,
  //                worldPillarOffset, sphereQuat, hfQuat, sphereBody, hfBody, sphereShape, hfShape, justTest);
  //       }

  //       if (justTest && intersecting) {
  //         return true;
  //       }

  //       let numContacts = result.length - numContactsBefore;

  //       if (numContacts > 2) {
  //         return;
  //       }
  //       /*
  //       // Skip all but 1
  //       for (let k = 0; k < numContacts - 1; k++) {
  //           result.pop();
  //       }
  //       */
  //     }
  //   }
  // }
}

// let numWarnings = 0;
// const maxWarnings = 10;
// function warn(msg: any) {
//   if (numWarnings > maxWarnings) {
//     return;
//   }
//   numWarnings++;
//   console.warn(msg);
// }
