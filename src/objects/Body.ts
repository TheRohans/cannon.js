import { EventTarget } from '../utils/EventTarget';
import { Vec3 } from '../math/Vec3';
import { Material } from '../material/Material';
import { Quaternion } from '../math/Quaternion';
import { Shape } from '../shapes/Shape';
import { Mat3 } from '../math/Mat3';
import { AABB } from '../collision/AABB';
import { Box } from '../shapes/Box';
import { World } from '../world/World';

export class BodyOptions {
  mass: number;
  position: Vec3;
  quaternion: Quaternion;
  type: 1|2|4;
  material: Material;
}

/**
 * Base class for all body types.
 * @class Body
 * @constructor
 * @extends EventTarget
 * @param {object} [options]
 * @param {Vec3} [options.position]
 * @param {Vec3} [options.velocity]
 * @param {Vec3} [options.angularVelocity]
 * @param {Quaternion} [options.quaternion]
 * @param {number} [options.mass]
 * @param {Material} [options.material]
 * @param {number} [options.type]
 * @param {number} [options.linearDamping=0.01]
 * @param {number} [options.angularDamping=0.01]
 * @param {boolean} [options.allowSleep=true]
 * @param {number} [options.sleepSpeedLimit=0.1]
 * @param {number} [options.sleepTimeLimit=1]
 * @param {number} [options.collisionFilterGroup=1]
 * @param {number} [options.collisionFilterMask=-1]
 * @param {boolean} [options.fixedRotation=false]
 * @param {Vec3} [options.linearFactor]
 * @param {Vec3} [options.angularFactor]
 * @param {Shape} [options.shape]
 * @example
 *     var body = new Body({
 *         mass: 1
 *     });
 *     var shape = new Sphere(1);
 *     body.addShape(shape);
 *     world.addBody(body);
 */
export class Body extends EventTarget {
  static idCounter = 0;

  /**
   * Dispatched after two bodies collide. This event is dispatched on each
   * of the two bodies involved in the collision.
   * @event collide
   * @param {Body} body The body that was involved in the collision.
   * @param {ContactEquation} contact The details of the collision.
   */
  static COLLIDE_EVENT_NAME = 'collide';

  /**
   * A dynamic body is fully simulated. Can be moved manually by the user,
   * but normally they move according to forces. A dynamic body can collide
   * with all body types. A dynamic body always has finite, non-zero mass.
   * @static
   * @property DYNAMIC
   * @type {Number}
   */
  static DYNAMIC = 1;

  /**
   * A static body does not move during simulation and behaves as if it has
   * infinite mass. Static bodies can be moved manually by setting the position
   * of the body. The velocity of a static body is always zero. Static bodies do
   * not collide with other static or kinematic bodies.
   * @static
   * @property STATIC
   * @type {Number}
   */
  static STATIC = 2;

  /**
   * A kinematic body moves under simulation according to its velocity. They do not
   * respond to forces. They can be moved manually, but normally a kinematic body is
   * moved by setting its velocity. A kinematic body behaves as if it has infinite
   * mass. Kinematic bodies do not collide with other static or kinematic bodies.
   * @static
   * @property KINEMATIC
   * @type {Number}
   */
  static KINEMATIC = 4;

  /**
   * @static
   * @property AWAKE
   * @type {number}
   */
  static AWAKE = 0;

  /**
   * @static
   * @property SLEEPY
   * @type {number}
   */
  static SLEEPY = 1;

  /**
   * @static
   * @property SLEEPING
   * @type {number}
   */
  static SLEEPING = 2;

  goid: string;
  // The original shape used to create this body. Some shapes are compound
  // shapes this will be the original shape from the caller.
  origShape: number;
  id: number;
  // This is used by World for some reason...
  index: number;
  /**
   * Reference to the world the body is living in
   * @property world
   * @type {World}
   */
  world: World;

  /**
   * Callback function that is used BEFORE stepping the system.
   * Use it to apply forces, for example. Inside the function,
   * "this" will refer to this Body object.
   * @property preStep
   * @type {Function}
   * @deprecated Use World events instead
   */
  preStep: Function;
  /**
   * Callback function that is used AFTER stepping the system. Inside
   * the function, "this" will refer to this Body object.
   * @property postStep
   * @type {Function}
   * @deprecated Use World events instead
   */
  postStep: Function;

  vlambda: Vec3;

  collisionFilterGroup: number;
  collisionFilterMask: number;

  /**
   * Whether to produce contact forces when in contact with other
   * bodies. Note that contacts will be generated, but they will be disabled.
   * @property {Number} collisionResponse
   */
  collisionResponse: boolean;

  /**
   * World space position of the body.
   * @property position
   * @type {Vec3}
   */
  position: Vec3;

  previousPosition: Vec3;
  interpolatedPosition: Vec3;
  initPosition: Vec3;
  /**
   * World space velocity of the body.
   * @property velocity
   * @type {Vec3}
   */
  velocity: Vec3;
  initVelocity: Vec3;
  force: Vec3;

  mass: number;
  invMass: number;

  material: Material;

  linearDamping: number;

  type: number; // 1|2|4;

  allowSleep: boolean;

  sleepState: number;

  sleepSpeedLimit: number;
  sleepTimeLimit: number;

  timeLastSleepy: number;

  _wakeUpAfterNarrowphase: boolean;

  torque: Vec3;
  quaternion: Quaternion;
  initQuaternion: Quaternion;
  previousQuaternion: Quaternion;
  interpolatedQuaternion: Quaternion;

  /**
   * Angular velocity of the body, in world space. Think of the
   * angular velocity as a vector, which the body rotates around.
   * The length of this vector determines how fast (in radians per
   * second) the body rotates.
   * @property angularVelocity
   * @type {Vec3}
   */
  angularVelocity: Vec3;
  initAngularVelocity: Vec3;

  /**
   * The shapes that make up this body. They start at 0,0,0
   * in model space but can be modifed by shapeOffsets
   */
  shapes: Shape[];
  /**
   * Position of a shape (in model space). Array index should
   * match the listing in shapes[]
   */
  shapeOffsets: Vec3[];
  /**
   * Rotations of a shape (in model space). Array index should
   * match the listing in shapes[]
   */
  shapeOrientations: Quaternion[];

  inertia: Vec3;
  invInertia: Vec3;

  invInertiaWorld: Mat3;

  invMassSolve: number;
  invInertiaSolve: Vec3;
  invInertiaWorldSolve: Mat3;

  /**
   * Set to true if you don't want the body to rotate. Make sure to run
   * .updateMassProperties() after changing this.
   * @property {Boolean} fixedRotation
   * @default false
   */
  fixedRotation: boolean;

  angularDamping: number;

  /**
   * Use this property to limit the motion along any world axis.
   * (1,1,1) will allow motion along all axes while (0,0,0) allows none.
   * @property {Vec3} linearFactor
   */
  linearFactor: Vec3;

  /**
   * Use this property to limit the rotational motion along any
   * world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
   * @property {Vec3} angularFactor
   */
  angularFactor: Vec3;

  aabb: AABB;
  aabbNeedsUpdate: boolean;
  boundingRadius: number;

  wlambda: Vec3;

  constructor(options?: BodyOptions | any) {
    super();
    options = options || {};

    // EventTarget.apply(this);
    this.goid = '';
    this.id = Body.idCounter++;

    this.world = null;
    this.preStep = null;
    this.postStep = null;

    this.vlambda = new Vec3();

    /**
     * @property {Number} collisionFilterGroup
     */
    this.collisionFilterGroup = typeof (options.collisionFilterGroup) === 'number' ? options.collisionFilterGroup : 1;

    /**
     * @property {Number} collisionFilterMask
     */
    this.collisionFilterMask = typeof (options.collisionFilterMask) === 'number' ? options.collisionFilterMask : -1;

    this.collisionResponse = true;

    this.position = new Vec3();

    /**
     * @property {Vec3} previousPosition
     */
    this.previousPosition = new Vec3();

    /**
     * Interpolated position of the body.
     * @property {Vec3} interpolatedPosition
     */
    this.interpolatedPosition = new Vec3();

    /**
     * Initial position of the body
     * @property initPosition
     * @type {Vec3}
     */
    this.initPosition = new Vec3();

    if (options.position) {
      this.position.copy(options.position);
      this.previousPosition.copy(options.position);
      this.interpolatedPosition.copy(options.position);
      this.initPosition.copy(options.position);
    }

    this.velocity = new Vec3();

    if (options.velocity) {
      this.velocity.copy(options.velocity);
    }

    /**
     * @property initVelocity
     * @type {Vec3}
     */
    this.initVelocity = new Vec3();

    /**
     * Linear force on the body in world space.
     * @property force
     * @type {Vec3}
     */
    this.force = new Vec3();

    const mass = options.mass ? options.mass : 0;

    /**
     * @property mass
     * @type {Number}
     * @default 0
     */
    this.mass = mass;

    /**
     * @property invMass
     * @type {Number}
     */
    this.invMass = mass > 0 ? 1.0 / mass : 0;

    /**
     * @property material
     * @type {Material}
     */
    this.material = options.material || null;

    /**
     * @property linearDamping
     * @type {Number}
     */
    this.linearDamping = typeof (options.linearDamping) === 'number' ? options.linearDamping : 0.01;

    /**
     * One of: Body.DYNAMIC, Body.STATIC and Body.KINEMATIC.
     * @property type
     * @type {Number}
     */
    this.type = (mass <= 0.0 ? Body.STATIC : Body.DYNAMIC);
    if (typeof (options.type) === typeof (Body.STATIC)) {
      this.type = options.type;
    }

    /**
     * If true, the body will automatically fall to sleep.
     * @property allowSleep
     * @type {Boolean}
     * @default true
     */
    this.allowSleep = (options.allowSleep !== 'undefined') ? options.allowSleep : true;

    /**
     * Current sleep state.
     * @property sleepState
     * @type {Number}
     */
    this.sleepState = Body.AWAKE;

    /**
     * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
     * @property sleepSpeedLimit
     * @type {Number}
     * @default 0.1
     */
    this.sleepSpeedLimit = options.sleepSpeedLimit ? options.sleepSpeedLimit : 0.1;

    /**
     * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
     * @property sleepTimeLimit
     * @type {Number}
     * @default 1
     */
    this.sleepTimeLimit = options.sleepTimeLimit ? options.sleepTimeLimit : 1;

    this.timeLastSleepy = 0;

    this._wakeUpAfterNarrowphase = false;

    /**
     * World space rotational force on the body, around center of mass.
     * @property {Vec3} torque
     */
    this.torque = new Vec3();

    /**
     * World space orientation of the body.
     * @property quaternion
     * @type {Quaternion}
     */
    this.quaternion = new Quaternion();

    /**
     * @property initQuaternion
     * @type {Quaternion}
     */
    this.initQuaternion = new Quaternion();

    /**
     * @property {Quaternion} previousQuaternion
     */
    this.previousQuaternion = new Quaternion();

    /**
     * Interpolated orientation of the body.
     * @property {Quaternion} interpolatedQuaternion
     */
    this.interpolatedQuaternion = new Quaternion();

    if (options.quaternion) {
      this.quaternion.copy(options.quaternion);
      this.initQuaternion.copy(options.quaternion);
      this.previousQuaternion.copy(options.quaternion);
      this.interpolatedQuaternion.copy(options.quaternion);
    }

    this.angularVelocity = new Vec3();

    if (options.angularVelocity) {
      this.angularVelocity.copy(options.angularVelocity);
    }

    /**
     * @property initAngularVelocity
     * @type {Vec3}
     */
    this.initAngularVelocity = new Vec3();

    /**
     * @property shapes
     * @type {array}
     */
    this.shapes = [];

    /**
     * Position of each Shape in the body, given in local Body space.
     * @property shapeOffsets
     * @type {array}
     */
    this.shapeOffsets = [];

    /**
     * Orientation of each Shape, given in local Body space.
     * @property shapeOrientations
     * @type {array}
     */
    this.shapeOrientations = [];

    /**
     * @property inertia
     * @type {Vec3}
     */
    this.inertia = new Vec3();

    /**
     * @property {Vec3} invInertia
     */
    this.invInertia = new Vec3();

    /**
     * @property {Mat3} invInertiaWorld
     */
    this.invInertiaWorld = new Mat3();

    this.invMassSolve = 0;

    /**
     * @property {Vec3} invInertiaSolve
     */
    this.invInertiaSolve = new Vec3();

    /**
     * @property {Mat3} invInertiaWorldSolve
     */
    this.invInertiaWorldSolve = new Mat3();

    this.fixedRotation = (options.fixedRotation !== undefined) ? options.fixedRotation : false;

    /**
     * @property {Number} angularDamping
     */
    this.angularDamping = (options.angularDamping !== undefined) ? options.angularDamping : 0.01;


    this.linearFactor = new Vec3(1, 1, 1);
    if (options.linearFactor) {
      this.linearFactor.copy(options.linearFactor);
    }

    this.angularFactor = new Vec3(1, 1, 1);
    if (options.angularFactor) {
      this.angularFactor.copy(options.angularFactor);
    }

    /**
     * World space bounding box of the body and its shapes.
     * @property aabb
     * @type {AABB}
     */
    this.aabb = new AABB();

    /**
     * Indicates if the AABB needs to be updated before use.
     * @property aabbNeedsUpdate
     * @type {Boolean}
     */
    this.aabbNeedsUpdate = true;

    /**
     * Total bounding radius of the Body including its shapes, relative to body.position.
     * @property boundingRadius
     * @type {Number}
     */
    this.boundingRadius = 0;

    this.wlambda = new Vec3();

    if (options.shape) {
      this.addShape(options.shape);
    }

    this.updateMassProperties();
  }

  /**
   * Dispatched after a sleeping body has woken up.
   * @event wakeup
   */
  static wakeupEvent = {
    type: 'wakeup'
  };

  /**
   * Dispatched after a body has gone in to the sleepy state.
   * @event sleepy
   */
  static sleepyEvent = {
    type: 'sleepy'
  };

  /**
   * Dispatched after a body has fallen asleep.
   * @event sleep
   */
  static sleepEvent = {
    type: 'sleep'
  };

  /**
   * Wake the body up.
   * @method wakeUp
   */
  wakeUp() {
    const s = this.sleepState;
    this.sleepState = Body.AWAKE;
    this._wakeUpAfterNarrowphase = false;
    if (s === Body.SLEEPING) {
      this.dispatchEvent(Body.wakeupEvent);
    }
  }

  /**
   * Force body sleep
   * @method sleep
   */
  sleep() {
    this.sleepState = Body.SLEEPING;
    this.velocity.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this._wakeUpAfterNarrowphase = false;
  }

  /**
   * Called every timestep to update internal sleep timer and change sleep state if needed.
   * @method sleepTick
   * @param {Number} time The world time in seconds
   */
  sleepTick(time: number) {
    if (this.allowSleep) {
      const sleepState = this.sleepState;
      const speedSquared = this.velocity.norm2() + this.angularVelocity.norm2();
      const speedLimitSquared = Math.pow(this.sleepSpeedLimit, 2);
      if (sleepState === Body.AWAKE && speedSquared < speedLimitSquared) {
        this.sleepState = Body.SLEEPY; // Sleepy
        this.timeLastSleepy = time;
        this.dispatchEvent(Body.sleepyEvent);
      } else if (sleepState === Body.SLEEPY && speedSquared > speedLimitSquared) {
        this.wakeUp(); // Wake up
      } else if (sleepState === Body.SLEEPY && (time - this.timeLastSleepy) > this.sleepTimeLimit) {
        this.sleep(); // Sleeping
        this.dispatchEvent(Body.sleepEvent);
      }
    }
  }

  /**
   * If the body is sleeping, it should be immovable / have infinite mass during
   * solve. We solve it by having a separate "solve mass".
   * @method updateSolveMassProperties
   */
  updateSolveMassProperties() {
    if (this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC) {
      this.invMassSolve = 0;
      this.invInertiaSolve.setZero();
      this.invInertiaWorldSolve.setZero();
    } else {
      this.invMassSolve = this.invMass;
      this.invInertiaSolve.copy(this.invInertia);
      this.invInertiaWorldSolve.copy(this.invInertiaWorld);
    }
  }

  /**
   * Convert a world point to local body frame.
   * @method pointToLocalFrame
   * @param  {Vec3} worldPoint
   * @param  {Vec3} result
   * @return {Vec3}
   */
  pointToLocalFrame(worldPoint: Vec3, result?: Vec3): Vec3 {
    result = result || new Vec3();
    worldPoint.vsub(this.position, result);
    this.quaternion.conjugate().vmult(result, result);
    return result;
  }

  /**
   * Convert a world vector to local body frame.
   * @method vectorToLocalFrame
   * @param  {Vec3} worldPoint
   * @param  {Vec3} result
   * @return {Vec3}
   */
  vectorToLocalFrame(worldVector: Vec3, result?: Vec3): Vec3 {
    result = result || new Vec3();
    this.quaternion.conjugate().vmult(worldVector, result);
    return result;
  }

  /**
   * Convert a local body point to world frame.
   * @method pointToWorldFrame
   * @param  {Vec3} localPoint
   * @param  {Vec3} result
   * @return {Vec3}
   */
  pointToWorldFrame(localPoint: Vec3, result?: Vec3): Vec3 {
    result = result || new Vec3();
    this.quaternion.vmult(localPoint, result);
    result.vadd(this.position, result);
    return result;
  }

  /**
   * Convert a local body point to world frame.
   * @method vectorToWorldFrame
   * @param  {Vec3} localVector
   * @param  {Vec3} result
   * @return {Vec3}
   */
  vectorToWorldFrame(localVector: Vec3, result?: Vec3): Vec3 {
    result = result || new Vec3();
    this.quaternion.vmult(localVector, result);
    return result;
  }

  private tmpVec = new Vec3();
  private tmpQuat = new Quaternion();
  /**
   * Add a shape to the body with a local offset and orientation.
   * @method addShape
   * @param {Shape} shape
   * @param {Vec3} [offset]
   * @param {Quaternion} [orientation]
   * @return {Body} The body object, for chainability.
   */
  addShape(shape: Shape, offset?: Vec3, orientation?: Quaternion) {
    this.shapes.push(shape);
    this.shapeOffsets.push(offset || new Vec3());
    this.shapeOrientations.push(orientation || new Quaternion());
    this.recalculateShapes();
    shape.body = this;

    return this;
  }

  /**
   * If you change the shape(s) or position of the shapes this body is
   * using you'll need to call this to recalculate the mass and bounding area
   */
  recalculateShapes() {
    this.updateMassProperties();
    this.updateBoundingRadius();
    this.aabbNeedsUpdate = true;
  }

  /**
   * Update the bounding radius of the body. Should be done if any of the shapes are changed.
   * @method updateBoundingRadius
   */
  updateBoundingRadius() {
    const shapes = this.shapes,
      shapeOffsets = this.shapeOffsets,
      N = shapes.length;
     let radius = 0;

    for (let i = 0; i !== N; i++) {
      const shape = shapes[i];
      shape.updateBoundingSphereRadius();
      const offset = shapeOffsets[i].norm(),
        r = shape.boundingSphereRadius;
      if (offset + r > radius) {
        radius = offset + r;
      }
    }

    this.boundingRadius = radius;
  }

  private computeAABB_shapeAABB = new AABB();

  /**
   * Updates the .aabb
   * @method computeAABB
   * @todo rename to updateAABB()
   */
  computeAABB() {
    const shapes = this.shapes,
      shapeOffsets = this.shapeOffsets,
      shapeOrientations = this.shapeOrientations,
      N = shapes.length,
      offset = this.tmpVec,
      orientation = this.tmpQuat,
      bodyQuat = this.quaternion,
      aabb = this.aabb,
      shapeAABB = this.computeAABB_shapeAABB;

    let i = N;
    while (i--) {
      const shape = shapes[i];

      // Get shape world position
      bodyQuat.vmult(shapeOffsets[i], offset);
      offset.vadd(this.position, offset);

      // Get shape world quaternion
      shapeOrientations[i].mult(bodyQuat, orientation);

      // Get shape AABB
      shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

      if (i === 0) {
        aabb.copy(shapeAABB);
      } else {
        aabb.extend(shapeAABB);
      }
    }

    this.aabbNeedsUpdate = false;
  }

  private uiw_m1 = new Mat3();
  private uiw_m2 = new Mat3();
  private uiw_m3 = new Mat3();

  /**
   * Update .inertiaWorld and .invInertiaWorld
   * @method updateInertiaWorld
   */
  updateInertiaWorld(force: boolean = false) {
    const I = this.invInertia;
    if (I.x === I.y && I.y === I.z && !force) {
      // If inertia M = s*I, where I is identity and s a scalar, then
      //    R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
      // where R is the rotation matrix.
      // In other words, we don't have to transform the inertia if all
      // inertia diagonal entries are equal.
    } else {
      const m1 = this.uiw_m1;
      const m2 = this.uiw_m2;
      const m3 = this.uiw_m3;
      m1.setRotationFromQuaternion(this.quaternion);
      m1.transpose(m2);
      m1.scale(I, m1);
      m1.mmult(m2, this.invInertiaWorld);
    }
  }

  private Body_applyForce_r = new Vec3();
  private Body_applyForce_rotForce = new Vec3();
  /**
   * Apply force to a world point. This could for example be a point on
   * the Body surface. Applying force this way will add to Body.force and Body.torque.
   * @method applyForce
   * @param  {Vec3} force The amount of force to add.
   * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
   */
  applyForce(force: Vec3, relativePoint: Vec3) {
    if (this.type !== Body.DYNAMIC) {
      return;
    }

    // Compute produced rotational force
    const rotForce = this.Body_applyForce_rotForce;
    relativePoint.cross(force, rotForce);

    // Add linear force
    this.force.vadd(force, this.force);

    // Add rotational force
    this.torque.vadd(rotForce, this.torque);
  }

  private Body_applyLocalForce_worldForce = new Vec3();
  private Body_applyLocalForce_relativePointWorld = new Vec3();
  /**
   * Apply force to a local point in the body.
   * @method applyLocalForce
   * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
   * @param  {Vec3} localPoint A local point in the body to apply the force on.
   */
  applyLocalForce(localForce: Vec3, localPoint: Vec3) {
    if (this.type !== Body.DYNAMIC) {
      return;
    }

    const worldForce = this.Body_applyLocalForce_worldForce;
    const relativePointWorld = this.Body_applyLocalForce_relativePointWorld;

    // Transform the force vector to world space
    this.vectorToWorldFrame(localForce, worldForce);
    this.vectorToWorldFrame(localPoint, relativePointWorld);

    this.applyForce(worldForce, relativePointWorld);
  }

  private Body_applyImpulse_r = new Vec3();
  private Body_applyImpulse_velo = new Vec3();
  private Body_applyImpulse_rotVelo = new Vec3();
  /**
   * Apply impulse to a world point. This could for example be a point on the Body
   * surface. An impulse is a force added to a body during a short period of time
   * (impulse = force * time). Impulses will be added to Body.velocity and
   * Body.angularVelocity.
   * @method applyImpulse
   * @param  {Vec3} impulse The amount of impulse to add.
   * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
   */
  applyImpulse(impulse: Vec3, relativePoint: Vec3) {
    if (this.type !== Body.DYNAMIC) {
      return;
    }

    // Compute point position relative to the body center
    const r = relativePoint;

    // Compute produced central impulse velocity
    const velo = this.Body_applyImpulse_velo;
    velo.copy(impulse);
    velo.mult(this.invMass, velo);

    // Add linear impulse
    this.velocity.vadd(velo, this.velocity);

    // Compute produced rotational impulse velocity
    const rotVelo = this.Body_applyImpulse_rotVelo;
    r.cross(impulse, rotVelo);

    /*
    rotVelo.x *= this.invInertia.x;
    rotVelo.y *= this.invInertia.y;
    rotVelo.z *= this.invInertia.z;
    */
    this.invInertiaWorld.vmult(rotVelo, rotVelo);

    // Add rotational Impulse
    this.angularVelocity.vadd(rotVelo, this.angularVelocity);
  }

  private Body_applyLocalImpulse_worldImpulse = new Vec3();
  private Body_applyLocalImpulse_relativePoint = new Vec3();
  /**
   * Apply locally-defined impulse to a local point in the body.
   * @method applyLocalImpulse
   * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
   * @param  {Vec3} localPoint A local point in the body to apply the force on.
   */
  applyLocalImpulse(localImpulse: Vec3, localPoint: Vec3) {
    if (this.type !== Body.DYNAMIC) {
      return;
    }
    const worldImpulse = this.Body_applyLocalImpulse_worldImpulse;
    const relativePointWorld = this.Body_applyLocalImpulse_relativePoint;

    // Transform the force vector to world space
    this.vectorToWorldFrame(localImpulse, worldImpulse);
    this.vectorToWorldFrame(localPoint, relativePointWorld);

    this.applyImpulse(worldImpulse, relativePointWorld);
  }

  private Body_updateMassProperties_halfExtents = new Vec3();
  /**
   * Should be called whenever you change the body shape or mass.
   * @method updateMassProperties
   */
  updateMassProperties() {
    const halfExtents = this.Body_updateMassProperties_halfExtents;

    this.invMass = this.mass > 0 ? 1.0 / this.mass : 0;
    const I = this.inertia;
    const fixed = this.fixedRotation;

    // Approximate with AABB box
    this.computeAABB();
    halfExtents.set(
      (this.aabb.upperBound.x - this.aabb.lowerBound.x) / 2,
      (this.aabb.upperBound.y - this.aabb.lowerBound.y) / 2,
      (this.aabb.upperBound.z - this.aabb.lowerBound.z) / 2
    );
    Box.calculateInertia(halfExtents, this.mass, I);

    this.invInertia.set(
      I.x > 0 && !fixed ? 1.0 / I.x : 0,
      I.y > 0 && !fixed ? 1.0 / I.y : 0,
      I.z > 0 && !fixed ? 1.0 / I.z : 0
    );
    this.updateInertiaWorld(true);
  }

  /**
   * Get world velocity of a point in the body.
   * @method getVelocityAtWorldPoint
   * @param  {Vec3} worldPoint
   * @param  {Vec3} result
   * @return {Vec3} The result vector.
   */
  getVelocityAtWorldPoint(worldPoint: Vec3, result: Vec3) {
    const r = new Vec3();
    worldPoint.vsub(this.position, r);
    this.angularVelocity.cross(r, result);
    this.velocity.vadd(result, result);
    return result;
  }

  private invI_tau_dt = new Vec3();
  private w = new Quaternion();
  private wq = new Quaternion();
  /**
   * Move the body forward in time.
   * @param {number} dt Time step
   * @param {boolean} quatNormalize Set to true to normalize the body quaternion
   * @param {boolean} quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization
   */
  integrate(dt: number, quatNormalize: boolean, quatNormalizeFast: boolean) {

    // Save previous position
    this.previousPosition.copy(this.position);
    this.previousQuaternion.copy(this.quaternion);

     // Only for dynamic bodies
    if (!(this.type === Body.DYNAMIC || this.type === Body.KINEMATIC) || this.sleepState === Body.SLEEPING) {
      return;
    }

    const velo = this.velocity,
      angularVelo = this.angularVelocity,
      pos = this.position,
      force = this.force,
      torque = this.torque,
      quat = this.quaternion,
      invMass = this.invMass,
      invInertia = this.invInertiaWorld,
      linearFactor = this.linearFactor;

    const iMdt = invMass * dt;
    velo.x += force.x * iMdt * linearFactor.x;
    velo.y += force.y * iMdt * linearFactor.y;
    velo.z += force.z * iMdt * linearFactor.z;

    const e = invInertia.elements;
    const angularFactor = this.angularFactor;
    const tx = torque.x * angularFactor.x;
    const ty = torque.y * angularFactor.y;
    const tz = torque.z * angularFactor.z;
    angularVelo.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
    angularVelo.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
    angularVelo.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);

    // Use new velocity  - leap frog
    pos.x += velo.x * dt;
    pos.y += velo.y * dt;
    pos.z += velo.z * dt;

    quat.integrate(this.angularVelocity, dt, this.angularFactor, quat);

    if (quatNormalize) {
      if (quatNormalizeFast) {
        quat.normalizeFast();
      } else {
        quat.normalize();
      }
    }

    this.aabbNeedsUpdate = true;

    // Update world inertia
    this.updateInertiaWorld();
  }
}
