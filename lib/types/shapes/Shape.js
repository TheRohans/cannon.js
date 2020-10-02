var Shape = (function () {
    function Shape(options) {
        options = options || {};
        this.id = Shape.idCounter++;
        this.type = options.type || 0;
        this.boundingSphereRadius = 0;
        this.collisionResponse = options.collisionResponse ? options.collisionResponse : true;
        this.collisionFilterGroup = options.collisionFilterGroup !== undefined ? options.collisionFilterGroup : 1;
        this.collisionFilterMask = options.collisionFilterMask !== undefined ? options.collisionFilterMask : -1;
        this.material = options.material ? options.material : null;
        this.body = null;
    }
    Shape.prototype.updateBoundingSphereRadius = function () {
        throw new Error('computeBoundingSphereRadius() not implemented for shape type ' + this.type);
    };
    Shape.prototype.volume = function () {
        throw new Error('volume() not implemented for shape type ' + this.type);
    };
    Shape.prototype.calculateLocalInertia = function (mass, target) {
        throw new Error('calculateLocalInertia() not implemented for shape type ' + this.type);
    };
    Shape.prototype.calculateWorldAABB = function (pos, quat, min, max) {
        throw new Error('calculateLocalInertia() not implemented for shape type ' + this.type);
    };
    Shape.idCounter = 0;
    Shape.types = {
        SPHERE: 1,
        PLANE: 2,
        BOX: 4,
        COMPOUND: 8,
        CONVEXPOLYHEDRON: 16,
        HEIGHTFIELD: 32,
        PARTICLE: 64,
        CYLINDER: 128,
        TRIMESH: 256
    };
    return Shape;
}());
export { Shape };
