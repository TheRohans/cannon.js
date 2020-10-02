var Material = (function () {
    function Material(options) {
        if (options === void 0) { options = {}; }
        options = Object.assign({
            name: 'default',
            friction: 0.3,
            restitution: 0.3
        }, options);
        this.name = options.name;
        this.id = Material.idCounter++;
        this.friction = options.friction;
        this.restitution = options.restitution;
    }
    Material.idCounter = 0;
    return Material;
}());
export { Material };
