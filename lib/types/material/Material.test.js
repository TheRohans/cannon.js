import { Material } from './Material';
describe('Material', function () {
    it('should construct default', function () {
        var cm = new Material();
        expect(cm).not.toBeUndefined();
        expect(cm.friction).toEqual(.3);
        expect(cm.restitution).toEqual(.3);
        expect(cm.name).toEqual('default');
    });
    it('should take options', function () {
        var cm = new Material({
            name: 'My Material',
            friction: 1,
            restitution: 0,
        });
        expect(cm).not.toBeUndefined();
        expect(cm.friction).toEqual(1);
        expect(cm.restitution).toEqual(0);
        expect(cm.name).toEqual('My Material');
    });
});
