import { Sphere } from './Sphere';
describe('Sphere', function () {
    it('should throwOnWrongRadius', function () {
        var s = new Sphere(1);
        expect(s).not.toBeUndefined();
        var s1 = new Sphere(0);
        expect(s1).not.toBeUndefined();
        expect(function () {
            var s2 = new Sphere(-1);
        }).toThrowError('The sphere radius cannot be negative.');
    });
});
