import { OverlapKeeper } from './OverlapKeeper';
describe('OverlapKeeper', function () {
    it('should construct', function () {
        var k = new OverlapKeeper();
        expect(k).not.toBeUndefined();
    });
    it('should set', function () {
        var keeper = new OverlapKeeper();
        keeper.set(1, 2);
        expect(keeper.current).toEqual([keeper.getKey(1, 2)]);
        keeper.set(3, 2);
        expect(keeper.current).toEqual([keeper.getKey(1, 2), keeper.getKey(3, 2)]);
        keeper.set(3, 1);
        expect(keeper.current).toEqual([keeper.getKey(1, 2), keeper.getKey(1, 3), keeper.getKey(3, 2)]);
    });
    it('should getDiff', function () {
        var keeper = new OverlapKeeper();
        keeper.set(1, 2);
        keeper.set(3, 2);
        keeper.set(3, 1);
        keeper.tick();
        keeper.set(1, 2);
        keeper.set(3, 2);
        keeper.set(3, 1);
        var additions = [];
        var removals = [];
        keeper.getDiff(additions, removals);
        expect(additions.length).toEqual(0);
        expect(removals.length).toEqual(0);
        keeper.tick();
        keeper.set(1, 2);
        keeper.getDiff(additions, removals);
        expect(additions.length).toEqual(0);
        expect(removals).toEqual([1, 3, 2, 3]);
        keeper.tick();
        keeper.set(1, 2);
        keeper.set(1, 2);
        additions = [];
        removals = [];
        keeper.getDiff(additions, removals);
        expect(additions.length).toEqual(0);
        expect(removals.length).toEqual(0);
        keeper.set(3, 2);
        keeper.set(3, 1);
        additions = [];
        removals = [];
        keeper.getDiff(additions, removals);
        expect(additions).toEqual([1, 3, 2, 3]);
        keeper.tick();
        keeper.set(4, 2);
        keeper.set(4, 1);
        additions = [];
        removals = [];
        keeper.getDiff(additions, removals);
        expect(additions).toEqual([1, 4, 2, 4]);
        expect(removals).toEqual([1, 2, 1, 3, 2, 3]);
    });
});