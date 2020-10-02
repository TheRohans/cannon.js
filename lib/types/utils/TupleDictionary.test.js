import { TupleDictionary } from './TupleDictionary';
describe('TupleDictionary', function () {
    it('should set', function () {
        var t = new TupleDictionary();
        t.set(1, 2, 'lol');
        expect(t.data['1-2']).toEqual('lol');
        t.set(2, 1, 'lol2');
        expect(t.data['1-2']).toEqual('lol2');
    });
    it('should get', function () {
        var t = new TupleDictionary();
        t.set(1, 2, '1');
        t.set(3, 2, '2');
        expect(t.data['1-2']).toEqual(t.get(1, 2));
        expect(t.data['1-2']).toEqual(t.get(2, 1));
        expect(t.data['2-3']).toEqual(t.get(2, 3));
        expect(t.data['2-3']).toEqual(t.get(3, 2));
    });
    it('should reset', function () {
        var t = new TupleDictionary(), empty = new TupleDictionary();
        t.reset();
        t.set(1, 2, '1');
        t.reset();
        expect(t.data).toEqual(empty.data);
    });
});
