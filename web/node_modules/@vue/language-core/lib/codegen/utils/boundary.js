"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.Boundary = void 0;
class Boundary {
    constructor(source, features) {
        this.source = source;
        this.features = features;
    }
    static *start(source, offset, features) {
        features = { ...features, __combineToken: Symbol() };
        yield [``, source, offset, features];
        return new Boundary(source, features);
    }
    end(offset) {
        return [``, this.source, offset, this.features];
    }
}
exports.Boundary = Boundary;
//# sourceMappingURL=boundary.js.map