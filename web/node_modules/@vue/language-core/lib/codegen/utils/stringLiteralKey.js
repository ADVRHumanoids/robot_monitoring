"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.generateStringLiteralKey = generateStringLiteralKey;
const boundary_1 = require("./boundary");
function* generateStringLiteralKey(code, offset, info) {
    if (offset === undefined || !info) {
        yield `'${code}'`;
    }
    else {
        const boundary = yield* boundary_1.Boundary.start('template', offset, info);
        yield `'`;
        yield [code, 'template', offset, boundary.features];
        yield `'`;
        yield boundary.end(offset + code.length);
    }
}
//# sourceMappingURL=stringLiteralKey.js.map