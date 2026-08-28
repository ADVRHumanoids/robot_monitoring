import type { Code, VueCodeInformation } from '../../types';
export declare class Boundary {
    source: string;
    features: VueCodeInformation;
    private constructor();
    static start(source: string, offset: number, features: VueCodeInformation): Generator<Code, Boundary>;
    end(offset: number): Code;
}
