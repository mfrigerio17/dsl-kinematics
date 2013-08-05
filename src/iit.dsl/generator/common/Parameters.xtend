package iit.dsl.generator.common

import iit.dsl.kinDsl.Vector3
import iit.dsl.kinDsl.ParameterLiteral
import java.util.ArrayList
import iit.dsl.kinDsl.Var
import iit.dsl.kinDsl.Expr
import iit.dsl.kinDsl.Identifier

/**
 * Utilities related to the parameters (i.e. non-constants) possibly included
 * in the robot description files
 */
class Parameters {
    /**
     * Returns a list of the ParameterLiteral instances referenced by the given
     * vector. The list will be empty if the components of the vector are all
     * constants.
     */
    def static getParameters(Vector3 vec) {
        val list = new ArrayList<ParameterLiteral>()
        if(isParameter(vec.x)) {
            list.add( (vec.x as Expr).identifier as ParameterLiteral)
        }
        if(isParameter(vec.y)) {
            list.add( (vec.y as Expr).identifier as ParameterLiteral)
        }
        if(isParameter(vec.z)) {
            list.add( (vec.z as Expr).identifier as ParameterLiteral)
        }
        return list
    }

    def public static dispatch boolean isParameter(Var x) { false }
    def public static dispatch boolean isParameter(Expr x) { isParameter(x.identifier) }
    def public static dispatch boolean isParameter(Identifier x) { false }
    def public static dispatch boolean isParameter(ParameterLiteral x) { true }
}