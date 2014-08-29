package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Var
import iit.dsl.generator.Common
import iit.dsl.kinDsl.ParameterLiteral

/**
 * Common facilities to deal with parameters in the robot model.
 *
 * The Matlab code fragments returned by methods of this class are for scripts that
 * work with workspace variables.
 */
class Parameters {
    /**
     * The name of the struct whose fields are the length parameters of the robot
     */
    def public static lengthsParamsStruct() {
        return iit::dsl::generator::common::Parameters::lengthsGroupName
    }
    /**
     * The name of the struct whose fields are the angular parameters of the robot
     */
    def public static anglesParamsStruct() {
        return iit::dsl::generator::common::Parameters::anglesGroupName
    }
    /**
     * The expression that evaluates to the value of the given property.
     * \param robotProperty a length parameter of the geometry of the robot
     */
    def public static value_length(Var robotProperty) {
        return value(robotProperty, lengthsParamsStruct)
    }
    /**
     * The expression that evaluates to the value of the given property.
     * \param robotProperty an angular parameter of the geometry of the robot
     */
    def public static value_angle(Var robotProperty) {
        return value(robotProperty, anglesParamsStruct)
    }

    def private static value(Var robotProperty, CharSequence struct) {
        if(iit::dsl::generator::common::Parameters::isParameter(robotProperty))
        {
            val ParameterLiteral p = iit::dsl::generator::common::Parameters::asParameter(robotProperty)
            var String sign = ""
            if(p.minus) sign = '-'
            return "(" + sign + struct + "." + p.varname + ")"
        }
        // not a parameter
        return Common::getInstance().asFloat(robotProperty)
    }
}