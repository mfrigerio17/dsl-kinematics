package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Var
import iit.dsl.generator.Common
import iit.dsl.kinDsl.ParameterLiteral

/**
 * Common facilities to deal with parameters in the robot model.
 *
 * The Matlab expressions returned by methods of this class are simply references
 * to fields of a structure, or numerical constants.
 */
class Parameters
{
    /**
     * The default name of the struct whose fields are the length parameters of the robot
     */
    def public static lengthsParamsStruct() {
        return iit::dsl::generator::common::Parameters::lengthsGroupName
    }
    /**
     * The default name of the struct whose fields are the angular parameters of the robot
     */
    def public static anglesParamsStruct() {
        return iit::dsl::generator::common::Parameters::anglesGroupName
    }
    /**
     * A default name of the struct whose fields are the inertia parameters of the robot
     */
    def public static inertiaParamsStruct() {
        return "inertiaparams";
    }

    /**
     * @name Value accessors.
     *
     * These functions return the expression that would evaluate to the value of
     * the given robot property. Such a property can either be a parameter or a
     * constant value.
     *
     * \return in the case of a parameter, the function returns an expression
     *   referencing a field of a structure; in the case of a constant, the
     *   value itself (as text) is returned.
     *
     * In the case of a parameter, the default name of the struct will be used,
     * unless the name is itself an argument of the method.
     */
    ///@{
    /** \param robotProperty a length parameter of the geometry of the robot  */
    def public static value_length(Var robotProperty) {
        return value(robotProperty, lengthsParamsStruct)
    }
    /** \param robotProperty an angular parameter of the geometry of the robot */
    def public static value_angle(Var robotProperty) {
        return value(robotProperty, anglesParamsStruct)
    }
    /** \param robotProperty a parameter of the inertia properties of the robot */
    def public static value_inertia(Var robotProperty) {
        return value(robotProperty, inertiaParamsStruct)
    }
    /** \param robotProperty a numerical property of the robot
     ** \param struct the name of the struct used in the returned expression */
    def public static value(Var robotProperty, CharSequence struct)
    {
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
    ///@}
}