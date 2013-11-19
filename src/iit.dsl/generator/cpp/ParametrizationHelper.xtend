package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.ParameterLiteral
import iit.dsl.generator.common.Parameters

import java.util.List
import java.util.ArrayList
import java.util.Map
import java.util.HashMap

/**
 * Utilities for C++ generation, related to the possible parameters in the
 * specification of the kinematics tree.
 *
 * Instances of this class must be constructed with an instance of a Robot.
 */
class ParametrizationHelper {


    new(Robot rob) {
        robot = rob
        transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        lengthsGroup = iit::dsl::generator::common::Parameters::getLengthParamsGroup(transformsModel)
        anglesGroup  = iit::dsl::generator::common::Parameters::getAngleParamsGroup(transformsModel)
        Parameters::getJointsParameters(robot, lengths, angles)
        for( l : lengths) {
            if( ! lengthParams.containsKey(l.varname) ) {
                lengthParams.put(l.varname, l)
            }
        }
        for( l : angles) {
            if( ! angleParams.containsKey(l.varname) ) {
                angleParams.put(l.varname, l)
            }
        }
    }

    /**
     * Returns the struct-type containing the parameters, to be used
     * to declare a variable of such type.
     * The returned value is qualified with the namespace containing the
     * parameters-related stuff. The caller might need to prepend the returned
     * type with the qualifier for the robot.
     */
    ///@{
    def public lengthsStructVarDeclaration()
    '''«IF lengthsGroup!=null»«ns»::«iit::dsl::coord::generator::cpp::Parameters::structName(lengthsGroup)»«ENDIF»'''

    def public anglesStructVarDeclaration()
    '''«IF anglesGroup!=null»«ns»::«iit::dsl::coord::generator::cpp::Parameters::structName(anglesGroup)»«ENDIF»'''
    ///@}

    /**
     * The field name that corresponds to the given parameter.
     * This is the name of the field of the struct that contains the parameters.
     * @return the string to be appended to a string in the form \c "<variable>."
     *    such that the resulting string is a valid piece of code that accesses
     *    the value of the given parameter
     * @throw a RuntimeException if the given parameter does not belong to the
     *    robot this instance was created with
     */
    def public structField(ParameterLiteral p) {
        if(lengthParams.containsKey(p.varname)) {
            return (Parameters::convertLengthParam(transformsModel, p)).name
        }
        if(angleParams.containsKey(p.varname)) {
            return (Parameters::convertAngleParam(transformsModel, p)).name
        }
        throw new RuntimeException("The parameter literal " + p.varname +
            " does not seem to belong to robot " + robot.name)
    }

    /**
     * The name of the getter function for the given parameter
     * @return the string that correspond to the name of the getter method for
     *   the given parameter
     * @throw a RuntimeException if the given parameter does not belong to the
     *    robot this instance was created with
     */
    def public getterFunction(ParameterLiteral p) {
        if(lengthParams.containsKey(p.varname)) {
            return iit::dsl::coord::generator::cpp::Parameters::getterFunctionName(
                Parameters::convertLengthParam(transformsModel, p))
        }
        if(angleParams.containsKey(p.varname)) {
            return iit::dsl::coord::generator::cpp::Parameters::getterFunctionName(
                Parameters::convertAngleParam(transformsModel, p))
        }
        throw new RuntimeException("The parameter literal " + p.varname +
            " does not seem to belong to robot " + robot.name)
    }

    def public getLengthParams() { return lengths}
    def public getAngleParams()  { return angles}



    private Robot robot
    private iit.dsl.coord.coordTransDsl.Model transformsModel
    private iit.dsl.coord.coordTransDsl.ParametersDeclaration lengthsGroup
    private iit.dsl.coord.coordTransDsl.ParametersDeclaration anglesGroup
    private Map<String, ParameterLiteral> lengthParams = new HashMap<String, ParameterLiteral>
    private Map<String, ParameterLiteral> angleParams  = new HashMap<String, ParameterLiteral>
    private List<ParameterLiteral> lengths = new ArrayList<ParameterLiteral>
    private List<ParameterLiteral> angles  = new ArrayList<ParameterLiteral>

    private static String ns = iit::dsl::coord::generator::cpp::Parameters::namespaceName

/*
    def public static structVarDeclaration(ParameterLiteral p)
    '''«iit::dsl::coord::generator::cpp::Parameters::structName(g)»'''
    def public static structVarDeclaration(ParametersDeclaration g)
    '''«iit::dsl::coord::generator::cpp::Parameters::namespaceName»::«iit::dsl::coord::generator::cpp::Parameters::structName(g)»'''
*/
}