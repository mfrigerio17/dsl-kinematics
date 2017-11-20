package iit.dsl.generator.common

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Common
import iit.dsl.generator.Jacobian
import iit.dsl.transspecs.transSpecs.FramePair
import iit.dsl.coord.coordTransDsl.Frame

import java.util.List
import java.util.ArrayList
import java.util.HashSet

/**
 * Common utilities related to the code generation for Jacobians
 */
class Jacobians {

    /**
     * This function takes a model of the Desired-Transforms-DSL, takes into
     * account only the specification of the desired Jacobians, and returns
     * the corresponding list of iit.dsl.generator.Jacobian objects
     * @param robot the robot model the jacobians refer to
     * @param desiredJacobians the model of the Desired-Transforms-DSL, which
     *        specifies also the desired Jacobians
     * @return a list with elements of type iit.dsl.generator.Jacobian, that
     *        corresponds to the desired Jacobians
     */
    def public static List<Jacobian> getJacobiansList(
        Robot robot,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacobians)
    {
        if( ! desiredJacobians.name.equals(robot.name)) {
            //TODO log warning
            System::err.println("Warning, the name of the desiredJacobians model ("
                + desiredJacobians.name + ") does not match the name of the given robot ("
                + robot.name + ").")
        }
        val ret = new ArrayList<Jacobian>()
        if( desiredJacobians.jacobians !== null ) {
            for(jSpec : desiredJacobians.jacobians.specs) {
                ret.add(new Jacobian(robot, jSpec))
            }
        }

        return ret
    }

    /**
     * The list of coordinate transforms which are required in order to compute
     * the given geometric Jacobian.
     * This function assumes that a Jacobian is computed with an iterative
     * method, given the forward kinematics of the kinematic-chain of interest.
     * The returned list is an object of the Desired-Transforms-DSL, which is
     * specifically designed to contain the specifications of a list of
     * coordinate transforms.
     * @param J the placeholder for the Jacobian matrix of interest
     * @return the specification of the coordinate transformation matrices
     *         which contain the information to compute the actual Jacobian
     */
    def public static iit.dsl.transspecs.transSpecs.TransformsList
    getRequiredTransformsSpecs(Jacobian J)
    {
        val framesFactory = iit::dsl::coord::coordTransDsl::impl::CoordTransDslFactoryImpl::init()
        val factory = iit::dsl::transspecs::transSpecs::impl::TransSpecsFactoryImpl::init()
        val list = factory.createTransformsList

        var FramePair tmpPair = factory.createFramePair
        var Frame    tmpFrame = null

        tmpFrame = framesFactory.createFrame
        tmpFrame.name = J.baseFrame.name
        tmpPair.setBase(tmpFrame)

        tmpFrame = framesFactory.createFrame
        tmpFrame.name = J.movingFrame.name
        tmpPair.setTarget(tmpFrame)

        list.specs.add(tmpPair)

        val utils = new Common()
        for(j : J.jointsChain) {
            tmpPair = factory.createFramePair

            tmpFrame = framesFactory.createFrame
            tmpFrame.name = J.baseFrame.name
            tmpPair.setBase(tmpFrame)

            tmpFrame = framesFactory.createFrame
            tmpFrame.name = utils.getFrameName(j).toString()
            tmpPair.setTarget(tmpFrame)

            list.specs.add(tmpPair)
        }

        return list
    }

    /**
     * The list of parameters referenced by the given Jacobian matrix.
     */
    def public static List<iit.dsl.coord.coordTransDsl.ParameterLiteral>
    getParameters(Jacobian J,  iit.dsl.coord.coordTransDsl.Model transforms)
    {
        // The parameters a Jacobian depends on are the same parameters of the
        //  coordinate transform between the same pair of frames
        return transformsUtils.getParams(
            transformsUtils.getTransform(transforms, J.baseFrame.name, J.movingFrame.name))
    }

    /**
     * The list of parameters-groups referenced by the given Jacobian matrix.
     */
    def public static List<iit.dsl.coord.coordTransDsl.ParametersDeclaration>
    getParametersGroups(Jacobian J,  iit.dsl.coord.coordTransDsl.Model transforms)
    {
        // The parameters a Jacobian depends on are the same parameters of the
        //  coordinate transform between the same pair of frames
        return transformsUtils.getParamsGroups(
            transformsUtils.getTransform(transforms, J.baseFrame.name, J.movingFrame.name))
    }

    /**
     * The list of the unique parameters-groups referenced by all the given
     * Jacobian matrices.
     */
    def public static getAllParametersGroups(
        List<Jacobian> jacs,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val namesSet = new HashSet<String>
        val groups = new ArrayList<iit.dsl.coord.coordTransDsl.ParametersDeclaration>
        var List<iit.dsl.coord.coordTransDsl.ParametersDeclaration> tmpGroups
        for (J : jacs) {
            tmpGroups = iit::dsl::generator::common::Jacobians::getParametersGroups(J, transformsModel)
            for( grp : tmpGroups ) {
                if( ! namesSet.contains(grp.name) ) {
                    namesSet.add(grp.name)
                    groups.add(grp)
                }
            }
        }
        return groups
    }

    /**
     * Creates a document compliant with the Maxima-DSL, containing all the
     * non-constant elements of the given Jacobian, as a list of expressions.
     *
     * Returns an empty text if the given Jacobian is not a function of any
     * joint (therefore it does not contain any expression which is a function
     * of a joint variable) nor of any parameter. The Jacobian may be constant
     * even if it formally depends on some joint.
     * @param the Jacobian placeholder
     * @param the textual representation of the Jacobian matrix, as returned by
     *     iit.dsl.generator.maxima.Converter.getjacobianText()
     * @param transformsModel the Transforms-DSL model with all the coordinate
     *     transforms of the same robot the Jacobian belongs to
     * @return the text of a document of the Maxima-DSL, with all the non
     *     constant algebraic expressions of the J
     */
    def public static expressionsAsMaximaDSLDocument(
        Jacobian J,
        String[][] JasText,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val params = getParameters(J, transformsModel)
        val vars_list_str = J.getArgsList()
        if(vars_list_str.length() == 0  &&  params.size() == 0) {
            //TODO log INFO, the Jacobian is constant
            return ''''''
        }
        val StringBuffer expressions = new StringBuffer
        // Write down the non constant expression of the given Jacobian
        for(row : JasText) {
            for(el : row) {
                if(!iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el))
                {
                    expressions.append(el + ";")
                }
            }
        }

        if(expressions.length == 0) {
            //TODO log INFO, the Jacobian is constant even though it apparently
            // depends on some joints
            return ''''''
        }
        // The Jacobian is not constant; return a non empty document text
        return
        '''
        Variables {
               «vars_list_str»
               «IF J.jointsChain.size > 0 && params.size > 0»,«ENDIF»
               «FOR p : params SEPARATOR ", "»«iit::dsl::coord::generator::maxima::Maxima::parameterToMaximaVarName(p)»«ENDFOR»
           }

        «expressions»
        '''
    }


    private static iit.dsl.coord.generator.Common transformsUtils = new iit.dsl.coord.generator.Common()
}