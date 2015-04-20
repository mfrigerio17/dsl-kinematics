package iit.dsl.generator.common

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Common
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.MotionDSLDocGenerator

/**
 * Miscellaneous utilities related to the Domain Specific Language for the
 * coordinate transforms (the Transforms-DSL project) and the one for the
 * desired transforms (the DesiredTransforms-DSL project)
 *
 * The "default coordinate transforms" for a robot (mentioned in the rest of
 * the comments) are in general those required by the dynamics algorithms, that
 * is the transforms whose implementation has to be generated regardless the
 * user's needs.
 */
class TransformsDSLsUtils {

    def public static documentDefaultName_TransformsDSL(Robot robot) {
        robot.name + ".ctdsl"
    }

    /**
     * Content of a document of the DesiredTransforms-DSL, with the default
     * coordinate transforms for the given robot.
     */
    def public CharSequence defaultDesiredTransformsDSLDoc(Robot robot) '''
        Robot «robot.name»

        Frames {
            «common.getFrameName(robot.base)»
            «FOR link : robot.links BEFORE ", " SEPARATOR ", "»«common.getFrameName(link)»«ENDFOR»
        }

        Transforms {
            «FOR link : robot.links»
                «val parent = common.getFrameName(common.getParent(link))»
                left=«common.getFrameName(link)»  right=«parent»
                left=«parent»   right=«common.getFrameName(link)»
            «ENDFOR»
        }
    '''


    def public iit.dsl.transspecs.transSpecs.DesiredTransforms
        addTransformsForJacobians(
            Robot robot,
            iit.dsl.transspecs.transSpecs.DesiredTransforms user)
    {
        val jacsList = Jacobians::getJacobiansList(robot, user)
        var iit.dsl.transspecs.transSpecs.TransformsList allTransforms =
            user.transforms
        for (J : jacsList ) {
            allTransforms = iit::dsl::transspecs::utils::Utils::merge(
                allTransforms,
                Jacobians::getRequiredTransformsSpecs(J) )
        }
        return iit::dsl::transspecs::utils::Utils::createModel(allTransforms)
    }

    /**
     * Content of a document of the Transforms-DSL, with the specification of
     * the coordinate transforms for the given robot.
     * The returned document contains both the default transforms for the robot
     * and those possibly specified by the second parameter
     * @param robot the robot of interest
     * @userTransforms the transforms to be included in the returned document,
     *   in addition to the default ones. Optional argument, can be null
     */
    def public CharSequence coordinateTransformsDSLDoc(Robot robot,
        iit.dsl.transspecs.transSpecs.DesiredTransforms userTransforms)
    {
        val defaultTransforms =
                desiredTrasformsAccessor.getModel( defaultDesiredTransformsDSLDoc(robot).toString() );

        var iit.dsl.transspecs.transSpecs.DesiredTransforms allDesiredTransforms = null

        if(userTransforms != null) {
            val allUserTransforms = addTransformsForJacobians(robot, userTransforms)

            allDesiredTransforms= iit::dsl::transspecs::utils::Utils::merge(allUserTransforms, defaultTransforms)
        } else {
            allDesiredTransforms = defaultTransforms
        }

        // The model of the rigid motions describing the pose of the robot frames
        val motionsModel = motionDSLAccessor.getModel(generatorOfMotionDSLDocs.documentContent(robot).toString())

        // The returned document of the Transforms-DSL is obtained by using the
        //  generator of the Motion-DSL
        return
        generatorOfTransformsDSLDocs.generateDSLDoc(motionsModel,
            iit::dsl::coord::generator::Utilities$MatrixConvention::TRANSFORMED_FRAME_ON_THE_RIGHT,
            allDesiredTransforms)
    }




    private  Common common = new Common()
    private TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    private MotionDSLDocGenerator generatorOfMotionDSLDocs =
                                                    new MotionDSLDocGenerator()

    // Data members from the Motion-DSL package
    private iit.dsl.motiondsl.generator.CoordTransDSL_generator generatorOfTransformsDSLDocs =
                       new iit.dsl.motiondsl.generator.CoordTransDSL_generator()

    private iit.dsl.motiondsl.utils.DSLAccessor motionDSLAccessor = new iit.dsl.motiondsl.utils.DSLAccessor()

}