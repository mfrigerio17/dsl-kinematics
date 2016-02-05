package iit.dsl.generator

import iit.dsl.generator.common.TransformsDSLsUtils
import iit.dsl.TransSpecsAccessor
import iit.dsl.kinDsl.Robot

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGenerator2
import org.eclipse.xtext.generator.IGeneratorContext


class KinDslGenerator implements IGenerator2
{
    MotionDSLDocGenerator motiongen = new MotionDSLDocGenerator()
    TransformsDSLsUtils transformsDSLsUtils = new TransformsDSLsUtils()

    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context)
    {
        val robot = resource.contents.head as Robot;

        val desiredTrasformsAccessor = new TransSpecsAccessor()
        val desired = desiredTrasformsAccessor.getDesiredTransforms(robot)

        val maximaGen = new iit.dsl.generator.maxima.Generator()
        val cppGen    = new iit.dsl.generator.cpp.Generator()
        val matlabGen = new iit.dsl.generator.matlab.Generator()
        val slGen     = new iit.dsl.generator.sl.Generator()
        val miscGen   = iit::dsl::generator::misc::Misc::getInstance()

        fsa.generateFile(MotionDSLDocGenerator::fileName(robot), motiongen.documentContent(robot))
        fsa.generateFile(
            TransformsDSLsUtils::documentDefaultName_TransformsDSL(robot),
            transformsDSLsUtils.coordinateTransformsDSLDoc(robot, desired))

        maximaGen.doGenerate(resource, fsa)
        cppGen   .doGenerate(resource, fsa)
        matlabGen.doGenerate(resource, fsa)
        slGen    .doGenerate(resource, fsa)
        fsa.generateFile(robot.name+".urdf", miscGen.URDF_ROS_model(robot))
        fsa.generateFile(robot.name+".sd", miscGen.SDFAST_model(robot))
    }

    override afterGenerate(Resource input, IFileSystemAccess2 fsa, IGeneratorContext context) {
    }
    override beforeGenerate(Resource input, IFileSystemAccess2 fsa, IGeneratorContext context) {
    }



}
