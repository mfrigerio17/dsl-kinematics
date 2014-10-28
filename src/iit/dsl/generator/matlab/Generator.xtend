package iit.dsl.generator.matlab

import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian


import java.io.File
import java.util.ArrayList

class Generator implements IGenerator {
    def static String robotFolderName(Robot robot) {
        return robot.name.toLowerCase()
    }

    private TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    private Jacobians  jacGen   = null
    private Jsim       jsimGen  = null
    private Transforms transGen = null
    private InertiaProperties inertiaGen = new InertiaProperties()
    private RoysModel roy = new RoysModel()

    private iit.dsl.generator.maxima.IConverterConfigurator maximaConverterConfig = null


    public new() {
        jsimGen = new Jsim()
    }

    public new(iit.dsl.generator.maxima.IConverterConfigurator maximaConverterCfg)
    {
        this()
        setMaximaConverterConfigurator(maximaConverterConfig)
    }

    /**
     * Sets the iit.dsl.generator.maxima.IConverterConfigurator instance
     * that will be used by this generator whenever a conversion from Maxima
     * code has to be performed.
     */
    def public void setMaximaConverterConfigurator(
        iit.dsl.generator.maxima.IConverterConfigurator config)
    {
        maximaConverterConfig = config
    }

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robotFolderName(robot) + "/feath_model.m", featherstoneMatlabModel(robot))

        generateJacobiansFiles(robot, fsa)
        generateTransformsFiles(robot, fsa)
        generatePlotFramesFile(robot, fsa)
        generateCommonDynamicsFiles(robot, fsa)
        generateJSIMFiles(robot, fsa)
    }

    def generatePlotFramesFile(Robot robot, IFileSystemAccess fsa) {
        // Uses the default transforms model
        generatePlotFramesFile( robot, fsa,
            iit::dsl::generator::common::Transforms::getTransformsModel(robot))
    }
    def generatePlotFramesFile(Robot robot, IFileSystemAccess fsa,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val PlotFrames gen = new PlotFrames(robot, transformsModel)
        fsa.generateFile(robotFolderName(robot) + "/plot_frames.m"  , gen.plotFramesCode())
    }

    def generateTransformsFiles(
        Robot robot,
        IFileSystemAccess fsa)
    {
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        generateTransformsFiles(robot, fsa, transformsModel)
    }

    def generateTransformsFiles(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        transGen = new Transforms(robot, transformsModel, maximaConverterConfig)
        fsa.generateFile(robotFolderName(robot) + "/init_homogeneous.m"  , transGen.homogeneous_init_fileContent(robot))
        fsa.generateFile(robotFolderName(robot) + "/update_homogeneous.m", transGen.homogeneous_update_fileContent(robot))
        fsa.generateFile(robotFolderName(robot) + "/init_6Dmotion.m"  , transGen.motion6D_init_fileContent(robot))
        fsa.generateFile(robotFolderName(robot) + "/update_6Dmotion.m", transGen.motion6D_update_fileContent(robot))
        fsa.generateFile(robotFolderName(robot) + "/init_6Dforce.m"  , transGen.force6D_init_fileContent(robot))
        fsa.generateFile(robotFolderName(robot) + "/update_6Dforce.m", transGen.force6D_update_fileContent(robot))
    }


    //  JACOBIANS //

     def generateJacobiansFiles(
         Robot robot,
         IFileSystemAccess fsa)
     {
         val desiredJacs     = desiredTrasformsAccessor.getDesiredTransforms(robot)
         val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
         if(transformsModel == null) {
             //TODO log warning
             System::err.println("WARNING, could not load the default transforms-model for the robot " + robot)
         }

         if(desiredJacs != null) {
             generateJacobiansFiles(robot, fsa, desiredJacs, transformsModel)
         }
     }

    def generateJacobiansFiles(
        Robot robot,
        IFileSystemAccess fsa,
        File desiredJacobians)
    {
        val desiredJacs     = desiredTrasformsAccessor.getDesiredTransforms(desiredJacobians)
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        if(transformsModel == null) {
             //TODO log warning
             System::err.println("WARNING, could not load the default transforms-model for the robot " + robot.name)
         }

        if(desiredJacs != null) {
            generateJacobiansFiles(robot, fsa, desiredJacs, transformsModel)
        }
    }

    def public generateJacobiansFiles(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desired,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        jacGen = new Jacobians(new MaximaReplSpecs(robot, transformsModel))
        jacGen.setMaximaConverterConfigurator(maximaConverterConfig)
        val jacobians = new ArrayList<Jacobian>()
        if(desired != null) {
            if(desired.jacobians != null) {
                for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
                    jacobians.add(new Jacobian(robot, jSpec))
                }
            }
        }
        fsa.generateFile(robotFolderName(robot) + "/init_jacs.m",
            jacGen.init_jacobians_file(robot, jacobians, transformsModel))
        fsa.generateFile(robotFolderName(robot) + "/update_jacs.m",
            jacGen.update_jacobians_file(robot, jacobians, transformsModel))
    }




    def generateJSIMFiles(Robot robot, IFileSystemAccess fsa)
    {
        // Get the default transforms model for this robot
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        generateJSIMFiles(robot, transformsModel, fsa)
    }

    def generateJSIMFiles(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        IFileSystemAccess fsa)
    {
        fsa.generateFile(robotFolderName(robot) + "/init_jsim.m"  , jsimGen.jsim_init_code(robot))
        fsa.generateFile(robotFolderName(robot) + "/update_jsim.m", jsimGen.jsim_update_code(robot, transformsModel))
        fsa.generateFile(robotFolderName(robot) + "/jsim_inverse.m", jsimGen.jsim_inverse_code(robot))
    }



    def generateCommonDynamicsFiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(robotFolderName(robot) + "/inertia_properties.m",
            inertiaGen.scriptContent(robot)  )
    }



    /**
     * This template generates a matlab file with the initialization of
     * a structure describing the robot kinematics, according to the model described
     * in "A beginner's guide to &-D vectors (part 2)" by Roy Featherstone.
     */
    def featherstoneMatlabModel(Robot robot) {
        roy.scriptBody(robot)
    }
}