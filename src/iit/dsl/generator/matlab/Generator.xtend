package iit.dsl.generator.matlab

import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian
import iit.dsl.generator.matlab.config.IConfigurator
import iit.dsl.generator.matlab.config.DefaultConfigurator
import iit.dsl.generator.maxima.DefaultConverterConfigurator

import static iit.dsl.coord.generator.matlab.Generator.initFunctionName
import static iit.dsl.coord.generator.matlab.Generator.updateFunctionName

import java.io.File
import java.util.ArrayList
import iit.dsl.coord.generator.Utilities


class Generator implements IGenerator {
    def static String robotFolderName(Robot robot) {
        return robot.name.toLowerCase()
    }

    private TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    private Jacobians  jacGen   = null
    private Jsim       jsimGen  = new Jsim()
    private Transforms transGen = null
    private InertiaProperties inertiaGen = new InertiaProperties()
    private CompositeInertia ciGen = new CompositeInertia()
    private InverseDynamics idGen = new InverseDynamics();
    private RoysModel roy = new RoysModel()

    private IConfigurator configurator = null


    public new()
    {
    }

    public new(IConfigurator config)
    {
        setConfigurator(config)
    }

    def public void setConfigurator(IConfigurator config)
    {
        configurator = config
    }

    override void doGenerate(Resource resource, IFileSystemAccess fsa)
    {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robotFolderName(robot) + "/" +
            RoysModel::functionName(robot) + ".m", featherstoneMatlabModel(robot))

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
        fsa.generateFile(robotFolderName(robot) + "/" + PlotFrames::functionName + ".m"  , gen.plotFramesCode())
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
        if(configurator == null) {
            // should never really get here, the user should pass a configurator
            configurator = new DefaultConfigurator(robot, transformsModel, new DefaultConverterConfigurator())
        }

        transGen = new Transforms(robot, transformsModel, configurator);

        var iit.dsl.coord.generator.Utilities$MatrixType mxType
        var String fileName

        mxType   = iit.dsl.coord.generator.Utilities$MatrixType::HOMOGENEOUS;

        fileName = robotFolderName(robot) + "/" + initFunctionName(mxType) + ".m"
        fsa.generateFile(fileName  , transGen.homogeneous_init_fileContent(robot))
        fileName = robotFolderName(robot) + "/" + updateFunctionName(mxType) + ".m"
        fsa.generateFile(fileName, transGen.homogeneous_update_fileContent(robot))


        mxType   = iit.dsl.coord.generator.Utilities$MatrixType::_6D;

        fileName = robotFolderName(robot) + "/" + initFunctionName(mxType) + ".m"
        fsa.generateFile(fileName, transGen.motion6D_init_fileContent(robot))
        fileName = robotFolderName(robot) + "/" + updateFunctionName(mxType) + ".m"
        fsa.generateFile(fileName, transGen.motion6D_update_fileContent(robot))


        mxType   = iit.dsl.coord.generator.Utilities$MatrixType::_6D_FORCE;

        fileName = robotFolderName(robot) + "/" + initFunctionName(mxType) + ".m"
        fsa.generateFile(fileName, transGen.force6D_init_fileContent(robot))
        fileName = robotFolderName(robot) + "/" + updateFunctionName(mxType) + ".m"
        fsa.generateFile(fileName, transGen.force6D_update_fileContent(robot))
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
        if(configurator == null) {
            // should never really get here, the user should pass a configurator
            configurator = new DefaultConfigurator(robot, transformsModel, new DefaultConverterConfigurator())
        }
        jacGen = new Jacobians( configurator )

        val jacobians = new ArrayList<Jacobian>()
        if(desired != null) {
            if(desired.jacobians != null) {
                for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
                    jacobians.add(new Jacobian(robot, jSpec))
                }
            }
        }
        fsa.generateFile(
            robotFolderName(robot) + "/" + Jacobians::initFunctionName + ".m",
            jacGen.init_jacobians_file(robot, jacobians, transformsModel))
        fsa.generateFile(
            robotFolderName(robot) + "/" + Jacobians::updateFunctionName + ".m",
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
        fsa.generateFile(robotFolderName(robot) + "/" + Jsim::updateFunctionName + ".m", jsimGen.jsim_update_code(robot, transformsModel))
        fsa.generateFile(robotFolderName(robot) + "/" + Jsim::invertFunctionName + ".m", jsimGen.jsim_inverse_code(robot))
    }

    def generateIDFiles(Robot robot, IFileSystemAccess fsa)
    {
        fsa.generateFile(robotFolderName(robot) + "/" + InverseDynamics::functionName + ".m",
            idGen.body(robot) )
    }

    def generateCommonDynamicsFiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(robotFolderName(robot) + "/" + InertiaProperties::functionName + ".m",
            inertiaGen.scriptContent(robot)  )

        fsa.generateFile(robotFolderName(robot) + "/" + CompositeInertia::functionName + ".m",
            ciGen.functionBody(robot)  )
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