package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import java.util.ArrayList
import java.io.File

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian
import iit.dsl.generator.cpp.dynamics.InverseDynamics
import iit.dsl.generator.cpp.dynamics.JointsSpaceInertia
import iit.dsl.generator.cpp.kinematics.Jacobians
import iit.dsl.generator.cpp.kinematics.Transforms
import iit.dsl.generator.cpp.dynamics.LinkInertias
import iit.dsl.generator.cpp.dynamics.ForwardDynamics

import iit.dsl.generator.cpp.config.IConfiguratorsGetter
import iit.dsl.generator.cpp.config.DefaultConfiguratorsGetter



class Generator implements IGenerator {

    new() {
        this(new DefaultConfiguratorsGetter)
    }

    new(IConfiguratorsGetter getter) {
        setConfiguratorsGetter(getter)
    }

//    def private init() {
//        headers = new RobotHeaders()
//         jacobs = new Jacobians()
//        invdyn  = new InverseDynamics()
//        jsI     = new JointsSpaceInertia()
//       inertias = new LinkInertias()
//         mkg    = new MakefileGenerator()
//         desiredTrasformsAccessor = new TransSpecsAccessor()
//    }


    private RobotHeaders  headers = new RobotHeaders()
    private Jacobians      jacobs = new Jacobians()
    private InverseDynamics invdyn= new InverseDynamics()
    private ForwardDynamics fordyn= new ForwardDynamics()
    private JointsSpaceInertia jsI= new JointsSpaceInertia()
    private LinkInertias inertias = new LinkInertias()
    private MakefileGenerator mkg = new MakefileGenerator()
    private TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()

    private IConfiguratorsGetter configGetter = null

    def public void setConfiguratorsGetter(IConfiguratorsGetter getter) {
        if(getter == null) {
            //TODO log warning
            return;
        }
        configGetter = getter

        Names::setConfigurators(
            configGetter.fileNamesConfigurator,
            configGetter.namespacesConfigurator,
            configGetter.classesAndTypesConfigurator)

        jacobs.setMaximaConverterConfigurator(configGetter.maximaConverterConfigurator)
    }

    /**
     * Sets the iit.dsl.coord.generator.MaximaConverter$IConfigurator object
     * that will be used by this generator whenever a conversion from Maxima
     * code has to be performed.
     */
    def public void setMaximaConverterConfigurator(
        iit.dsl.generator.maxima.IConverterConfigurator config)
    {
        jacobs.setMaximaConverterConfigurator(config)
    }

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        // The default transforms model for the robot
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot);
        generateCommons(robot, fsa);
        generateTransforms(robot, fsa, transformsModel);
        generateInverseDynamicsStuff(robot, fsa, transformsModel);
        generateForwardDynamicsStuff(robot, fsa, transformsModel);
        generateInertiaMatrixStuff(robot, fsa, transformsModel);
        generateDynamicsTests(robot, fsa);
        generateJacobiansFiles(robot, fsa)
        generateLinkInertias(robot, fsa)
        generateMakefiles(robot, fsa)
        //System::out.println(rbd.LTLfactorization(robot))
        //System::out.println(rbd.Linverse(robot))
        //System::out.println(rbd.Minverse(robot))
    }

    def generateMakefiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(Names$Files::folder(robot) + "/" + "Makefile", mkg.makefileBody(robot))
        fsa.generateFile(Names$Files::folder(robot) + "/" + "CMakeLists.txt", mkg.CMakeFileBody(robot))
    }

    def generateCommons(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(
            Names$Files::folder(robot) + "/" + Names$Files::mainHeader(robot) + ".h",
            headers.main(robot)
        )
        fsa.generateFile(
            Names$Files::folder(robot) + "/" + Names$Files::linkDataMapHeader(robot) + ".h",
            headers.linkDataMap(robot)
        )
        fsa.generateFile(
            Names$Files::folder(robot) + "/" + Names$Files::jointDataMapHeader(robot) + ".h",
            headers.jointDataMap(robot)
        )
    }



    def generateTransforms(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val transforms = new Transforms(
            configGetter.getTransformsDSLGeneratorConfigurator(robot))
        transforms.generate(robot, fsa, transformsModel);
    }


    def generateInverseDynamicsStuff(Robot robot, IFileSystemAccess fsa, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::header(robot) + ".h"  ,
            invdyn.mainHeader(robot, transformsModel) )
        fsa.generateFile(folder + "/" + Names$Files$RBD::source(robot) + ".cpp",
            invdyn.inverseDynamicsImplementation(robot, transformsModel) )
    }
    def generateForwardDynamicsStuff(Robot robot, IFileSystemAccess fsa, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::abaHeader(robot) + ".h"  , fordyn.headerContent(robot, transformsModel))
        fsa.generateFile(folder + "/" + Names$Files$RBD::abaHeader(robot) + ".cpp", fordyn.implementationFileContent(robot, transformsModel))
    }
    def generateDynamicsTests(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_sine_task_ID(robot) + ".cpp", invdyn.main_sine_task(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::testMain(robot) + ".cpp", invdyn.main_test(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_benchmarkID(robot) + ".cpp", invdyn.main_benchmarkID(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_jsim_test(robot) + ".cpp", jsI.main_test(robot))
    }

    def generateInertiaMatrixStuff(Robot robot, IFileSystemAccess fsa, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val String folder = Names$Files::folder(robot);

        fsa.generateFile(folder + "/" + Names$Files$RBD::jsimHeader(robot)   + ".h",   jsI.inertiaMatrixHeader(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::jsimHeader(robot)   + ".cpp", jsI.inertiaMatrixSource(robot,transformsModel))
    }

    def generateLinkInertias(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::inertiaHeader(robot) + ".h", inertias.header(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::inertiaHeader(robot) + ".cpp", inertias.source(robot))
    }





    def public generateJacobiansFiles(
        Robot robot,
        IFileSystemAccess fsa)
    {
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(robot)
        generateJacobiansFiles(robot, fsa, desiredJacs, transformsModel);
    }


    def public generateJacobiansFiles(
        Robot robot,
        IFileSystemAccess fsa,
        File desired)
    {
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(desired)
        generateJacobiansFiles(robot, fsa, desiredJacs, transformsModel);
    }


    def public generateJacobiansFiles(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desired,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val jacobians = new ArrayList<Jacobian>()
        if(desired != null) {
            if(desired.jacobians != null) {
                for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
                    jacobians.add(new Jacobian(robot, jSpec))
                }
            }
        }
        val paramsHeader = configGetter.getTransformsDSLGeneratorConfigurator(robot).paramsHeaderFileName(transformsModel)
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(
            folder + "/" + Names$Files::jacobiansHeader(robot) + ".h",
            jacobs.headerFile(robot, transformsModel, jacobians, paramsHeader)
        )
        fsa.generateFile(
            folder + "/" + Names$Files::jacobiansHeader(robot) + ".cpp",
            jacobs.sourceFile(robot, transformsModel, jacobians)
        )
    }

}

