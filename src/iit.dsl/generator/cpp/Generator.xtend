package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import java.util.ArrayList
import java.io.File
import com.google.inject.Inject

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian
import iit.dsl.generator.cpp.dynamics.InverseDynamics
import iit.dsl.generator.cpp.dynamics.JointsSpaceInertia
import iit.dsl.generator.cpp.kinematics.Jacobians
import iit.dsl.generator.cpp.kinematics.Transforms
import iit.dsl.generator.cpp.dynamics.LinkInertias
import iit.dsl.generator.cpp.dynamics.ForwardDynamics


class Generator implements IGenerator {
    RobotHeaders  headers = new RobotHeaders()
    Jacobians      jacobs = new Jacobians()
    Transforms transforms = new Transforms(new iit.dsl.coord.generator.cpp.DefaultConfigurator())
    InverseDynamics invdyn= new InverseDynamics()
    ForwardDynamics fordyn = new ForwardDynamics()
    JointsSpaceInertia jsI= new JointsSpaceInertia()
    LinkInertias inertias = new LinkInertias()
    MakefileGenerator mkg = new MakefileGenerator()
    @Inject TransSpecsAccessor desiredTrasformsAccessor

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
//        generateCommons(robot, fsa);
//        generateTransforms(robot, fsa);
//        generateInverseDynamicsStuff(robot, fsa);
        generateForwardDynamicsStuff(robot, fsa);
//        generateInertiaMatrixStuff(robot, fsa);
//        generateDynamicsTests(robot, fsa);
//        generateJacobiansFiles(robot, fsa)
//        generateLinkInertias(robot, fsa)
//        generateMakefiles(robot, fsa)
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

    def generateTransforms(Robot robot, IFileSystemAccess fsa) {
        transforms.generate(robot, fsa);
    }

    def generateInverseDynamicsStuff(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::header(robot)   + ".h"  , invdyn.mainHeader(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::source(robot)   + ".cpp", invdyn.inverseDynamicsImplementation(robot))
    }
    def generateForwardDynamicsStuff(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::abaHeader(robot) + ".h"  , fordyn.headerContent(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::abaHeader(robot) + ".cpp", fordyn.implementationFileContent(robot))
    }
    def generateDynamicsTests(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_sine_task_ID(robot) + ".cpp", invdyn.main_sine_task(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::testMain(robot) + ".cpp", invdyn.testMain(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_benchmarkID(robot) + ".cpp", invdyn.main_benchmarkID(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_jsim_test(robot) + ".cpp", jsI.main_test(robot))
    }

    def generateInertiaMatrixStuff(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);

        fsa.generateFile(folder + "/" + Names$Files$RBD::jsimHeader(robot)   + ".h",   jsI.inertiaMatrixHeader(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::jsimHeader(robot)   + ".cpp", jsI.inertiaMatrixSource(robot))
    }

    def generateLinkInertias(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$LinkInertias::header(robot) + ".h", inertias.header(robot))
        fsa.generateFile(folder + "/" + Names$Files$LinkInertias::source(robot) + ".cpp", inertias.source(robot))
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(robot)
        jacobiansFile(robot, fsa, desiredJacs);
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa, File desired) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(desired)
        jacobiansFile(robot, fsa, desiredJacs);
    }

    def private jacobiansFile(Robot robot, IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desired)
    {
        val jacobians = new ArrayList<Jacobian>()
        if(desired != null) {
            for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
                jacobians.add(new Jacobian(robot, jSpec))
            }
        }
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(
            folder + "/" + Names$Files::jacobiansHeader(robot) + ".h",
            jacobs.headerFile(robot, jacobians)
        )
        fsa.generateFile(
            folder + "/" + Names$Files::jacobiansHeader(robot) + ".cpp",
            jacobs.implementationFile(robot, jacobians)
        )
    }

}

