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


class Generator implements IGenerator {
    RobotHeaders  headers = new RobotHeaders()
    Jacobians      jacobs = new Jacobians()
    Transforms transforms = new Transforms()
    InverseDynamics invdyn= new InverseDynamics()
    JointsSpaceInertia jsI= new JointsSpaceInertia()
    MakefileGenerator mkg = new MakefileGenerator()
    @Inject TransSpecsAccessor desiredTrasformsAccessor

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        generateCommons(robot, fsa);
        generateTransforms(robot, fsa);
        generateInverseDynamicsStuff(robot, fsa);
        generateJacobiansFiles(robot, fsa)
        generateInertiaMatrixStuff(robot, fsa);
        fsa.generateFile(Names$Files::folder(robot) + "/" + "Makefile", mkg.makefileBody(robot))

        //System::out.println(rbd.LTLfactorization(robot))
        //System::out.println(rbd.Linverse(robot))
        //System::out.println(rbd.Minverse(robot))
    }

    def generateCommons(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(
            Names$Files::folder(robot) + "/" + Names$Files::mainHeader(robot) + ".h",
            headers.main(robot)
        )
    }

    def generateTransforms(Robot robot, IFileSystemAccess fsa) {
        transforms.generate(robot, fsa);
    }

    def generateInverseDynamicsStuff(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);
        fsa.generateFile(folder + "/" + Names$Files$RBD::header(robot)   + ".h"  , invdyn.mainHeader(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::source(robot)   + ".cpp", invdyn.inverseDynamicsImplementation(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::testMain(robot) + ".cpp", invdyn.testMain(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::main_benchmarkID(robot) + ".cpp", invdyn.main_benchmarkID(robot))
    }

    def generateInertiaMatrixStuff(Robot robot, IFileSystemAccess fsa) {
        val String folder = Names$Files::folder(robot);

        fsa.generateFile(folder + "/" + Names$Files$RBD::inertiaMatrixHeader(robot)   + ".h",   jsI.inertiaMatrixHeader(robot))
        fsa.generateFile(folder + "/" + Names$Files$RBD::inertiaMatrixHeader(robot)   + ".cpp", jsI.inertiaMatrixSource(robot))
        //fsa.generateFile(folder + "/" + Names$Files$RBD::inertiaMatrixTestMain(robot) + ".cpp", jsI.inertiaMatrixTestMain(robot))
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredJacs != null) {
            jacobiansFile(robot, fsa, desiredJacs);
        }
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa, File desired) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(desired)
        if(desiredJacs != null) {
            jacobiansFile(robot, fsa, desiredJacs);
        }
    }

    def private jacobiansFile(Robot robot, IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desired)
    {
        val jacobians = new ArrayList<Jacobian>()
        for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
            jacobians.add(new Jacobian(robot, jSpec))
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

        // To generate C++ files with Jacobians, I first need to make sure that
        //  we have the corresponding Maxima sources
        //KinDslStandaloneSetup_Maxima::doSetup();
        //maximaGenerator.generateJacobiansSources(robot, fsa)

