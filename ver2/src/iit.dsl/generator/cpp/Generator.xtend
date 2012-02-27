package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import java.util.ArrayList
import com.google.inject.Inject

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian


class Generator implements IGenerator {
    @Inject RobotHeaders headers
    RigidBodyDynamics rbd = new RigidBodyDynamics()
    @Inject Jacobians jacobs
    @Inject TransSpecsAccessor desiredTrasformsAccessor

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
//        fsa.generateFile(Names$Files::mainHeader(robot)+".h", headers.main(robot))
//        fsa.generateFile(Names$Files$RBD::header(robot) + ".h", rbd.mainHeader(robot))
//        fsa.generateFile(Names$Files$RBD::source(robot) + ".cpp", rbd.inverseDynamicsImplementation(robot))
//        fsa.generateFile(Names$Files$RBD::testMain(robot) + ".cpp", rbd.testMain(robot))
        fsa.generateFile(Names$Files$RBD::inertiaMatrixHeader(robot)   + ".h",   rbd.inertiaMatrixHeader(robot))
        fsa.generateFile(Names$Files$RBD::inertiaMatrixHeader(robot)   + ".cpp", rbd.inertiaMatrixSource(robot))
        fsa.generateFile(Names$Files$RBD::inertiaMatrixTestMain(robot) + ".cpp", rbd.inertiaMatrixTestMain(robot))
//        generateJacobiansFiles(robot, fsa)
//System::out.println(rbd.inertiaMatrixSource(robot))
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredTransforms = desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredTransforms != null) {
            val jacobians = new ArrayList<Jacobian>()
            for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredTransforms.jacobians.getSpecs()) {
                jacobians.add(new Jacobian(robot, jSpec))
            }
            fsa.generateFile(Names$Files::jacobiansHeader(robot) + ".h", headers.jacobiansHeader(robot, jacobians))
            fsa.generateFile(Names$Files::jacobiansSource(robot) + ".cpp", jacobs.implementationFile(robot, jacobians))
        }
    }

        // To generate C++ files with Jacobians, I first need to make sure that
        //  we have the corresponding Maxima sources
        //KinDslStandaloneSetup_Maxima::doSetup();
        //maximaGenerator.generateJacobiansSources(robot, fsa)



}