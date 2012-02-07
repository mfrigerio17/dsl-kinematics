package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import java.util.ArrayList
import iit.dsl.generator.Jacobian

class Generator implements IGenerator {
    @Inject RobotHeaders headers
    @Inject RigidBodyDynamics rbd
    @Inject Jacobians jacobs
    @Inject TransSpecsAccessor desiredTrasformsAccessor

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
//        fsa.generateFile(Names$Files::mainHeader(robot)+".h", headers.main(robot))
//        fsa.generateFile(Names$Files$RBD::header(robot) + ".h", rbd.mainHeader(robot))
//        fsa.generateFile(Names$Files$RBD::source(robot) + ".cpp", rbd.inverseDynamicsImplementation(robot))
//        fsa.generateFile(Names$Files$RBD::testMain(robot) + ".cpp", rbd.testMain(robot))
        generateJacobiansFile(robot, fsa)
    }

    def generateJacobiansFile(Robot robot, IFileSystemAccess fsa) {
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


}