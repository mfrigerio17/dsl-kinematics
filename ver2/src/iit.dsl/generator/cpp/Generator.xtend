package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {
    @Inject RobotHeaders headers
    @Inject RigidBodyDynamics rbd
    @Inject Jacobians jacobs

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
//        fsa.generateFile(Names$Files::mainHeader(robot)+".h", headers.main(robot))
//        fsa.generateFile(Names$Files$RBD::header(robot) + ".h", rbd.mainHeader(robot))
//        fsa.generateFile(Names$Files$RBD::source(robot) + ".cpp", rbd.inverseDynamicsImplementation(robot))
//        fsa.generateFile(Names$Files$RBD::testMain(robot) + ".cpp", rbd.testMain(robot))
        //jacobs.jacobian(robot.links.get(0), robot.links.get(1), null, null)
        //jacobs.temp(robot)
        System::out.println(headers.jacobiansHeader(robot))
    }
}