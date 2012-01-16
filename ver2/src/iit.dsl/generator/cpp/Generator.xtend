package iit.dsl.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {
    @Inject RobotHeaders headers

	override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(RobotHeaders::mainHeaderFileName(robot)+".h", headers.main(robot))
    }
}