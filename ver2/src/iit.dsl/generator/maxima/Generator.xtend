package iit.dsl.generator.maxima

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.generator.Common

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink

class Generator implements IGenerator {
    @Inject extension Common common

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        common.init(robot)
        fsa.generateFile(robot.name+"_transforms.maxima", generateCode(robot))
    }

    def generateCode(Robot robot) '''
        «FOR link : robot.links»
            «common.getParent(link)»
        «ENDFOR»
    '''
}
