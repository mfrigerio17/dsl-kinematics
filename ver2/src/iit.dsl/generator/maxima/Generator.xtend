package iit.dsl.generator.maxima

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.generator.Common

import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {
    @Inject extension Common common
    @Inject Jacobians jacs

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(Jacobians::fileName(robot).toString(), generateJacobiansFile(robot))
    }

    def generateJacobiansFile(Robot robot)'''
        «IF(robot.name.equals("FixedHyQ"))»
            «val link = robot.getLinkByName("LF_lowerleg")»
            «jacs.jacobian(robot.getLinkByName("trunk"), link.getFrameByName("LF_foot"))»
        «ENDIF»
        «IF(robot.name.equals("FixedLeg"))»
            «val link = robot.getLinkByName("lowerLeg")»
            «jacs.jacobian(robot.getLinkByName("Hip"), link.getFrameByName("Foot"))»
        «ENDIF»
    '''
}
