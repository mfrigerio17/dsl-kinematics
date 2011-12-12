package iit.dsl.generator.sl

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink

class Generator implements IGenerator {
    @Inject extension iit.dsl.generator.sl.Common slCommon
    @Inject extension iit.dsl.generator.Common common

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        common.init(robot);
        fsa.generateFile(slCommon.robotUserFolderName(robot)+"/"+
            Utilities::userConfigFolder +"/" +
            Utilities::linkParamsFile, generateLinkParams(robot))
    }

    /**
    Configuration file with Inertia parameters
    It is assumed that the provided moments of inertia strictly follow their definition,
    so that the off-diagonal elements of the inertia tensor have a minus sign. It is also
    assumed that the configuration file for SL expects the inertia tensor elements, so we
    put that minus in front of the values
    */
    def generateLinkParams(Robot robot) { '''
        «val bparams = Utilities::tuneForSL(robot.base.inertiaParams)»
        BASE  «bparams.mass»  «bparams.com.items.get(0)» «bparams.com.items.get(1)» «bparams.com.items.get(2)»  «bparams.ix» «bparams.ixy» «bparams.ixz» «bparams.iy» «bparams.iyz» «bparams.iz» 0.1 0 0 0
        «FOR link : robot.links»
            «val params = Utilities::tuneForSL(link.inertiaParams)»
            «link.connectingJoint.name»  «params.mass»  «params.com.items.get(0)» «params.com.items.get(1)» «params.com.items.get(2)»  «params.ix» «params.ixy» «params.ixz» «params.iy» «params.iyz» «params.iz» 0.1 0 0 0
        «ENDFOR»
        '''
    }

}
