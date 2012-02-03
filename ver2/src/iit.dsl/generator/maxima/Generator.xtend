package iit.dsl.generator.maxima

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.generator.Common

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RefFrame
import iit.dsl.TransSpecsAccessor
import org.eclipse.xtend2.lib.StringConcatenation

class Generator implements IGenerator {
    @Inject extension Common common
    @Inject Jacobians jacs
    @Inject TransSpecsAccessor desiredTrasformsAccessor

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(Jacobians::fileName(robot).toString(), jacobiansFileCode(robot))
    }

    def jacobiansFileCode(Robot robot) {
        val StringConcatenation strBuff = new StringConcatenation();
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredTransforms = desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredTransforms != null) {
            for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredTransforms.jacobians.getSpecs()) {
                // Convert the frames to local types
                val RefFrame base   = common.getFrameByName(robot, jSpec.base.name)
                val RefFrame target = common.getFrameByName(robot, jSpec.target.name)
                strBuff.append(jacs.jacobian(robot, base, target))
            }
        }
        return strBuff;
     }

}
