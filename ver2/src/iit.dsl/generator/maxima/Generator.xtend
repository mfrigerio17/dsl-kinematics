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
import java.io.File
//import java.util.ArrayList
//import iit.dsl.generator.Jacobian

class Generator implements IGenerator {
    @Inject extension Common common
    @Inject Jacobians jacs
    @Inject TransSpecsAccessor desiredTrasformsAccessor

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        generateJacobiansSources(robot, fsa);
    }

    def generateJacobiansSources(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(Jacobians::fileName(robot).toString(), jacobiansFileCode(robot))
    }
    def generateJacobiansSources(Robot robot, IFileSystemAccess fsa, File desiredJacobians) {
        fsa.generateFile(Jacobians::fileName(robot).toString(), jacobiansFileCode(robot, desiredJacobians))
    }

    def jacobiansFileCode(Robot robot) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredTransforms = desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredTransforms != null) {
//            val jacobians = new ArrayList<Jacobian>()
//            for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredTransforms.jacobians.getSpecs()) {
//                jacobians.add(new Jacobian(robot, jSpec))
//            } // TODO
            return jacsCode(robot, desiredTransforms)
        }
        return new StringConcatenation();
    }

    def jacobiansFileCode(Robot robot, File desiredJacobians) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
            desiredTrasformsAccessor.getDesiredTransforms(desiredJacobians)
        if(desiredJacs != null) {
            return jacsCode(robot, desiredJacs);
        } else {
            return new StringConcatenation();//empty
        }
    }

    def private jacsCode(Robot robot, iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs) {
        val StringConcatenation strBuff = new StringConcatenation();
        for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredJacs.jacobians.getSpecs()) {
            // Convert the frames to local types
            val RefFrame base   = common.getFrameByName(robot, jSpec.base.name)
            val RefFrame target = common.getFrameByName(robot, jSpec.target.name)
            strBuff.append(jacs.jacobian(robot, base, target))
        }
        return strBuff;
    }

}
