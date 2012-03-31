package iit.dsl.generator.maxima

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.xtend2.lib.StringConcatenation

import java.io.File
import com.google.inject.Inject

import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian

class Generator implements IGenerator {
    Transforms transforms = new Transforms()
    @Inject Jacobians jacs
    @Inject TransSpecsAccessor desiredTrasformsAccessor

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        generateTransformsSources(robot, fsa);
        generateJacobiansSources(robot, fsa);
    }

    def generateTransformsSources(Robot robot, IFileSystemAccess fsa) {
        transforms.generate(robot, fsa);
    }


    def generateJacobiansSources(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(Jacobians::fileName(robot).toString(), jacobiansFileCode(robot))
    }
    def generateJacobiansSources(Robot robot, IFileSystemAccess fsa, File desiredJacobians) {
        fsa.generateFile(Jacobians::fileName(robot).toString(), jacobiansFileCode(robot, desiredJacobians))
    }

    def jacobiansFileCode(Robot robot) {
        return jacobiansFileCode(robot,
                    // gets the default desired-jacobians model
                    desiredTrasformsAccessor.getDesiredTransforms(robot));
    }

    def jacobiansFileCode(Robot robot, File desiredJacobians) {
         return jacobiansFileCode(robot,
                    desiredTrasformsAccessor.getDesiredTransforms(desiredJacobians));
    }

    def jacobiansFileCode(Robot robot, iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs) {
        if(desiredJacs != null) {
            return jacsCode(robot, desiredJacs);
        } else {
            return new StringConcatenation();//empty
        }
    }


    def private jacsCode(Robot robot, iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs) {
        val StringConcatenation strBuff = new StringConcatenation();
        for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredJacs.jacobians.getSpecs()) {
            strBuff.append(jacs.jacobian(new Jacobian(robot, jSpec)));
        }
        return strBuff;
    }

}
