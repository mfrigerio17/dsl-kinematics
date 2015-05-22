package iit.dsl.generator.maxima

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.xtend2.lib.StringConcatenation


import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor
import iit.dsl.generator.Jacobian


class Generator implements IGenerator {
    Transforms transforms = new Transforms()
    Jacobians jacs = new Jacobians()
    TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robot.name + "_inertia", inertiaTensorSource(robot))
        generateTransformsSources(robot, fsa);
        generateJacobiansSources(robot, fsa);
    }

    def generateTransformsSources(
        Robot robot,
        IFileSystemAccess fsa)
    {
        val transformsModel =
           iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        generateTransformsSources(robot, fsa, transformsModel);
    }
    def generateTransformsSources(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms userTransforms)
    {
        val transformsModel =
           iit::dsl::generator::common::Transforms::getAllTransformsModel(robot, userTransforms)
        generateTransformsSources(robot, fsa, transformsModel);
    }
    def generateTransformsSources(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        transforms.generate(robot, fsa, transformsModel);
    }


    def generateJacobiansSources(
        Robot robot,
        IFileSystemAccess fsa)
    {
        generateJacobiansSources(
            robot,
            fsa,
            desiredTrasformsAccessor.getDesiredTransforms(robot))
    }

    def generateJacobiansSources(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacobians)
    {
        val transformsModel =
           iit::dsl::generator::common::Transforms::getAllTransformsModel(robot, desiredJacobians)
        generateJacobiansSources(
            robot,
            fsa,
            desiredTrasformsAccessor.getDesiredTransforms(robot),
            transformsModel)
    }

    def generateJacobiansSources(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacobians,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        fsa.generateFile(
            Jacobians::fileName(robot).toString(),
            jacobiansFileCode(robot, transformsModel, desiredJacobians)
        )
    }


    def private jacobiansFileCode(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs)
    {
        if(desiredJacs != null) {
            val StringConcatenation strBuff = new StringConcatenation();
            if(desiredJacs.jacobians != null) {
                for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredJacs.jacobians.getSpecs()) {
                    strBuff.append(jacs.jacobian(new Jacobian(robot, jSpec), transformsModel));
                }
            }
            return strBuff;
        } else {
            return new StringConcatenation();//empty
        }
    }



    def inertiaTensorSource(Robot robot) '''
        «val bp = robot.base.inertiaParams»
        I_«robot.base.name» : I(«bp.ix»,«bp.iy»,«bp.iz»,«bp.ixy»,«bp.ixz»,«bp.iyz»);
        «FOR l : robot.links»
            I_«l.name» : I(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»);
        «ENDFOR»
    '''

}
