package iit.dsl.generator.matlab

import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Common
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.InertiaParams
import iit.dsl.TransSpecsAccessor
import java.util.ArrayList
import iit.dsl.generator.Jacobian

class Generator implements IGenerator {
    extension Common common = new Common()

    TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    Jacobians jacGen = new Jacobians()

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robot.name.toLowerCase() + "_inertia.m", inertiaParams(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_feath_model.m", featherstoneMatlabModel(robot))

        generateJacobiansFiles(robot, fsa)
    }

    def generateJacobiansFiles(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredJacs =
                    desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredJacs != null) {
            jacobiansFiles(robot, fsa, desiredJacs)
        }
    }

    def private jacobiansFiles(Robot robot, IFileSystemAccess fsa,
        iit.dsl.transspecs.transSpecs.DesiredTransforms desired)
    {
        val jacobians = new ArrayList<Jacobian>()
        for(iit.dsl.transspecs.transSpecs.FramePair jSpec : desired.jacobians.getSpecs()) {
            jacobians.add(new Jacobian(robot, jSpec))
        }
        fsa.generateFile(robot.name.toLowerCase() + "_init_jacs.m", jacGen.init_jacobians_file(robot, jacobians))
        fsa.generateFile(robot.name.toLowerCase() + "_update_jacs.m", jacGen.update_jacobians_file(robot, jacobians))
    }




    def inertiaParams(Robot robot) '''
         «val bp = robot.base.inertiaParams»
        I_«robot.base.name» = [[ «bp.ix», -(«bp.ixy»), -(«bp.ixz»)];
                               [-(«bp.ixy»), «bp.iy» , -(«bp.iyz»)];
                               [-(«bp.ixz»),-(«bp.iyz»),  «bp.iz»]];
        «FOR l : robot.links»
            I_«l.name» = [[ «l.inertiaParams.ix», -(«l.inertiaParams.ixy»), -(«l.inertiaParams.ixz»)];
                          [-(«l.inertiaParams.ixy»), «l.inertiaParams.iy» , -(«l.inertiaParams.iyz»)];
                          [-(«l.inertiaParams.ixz»),-(«l.inertiaParams.iyz»),  («l.inertiaParams.iz»)]];
        «ENDFOR»
    '''

    /**
     * This template generates a matlab file with the initialization of
     * a structure describing the robot kinematics, according to the model described
     * in "A beginner's guide to &-D vectors (part 2)" by Roy Featherstone.
     */
    def featherstoneMatlabModel(Robot robot) '''
        «val structName = robot.name.toLowerCase + "model"»
        «structName».robotname = '«robot.name»';
        «structName».NB = «robot.movingBodiesCount»;
        «structName».parent = zeros(1, «robot.links.size»);

        «IF robot.base.isFloating»
            «structName».parent(«robot.base.ID») = 0;
        «ENDIF»
        «FOR l : robot.links»
            «structName».parent(«l.ID») = «l.parent.ID»;
        «ENDFOR»

        «FOR j : robot.joints»
            «structName».pitch(«j.num») = «jointPitch(j)»;
        «ENDFOR»

        «FOR j : robot.joints»
            «structName».Xtree{«j.num»} = «jointTransform(j)»;
        «ENDFOR»

        «FOR l : robot.links»
            «structName».«inertiaParams(l)»
        «ENDFOR»
    '''


    def private dispatch jointPitch(RevoluteJoint j) '''0.0'''
    def private dispatch jointPitch(PrismaticJoint j) '''inf'''
    def private jointTransform(Joint j) '''
        Xrotz(«j.refFrame.rotation.z.asFloat» ) * ...
        Xroty(«j.refFrame.rotation.y.asFloat» ) * ...
        Xrotx(«j.refFrame.rotation.x.asFloat» ) * ...
        Xtrans([«j.refFrame.translation.listCoordinates()»]);
    '''

    def private inertiaParams(AbstractLink l) '''
        I{«l.ID»} = mcI(«l.inertiaParams.mass», [«l.inertiaParams.com.listCoordinates»], ...
        «inertiaTensor(l.inertiaParams)» );
    '''
    def private inertiaTensor(InertiaParams ip)  '''
        [ [ «ip.ix»    -(«ip.ixy») -(«ip.ixz»)]; ...
          [-(«ip.ixy»)   «ip.iy»   -(«ip.iyz»)]; ...
          [-(«ip.ixz») -(«ip.iyz»)   «ip.iz»] ] ...
    '''
}