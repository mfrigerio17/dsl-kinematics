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
import iit.dsl.generator.Utilities

class Generator implements IGenerator {
    extension Common common = new Common()

    TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    Jacobians  jacGen   = new Jacobians()
    Transforms transGen = null

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        //fsa.generateFile(robot.name.toLowerCase() + "_inertia.m", inertiaParams(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_feath_model.m", featherstoneMatlabModel(robot))

        //generateJacobiansFiles(robot, fsa)
        //generateTransformsFiles(robot, fsa);
    }

    def generateTransformsFiles(Robot robot, IFileSystemAccess fsa) {
        transGen = new Transforms(robot)
        fsa.generateFile(robot.name.toLowerCase() + "_init_homogeneous.m"  , transGen.homogeneous_init_fileContent(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_update_homogeneous.m", transGen.homogeneous_update_fileContent(robot))
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
        «val TAB = "\t"»
        I_«robot.base.name» = ...
            [[ «bp.ix»,«TAB»-(«bp.ixy»),«TAB» -(«bp.ixz»)];
             [-(«bp.ixy»),«TAB»  «bp.iy»,«TAB» -(«bp.iyz»)];
             [-(«bp.ixz»),«TAB»-(«bp.iyz»),«TAB»   «bp.iz»]];

        «FOR l : robot.links»
            I_«l.name» = ...
                [[ «l.inertiaParams.ix»,«TAB»-(«l.inertiaParams.ixy»),«TAB»-(«l.inertiaParams.ixz»)];
                 [-(«l.inertiaParams.ixy»),«TAB»  «l.inertiaParams.iy»,«TAB»-(«l.inertiaParams.iyz»)];
                 [-(«l.inertiaParams.ixz»),«TAB»-(«l.inertiaParams.iyz»),«TAB»  «l.inertiaParams.iz»]];

        «ENDFOR»

        % Now the same inertia parameters expressed in the link frame (may be equal or not to
        %  the previous ones, depending on the robot model description)
        «val bp_lf = common.getLinkFrameInertiaParams(robot.base)»
        I_«robot.base.name»_lf = ...
            [[  «bp_lf.ix»,«TAB»-(«bp_lf.ixy»),«TAB»-(«bp_lf.ixz»)];
             [-(«bp_lf.ixy»),«TAB»  «bp_lf.iy» ,«TAB»-(«bp_lf.iyz»)];
             [-(«bp_lf.ixz»),«TAB»-(«bp_lf.iyz»),«TAB»  «bp_lf.iz»]];

        «FOR l : robot.links»
            «val params = common.getLinkFrameInertiaParams(l)»
            I_«l.name»_lf = ...
                [[  «params.ix»,«TAB»-(«params.ixy»),«TAB»-(«params.ixz»)];
                 [-(«params.ixy»),«TAB»  «params.iy» ,«TAB»-(«params.iyz»)];
                 [-(«params.ixz»),«TAB»-(«params.iyz»),«TAB»  «params.iz»]];

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

    /* Get the parameters in a frame centered in the COM, since Featherstone's model
       wants the inertia tensor expressed in such a frame
    */
    def private inertiaParams(AbstractLink l) '''
        «val inertia_lf = l.linkFrameInertiaParams»
        «val com_lf = l.inertiaParams.com»
        «val inertia_com = Utilities::rototranslate(inertia_lf,
            com_lf.x.asFloat, com_lf.y.asFloat, com_lf.z.asFloat, 0,0,0, false)»
        I{«l.ID»} = mcI(«inertia_lf.mass», [«inertia_lf.com.listCoordinates»], ...
        «inertiaTensor(inertia_com)» );
    '''
    def private inertiaTensor(InertiaParams ip)  '''
        [ [ «ip.ix»    -(«ip.ixy») -(«ip.ixz»)]; ...
          [-(«ip.ixy»)   «ip.iy»   -(«ip.iyz»)]; ...
          [-(«ip.ixz») -(«ip.iyz»)   «ip.iz»] ] ...
    '''

}