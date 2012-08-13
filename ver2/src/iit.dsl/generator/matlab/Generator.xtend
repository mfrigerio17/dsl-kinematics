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

    private String TAB = "\t"

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        //fsa.generateFile(robot.name.toLowerCase() + "_inertia.m", inertiaParams(robot))
        //fsa.generateFile(robot.name.toLowerCase() + "_feath_model.m", featherstoneMatlabModel(robot))

        //generateJacobiansFiles(robot, fsa)
        //generateTransformsFiles(robot, fsa);
        generatePlotFramesFile(robot, fsa)
    }


    def generatePlotFramesFile(Robot robot, IFileSystemAccess fsa) {
        val PlotFrames gen = new PlotFrames(robot)
        fsa.generateFile(robot.name.toLowerCase() + "_plot_frames.m"  , gen.plotFramesCode())
    }

    def generateTransformsFiles(Robot robot, IFileSystemAccess fsa) {
        transGen = new Transforms(robot)
        fsa.generateFile(robot.name.toLowerCase() + "_init_homogeneous.m"  , transGen.homogeneous_init_fileContent(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_update_homogeneous.m", transGen.homogeneous_update_fileContent(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_init_6Dmotion.m"  , transGen.motion6D_init_fileContent(robot))
        fsa.generateFile(robot.name.toLowerCase() + "_update_6Dmotion.m", transGen.motion6D_update_fileContent(robot))
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

        % Inertia parameters as written in the .kindsl model file
        inertia_«robot.base.name».tensor = ...
            «tensor(bp)»;
        inertia_«robot.base.name».com = «com(bp)»;

        «FOR l : robot.links»
            inertia_«l.name».tensor = ...
                «tensor(l.inertiaParams)»;
            inertia_«l.name».com = «com(l.inertiaParams)»;

        «ENDFOR»

        % Now the same inertia parameters expressed in the link frame (may be equal or not to
        %  the previous ones, depending on the model file)
        «val bp_lf = common.getLinkFrameInertiaParams(robot.base)»
        inertia_lf_«robot.base.name».tensor = ...
             «tensor(bp_lf)»;
        inertia_lf_«robot.base.name».com = «com(bp_lf)»;

        «FOR l : robot.links»
            «val params = common.getLinkFrameInertiaParams(l)»
            inertia_lf_«l.name».tensor = ...
                «tensor(params)»;
            inertia_lf_«l.name».com = «com(params)»;

        «ENDFOR»

        % Same inertial properties expressed in a frame with origin in the COM of the link
        %  oriented as the default link-frame (the COM coordinates in such a frame should
        %  always be [0,0,0] ).
        «val bp_com = Utilities::rototranslate(bp,
                       bp.com.x.asFloat, bp.com.y.asFloat, bp.com.z.asFloat, 0,0,0, false)»
        inertia_com_«robot.base.name».tensor = ...
             «tensor(bp_com)»;
        inertia_com_«robot.base.name».com = «com(bp_com)»;

        «FOR l : robot.links»
            «val params = l.inertiaParams»
            «val params_com = Utilities::rototranslate(params, params.com.x.asFloat,
                params.com.y.asFloat, params.com.z.asFloat, 0,0,0, false)»
            inertia_com_«l.name».tensor = ...
                «tensor(params_com)»;
            inertia_com_«l.name».com = «com(params_com)»;

        «ENDFOR»
    '''
    def private tensor(InertiaParams params) '''
         [[  «params.ix»,«TAB»-(«params.ixy»),«TAB»-(«params.ixz»)];
          [-(«params.ixy»),«TAB»  «params.iy» ,«TAB»-(«params.iyz»)];
          [-(«params.ixz»),«TAB»-(«params.iyz»),«TAB»  «params.iz»]]'''
    def private com(InertiaParams params) '''
         [«params.com.x.str»; «params.com.y.str»; «params.com.z.str»]'''

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
            «structName».pitch(«j.ID») = «jointPitch(j)»;
        «ENDFOR»

        «FOR j : robot.joints»
            «structName».Xtree{«j.ID»} = «jointTransform(j)»;
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