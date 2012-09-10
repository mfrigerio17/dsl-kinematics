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
import java.util.List

class Generator implements IGenerator {
    extension Common common = new Common()

    TransSpecsAccessor desiredTrasformsAccessor = new TransSpecsAccessor()
    Jacobians  jacGen   = new Jacobians()
    Jsim       jsimGen  = new Jsim()
    Transforms transGen = null

    private String TAB = "\t"

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robot.name.toLowerCase() + "_inertia.m", inertiaParams(robot))
//        fsa.generateFile(robot.name.toLowerCase() + "_feath_model.m", featherstoneMatlabModel(robot))
//
//        generateJacobiansFiles(robot, fsa)
//        generateTransformsFiles(robot, fsa);
//        generatePlotFramesFile(robot, fsa)
//        generateJSIMFiles(robot, fsa);
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

//    def generateJSIMFiles(Robot robot, IFileSystemAccess fsa) {
//        fsa.generateFile(robot.name.toLowerCase() + "_init_jsim.m"  , jsimGen.jsim_init_code(robot))
//        fsa.generateFile(robot.name.toLowerCase() + "_update_jsim.m", jsimGen.jsim_update_code(robot))
//    }

    /**
     * Matlab code that creates several structs with the inertia parameters of the links
     * of the given robot.
     * Each struct has two members, the inertia tensor and the COM location (3d vector).
     * This code contains the values corresponding to the inertia parameters as contained
     * in the robot model, as well as parameters expressed in the default frame of each link
     * and in a frame centered in the COM, aligned with the link-frame. Therefore, three
     * structures are created for each rigid body of the given robot.
     */
    def inertiaParams(Robot robot) {
        val List<InertiaParams> userFrame = new ArrayList<InertiaParams>()
        val List<InertiaParams> linkFrame = new ArrayList<InertiaParams>()
        val List<InertiaParams> comFrame  = new ArrayList<InertiaParams>()
        allInertiaParams(robot, userFrame, linkFrame, comFrame)
        val userFrameIt = userFrame.iterator()
        val linkFrameIt = linkFrame.iterator()
        val comFrameIt  = comFrame.iterator()
        val allLinks = robot.abstractLinks

        return'''

        % Inertia parameters as written in the .kindsl model file
        «FOR l : allLinks»
            «val tmp = userFrameIt.next»
            «val struct = "inertia_" + l.name»
            «struct».mass = «tmp.mass»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;

        «ENDFOR»

        % Now the same inertia parameters expressed in the link frame (may be equal or not to
        %  the previous ones, depending on the model file)
        «FOR l : allLinks»
            «val tmp = linkFrameIt.next»
            «val struct = "inertia_lf_" + l.name»
            «struct».mass = «tmp.mass»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;
            block = «struct».mass*crossProductMatrix(«struct».com);
            «struct».tensor6D = [«struct».tensor, block; block', «struct».mass*eye(3)];

        «ENDFOR»

        % Same inertial properties expressed in a frame with origin in the COM of the link
        %  oriented as the default link-frame (the COM coordinates in such a frame should
        %  always be [0,0,0] ).
        «FOR l : allLinks»
            «val tmp = comFrameIt.next»
            «val struct = "inertia_com_" + l.name»
            «struct».mass = «tmp.mass»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;

        «ENDFOR»
    '''}


    def private tensor(InertiaParams params) '''
         [[  «params.ix»,«TAB»-(«params.ixy»),«TAB»-(«params.ixz»)];
          [-(«params.ixy»),«TAB»  «params.iy» ,«TAB»-(«params.iyz»)];
          [-(«params.ixz»),«TAB»-(«params.iyz»),«TAB»  «params.iz»]]'''
    def private com(InertiaParams params) '''
         [«params.com.x.str»; «params.com.y.str»; «params.com.z.str»]'''
    /*
     * Fills three lists with the inertia parameters of all the links, expressed in
     * different frames.
     * First list: parameters as written in the robot model
     * Second list: parameters expressed in the default link-frame
     * Third list: parameters expressed in a frame with origin in the COM, aligned
     *             as the default link-frame
     * All the lists are ordered as robot.abstractLinks
     */
    def private allInertiaParams(Robot rob,
        List<InertiaParams> userFrame,
        List<InertiaParams> linkFrame,
        List<InertiaParams> comFrame)
    {
        // Start with the robot base
        var InertiaParams inLinkFrame = null
        var InertiaParams inComFrame  = null
        for(l : rob.abstractLinks) {
            inLinkFrame = l.linkFrameInertiaParams
            inComFrame  = Utilities::rototranslate(inLinkFrame, inLinkFrame.com.x.asFloat,
                inLinkFrame.com.y.asFloat, inLinkFrame.com.z.asFloat, 0,0,0, false)

            userFrame.add(l.inertiaParams)
            linkFrame.add(inLinkFrame)
            comFrame.add(inComFrame)
        }
    }

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