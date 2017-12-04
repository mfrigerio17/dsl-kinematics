package iit.dsl.generator.esrocos

import java.util.Set
import java.util.HashSet
import java.util.Objects
import java.util.ArrayList
import java.util.List

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.transspecs.transSpecs.DesiredTransforms

import iit.dsl.generator.Common
import iit.dsl.generator.common.TreeUtils
import iit.dsl.generator.common.gr.Frame
import iit.dsl.generator.common.gr.RobotFrameUtils
import iit.dsl.generator.common.gr.RobotPoseUtils
import iit.dsl.generator.common.gr.RelativePose
import iit.dsl.generator.common.gr.FrameRelationKind
import iit.dsl.generator.common.gr.RobotPoseUtils.ConstantPose
import iit.dsl.generator.common.gr.RobotPoseUtils.JointSuccessorPose
import iit.dsl.generator.common.gr.PoseCompose
import iit.dsl.generator.common.gr.AttachedFrame

class FrameNamePair
{
    public new (String targ, String ref) {
        target    = targ
        reference = ref
    }
    def public getTarget()   { return target }
    def public getReference(){ return reference }

    override public int hashCode() {
        return Objects.hash(target, reference)
    }
    override public boolean equals(Object rhs) {
        if( ! (rhs instanceof FrameNamePair) ) {
            return false
        } else {
            val rh = rhs as FrameNamePair;
            return target.equals(rh.target) &&
                   reference.equals(rh.reference)
        }
    }

    String target;
    String reference;
}


class Generator
{
    public new(Robot rob) {
        robot     = rob
        treeUtils = new TreeUtils(robot)
        poseUtils = new RobotPoseUtils(robot)
    }

    def boom() {
//        val g = RobotFrameUtils::makeFramesGraph(robot)
//        for( v : g.vertexSet() ) {
//            System.out.println(v.frame.name);
//        }
        val l1 = robot.getLinkByName("link5")
        val l2 = robot.getLinkByName("link5")
        val lca = treeUtils.lowestCommonAncestor(l1 , l2)
        System.out.println( lca.name )
        val lca2 = TreeUtils::commonAncestor(l1, l2)
        System.out.println( lca2.name )
    }

    def load(DesiredTransforms query)
    {
        reset()

        val infoList = new ArrayList<RobotPoseUtils$PoseInfo>()

        // Fetch the required information associated with each request in the
        // query, for subsequent use.
        for(pair : query.transforms.specs)
        {
            // Build the placeholders for the reference and target frame
            //
            val target    = RobotFrameUtils::getFrameByName(robot, pair.target.name)
            val reference = RobotFrameUtils::getFrameByName(robot, pair.base.name)
            if( target===null ) {
                throw new RuntimeException("Fatal: reference frame "+ pair.target.name +" not found on the given robot")
            }
            if( reference===null) {
                throw new RuntimeException("Fatal: reference frame " + pair.base.name +" not found on the given robot")
            }

            val info = poseUtils.buildPoseInfo(new RelativePose(target, reference))
            infoList.add( info )

            // Enumerate, in the text, all the distance-1 frame pairs which will
            // be referenced afterwards in the compositions. The interpreter of
            // the text we generate here will have to resolve these poses from
            // the model.
            val path = info.framesPath
            if( path.length > 0)
            {
                val vertices = path.vertexList     // these are AttachedFrame(s)
                val vi1 = vertices.listIterator(0)
                val vi2 = vertices.listIterator(1)
                val ei  = path.edgeList.iterator // can be used to tag the pose with its kind (geometrical constant of joint dependent)
                do {
                    val v1a= vi1.next  // a AttachedFrame
                    val v2a= vi2.next
                    ////System.out.println( v1a.frame().name() + " " + v2a.frame().name() )
                    val poseKind = ei.next.getKind()
                    if( notThereYet(v1a.frame, v2a.frame) )
                    {
                        if( poseKind == FrameRelationKind.jointPredecessor ||
                            poseKind == FrameRelationKind.user )
                        {
                            // in this case the relative pose is a constant
                            constPoses.add( poseUtils.getConstantPose(v1a, v2a) )
                        } else {
                            // poseKind must be FrameRelationKind.jointSuccessor
                            jointPoses.add( poseUtils.getJointSuccessorPose(v1a, v2a) )
                        }
                    }
                } while(vi2.hasNext)
            }
        }

        // Enumerate all the 1-step-compose in the form parent-to-child (or vice versa)
        // required by the query. Avoid duplicates.
        for( info : infoList )
        {
            val chain = info.longestLinkToLink()
            if(chain.length > 1)
            {
                val iter1 = chain.listIterator(0)
                val iter2 = chain.listIterator(1)
                do {
                    val target   = iter1.next
                    val reference= iter2.next
                    if( notThereYet(target, reference) ) {
                        ////System.out.println( target.name + " " + reference.name)
                        composes.add( parentChildCompose(target, reference) )
                    }
                } while(iter2.hasNext())
            }
        }

        // Now generate the compositions along the chain, required to end up
        // with the actual, desired relative pose. If the kinematic chain has 2
        // or fewer links, no further compositions are necessary, as those have
        // been considered in the previous steps
        //
        // Again, avoid the repetitions arising from overlapping
        // relative poses (e.g. in A-B-C, A <- B and A <- C, the latter requires
        // the first, but I should not generate the first twice)
        for( info : infoList )
        {
            val chain = info.longestLinkToLink()
            // The actual target and reference AttachedFrame s
            val refF  = info.pose.reference
            val tgtF  = info.pose.target
            outputs.add( new RelativePose(tgtF, refF) )
            // These two are the link-frame of the links at both ends of the chain.
            // They may or may not be the same as the actual target and reference.
            val startF = RobotFrameUtils.getLinkFrame( chain.get(0) )
            val endF   = RobotFrameUtils.getLinkFrame( chain.last )
            if(chain.length > 2)
            {
                val iter2 = chain.listIterator(1) // start from the second ...
                val iter3 = chain.listIterator(2) // ... and the third. The first one is 'startF'
                do {
                   val fr2 = RobotFrameUtils.getLinkFrame( iter2.next() )
                   val fr3 = RobotFrameUtils.getLinkFrame( iter3.next() )
                   if( notThereYet(startF, fr3) )
                   {
                       addCompose(new RelativePose(startF,fr2),
                                  new RelativePose(fr2  ,fr3) )
                    }
                } while(iter3.hasNext())
            }

            // Now check whether the actual target/reference frame is the
            // default link-frame on the corresponding link. If not, it means
            // that the target/reference frame is another, custom frame supported
            // by the same link, OR a joint frame on another, adjacent, link.
            // In both cases, and one more composition has to be generated.
            if( ! info.referenceIsLinkFrame && notThereYet(startF, refF) )
            {
                // here, endF != refF (so 'end' is not really the end...)
                addCompose( new RelativePose(startF, endF),
                            new RelativePose(endF, refF) )
            }
            if( ! info.targetIsLinkFrame && notThereYet(tgtF, refF) )
            {
                // here, startF != refF
                addCompose( new RelativePose(tgtF, startF),
                            new RelativePose(startF, refF) )
            }
        }
    }

    def private addCompose(RelativePose p1, RelativePose p2) {
        this.composes.add( new PoseCompose(p1, p2))
    }

    def public getConstantPoses() { return this.constPoses }
    def public getJointPoses()    { return this.jointPoses }
    def public getOutputs()       { return this.outputs    }


    def cpp_header_robot_defs() '''
    #ifndef EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H
    #define EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H

    #include <Eigen/Core>

    typedef Eigen::Matrix<double, «robot.joints.length», 1> joint_state;

    #endif
    '''

    def lua_ilk()
    '''
    return {
        { op='model_const', args={ «FOR kk:constPoses SEPARATOR","»'«kk.name»'«ENDFOR»} },
        «FOR c : jointPoses SEPARATOR"," AFTER","»
            { op='model_T_joint_local', name='«c.name()»', jtype='«c.joint.typeString»', dir='«directionTag(c)»', input=«c.joint.arrayIdx» }
        «ENDFOR»
        «FOR c : composes SEPARATOR ","AFTER","»
            { op='compose', args={ '«c.arg1.name»', '«c.arg2.name»', '«c.result.name»' } }
        «ENDFOR»
        «FOR c : outputs SEPARATOR ","»
            { op='output', '«c.name»' }
        «ENDFOR»
    }
    '''

    def private directionTag(JointSuccessorPose pose) {
        val targetKind = pose.target().role()
        if( targetKind == AttachedFrame.FrameRole.link ) {
            return "a_x_b"
        } else {
            return "b_x_a"
        }
    }


    def private parentChildCompose(AbstractLink target, AbstractLink reference)
    {
        val joint = treeUtils.connectingJoint(target, reference)
        if(joint === null) {
            throw new RuntimeException("Links " + target.name + " and " +
                reference.name + " are not a parent/child pair")
        }
        var AbstractLink carrier = target // just to start; 50% probability
        if( reference.childrenList.contains(joint) ) {
            carrier = reference
        }

        val tFr = RobotFrameUtils.getLinkFrame(target)
        val rFr = RobotFrameUtils.getLinkFrame(reference)
        val jFr = RobotFrameUtils.getJointFrame(joint, carrier)

        return new PoseCompose( new RelativePose(tFr,jFr), new RelativePose(jFr,rFr) )
    }

    def private reset() {
        framePairs.clear()
    }

    def private notThereYet(AbstractLink target, AbstractLink reference) {
        return notThereYet( target.frameName.toString, reference.frameName.toString )
    }
    def private notThereYet(Frame target, Frame reference) {
        return notThereYet( target.name, reference.name)
    }
    def private notThereYet(AttachedFrame target, AttachedFrame reference) {
        return notThereYet( target.frame.name, reference.frame.name)
    }
    def private notThereYet(String target, String reference) {
        return framePairs.add( new FrameNamePair(target, reference) )
    }

    private Robot     robot
    private TreeUtils treeUtils
    private RobotPoseUtils poseUtils
    private Set<FrameNamePair> framePairs = new HashSet<FrameNamePair>()
    private List<ConstantPose> constPoses = new ArrayList<ConstantPose>()
    private List<JointSuccessorPose> jointPoses = new ArrayList<JointSuccessorPose>()
    private List<PoseCompose> composes = new ArrayList<PoseCompose>()
    private List<RelativePose> outputs = new ArrayList<RelativePose>()
    private extension Common helper = Common.getInstance()

}

