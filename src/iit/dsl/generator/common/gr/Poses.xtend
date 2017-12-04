package iit.dsl.generator.common.gr

import iit.dsl.transspecs.transSpecs.FramePair

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RotoTrasl

import iit.dsl.generator.common.gr.AttachedFrame.FrameRole
import iit.dsl.generator.common.TreeUtils
import iit.dsl.generator.common.gr.RobotFrameUtils.FramesGraphEdge
import iit.dsl.generator.Common
import iit.dsl.generator.Utilities
import iit.dsl.generator.common.Vector3D


import java.util.List
import iit.dsl.kinDsl.Joint

/**
 * A placeholder for the relative pose of two reference frames.
 *
 * The semantic of this class is the relative pose of the 'target' frame with
 * respect to the 'reference' frame.
 */
class RelativePose
{
    public new(AttachedFrame tgt, AttachedFrame ref) {
        target    = tgt
        reference = ref
    }

    def public target()    { return target }
    def public reference() { return reference }
    def public name()      { return target.frame.name + "__" + reference.frame.name }

    override public equals(Object rhs) {
        if( ! (rhs instanceof RelativePose) ) return false // accounts for rhs being null
        return target   .equals((rhs as RelativePose).target) &&
               reference.equals((rhs as RelativePose).reference)
    }
    override public hashCode() {
        return target.hashCode() + 31*reference.hashCode()
    }

    private final AttachedFrame target
    private final AttachedFrame reference
}

/**
 * A placeholder for the semantic operation of relative-pose composition
 */
class PoseCompose
{
    public new(RelativePose arg1, RelativePose arg2) {
        this.arg1 = arg1
        this.arg2 = arg2
        this.res = new RelativePose(arg1.target(), arg2.reference())
    }

    def public arg1() { return arg1 }
    def public arg2() { return arg2 }
    def public result() { return res }

    private RelativePose arg1
    private RelativePose arg2
    private RelativePose res
}



class DTDSLPoseUtils
{
    def public static RelativePose convert(FramePair pair, Robot robot)
    {
        val reference = RobotFrameUtils::getFrameByName(robot, pair.base.name)
        val target    = RobotFrameUtils::getFrameByName(robot, pair.target.name)
        if( reference===null || target===null) return null
        return new RelativePose(target, reference)
    }
}







class RobotPoseUtils
{
    public new(Robot rob) {
        robot = rob
        tree  = new TreeUtils(rob)
    }

    def public getConstantPose(AttachedFrame target, AttachedFrame ref)
    {
        var RotoTrasl rototrasl = null
        if( target.role == FrameRole.joint )
        {
            val j = RobotFrameUtils.getJointByFrame( robot, target )
            rototrasl = j.refFrame // this is really roto-translation parameters, not a reference frame; my unfortunate name choice in the grammar
        }else if( ref.role == FrameRole.joint )
        {
            val j = RobotFrameUtils.getJointByFrame( robot, ref )
            rototrasl = j.refFrame
        }
        if( target.role == FrameRole.user ) {
            val robotFrame = getFrameByName( target.carrier, target.frame.name )
            rototrasl = robotFrame.transform
        } else if( ref.role == FrameRole.user ) {
            val robotFrame = getFrameByName( ref.carrier, ref.frame.name )
            rototrasl = robotFrame.transform
        }
        if(rototrasl === null) {
            throw new RuntimeException(target.frame.name + " wrt " + ref.frame.name + " does not seem to be a model constant")
        }
        return new ConstantPose(target, ref, rototrasl)
    }

    /**
     *
     */
    def public getJointSuccessorPose(AttachedFrame target, AttachedFrame ref)
    {
        var Joint joint = null
        if( ref.role == FrameRole.joint ) {
            joint = RobotFrameUtils.getJointByFrame(robot, ref)
        } else {
            joint = RobotFrameUtils.getJointByFrame(robot, target)
        }
        if(joint=== null) {
            throw new RuntimeException(target.frame.name + " wrt " + ref.frame.name + " does not seem to be a joint-successor pose")
        }

        return new JointSuccessorPose(target, ref, joint)
    }

    def public buildPoseInfo(FramePair pose)
    {
        return buildPoseInfo( DTDSLPoseUtils::convert(pose, robot))
    }

    /**
     *
     */
    def public buildPoseInfo(RelativePose pose)
    {
        if( pose===null ) {
            return null
        }

        val target    = pose.target
        val reference = pose.reference

        val chain = TreeUtils.buildChain(target.carrier, reference.carrier)

        // Find out whether one end of the chain is involved only because of
        // a joint frame and not because of the link frame. Note that ancestorOf()
        // is true only for proper ancestors, so it implies a chain which is
        // at least two link long.
        //
        var int start = 0
        var int end   = chain.length
        if( (target.role == FrameRole::joint) &&
                tree.ancestorOf(target.carrier, reference.carrier)) {
            start = 1
        } else if( (reference.role == FrameRole::joint) &&
                tree.ancestorOf(reference.carrier, target.carrier)) {
            end = chain.length-1
        }
        val subchain = chain.subList(start, end)

        // Find the path from the target frame to the reference frame
        val framesGraph= RobotFrameUtils::makeFramesGraph(robot)
        val framePath  = org.jgrapht.alg.shortestpath.DijkstraShortestPath::
                      findPathBetween(framesGraph, target, reference)

        return new PoseInfo(pose, chain, subchain, framePath)
    }

    private final Robot     robot
    private final TreeUtils tree



    static class PoseInfo
    {
        public new(RelativePose p,
                   List<AbstractLink> linkChain,
                   List<AbstractLink> sublist,
                   org.jgrapht.GraphPath<AttachedFrame, FramesGraphEdge> path)
        {
            pose = p
            inducedLinkChain = linkChain
            longestLinkToLink= sublist
            framesPath       = path
            targetIsLink    = ( pose.   target.role == FrameRole::link )
            referenceIsLink = ( pose.reference.role == FrameRole::link )
        }

        def public pose()    { return pose }
        def public inducedLinkChain() { return inducedLinkChain }
        /**
         * The longest kinematic chain whose both first and last link must be
         * traversed completely to travel between the target and the reference
         */
        def public longestLinkToLink(){ return longestLinkToLink }
        /**
         * A path of the frames graph, from target to reference
         */
        def public org.jgrapht.GraphPath<AttachedFrame, FramesGraphEdge>
                  framesPath() { return framesPath }
        def public targetIsLinkFrame()    { return targetIsLink }
        def public referenceIsLinkFrame() { return referenceIsLink }

        private final RelativePose pose
        private final List<AbstractLink> inducedLinkChain
        private final List<AbstractLink> longestLinkToLink
        private final org.jgrapht.GraphPath<AttachedFrame, FramesGraphEdge> framesPath
        private boolean targetIsLink    = true
        private boolean referenceIsLink = true
    }

    /**
     * A constant relative pose of a robot model.
     *
     * Constant poses are a representation of the geometrical properties of a
     * robot. Constant poses relate any joint-frame with the frame of the link
     * supporting the joint, as well as any user-frame and the frame of
     * the link the user-frame is attached to.
     *
     * TODO actually, the relative pose between joint frame and user frame, or
     * between two user frames, is also constant. But those are already the
     * composition of two poses, and things currently would not work in those
     * cases
     */
    static class ConstantPose extends RelativePose
    {
        public new(AttachedFrame tgt, AttachedFrame ref, RotoTrasl params)
        {
            super(tgt, ref)
            if( ! tgt.carrier.equals(ref.carrier)) {
                throw new RuntimeException("A constant relative pose exists only between frames on the same link")
            }
            if( ! ((ref.role == FrameRole.link) || (tgt.role == FrameRole.link)) ) {
                throw new RuntimeException("A constant relative pose must involve a link-frame")
            }

            refToTarget = Vector3D.convert( params.translation )

            if( ref.role == FrameRole.link ) {
                ref_R_target = Utilities.original_X_rotated(
                                       utils.asFloat(params.rotation.x),
                                       utils.asFloat(params.rotation.y),
                                       utils.asFloat(params.rotation.z) )
            } else {
                ref_R_target = Utilities.rotated_X_original(
                                              utils.asFloat(params.rotation.x),
                                              utils.asFloat(params.rotation.y),
                                              utils.asFloat(params.rotation.z))
                val newvec = Utilities.matrix3x3Mult(ref_R_target, refToTarget)
                refToTarget.x = -newvec.x
                refToTarget.y = -newvec.y
                refToTarget.z = -newvec.z
            }
        }
        /**
         * The rotation matrix in the form reference_R_target.
         * This is the matrix that rotates coordinates in the target frame, and
         * return coordinates in the reference frame.
         */
        def public ref_R_target() { return ref_R_target }

        public final Vector3D refToTarget ///! the position vector from the reference to the target
        private double[][] ref_R_target = #[#[0, 0, 0],
                                            #[0, 0, 0],
                                            #[0, 0, 0]]
    }

    static class JointSuccessorPose extends RelativePose
    {
        public new(AttachedFrame tgt, AttachedFrame ref, Joint j)
        {
            super(tgt, ref)
            if( tgt.carrier.equals(ref.carrier)) {
                throw new RuntimeException("A joint-successor pose must involve two different, subsequent links")
            }
            val refrole = ref.role
            val tgtrole = tgt.role
            val cond = (refrole==FrameRole.link && tgtrole==FrameRole.joint) || (refrole==FrameRole.joint && tgtrole==FrameRole.link)
            if( ! cond ) {
                throw new RuntimeException("A joint-successor pose must involve a link-frame and a joint-frame")
            }
            jointFrameIsTarget = (tgtrole == FrameRole.joint)

            joint = j
        }

        public final Joint joint
        public final boolean jointFrameIsTarget
    }

    private static extension Common utils = Common.getInstance()
}





