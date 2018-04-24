package iit.dsl.generator.esrocos

import java.util.Set
import java.util.HashSet
import java.util.Objects
import java.util.ArrayList
import java.util.List
import java.util.Map
import java.util.HashMap
import java.util.Comparator
import java.util.TreeSet

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.transspecs.transSpecs.DesiredTransforms

import iit.dsl.generator.Common
import iit.dsl.generator.common.TreeUtils
import iit.dsl.generator.common.JacUtils
import iit.dsl.generator.common.gr.RobotFrameUtils
import iit.dsl.generator.common.gr.RobotPoseUtils
import iit.dsl.generator.common.gr.RelativePose
import iit.dsl.generator.common.gr.FrameRelationKind
import iit.dsl.generator.common.gr.RobotPoseUtils.ConstantPose
import iit.dsl.generator.common.gr.RobotPoseUtils.JointSuccessorPose
import iit.dsl.generator.common.gr.PoseCompose
import iit.dsl.generator.common.gr.AttachedFrame
import iit.dsl.generator.common.gr.RobotPoseUtils.PoseInfo


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


class QueryProcessor
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

    def bam() {
        val jutils = new JacUtils(treeUtils, poseUtils)
        val reference = RobotFrameUtils::getFrameByName(robot, "fr_base0")
        val target    = RobotFrameUtils::getFrameByName(robot, "ee")
        val jph  = jutils.makeJacPlaceholder(target, reference)
//        for(j : jph.jchain) {
//            System.out.println(j.getName())
//        }
        val jposes = jutils.getRequiredPoses( jph )
//        for( pose : jposes ) {
//            System.out.println( pose.target.frame.name + " wrt " + pose.reference.frame.name )
//        }
        val info = new ArrayList<RobotPoseUtils$PoseInfo>
        for( rp : jposes ) {
            info.add( poseUtils.buildPoseInfo( rp ) )
        }

        loadPoses( info )
    }

    def public printMapByReference()
    {
        for( k : outputByRef.keySet ) {
            System.out.println("reference: " + k)
            for( poseinfo : outputByRef.get(k) ) {
                System.out.println( poseinfo.pose.target + " (" + poseinfo.framesPath.length + ")" )
            }
            System.out.println();
        }
    }


    def public loadPoses(List<RobotPoseUtils$PoseInfo> request)
    {
        populateMapByReference( request )
        addModelConstantPoses( request )
        addParentChildComposes( request )
        addAllComposes( request )
    }

    def public load(DesiredTransforms query)
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

            val rp = new RelativePose(target, reference)
            outputs.add( rp )

            val info = poseUtils.buildPoseInfo( rp )
            infoList.add( info )
        }
        loadPoses( infoList )
    }


    def private addModelConstantPoses(List<RobotPoseUtils$PoseInfo> desires)
    {
        for( info : desires)
        {
            // Find all the distance-1 frame pairs which will be referenced
            // afterwards in the compositions.
            val path = info.framesPath
            if( path.length > 0)
            {
                val vertices = path.vertexList     // these are AttachedFrame(s)
                val vi1 = vertices.listIterator(0)
                val vi2 = vertices.listIterator(1)
                val ei  = path.edgeList.iterator // can be used to tag the pose with its kind (geometrical constant of joint dependent)
                do {
                    val v1a = vi1.next  // a AttachedFrame
                    val v2a = vi2.next
                    ////System.out.println( v1a.frame().name() + " " + v2a.frame().name() )
                    val poseKind = ei.next.getKind()
                    if( notThereYet(v1a, v2a) )
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
    }



    def private addAllComposes(List<RobotPoseUtils$PoseInfo> request)
    {
        for( reference : outputByRef.keySet ) {
            var RelativePose last = null
            val pinfos = outputByRef.get( reference )
            for( pinfo : pinfos ) {
                if( last === null) {
                    iterativeCompose( pinfo.pose, pinfo )
                    last = pinfo.pose
                } else {
                    val newpose = new RelativePose( pinfo.pose.target, last.target)
                    val newinfo = poseUtils.buildPoseInfo(newpose)
                    iterativeCompose( newpose, newinfo)
                    val comp = addCompose(newpose, last)
                    last = comp.result
                }
            }
        }
    }

    def private iterativeCompose(RelativePose rpose, RobotPoseUtils$PoseInfo info)
    {
        val chain = info.longestLinkToLink()
        // The actual target and reference AttachedFrame s
        val refF  = info.pose.reference
        val tgtF  = info.pose.target
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


    def private addCompose(RelativePose p1, RelativePose p2)
    {
        val comp = new PoseCompose(p1, p2)
        this.composes.add( comp )
        return comp
    }


    private static class PoseLengthComparator implements Comparator<RobotPoseUtils$PoseInfo>
    {
        override compare(PoseInfo o1, PoseInfo o2) {
            return Integer.compare(o1.framesPath.length, o2.framesPath.length)
        }
    }
    def private populateMapByReference(List<RobotPoseUtils$PoseInfo> request)
    {
        for( i : request ) {
            val targets = outputByRef.get( i.pose.reference )
            if( targets === null ) {
                outputByRef.put( i.pose.reference, new TreeSet<PoseInfo>(new PoseLengthComparator) )
            }
        }
        for( i : request ) {
            outputByRef.get( i.pose.reference ) .add( i )
        }
    }

    def public getConstantPoses() { return this.constPoses }
    def public getJointPoses()    { return this.jointPoses }
    def public getComposes()      { return this.composes   }
    def public getOutputs()       { return this.outputs    }

    def public treeUtils() { return this.treeUtils }
    def public poseUtils() { return this.poseUtils }


    def cpp_header_robot_defs() '''
    #ifndef EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H
    #define EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H

    #include <Eigen/Core>

    typedef Eigen::Matrix<double, «robot.joints.length», 1> joint_state;

    #endif
    '''

    def private addParentChildComposes(List<RobotPoseUtils$PoseInfo> desires)
    {
        // Enumerate all the 1-step-compose in the form parent-to-child (or vice versa)
        // required by the query. Avoid duplicates.
        for( info : desires )
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
                        parentChildCompose(target, reference)
                    }
                } while(iter2.hasNext())
            }
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

        addCompose( new RelativePose(tFr,jFr), new RelativePose(jFr,rFr) )
    }

    def private reset() {
        framePairs.clear()
        constPoses.clear()
        jointPoses.clear()
        composes.clear()
        outputs.clear()
        outputByRef.clear()
    }

    def private notThereYet(AbstractLink target, AbstractLink reference) {
        return notThereYet( target.frameName.toString, reference.frameName.toString )
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

    private Map<AttachedFrame, Set<RobotPoseUtils$PoseInfo> > outputByRef =
                              new HashMap<AttachedFrame, Set<RobotPoseUtils$PoseInfo> >()
    private extension Common helper = Common.getInstance()

}

