package iit.dsl.generator.common.gr

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.Joint

import iit.dsl.generator.common.gr.AttachedFrame.FrameRole

import org.jgrapht.Graph
import org.jgrapht.graph.SimpleGraph


/**
 * A coordinate frame.
 *
 * A pure placeholder, just a named entity. Use it if you can instead of the
 * grammar-generated RefFrame type, which is really a container for roto-translation
 * parameters and thus it is not very neat.
 */
public class Frame
{
    public new (String n) { name = n }

    def public name() {
        return name
    }
    def override public equals(Object rhs) {
        if( ! (rhs instanceof Frame) ) return false
        return name.equals((rhs as Frame).name)
    }
    def override public hashCode() {
        return 17*name.hashCode
    }

    private final String name
}

/**
 * A coordinate frame attached to a link of some mechanism.
 *
 * In fact, this class models the attachment relation between a frame and a link.
 * However, instances will be typically used to represent the frames themselves,
 * augmented with attachment information.
 *
 * The frame role does *not* affect equality checks (equals() and hashCode()),
 * as there shall not be multiple frames with the same name on the same link,
 * regardless of the role.
 */
public class AttachedFrame
{
    /**
     * Possible roles a reference frame attached to a link might have.
     */
    public enum FrameRole
    {
        link,  ///< the default frame on the link; all the other ones are defined wrt to it
        joint, ///< the frame of a joint supported by the link
        user   ///< an additional, user-defined reference frame
    }

    public new (Frame f, AbstractLink c, FrameRole r)
    {
        frame   = f
        carrier = c
        frole   = r
    }
    def public frame()  { return frame }
    def public carrier(){ return carrier }
    def public role()   { return frole }

    override public equals(Object rhs) {
        if( ! (rhs instanceof AttachedFrame) ) return false
        return frame  .equals((rhs as AttachedFrame).frame) &&
               carrier.equals((rhs as AttachedFrame).carrier)
    }
    override public hashCode() {
        return frame.hashCode + 31*carrier.hashCode
    }

    private final Frame frame
    private final AbstractLink carrier
    private final FrameRole frole
}



/**
 * The possible relations between a link-frame and any other proximal neighbor
 * frame.
 *
 * A proximal neighbor frame is a frame at distance 1 from the current reference.
 * Distance 1 means that the relative pose between the two frames is a model
 * parameter (and not a composition of multiple relative poses).
 * The set of all possible frames at distance 1 from a link-frame contains:
 * custom user-defined frames (if any), the frame of the joints supported by the
 * link (every link but the leafs), the frame of the joint supporting the link
 * (every link but the base). The last is the only case of a proximal frame
 * which is attached to a different link.
 */
public enum FrameRelationKind
{
    user,             ///<  between a custom user frame and the link frame
    jointPredecessor, ///<  between a joint frame and the frame of the link supporting it
    jointSuccessor    ///<  between a joint frame and the frame of the link supported by the joint
    // Note that the first two types correspond to geometric constants
    // of the robot, while the last one has to do with the degree of
    // freedom of the joints
}


class RobotFrameUtils
{
    /**
     * The AttachedFrame instance modeling the attachment of the link-frame to
     * its own link.
     */
    def public static AttachedFrame getLinkFrame(AbstractLink link) {
        return new AttachedFrame( new Frame(link.frameName.toString), link, AttachedFrame$FrameRole::link)
    }
    /**
     *
     */
    def public static AttachedFrame getJointFrame(Joint joint, AbstractLink carrier) {
        return new AttachedFrame( new Frame(joint.frameName.toString), carrier, AttachedFrame$FrameRole::joint )
    }
    /**
     * The AttachedFrame modeling the attachment of the given frame (identified
     * by the name) to the given link.
     *
     * @return ..., null if no frame with the given name is found on the given
     * link.
     */
    def public static AttachedFrame getFrameByName(AbstractLink link, String frameName)
    {
        var AttachedFrame ret = null

        // Is it the default link frame?
        val frameLinkName = link.frameName.toString()
        if(frameName.equals(frameLinkName)) {
            ret = new AttachedFrame( new Frame(frameName), link, AttachedFrame$FrameRole::link )
        }
        // Is it the joint frame ?
        val iter = link.childrenList.children.iterator
        while( iter.hasNext && ret===null ) {
            if(frameName.equals(iter.next.joint.frameName.toString)) { // do not forget the toString() !
                ret = new AttachedFrame( new Frame(frameName), link, AttachedFrame$FrameRole::joint )
            }
        }
        // Is it a user-defined frame ?
        for(RefFrame f : link.frames) {
            if(f.name.equals(frameName)) {
                return new AttachedFrame( new Frame(frameName), link, AttachedFrame$FrameRole::user)
            }
        }
        return ret
    }

    /**
     * The AttachedFrame representing the attachment of a frame, identified by
     * the given name, on a link of the given robot.
     *
     * @return ..., null if no frame with the given name is attached to the robot
     */
    def public static AttachedFrame getFrameByName(Robot robot, String frameName)
    {
        var AttachedFrame ret = null

        var links = robot.linksAndBase.iterator
        while( links.hasNext && ret===null) {
            ret = RobotFrameUtils.getFrameByName(links.next, frameName)
        }

        return ret
    }

    /**
     * Fetches the robot joint associated with the given frame, or null if such
     * a frame is not a Joint frame.
     */
    def public static Joint getJointByFrame(Robot robot, AttachedFrame jointFr)
    {
        if( jointFr.role != FrameRole.joint ) return null
        return getJointByName( jointFr.carrier, getItemNameByFrameName(jointFr.frame.name) )
    }

    public static class FramesGraphEdge
    {
        public new(FrameRelationKind k) { kind = k }
        def public getKind()            { return kind }

        private final FrameRelationKind kind
    }

    /**
     * Constructs a graph whose vertices are all the frames attached
     * to the links of the given robot.
     */
    def public static Graph<AttachedFrame, FramesGraphEdge> makeFramesGraph(Robot robot)
    {
        val graph = new SimpleGraph(FramesGraphEdge)
        populate(graph, robot, null, robot.base)
        return graph
    }
    def private static void populate(
        Graph<AttachedFrame, FramesGraphEdge> g,
        Robot robot,
        AttachedFrame jointFr,
        AbstractLink current)
    {
        val linkf = RobotFrameUtils::getLinkFrame(current)
        g.addVertex( linkf )

        if(jointFr !== null) {
            g.addEdge(linkf, jointFr, new FramesGraphEdge(FrameRelationKind::jointSuccessor) )
        }

        for( f : current.frames )
        {
            val userf = new AttachedFrame( new Frame(f.name), current, AttachedFrame$FrameRole::user)
            g.addVertex( userf )
            g.addEdge(linkf, userf, new FramesGraphEdge(FrameRelationKind::user) )
        }
        for(c : current.childrenList.children)
        {
            val jointf = new AttachedFrame(new Frame(c.joint.frameName.toString()), current, AttachedFrame$FrameRole::joint)
            g.addVertex( jointf )
            g.addEdge( linkf, jointf,
                new FramesGraphEdge(FrameRelationKind::jointPredecessor) )
            populate(g, robot, jointf, c.link)
        }
    }

    private static extension iit.dsl.generator.Common common = iit.dsl.generator.Common.getInstance()
}


