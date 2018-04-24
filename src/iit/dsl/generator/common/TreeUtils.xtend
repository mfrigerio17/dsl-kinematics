package iit.dsl.generator.common

import java.util.List
import java.util.ArrayList
import java.util.Map
import java.util.HashMap

import org.jgrapht.graph.SimpleDirectedGraph

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.RobotBase
import iit.dsl.kinDsl.Link
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.Utilities


/**
 * Utility functions for the inspection of the kinematic tree structure.
 *
 * Currently, the class has some static methods as well as instance methods,
 * static methods are going to be deprecated.
 *
 * Instances of this class use an additional, alternative graph representation
 * of the given robot, hopefully more effective for traversal than the
 * grammar-generated EMF data structure (used by the static methods).
 */
class TreeUtils
{
    public new(Robot rob)
    {
        robot = rob
        childToParent = new SimpleDirectedGraph(GraphEdge)
        parentToChild = new SimpleDirectedGraph(GraphEdge)
        val allLinks = robot.linksAndBase.sortBy(link | getID(link))

        for( l : allLinks )
        {
            childToParent.addVertex( l )
            parentToChild.addVertex( l )

            for(childSpec : l.childrenList.children)
            {
                val child = childSpec.link
                childToParent.addVertex( child )
                parentToChild.addVertex( child )
                childToParent.addEdge( child, l, new GraphEdge(childSpec.joint, l, child ) )
                parentToChild.addEdge( l, child, new GraphEdge(childSpec.joint, l, child ) )
            }
        }
    }

    def public robot() { return this.robot }

    /**
     * The joint joining the two given links, null if there is no such a joint.
     */
    def public Joint connectingJoint(AbstractLink l1, AbstractLink l2)
    {
        var GraphEdge edge = null
        edge = childToParent.getEdge(l1, l2)
        if(edge !== null) {
            return edge.joint
        }
        edge = parentToChild.getEdge(l1, l2)
        if(edge !== null) {
            return edge.joint
        }
        return null
    }

    /**
     * The parent link of the given link
     */
    def public dispatch AbstractLink parent(RobotBase base) { return null }
    def public dispatch AbstractLink parent(Link l) {
        return toParent(l).parent()

    }
    /**
     * The joint moving the given link
     */
    def public dispatch supportingJoint(RobotBase base) { return null }
    def public dispatch supportingJoint(Link l) {
        return toParent(l).joint()
    }

    def public predecessor(Joint j) {
        for( e : parentToChild.edgeSet ) {
            if( e.joint().equals(j) ) {
                return e.parent()
            }
        }
        return null // should never get here, it means the joint was not found in the graph
    }
    def public successor(Joint j) {
        for( e : parentToChild.edgeSet ) {
            if( e.joint().equals(j) ) {
                return e.child()
            }
        }
        return null // should never get here, it means the joint was not found in the graph
    }

    /**
     * Tells whether a link is an ancestor of another link.
     * Any link is an ancestor/descendant of itself.
     * \params possibleAncestor
     * \params target
     * \return true if target belongs to a kinematic subtree rooted at the
     *         possibleAncestor, false otherwise
     */
    def public boolean ancestorOf(AbstractLink possibleAncestor, AbstractLink target)
    {
        if(possibleAncestor===null || target===null) return false
        if(possibleAncestor.ID > target.ID) return false
        return possibleAncestor.equals(target) || ancestorOf(possibleAncestor, target.parent())
        // note that the equals() deals with parent being null
    }

    /**
     * The Lowest Common Ancestor of the two given links.
     *
     * The lowest common ancestor is the deepest (i.e. farthest from the base)
     * link which is an ancestor of both the arguments.
     */
    def public AbstractLink lowestCommonAncestor(AbstractLink l1, AbstractLink l2)
    {
        if( l1===null  || l2===null) return null

        var AbstractLink lca = null
        var AbstractLink current1 = l1
        var AbstractLink current2 = l2

        while( lca === null )
        {
            if( current1.ancestorOf(l2) ) {
                lca = current1
            } else {
                current1 = current1.parent() // cant be null, as that implies current1 is the base, and the base is always an ancestor
                if( current2.ancestorOf(l1) ) {
                    lca = current2
                } else {
                    current2 = current2.parent()
                }
            }
        }
        return lca
    }

    def private GraphEdge toParent(AbstractLink l) {
        // is there a simpler function call for graph where one knows that the outgoing
        // edge is exactly one?
        return childToParent.outgoingEdgesOf(l).iterator().next()
    }

    private Robot robot
    private SimpleDirectedGraph<AbstractLink, GraphEdge> childToParent
    private SimpleDirectedGraph<AbstractLink, GraphEdge> parentToChild


    private static class GraphEdge
    {
        public new (Joint j, AbstractLink par, AbstractLink chi)
        {
            joint = j
            parent = par
            child  = chi
        }
        def public joint() { return joint }
        def public parent(){ return parent}
        def public child() { return child }

        private final Joint joint
        private final AbstractLink parent
        private final AbstractLink child
    }


    private static extension iit.dsl.generator.Common common = iit.dsl.generator.Common.getInstance()

    /**
     * Tells whether a link belongs to a kinematic subtree rooted in another
     * link.
     * \params candidate the link whose position has to be investigated
     * \params start the first link of the kinematic subtree to be visited
     * \return true if candidate belongs to the kinematic subtree rooted
     *         at start, false otherwise
     */
    def static boolean isDescendant(AbstractLink candidate, AbstractLink start) {
        if(common.contains(start.childrenList, candidate)) {
            return true
        }
        for(ChildSpec child : start.childrenList.children) {
            if(isDescendant(candidate, child.link)) {
                return true
            }
        }
        return false
    }

    /**
     * Searches for the common ancestor of the two given links, that is
     * the root of the smallest kinematic tree that contains both links.
     * This function throws an exception if no such link is found, since
     * it expects two links of the same robot (which is always a connected
     * kinematic tree).
     * \params l1 the first link
     * \params l2 the second link
     * \return the root of the smallest kinematic tree that contains both
     *         l1 and l2.
     *
     * Note: I think this is flawed, it does not work with branching at multiple
     * layers (e.g. hand fingers in humanoid)
     */
    def static AbstractLink commonAncestor(AbstractLink l1, AbstractLink l2) {
        if(l1.equals(l2)) return l1;
        if(isDescendant(l1, l2)) return l2;
        if(isDescendant(l2, l1)) return l1;
        var AbstractLink child1  = l1
        var AbstractLink child2  = l2
        var AbstractLink parent1 = common.getParent(l1)
        var AbstractLink parent2 = common.getParent(l2)
        while(parent1 !== null && parent2 !== null) {
            if(parent1.equals(parent2)) return parent1;
            // follow the branches up in the hierarchy
            child1  = parent1
            child2  = parent2
            parent1 = common.getParent(parent1)
            parent2 = common.getParent(parent2)
        }
        while(parent1 !== null) {
            if(parent1.equals(child2)) return parent1;
            // follow the branch 1 up in the hierarchy
            child1  = parent1
            parent1 = common.getParent(parent1)
        }
        while(parent2 !== null) {
            if(parent2.equals(child1)) return parent2;
            // follow the branch 2 up in the hierarchy
            child2  = parent2
            parent2 = common.getParent(parent2)
        }
        //should never get here
        throw(new RuntimeException("looks like these two links do not have a common ancestor!!"))
    }

    /**
     * Constructs the kinematic chain whose ends are the two links in the argument.
     * \param first the first link of the chain
     * \param last the last link of the chain
     * \return a list of links that correspond to a connected kinematic chain of
     *         the robot, delimited by the two given links first and last
     */
    def static List<AbstractLink> buildChain(AbstractLink first, AbstractLink last) {
        val List<AbstractLink> chain = new ArrayList<AbstractLink>()
        if(first.equals(last)) {
            chain.add(first)
            return chain
        }
        val AbstractLink ancestor = commonAncestor(first, last);
        var AbstractLink parent

        if(last.equals(ancestor)) {
            chain.add(first)
            parent = common.getParent(first)
            while(! parent.equals(last)){
                chain.add(parent)
                parent = common.getParent(parent)
            }
            chain.add(last)
            return chain
        }
        if(first.equals(ancestor)) {
            chain.add(last)
            parent = common.getParent(last)
            while(! parent.equals(first)) {
                chain.add(parent)
                parent = common.getParent(parent)
            }
            chain.add(first)
            return chain.reverse
        }

        // The two links belong to different branches starting from the common ancestor
        val List<AbstractLink> head = buildChain(first, ancestor);
        val List<AbstractLink> tail = buildChain(ancestor, last);

        head.addAll( tail.drop(1) ) //drop the first because it is the second copy of the ancestor
        return head
    }

    /**
     * Constructs the chain (list) of links connecting the argument to the
     * robot base, except the base itself
     */
    def static List<AbstractLink> chainToBase(AbstractLink l) {
        val chain = buildChain(l, (l.eContainer() as Robot).base)
        chain.remove(chain.size() - 1) // removes the last element, which is the base
        return chain
    }

    /**
     * Calculate the rotation matrix from the default link frame to the base,
     * for each link of the robot.
     * @param base the robot base link
     * @return a map that associate every link of the robot (but the base) to
     *    the 3x3 rotation matrix that transforms coordinates of the link frame
     *    into coordinates of the robot base frame.
     */
    def static Map<Link, double[][]> getLinkToBaseRotationMatrices(RobotBase base)
    {
        val ret = new HashMap<Link, double[][]>()
        addChildrenTransforms(base, null, ret)
        return ret
    }
    def private static void addChildrenTransforms(
        AbstractLink currentLink, double[][] base_R_current, Map<Link, double[][]> map  )
    {
        if(currentLink.childrenList.children.empty) return;
        for(c : currentLink.childrenList.children) {
            val rot = c.joint.refFrame.rotation
            val R   = Utilities::original_X_rotated(rot.x.asFloat, rot.y.asFloat, rot.z.asFloat)
            var double[][] RR
            if(base_R_current !== null) {
                RR  = Utilities::matrix3x3Mult(base_R_current, R)
            } else {
                RR = R
            }

            map.put(c.link, RR)
            addChildrenTransforms(c.link, RR, map)
        }
    }
}