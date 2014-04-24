package iit.dsl.generator.common

import java.util.List

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.ChildSpec
import java.util.ArrayList

/**
 * Utility functions for the visit of the kinematic tree structure and
 * the creation of subtrees (e.g. chains)
 */
class TreeUtils {
    private static iit.dsl.generator.Common common = new iit.dsl.generator.Common()

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
     */
    def static AbstractLink commonAncestor(AbstractLink l1, AbstractLink l2) {
        if(l1.equals(l2)) return l1;
        if(isDescendant(l1, l2)) return l2;
        if(isDescendant(l2, l1)) return l1;
        var AbstractLink child1  = l1
        var AbstractLink child2  = l2
        var AbstractLink parent1 = common.getParent(l1)
        var AbstractLink parent2 = common.getParent(l2)
        while(parent1 != null && parent2 != null) {
            if(parent1.equals(parent2)) return parent1;
            // follow the branches up in the hierarchy
            child1  = parent1
            child2  = parent2
            parent1 = common.getParent(parent1)
            parent2 = common.getParent(parent2)
        }
        while(parent1 != null) {
            if(parent1.equals(child2)) return parent1;
            // follow the branch 1 up in the hierarchy
            child1  = parent1
            parent1 = common.getParent(parent1)
        }
        while(parent2 != null) {
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
}