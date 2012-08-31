package iit.dsl.generator.common

import java.util.List
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot

/**
 * Utility functions for the visit of the kinematic tree structure and
 * the creation of subtrees (e.g. chains)
 */
class TreeUtils {
    private static iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    /**
     * Constructs the chain (list) of links connecting the argument to the
     * robot base, except the base itself
     */
    def static List<AbstractLink> chainToBase(AbstractLink l) {
        val chain = common.buildChain(l, (l.eContainer() as Robot).base)
        chain.remove(chain.size() - 1) // removes the last element, which is the base
        return chain
    }
}