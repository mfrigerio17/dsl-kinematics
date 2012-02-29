package iit.dsl.generator

import java.util.List
import java.util.ArrayList

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink

/**
 * Simple class to model the sparsity of the joint-space inertia matrix.
 */
class SparsityMap {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
	private List<List<Boolean>> map
	
	private static Boolean ZERO = false
	
    /**
     * Constructs the chain (list) of links connecting the argument to the
     * robot base, except the base itself
     */
    def private List<AbstractLink> chainToBase(AbstractLink l) {
        val chain = common.buildChain(l, (l.eContainer() as Robot).base)
        chain.remove(chain.size() - 1) // removes the last element, which is the base
        return chain
    }
	
	new(Robot robot) {
	    map = new ArrayList<List<Boolean>>()
	    for(j : robot.joints) {
	        val row = new ArrayList<Boolean>()
	        for(j2 : robot.joints) {
	            row.add(ZERO)
	        }
	        map.add(row)
	    }

	    for(j : robot.joints) {
	        val chain = chainToBase(j.successorLink)
	        for(l : chain) {
	            val other = l.connectingJoint
	            map.get(j.ID-1).set(other.ID-1, !ZERO)
	            map.get(other.ID-1).set(j.ID-1, !ZERO) // because of symmetry
	        }
	    }
	}
	/**
	 * Tells whether the specified element of the joint space-inertia matrix is zero
	 * or not.
	 */
	def public boolean isZero(int r, int c) {
	    return map.get(r).get(c) == ZERO
	}
	
	/**
	 * A string representation of the sparsity map, with a 'X' for non-zero elements
	 */
    def public asText() '''
        «FOR row : map»
            «FOR el : row»  «IF el»X«ELSE».«ENDIF»«ENDFOR»
            
        «ENDFOR»
    '''
}