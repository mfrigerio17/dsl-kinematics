package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint

class Common {
    static iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def public static linkIdentifier(AbstractLink l) '''«l.name.toUpperCase()»'''
    def public static jointIdentifier(Joint j) '''«j.name.toUpperCase()»'''
    def public static variableForCosineOf(Joint j) '''cos_«common.getVariableName(j)»'''
    def public static variableForSineOf(Joint j)   '''sin_«common.getVariableName(j)»'''
    def public static valueAccessorOf(Joint j) '''«jointsStateVarName()»(«jointIdentifier(j)»)'''
    def public static jointsStateVarName() '''jState'''

    /**
     * The index of the coordinate of a 6D spatial vector that corresponds to the given joint.
     * These functions assume that the index is in the range [0..5], and that the axis
     * of the joint is always lying on the z axis. Therefore the possible output of these
     * functions is either 2 or 5, which correspond to z axis, revolute or prismatic.
     * Telling whether a revolute joint is 2 or 5 (and similarly for the prismatic) is
     * basically the job of these functions. In other words, their implementation must
     * be compliant with the convention about spatial vectors: rotational coordinates
     * come first and linear ones afterwards, or the other way round.
     */
    def public static dispatch spatialVectIndex(RevoluteJoint joint) {
        return 2
    }
    def public static dispatch spatialVectIndex(PrismaticJoint joint) {
        return 5
    }
}