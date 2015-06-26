package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Robot

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
     * These functions assume that the axis of the joint is always lying on the
     * z axis of the joint frame, and they work only with 1-dof prismatic or
     * revolute joints (these are basically conventions of the whole Kinematics
     * DSL, anyway).
     * These functions rely on the constants defined in the 'iit::rbd'
     * namespace that identify the coordinates for linear motion along z and
     * rotational motion about z.
     */
    ///@{
    def public static dispatch spatialVectIndex(RevoluteJoint joint)
        '''«Names$Namespaces$Qualifiers::iit_rbd»::AZ'''
    def public static dispatch spatialVectIndex(PrismaticJoint joint)
        '''«Names$Namespaces$Qualifiers::iit_rbd»::LZ'''
    def public static dispatch spatialVectIndex_no_ns(RevoluteJoint joint)
        '''AZ'''
    def public static dispatch spatialVectIndex_no_ns(PrismaticJoint joint)
        '''LZ'''
    ///@}

    def public static enclosingNamespacesOpen() '''
        «FOR ns : Names$Namespaces::enclosing()»
            namespace «ns» {
        «ENDFOR»
    '''
    def public static enclosingNamespacesClose() '''
        «FOR ns : Names$Namespaces::enclosing()»
            }
        «ENDFOR»
    '''
    def public static enclosingNamespacesQualifier()
    '''«FOR ns : Names$Namespaces::enclosing() SEPARATOR "::"»«ns»«ENDFOR»'''

    def public static enclosingNamespacesOpen(Robot r) '''
        «enclosingNamespacesOpen»
        namespace «Names$Namespaces::rob(r)» {
    '''
    def public static enclosingNamespacesClose(Robot r) '''
        }
        «enclosingNamespacesClose»
    '''
    def public static enclosingNamespacesQualifier(Robot r)
    '''«enclosingNamespacesQualifier»::«Names$Namespaces::rob(r)»'''
}