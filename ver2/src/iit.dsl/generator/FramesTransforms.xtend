package iit.dsl.generator

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Var
import iit.dsl.kinDsl.AbstractLink

/**
 * Static utilities for the generation of text related to the reference frames
 * of the robot.
 * The functions generate simple strings that represent various coordinate
 * transformations.
 */
class FramesTransforms {
    static Common common = new Common()

	def static dispatch joint_X_successor(RevoluteJoint  joint)
    '''Rz(«common.getVariableName(joint)»)'''

    def static dispatch joint_X_successor(PrismaticJoint joint)
    '''Tz(«common.getVariableName(joint)»)'''

    def static dispatch successor_X_joint(RevoluteJoint  joint)
    '''Rz(-«common.getVariableName(joint)»)'''

    def static dispatch successor_X_joint(PrismaticJoint joint)
    '''Tz(-«common.getVariableName(joint)»)'''

    def static predecessor_X_joint(Joint joint) {
        val tx = joint.refFrame.translation.x
        val ty = joint.refFrame.translation.y
        val tz = joint.refFrame.translation.z
        val rx = joint.refFrame.rotation.x
        val ry = joint.refFrame.rotation.y
        val rz = joint.refFrame.rotation.z
        '''«TxString(tx)» «TyString(ty)» «TzString(tz)» «RxString(rx)» «RyString(ry)» «RzString(rz)»'''
    }
    def static joint_X_predecessor(Joint joint) {
        val tx = common.invert(joint.refFrame.translation.x)
        val ty = common.invert(joint.refFrame.translation.y)
        val tz = common.invert(joint.refFrame.translation.z)
        val rx = common.invert(joint.refFrame.rotation.x)
        val ry = common.invert(joint.refFrame.rotation.y)
        val rz = common.invert(joint.refFrame.rotation.z)
        '''«TxString(tx)» «TyString(ty)» «TzString(tz)» «RzString(rz)» «RyString(ry)» «RxString(rx)»'''
    }

    def static TxString(Var tx) '''«IF !common.isZero(tx)»Tx(«common.str(tx)»)«ENDIF»'''
    def static TyString(Var ty) '''«IF !common.isZero(ty)»Ty(«common.str(ty)»)«ENDIF»'''
    def static TzString(Var tz) '''«IF !common.isZero(tz)»Tz(«common.str(tz)»)«ENDIF»'''
    def static RxString(Var rx) '''«IF !common.isZero(rx)»Rx(«common.str(rx)»)«ENDIF»'''
    def static RyString(Var ry) '''«IF !common.isZero(ry)»Ry(«common.str(ry)»)«ENDIF»'''
    def static RzString(Var rz) '''«IF !common.isZero(rz)»Rz(«common.str(rz)»)«ENDIF»'''

    /**
     * Returns the "code" of the transformation 'dest_X_source' for two arbitrary links
     */
    def static dest_X_source(AbstractLink dest, AbstractLink source) '''
        «val AbstractLink ancestor = common.commonAncestor(source,dest)»
        «descendant_X_ancestor(ancestor, dest)» «ancestor_X_descendant(ancestor, source)»
        '''

    def private static ancestor_X_descendant(AbstractLink ancestor, AbstractLink descendant) {
        if(descendant.equals(ancestor)) return ''''''
        val Joint j = common.getConnectingJoint(descendant)
        return
        '''«ancestor_X_descendant(ancestor, common.getPredecessorLink(j))» «predecessor_X_joint(j)» «joint_X_successor(j)»'''
    }
    def private static descendant_X_ancestor(AbstractLink ancestor, AbstractLink descendant) {
        if(descendant.equals(ancestor)) return ''''''
        val Joint j = common.getConnectingJoint(descendant)
        return
        '''«successor_X_joint(j)» «joint_X_predecessor(j)» «descendant_X_ancestor(ancestor, common.getPredecessorLink(j))»'''
    }
}