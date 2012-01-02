package iit.dsl.generator

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
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
        val tx = common.str(joint.refFrame.translation.x)
        val ty = common.str(joint.refFrame.translation.y)
        val tz = common.str(joint.refFrame.translation.z)
        val rx = common.str(joint.refFrame.rotation.x)
        val ry = common.str(joint.refFrame.rotation.y)
        val rz = common.str(joint.refFrame.rotation.z)
        '''Tx(«tx») Ty(«ty») Tz(«tz») Rx(«rx») Ry(«ry») Rz(«rz»)'''
    }
    def static joint_X_predecessor(Joint joint) {
        val tx = common.str(common.invert(joint.refFrame.translation.x))
        val ty = common.str(common.invert(joint.refFrame.translation.y))
        val tz = common.str(common.invert(joint.refFrame.translation.z))
        val rx = common.str(common.invert(joint.refFrame.rotation.x))
        val ry = common.str(common.invert(joint.refFrame.rotation.y))
        val rz = common.str(common.invert(joint.refFrame.rotation.z))
        return '''Tx(«tx») Ty(«ty») Tz(«tz») Rz(«rz») Ry(«ry») Rx(«rx»)'''
    }
}