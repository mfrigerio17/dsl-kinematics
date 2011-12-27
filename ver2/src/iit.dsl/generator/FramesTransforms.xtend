package iit.dsl.generator

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint

import static iit.dsl.generator.FramesTransforms.*
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

    def static predecessor_X_joint(Joint joint)
    '''Tx(«joint.refFrame.translation.x») Ty(«joint.refFrame.translation.y») Tz(«joint.refFrame.translation.z») Rx(«joint.refFrame.rotation.x») Ry(«joint.refFrame.rotation.y») Rz(«joint.refFrame.rotation.z»)'''

    def static joint_X_predecessor(Joint joint) {
        val tx = Utilities::invert(joint.refFrame.translation.x)
        val ty = Utilities::invert(joint.refFrame.translation.y)
        val tz = Utilities::invert(joint.refFrame.translation.z)
        val rx = Utilities::invert(joint.refFrame.rotation.x)
        val ry = Utilities::invert(joint.refFrame.rotation.y)
        val rz = Utilities::invert(joint.refFrame.rotation.z)
        return '''Tx(«tx») Ty(«ty») Tz(«tz») Rz(«rz») Ry(«ry») Rx(«rx»)'''
    }
}