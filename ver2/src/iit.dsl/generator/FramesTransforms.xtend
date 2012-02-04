package iit.dsl.generator

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Var
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.Robot
import iit.dsl.TransSpecsAccessor

import org.eclipse.xtend2.lib.StringConcatenation
import com.google.inject.Inject



/**
 * Static utilities for the generation of text related to the reference frames
 * of the robot.
 * The functions generate simple strings that represent various coordinate
 * transformations.
 */
class FramesTransforms {
    @Inject TransSpecsAccessor desiredTrasformsAccessor

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
     * Return the transform between a parent and a child link, both directions
     */
    def static predecessor_X_successor(Joint j) '''
        «predecessor_X_joint(j)» «joint_X_successor(j)»'''
    def static successor_X_predecessor(Joint j) '''
        «successor_X_joint(j)» «joint_X_predecessor(j)»'''
    /**
     * Returns the "code" of the transformation 'dest_X_source' for two arbitrary links
     */
    def static dest_X_source(AbstractLink dest, AbstractLink source) '''
        «val AbstractLink ancestor = common.commonAncestor(source,dest)»
        «descendant_X_ancestor(ancestor, dest)» «ancestor_X_descendant(ancestor, source)»'''

    def private static ancestor_X_descendant(AbstractLink ancestor, AbstractLink descendant) {
        if(descendant.equals(ancestor)) return ''''''
        val Joint j = common.getConnectingJoint(descendant)
        return
        '''«ancestor_X_descendant(ancestor, common.getPredecessorLink(j))» «predecessor_X_successor(j)»'''
    }
    def private static descendant_X_ancestor(AbstractLink ancestor, AbstractLink descendant) {
        if(descendant.equals(ancestor)) return ''''''
        val Joint j = common.getConnectingJoint(descendant)
        return
        '''«successor_X_predecessor(j)» «descendant_X_ancestor(ancestor, common.getPredecessorLink(j))»'''
    }

    def static link_X_frame(AbstractLink link, RefFrame frame) {
        /* Concatenates the transform between the link in the argument and the link containing the frame,
         * with the transform between that link and that frame
         */
        val AbstractLink source = common.getContainingLink((link.eContainer as Robot), frame)
        if(source == null) {
            throw new RuntimeException(unknownFrameErrorMsg((link.eContainer as Robot), frame))
        }
        val tx = frame.transform.translation.x
        val ty = frame.transform.translation.y
        val tz = frame.transform.translation.z
        val rx = frame.transform.rotation.x
        val ry = frame.transform.rotation.y
        val rz = frame.transform.rotation.z
        '''«dest_X_source(link, source)» «TxString(tx)» «TyString(ty)» «TzString(tz)» «RxString(rx)» «RyString(ry)» «RzString(rz)»'''
    }
    def static frame_X_link(RefFrame frame, AbstractLink link) {
        val AbstractLink dest = common.getContainingLink((link.eContainer as Robot), frame)
        if(dest == null) {
            throw new RuntimeException(unknownFrameErrorMsg((link.eContainer as Robot), frame))
        }
        val tx = common.invert(frame.transform.translation.x)
        val ty = common.invert(frame.transform.translation.y)
        val tz = common.invert(frame.transform.translation.z)
        val rx = common.invert(frame.transform.rotation.x)
        val ry = common.invert(frame.transform.rotation.y)
        val rz = common.invert(frame.transform.rotation.z)
        '''«TxString(tx)» «TyString(ty)» «TzString(tz)» «RxString(rx)» «RyString(ry)» «RzString(rz)» «dest_X_source(dest, link)»'''
    }

    def static transformLiteral(AbstractLink dest, AbstractLink source)
        '''{«common.getFrameName(dest)»}_X_{«common.getFrameName(source)»}'''
    def static transformLiteral(AbstractLink dest, RefFrame source)
        '''{«common.getFrameName(dest)»}_X_{«source.name»}'''
    def static transformLiteral(RefFrame dest, AbstractLink source)
        '''{«dest.name»}_X_{«common.getFrameName(source)»}'''
    def static transformLiteral(RefFrame dest, RefFrame source)
        '''{«dest.name»}_X_{«source.name»}'''



    def parentChildTransforms(Robot robot) '''
        «FOR link : robot.links»
            «val joint  = common.getConnectingJoint(link)»
            «val parent = common.getParent(link)»
            «transformLiteral(parent, link)» = «predecessor_X_successor(joint)»
            «transformLiteral(link, parent)» = «successor_X_predecessor(joint)»
        «ENDFOR»
        '''

    def transformsForJacobian(Robot robot, iit.dsl.transspecs.transSpecs.FramePair jacSpec) {
        // Create two reference-frame instances using local types
        val base   = common.getFrameByName(robot, jacSpec.base.name)
        val target = common.getFrameByName(robot, jacSpec.target.name)
        return transformsForJacobian(robot, base, target)
    }

    def transformsForJacobian(Robot robot, AbstractLink base, AbstractLink targetLink) {
        return transformsForJacobian(robot, base, targetLink,
            common.getDefaultFrame(base), common.getDefaultFrame(targetLink))
    }

    def transformsForJacobian(Robot robot, AbstractLink base, RefFrame targetFrame) {
        val targetLink = common.getContainingLink(robot,targetFrame)
        if(targetLink == null) throw(new RuntimeException(unknownFrameErrorMsg(robot, targetFrame)))

        return transformsForJacobian(robot, base, targetLink,
            common.getDefaultFrame(base), targetFrame)
    }

    def transformsForJacobian(Robot robot, RefFrame baseFrame, AbstractLink movingLink ) {
        val baseLink = common.getContainingLink(robot,baseFrame)
        if(baseLink == null) throw(new RuntimeException(unknownFrameErrorMsg(robot, baseFrame)))

        return transformsForJacobian( robot, baseLink, movingLink,
            baseFrame, common.getDefaultFrame(movingLink))
    }

    def transformsForJacobian(Robot robot, RefFrame baseFrame, RefFrame movingFrame) {
        val baseLink   = common.getContainingLink(robot,baseFrame)
        if(baseLink == null) throw(new RuntimeException(unknownFrameErrorMsg(robot, baseFrame)))
        val movingLink = common.getContainingLink(robot,movingFrame)
        if(movingLink == null)  throw(new RuntimeException(unknownFrameErrorMsg(robot, movingFrame)))

        return transformsForJacobian(robot, baseLink, movingLink, baseFrame, movingFrame)
    }

    def private static unknownFrameErrorMsg(Robot robot, RefFrame frame) {
        '''Hey!, seems that frame «frame.name» does not belong to any of the links of robot «robot.name»'''.toString()
    }

    def private transformsForJacobian(Robot robot, AbstractLink base, AbstractLink targetLink, RefFrame baseFr, RefFrame targetFr) {
        val StringConcatenation strBuff = new StringConcatenation();
        val chain = common.buildChain(base, targetLink).drop(1) //drops the base itself

        for(el : chain) {
            strBuff.append('''«transformLiteral(baseFr, el)» = «frame_X_link(baseFr, el)»''')
            strBuff.append("\n")
        }
        // if the moving frame is the default frame of the moving link, the corresponding
        //  transform has been already generated
        if(! common.getFrameName(targetLink).toString().equals(targetFr.name)) {
            strBuff.append('''«transformLiteral(baseFr, targetFr)» = «frame_X_link(baseFr, targetLink)» «link_X_frame(targetLink, targetFr)»''')
            strBuff.append("\n")
        }
        return strBuff
    }

    def coordinateTransformsDSLDocument(Robot robot) '''
        Model «robot.name»
        Frames {
            «common.getFrameName(robot.base)»
            «FOR link : robot.links»
                , «common.getFrameName(link)»
                «FOR RefFrame frame : link.frames»
                    , «frame.name»
                «ENDFOR»
            «ENDFOR»
            «FOR joint : robot.joints»
                , «common.getFrameName(joint)»
            «ENDFOR»
        }

        TransformedFramePos = right

        /* All the <child>_X_<parent> transforms (plus the inverse), required for
            instance by the inverse dynamics algorithm */
        «parentChildTransforms(robot)»

        «val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredTransforms = desiredTrasformsAccessor.getDesiredTransforms(robot)»
        // Additional transforms required by the user
        «IF(desiredTransforms != null)»
            «FOR iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredTransforms.transforms.getSpecs()»
                «val dest   = common.getFrameByName(robot, jSpec.base.name)»
                «val source = common.getFrameByName(robot, jSpec.target.name)»
                «val destLink   = common.getContainingLink(robot, dest)»
                «val sourceLink = common.getContainingLink(robot, source)»
                «transformLiteral(dest, source)» = «frame_X_link(dest,destLink)» «dest_X_source(destLink, sourceLink)» «link_X_frame(sourceLink, source)»
            «ENDFOR»
        «ENDIF»

        // Transforms required to compute Jacobians more conveniently
        «IF(desiredTransforms != null)»
            «FOR iit.dsl.transspecs.transSpecs.FramePair jSpec : desiredTransforms.jacobians.getSpecs()»
                «transformsForJacobian(robot, jSpec)»
            «ENDFOR»
        «ENDIF»
        '''

}

