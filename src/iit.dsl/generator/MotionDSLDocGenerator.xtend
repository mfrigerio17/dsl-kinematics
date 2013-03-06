package iit.dsl.generator

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.FloatLiteral
import iit.dsl.kinDsl.Expr
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.RotoTrasl

import org.eclipse.xtend2.lib.StringConcatenation
import org.eclipse.xtext.nodemodel.util.NodeModelUtils

class MotionDSLDocGenerator {
    def public static String fileName(Robot robot) {
        return robot.name + "_frames.motdsl"
    }

    private extension Common common = new Common()

    def public documentContent(Robot robot) '''
        Model «robot.name»_frames
        Frames {
            «val alllinks = common.abstractLinks(robot)»
            «FOR link : alllinks
            SEPARATOR ", "
            AFTER ","»«common.getFrameName(link)»«ENDFOR»

            «FOR link : alllinks»
                «FOR frame : link.frames
                SEPARATOR ", "
                AFTER ","»«frame.name»«ENDFOR»
            «ENDFOR»

            «FOR joint : robot.joints
            SEPARATOR ", "»«common.getFrameName(joint)»«ENDFOR»
        }

        Convention = local

        «FOR joint : robot.joints»
            «joint.predecessorLink.frameName» -> «joint.frameName» : «motionStepsFromPredecessor(joint)»
        «ENDFOR»

        «FOR joint : robot.joints»
            «joint.frameName» -> «joint.successorLink.frameName» : «motionStepsToSuccessor(joint)»
        «ENDFOR»

        «FOR link : alllinks»
            «FOR frame : link.frames»
                «link.frameName» -> «frame.name» : «motionSteps(frame.transform)»
            «ENDFOR»
        «ENDFOR»
    '''

    def private motionStepsFromPredecessor(Joint joint) {
        return motionSteps(joint.refFrame)
    }

    def private motionSteps(RotoTrasl rotoTransl){
        val StringConcatenation text = new StringConcatenation()
        val transl = rotoTransl.translation
        val rot    = rotoTransl.rotation

        var CharSequence tmp
        var isEmpty = true
        // Translation
        tmp = value(transl.x)
        if(tmp.length() > 0) {
            text.append('''trx(«tmp»)''')
            isEmpty = false
        }

        tmp = value(transl.y)
        if(tmp.length() > 0) {
            if(!isEmpty) text.append(" ")
            text.append('''try(«tmp»)''')
            isEmpty = false
        }

        tmp = value(transl.z)
        if(tmp.length() > 0) {
            if(!isEmpty) text.append(" ")
            text.append('''trz(«tmp»)''')
            isEmpty = false
        }

        //Rotation
        tmp = value(rot.x)
        if(tmp.length() > 0) {
            if(!isEmpty) text.append(" ")
            text.append('''rotx(«tmp»)''')
            isEmpty = false
        }

        tmp = value(rot.y)
        if(tmp.length() > 0) {
            if(!isEmpty) text.append(" ")
            text.append('''roty(«tmp»)''')
            isEmpty = false
        }

        tmp = value(rot.z)
        if(tmp.length() > 0) {
            if(!isEmpty) text.append(" ")
            text.append('''rotz(«tmp»)''')
        }

        return text
    }

    def private dispatch motionStepsToSuccessor(PrismaticJoint joint)
    '''trz(«joint.variableName»)'''
    def private dispatch motionStepsToSuccessor(RevoluteJoint joint)
    '''rotz(«joint.variableName»)'''

    def private dispatch value(FloatLiteral fl)
        '''«IF fl.value != 0.0»«fl.value»«ENDIF»'''
    def private dispatch value(Expr expr)
        '''«NodeModelUtils::findActualNodeFor(expr).getText()»'''
}