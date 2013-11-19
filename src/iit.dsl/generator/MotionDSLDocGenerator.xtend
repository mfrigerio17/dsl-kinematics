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
import iit.dsl.kinDsl.ParameterLiteral
import java.util.ArrayList
import iit.dsl.generator.common.Parameters
import java.util.HashSet
import java.util.List
import iit.dsl.kinDsl.PlainExpr
import iit.dsl.kinDsl.PILiteral

class MotionDSLDocGenerator {
    def public static String fileName(Robot robot) {
        return robot.name + "_frames.motdsl"
    }

    def public static String lengthsParametersGroupName() { "lengths" }
    def public static String anglesParametersGroupName()  { "angles" }

    private extension Common common = new Common()

    def public documentContent(Robot robot) '''
        Model «robot.name»
        Frames {
            «val alllinks = common.abstractLinks(robot)»
            «FOR link : alllinks
            SEPARATOR ", "
            AFTER ","»«common.getFrameName(link)»«ENDFOR»

            «FOR link : alllinks»
                «FOR frame : link.frames
                SEPARATOR ", " AFTER ","»«frame.name»«ENDFOR»
            «ENDFOR»

            «FOR joint : robot.joints
            SEPARATOR ", "»«common.getFrameName(joint)»«ENDFOR»
        }

        «parametersBlocks(robot)»

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
        currentParameterGroup = lengthsParametersGroupName
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
        currentParameterGroup = anglesParametersGroupName
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

    def private parametersBlocks(Robot robot) {
        val lengthsList = new ArrayList<ParameterLiteral>()
        val anglesList  = new ArrayList<ParameterLiteral>()
        val lengthsSet  = new HashSet<String>()
        val anglesSet   = new HashSet<String>()
        var List<ParameterLiteral> lengthsTmplist = null
        var List<ParameterLiteral>  anglesTmplist = null

        for(Joint j : robot.joints) {
            lengthsTmplist = Parameters::getParameters(j.refFrame.translation)
             anglesTmplist = Parameters::getParameters(j.refFrame.rotation)
            for(p : lengthsTmplist) {
                if( ! lengthsSet.contains(p.varname) ) {
                    lengthsSet.add(p.varname)
                    lengthsList.add(p)
                }
            }
            for(p : anglesTmplist) {
                if( ! anglesSet.contains(p.varname) ) {
                    anglesSet.add(p.varname)
                    anglesList.add(p)
                }
            }
        }
        for( l : common.abstractLinks(robot)) {
            for(f : l.frames) {
                lengthsTmplist = Parameters::getParameters(f.transform.translation)
                 anglesTmplist = Parameters::getParameters(f.transform.rotation)
                for(p : lengthsTmplist) {
                    if( ! lengthsSet.contains(p.varname) ) {
                        lengthsSet.add(p.varname)
                        lengthsList.add(p)
                    }
                }
                for(p : anglesTmplist) {
                    if( ! anglesSet.contains(p.varname) ) {
                        anglesSet.add(p.varname)
                        anglesList.add(p)
                    }
                }
            }
        }
        if(lengthsList.size == 0 && anglesList.size == 0) return ''''''
        return'''
        «IF lengthsList.size > 0»
            Params «lengthsParametersGroupName» {
                «FOR p:lengthsList SEPARATOR ", "»«p.varname»«ENDFOR»
            }
        «ENDIF»

        «IF anglesList.size > 0»
            Params «anglesParametersGroupName» {
                «FOR p:anglesList SEPARATOR ", "»«p.varname»«ENDFOR»
            }
        «ENDIF»
        '''
    }


    def private dispatch motionStepsToSuccessor(PrismaticJoint joint)
    '''trz(«joint.variableName»)'''
    def private dispatch motionStepsToSuccessor(RevoluteJoint joint)
    '''rotz(«joint.variableName»)'''

    def private dispatch value(FloatLiteral fl)
        '''«IF fl.value != 0.0»«fl.value»«ENDIF»'''
    def private dispatch value(Expr expr)
        '''«NodeModelUtils::findActualNodeFor(expr).getText()»'''
    def private dispatch value(PlainExpr expr)
        '''«value(expr.identifier)»'''
    def private dispatch value(PILiteral p)
        '''«p.varname»'''
    def private dispatch value(ParameterLiteral p)
        '''«IF p.minus»-«ENDIF»«currentParameterGroup».«p.varname»'''

    private String currentParameterGroup = null
}