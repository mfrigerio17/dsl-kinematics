package iit.dsl.generator.cpp

import com.google.inject.Inject

import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint
import java.util.List
import iit.dsl.TransSpecsAccessor
import iit.dsl.transspecs.transSpecs.FramePair
import iit.dsl.generator.Jacobian
import java.util.ArrayList


class Jacobians {
    @Inject TransSpecsAccessor desiredTrasformsAccessor
    @Inject iit.dsl.generator.maxima.Converter maximaConverter
    @Inject extension iit.dsl.generator.Common common

    iit.dsl.coord.generator.IMaximaConversionSpec maximaConversionSpec = new iit.dsl.coord.generator.cpp.MaximaConversion()

    def String[][] jacobian(Robot robot, AbstractLink baseLink, RefFrame targetFr) {
        return jacobian(robot, common.getDefaultFrame(baseLink), targetFr)
    }


	def String[][] jacobian(Robot robot, RefFrame baseFr, RefFrame targetFr) {
	    val jacText = maximaConverter.getJacobianText(
	        robot,
	        baseFr, targetFr,
	        new iit.dsl.coord.generator.cpp.MaximaConversion()
	    )
	    return jacText
    }

    def private dynamicAssignmentsCode(Robot robot, String[][] matrixAsText, String matrixVarName, List<iit.dsl.coord.coordTransDsl.VariableLiteral> args) {
        val StringConcatenation strBuff = new StringConcatenation();
        var r = 0 // row index
        var c = 0 // column index
        var Joint j;
        for(iit.dsl.coord.coordTransDsl.VariableLiteral arg : args) {
            j = common.getJointFromVariableName(robot, arg.varname)
            if(j instanceof RevoluteJoint) {
                strBuff.append('''
                static double «Common::variableForCosineOf(j)»;
                static double «Common::variableForSineOf(j)»;
                «Common::variableForCosineOf(j)» = std::cos(«Common::valueAccessorOf(j)»);
                «Common::variableForSineOf(j)» = std::sin(«Common::valueAccessorOf(j)»);
                '''
                );
            }
        }
        for(row : matrixAsText) {
            for(el : row) {
                if( !iit::dsl::coord::generator::MaximaConverter::isConstant(el)) {
                    strBuff.append('''«matrixVarName»(«r»,«c») = «el»;''')
                    strBuff.append("\n");
                }
                c = c+1
            }
            r = r+1 // next row
            c = 0   // back to first colulmn
        }
        return strBuff
    }

    def declarations(Robot robot, List<Jacobian> jacs) '''
        «FOR Jacobian j : jacs»
            extern «Names$Types::jacobianLocal»<«j.cols»> «j.name»;
        «ENDFOR»

        «FOR Jacobian j : jacs»
            void «staticInitFunc_name(j)»(«staticInitFunc_argsList(j)»);
        «ENDFOR»

        «FOR Jacobian j : jacs»
            void «runtimeAssignementsFunc_name(j)»(«runtimeAssignementsFunc_argsList(j)»);
        «ENDFOR»
        '''

    def definitions(Robot robot, List<Jacobian> jacs) '''
        «FOR Jacobian j : jacs»
            «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jacobianLocal»<«j.cols»> «j.name»;
        «ENDFOR»

        void «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::jacobians»::initAll() {
            «FOR Jacobian j : jacs»
                «Names$Namespaces::internal»::«staticInitFunc_name(j)»(«j.name»);
            «ENDFOR»
        }

        «FOR Jacobian j : jacs»
            void «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::jacobians»::«Names$Namespaces::internal»::«runtimeAssignementsFunc_name(j)»(«runtimeAssignementsFunc_argsList(j)»)
            {
                «iit::dsl::coord::generator::cpp::EigenCommons::
                matrixInitializationCode(maximaConverter.getJacobianText(j, maximaConversionSpec), "mx")»
            }
        «ENDFOR»

        '''



    def staticInitFunc_name(Jacobian J) '''init__«J.name»'''
    def runtimeAssignementsFunc_name(Jacobian J) '''jointStateSetter__«J.name»'''

    def staticInitFunc_argsList(Jacobian J)
        '''«Names$Types::jacobianLocal»<«J.cols»>& mx'''

    def runtimeAssignementsFunc_argsList(Jacobian J)
        '''const «Names$Types::jointState»& state, «Names$Types::jacobianLocal»<«J.cols»>& mx'''


    def temp(Robot robot) {
        val StringConcatenation strBuff = new StringConcatenation();
        val iit.dsl.transspecs.transSpecs.DesiredTransforms desiredTransforms = desiredTrasformsAccessor.getDesiredTransforms(robot)
        if(desiredTransforms != null) {
            val jacobians = new ArrayList<Jacobian>()
            for(FramePair jSpec : desiredTransforms.jacobians.getSpecs()) {
                jacobians.add(new Jacobian(robot, jSpec))
//                // Convert the frames to local types
//                val RefFrame base   = common.createDefaultFrame()
//                val RefFrame target = common.createDefaultFrame()
//                base.name   = jSpec.base.name
//                target.name = jSpec.target.name
//                val text = jacobian(robot, base, target)
//                for(row : text) {
//                    System::out.println(Arrays::toString(row))
//                }
            }
            System::out.println(declarations(robot, jacobians))
            System::out.println(definitions(robot, jacobians))
        }
    }

    def implementationFile(Robot robot, List<Jacobian> jacs) '''
    /* Definitions of the Jacobian matrices for the robot «robot.name»: */'''

}