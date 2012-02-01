package iit.dsl.generator.cpp

import com.google.inject.Inject

import java.util.Arrays
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint
import java.util.List


class Jacobians {
    @Inject iit.dsl.generator.maxima.Converter maximaConverter
    @Inject extension iit.dsl.generator.Common common

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

    def temp(Robot robot) {
        if(robot.name.equals("FixedLeg")) {
            val lleg = robot.getLinkByName("lowerLeg")
            val text = jacobian(robot, common.getLinkByName(robot,"Hip"), common.getFrameByName(lleg,"Foot"))
            for(row : text) {
                System::out.println(Arrays::toString(row))
                }
        }
    }

}