package iit.dsl.generator.cpp

import java.util.List
import com.google.inject.Inject
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint

import iit.dsl.generator.Jacobian


class Jacobians {
    @Inject iit.dsl.generator.maxima.Converter maximaConverter

    def private dynamicAssignmentsCode(Jacobian J, String[][] JasText, String varName) {
        val StringConcatenation strBuff = new StringConcatenation();
        var r = 0 // row index
        var c = 0 // column index
        for(Joint j : J.jointsChain) {
            if(j instanceof RevoluteJoint) {
                strBuff.append('''
                static double «Common::variableForCosineOf(j)»;
                static double «Common::variableForSineOf(j)»;
                '''
                );
            }
        }
        for(Joint j : J.jointsChain) {
            if(j instanceof RevoluteJoint) {
                strBuff.append('''
                «Common::variableForCosineOf(j)» = std::cos(«Common::valueAccessorOf(j)»);
                «Common::variableForSineOf(j)» = std::sin(«Common::valueAccessorOf(j)»);
                '''
                );
            }
        }
        for(row : JasText) {
            for(el : row) {
                if( !iit::dsl::coord::generator::MaximaConverter::isConstant(el)) {
                    strBuff.append('''«varName»(«r»,«c») = «el»;
                    ''')
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

        namespace «Names$Namespaces::internal» {
            «FOR Jacobian j : jacs»
                void «staticInitFunc_name(j)»(«staticInitFunc_argsList(j)»);
            «ENDFOR»

            «FOR Jacobian j : jacs»
                void «runtimeAssignementsFunc_name(j)»(«runtimeAssignementsFunc_argsList(j)»);
            «ENDFOR»
        } //namespace '«Names$Namespaces::internal»'
        '''

    def definitions(Robot robot, List<Jacobian> jacs) '''
        «FOR Jacobian j : jacs»
            «Names$Types::jacobianLocal»<«j.cols»> «Names$Namespaces$Qualifiers::roboJacs(robot)»::«j.name»(«Names$Namespaces::internal»::«runtimeAssignementsFunc_name(j)»);
        «ENDFOR»

        void «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::jacobians»::initAll() {
            «FOR Jacobian j : jacs»
                «Names$Namespaces::internal»::«staticInitFunc_name(j)»(«j.name»);
            «ENDFOR»
        }

        «val MaximaConversion maximaConversion = new MaximaConversion(robot)»
        «FOR Jacobian j : jacs»
            «val jText = maximaConverter.getJacobianText(j, maximaConversion)»
            void «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::jacobians»::«Names$Namespaces::internal»::«staticInitFunc_name(j)»(«staticInitFunc_argsList(j)»)
            {
                «iit::dsl::coord::generator::cpp::EigenCommons::
                matrixInitializationCode(jText, "mx")»
            }

            void «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::jacobians»::«Names$Namespaces::internal»::«runtimeAssignementsFunc_name(j)»(«runtimeAssignementsFunc_argsList(j)»)
            {
                «dynamicAssignmentsCode(j, jText, "mx")»
            }
        «ENDFOR»
        '''



    def staticInitFunc_name(Jacobian J) '''init__«J.name»'''
    def runtimeAssignementsFunc_name(Jacobian J) '''jointStateSetter__«J.name»'''

    def staticInitFunc_argsList(Jacobian J)
        '''«Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, «J.cols»>& mx'''

    def runtimeAssignementsFunc_argsList(Jacobian J)
        '''const «Names$Types::jointState»& «Common::jointsStateVarName», «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, «J.cols»>& mx'''

    def implementationFile(Robot robot, List<Jacobian> jacs) '''
        #include "«Names$Files::jacobiansHeader(robot)».h"

        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::roboJacs(robot)»;

        «definitions(robot, jacs)»
        '''
}