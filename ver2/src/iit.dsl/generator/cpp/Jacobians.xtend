package iit.dsl.generator.cpp

import java.util.List
import com.google.inject.Inject
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Jacobian
import iit.dsl.MaximaDSLAccessor
import java.util.Iterator


class Jacobians {
    @Inject iit.dsl.generator.maxima.Converter maximaConverter

    MaximaDSLAccessor maxdslAccess = new MaximaDSLAccessor()
    iit.dsl.maxdsl.generator.cpp.Utils exprGenerator = new iit.dsl.maxdsl.generator.cpp.Utils()


    def private dynamicAssignmentsCode(Jacobian J, String[][] JasText, String varName) {
        val StringConcatenation strBuff = new StringConcatenation();
        var r = 0 // row index
        var c = 0 // column index

        val iit.dsl.maxdsl.maximaDsl.Model expressionsModel =
                maxdslAccess.getParsedTextModel( MaximaDSLDocumentText(J, JasText).toString() )
        val Iterator<iit.dsl.maxdsl.maximaDsl.Expression> exprIter =
            expressionsModel.expressions.iterator

        val replaceSpecs = new MaximaReplacementSpec(J.robot)
        // declarations of variables for trigonometric functions and assignements:
        strBuff.append(exprGenerator.trigFunctionsCode(expressionsModel, replaceSpecs))
        strBuff.append("\n");
        for(row : JasText) {
            for(el : row) {
                if( !iit::dsl::coord::generator::MaximaConverter::isConstant(el)) {
                    // we assume we as many parsed expressions as the number of non constant elements of the Jacobian
                    if( ! exprIter.hasNext()) {
                        throw new RuntimeException("The number of expressions does not match the Jacobian")
                    }
                    strBuff.append('''«varName»(«r»,«c») = «exprGenerator.toCode(exprIter.next(), replaceSpecs)»;
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

        «FOR Jacobian j : jacs»
            «val jText = maximaConverter.getJacobianText(j)»
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

     /**
      * Creates a document compliant with the MaximaDSL, containing all the non
      * constant elements of the given Jacobian, as a list of expressions.
      *
      */
     def MaximaDSLDocumentText(Jacobian J, String[][] jText) {
         val strBuff = new StringConcatenation();
         strBuff.append('''
            Variables {
                «J.getArgsList()»
            }
            ''')

         for(row : jText) {
            for(el : row) {
                if( !iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el)) {
                    strBuff.append('''«el»;''')
                    strBuff.append("\n");
                }
            }
        }
        return strBuff;
     }

     def temp(Jacobian J) {
        val J_asText = maximaConverter.getJacobianText(J)
        return dynamicAssignmentsCode(J, J_asText, "J");
     }
}