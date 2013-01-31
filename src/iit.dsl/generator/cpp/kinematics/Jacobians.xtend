package iit.dsl.generator.cpp.kinematics

import java.util.List
import java.util.Iterator
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Jacobian
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.MaximaReplacementSpec
import iit.dsl.generator.cpp.Common


class Jacobians {
    iit.dsl.generator.maxima.Converter maximaConverter = new iit.dsl.generator.maxima.Converter()
    iit.dsl.maxdsl.utils.DSLAccessor   maxdslAccess    = new iit.dsl.maxdsl.utils.DSLAccessor()
    iit.dsl.maxdsl.generator.cpp.Utils exprGenerator   = new iit.dsl.maxdsl.generator.cpp.Utils()

    def headerFile(Robot robot, List<Jacobian> jacs) '''
        #ifndef «robot.name.toUpperCase()»_JACOBIANS_H_
        #define «robot.name.toUpperCase()»_JACOBIANS_H_

        #include <iit/rbd/JStateDependentMatrix.h>
        #include "«Names$Files::mainHeader(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        template<int COLS>
        class «Names$Types::jacobianLocal» : public «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, COLS> {
            private:
                typedef «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, COLS> Base;
            public:
                JacobianT(typename Base::JointStateSetter setter) : Base(setter) {}
        };


        /**
         * The namespace with the Jacobian matrices for the robot «robot.name»
         */
        namespace «Names$Namespaces::jacobians» {
            /* Declarations */
            «declarations(robot, jacs)»


            /**
             * Call this function once at initialization time, to prepare the
             * transform matrices.
             **/
            void initAll();

        } // end of namespace '«Names$Namespaces::jacobians»'
        } // end of namespace '«Names$Namespaces::rob(robot)»'
        } // end of namespace '«Names$Namespaces::enclosing»'

        #endif
        '''


    def private dynamicAssignmentsCode(Jacobian J, String[][] JasText, String varName) {
        val StringConcatenation strBuff = new StringConcatenation();
        var r = 0 // row index
        var c = 0 // column index

        val maxdslDoc = iit::dsl::generator::maxima::MaximaDSLUtils::MaximaDSLDocumentText(J, JasText)
        //Check if the current Jacobian is actually a function of something. If not, it is a constant,
        // and no code generation is required here
        if(maxdslDoc.length > 0) {
            val iit.dsl.maxdsl.maximaDsl.Model expressionsModel =
                    maxdslAccess.getParsedTextModel(maxdslDoc.toString())
            val Iterator<iit.dsl.maxdsl.maximaDsl.Expression> exprIter =
                expressionsModel.expressions.iterator

            val replaceSpecs = new MaximaReplacementSpec(J.robot)
            // declarations of variables for trigonometric functions and assignements:
            strBuff.append(exprGenerator.trigFunctionsCode(expressionsModel, replaceSpecs))
            strBuff.append("\n");
            for(row : JasText) {
                for(el : row) {
                    if( !iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el)) {
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


     def temp(Jacobian J) {
        val J_asText = maximaConverter.getJacobianText(J)
        return dynamicAssignmentsCode(J, J_asText, "J");
     }
}