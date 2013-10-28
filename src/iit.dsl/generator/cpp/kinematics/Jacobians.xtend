package iit.dsl.generator.cpp.kinematics

import java.util.List
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Jacobian
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common

import iit.dsl.maxdsl.maximaDsl.VarLiteral
import iit.dsl.coord.coordTransDsl.ParameterLiteral
import iit.dsl.coord.coordTransDsl.ParametersDeclaration
import iit.dsl.coord.generator.cpp.Parameters

class Jacobians {
    def public static mainClassName() '''Jacobians'''
    def public static localClassName(Jacobian J) '''Type_«J.name»'''
    def public static updateParametersFunctionName() '''updateParameters'''
    /**
     * The name of the function that updates the Jacobian given a joint-state
     * variable.
     * The name must be consistent with the StateDependentMatrix defined in the
     * iit-rbd library
     */
    def public static updateFunctionName() '''update'''

    def public containerClassQualifier(Robot rob)
        '''«Common::enclosingNamespacesQualifier(rob)»::«containerClassName»'''

    public new()
    {
        maximaConverter = new iit.dsl.generator.maxima.Converter()
        maxdslAccess    = new iit.dsl.maxdsl.utils.DSLAccessor()
        exprGenerator   = new iit.dsl.maxdsl.generator.cpp.Utils()
        localBaseClassName = Names$Types::jacobianLocal
        jointStateTypeName = Names$Types::jointState
        containerClassName = mainClassName.toString
    }


    def public setMaximaConverterConfigurator(
        iit.dsl.generator.maxima.IConverterConfigurator conf)
    {
        maximaConverter.setConfigurator(conf)
    }


    def headerFile(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        List<Jacobian> jacs,
        String parametersHeaderFileName)
    '''
        #ifndef «robot.name.toUpperCase()»_JACOBIANS_H_
        #define «robot.name.toUpperCase()»_JACOBIANS_H_

        #include <iit/rbd/StateDependentMatrix.h>
        #include "«Names$Files::mainHeader(robot)».h"
        #include "«parametersHeaderFileName».h"

        «Common::enclosingNamespacesOpen(robot)»

        template<int COLS, class M>
        class «localBaseClassName» : public «Names$Namespaces$Qualifiers::iit_rbd»::«Names$Types$IIT_RBD::jacobianBase(jointStateTypeName, "COLS", "M")»
        {};

        /**
         *
         */
        class «containerClassName» {
            «val paramGroups =  iit::dsl::generator::common::Jacobians::getAllParametersGroups(jacs, transformsModel)»
            public:
                «localClassesDeclaration(jacs, transformsModel)»
            public:
                «containerClassName»(«FOR gr:paramGroups SEPARATOR ', '»const «paramsGetterType(gr)»&«ENDFOR»);
                void «updateParametersFunctionName»();
            public:
                «FOR J : jacs»
                    «localClassName(J)» «J.name»;
                «ENDFOR»

            protected:
                «FOR gr : paramGroups»
                    «Parameters::namespaceName»::«Parameters::structName(gr)» «member_paramValues(gr)»;
                «ENDFOR»

                «FOR gr : paramGroups»
                    const «Parameters::namespaceName»::«Parameters::className(gr)»* «member_paramsGetter(gr)»;
                «ENDFOR»
        };


        «Common::enclosingNamespacesClose(robot)»

        #endif
        '''

    def sourceFile(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        List<Jacobian> jacs)
    '''
        #include "«Names$Files::jacobiansHeader(robot)».h"

        «val paramsGroups = iit::dsl::generator::common::Jacobians::getAllParametersGroups(jacs, transformsModel)»

        «containerClassQualifier(robot)»::«containerClassName»
            («FOR gr : paramsGroups SEPARATOR ', '»const «paramsGetterType(gr)»& getter_«gr.name»«ENDFOR»)
            «IF paramsGroups.size > 0 || jacs.size > 0» : «ENDIF»
            «FOR J : jacs SEPARATOR ', '»
                «val groupsForJ = iit::dsl::generator::common::Jacobians::getParametersGroups(J, transformsModel)»
                «J.name»(«FOR gr:groupsForJ SEPARATOR ','»«member_paramValues(gr)»«ENDFOR»)
            «ENDFOR»
            «IF paramsGroups.size>0 && jacs.size > 0»,«ENDIF»
            «FOR gr : paramsGroups SEPARATOR ', '»«member_paramsGetter(gr)»(& getter_«gr.name»)«ENDFOR»
        {
            «updateParametersFunctionName()»();
        }


        void «containerClassQualifier(robot)»::«updateParametersFunctionName»() {
            «FOR gr : paramsGroups»
                «FOR p : gr.params»
                    «member_paramValues(gr)».«p.name» = «member_paramsGetter(gr)» -> «Parameters::getterFunctionName(p)»();
                «ENDFOR»
            «ENDFOR»
        }


        «localClassesDefinitions(jacs, transformsModel)»
    '''




    def private localClassesDeclaration( List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transforms)
    '''
        «FOR J : jacs»
            «val param_groups = iit::dsl::generator::common::Jacobians::getParametersGroups(J, transforms)»
            class «localClassName(J)» : public «localBaseClassName»<«J.cols», «localClassName(J)»>
            {
            public:
                «localClassName(J)»(«constructorArguments(param_groups)»);
                const «localClassName(J)»& «updateFunctionName»(const «jointStateTypeName»&);
            protected:
                «FOR gr : param_groups»
                    const «Parameters::namespaceName»::«Parameters::structName(gr)»* «member_paramValues(gr)»;
                «ENDFOR»
            };

        «ENDFOR»
    '''

    def private localClassesDefinitions(List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transforms)
    '''
        «FOR J : jacs»
            «val jText = maximaConverter.getJacobianText(J, transforms)»
            «constructorCode(J, jText, transforms)»

            «updateMethodCode(J, jText, transforms)»
        «ENDFOR»
    '''


    def private constructorCode(Jacobian J, String[][] jText, iit.dsl.coord.coordTransDsl.Model transforms)
    '''
        «val param_groups = iit::dsl::generator::common::Jacobians::getParametersGroups(J, transforms)»
        «containerClassQualifier(J.robot)»::«localClassName(J)»::«localClassName(J)»(«constructorArguments(param_groups)»)
            «FOR gr:param_groups BEFORE ': ' SEPARATOR ", "»«member_paramValues(gr)»(& «formalparam_paramValues(gr)»)«ENDFOR»
        {
            «iit::dsl::coord::generator::cpp::EigenCommons::
                matrixInitializationCode(jText, "(*this)")»
        }'''

    def private updateMethodCode(Jacobian J, String[][] jText, iit.dsl.coord.coordTransDsl.Model transforms)
    '''
        «val qual = containerClassQualifier(J.robot)»
        const «qual»::«localClassName(J)»& «qual»::«localClassName(J)»::«updateFunctionName»(const «jointStateTypeName»& «Common::jointsStateVarName») {
            «dynamicAssignmentsCode(J, jText, "(*this)", transforms)»
            return *this;
        }'''


    def public static parameterValueAccessor(ParameterLiteral p)
        '''«member_paramValues( iit::dsl::coord::generator::Common::getGroup(p))» -> «p.name»'''

    def private static constructorArguments(List<ParametersDeclaration> param_groups)
        '''«FOR gr:param_groups SEPARATOR ", "»const «Parameters::namespaceName»::«Parameters::structName(gr)»& «formalparam_paramValues(gr)»«ENDFOR»'''

    def private static formalparam_paramValues(ParametersDeclaration gr)
        '''_«member_paramValues(gr)»'''

    def private static paramsGetterType(ParametersDeclaration gr)
        '''«Parameters::namespaceName»::«Parameters::className(gr)»'''

    def private static member_paramValues(ParametersDeclaration gr)
       '''«gr.name»_values'''

    def private static member_paramsGetter(ParametersDeclaration gr)
       '''valuesGetter_«gr.name»'''


    def private dynamicAssignmentsCode(Jacobian J, String[][] JasText, String varName, iit.dsl.coord.coordTransDsl.Model transforms)
    {
        val StringConcatenation strBuff = new StringConcatenation();
        var r = 0 // row index
        var c = 0 // column index

        val maxdslDoc = iit::dsl::generator::common::Jacobians::expressionsAsMaximaDSLDocument(J, JasText, transforms)
        // Check if the current Jacobian is actually a function of something.
        // If not, it is a constant, and no code generation is required here
        if(maxdslDoc.length > 0) {
            val expressionsModel = maxdslAccess.getParsedTextModel(maxdslDoc.toString())
            val exprIter = expressionsModel.expressions.iterator

            val replaceSpecs = new MaximaReplacementForJacobian(J.robot, transforms)
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



    private String containerClassName = null
    private String localBaseClassName = null
    private String jointStateTypeName = null
    private iit.dsl.generator.maxima.Converter maximaConverter = null
    private iit.dsl.maxdsl.utils.DSLAccessor   maxdslAccess    = null
    private iit.dsl.maxdsl.generator.cpp.Utils exprGenerator   = null

}


class MaximaReplacementForJacobian implements iit.dsl.maxdsl.generator.IIdentifiersReplacement
{
    private iit.dsl.generator.Common utils
    private Robot robot
    private iit.dsl.coord.coordTransDsl.Model transformsModel

    new(Robot rob, iit.dsl.coord.coordTransDsl.Model transforms)
    {
        this.robot = rob;
        this.utils = new iit.dsl.generator.Common();
        this.transformsModel = transforms;
    }

    override String variableStr(VarLiteral argument) {
        val varname = argument.getValue().getName();
        val paramFQN = iit::dsl::coord::generator::maxima::Maxima::maximaVarNameToParamName(varname);
        if(paramFQN == null) { // the identifier is not a parameter
            val j = utils.getJointFromVariableName(robot, argument.getValue().getName());
            if(j == null) {
                throw new RuntimeException("Did not find any joint corresponding to variable " +
                        argument.getValue().getName() + " in robot " + robot.getName());
            }
            return Common::valueAccessorOf(j).toString();
        }
        // the identifier seems to be a parameter
        val param = iit::dsl::coord::generator::Common::getParameterFromName(transformsModel, paramFQN);
        if(param == null) {
            throw(new RuntimeException("Could not find a parameter called " +
           paramFQN + " in model " + transformsModel.getName()));
        }
        return Jacobians::parameterValueAccessor(param).toString();
    }

    override String cosineStr() {
        return "std::cos";
    }

    override String sineStr() {
        return "std::sin";
    }
}




