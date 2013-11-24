package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.ParameterLiteral
import iit.dsl.kinDsl.Robot
import iit.dsl.generator.common.Parameters

import java.util.List
import iit.dsl.generator.cpp.Common
import iit.dsl.generator.cpp.Names

class InertiaParameters {

    public static CharSequence className  = '''RuntimeParamsGetter'''
    public static CharSequence valuesStructName = '''RuntimeInertiaParams'''

    def public static getterFunctionName(ParameterLiteral p)
        '''getValue_«p.varname»'''
    def public static structFieldName(ParameterLiteral p)
        '''«p.varname»'''

    def public static getNamespace() '''params'''


    new(Robot r) {
        robot = r
        params = Parameters::getInertiaParameters(r)
    }


    def public headerFileContent() '''
        #ifndef _«robot.name.toUpperCase»_RUNTIME_INERTIA_PARAMETERS_
        #define _«robot.name.toUpperCase»_RUNTIME_INERTIA_PARAMETERS_

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {
        namespace «getNamespace» {

            /**
             * A container for the set of non-constant inertia properties of the robot «robot.name»
             */
            «valuesStructDefinition»

            /**
             * The interface for classes that can compute the actual value of the
             * non-constant inertia properties of the robot «robot.name».
             */
            «classDefinition»

        }
        }
        «Common::enclosingNamespacesClose(robot)»
        #endif
    '''



    def protected valuesStructDefinition() '''
        typedef struct _«valuesStructName» {
            «FOR p : params»
                double «structFieldName(p)»;
            «ENDFOR»
        } «valuesStructName»;
    '''

    def protected classDefinition() '''
        class «className» {
            public:
            «FOR p : params»
                virtual double «getterFunctionName(p)»() const { return 0; }
            «ENDFOR»
        };
    '''

    private Robot robot = null
    private List<ParameterLiteral> params = null;
}