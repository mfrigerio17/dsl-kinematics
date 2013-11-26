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
        /**
         * This namespace contains facilities related to the parameters of the
         * inertia properties of the robot «robot.name».
         *
         * Inertia parameters are non-constants used in the robot model, where the
         * inertia properties (mass, center of mass, intertia tensor) of the links
         * are specified. Since the value of such parameters must be resolved
         * at runtime, we sometimes refer to them as "runtime parameters", "runtime
         * dynamics parameters", "runtime inertia parameters", etc.
         *
         * Do not confuse them with the "inertia properties" of links, which
         * unfortunately, in the literature, are commonly referred to as
         * "inertia parameters"... Here, the parameters are the non constant
         * fields of the inertia properties.
         */
        namespace «getNamespace» {

            /**
             * A container for the set of non-constant inertia parameters of the robot «robot.name»
             */
            «valuesStructDefinition»

            /**
             * The interface for classes that can compute the actual value of the
             * non-constant inertia parameters of the robot «robot.name».
             */
            «classDefinition»

        }
        }
        «Common::enclosingNamespacesClose(robot)»
        #endif
    '''

    def public defaultGetterHeader() '''
        #ifndef _«robot.name.toUpperCase»_DEFAULT_GETTER_INERTIA_PARAMETERS_
        #define _«robot.name.toUpperCase»_DEFAULT_GETTER_INERTIA_PARAMETERS_

        #include "«Names$Files$RBD::massParametersHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {
        namespace «getNamespace» {

        class DefaultParamsGetter : public «className»
        {
            public:
                DefaultParamsGetter() {
                    resetDefaults();
                }
                ~DefaultParamsGetter() {};

            public:
                void resetDefaults() {
                    «FOR p : params»
                        values.«structFieldName(p)» = 0; // TODO auto-generated -- fix me
                    «ENDFOR»
                }
                «FOR p : params»
                    double «getterFunctionName(p)»() const {
                        return values.«structFieldName(p)»;
                    }
                «ENDFOR»

            private:
                «valuesStructName» values;
        };

        }
        }
        «Common::enclosingNamespacesClose(robot)»
        #endif
    '''


    def protected valuesStructDefinition() '''
        struct «valuesStructName» {
            «FOR p : params»
                double «structFieldName(p)»;
            «ENDFOR»
        };
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