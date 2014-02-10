package iit.dsl.generator.cpp.config


import iit.dsl.coord.coordTransDsl.Model
import iit.dsl.coord.coordTransDsl.Variable

import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common
import iit.dsl.kinDsl.Robot

/**
 * The configurator for the C++ code generator in the Transforms DSL package.
 *
 * This implementation configures the C++ generator of the Transforms DSL such
 * that it generates code consistent with the rest of the robot-specific C++
 * code generated directly by this package.
 *
 * @see iit.dsl.coord.generator.cpp.DefaultConfigurator,
 *      iit.dsl.coord.generator.cpp.IConfigurator
 */
class TransformsGeneratorConfigurator extends iit.dsl.coord.generator.cpp.DefaultConfigurator
{

    new(Robot robot, IConfigurator$Paths pathConfig) {
        this.robot = robot
        maximaConfigurator = new MaximaConverterConfigurator(pathConfig)
    }

    def public setRobot(Robot rob) {
        this.robot = rob
    }

    override headerFileName(Model model) {
        Names$Files::transformsHeader(robot)
    }

    override paramsHeaderFileName(Model model) {
        Names$Files::parametersHeader(robot)
    }

    override includeDirectives(Model model)
        '''
        #include <Eigen/Dense>
        #include <iit/rbd/TransformsBase.h>
        #include "«Names$Files::mainHeader(robot)».h"
        '''

    override enclosingNamespaces(Model model) {
        val enclosing = Names$Namespaces::enclosing
        enclosing.add(Names$Namespaces::rob(robot))
        return enclosing
    }


    override className(iit.dsl.coord.generator.Utilities$MatrixType mxtype) {
        switch(mxtype) {
             case iit::dsl::coord::generator::Utilities$MatrixType::_6D:         Names$Types$Transforms::spatial_motion
             case iit::dsl::coord::generator::Utilities$MatrixType::_6D_FORCE:   Names$Types$Transforms::spatial_force
             case iit::dsl::coord::generator::Utilities$MatrixType::HOMOGENEOUS: Names$Types$Transforms::homogeneous
             default: throw(new RuntimeException("Unknown MatrixType: " + mxtype))
         }
    }

//    override localTypeName(MatrixType matrixtype) {
//        throw new UnsupportedOperationException("Auto-generated function stub")
//    }


    override valueExpression(Model model, Variable arg) {
        val joint = common.getJointFromVariableName(robot, arg.varname)
        if(joint == null){
            throw new RuntimeException("Could not find the joint corresponding to "+
                "variable " + arg.varname + ", for the robot " + robot.name)
        }
        '''«variablesStatusVector_varname(model)»(«Common::jointIdentifier(joint)»)'''
    }

    override variablesStatusVector_type(Model model)
        '''«Common::enclosingNamespacesQualifier(robot)»::«Names$Types::jointState»'''


    override variablesStatusVector_varname(Model model) {
        "q"
    }

    override getMaximaConverterConfigurator() {
        return maximaConfigurator
    }



    private iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private Robot robot = null
    private MaximaConverterConfigurator maximaConfigurator = null

}

