package iit.dsl.generator.cpp.config


import iit.dsl.coord.coordTransDsl.Model
import iit.dsl.coord.coordTransDsl.VariableLiteral
import iit.dsl.coord.generator.Utilities$MatrixType
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

    override includeDirectives(Model model)
        '''
        #include <Eigen/Dense>
        #include <iit/rbd/JStateDependentMatrix.h>
        #include "«Names$Files::mainHeader(robot)».h"
        '''

    override enclosingNamespaces(Model model) {
        val enclosing = Names$Namespaces::enclosing
        enclosing.add(Names$Namespaces::rob(robot))
        return enclosing
    }

//    override localTypeName(MatrixType matrixtype) {
//        throw new UnsupportedOperationException("Auto-generated function stub")
//    }

    override matrixType(Model model, MatrixType transformtype) {
        Names$Types::jstateDependentMatrix(robot, transformtype.size)
    }

    override namespace(MatrixType matrixtype) {
        switch(matrixtype) {
            case Utilities$MatrixType::_6D: Names$Namespaces::transforms6D
            case Utilities$MatrixType::_6D_FORCE: Names$Namespaces::T6D_force
            case Utilities$MatrixType::HOMOGENEOUS: Names$Namespaces::THomogeneous
            default: throw(new RuntimeException("Unknown MatrixType: " + matrixtype))
        }
    }

    override valueExpression(Model model, VariableLiteral arg) {
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

