package iit.dsl.generator.matlab.config;

import iit.dsl.coord.coordTransDsl.ParameterLiteral;
import iit.dsl.coord.generator.ParameterFQN;

import iit.dsl.kinDsl.Joint;
import iit.dsl.kinDsl.Robot;

import iit.dsl.maxdsl.generator.IIdentifiersReplacement;
import iit.dsl.maxdsl.maximaDsl.VarLiteral;

public class MaximaReplacementStrategy implements IIdentifiersReplacement
{
    private final iit.dsl.generator.Common utils;
    private final Robot robot;
    private final iit.dsl.coord.coordTransDsl.Model transformsModel;
    private final String jointStatusVectorName;
    private final String paramsStructName;

    public MaximaReplacementStrategy(
            Robot rob,
            iit.dsl.coord.coordTransDsl.Model transforms,
            String jointStatusName,
            String paramsStruct)
    {
        robot = rob;
        transformsModel = transforms;
        utils = new iit.dsl.generator.Common();
        jointStatusVectorName = jointStatusName;
        paramsStructName = paramsStruct;
    }

    @Override
    public String variableStr(VarLiteral argument)
    {
        String varname = argument.getValue().getName();
        ParameterFQN paramFQN = iit.dsl.coord.generator.maxima.Maxima.maximaVarNameToParamName(varname);
        if(paramFQN == null) { // the identifier is not a parameter
            Joint j = utils.getJointFromVariableName(robot, argument.getValue().getName());
            if(j == null) {
                throw new RuntimeException("Did not find any joint corresponding to variable " +
                        argument.getValue().getName() + " in robot " + robot.getName());
            }
            return jointStatusVectorName + "(" + utils.getID(j) + ")";
        }
        // the identifier seems to be a parameter
        ParameterLiteral param = iit.dsl.coord.generator.Common.getParameterFromName(transformsModel, paramFQN);
        if(param == null) {
            throw(new RuntimeException("Could not find a parameter called " +
           paramFQN + " in model " + transformsModel.getName()));
        }
        return paramsStructName + "." + iit.dsl.coord.generator.matlab.Generator.parameterValueAccessor(param).toString();
    }

    @Override
    public String cosineStr() {
        return "cos";
    }

    @Override
    public String sineStr() {
        return "sin";
    }



}
