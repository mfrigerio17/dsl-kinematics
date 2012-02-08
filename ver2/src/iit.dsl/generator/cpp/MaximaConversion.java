package iit.dsl.generator.cpp;

import iit.dsl.coord.coordTransDsl.VariableLiteral;
import iit.dsl.coord.generator.IMaximaConversionSpec;
import iit.dsl.kinDsl.Robot;

public class MaximaConversion implements IMaximaConversionSpec {
    private final iit.dsl.generator.Common utils;
    private final Robot robot;

    MaximaConversion(Robot rob) {
        this.robot = rob;
        this.utils = new iit.dsl.generator.Common();
    }

    @Override
    public String cosineStr(VariableLiteral argument) {
        return Common.variableForCosineOf( utils.getJointFromVariableName(robot, argument.getVarname()) ).toString();
    }

    @Override
    public String sineStr(VariableLiteral argument) {
        return Common.variableForSineOf( utils.getJointFromVariableName(robot, argument.getVarname()) ).toString();
    }

    @Override
    public String variableStr(VariableLiteral argument) {
        return Common.valueAccessorOf( utils.getJointFromVariableName(robot, argument.getVarname()) ).toString();
    }

    @Override
    public String cosineStr() {
        return "std::cos";
    }

    @Override
    public String sineStr() {
        return "std::sin";
    }

}
