package iit.dsl.generator.cpp;

import iit.dsl.kinDsl.Joint;
import iit.dsl.kinDsl.Robot;
import iit.dsl.maxdsl.maximaDsl.VarLiteral;

public class MaximaReplacementSpec implements iit.dsl.maxdsl.generator.IIdentifiersReplacement
{
    private final iit.dsl.generator.Common utils;
    private final Robot robot;

    public MaximaReplacementSpec(Robot rob) {
        this.robot = rob;
        this.utils = new iit.dsl.generator.Common();
    }
    @Override
    public String variableStr(VarLiteral argument) {
        Joint j = utils.getJointFromVariableName(robot, argument.getValue().getName());
        if(j == null) {
            throw new RuntimeException("Did not find any joint corresponding to variable " +
                    argument.getValue().getName() + " in robot " + robot.getName());
        }
        return Common.valueAccessorOf(j).toString();
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
