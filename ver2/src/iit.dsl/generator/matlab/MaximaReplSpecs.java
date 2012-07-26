package iit.dsl.generator.matlab;

import iit.dsl.kinDsl.Joint;
import iit.dsl.kinDsl.Robot;
import iit.dsl.maxdsl.generator.IIdentifiersReplacement;
import iit.dsl.maxdsl.maximaDsl.VarLiteral;

public class MaximaReplSpecs implements IIdentifiersReplacement {

    private final iit.dsl.generator.Common utils;
    private final Robot robot;

    public MaximaReplSpecs(Robot rob) {
        robot = rob;
        utils = new iit.dsl.generator.Common();
    }
    @Override
    public String variableStr(VarLiteral argument) {
        Joint j = utils.getJointFromVariableName(robot, argument.getValue().getName());
        if(j == null) {
            throw new RuntimeException("Did not find any joint corresponding to variable " +
                    argument.getValue().getName() + " in robot " + robot.getName());
        }
        return "q(" + utils.getID(j) + ")";
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
