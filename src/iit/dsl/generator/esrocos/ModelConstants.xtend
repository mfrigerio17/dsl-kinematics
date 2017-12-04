package iit.dsl.generator.esrocos

import java.util.List
import iit.dsl.generator.common.gr.RobotPoseUtils.ConstantPose
import org.eclipse.xtend2.lib.StringConcatenation

class ModelConstants
{
    public new(List<ConstantPose> list) {
        constPoses = list
    }


    def asLuaTable() '''
    return  {
        «FOR c : constPoses SEPARATOR ","»
            «c.name()» = {
                p = {«c.refToTarget.toString()»},
                r = {«listElementsAsText(c.ref_R_target)»}
            }
        «ENDFOR»
    }
    '''

    def private listElementsAsText(double[][] mx) {
        val text = new StringConcatenation();
        for (row : mx) {
            for (el : row) {
                text.append( String.format("%.6f", el) + ",");
            }
        }
        return text.subSequence(0, text.length-2)
    }


    private List<ConstantPose> constPoses = null
}