package iit.dsl.generator.esrocos

import java.util.List

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.common.gr.RobotPoseUtils.ConstantPose
import iit.dsl.generator.common.gr.RelativePose

class RCGCompare
{
    def mainSrc(Robot robot, List<ConstantPose> constPoses, List<RelativePose> outputs) '''
    #include <iit/robots/«robot.name.toLowerCase»/declarations.h>
    #include <iit/robots/«robot.name.toLowerCase»/transforms.h>

    #include <iostream>
    #include "fk.h"

    using namespace std;
    using namespace iit;

    static struct mc_config mc;
    «val ns = robot.name»
    void transforms(«ns»::HomogeneousTransforms& xh)
    {
        «ns»::JointState q;
        q.setRandom();

        joint_state input = q;

        «FOR op : outputs»
            pose_t «op.name»;
        «ENDFOR»
        fk_compute(mc, input, «FOR op:outputs SEPARATOR","»«op.name»«ENDFOR»);

        «FOR op : outputs»
            cout << «op.name» - xh.«rcgName(op)»(q) << endl;
            std::cout << "---------------" << std::endl;
        «ENDFOR»
    }


    void constants(const «ns»::HomogeneousTransforms& xh)
    {
        mc_config kk;
        «FOR c : constPoses»
            cout << kk.«c.name» - xh.«rcgName(c)» << endl;
            std::cout << "---------------" << std::endl;
        «ENDFOR»
    }


    int main(int argc, char** argv)
    {
        «ns»::HomogeneousTransforms xh;
        constants(xh);
        transforms(xh);
        return 0;
    }
    '''

    def private rcgName(RelativePose pose) '''«pose.reference.frame.name»_X_«pose.target.frame.name»'''
}