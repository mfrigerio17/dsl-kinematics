package iit.dsl.generator.sl

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess
import iit.dsl.kinDsl.Robot


class Generator implements IGenerator {
    extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()
    RobotUserFiles roboUserFiles = new RobotUserFiles()
    iit.dsl.generator.sl.robot.math.Headers mathFilesGen = new iit.dsl.generator.sl.robot.math.Headers()
    iit.dsl.generator.sl.robot.Headers roboHeadGen = new iit.dsl.generator.sl.robot.Headers()
    iit.dsl.generator.sl.robot.Sources roboSrcGen = new iit.dsl.generator.sl.robot.Sources()
    iit.dsl.generator.sl.robot.Makefiles roboMake = new iit.dsl.generator.sl.robot.Makefiles()
    iit.dsl.generator.sl.robotUser.ConfigFiles userConfigGen = new iit.dsl.generator.sl.robotUser.ConfigFiles()
    iit.dsl.generator.sl.robotUser.Makefiles userMakeGen = new iit.dsl.generator.sl.robotUser.Makefiles()

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        generateRobotFiles(robot, fsa);
        generateRobotUserFiles(robot, fsa);
        //System::out.println(roboSrcGen.kinematics(robot))
    }

     def generateRobotFiles(Robot robot, IFileSystemAccess fsa) {
        val folder = slCommon.robotFolderName(robot)

        fsa.generateFile(folder + "/include/SL_user.h",
            roboHeadGen.SL_user(robot))

        fsa.generateFile(folder + "/include/" + Common::robcogenGlobalsFileName(robot) + ".h",
            roboHeadGen.robcogenGlobals(robot)   )

        fsa.generateFile(folder + "/src/SL_user_common.cpp",
            roboSrcGen.SL_user_common(robot))
        fsa.generateFile(folder + "/src/SL_kinematics.cpp",
            roboSrcGen.kinematics(robot))
        fsa.generateFile(folder + "/src/SL_dynamics.cpp",
            roboSrcGen.dynamics(robot))

        fsa.generateFile(folder + "/CMakeLists.txt", roboMake.CMakeLists(robot))


        val math_dir = folder + "/" + Common::mathFolderName

        fsa.generateFile(
            math_dir + "/Prismatic_Joints.h",
            mathFilesGen.prismatic_joints(robot))
        fsa.generateFile(
            math_dir + "/Floating_Base.h",
            mathFilesGen.floating_base(robot))
        fsa.generateFile(
            math_dir + "/OpenGL.h",
            mathFilesGen.opengl(robot))
        fsa.generateFile(
            math_dir + "/GJac_declare.h",
            mathFilesGen.gjac_declare(robot))
        fsa.generateFile(
            math_dir + "/GJac_math.h",
            mathFilesGen.gjac_math(robot))
        fsa.generateFile(
            math_dir + "/Contact_GJac_declare.h",
            mathFilesGen.contact_gjac_declare(robot))
        fsa.generateFile(
            math_dir + "/Contact_GJac_math.h",
            mathFilesGen.contact_gjac_math(robot))
    }

    def generateRobotUserFiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/Gains.cf", userConfigGen.gains(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorCalibration.cf", userConfigGen.sensorCalibration(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/WhichDOFs.cf", userConfigGen.whichDOFs(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorOffset.cf", userConfigGen.sensorOffset(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/LinkParameters.cf",
            userConfigGen.linkParameters(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorFilter.cf",
            userConfigGen.sensorFilters(robot))

        fsa.generateFile(slCommon.robotUserFolderName(robot) + "/CMakeLists.txt", userMakeGen.CMakeLists(robot))

        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + benchmarkIDFileName(robot) + ".cpp",
            roboUserFiles.main_benchmarkID(robot))

        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + main_inertiaM_filename(robot) + ".cpp",
            roboUserFiles.main_inertiaM(robot))

        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + main_compare_fwd_dyn(robot) + ".cpp",
            roboUserFiles.main_compare_fwd_dyn(robot))
    }

}
