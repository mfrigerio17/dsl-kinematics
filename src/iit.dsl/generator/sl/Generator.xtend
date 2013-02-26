package iit.dsl.generator.sl

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {
    extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()
    RobotFiles roboFiles = new RobotFiles()
    RobotUserFiles roboUserFiles = new RobotUserFiles()

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        generateRobotFiles(robot, fsa);
        generateRobotUserFiles(robot, fsa);
    }

     def generateRobotFiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(slCommon.robotFolderName(robot)+"/"+
            Utilities::makefileFolder +"/imakefile.unix",
            roboFiles.makefileUnix(robot))

        fsa.generateFile(slCommon.robotFolderName(robot)+"/"+
            Utilities::dynModelFolder +"/" + robot.name + ".dyn",
            roboFiles.dynModel(robot))
        fsa.generateFile(slCommon.robotFolderName(robot)+"/"+
            Utilities::dynModelFolder +"/" + robot.name + ".nb",
            roboFiles.mathematicaNotebook(robot, "/home/phd/sl_root"))

        fsa.generateFile(slCommon.robotFolderName(robot) + "/include/SL_user.h",
            roboFiles.SL_user_dot_h(robot))

        fsa.generateFile(slCommon.robotFolderName(robot) + "/src/SL_user_common.c",
            roboFiles.SL_user_common_dot_c(robot))

        fsa.generateFile(slCommon.robotFolderName(robot) + "/Makefile", roboFiles.Makefile(robot))
    }

    def generateRobotUserFiles(Robot robot, IFileSystemAccess fsa) {
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/Gains.cf", roboUserFiles.confFile_gains(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorCalibration.cf", roboUserFiles.confFile_sensorCalibration(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/WhichDOFs.cf", roboUserFiles.confFile_whichDOFs(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorOffset.cf", roboUserFiles.confFile_sensorOffset(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/LinkParameters.cf",
            roboUserFiles.linkParameters(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/SensorFilter.cf",
            roboUserFiles.confFile_sensorFilters(robot))

        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/" + Utilities::makefileFolder +"/imakefile.unix",
            roboUserFiles.imakefileUnix(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + benchmarkIDFileName(robot) + ".cpp",
            roboUserFiles.main_benchmarkID(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + main_inertiaM_filename(robot) + ".cpp",
            roboUserFiles.main_inertiaM(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) + "/Makefile", roboUserFiles.Makefile(robot))
    }

}
