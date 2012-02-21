package iit.dsl.generator.sl

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject

import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {
    extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()
    @Inject RobotFiles roboFiles
    @Inject RobotUserFiles roboUserFiles

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;

        // ROBOT FILES
/*
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
            roboFiles.SL_user_dot_h(robot))
*/
        // ROBOT USER FILES
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/config/LinkParameters.cf",
            roboUserFiles.linkParameters(robot))
///*
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/" + Utilities::makefileFolder +"/imakefile.unix",
            roboUserFiles.imakefileUnix(robot))
        fsa.generateFile(slCommon.robotUserFolderName(robot) +
            "/src/" + benchmarkFileName(robot) + ".cpp",
            roboUserFiles.benchmarkMain(robot))
//*/
    }
}
