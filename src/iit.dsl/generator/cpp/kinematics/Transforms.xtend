package iit.dsl.generator.cpp.kinematics

import org.eclipse.xtext.generator.IFileSystemAccess

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.cpp.Names


class Transforms {

    def public static child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«child.name»_X_fr_«parent.name»'''
    def public static parent_X_child__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«parent.name»_X_fr_«child.name»'''

    private iit.dsl.coord.generator.cpp.EigenFiles eigenCppTransformsGenerator =
       new iit.dsl.coord.generator.cpp.EigenFiles()

    new() {
         // Configure the Maxima converter that will be used by this generator
        iit::dsl::coord::generator::MaximaConverter::setGenMaximaCodeFolder(
            iit::dsl::generator::common::Transforms::getPath_transformsMaxima()
        );
    }


    def public generate(Robot robot, IFileSystemAccess fsa) {
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot);
        val folder = Names$Files::folder(robot);
        fsa.generateFile(
            folder + "/" + Names$Files::transformsHeader(robot) + ".h",
            eigenCppTransformsGenerator.declarationsFileContent(transformsModel)
        );
        fsa.generateFile(
            folder + "/" + Names$Files::transformsSource(robot) + ".cpp",
            eigenCppTransformsGenerator.implementationsFileContent(transformsModel)
        );
    }
}