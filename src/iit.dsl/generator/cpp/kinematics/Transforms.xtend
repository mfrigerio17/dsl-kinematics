package iit.dsl.generator.cpp.kinematics

import org.eclipse.xtext.generator.IFileSystemAccess

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.cpp.Names

/**
 * Main generator of C++ code for coordinate transforms.
 *
 * This generator relies in turn on the code generator of the Transforms DSL
 * package.
 */
class Transforms {

    def public static child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«child.name»_X_fr_«parent.name»'''
    def public static parent_X_child__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«parent.name»_X_fr_«child.name»'''


    /**
     */
    new(iit.dsl.coord.generator.cpp.IConfigurator configurator) {
       setTransformsDSLConfigurator(configurator)
    }

    /**
     * Sets the iit.dsl.coord.generator.cpp.IConfigurator object
     * that will be used by this generator
     */
    def public void setTransformsDSLConfigurator(
        iit.dsl.coord.generator.cpp.IConfigurator configurator)
    {
        if(configurator == null) {
            System::err.println("Transforms::setTransformsDSLConfigurator() : null configurator")
            //TODO log warning
            return
        }
        eigenCppTransformsGenerator =
            new iit.dsl.coord.generator.cpp.EigenFiles(configurator)
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

    private iit.dsl.coord.generator.cpp.EigenFiles eigenCppTransformsGenerator = null
}