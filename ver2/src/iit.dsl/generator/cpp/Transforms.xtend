package iit.dsl.generator.cpp

import iit.dsl.TransformsAccessor
import iit.dsl.kinDsl.Robot
import org.eclipse.xtext.generator.IFileSystemAccess
import java.io.File
import iit.dsl.generator.FramesTransforms

class Transforms {
    private static String path_transformsModel  = "generated_code/misc" // default value
    private static String path_transformsMaxima = "generated_code/maxima"
    /**
     * Sets the filesystem path of the .ctdsl file.
     * This is the path that will be used to look for the DSL document
     * with the abstrac description of the coordinate transformation matrices.
     */
    def static setPath_transformsModel(String path) {
        path_transformsModel = path;
    }
    /**
     * Sets the filesystem path of the Maxima source files.
     * This is the path that will be used to look for generated files with
     * the Maxima implementation of the transformation matrices, which are
     * required to generate the C++ implementation.
     */
    def static setPath_transformsMaxima(String path) {
        path_transformsMaxima = path;
    }

	private TransformsAccessor transformsAccessor = new TransformsAccessor()
	private iit.dsl.coord.generator.cpp.EigenFiles eigenCppTransformsGenerator =
	   new iit.dsl.coord.generator.cpp.EigenFiles()


	
	def public generate(Robot robot, IFileSystemAccess fsa) {
	    // Configure the Maxima converter that will be used by the generator
	    iit::dsl::coord::generator::MaximaConverter::setGenMaximaCodeFolder(path_transformsMaxima);
	
	    val File ctdslFile = new File(path_transformsModel + "/" + FramesTransforms::fileName(robot));
        val iit.dsl.coord.coordTransDsl.Model transformsModel =
           transformsAccessor.getTransformsModel(robot, ctdslFile);
	
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