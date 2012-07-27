package iit.dsl.generator.common

import iit.dsl.TransformsAccessor
import java.io.File
import iit.dsl.kinDsl.Robot
import iit.dsl.generator.FramesTransforms

class Transforms {
    private static TransformsAccessor transformsAccessor = new TransformsAccessor()
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

    def static getPath_transformsMaxima() {
        return path_transformsMaxima;
    }

    def static iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot) {
        val File ctdslFile = new File(path_transformsModel + "/" + FramesTransforms::fileName(robot));
        return  transformsAccessor.getTransformsModel(robot, ctdslFile);
    }





}