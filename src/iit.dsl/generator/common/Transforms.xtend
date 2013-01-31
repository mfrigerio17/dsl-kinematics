package iit.dsl.generator.common

import iit.dsl.TransformsAccessor
import java.io.File
import iit.dsl.kinDsl.Robot
import iit.dsl.generator.FramesTransforms
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.Common

class Transforms {
    private static TransformsAccessor transformsAccessor = new TransformsAccessor()
	private static String path_transformsModel  = "generated_code/misc" // default value
    private static String path_transformsMaxima = "generated_code/maxima"
    private static iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()
    private static Common common = new Common()
    /**
     * Sets the filesystem path of the .ctdsl file.
     * This is the path that will be used to look for the DSL document
     * with the abstract description of the coordinate transformation matrices.
     */
    def static setPath_transformsModel(String path) {
        path_transformsModel = path;
    }
    /**
     * Sets the filesystem path of the Maxima source files.
     * This is the path that will be used to look for generated files with
     * the Maxima implementation of the transformation matrices, which are
     * required to generate the implementation in other languages.
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

    def static iit.dsl.coord.coordTransDsl.Transform getTransform(
        iit.dsl.coord.coordTransDsl.Model model, AbstractLink left, AbstractLink right)
    {
        return coordTransCommon.getTransform(model,
                common.getFrameName(left).toString(),
                common.getFrameName(right).toString())
    }

    def static l1_X_l2__defaultName(iit.dsl.coord.coordTransDsl.Model model,
        AbstractLink l1, AbstractLink l2)
    {
        return coordTransCommon.name(
            coordTransCommon.getTransform(model,
                common.getFrameName(l1).toString(),
                common.getFrameName(l2).toString())  )
    }

    def static l2_X_l1__defaultName(iit.dsl.coord.coordTransDsl.Model model,
        AbstractLink l1, AbstractLink l2)
    {
        return coordTransCommon.name(
            coordTransCommon.getTransform(model,
                common.getFrameName(l2).toString(),
                common.getFrameName(l1).toString())  )
    }





}