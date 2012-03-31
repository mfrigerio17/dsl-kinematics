package iit.dsl.generator.maxima

import iit.dsl.TransformsAccessor
import iit.dsl.kinDsl.Robot
import org.eclipse.xtext.generator.IFileSystemAccess
import java.io.File
import iit.dsl.generator.FramesTransforms

class Transforms {
	private static String transformsModelPath = "generated_code/misc" // default value
    /**
     * Sets the filesystem path where to look for the DSL document with the description
     * of the coordinate transformation matrices
     */
    def static setTransformsModelPath(String path) {
        transformsModelPath = path;
    }

    private TransformsAccessor transformsAccessor = new TransformsAccessor()
    private iit.dsl.coord.generator.CoordTransDslGenerator gen =
            new iit.dsl.coord.generator.CoordTransDslGenerator()

    def public generate(Robot robot, IFileSystemAccess fsa) {
        val File ctdslFile = new File(transformsModelPath + "/" + FramesTransforms::fileName(robot));
        val iit.dsl.coord.coordTransDsl.Model transformsModel =
           transformsAccessor.getTransformsModel(robot, ctdslFile);

        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transformsFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima())
        );
        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transforms6DFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima6D())
        );
        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transforms6DForceFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima6DForce())
        );
    }
}