package iit.dsl.generator.maxima

import iit.dsl.generator.Jacobian
import iit.dsl.generator.FramesTransforms
import iit.dsl.TransformsAccessor

import java.io.File

/**
 * This class should be used to convert the output of the Maxima code generated
 * by the generator in this package, to some other form.
 */
class Converter {
    private static TransformsAccessor transformsAccessor = new TransformsAccessor()

    /**
     * Default constructor, uses the default configurator.
     */
    public new() {
        this(new DefaultConfigurator)
    }

    /**
     * @param config the iit.dsl.generator.maxima.IConverterConfigurator to be
     *        used by this instance.
     */
    public new(IConverterConfigurator config) {
        setConfigurator(config)
    }

    def public setConfigurator(IConverterConfigurator config) {
        maximaLibTransforms  = config.libsPath + "/" + config.transformsLibName
        maximaLibUtils       = config.libsPath + "/" + config.utilsLibName
        maximaJacobiansPath  = config.generatedCodeLocation
        maximaTransformsPath = config.generatedCodeLocation
        transformsDSLFilesPath = config.transformsDSLFilesPath
        maximaCfg = config.maximaEngineConfigurator
    }

    /**
     * Returns a matrix of strings representing a Jacobian matrix.
     * The Maxima source file with the corresponding definition is retrieved
     * and executed, and its output is parsed and returned.
     */
    def String[][] getJacobianText(Jacobian J)
    {
        val maximaRunner = new iit.dsl.maxdsl.utils.MaximaRunner(maximaCfg)
        maximaRunner.runBatch(maximaLibTransforms)
        maximaRunner.runBatch(maximaLibUtils)

        val File transformsFile = new File(
            transformsDSLFilesPath + "/" + FramesTransforms::fileName(J.robot)
        );
        maximaRunner.runBatch(maximaTransformsPath + "/" +
            iit::dsl::coord::generator::maxima::Maxima::transformsFileName(
                transformsAccessor.getTransformsModel(J.robot, transformsFile)
            )
        )
        maximaRunner.runBatch(maximaJacobiansPath +  "/" + Jacobians::fileName(J.robot))

        val jacName = '''«J.getName()»(«J.getArgsList()»)'''
        val code = '''J : float(ratexpand(ratsimp(trigreduce(«jacName»))));'''
        maximaRunner.run(code.toString())
        val ret = iit::dsl::maxdsl::utils::MaximaConversionUtils::getMatrix(maximaRunner, "J", 6, J.cols);

        maximaRunner.terminate();
        return ret
    }

    //filesytem location where to find required code to set up the environment
    private String maximaLibTransforms  = null
    private String maximaLibUtils       = null
    private String maximaJacobiansPath  = null
    private String maximaTransformsPath = null
    private String transformsDSLFilesPath = null
    private iit.dsl.maxdsl.utils.MaximaRunner$IConfigurator maximaCfg = null

}

class DefaultConfigurator
    extends iit.dsl.coord.generator.maxima.DefaultConfigurator
    implements IConverterConfigurator
{
    override getTransformsDSLFilesPath() {
        return "generated_code/misc"
    }

    override getUtilsLibName() {
        return "utils"
    }
}
