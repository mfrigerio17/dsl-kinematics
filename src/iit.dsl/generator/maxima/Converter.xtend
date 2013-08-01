package iit.dsl.generator.maxima

import iit.dsl.generator.Jacobian


/**
 * This class should be used to convert the output of the Maxima code generated
 * by the generator in this package, to some other form.
 */
class Converter {
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
     * Returns a matrix of strings representing the given Jacobian matrix.
     * The Maxima source file with the definition of the Jacobian is retrieved
     * and interpreted, and the output is parsed and returned.
     * @param J the placeholder for the Jacobian matrix of interest
     * @transformsModel the model of the Transforms-DSL, with the definition of
     *                  the coordinate transformations of the same robot the
     *                  given Jacobian refers to
     * @return a String matrix (array of arrays) with the textual representation
     *         of the actual elements of the Jacobian
     */
    def String[][] getJacobianText(Jacobian J, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        if(transformsModel == null) {
            throw new RuntimeException("Cannot convert the Jacobian to text, the given transforms-model is null.")
        }
        val maximaRunner = new iit.dsl.maxdsl.utils.MaximaRunner(maximaCfg)
        maximaRunner.runBatch(maximaLibTransforms)
        maximaRunner.runBatch(maximaLibUtils)

        maximaRunner.runBatch(maximaTransformsPath + "/" +
            iit::dsl::coord::generator::maxima::Maxima::transformsFileName(transformsModel)
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
