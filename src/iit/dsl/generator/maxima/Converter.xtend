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
        maximaLibJacobians   = config.libsPath + "/" + config.jacobiansLibName
        maximaLibUtils       = config.libsPath + "/" + config.utilsLibName
        maximaJacobiansPath  = config.generatedCodeLocation
        maximaTransformsPath = config.generatedCodeLocation
        maximaCfg = config.maximaEngineConfigurator
        mode = iit::dsl::coord::generator::MaximaConverter::parseStringMode(config.conversionMode)
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
        maximaRunner.runBatch(maximaLibJacobians)
        maximaRunner.runBatch(maximaLibUtils)

        maximaRunner.runBatch(maximaTransformsPath + "/" +
            iit::dsl::coord::generator::maxima::Maxima::transformsFileName(transformsModel)
        )
        maximaRunner.runBatch(maximaJacobiansPath +  "/" + Jacobians::fileName(J.robot))

        val jacName = '''«J.getName()»(«J.getArgsList()»)'''
        var String code
        switch mode {
            case iit::dsl::coord::generator::MaximaConverter$Mode::PLAIN:
                code = '''J : «jacName»;'''

            case iit::dsl::coord::generator::MaximaConverter$Mode::TRIGREDUCE:
                code = '''J : float(ratexpand(ratsimp(trigreduce(«jacName»))));'''

            case iit::dsl::coord::generator::MaximaConverter$Mode::TRIGSIMP:
                code = '''J : float(trigsimp(«jacName»));'''

            case iit::dsl::coord::generator::MaximaConverter$Mode::__UNKNOWN:
                throw new RuntimeException("Unknown Maxima conversion mode.")

            default:
                throw new RuntimeException("Unknown Maxima conversion mode.")
        }

        maximaRunner.run(code.toString())
        val ret = iit::dsl::maxdsl::utils::MaximaConversionUtils::getMatrix(maximaRunner, "J", 6, J.cols);

        maximaRunner.terminate();
        return ret
    }

    //filesytem location where to find required code to set up the environment
    private String maximaLibTransforms  = null
    private String maximaLibJacobians   = null
    private String maximaLibUtils       = null
    private String maximaJacobiansPath  = null
    private String maximaTransformsPath = null
    private iit.dsl.maxdsl.utils.MaximaRunner$IConfigurator maximaCfg = null
    private iit.dsl.coord.generator.MaximaConverter$Mode  mode =
        iit::dsl::coord::generator::MaximaConverter$Mode::PLAIN;

}

class DefaultConfigurator
    extends iit.dsl.coord.generator.maxima.DefaultConfigurator
    implements IConverterConfigurator
{
    override getUtilsLibName() {
        return "utils"
    }

    override getJacobiansLibName() {
        return "jacobians"
    }
}
