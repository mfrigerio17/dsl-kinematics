package iit.dsl.generator.maxima

import com.google.inject.Inject

import iit.dsl.TransformsAccessor
import iit.dsl.coord.generator.Maxima
import iit.dsl.generator.Jacobian

/**
 * This class should be used to convert the output of the Maxima code generated
 * by the generator in this package, to some other form.
 */
class Converter {
    @Inject TransformsAccessor transformsAccessor

    //filesytem location where to find required code to set up the environment
    static String maximaBasePath = System::getenv("MAXIMA_LIBS_PATH")
    static String maximaJacobiansPath = "generated_code/maxima"

    /**
     * Returns a matrix of strings representing a Jacobian matrix.
     * The Maxima source file with the corresponding definition is retrieved
     * and executed, and its output is parsed and returned.
     */
    def String[][] getJacobianText(Jacobian J)
    {
        val maximaRunner = new iit.dsl.maxdsl.utils.MaximaRunner()
        maximaRunner.runBatch(maximaBasePath+"/coord_transforms")
        maximaRunner.runBatch(maximaBasePath+"/utils")

        val iit.dsl.coord.coordTransDsl.Model transforms = transformsAccessor.getTransformsModel(J.robot)
        maximaRunner.runBatch(maximaBasePath +"/robots/" + Maxima::transformsFileName(transforms))
        maximaRunner.runBatch(maximaJacobiansPath + "/" + Jacobians::fileName(J.robot))

        val jacName = '''«J.getName()»(«J.getArgsList()»)'''
        val code = '''J : float(ratexpand(ratsimp(trigreduce(«jacName»))));'''
        maximaRunner.run(code.toString())
        val ret = iit::dsl::maxdsl::utils::MaximaConversionUtils::getMatrix(maximaRunner, "J", 6, J.cols);

        maximaRunner.terminate();
        return ret
    }
}