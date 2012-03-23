package iit.dsl.generator.maxima

import iit.dsl.generator.Jacobian
import iit.dsl.kinDsl.Robot

/**
 * This class should be used to convert the output of the Maxima code generated
 * by the generator in this package, to some other form.
 */
class Converter {
    //filesytem location where to find required code to set up the environment
    static String maximaLibsPath = System::getenv("MAXIMA_LIBS_PATH")
    static String maximaJacobiansPath = "generated_code/maxima"
    static String maximaTransformsPath = maximaLibsPath + "/robots"

    def public static setMaximaLibsPath(String path) {
        maximaLibsPath = path;
    }
    def public static setMaximaJacobiansPath(String path) {
        maximaJacobiansPath = path;
    }
    def public static setMaximaTransformsPath(String path) {
        maximaTransformsPath = path;
    }

    /**
     * Returns a matrix of strings representing a Jacobian matrix.
     * The Maxima source file with the corresponding definition is retrieved
     * and executed, and its output is parsed and returned.
     */
    def String[][] getJacobianText(Jacobian J)
    {
        val maximaRunner = new iit.dsl.maxdsl.utils.MaximaRunner()
        maximaRunner.runBatch(maximaLibsPath+"/coord_transforms")
        maximaRunner.runBatch(maximaLibsPath+"/utils")

        maximaRunner.runBatch(maximaTransformsPath + "/" + transformsFileName(J.robot))
        maximaRunner.runBatch(maximaJacobiansPath +  "/" + Jacobians::fileName(J.robot))

        val jacName = '''«J.getName()»(«J.getArgsList()»)'''
        val code = '''J : float(ratexpand(ratsimp(trigreduce(«jacName»))));'''
        maximaRunner.run(code.toString())
        val ret = iit::dsl::maxdsl::utils::MaximaConversionUtils::getMatrix(maximaRunner, "J", 6, J.cols);

        maximaRunner.terminate();
        return ret
    }

    /**
     *
     * This must be kept consistent with the file generated by the coordTrans DSL
     */
    def String transformsFileName(Robot robot) {
        return robot.name + "_transforms.maxima"
    }
}