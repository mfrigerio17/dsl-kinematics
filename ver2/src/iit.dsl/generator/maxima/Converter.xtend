package iit.dsl.generator.maxima


import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RefFrame
import iit.dsl.TransformsAccessor

import iit.dsl.coord.generator.Maxima

import com.google.inject.Inject
import java.util.List
import iit.dsl.generator.Jacobian

/**
 * This class should be used to convert the output of the Maxima code generated
 * by the generator in this package, to some other form.
 */
class Converter {
    @Inject TransformsAccessor transformsAccessor
    @Inject iit.dsl.coord.generator.Maxima coord_maxima
    @Inject iit.dsl.coord.generator.Common coord_common

	iit.dsl.coord.generator.MaximaRunner maximaRunner = null

    //filesytem location where to find required code to set up the environment
    static String maximaBasePath = System::getenv("MAXIMA_LIBS_PATH")
    static String maximaJacobiansPath = "generated_code/maxima"

    /**
     * Returns a matrix of strings representing a Jacobian matrix.
     * The Jacobian is identified by two reference frames; the Maxima source file
     * with the corresponding definition is executed, and its output is parsed
     * and returned.
     */
    def String[][] getJacobianText(Robot robot, RefFrame baseFrame, RefFrame movingFrame,
        iit.dsl.coord.generator.IMaximaConversionSpec convSpec)
    {
        maximaRunner = new iit.dsl.coord.generator.MaximaRunner()
        maximaRunner.runBatch(maximaBasePath+"/coord_transforms")
        maximaRunner.runBatch(maximaBasePath+"/utils")

        val iit.dsl.coord.coordTransDsl.Model transforms = transformsAccessor.getTransformsModel(robot)

        maximaRunner.runBatch(maximaBasePath +"/robots/" + Maxima::transformsFileName(transforms))
        maximaRunner.runBatch(maximaJacobiansPath + "/" + Jacobians::fileName(robot))

        val List<iit.dsl.coord.coordTransDsl.VariableLiteral> variables =
            coord_common.getVars(coord_common.getTransform(transforms, baseFrame.name, movingFrame.name))
        val jacName = '''«Jacobians::jacName(baseFrame.name, movingFrame.name)»(«coord_maxima.argsListText(variables)»)'''
        val code = '''J : float(ratexpand(ratsimp(trigreduce(«jacName»))));'''
        maximaRunner.run(code.toString())
        val ret = iit::dsl::coord::generator::MaximaConverter::parseMatrix(maximaRunner, convSpec, "J", 6, variables.size(), variables);

        maximaRunner.terminate();
        maximaRunner = null;
        return ret
    }

    def String[][] getJacobianText(Jacobian J, iit.dsl.coord.generator.IMaximaConversionSpec convSpec)
    {
        return getJacobianText(J.robot, J.baseFrame, J.movingFrame, convSpec)
    }
}