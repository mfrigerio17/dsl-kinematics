package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import java.util.List
import iit.dsl.generator.Jacobian
import org.eclipse.xtend2.lib.StringConcatenation
import java.util.Iterator

class Jacobians {

    public new() {
        maximaConverter = new iit.dsl.generator.maxima.Converter()
        maxdslAccess    = new iit.dsl.maxdsl.utils.DSLAccessor()
        maxdslUtils     = new iit.dsl.maxdsl.generator.matlab.Utils()
    }
    /**
     * This constructor takes the maxima.Converter configurator that will be
     * used to configure the maxima.Converter used by this instance
     */
    public new(iit.dsl.generator.maxima.IConverterConfigurator conf) {
        this()
        setMaximaConverterConfigurator(conf)
    }

    def public setMaximaConverterConfigurator(
        iit.dsl.generator.maxima.IConverterConfigurator conf)
    {
        maximaConverter.setConfigurator(conf)
    }

    def init_jacobians_file(Robot robot, List<Jacobian> jacs) '''
        «FOR Jacobian j : jacs»
            «val jText = maximaConverter.getJacobianText(j)»
            «j.name» = zeros(«j.rows»,«j.cols»);
            «maxdslUtils.staticInitAssignements(jText, j.name)»

        «ENDFOR»
    '''

    def update_jacobians_file(Robot robot, List<Jacobian> jacs) {
        val StringConcatenation strBuff = new StringConcatenation();
        val replaceSpecs = new MaximaReplSpecs(robot)
        val identifiers = iit::dsl::maxdsl::generator::Identifiers::getInstance()
        var r = 1 // row index
        var c = 1 // column index

        for (J : jacs) {
            val jText     = maximaConverter.getJacobianText(J);
            val maxdslDoc = iit::dsl::generator::maxima::MaximaDSLUtils::MaximaDSLDocumentText(J, jText)
            //Check if the current Jacobian is actually a function of something. If not, it is a constant,
            // and no code generation is required here
            if(maxdslDoc.length > 0) {
                val iit.dsl.maxdsl.maximaDsl.Model expressionsModel =
                        maxdslAccess.getParsedTextModel(maxdslDoc.toString())
                val Iterator<iit.dsl.maxdsl.maximaDsl.Expression> exprIter = expressionsModel.expressions.iterator

                // declarations of variables for trigonometric functions and assignements:
                strBuff.append(maxdslUtils.trigFunctionsCode(expressionsModel, replaceSpecs))
                strBuff.append("\n");
                r = 1
                c = 1
                for(row : jText) {
                    for(el : row) {
                        if( !iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el)) {
                            // we assume we as many parsed expressions as the number of non constant elements of the Jacobian
                            if( ! exprIter.hasNext()) {
                                throw new RuntimeException("The number of expressions does not match the Jacobian")
                            }
                            strBuff.append('''«J.name»(«r»,«c») = «identifiers.toCode(exprIter.next(), replaceSpecs)»;
                            ''')
                        }
                        c = c+1
                    }
                    r = r+1 // next row
                    c = 1   // back to first colulmn
                }
                strBuff.append("\n\n");
            }
        }
        return strBuff
    }

    private iit.dsl.generator.maxima.Converter  maximaConverter = null
    private iit.dsl.maxdsl.utils.DSLAccessor      maxdslAccess  = null
    private iit.dsl.maxdsl.generator.matlab.Utils maxdslUtils   = null
}