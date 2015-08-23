package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Jacobian
import iit.dsl.generator.matlab.config.IConfigurator

import org.eclipse.xtend2.lib.StringConcatenation

import java.util.List
import java.util.ArrayList


class Jacobians {

    public static CharSequence updateFunctionName = '''updateJacobians'''
    public static CharSequence initFunctionName   = '''initJacobians'''

    public new(IConfigurator config)
    {
        configurator    = config
        replaceSpecs    = configurator.maximaReplacementStrategy
        maximaConverter = new iit.dsl.generator.maxima.Converter(
            configurator.kindslMaximaConverterConfigurator )
        maxdslAccess    = new iit.dsl.maxdsl.utils.DSLAccessor()
        maxdslUtils     = new iit.dsl.maxdsl.generator.matlab.Utils()
    }

    def public setMaximaConverterConfigurator(
        iit.dsl.generator.maxima.IConverterConfigurator conf)
    {
        maximaConverter.setConfigurator(conf)
    }

    def public getJacobiansLocalVarName() { return jacobiansStructName }


    /**
     * The complete content of the file with Matlab function to initialize the Jacobians
     */
    def public init_jacobians_file(Robot robot, List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        function «jacobiansStructName» = «initFunctionName»()

        «initFunctionBody(robot, jacs, transformsModel)»
    '''

    /**
     * The complete content of the file with Matlab function to update the Jacobians
     */
    def public update_jacobians_file(
        Robot robot, List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transformsModel )
    '''
        function out = «updateFunctionName»(«updateFunctionArgsList()»)

        «updateFunctionBody(robot, jacs, transformsModel)»

        out = «jacobiansStructName»;
    '''

    /**
     * The body of the Matlab function to initialize the Jacobians
     */
    def public initFunctionBody(
        Robot robot, List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transformsModel )
    '''
        «FOR Jacobian j : jacs»
            «val jText = maximaConverter.getJacobianText(j, transformsModel)»
            «val name = jacobiansStructName + "." + j.name»
            «name» = zeros(«j.rows»,«j.cols»);
            «maxdslUtils.staticInitAssignements(jText, name)»

        «ENDFOR»
    '''

    /**
     * The body of the Matlab function to update the Jacobians
     */
    def public updateFunctionBody(Robot robot, List<Jacobian> jacs, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val StringConcatenation strBuff = new StringConcatenation()
        val maxdslCommons = new iit.dsl.maxdsl.generator.Common()
        val sines      = new ArrayList<iit.dsl.maxdsl.maximaDsl.Sine>()
        val cosines    = new ArrayList<iit.dsl.maxdsl.maximaDsl.Cosine>()
        val jacsText   = new ArrayList<String[][]>()
        val exprModels = new ArrayList<iit.dsl.maxdsl.maximaDsl.Model>()

        var iit.dsl.maxdsl.maximaDsl.Model expressionsModel = null;

        // First pass: identify the set of unique sines and cosines functions
        //   required for all the given Jacobians. Also, get the textual representation
        //   of each matrix
        for (J : jacs) {
            val jText = maximaConverter.getJacobianText(J, transformsModel)
            jacsText.add( jText )

            // Check if the current Jacobian is actually a function of something
            val maxdslDoc = iit::dsl::generator::common::Jacobians::expressionsAsMaximaDSLDocument(J, jText, transformsModel)
            if(maxdslDoc.length > 0) {
                // The Maxima-DSL model with the trigonometric expressions
                expressionsModel = maxdslAccess.getParsedTextModel(maxdslDoc.toString())
                // Add to the lists the new sines and cosines
                maxdslCommons.getTrigonometricExpressions(expressionsModel, sines, cosines)
            } else {
                expressionsModel = null
            }
            exprModels.add( expressionsModel )
        }

        // Append the code with the initialization of the variables for the
        // trigonometric functions:
        strBuff.append(maxdslUtils.trigFunctionsCode(sines, cosines, replaceSpecs))
        strBuff.append("\n");

        // Second pass: append the code that updates the Jacobian matrix
        var j = 0
        for (J : jacs) {
            val varName = jacobiansStructName + "." + J.name
            if( exprModels.get(j) != null ) {
                strBuff.append(maxdslUtils.updateAssignements(
                        exprModels.get(j), jacsText.get(j), varName, replaceSpecs))
                strBuff.append("\n")
            }
            strBuff.append("\n\n");
            j = j+1
        }
        return strBuff
    }


    def private updateFunctionArgsList()
    {
        var StringConcatenation ret = new StringConcatenation()
        ret.append(jacobiansStructName)
        for(arg : configurator.getUpdateFunctionArguments() ) {
            ret.append(", " + arg)
        }
        return ret
    }

    private CharSequence jacobiansStructName = '''jacs'''

    private IConfigurator configurator = null
    private iit.dsl.maxdsl.generator.IIdentifiersReplacement replaceSpecs = null
    private iit.dsl.generator.maxima.Converter  maximaConverter = null
    private iit.dsl.maxdsl.utils.DSLAccessor      maxdslAccess  = null
    private iit.dsl.maxdsl.generator.matlab.Utils maxdslUtils   = null
}