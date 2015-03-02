package iit.dsl.generator.matlab.config

import iit.dsl.kinDsl.Robot


class DefaultConfigurator implements IConfigurator
{
    new(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        iit.dsl.generator.maxima.IConverterConfigurator maximaConfig )
    {
        maximaReplacementStrategy = new MaximaReplacementStrategy(
            robot, transformsModel, jointStatusName, paramsStructName  )
        maximaConverterConfig = maximaConfig
    }

    override getKindslMaximaConverterConfigurator() {
        return this.maximaConverterConfig
    }

    override getMaximaConverterConfigurator() {
        return this.maximaConverterConfig
    }

    override getMaximaReplacementStrategy() {
        return this.maximaReplacementStrategy
    }

    override getUpdateFunctionArguments() {
        return updateFArgs
    }


    override getJointStatusVectorName() {
        return jointStatusName
    }

    override getParamsStructName() {
        return paramsStructName
    }

    private MaximaReplacementStrategy maximaReplacementStrategy
    private iit.dsl.generator.maxima.IConverterConfigurator maximaConverterConfig

    private String[] updateFArgs = #[jointStatusName, paramsStructName]

    public static String paramsStructName = "params"
    public static String jointStatusName = "q";

}

