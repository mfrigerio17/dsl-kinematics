package iit.dsl.generator.cpp.config

import iit.dsl.generator.cpp.config.IConfiguratorsGetter
import iit.dsl.kinDsl.Robot

/**
 * A default implementation of the IConfiguratorsGetter interface.
 *
 * The various getter methods return in turn default implementation of the
 * various configuration interfaces.
 */
class DefaultConfiguratorsGetter implements IConfiguratorsGetter {

    new() {
        namesConfigurator = new DefaultConfigurator()
    }


    override getFileNamesConfigurator() {
        return namesConfigurator
    }
    override getNamespacesConfigurator() {
        return namesConfigurator
    }
    override getPathsConfigurator() {
        return namesConfigurator
    }


    override getMaximaConverterConfigurator() {
        new iit.dsl.generator.maxima.DefaultConfigurator
    }



    override getTransformsDSLGeneratorConfigurator(Robot robot) {
        new TransformsGeneratorConfigurator(robot, namesConfigurator)
    }

    override getTransformsDSLMaximaConfigurator() {
        new MaximaConverterConfigurator(namesConfigurator)
    }


    private DefaultConfigurator namesConfigurator = null


}