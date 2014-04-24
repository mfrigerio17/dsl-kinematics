package iit.dsl.generator.cpp.config


/**
 * A configurator for the conversion of Maxima code, as required by the
 * Transforms DSL package.
 *
 * This implementation basically relies on the default configurator
 * iit.dsl.coord.generator.maxima.DefaultConfigurator
 * and only overrides the functions returning paths.
 */
class MaximaConverterConfigurator extends iit.dsl.coord.generator.maxima.DefaultConfigurator
{
    new(IConfigurator$Paths pathsConfig) {
        if(pathsConfig == null) {
            throw new RuntimeException("Null argument (IConfigurator$Paths) to constructor")
        }
        pathsConfigurator = pathsConfig
    }

    override getGeneratedCodeLocation() {
        pathsConfigurator.maximaCodeTransforms
    }

    override getLibsPath() {
        pathsConfigurator.maximaLibs
    }

    private IConfigurator$Paths pathsConfigurator;
}