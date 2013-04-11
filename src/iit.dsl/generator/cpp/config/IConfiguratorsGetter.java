package iit.dsl.generator.cpp.config;

import iit.dsl.kinDsl.Robot;

/**
 * Interface for a component that returns the various configurators required by
 * classes of the C++ code generator.
 *
 * See the documentation of the various interfaces referenced by this one, to
 * have more information.
 * @author Marco Frigerio
 *
 */
public interface IConfiguratorsGetter {
    /**
     * The configurator for paths
     */
    public IConfigurator.Paths getPathsConfigurator();
    /**
     * The configurator for file names
     */
    public IConfigurator.Names.Files getFileNamesConfigurator();
    /**
     * The configurator for namespace names
     */
    public IConfigurator.Names.Namespaces getNamespacesConfigurator();
    /**
     * The configurator for the Maxima conversion, required by generators of
     * this package.
     */
    public iit.dsl.generator.maxima.IConverterConfigurator
        getMaximaConverterConfigurator();

    /**
     * The configurator for the C++ code generation happening in the Transforms
     * DSL infrastructure.
     * @param robot the robot that the configurator has to be tailored for
     */
    public iit.dsl.coord.generator.cpp.IConfigurator
        getTransformsDSLGeneratorConfigurator(Robot robot);
    /**
     * The configurator for hte Maxima conversion, required by generators of
     * the Transforms DSL infrastructure.
     */
    public iit.dsl.coord.generator.MaximaConverter.IConfigurator
        getTransformsDSLMaximaConfigurator();
}
