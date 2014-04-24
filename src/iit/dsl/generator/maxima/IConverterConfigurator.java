package iit.dsl.generator.maxima;

/**
 * The configurator interface for the Maxima Converter of this package.
 * It extends the interface provided by iit.dsl.coord.generator.MaximaConverter.IConfigurator,
 * by adding some methods required specifically by the Converter of this
 * package.
 * @author Marco Frigerio
 *
 */
public interface IConverterConfigurator extends
    iit.dsl.coord.generator.MaximaConverter.IConfigurator
{
    /**
     * The full path of the Maxima library with utility functions
     */
    public String getUtilsLibName();
    /**
     * The full path of the Maxima library with functions for geometric Jacobians
     */
    public String getJacobiansLibName();

}