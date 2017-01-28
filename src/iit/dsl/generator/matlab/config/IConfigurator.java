package iit.dsl.generator.matlab.config;


/**
 * This interface specifies the configuration requirements of the Matlab code
 * generator of the Kinematics DSL.
 *
 * An implementation of this interface would be needed to construct the Matlab
 * code generator.
 *
 * This interfaces extends the corresponding one of the Transforms-DSL.
 *
 * @author Marco Frigerio
 *
 */
public interface IConfigurator extends iit.dsl.coord.generator.matlab.IConfigurator
{
    /**
     * @return the maxima-conversion configurator, that is, an object
     *         implementing the interface iit.dsl.generator.maxima.IConverterConfigurator
     */
    public iit.dsl.generator.maxima.IConverterConfigurator
      getKindslMaximaConverterConfigurator();

}
