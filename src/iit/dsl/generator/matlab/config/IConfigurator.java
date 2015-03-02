package iit.dsl.generator.matlab.config;


/**
 * This interface specifies the configuration requirements of the Matlab code
 * generator.
 * 
 * An implementation of this interface would be needed to configure the Matlab
 * code generator.
 * 
 * @author Marco Frigerio
 *
 */
public interface IConfigurator extends iit.dsl.coord.generator.matlab.IConfigurator
{
    @Override
    public iit.dsl.coord.generator.MaximaConverter.IConfigurator
      getMaximaConverterConfigurator();

    /**
     * @return the maxima-conversion configurator, that is, an object
     *         implementing the interface iit.dsl.generator.maxima.IConverterConfigurator
     */
    public iit.dsl.generator.maxima.IConverterConfigurator
      getKindslMaximaConverterConfigurator();

    /**
     * @return the name of the vector representing the joint status, which
     *         will appear in the generated Matlab code
     */
    public String getJointStatusVectorName();
    /**
     * @return the name of the variable (of type struct) with the values of the
     *         parameters, which will appear in the generated Matlab code
     */
    public String getParamsStructName();
}
