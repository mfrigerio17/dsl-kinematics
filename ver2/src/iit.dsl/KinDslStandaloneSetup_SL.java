package iit.dsl;

import com.google.inject.Guice;
import com.google.inject.Injector;

public class KinDslStandaloneSetup_SL extends KinDslStandaloneSetupGenerated{

	public static void doSetup() {
		new KinDslStandaloneSetup_SL().createInjectorAndDoEMFRegistration();
	}

	@Override
	public Injector createInjector() {
		return Guice.createInjector(
				new iit.dsl.KinDslRuntimeModule() { // local class that inherits from iit.dsl.KinDslRuntimeModule
					@Override
					public Class<? extends org.eclipse.xtext.generator.IGenerator> bindIGenerator() {
						return iit.dsl.generator.sl.Generator.class;
					}
				});
	}
}
