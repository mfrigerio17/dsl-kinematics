package iit.dsl;

import java.io.IOException;
import java.io.InputStream;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.util.StringInputStream;

import com.google.inject.Injector;

/**
 * This class loads the machinery of the Maxima DSL (package iit.dsl.maxdsl...)
 * and allows to retrieve the corresponding models, given a string with the
 * text of a compliant document (ie the text model)
 * @author Marco Frigerio
 */
public class MaximaDSLAccessor {

    private XtextResourceSet resourceSet = null;
    private Resource resource = null;

    public MaximaDSLAccessor() {
        new org.eclipse.emf.mwe.utils.StandaloneSetup().setPlatformUri("../");
        Injector injector = new iit.dsl.maxdsl.MaximaDslStandaloneSetup().createInjectorAndDoEMFRegistration();
        resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);

        resource = resourceSet.createResource(URI.createURI("dummy:/example.maxdsl"));
    }

    public iit.dsl.maxdsl.maximaDsl.Model getParsedTextModel(String model) throws IOException {
        InputStream in = new StringInputStream(model);
        resource.load(in, resourceSet.getLoadOptions());
        return (iit.dsl.maxdsl.maximaDsl.Model) resource.getContents().get(0);
    }

}