package iit.dsl;

import iit.dsl.kinDsl.Robot;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;


/**
 * Class that lets access the models created by the TransSpecs DSL.
 * Documents of such DSL contain the list of the desired transformations
 * and Jacobians for a given Robot.
 * @author phd
 *
 */
public class TransSpecsAccessor {
    @Inject
    Provider<ResourceSet> resourceSetProvider;

    private Resource resource;
    private ResourceSet set;

    private static Injector injector = new iit.dsl.transspecs.TransSpecsStandaloneSetup().createInjectorAndDoEMFRegistration();

    public iit.dsl.transspecs.transSpecs.DesiredTransforms getDesiredTransforms(Robot robot) {

        set = resourceSetProvider.get();
        String modelFilePath = "models/"+robot.getName() + ".dtdsl";
        if(new java.io.File(modelFilePath).isFile()) {
            resource = set.getResource(URI.createURI(modelFilePath), true);
            return (iit.dsl.transspecs.transSpecs.DesiredTransforms)resource.getContents().get(0);
        }
        return null;
    }
}
