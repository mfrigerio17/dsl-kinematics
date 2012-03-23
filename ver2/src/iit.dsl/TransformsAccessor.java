package iit.dsl;

import java.io.File;

import iit.dsl.coord.coordTransDsl.Model;
import iit.dsl.kinDsl.Robot;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;

public class TransformsAccessor {
    @Inject
    Provider<ResourceSet> resourceSetProvider;

    private Resource resource;
    private ResourceSet set;

    private static Injector injector = new iit.dsl.coord.CoordTransDslStandaloneSetupGenerated().createInjectorAndDoEMFRegistration();

    public iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot) {
        set = resourceSetProvider.get();
        resource = set.getResource(URI.createURI("generated_code/misc/"+robot.getName() + ".ctdsl"), true);
        return (Model)resource.getContents().get(0);
    }

    public iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot, File modelFile) {
        set = resourceSetProvider.get();
        if(modelFile.isFile()) {
            resource = set.getResource(URI.createURI(modelFile.getAbsolutePath()), true);
            return (Model)resource.getContents().get(0);
        } else {
            System.err.println("Could not find file " + modelFile.getPath() + ", skipping...");
            return null;
        }
    }

}
