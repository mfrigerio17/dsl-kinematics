package iit.dsl;

import java.io.File;

import iit.dsl.coord.coordTransDsl.Model;
import iit.dsl.generator.FramesTransforms;
import iit.dsl.kinDsl.Robot;

import com.google.inject.Injector;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;

public class TransformsAccessor {
    private static Injector injector =
            new iit.dsl.coord.CoordTransDslStandaloneSetupGenerated()
                .createInjectorAndDoEMFRegistration();
    private Resource resource = null;
    private XtextResourceSet set = null;

    public TransformsAccessor() {
        set = injector.getInstance(XtextResourceSet.class);
        set.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    }

    public iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot) {
        resource = set.getResource(
                URI.createURI(
                        "generated_code/misc/"+ FramesTransforms.fileName(robot)), true);
        return (Model)resource.getContents().get(0);
    }

    public iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot, File modelFile) {
        if(modelFile.isFile()) {
            resource = set.getResource(URI.createURI(modelFile.getAbsolutePath()), true);
            return (Model)resource.getContents().get(0);
        } else {
            System.err.println("Could not find file " + modelFile.getPath() + ", skipping...");
            return null;
        }
    }

}
