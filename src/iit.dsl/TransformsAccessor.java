package iit.dsl;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import iit.dsl.coord.coordTransDsl.Model;
import iit.dsl.generator.FramesTransforms;
import iit.dsl.kinDsl.Robot;

import com.google.inject.Injector;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.util.StringInputStream;

/**
 * Class that lets access the models of the "Coordinate Transforms-DSL".
 * Documents of such DSL contain a specification of abstract transformation
 * matrices.
 * @author Marco Frigerio
 *
 */
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

    public iit.dsl.coord.coordTransDsl.Model getModel(String model) throws IOException
    {
        URI uri = URI.createURI("dummy:/"+Long.toString(System.nanoTime())+".ctdsl");
        resource = set.createResource(uri);
        InputStream in = new StringInputStream(model);
        resource.load(in, set.getLoadOptions());
        return getModel(uri);
    }

    private iit.dsl.coord.coordTransDsl.Model getModel(final URI uri)
    {
        resource = set.getResource(uri, true);
        List<Resource.Diagnostic> errors = resource.getErrors();
        if(errors.size() > 0) {
            StringBuffer msg = new StringBuffer();
            msg.append("Errors while loading a document of the Transforms-DSL ("
                        + uri.toString() + "):\n");
            for(Resource.Diagnostic err : errors) {
                msg.append("\n\t " + err.getMessage() + "\n");
            }
            throw new RuntimeException(msg.toString());
        }
        return (iit.dsl.coord.coordTransDsl.Model)resource.getContents().get(0);
    }

}
