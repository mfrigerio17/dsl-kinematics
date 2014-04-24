package iit.dsl;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import iit.dsl.kinDsl.Robot;

import com.google.inject.Injector;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.util.StringInputStream;


/**
 * Class that lets access the models created by the TransSpecs-DSL.
 * Documents of such DSL contain the list of the desired transformations
 * and Jacobians for a given Robot.
 * @author Marco Frigerio
 *
 */
public class TransSpecsAccessor {
    private Resource resource;
    private final XtextResourceSet set;

    public TransSpecsAccessor() {
        Injector injector = new iit.dsl.transspecs.TransSpecsStandaloneSetup().
                createInjectorAndDoEMFRegistration();

        set = injector.getInstance(XtextResourceSet.class);
        set.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    }

    public iit.dsl.transspecs.transSpecs.DesiredTransforms getDesiredTransforms(Robot robot) {
        String modelFilePath = "models/"+robot.getName() + ".dtdsl";
        return getDesiredTransforms(new java.io.File(modelFilePath));
    }

    public iit.dsl.transspecs.transSpecs.DesiredTransforms getDesiredTransforms(File modelFile) {
        if(modelFile == null) return null;
        if(modelFile.isFile()) {
            return getModel(URI.createURI(modelFile.getAbsolutePath()));
        } else {
            System.out.println("Did not find the file " + modelFile.getPath());
        }
        return null;
    }

    public iit.dsl.transspecs.transSpecs.DesiredTransforms getModel(String model) throws IOException
    {
        URI uri = URI.createURI("dummy:/"+Long.toString(System.nanoTime())+".dtdsl");
        resource = set.createResource(uri);
        InputStream in = new StringInputStream(model);
        resource.load(in, set.getLoadOptions());
        return getModel(uri);
    }

    private iit.dsl.transspecs.transSpecs.DesiredTransforms getModel(final URI uri) {
        resource = set.getResource(uri, true);
        List<Resource.Diagnostic> errors = resource.getErrors();
        if(errors.size() > 0) {
            StringBuffer msg = new StringBuffer();
            msg.append("Errors while loading a document of the Transforms-specs-DSL ("
                        + uri.toString() + "):\n");
            for(Resource.Diagnostic err : errors) {
                msg.append("\n\t " + err.getMessage() + "\n");
            }
            throw new RuntimeException(msg.toString());
        }
        return (iit.dsl.transspecs.transSpecs.DesiredTransforms)resource.getContents().get(0);
    }

}
