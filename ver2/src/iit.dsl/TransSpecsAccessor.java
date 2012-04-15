package iit.dsl;

import java.io.File;
import java.util.List;

import iit.dsl.kinDsl.Robot;

import com.google.inject.Injector;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;


/**
 * Class that lets access the models created by the TransSpecs DSL.
 * Documents of such DSL contain the list of the desired transformations
 * and Jacobians for a given Robot.
 * @author phd
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
        if(new java.io.File(modelFilePath).isFile()) {
            return getModel(URI.createURI(modelFilePath));
        } else {
            System.err.println("Could not find file " + modelFilePath + ", skipping...");
        }
        return null;
    }

    public iit.dsl.transspecs.transSpecs.DesiredTransforms getDesiredTransforms(File modelFile) {
        if(modelFile.isFile()) {
            return getModel(URI.createURI(modelFile.getAbsolutePath()));
        } else {
            System.err.println("Could not find file " + modelFile.getPath() + ", skipping...");
        }
        return null;
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
