package iit.dsl.generator;

import org.eclipse.emf.common.util.URI;
import org.eclipse.xtext.mwe.UriFilter;

public class MyUriFilter implements UriFilter {

    private final String match = "/home/phd/work/eclipse_workspace/kindsl/models/hyq_fixed.kindsl";
    @Override
    public boolean matches(URI uri) {
        if(match.equals(uri.toFileString())) {
            return true;
        }
        return false;
    }

}
