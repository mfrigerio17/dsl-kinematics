package iit.dsl.generator;

import org.eclipse.emf.common.util.URI;
import org.eclipse.xtext.mwe.UriFilter;

public class MyUriFilter implements UriFilter {

    private final String match1 = "/home/phd/work/eclipse_workspace/kindsl/models/hyq_fixed.kindsl";
    private final String match2 = "/home/phd/work/eclipse_workspace/kindsl/models/hyq.kindsl";
    @Override
    public boolean matches(URI uri) {
        String file = uri.toFileString();
        if(match1.equals(file) || match2.equals(file)) {
            return true;
        }
        return false;
    }

}
