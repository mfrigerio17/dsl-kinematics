package iit.dsl.generator;

import org.eclipse.emf.common.util.URI;
import org.eclipse.xtext.mwe.UriFilter;

public class MyUriFilter implements UriFilter {

    private static final String prefix = "/home/phd/work/eclipse_workspace/kindsl/models/";
    private final String match1 = prefix + "hyq/hyq.kindsl";
    private final String hyl2   = prefix + "hyl/hyl2.kindsl";
    private final String fancy  = prefix + "fancy.kindsl";
    private final String match4 = prefix + "fancybranch.kindsl";
    @Override
    public boolean matches(URI uri) {
        String file = uri.toFileString();
        if(fancy.equals(file) /*|| match2.equals(file)  ||
                match3.equals(file) || match4.equals(file)*/)
        {
            return true;
        }
        return false;
    }

}
