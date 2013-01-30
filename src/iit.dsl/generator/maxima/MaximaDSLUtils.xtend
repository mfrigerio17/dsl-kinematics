package iit.dsl.generator.maxima

import org.eclipse.xtend2.lib.StringConcatenation
import iit.dsl.generator.Jacobian

class MaximaDSLUtils {
	/**
     * Creates a document compliant with the MaximaDSL, containing all the non
     * constant elements of the given Jacobian, as a list of expressions.
     *
     * Returns an empty text if the given Jacobian is not a function of any
     * joint (therefore it does not contain any expression which is a function
     * of a joint variable) or if does not contain any expression (which can
     * happen even if it formally depends on some joint)
     *
     */
     def static MaximaDSLDocumentText(Jacobian J, String[][] jText) {
        val strBuff = new StringConcatenation();
        if(J.getJointsChain().size() == 0) {
            //The jacobian does not depend on any joint
            return strBuff
        }

        strBuff.append('''
           Variables {
               «J.getArgsList()»
           }
           ''')

        var boolean empty = true // true if there is no expression in this jacobian,
        for(row : jText) {
            for(el : row) {
                if( !iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el)) {
                    strBuff.append('''«el»;''')
                    strBuff.append("\n");
                    empty = false
                }
            }
        }
        if(!empty) {
            return strBuff;
        } else {
            return new StringConcatenation() // empty text
        }
     }
}