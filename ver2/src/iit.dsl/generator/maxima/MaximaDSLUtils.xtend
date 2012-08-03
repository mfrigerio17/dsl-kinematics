package iit.dsl.generator.maxima

import org.eclipse.xtend2.lib.StringConcatenation
import iit.dsl.generator.Jacobian

class MaximaDSLUtils {
	/**
      * Creates a document compliant with the MaximaDSL, containing all the non
      * constant elements of the given Jacobian, as a list of expressions.
      *
      */
     def static MaximaDSLDocumentText(Jacobian J, String[][] jText) {
         val strBuff = new StringConcatenation();
         strBuff.append('''
            Variables {
                «J.getArgsList()»
            }
            ''')

         for(row : jText) {
            for(el : row) {
                if( !iit::dsl::maxdsl::utils::MaximaConversionUtils::isConstant(el)) {
                    strBuff.append('''«el»;''')
                    strBuff.append("\n");
                }
            }
        }
        return strBuff;
     }
}