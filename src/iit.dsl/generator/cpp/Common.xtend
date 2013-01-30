package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink

class Common {
    static iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def public static linkIdentifier(AbstractLink l) '''«l.name.toUpperCase()»'''
    def public static jointIdentifier(Joint j) '''«j.name.toUpperCase()»'''
    def public static variableForCosineOf(Joint j) '''cos_«common.getVariableName(j)»'''
    def public static variableForSineOf(Joint j)   '''sin_«common.getVariableName(j)»'''
    def public static valueAccessorOf(Joint j) '''«jointsStateVarName()»(«jointIdentifier(j)»)'''
    def public static jointsStateVarName() '''jState'''
}