package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint
import com.google.inject.Inject

class Common {
    @Inject static iit.dsl.generator.Common common

    def public static jointIdentifier(Joint j) '''«j.name.toUpperCase()»'''
    def public static variableForCosineOf(Joint j) '''cos_«common.getVariableName(j)»'''
    def public static variableForSineOf(Joint j)   '''sin_«common.getVariableName(j)»'''
    def public static valueAccessorOf(Joint j) '''«jointsStateVarName()»(«jointIdentifier(j)»)'''
    def public static jointsStateVarName() '''jState'''
}