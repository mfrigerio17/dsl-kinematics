package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.AbstractLink

class Common {
    def child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«child.name»_X_fr_«parent.name»'''
    def parent_X_child__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«parent.name»_X_fr_«child.name»'''
}