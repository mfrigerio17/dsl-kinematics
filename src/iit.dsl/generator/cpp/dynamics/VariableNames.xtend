package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.AbstractLink

class VariableNames {
    def velocity(AbstractLink l)     '''«l.name»_v'''
    def acceleration(AbstractLink l) '''«l.name»_a'''
    def force(AbstractLink l)        '''«l.name»_f'''
    def inertia(AbstractLink l)      '''«l.name»_I'''
    def inertiaC(AbstractLink l)     '''«l.name»_Ic'''
}