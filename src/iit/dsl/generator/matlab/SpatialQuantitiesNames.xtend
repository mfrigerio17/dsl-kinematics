package iit.dsl.generator.matlab

import iit.dsl.kinDsl.AbstractLink

class SpatialQuantitiesNames
{
    def public static getInstance() {
        return instance
    }

    def velocity(AbstractLink l)     '''«l.name»_v'''
    def acceleration(AbstractLink l) '''«l.name»_a'''
    def force(AbstractLink l)        '''«l.name»_f'''
    def inertia(AbstractLink l)      '''«l.name»_I'''
    def inertiaC(AbstractLink l)     '''«l.name»_Ic'''


    private new() {}

    private static SpatialQuantitiesNames instance = new SpatialQuantitiesNames()
}