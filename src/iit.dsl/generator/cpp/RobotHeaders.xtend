package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.cpp.dynamics.InverseDynamics
import iit.dsl.generator.cpp.dynamics.ForwardDynamics


class RobotHeaders {
    private iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def main(Robot robot)'''
        #ifndef IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files::mainHeader(robot).toUpperCase()»_H_
        #define IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files::mainHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>

        «Common::enclosingNamespacesOpen(robot)»

        static const int JointSpaceDimension = «robot.joints.size»;
        static const int jointsCount = «robot.joints.size»;
        /** The total number of rigid bodies of this robot, including the base */
        static const int linksCount  = «robot.links.size + 1»;

        typedef Eigen::Matrix<double, «robot.joints.size», 1> Column«robot.joints.size»d;
        typedef Column«robot.joints.size»d JointState;

        enum JointIdentifiers {
            «Common::jointIdentifier(robot.joints.get(0))» = 0
            «FOR Joint j : robot.joints.drop(1)»
            , «Common::jointIdentifier(j)»
            «ENDFOR»
        };

        enum LinkIdentifiers {
            «Common::linkIdentifier(robot.base)» = 0
            «FOR l : robot.links»
            , «Common::linkIdentifier(l)»
            «ENDFOR»
        };

        static const JointIdentifiers orderedJointIDs[jointsCount] =
            {«FOR j : robot.joints SEPARATOR ','»«Common::jointIdentifier(j)»«ENDFOR»};

        static const LinkIdentifiers orderedLinkIDs[linksCount] =
            {«Common::linkIdentifier(robot.base)»,«FOR l : robot.links SEPARATOR ','»«Common::linkIdentifier(l)»«ENDFOR»};

        «Common::enclosingNamespacesClose(robot)»
        #endif
        '''

    def static linkDataMap_type() '''LinkDataMap'''
    def linkDataMap(Robot robot) '''
        «val className = linkDataMap_type()»
        #ifndef IIT_«robot.name.toUpperCase()»_«Names$Files::linkDataMapHeader(robot).toUpperCase()»_H_
        #define IIT_«robot.name.toUpperCase()»_«Names$Files::linkDataMapHeader(robot).toUpperCase()»_H_

        #include "«Names$Files::mainHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»

        /**
         * A very simple container to associate a generic data item to each link
         */
        template<typename T> class «className» {
        private:
            T data[linksCount];
        public:
            «className»() {};
            «className»(const T& defaultValue);
            «className»(const «className»& rhs);
            «className»& operator=(const «className»& rhs);
            «className»& operator=(const T& rhs);
                  T& operator[](LinkIdentifiers which);
            const T& operator[](LinkIdentifiers which) const;
        private:
            void copydata(const «className»& rhs);
            void assigndata(const T& commonValue);
        };

        template<typename T> inline
        «className»<T>::«className»(const T& value) {
            assigndata(value);
        }

        template<typename T> inline
        «className»<T>::«className»(const «className»& rhs)
        {
            copydata(rhs);
        }

        template<typename T> inline
        «className»<T>& «className»<T>::operator=(const «className»& rhs)
        {
            if(&rhs != this) {
                copydata(rhs);
            }
            return *this;
        }

        template<typename T> inline
        «className»<T>& «className»<T>::operator=(const T& value)
        {
            assigndata(value);
            return *this;
        }

        template<typename T> inline
        T& «className»<T>::operator[](LinkIdentifiers l) {
            return data[l];
        }

        template<typename T> inline
        const T& «className»<T>::operator[](LinkIdentifiers l) const {
            return data[l];
        }

        template<typename T> inline
        void «className»<T>::copydata(const «className»& rhs) {
            data[«Common::linkIdentifier(robot.base)»] = rhs[«Common::linkIdentifier(robot.base)»];
            «FOR l : robot.links»
                data[«Common::linkIdentifier(l)»] = rhs[«Common::linkIdentifier(l)»];
            «ENDFOR»
        }

        template<typename T> inline
        void «className»<T>::assigndata(const T& value) {
            data[«Common::linkIdentifier(robot.base)»] = value;
            «FOR l : robot.links»
                data[«Common::linkIdentifier(l)»] = value;
            «ENDFOR»
        }

        template<typename T> inline
        std::ostream& operator<<(std::ostream& out, const «className»<T>& map) {
            out
            «FOR l : common.abstractLinks(robot)»
                << "   «l.name» = "
                << map[«Common::linkIdentifier(l)»]
            «ENDFOR»
            ;
            return out;
        }

        «Common::enclosingNamespacesClose(robot)»
        #endif
    '''

    def static jointDataMap_type() '''JointDataMap'''
    def jointDataMap(Robot robot) '''
        «val className = jointDataMap_type()»
        #ifndef IIT_«robot.name.toUpperCase()»_«Names$Files::jointDataMapHeader(robot).toUpperCase()»_H_
        #define IIT_«robot.name.toUpperCase()»_«Names$Files::jointDataMapHeader(robot).toUpperCase()»_H_

        #include "«Names$Files::mainHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»

        /**
         * A very simple container to associate a generic data item to each joint
         */
        template<typename T> class «className» {
        private:
            T data[jointsCount];
        public:
            «className»() {};
            «className»(const T& defaultValue);
            «className»(const «className»& rhs);
            «className»& operator=(const «className»& rhs);
            «className»& operator=(const T& rhs);
                  T& operator[](JointIdentifiers which);
            const T& operator[](JointIdentifiers which) const;
        private:
            void copydata(const «className»& rhs);
            void assigndata(const T& rhs);
        };

        template<typename T> inline
        «className»<T>::«className»(const T& value) {
            assigndata(value);
        }

        template<typename T> inline
        «className»<T>::«className»(const «className»& rhs)
        {
            copydata(rhs);
        }

        template<typename T> inline
        «className»<T>& «className»<T>::operator=(const «className»& rhs)
        {
            if(&rhs != this) {
                copydata(rhs);
            }
            return *this;
        }

        template<typename T> inline
        «className»<T>& «className»<T>::operator=(const T& value)
        {
            assigndata(value);
            return *this;
        }

        template<typename T> inline
        T& «className»<T>::operator[](JointIdentifiers j) {
            return data[j];
        }

        template<typename T> inline
        const T& «className»<T>::operator[](JointIdentifiers j) const {
            return data[j];
        }

        template<typename T> inline
        void «className»<T>::copydata(const «className»& rhs) {
            «FOR j : robot.joints»
                data[«Common::jointIdentifier(j)»] = rhs[«Common::jointIdentifier(j)»];
            «ENDFOR»
        }

        template<typename T> inline
        void «className»<T>::assigndata(const T& value) {
            «FOR j : robot.joints»
                data[«Common::jointIdentifier(j)»] = value;
            «ENDFOR»
        }

        template<typename T> inline
        std::ostream& operator<<(std::ostream& out, const «className»<T>& map) {
            out
            «FOR j : robot.joints»
                << "   «j.name» = "
                << map[«Common::jointIdentifier(j)»]
            «ENDFOR»
            ;
            return out;
        }

        «Common::enclosingNamespacesClose(robot)»
        #endif
    '''

    def public traits(Robot robot) '''
        #ifndef IIT_ROBOGEN__«robot.name.toUpperCase()»_TRAITS_H_
        #define IIT_ROBOGEN__«robot.name.toUpperCase()»_TRAITS_H_

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::invDynHeader(robot)».h"
        #include "«Names$Files$RBD::fwdDynHeader(robot)».h"
        #include "«Names$Files$RBD::jsimHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»

        «val ns  = Names$Namespaces::rob(robot)»
        «val dyn = Names$Namespaces::dynamics»
        struct Traits {
            typedef typename «ns»::«Names$Types::jointState» «Names$Types::jointState»;

            typedef typename «ns»::JointIdentifiers JointID;
            typedef typename «ns»::LinkIdentifiers  LinkID;

            typedef typename «ns»::«Names$Types$Transforms::homogeneous» «Names$Types$Transforms::homogeneous»;
            typedef typename «ns»::«Names$Types$Transforms::spatial_motion» «Names$Types$Transforms::spatial_motion»;
            typedef typename «ns»::«Names$Types$Transforms::spatial_force» «Names$Types$Transforms::spatial_force»;

            typedef typename «ns»::«dyn»::«ForwardDynamics::className(robot)» FwdDynEngine;
            typedef typename «ns»::«dyn»::«InverseDynamics::className(robot)» InvDynEngine;
            typedef typename «ns»::«dyn»::«Names$Types::jspaceMLocal» JSIM;

            static const int joints_count = «ns»::jointsCount;
            static const int links_count  = «ns»::linksCount;
            static const bool floating_base = «IF common.isFloating(robot.base)»true«ELSE»false«ENDIF»;

            static inline const JointID* orderedJointIDs();
            static inline const LinkID*  orderedLinkIDs();
        };


        inline const Traits::JointID*  Traits::orderedJointIDs() {
            return «ns»::orderedJointIDs;
        }
        inline const Traits::LinkID*  Traits::orderedLinkIDs() {
            return «ns»::orderedLinkIDs;
        }

        «Common::enclosingNamespacesClose(robot)»

        #endif
    '''
}