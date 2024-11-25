#ifndef __LIO_SAM_SEMANTIC_OBJECT_H__
#define __LIO_SAM_SEMANTIC_OBJECT_H__
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/DualConic.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
using namespace std;

namespace LIO_SAM_SEMANTIC{
    class Object{
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            size_t id_;
            string name_;
            gtsam_quadrics::ConstrainedDualQuadric Q_;
        public:
            Object();

            Object(const Object& obj);
            
            Object(const string& name, size_t id, const gtsam_quadrics::ConstrainedDualQuadric& Q);

            string getClassName() const;

            size_t id() const;

            gtsam_quadrics::ConstrainedDualQuadric Q() const;

            void setQ(const gtsam_quadrics::ConstrainedDualQuadric& Q);
    };
}

#endif