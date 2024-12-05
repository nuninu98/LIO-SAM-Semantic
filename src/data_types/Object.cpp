#include "Object.h"

namespace LIO_SAM_SEMANTIC{
    Object::Object(){

    }

    Object::Object(const Object& obj): Q_(obj.Q_), id_(obj.id_), name_(obj.name_){

    }
    
    Object::~Object(){
        
    }

    Object::Object(const string& name, size_t id, const gtsam_quadrics::ConstrainedDualQuadric& Q): Q_(Q), id_(id), name_(name){

    }

    string Object::getClassName() const{
        return name_;
    }

    size_t Object::id() const{
        return id_;
    }

    gtsam_quadrics::ConstrainedDualQuadric Object::Q() const{
        return Q_;
    }

    void Object::setQ(const gtsam_quadrics::ConstrainedDualQuadric& Q){
        Q_ = Q;
    }

}