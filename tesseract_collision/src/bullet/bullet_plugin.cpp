#include <class_loader/class_loader.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_managers.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_managers.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_managers.h>
#include <tesseract_collision/bullet/bullet_cast_simple_managers.h>

CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletDiscreteSimpleManager,
                            tesseract::DiscreteContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletDiscreteBVHManager,
                            tesseract::DiscreteContactManagerBase)

CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletCastSimpleManager,
                            tesseract::ContinuousContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletCastBVHManager, tesseract::ContinuousContactManagerBase)
