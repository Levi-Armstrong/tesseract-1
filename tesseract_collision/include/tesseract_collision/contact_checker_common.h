#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H

#include <tesseract_core/basic_types.h>
#include <ros/console.h>

namespace tesseract
{

typedef std::pair<std::string, std::string> ObjectPairKey;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
static ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool isContactAllowed(const std::string& name1, const std::string& name2, const IsContactAllowedFn acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm != nullptr && acm(name1, name2))
  {
    if (verbose)
    {
      ROS_DEBUG("Collision between '%s' and '%s' is allowed. No contacts are computed.",
                name1.c_str(),
                name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    ROS_DEBUG("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

/// Destance query results information
struct ContactDistanceData
{
  ContactDistanceData(const ContactRequest* req, ContactResultMap* res) : req(req), res(res), done(false) {}
  /// Distance query request information
  const ContactRequest* req;

  /// Destance query results information
  ContactResultMap* res;

  /// Indicate if search is finished
  bool done;
};

inline ContactResult* processResult(ContactDistanceData& cdata,
                                    ContactResult& contact,
                                    const std::pair<std::string, std::string>& key,
                                    bool found)
{
  if (!found)
  {
    ContactResultVector data;
    if (cdata.req->type == ContactRequestType::FIRST)
    {
      data.emplace_back(contact);
      cdata.done = true;
    }
    else
    {
      data.reserve(100);  // TODO: Need better way to initialize this
      data.emplace_back(contact);
    }

    return &(cdata.res->insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    assert(cdata.req->type != ContactRequestType::FIRST);
    ContactResultVector& dr = cdata.res->at(key);
    if (cdata.req->type == ContactRequestType::ALL)
    {
      dr.emplace_back(contact);
      return &(dr.back());
    }
    else if (cdata.req->type == ContactRequestType::CLOSEST)
    {
      if (contact.distance < dr[0].distance)
      {
        dr[0] = contact;
        return &(dr[0]);
      }
    }
    //    else if (cdata.req->type == DistanceRequestType::LIMITED)
    //    {
    //      assert(dr.size() < cdata.req->max_contacts_per_body);
    //      dr.emplace_back(contact);
    //      return &(dr.back());
    //    }
  }

  return nullptr;
}

}

#endif // TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
