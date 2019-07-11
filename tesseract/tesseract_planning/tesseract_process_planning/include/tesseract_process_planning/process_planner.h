#ifndef TESSERACT_PLANNING_PROCESS_PLANNER_H
#define TESSERACT_PLANNING_PROCESS_PLANNER_H

#include <tesseract_process_planning/process_definition.h>

namespace tesseract_process_planning
{

/**
 * @brief Is a results structure to hold the results generated based on the process segment definition
 *
 * An empty trajectory indicates that this process step is to be omitted.
 *
 * TODO: Should we store the planner response for each?
 *
*/
struct ProcessSegmentPlan
{
  tesseract_common::TrajArray approach;  /**< The generated approach trajectory generated based on the Process Segment Definition */
  tesseract_common::TrajArray process;   /**< The generated process trajectory generated based on the Process Segment Definition */
  tesseract_common::TrajArray departure; /**< The generated departure trajectory generated based on the Process Segment Definition */
  bool valid = true;              /**< If true, a motion plan was sucessfully found for approach, process and departure, otherwise false */
};

/**
 * @brief Is a result structure to hold the results generated based on the process definition.
 *
 * Note: An empty trajectory for any process step indicates that it is to be omitted.
 *
*/
struct ProcessPlan
{
  tesseract_common::TrajArray from_home;                          /**< A trajectory from the home position to the firts point in the first segment */
  std::vector<ProcessSegmentPlan> segments;                /**< A vector of process segment results */
  std::vector<tesseract_common::TrajArray> transition_from_start; /**< A vector of transition plans from the start of segment[i] to the end of segment[i+1]
   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	, this data can be useful to make quick exit moves after canceling an ongoing process */
  std::vector<tesseract_common::TrajArray> transition_from_end;   /**< A vector of transition plans from the end of segment[i] to the start of segment[i+1] */
  tesseract_common::TrajArray to_home;                            /**< A trajectory from the last segments waypoint to the home position */
  bool valid = true;                                       /**< If true, a motion plan was sucessfully found for everything, otherwise false */
};

/**
 * @brief An abstract class for implementing process planners
 */
class ProcessPlanner
{
public:
  virtual ~ProcessPlanner() = default;

  /**
   * @brief Given a tool path generate process specific plan
   * @param tool_path Process tool path
   */
  virtual void setToolPath(const std::vector<std::vector<tesseract::tesseract_planning::WaypointPtr>>& tool_path) = 0;

  /**
   * @brief Get process definintion for the set tool path
   * @return Process definition
   */
  virtual const ProcessDefinition& getProcessDefinition() const = 0;

  /**
   * @brief Generate process plan for set tool path
   * @return Process motion plan
   */
  virtual ProcessPlan plan() = 0;

  /**
   * @brief Generate process plan for set tool path using the provided seed
   * @param seed The seed position for first tool path segment
   * @return Process motion plan
   */
  virtual ProcessPlan plan(const Eigen::VectorXd& seed) = 0;
};
}
#endif // TESSERACT_PLANNING_PROCESS_PLANNER_H
