
#include <tesseract_process_planners/process_definition.h>

namespace tesseract_process_planners
{
ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const ProcessSegmentDefinitionConfig& segment_config)
{
  ProcessDefinition process_definition;
  process_definition.start = process_config.start;
  process_definition.segments.reserve(process_config.tool_paths.size());
  process_definition.transitions.reserve(process_config.tool_paths.size() - 1);

  for (size_t i = 0; i < process_config.tool_paths.size(); ++i)
  {
    ProcessSegmentDefinition segment_def;
    if (segment_config.approach != nullptr)
      segment_def.approach = segment_config.approach->generate(process_config.tool_paths[i], process_config);

    if (segment_config.process != nullptr)
      segment_def.process = segment_config.process->generate(process_config.tool_paths[i], process_config);

    if (segment_config.departure != nullptr)
      segment_def.departure = segment_config.departure->generate(process_config.tool_paths[i], process_config);

    process_definition.segments.push_back(segment_def);
  }

  for (size_t i = 0; i < (process_config.tool_paths.size() - 1); ++i)
  {
    ProcessTransitionDefinition transition_def;
    assert(process_config.transition_generator[i] != nullptr);  // transition generators should never be nullptrs
    if (process_config.transition_generator[i] != nullptr)
    {
      if ((process_definition.segments[i].process.front() != nullptr) &&
          (process_definition.segments[i + 1].process.front() != nullptr) &&
          (process_definition.segments[i].process.back() != nullptr) &&
          (process_definition.segments[i + 1].process.back() != nullptr))
      {
        // Connect the appropriate waypoints for the transitions
        if (((process_definition.segments[i].departure.empty()) ||
             (process_definition.segments[i].departure.back() == nullptr)) &&
            ((process_definition.segments[i + 1].approach.empty()) ||
             (process_definition.segments[i + 1].approach.front() == nullptr)))
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].process.front());
        else if (process_definition.segments[i].departure.empty() ||
                 process_definition.segments[i].departure.back() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].approach.front());
        else if (process_definition.segments[i + 1].approach.empty() ||
                 process_definition.segments[i + 1].approach.front() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].process.front());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].approach.front());

        if ((process_definition.segments[i].approach.empty() ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            (process_definition.segments[i + 1].departure.empty() ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());
      }

      process_definition.transitions.push_back(transition_def);
    }
  }

  return process_definition;
}

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const std::vector<ProcessSegmentDefinitionConfig>& segment_config)
{
  ProcessDefinition process_definition;
  process_definition.start = process_config.start;
  process_definition.segments.reserve(process_config.tool_paths.size());
  process_definition.transitions.reserve(process_config.tool_paths.size() - 1);

  for (size_t i = 0; i < process_config.tool_paths.size(); ++i)
  {
    ProcessSegmentDefinition segment_def;
    if (segment_config[i].approach != nullptr)
      segment_def.approach = segment_config[i].approach->generate(process_config.tool_paths[i], process_config);

    if (segment_config[i].process != nullptr)
      segment_def.process = segment_config[i].process->generate(process_config.tool_paths[i], process_config);

    if (segment_config[i].departure != nullptr)
      segment_def.departure = segment_config[i].departure->generate(process_config.tool_paths[i], process_config);

    process_definition.segments.push_back(segment_def);
  }

  for (size_t i = 0; i < (process_config.tool_paths.size() - 1); ++i)
  {
    ProcessTransitionDefinition transition_def;
    assert(process_config.transition_generator[i] != nullptr);  // transition generators should never be nullptrs
    if (process_config.transition_generator[i] != nullptr)
    {
      if ((process_definition.segments[i].process.front() != nullptr) &&
          (process_definition.segments[i + 1].process.front() != nullptr) &&
          (process_definition.segments[i].process.back() != nullptr) &&
          (process_definition.segments[i + 1].process.back() != nullptr))
      {
        // Connect the appropriate waypoints for the transitions
        if (((process_definition.segments[i].departure.empty()) ||
             (process_definition.segments[i].departure.back() == nullptr)) &&
            ((process_definition.segments[i + 1].approach.empty()) ||
             (process_definition.segments[i + 1].approach.front() == nullptr)))
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].process.front());
        else if (process_definition.segments[i].departure.empty() ||
                 process_definition.segments[i].departure.back() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].approach.front());
        else if (process_definition.segments[i + 1].approach.empty() ||
                 process_definition.segments[i + 1].approach.front() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].process.front());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].approach.front());

        if ((process_definition.segments[i].approach.empty() ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            (process_definition.segments[i + 1].departure.empty() ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());
      }

      process_definition.transitions.push_back(transition_def);
    }
  }

  return process_definition;
}

std::size_t ProcessDefinition::size() const
{
  std::size_t s = (start == nullptr)  ? 0 : 1;
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    s+=segments[i].size();
    if (i < (segments.size() - 1))
      s+=transitions[i].transition_from_end.size();
  }
  return s;
}

ProcessDefinition::Iterator ProcessDefinition::begin()
{
  return Iterator(*this, 0);
}

ProcessDefinition::Iterator ProcessDefinition::end()
{
  return Iterator(*this, size());
}

tesseract_motion_planners::Waypoint::Ptr& ProcessDefinition::operator[](std::size_t index)
{
  assert(index < size());
  std::size_t s = 0;
  std::size_t s_prev = 0;

  if (index == 0 && start != nullptr)
    return start;

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    s+=segments[i].size();
    if (index < s)
      return segments[i][index - s_prev];

    s_prev = s;

    if (i < (segments.size() - 1))
    {
      s+=transitions[i].transition_from_end.size();
      if (index < s)
        return transitions[i].transition_from_end[index - s_prev];
    }

    s_prev = s;
  }

  // This should never be reached
  return segments.back()[index - s];
}

const tesseract_motion_planners::Waypoint::Ptr& ProcessDefinition::operator[](std::size_t index) const
{
  assert(index < size());
  std::size_t s = 0;
  std::size_t s_prev = 0;

  if (index == 0 && start != nullptr)
    return start;

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    s+=segments[i].size();
    if (index < s)
      return segments[i][index - s_prev];

    s_prev = s;

    if (i < (segments.size() - 1))
    {
      s+=transitions[i].transition_from_end.size();
      if (index < s)
        return transitions[i].transition_from_end[index - s_prev];
    }

    s_prev = s;
  }

  // This should never be reached
  return segments.back()[index - s];
}

ProcessDefinition::Iterator ProcessDefinition::erase(Iterator pos)
{
  std::size_t index = std::distance(begin(), pos);
  Iterator ret = ++pos;

  assert(index < size());
  std::size_t s = 0;
  std::size_t s_prev = 0;

  if (index == 0 && start != nullptr)
    return start;

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    s+=segments[i].size();
    if (index < s)
      return segments[i][index - s_prev];

    s_prev = s;

    if (i < (segments.size() - 1))
    {
      s+=transitions[i].transition_from_end.size();
      if (index < s)
        return transitions[i].transition_from_end[index - s_prev];
    }

    s_prev = s;
  }




  std::size_t index = std::distance(begin(), pos);
  Iterator ret = ++pos;

  if (index < approach.size())
    approach.erase(approach.begin() + static_cast<long>(index));
  else if (index < (approach.size() + process.size()))
    process.erase(process.begin() +  static_cast<long>(index - approach.size()));
  else if (index < (approach.size() + process.size() + departure.size()))
    departure.erase(departure.begin() +  static_cast<long>(index - approach.size() - process.size()));

  return ret;
}

ProcessDefinition::Iterator ProcessDefinition::erase(Iterator first, Iterator last)
{
  std::size_t index = static_cast<std::size_t>(std::distance(begin(), first));
  std::size_t length = static_cast<std::size_t>(std::distance(first, last));
  Iterator ret = ++last;
  for (std::size_t i = 0; i < length; ++i)
  {
    if (index < approach.size())
      approach.erase(approach.begin() +  static_cast<long>(index));
    else if (index < (approach.size() + process.size()))
      process.erase(process.begin() +  static_cast<long>(index - approach.size()));
    else if (index < (approach.size() + process.size() + departure.size()))
      departure.erase(departure.begin() +  static_cast<long>(index - approach.size() - process.size()));
    else
      throw std::runtime_error("Index is out of range!");
  }

  return ret;
}


ProcessDefinition::Iterator::Iterator(ProcessDefinition& container, std::size_t pos) : pos_(pos), container_(&container)
{

}

// this method must be defined after the definition of IntVector
// since it needs to use it
tesseract_motion_planners::Waypoint::Ptr& ProcessDefinition::Iterator::operator* ()
{
  if (pos_ < container_->approach.size())
    return container_->approach.at(pos_);
  else if (pos_ < (container_->approach.size() + container_->process.size()))
    return container_->process.at(pos_ - container_->approach.size());
  else if (pos_ < (container_->approach.size() + container_->process.size() + container_->departure.size()))
    return container_->departure.at(pos_ - container_->approach.size() - container_->process.size());
  else
    throw std::runtime_error("Index is out of range!");
}

const tesseract_motion_planners::Waypoint::Ptr& ProcessDefinition::Iterator::operator* () const
{
  if (pos_ < container_->approach.size())
    return container_->approach.at(pos_);
  else if (pos_ < (container_->approach.size() + container_->process.size()))
    return container_->process.at(pos_ - container_->approach.size());
  else if (pos_ < (container_->approach.size() + container_->process.size() + container_->departure.size()))
    return container_->departure.at(pos_ - container_->approach.size() - container_->process.size());
  else
    throw std::runtime_error("Index is out of range!");
}

const ProcessDefinition::Iterator& ProcessDefinition::Iterator::operator++ ()
{
   ++pos_;
   // although not strictly necessary for a range-based for loop
   // following the normal convention of returning a value from
   // operator++ is a good idea.
   return *this;
}

ProcessDefinition::Iterator ProcessDefinition::Iterator::operator++ (int)
{
   ProcessDefinition::Iterator ret = *this;
   ++pos_;
   return ret;
}

const ProcessDefinition::Iterator& ProcessDefinition::Iterator::operator-- ()
{
   --pos_;
   // although not strictly necessary for a range-based for loop
   // following the normal convention of returning a value from
   // operator++ is a good idea.
   return *this;
}

ProcessDefinition::Iterator ProcessDefinition::Iterator::operator-- (int)
{
   ProcessSegmentDefinition::Iterator ret = *this;
   --pos_;
   return ret;
}

ProcessDefinition::Iterator::difference_type ProcessDefinition::Iterator::operator-(const Iterator& rhs) const
{
  return static_cast<ProcessDefinition::Iterator::difference_type>(pos_ - rhs.pos_);
}

ProcessDefinition::Iterator ProcessDefinition::Iterator::operator+(difference_type rhs) const
{
  return Iterator(*container_, pos_ + rhs);
}

ProcessDefinition::Iterator ProcessDefinition::Iterator::operator-(difference_type rhs) const
{
  return Iterator(*container_, pos_ - rhs);
}

bool ProcessDefinition::Iterator::operator==(const Iterator& rhs) const
{
  return ((pos_ == rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessDefinition::Iterator::operator!=(const Iterator& rhs) const
{
  return ((pos_ != rhs.pos_) || (container_ != rhs.container_));
}

bool ProcessDefinition::Iterator::operator>(const Iterator& rhs) const
{
  return ((pos_ > rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessDefinition::Iterator::operator<(const Iterator& rhs) const
{
  return ((pos_ < rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessDefinition::Iterator::operator>=(const Iterator& rhs) const
{
  return ((pos_ >= rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessDefinition::Iterator::operator<=(const Iterator& rhs) const
{
  return ((pos_ <= rhs.pos_) && (container_ == rhs.container_));
}

}  // namespace tesseract_process_planners
