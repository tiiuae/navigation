#pragma once

#include <cassert>
#include <fog_msgs/msg/navigation_state.h>
#include <fog_msgs/msg/navigation_waypoint_state.h>

namespace navigation
{
#define ERR_MSG "Inconsistent message received, please rebuild the package that crashed!"

  /* nav_state_t enumeration type //{ */
  
  enum struct nav_state_t
  {
    invalid,
    not_ready,
    idle,
    planning,
    commanding,
    moving,
    avoiding,
  };
  
  static inline nav_state_t to_enum(const fog_msgs::msg::NavigationState msg)
  {
    switch (msg.state)
    {
      case fog_msgs::msg::NavigationState::NOT_READY:   return nav_state_t::not_ready;
      case fog_msgs::msg::NavigationState::IDLE:        return nav_state_t::idle;
      case fog_msgs::msg::NavigationState::PLANNING:    return nav_state_t::planning;
      case fog_msgs::msg::NavigationState::COMMANDING:  return nav_state_t::commanding;
      case fog_msgs::msg::NavigationState::MOVING:      return nav_state_t::moving;
      case fog_msgs::msg::NavigationState::AVOIDING:    return nav_state_t::avoiding;
      default:                                          assert(false && ERR_MSG); return nav_state_t::invalid;
    }
  }
  
  static inline fog_msgs::msg::NavigationState to_msg(const nav_state_t enum_val)
  {
    fog_msgs::msg::NavigationState msg;
    switch (enum_val)
    {
      case nav_state_t::not_ready:          msg.state = fog_msgs::msg::NavigationState::NOT_READY; break;
      case nav_state_t::idle:               msg.state = fog_msgs::msg::NavigationState::IDLE; break;
      case nav_state_t::planning:           msg.state = fog_msgs::msg::NavigationState::PLANNING; break;
      case nav_state_t::commanding:         msg.state = fog_msgs::msg::NavigationState::COMMANDING; break;
      case nav_state_t::moving:             msg.state = fog_msgs::msg::NavigationState::MOVING; break;
      case nav_state_t::avoiding:           msg.state = fog_msgs::msg::NavigationState::AVOIDING; break;
      default:                              assert(false && ERR_MSG); msg.state = fog_msgs::msg::NavigationState::INVALID; break;
    }
    return msg;
  }

  static inline std::string to_string(const nav_state_t enum_val)
  {
    switch (enum_val)
    {
      case nav_state_t::not_ready:      return "not_ready";
      case nav_state_t::idle:           return "idle";
      case nav_state_t::planning:       return "planning";
      case nav_state_t::commanding:     return "commanding";
      case nav_state_t::moving:         return "moving";
      case nav_state_t::avoiding:       return "avoiding";
      default:                          assert(false && ERR_MSG); return "invalid";
    }
  }
  
  //}

  /* mission_state_t enumeration type //{ */
  
  enum struct waypoint_state_t
  {
    invalid,
    empty,
    ongoing,
    reached,
    unreachable,
  };
  
  static inline waypoint_state_t to_enum(const fog_msgs::msg::NavigationWaypointState msg)
  {
    switch (msg.state)
    {
      case fog_msgs::msg::NavigationWaypointState::EMPTY:         return waypoint_state_t::empty;
      case fog_msgs::msg::NavigationWaypointState::ONGOING:       return waypoint_state_t::ongoing;
      case fog_msgs::msg::NavigationWaypointState::REACHED:       return waypoint_state_t::reached;
      case fog_msgs::msg::NavigationWaypointState::UNREACHABLE:   return waypoint_state_t::unreachable;
      default:                                                    assert(false && ERR_MSG); return waypoint_state_t::invalid;
    }
  }
  
  static inline fog_msgs::msg::NavigationWaypointState to_msg(const waypoint_state_t enum_val)
  {
    fog_msgs::msg::NavigationWaypointState msg;
    switch (enum_val)
    {
      case waypoint_state_t::empty:             msg.state = fog_msgs::msg::NavigationWaypointState::EMPTY; break;
      case waypoint_state_t::ongoing:           msg.state = fog_msgs::msg::NavigationWaypointState::ONGOING; break;
      case waypoint_state_t::reached:           msg.state = fog_msgs::msg::NavigationWaypointState::REACHED; break;
      case waypoint_state_t::unreachable:       msg.state = fog_msgs::msg::NavigationWaypointState::UNREACHABLE; break;
      default:                                  assert(false && ERR_MSG); msg.state = fog_msgs::msg::NavigationWaypointState::INVALID; break;
    }
    return msg;
  }

  static inline std::string to_string(const waypoint_state_t enum_val)
  {
    switch (enum_val)
    {
      case waypoint_state_t::empty:             return "empty";
      case waypoint_state_t::ongoing:           return "ongoing";
      case waypoint_state_t::reached:           return "reached";
      case waypoint_state_t::unreachable:       return "unreachable";
      default:                                  assert(false && ERR_MSG); return "invalid";
    }
  }
  
  //}
}
