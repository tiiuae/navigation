/**  \file
     \brief Defines helper routines for getting and setting variables under mutex locks
     \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef MUTEX_H
#define MUTEX_H

#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <tuple>

namespace mrs_lib
{

/**
 * @brief thread-safe getter and setter for values of variables (args)
 *
 * @tparam GetArgs types of the variables to get
 * @tparam SetArgs types of the variables to set
 * @param mut mutex which protects the variables
 * @param get tuple of variable references to obtain the values from
 * @param to_set tuple of variable references to set the new values from \p from_set
 * @param from_set tuple of the new values to be set to \p to_set
 *
 * @return tuple of the values from \p get
 */
template <class... GetArgs, class... SetArgs>
std::tuple<GetArgs...> get_set_mutexed(std::mutex& mut, std::tuple<GetArgs&...> get, std::tuple<SetArgs...> from_set, std::tuple<SetArgs&...> to_set) {

  std::scoped_lock lock(mut);

  std::tuple<GetArgs...> result = get;
  to_set = from_set;

  return result;
}

/**
 * @brief thread-safe getter for values of variables (args)
 *
 * @tparam Args types of the variables
 * @param mut mutex which protects the variables
 * @param args variables to obtain the values from
 *
 * @return std::tuple of the values
 */
template <class... Args>
std::tuple<Args...> get_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  std::tuple result = std::tuple(args...);

  return result;
}

/**
 * @brief thread-safe getter for values of variables (args)
 *
 * @tparam Args types of the variables
 * @param mut mutex which protects the variables
 * @param args variables to obtain the values from
 *
 * @return std::tuple of the values
 */
template <class... Args>
std::tuple<Args...> get_mutexed(std::shared_mutex& mut, Args&... args) {

  std::shared_lock lock(mut);

  std::tuple result = std::tuple(args...);

  return result;
}

/**
 * @brief thread-safe getter a value from a variable
 *
 * @tparam T type of the variable
 * @param mut mutex which protects the variable
 * @param arg variable to obtain the value from
 *
 * @return value of the variable
 */
template <class T>
T get_mutexed(std::mutex& mut, T& arg) {

  std::scoped_lock lock(mut);

  return arg;
}

/**
 * @brief thread-safe getter a value from a variable
 *
 * @tparam T type of the variable
 * @param mut mutex which protects the variable
 * @param arg variable to obtain the value from
 *
 * @return value of the variable
 */
template <class T>
T get_mutexed(std::shared_mutex& mut, T& arg) {

  std::shared_lock lock(mut);

  return arg;
}

/**
 * @brief base case of the variadic template for set_mutexed()
 *
 * @tparam T variable type
 * @param what value to set
 * @param where reference to be set
 */
template <class T>
void set_mutexed_impl(const T what, T& where) {

  where = what;
}

/**
 * @brief general case of the variadic template for set_mutexed()
 *
 * @tparam T type of the next variable to set
 * @tparam Args types of the rest of the variables
 * @param what value to set
 * @param where reference to be set
 * @param args the remaining arguments
 */
template <class T, class... Args>
void set_mutexed_impl(const T what, T& where, Args... args) {

  where = what;

  set_mutexed_impl(args...);
}

/**
 * @brief thread-safe setter for a variable
 *
 * @tparam T type of the variable
 * @param mut mutex to be locked
 * @param what value to set
 * @param where reference to be set
 *
 * @return
 */
template <class T>
auto set_mutexed(std::mutex& mut, const T what, T& where) {

  std::scoped_lock lock(mut);

  where = what;

  return where;
}

/**
 * @brief thread-safe setter for multiple variables
 *
 * example:
 *   set_mutexed(my_mutex_, a, a_, b, b_, c, c_);
 *   where a, b, c are the values to be set
 *         a_, b_, c_ are the variables to be updated
 *
 * @tparam Args types of the variables
 * @param mut mutex to be locked
 * @param args
 *
 * @return alternating list of values that were just set
 */
template <class... Args>
auto set_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  set_mutexed_impl(args...);

  return std::tuple(args...);
}

/**
 * @brief thread-safe setter for multiple variables
 *
 * example:
 *   set_mutexed(mu_mutex, std::tuple(a, b, c), std::forward_as_tuple(a_, b_, c_));
 *   where a, b, c are the values to be set
 *         a_, b_, c_ are the updated variables
 *
 * @tparam Args types of the variables
 * @param mut mutex to be locked
 * @param from std::tuple of the values
 * @param to std::tuple of reference to the variablaes
 *
 * @return
 */
template <class... Args>
auto set_mutexed(std::mutex& mut, const std::tuple<Args...> from, std::tuple<Args&...> to) {

  std::scoped_lock lock(mut);

  to = from;

  return to;
}

}  // namespace mrs_lib

#endif
