/*
 * SCURM_ROS 中可能使用的QoS配置
 */

#ifndef RM_INTERFACES__QOS_POLICY_HPP
#define RM_INTERFACES__QOS_POLICY_HPP

#ifdef __cplusplus
extern "C"
{
#endif

#include "rmw/types.h"

  /**
 * Best effort qos policy(最佳效率QoS配置)
 * 队列深度: 1
 * 可靠性: Best effort
 * 持续性: Transient local
 */
  static const rmw_qos_profile_t best_effort_qos_policy = {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false};

#ifdef __cplusplus
}
#endif

#endif // RM_INTERFACES__QOS_POLICY_HPP
