#include "lease.h"

bool head_lease_is_expired(const struct head_runtime *runtime, uint32_t now_ms)
{
  return runtime->active_lease_token == 0u ||
         (int32_t)(now_ms - runtime->lease_expires_ms) >= 0;
}

bool head_lease_token_is_valid(const struct head_runtime *runtime,
                               uint32_t lease_token, uint32_t now_ms)
{
  return lease_token != 0u && lease_token == runtime->active_lease_token &&
         !head_lease_is_expired(runtime, now_ms);
}

void head_lease_renew(struct head_runtime *runtime, uint32_t now_ms)
{
  runtime->lease_expires_ms = now_ms + HEAD_LEASE_DURATION_MS;
}
