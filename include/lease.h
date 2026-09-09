#ifndef HEAD_LEASE_H_
#define HEAD_LEASE_H_

#include "head.h"

bool head_lease_is_expired(const struct head_runtime *runtime, uint32_t now_ms);
bool head_lease_token_is_valid(const struct head_runtime *runtime,
                               uint32_t lease_token, uint32_t now_ms);
void head_lease_renew(struct head_runtime *runtime, uint32_t now_ms);

#endif
