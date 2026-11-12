#include "pdec_range_request.h"
#include "pdec_ranging.h"
#include "pdec_radio.h"

#include <errno.h>

static K_MUTEX_DEFINE(request_lock);
static K_SEM_DEFINE(request_done, 0, 1);
static uint32_t next_request_id;
static struct {
    bool owned;
    bool done;
    uint32_t id;
    int result;
    double distance;
} request;

bool pdec_range_request_is_pending(uint32_t request_id)
{
    k_mutex_lock(&request_lock, K_FOREVER);
    bool pending = request_id != 0 && request.owned && !request.done &&
                   request.id == request_id;
    k_mutex_unlock(&request_lock);
    return pending;
}

void pdec_range_request_complete(uint32_t request_id, int result, double distance)
{
    k_mutex_lock(&request_lock, K_FOREVER);
    if (request_id != 0 && request.owned && !request.done && request.id == request_id) {
        request.result = result;
        request.distance = distance;
        request.done = true;
        k_sem_give(&request_done);
    }
    k_mutex_unlock(&request_lock);
}

int request_range(uint16_t dst_id, bool double_sided, double *distance, k_timeout_t timeout)
{
    k_mutex_lock(&request_lock, K_FOREVER);
    if (request.owned) {
        k_mutex_unlock(&request_lock);
        return -EBUSY;
    }
    if (++next_request_id == 0) {
        ++next_request_id;
    }
    uint32_t id = next_request_id;
    request.owned = true;
    request.done = false;
    request.id = id;
    request.result = -ETIMEDOUT;
    request.distance = 0;
    k_sem_reset(&request_done);
    k_mutex_unlock(&request_lock);

    int ret = pdec_radio_submit_range(dst_id, double_sided, id);
    if (ret == 0) {
        (void)k_sem_take(&request_done, timeout);
    }

    /* Keep ownership until the waiting caller has consumed its own result.
     * A completion racing the caller deadline wins if it reached this lock first.
     * Old queued/running exchanges cannot complete a newer request with this ID.
     */
    k_mutex_lock(&request_lock, K_FOREVER);
    if (ret == 0) {
        ret = request.done ? request.result : -ETIMEDOUT;
    }
    if (ret == 0 && distance != NULL) {
        *distance = request.distance;
    }
    request.owned = false;
    k_mutex_unlock(&request_lock);
    return ret;
}
