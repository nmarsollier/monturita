/* Proto — proto_query.c — QUERY: read-only predictions.
 *
 * Unlike `action`, a query never moves the mount, never enqueues a command and
 * never changes tracking, HOME or the reported pier side.  It answers a
 * parameterised question about what the mount *would* do, using the same
 * selection a real goto would.
 */
#include "proto_internal.h"

#include <stdio.h>
#include <string.h>

static void emit_query_error(char *out, size_t cap, uint32_t id, const char *error) {
    snprintf(out, cap, "{\"type\":\"query\",\"id\":%lu,\"ok\":false,\"error\":\"%s\"}",
             (unsigned long) id, error ? error : "error");
}

void proto_handle_query(const char *req, uint32_t id, char *out, size_t cap) {
    char query[32];
    if (!proto_json_get_string(req, "query", query, sizeof(query))) {
        emit_query_error(out, cap, id, "missing query");
        return;
    }

    if (strcmp(query, "destination_side_of_pier") != 0) {
        emit_query_error(out, cap, id, "unknown query");
        return;
    }

    float ra = 0.0f;
    float dec = 0.0f;
    if (!proto_json_get_float(req, "ra", &ra) || !proto_json_get_float(req, "dec", &dec)) {
        emit_query_error(out, cap, id, "destination_side_of_pier requires ra and dec");
        return;
    }
    if (ra < 0.0f || ra >= 24.0f) {
        emit_query_error(out, cap, id, "ra out of range [0,24)");
        return;
    }
    if (dec < -90.0f || dec > 90.0f) {
        emit_query_error(out, cap, id, "dec out of range [-90,90]");
        return;
    }

    int side = -1;
    MountResult r = mount_destination_side_of_pier(ra, dec, &side);
    if (!r.ok) {
        emit_query_error(out, cap, id, r.message);
        return;
    }

    snprintf(out, cap, "{\"type\":\"query\",\"id\":%lu,\"ok\":true,\"side\":\"%s\"}",
             (unsigned long) id, side == 0 ? "east" : "west");
}
