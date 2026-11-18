/* Proto — proto_config.c — CONFIG: read and update persistent configuration.
 *
 * A request with no config fields is a read; the response echoes the current
 * config.  Any recognised field updates only that field (partial update) and,
 * on success, bumps the persisted config revision.  Time (`utc`) is applied
 * immediately but not persisted — it is the desktop app's job to keep the
 * mount's clock set.
 */
#include "proto_internal.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static void record_error(const MountResult *r, char *err, size_t cap) {
    if (!r->ok && err[0] == '\0') {
        snprintf(err, cap, "%s", r->message);
    }
}

/* Format the current system clock as UTC ISO 8601 ("2026-09-28T02:00:00Z"). */
static void format_utc_now(char *buf, size_t cap) {
    time_t now = time(NULL);
    struct tm tm = {0};
    gmtime_r(&now, &tm);
    strftime(buf, cap, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

void proto_handle_config(const char *req, uint32_t id, char *out, size_t cap) {
    char err[128] = {0};
    bool changed = false;

    /* Site location — partial: start from the current value, override present.
     * A present field with the wrong JSON type is an error, not an absence:
     * it must reject the whole request rather than silently keep the old value. */
    MountSettings settings = mount_get_visible_status().settings;
    float f;
    int iv;
    bool site_changed = false;

    if (proto_json_has_key(req, "lat")) {
        if (!proto_json_get_float(req, "lat", &f)) {
            if (err[0] == '\0') snprintf(err, sizeof(err), "lat must be a number");
        } else { settings.lat = f; site_changed = true; }
    }
    if (proto_json_has_key(req, "lon")) {
        if (!proto_json_get_float(req, "lon", &f)) {
            if (err[0] == '\0') snprintf(err, sizeof(err), "lon must be a number");
        } else { settings.lon = f; site_changed = true; }
    }
    if (proto_json_has_key(req, "elevation")) {
        if (!proto_json_get_int(req, "elevation", &iv)) {
            if (err[0] == '\0') snprintf(err, sizeof(err), "elevation must be an integer");
        } else { settings.elevation = iv; site_changed = true; }
    }

    /* Guide rates (in-memory).  A present field must be a finite number in
     * 0..1 deg/s; zero is allowed and means "guiding disabled for this axis".
     * Everything is parsed and validated before anything is applied, so an
     * invalid field changes no other field, no rate, the revision, or the
     * persisted settings. */
    float new_guide_ra = 0.0f;
    float new_guide_dec = 0.0f;
    bool guide_ra_set = false;
    bool guide_dec_set = false;

    if (proto_json_has_key(req, "guide_rate_ra")) {
        if (!proto_json_get_float(req, "guide_rate_ra", &f)
            || !mount_guide_rate_is_valid(f)) {
            if (err[0] == '\0')
                snprintf(err, sizeof(err), "guide_rate_ra must be 0..1 deg/s");
        } else {
            new_guide_ra = f;
            guide_ra_set = true;
        }
    }
    if (proto_json_has_key(req, "guide_rate_dec")) {
        if (!proto_json_get_float(req, "guide_rate_dec", &f)
            || !mount_guide_rate_is_valid(f)) {
            if (err[0] == '\0')
                snprintf(err, sizeof(err), "guide_rate_dec must be 0..1 deg/s");
        } else {
            new_guide_dec = f;
            guide_dec_set = true;
        }
    }

    /* UTC time — applied, not persisted.  A present field must be a valid
     * ISO-8601 string; it is validated here so an invalid UTC rejects the
     * whole request before any rate or site field is applied. */
    char utc[64];
    bool utc_set = false;
    if (proto_json_has_key(req, "utc")) {
        if (!proto_json_get_string(req, "utc", utc, sizeof(utc))
            || !mount_time_string_is_valid(utc)) {
            if (err[0] == '\0')
                snprintf(err, sizeof(err), "utc must be a valid ISO-8601 timestamp");
        } else {
            utc_set = true;
        }
    }

    /* Apply only if every present field parsed and validated cleanly. */
    if (err[0] == '\0' && site_changed) {
        MountResult r = mount_settings_update(settings);
        record_error(&r, err, sizeof(err));
        if (r.ok) changed = true;
    }
    if (err[0] == '\0' && guide_ra_set) { mount_set_guide_rate_ra(new_guide_ra); changed = true; }
    if (err[0] == '\0' && guide_dec_set) { mount_set_guide_rate_dec(new_guide_dec); changed = true; }
    if (err[0] == '\0' && utc_set) {
        MountResult r = mount_set_system_time(utc);
        record_error(&r, err, sizeof(err));
        if (r.ok) changed = true;
    }

    if (changed) {
        proto_config_rev_bump();
    }

    size_t len = 0;
    out[0] = '\0';

    if (err[0] != '\0') {
        proto_buf_append(out, cap, &len,
            "{\"type\":\"config\",\"id\":%lu,\"ok\":false,\"error\":\"%s\"}",
            (unsigned long) id, err);
        return;
    }

    MountSettings cur = mount_get_visible_status().settings;
    char utc_buf[32];
    format_utc_now(utc_buf, sizeof(utc_buf));
    proto_buf_append(out, cap, &len,
        "{\"type\":\"config\",\"id\":%lu,\"ok\":true,\"config_rev\":%lu,"
        "\"utc\":\"%s\","
        "\"config\":{\"lat\":%.4f,\"lon\":%.4f,\"elevation\":%d,"
        "\"guide_rate_ra\":%.6f,\"guide_rate_dec\":%.6f}}",
        (unsigned long) id, (unsigned long) proto_config_rev(),
        utc_buf,
        cur.lat, cur.lon, cur.elevation,
        mount_get_guide_rate_ra(), mount_get_guide_rate_dec());
}
