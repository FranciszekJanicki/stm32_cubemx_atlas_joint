#include "atlas_err.h"

char const* atlas_err_to_string(atlas_err_t err)
{
    switch (err) {
        case ATLAS_ERR_OK:
            return "ATLAS_ERR_OK";
        case ATLAS_ERR_FAIL:
            return "ATLAS_ERR_FAIL";
        case ATLAS_ERR_NOT_RUNNING:
            return "ATLAS_ERR_NOT_RUNNING";
        case ATLAS_ERR_ALREADY_RUNNING:
            return "ATLAS_ERR_ALREADY_RUNNING";
        case ATLAS_ERR_UNKNOWN_EVENT:
            return "ATLAS_ERR_UNKNOWN_EVENT";
        case ATLAS_ERR_UNKNOWN_NOTIFY:
            return "ATLAS_ERR_UNKNOWN_NOTIFY";
        case ATLAS_ERR_UNKNOWN_ORIGIN:
            return "ATLAS_ERR_UNKNOWN_ORIGIN";
        case ATLAS_ERR_IMPROPER_STATE:
            return "ATLAS_ERR_IMPROPER_STATE";
        default:
            return "";
    }
}
