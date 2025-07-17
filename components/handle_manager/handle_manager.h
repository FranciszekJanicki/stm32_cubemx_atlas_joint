#ifndef HANDLE_MANAGER_HANDLE_MANAGER_H
#define HANDLE_MANAGER_HANDLE_MANAGER_H

#include <assert.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DECLARE_HANDLE_MANAGER(NAME, KEY_TYPE, HANDLE_TYPE, NUM_HANDLES) \
                                                                         \
    void NAME##_manager_set(KEY_TYPE key, HANDLE_TYPE handle);           \
                                                                         \
    HANDLE_TYPE NAME##_manager_get(KEY_TYPE key);

#define DEFINE_HANDLE_MANAGER(NAME, KEY_TYPE, HANDLE_TYPE, NUM_HANDLES) \
    static HANDLE_TYPE NAME##_manager_handles[NUM_HANDLES] = {0};       \
                                                                        \
    void NAME##_manager_set(KEY_TYPE key, HANDLE_TYPE handle)           \
    {                                                                   \
        assert(key < NUM_HANDLES);                                      \
        assert(!NAME##_manager_handles[key]);                           \
        assert(handle);                                                 \
        NAME##_manager_handles[key] = handle;                           \
    }                                                                   \
                                                                        \
    HANDLE_TYPE NAME##_manager_get(KEY_TYPE key)                        \
    {                                                                   \
        assert(key < NUM_HANDLES);                                      \
        assert(NAME##_manager_handles[key]);                            \
        return NAME##_manager_handles[key];                             \
    }

#ifdef __cplusplus
}
#endif

#endif // #ifndef HANDLE_MANAGER_HANDLE_MANAGER_H