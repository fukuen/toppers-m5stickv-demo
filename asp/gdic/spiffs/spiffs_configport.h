

//#include "global_config.h"

#define SPIFFS_START_ADDR 0xD00000
#define SPIFFS_SIZE 0x300000
#define SPIFFS_EREASE_SIZE 0x1000
#define SPIFFS_LOGICAL_BLOCK_SIZE 0x20000
#define SPIFFS_LOGICAL_PAGE_SIZE 0x1000
#define SPIFFS_OBJ_NAME_LEN 128
#define SPIFFS_USE_MAGIC 1
#define SPIFFS_USE_MAGIC_LENGTH 1
#define SPIFFS_META_LENGTH 0
#define SPIFFS_ALIGNED_OBJECT_INDEX_TABLES 1
//#define SPIFFS_HAL_CALLBACK_EXTRA 1

//debug
//#define SPIFFS_DBG 0
//#define SPIFFS_API_DBG 0
//#define SPIFFS_GC_DBG 0
//#define SPIFFS_CACHE_DBG 0

#define FS_PHYS_ADDR SPIFFS_START_ADDR
#define FS_PHYS_SIZE SPIFFS_SIZE
#define FS_PHYS_PAGE SPIFFS_LOGICAL_PAGE_SIZE
#define FS_PHYS_BLOCK SPIFFS_LOGICAL_BLOCK_SIZE
#define FLASH_SECTOR_SIZE SPIFFS_EREASE_SIZE
//#define SPIFFS_buffer_bytes_for_filedescs
//#define SPIFFS_buffer_bytes_for_cache

// Set generic spiffs debug output call.
#if CONFIG_SPIFFS_DBG
    #define SPIFFS_DBG(_f, ...) printf(_f, ## __VA_ARGS__)
#else
    #define SPIFFS_DBG(_f, ...)
#endif

// Set spiffs debug output call for garbage collecting.
#if CONFIG_SPIFFS_GC_DBG
    #define SPIFFS_GC_DBG(_f, ...) printf(_f, ## __VA_ARGS__)
#else
    #define SPIFFS_GC_DBG(_f, ...)
#endif

// Set spiffs debug output call for caching.
#if CONFIG_SPIFFS_CACHE_DBG
    #define SPIFFS_CACHE_DBG(_f, ...) printf(_f, ## __VA_ARGS__)
#else
    #define SPIFFS_CACHE_DBG(_f, ...)
#endif

// Set spiffs debug output call for system consistency checks.
#if CONFIG_SPIFFS_CHECK_DBG
    #define SPIFFS_CHECK_DBG(_f, ...) printf(_f, ## __VA_ARGS__)
#else
    #define SPIFFS_CHECK_DBG(_f, ...)
#endif

// Set spiffs debug output call for all api invocations.
#if CONFIG_SPIFFS_API_DBG
    #define SPIFFS_API_DBG(_f, ...) printf(_f, ## __VA_ARGS__)
#else
    #define SPIFFS_API_DBG(_f, ...)
#endif

#define  FS_PATCH_LENGTH (SPIFFS_OBJ_NAME_LEN*20+20)

typedef signed int s32_t;
typedef unsigned int u32_t;
typedef signed short s16_t;
typedef unsigned short u16_t;
typedef signed char s8_t;
typedef unsigned char u8_t;
