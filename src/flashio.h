
#ifndef FILEIO_H_
#define FILEIO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/types.h>

#define FS_ROOT "/lfs1"

#define FILE_CHAR_LOCATION FS_ROOT "/characteristics"
#define FILE_SENSOR_DATA_LOCATION FS_ROOT "/sdata"
#define FILE_SENSOR_DATA_LOCATION_BAK FS_ROOT "/sdata_bak"
#define FILE_LOGS_LOCATION FS_ROOT "/logs"

const int data_append(const char *filename, void *data, ssize_t len);
const int data_overwrite(const char *filename, void *data, ssize_t len);
const int data_read(const char *filename, void *buf, ssize_t offset, ssize_t len);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* FILEIO_H_ */