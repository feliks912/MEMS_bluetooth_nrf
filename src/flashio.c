


#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>


#include <errno.h>

LOG_MODULE_REGISTER(flash_logger, LOG_LEVEL_DBG);


const int data_append(const char *filename, void *data, ssize_t len)
{
	int written = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_CREATE | FS_O_APPEND);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Opened file %s.", filename);
	}

	err = fs_write(&l_file, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to append %d bytes to %s. Err %d (%s)", len, filename, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}
	else
	{
		LOG_DBG("Written %d bytes to %s.", len, filename);
		written = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return written;
}

const int data_overwrite(const char *filename, void *data, ssize_t len)
{
	int written = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Opened file %s.", filename);
	}

	err = fs_write(&l_file, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to write %d bytes to %s. Err %d (%s)", len, filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Written %d bytes to %s.", len, filename);
		written = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return written;
}

const int data_read(const char *filename, void *buf, ssize_t offset, ssize_t len)
{
	int read = 0;
	struct fs_file_t l_file;
	fs_file_t_init(&l_file);

	int err = fs_open(&l_file, filename, FS_O_READ);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Oppened file %s.", filename);
	}

	err = fs_seek(&l_file, offset, FS_SEEK_SET);
	if (err < 0)
	{
		LOG_ERR("Failed to set file seek of file %s to %d. Err %d (%s)", filename, offset, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}

	err = fs_read(&l_file, buf, len);
	if (err < 0)
	{
		LOG_ERR("Failed to read %d bytes from %s. Err %d (%s)", len, filename, err, strerror(-err));
		fs_close(&l_file);
		return err;
	}
	else
	{
		LOG_DBG("Read %d bytes from %s.", len, filename);
		read = err;
	}

	err = fs_close(&l_file);
	if (err < 0)
	{
		LOG_ERR("Failed to close file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Closed file %s", filename);
	}

	return read;
}