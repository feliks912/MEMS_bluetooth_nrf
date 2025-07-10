

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>

#include <errno.h>

#define FS_ROOT "/lfs1"

LOG_MODULE_REGISTER(flash_logger, LOG_LEVEL_DBG);

const int create_directory(const char *full_path)
{
	int err;

	err = fs_mkdir(full_path);
	if (err < 0 && err != -EEXIST)
	{
		LOG_ERR("Failed to create directory %s. Err %d", full_path, err);
		return err;
	}
	return 0;
}

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
		LOG_DBG("Written %d bytes to %s.", err, filename);
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
		LOG_DBG("Written %d bytes to %s.", err, filename);
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

	int err = fs_open(&l_file, filename, FS_O_CREATE | FS_O_READ);
	if (err < 0)
	{
		LOG_ERR("Failed to open file %s. Err %d (%s)", filename, err, strerror(-err));
		return err;
	}
	else
	{
		LOG_DBG("Oppened file %s.", filename);
	}

	off_t offset_setting = (off_t)offset;

	struct fs_dirent file_info;
	fs_stat(filename, &file_info);

	LOG_DBG("File read offset is set to %ld. Total file size %d", offset_setting, file_info.size);

	err = fs_seek(&l_file, offset_setting, FS_SEEK_SET);
	if (err < 0)
	{
		LOG_ERR("Failed to set file seek of file %s to %ld. Err %d (%s)", filename, offset_setting, err, strerror(-err));
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
		LOG_DBG("Read %d bytes from %s.", err, filename);
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

int print_zephyr_fs_details(void) {
    struct fs_statvfs s_buf; // Declare the statvfs buffer
    int rc;

    LOG_INF("Attempting to get filesystem statistics for path: %s", FS_ROOT);

    // Call fs_statvfs to get the filesystem information
    rc = fs_statvfs(FS_ROOT, &s_buf);

    if (rc < 0) {
        LOG_ERR("Failed to get filesystem info for %s. Error: %d", FS_ROOT, rc);
        return rc;
    }

    // Calculation for total, used, and free space in bytes
    // Using uint64_t for calculations to prevent overflow with large filesystems
    uint64_t total_blocks_val = s_buf.f_blocks;
    uint64_t free_blocks_val = s_buf.f_bfree;
    uint64_t allocation_unit_size = s_buf.f_frsize; // Use f_frsize for calculations related to f_blocks and f_bfree

    uint64_t total_space_bytes = total_blocks_val * allocation_unit_size;
    uint64_t free_space_bytes = free_blocks_val * allocation_unit_size;
    uint64_t used_space_bytes = total_space_bytes - free_space_bytes;

    LOG_INF("--- Zephyr Filesystem Information for %s ---", FS_ROOT);
    LOG_INF("  Optimal Transfer Block Size (f_bsize): %lu bytes", s_buf.f_bsize);
    LOG_INF("  Allocation Unit Size (f_frsize):       %lu bytes", s_buf.f_frsize);
    LOG_INF("  Total Blocks (f_blocks):               %lu", s_buf.f_blocks);
    LOG_INF("  Number of Free Blocks (f_bfree):       %lu", s_buf.f_bfree);

    LOG_INF("\n--- Zephyr Filesystem Space Usage ---");
    LOG_INF("  Total Space: %llu bytes", total_space_bytes);
    LOG_INF("  Used Space:  %llu bytes", used_space_bytes);
    LOG_INF("  Free Space:  %llu bytes", free_space_bytes);

    // You can also print percentages if desired
    if (total_space_bytes > 0) {
        double used_percent = (double)used_space_bytes / total_space_bytes * 100.0;
        double free_percent = (double)free_space_bytes / total_space_bytes * 100.0;
        LOG_INF("  Used Percentage: %.2f %%", used_percent);
        LOG_INF("  Free Percentage: %.2f %%", free_percent);
    } else {
        LOG_INF("  Filesystem reports 0 total space.");
    }

    return 0;
}