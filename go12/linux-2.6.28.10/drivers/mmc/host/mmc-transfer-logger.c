/*
 *  linux/drivers/mmc/host/mmc-transfer-logger.c - MMC transfer logger tool
 */

#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/mmc/host.h>


struct partition_description {
	unsigned int partition_number;
	unsigned long start;
	unsigned long size;
	unsigned long bytes_written;
	unsigned long bytes_read;
	struct list_head list;
};

struct host_description {
	struct mmc_host *host;
	struct list_head partition_list;	
	spinlock_t		logger_lock;		/* Mutex */
	unsigned long bytes_read;
	unsigned long bytes_written;
	struct list_head list;
};

static LIST_HEAD(mmc_host_list);


static struct host_description* find_host_description(struct mmc_host *host)
{
	struct host_description* entry;
	
	list_for_each_entry(entry, &mmc_host_list, list)
	{
		if (entry->host == host)
		{
			return entry;
		}
	}
	return NULL;
}

static void clear_partition_list(struct host_description *host_desc)
{
	struct partition_description* entry;
	struct partition_description* temp_storage;
	
	list_for_each_entry_safe(entry, temp_storage, &host_desc->partition_list, list)
	{
		list_del(&entry->list);
		kfree(entry);
	}
	host_desc->bytes_read = 0;
	host_desc->bytes_written = 0;
}

ssize_t write_logger_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mmc_host *host;
	struct partition_description* entry;
	struct host_description* host_desc;
	unsigned long logger_flags;
	ssize_t ret = 0;

	if (dev == NULL)
		return 0;

	host = container_of(dev, struct mmc_host, class_dev);

	host_desc = find_host_description(host);

	if ((host == NULL) || (host_desc == NULL))
		return 0;

	spin_lock_irqsave(&host_desc->logger_lock, logger_flags);
	list_for_each_entry(entry, &host_desc->partition_list, list)
	{
		ret += scnprintf(buf + ret, PAGE_SIZE, 
		"Host: %s, Part: %u (Start:%lu, End:%lu), Written: %lu, Read: %lu\n",
		mmc_hostname(host), entry->partition_number, entry->start, entry->start + entry->size,
		entry->bytes_written, entry->bytes_read);
	}
	// only if we have any partitions registered - since otherwise we must return empty output.
	if (ret > 0) {
		ret += scnprintf(buf + ret, PAGE_SIZE, "Host: %s : Total : Written: %ld, Read: %ld\n",
		                 mmc_hostname(host), host_desc->bytes_written, host_desc->bytes_read);
	}
	spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);

	return ret;
}

static int add_partition_description(const char *buf,
				     struct host_description* host_desc)
{
		unsigned long logger_flags;
		struct partition_description* new_entry = 
		kmalloc(sizeof(struct partition_description), GFP_KERNEL);
		
		if (new_entry == NULL)
			return 1;
		
		memset(new_entry,0,sizeof(struct partition_description));
		
		sscanf(buf, "partition %u %lu %lu", &new_entry->partition_number,
		       &new_entry->start, &new_entry->size);
		
		spin_lock_irqsave(&host_desc->logger_lock, logger_flags);	
		list_add_tail(&new_entry->list, &host_desc->partition_list);
		spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
		
		return 0;
}

ssize_t write_logger_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host;
	unsigned long logger_flags;
	struct host_description* host_desc;

	if(dev == NULL)
		return 0;

	host = container_of(dev, struct mmc_host, class_dev);

	host_desc = find_host_description(host);
	
	if ((host == NULL) || (host_desc == NULL))
		return 0;
	
	if (!strncmp(buf,"reset",strlen("reset")))
	{
		spin_lock_irqsave(&host_desc->logger_lock, logger_flags);	
		clear_partition_list(host_desc);
		spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
	}
	else
	{
		if (add_partition_description(buf, host_desc))
			return 0;
	}
	return count;
}

DEVICE_ATTR(transfer_log, S_IRUGO|S_IWUGO, write_logger_show, write_logger_store);

void transfer_logger_log_transfer(struct mmc_host *host,
	unsigned long start, unsigned int blksz,
	unsigned long bytes_xfered, bool write)
{
	struct partition_description* entry;
	struct host_description* host_desc;
	unsigned long logger_flags;
	unsigned long end = start + bytes_xfered / blksz;
	
	host_desc = find_host_description(host);
	
	if ((host == NULL) || (host_desc == NULL))
		return;
	
	spin_lock_irqsave(&host_desc->logger_lock, logger_flags);

	// total stats for the device
	if (write) {
		host_desc->bytes_written += bytes_xfered;
	} else {
		host_desc->bytes_read += bytes_xfered;
	}

	// and for each partition
	list_for_each_entry_reverse(entry, &host_desc->partition_list, list)
	{
		unsigned long part_size;

		if (start >= entry->start + entry->size)
			break;
		if (end <= entry->start)
			continue;

		if (end > entry->start + entry->size) {
			unsigned long unaccounted = end - (entry->start + entry->size);
			end -= unaccounted;
		}

		if (entry->start <= start) {
			part_size = end - start;
			end = start;
		} else {
			part_size = end - entry->start;
			end = entry->start;
		}

		if (write) {
			entry->bytes_written += part_size * blksz;
		} else {
			entry->bytes_read += part_size * blksz;
		}

		if (end == start)
			break;
	}

	spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
}

int transfer_logger_add(struct mmc_host *host)
{
	struct host_description* new_host_desc;
	
	// Exit if we already know this host
	if (find_host_description(host) != NULL)
	{
		return 0;
	}
	
	if(device_create_file(&host->class_dev, &dev_attr_transfer_log))
	{
		printk(KERN_ERR "Failed to add write_log attribute.\n");
	}
	else
	{
		new_host_desc = kmalloc(sizeof(struct host_description), GFP_KERNEL);
		if (new_host_desc == NULL)
		{
			device_remove_file(&host->class_dev, &dev_attr_transfer_log);
			return -ENODEV;
		}

		new_host_desc->host = host;

		new_host_desc->bytes_written = 0;
		new_host_desc->bytes_read = 0;

		INIT_LIST_HEAD(&new_host_desc->partition_list);
		spin_lock_init(&new_host_desc->logger_lock);
		list_add_tail(&new_host_desc->list, &mmc_host_list);
	}
	return 0;
}

int transfer_logger_remove(struct mmc_host *host)
{
	unsigned long logger_flags;
	struct host_description* host_desc;

	host_desc = find_host_description(host);
	if (host_desc == NULL)
	{
		return -ENODEV;
	}

	spin_lock_irqsave(&host_desc->logger_lock, logger_flags);

	device_remove_file(&host->class_dev, &dev_attr_transfer_log);
	clear_partition_list(host_desc);

	spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
	list_del(&host_desc->list);
	kfree(host_desc);
	return 0;
}